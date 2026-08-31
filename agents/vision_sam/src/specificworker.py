#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QImage, QPixmap
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import os
import cv2
# from ultralytics import SAM
import torch
import sys
import threading
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *

# ================ ZED CAMERA CALIBRATION (Shadow.proto) ================
# Mount offset and rotation of the "zed" RGB camera relative to the robot's
# local frame, taken from webots-shadow/protos/Shadow.proto (the Camera child
# node under the robot Group, name "zed"). Webots gives these in meters;
# converted here to millimeters to match the mm convention used elsewhere
# in this DSR (sim_scene.json, causes.json, problem_position).
ZED_CAMERA_NAME = "zed"
ZED_MOUNT_OFFSET_MM = [0.0, -75.0, 945.0]
ZED_MOUNT_ROTATION_AXIS = [0.0, 0.0, 1.0]
ZED_MOUNT_ROTATION_ANGLE_RAD = 1.57  # ~90 deg about Z, robot-local frame

# concept_robot writes the live room->robot RT translation in METERS (it
# divides the raw Webots pose by 1000 before storing it), while every other
# position in this project (sim_scene.json, causes.json, problem_position) is
# in MILLIMETERS. Pre-existing unit mismatch, not introduced here: converted
# back to mm on read so this module stays consistent with the rest of the DSR.
ROOM_ROBOT_RT_TRANSLATION_IS_METERS = True


def _rotation_matrix_from_axis_angle(axis, angle):
    """Rodrigues' formula: 3x3 rotation matrix from an axis-angle rotation."""
    axis = np.array(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def _rotation_matrix_from_quaternion(qx, qy, qz, qw):
    """3x3 rotation matrix from a quaternion (x, y, z, w)."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


def _homogeneous_transform(rotation_3x3, translation_3):
    """Build a 4x4 homogeneous transform from a 3x3 rotation and a translation."""
    t = np.eye(4)
    t[:3, :3] = rotation_3x3
    t[:3, 3] = translation_3
    return t


# Animated spinner class to show progress while processing the image with SAM
class AnimatedSpinner:
    def __init__(self, message="Processing image with SAM..."):
        self.spinner = ['|', '/', '-', '\\']
        self.stopped = threading.Event()
        self.message = message
        self.idx = 0
        self.thread = threading.Thread(target=self._animate)

    def start(self):
        self.stopped.clear()
        self.thread.start()

    def _animate(self):
        while not self.stopped.is_set():
            sys.stdout.write(f"\r{self.message} {self.spinner[self.idx % len(self.spinner)]}")
            sys.stderr.flush()
            time.sleep(0.1)
            self.idx += 1
        sys.stdout.write(f"\r{self.message} Done!{' ' * 10}\n")
        sys.stdout.flush()

    def stop(self):
        self.stopped.set()
        self.thread.join()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # activate mouse tracking and install event filter to capture mouse clicks on the image label
        self.ui.image_label.setMouseTracking(True)
        self.ui.image_label.installEventFilter(self)

        self.selected_point = None
        self.ui.segment_button.clicked.connect(self.on_segment_button_clicked)

        self.current_save_dir = "segmented_objects"
        self.ui.save_new_folder_button.clicked.connect(self.on_save_new_folder_button_clicked)

        # Full-frame classifier dataset (no SAM, no crop): the robot's raw view, labeled by
        # the person capturing depending on whether the bump happens to be in frame or not.
        self.ui.bump_present_button.clicked.connect(lambda: self.save_full_frame("con_bache"))
        self.ui.bump_absent_button.clicked.connect(lambda: self.save_full_frame("sin_bache"))

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.qimage = None
        # self.sam = SAM("sam_b.pt")
        self.sam_masks = None

        self.current_node_id = None
        self._projected_problem_positions = {}  # node id -> last-projected problem_position value

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("dsr signals connected")
        except RuntimeError as e:
            print(e)
    
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)


    def __del__(self):
        """Destructor"""

    def log(self, msg):
        """Print to terminal and mirror the same message in the UI's debug log."""
        print(msg)
        self.ui.debug_log.appendPlainText(str(msg))


    @QtCore.Slot()
    def compute(self):
        try:
            # get the image from the camera
            image_struct = self.camerargbdsimple_proxy.getImage("camera")
            if not image_struct.image:
                print("ERROR: Received empty image")
                return True
            
            # convert the image to a numpy array and check its size
            image_np = np.frombuffer(image_struct.image, dtype=np.uint8)
            expected_size = image_struct.width * image_struct.height * 3
            if image_np.size != expected_size:
                print(f"ERROR: Image size mismatch. Expected {expected_size}, got {image_np.size}")
                return True

            # convert the image from BGR to RGB and get its dimensions
            image_np = image_np.reshape((image_struct.height, image_struct.width, 3))
            image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            h, w, ch = image_rgb.shape

            # process the image with SAM
            # if self.sam_masks is None:
            #     spinner = AnimatedSpinner()
            #     spinner.start()
            #     results = self.sam(image_rgb, device=self.device, verbose=False)
            #     spinner.stop()
            #     if results[0].masks is not None:
            #         self.sam_masks = results[0].masks.data.cpu().numpy()
            
            # # overlay masks on the image with random colors
            # if self.sam_masks is None:
            #     print("No masks detected by SAM.")
            #     return True

            # for mask in self.sam_masks:
            #     mask = mask.astype(np.uint8)
            #     if mask.shape != (h, w):
            #         mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            #     color = np.random.randint(0, 255, (3,), dtype=np.uint8)
            #     image_rgb[mask > 0] = image_rgb[mask > 0] * 0.5 + color * 0.5

            # Show the image in the GUI
            # if self.sam_masks is not None:

            qimage = QImage(image_rgb.data, w, h, w * ch, QImage.Format_RGB888).copy()
            self.ui.image_label.setPixmap(QPixmap.fromImage(qimage))

        except RuntimeError as e:
            # if 'spinner' in locals():
            #     spinner.stop()
            if "CUDA out of memory" in str(e):
                print("CUDA out of memory error. Consider using a smaller model or reducing the image size.")
                torch.cuda.empty_cache()
                QApplication.instance().quit()
                return False
            else:
                print(f"RUNTIME ERROR ON COMPUTE: {e}")

        except Exception as e:
            # if 'spinner' in locals():
            #     spinner.stop()
            print(f"ERROR ON COMPUTE: {e}")

        return True

    def _scale_label_pos_to_image(self, pos):
        """Scale a mouse position in image_label's widget coordinates to the
        underlying pixmap's native resolution (label may display it scaled).
        """
        if self.ui.image_label.pixmap():
            pix_w = self.ui.image_label.pixmap().width()
            pix_h = self.ui.image_label.pixmap().height()
            lbl_w = self.ui.image_label.width()
            lbl_h = self.ui.image_label.height()
            return int(pos.x() * pix_w / lbl_w), int(pos.y() * pix_h / lbl_h)
        return pos.x(), pos.y()

    def eventFilter(self, watched, event):
        # filter events within image label
        if watched == self.ui.image_label:
            # handle mouse hover tracking
            if event.type() == QtCore.QEvent.MouseMove:
                pos = event.position().toPoint()
                scaled_x, scaled_y = self._scale_label_pos_to_image(pos)
                self.ui.image_coords_label.setText(f"Mouse at: ({scaled_x}, {scaled_y})")
            # handle mouse click events
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                pos = event.position().toPoint()
                scaled_x, scaled_y = self._scale_label_pos_to_image(pos)
                self.selected_point = (scaled_x, scaled_y)
                self.ui.image_sel_coords_label.setText(f"Selected point: ({scaled_x}, {scaled_y})")
        return super(SpecificWorker, self).eventFilter(watched, event)

    def on_segment_button_clicked(self):
        if self.selected_point is not None:
            x, y = self.selected_point
            self.log(f"Segment button clicked. Processing SAM on point: ({x}, {y})")
            self.process_sam_on_point(x, y)
        else:
            self.log("No point selected. Please click on the image to select a point before segmenting.")

    def on_save_new_folder_button_clicked(self):
        folder_name = f"session_{time.strftime('%Y%m%d_%H%M%S')}"
        self.current_save_dir = os.path.join("segmented_objects", folder_name)
        os.makedirs(self.current_save_dir, exist_ok=True)
        self.ui.current_folder_label.setText(f"Saving to: {self.current_save_dir}")
        self.log(f"New save folder: {self.current_save_dir}")

    def get_current_rgbd(self):
        """Fetch the current RGB image and depth (meters) from the live "camera",
        aligned to the same resolution.
        """
        try:
            rgbd = self.camerargbdsimple_proxy.getAll("camera")
            image_struct = rgbd.image
            depth_struct = rgbd.depth
            if not image_struct.image or not depth_struct.depth:
                return None, None

            image_np = np.frombuffer(image_struct.image, dtype=np.uint8)
            image_np = image_np.reshape((image_struct.height, image_struct.width, 3))
            image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

            depth_np = np.frombuffer(depth_struct.depth, dtype=np.float32)
            depth_np = depth_np.reshape((depth_struct.height, depth_struct.width))
            depth_m = depth_np * depth_struct.depthFactor

            if depth_m.shape != image_rgb.shape[:2]:
                depth_m = cv2.resize(depth_m, (image_rgb.shape[1], image_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

            return image_rgb, depth_m
        except Exception as e:
            self.log(f"ERROR getting current RGBD image: {e}")
            return None, None


    def process_sam_on_point(self, x, y):
        # Load SAM
        if not hasattr(self, 'sam') or self.sam is None:
            self.log("Loading SAM model...")
            from ultralytics import SAM # import here for lazy loading
            self.sam = SAM("sam2.1_l.pt")

        # Get current RGBD
        image_rgb, depth_m = self.get_current_rgbd()
        if image_rgb is None:
            self.log("No image available to process.")
            return

        # Process the image with SAM
        self.log(f"Processing SAM on point: ({x}, {y})")
        results = self.sam(image_rgb, points=[(x, y)], labels=[1], device=self.device, verbose=False)

        # Get mask and save results
        if results[0].masks is not None:
            mask = results[0].masks.data.cpu().numpy()[0]
            self.save_segmented_rgbd(mask, image_rgb, depth_m)
        else:
            self.log("No mask detected by SAM for the given point.")


    def get_room_to_camera_transform(self):
        """Compose the fixed zed mount transform with the live room->robot RT edge
        to get the room->camera transform (4x4 homogeneous, millimeters).
            Returns:
                - np.ndarray | None: 4x4 transform, or None if robot/room/RT missing.
        """
        robot_node = self.g.get_node("robot")
        room_node = self.g.get_node("room")
        if robot_node is None or room_node is None:
            return None

        rt_edge = self.g.get_edge(room_node.id, robot_node.id, "RT")
        if rt_edge is None or "rt_translation" not in rt_edge.attrs or "rt_quaternion" not in rt_edge.attrs:
            return None

        translation = list(rt_edge.attrs["rt_translation"].value)
        if ROOM_ROBOT_RT_TRANSLATION_IS_METERS:
            translation = [v * 1000.0 for v in translation]
        qx, qy, qz, qw = rt_edge.attrs["rt_quaternion"].value
        print(f"[DEBUG] room->robot translation (mm): {translation}, quaternion (x,y,z,w): {(qx, qy, qz, qw)}")

        room_to_robot = _homogeneous_transform(_rotation_matrix_from_quaternion(qx, qy, qz, qw), translation)
        robot_to_camera = _homogeneous_transform(
            _rotation_matrix_from_axis_angle(ZED_MOUNT_ROTATION_AXIS, ZED_MOUNT_ROTATION_ANGLE_RAD),
            ZED_MOUNT_OFFSET_MM,
        )
        return room_to_robot @ robot_to_camera

    def project_point_3d_to_2d(self, point_room_mm):
        """Project a 3D point expressed in the 'room' frame (millimeters) onto the
        zed camera's image plane.
            Parameters:
                - point_room_mm (list[float]): [x, y, z] in the 'room' frame, mm.
            Returns:
                - tuple[int, int] | None: (u, v) pixel coordinates, or None if the
                  point is behind the camera, out of frame, or data is unavailable.
        """
        room_to_camera = self.get_room_to_camera_transform()
        if room_to_camera is None:
            print("Cannot project: room->camera transform unavailable (missing robot/room/RT).")
            return None

        camera_to_room = np.linalg.inv(room_to_camera)
        point_room_h = np.array([point_room_mm[0], point_room_mm[1], point_room_mm[2], 1.0])
        point_camera = camera_to_room @ point_room_h
        print(f"[DEBUG] point in room frame (mm): {point_room_mm}, point in camera frame (mm): {point_camera[:3].tolist()}")

        # Webots device convention (Camera/RangeFinder/Lidar): looks down local +X,
        # with +Y left and +Z up (ROS-style body axes, not OpenGL -Z-forward).
        # Confirmed against real debug data: using +Z as depth gave ~1000mm for a
        # point actually ~3900mm away; +X matches the true distance.
        depth = point_camera[0]
        if depth <= 0:
            print(f"Cannot project: point is behind the camera (depth={depth:.1f}mm).")
            return None

        try:
            image_struct = self.camerargbdsimple_proxy.getImage(ZED_CAMERA_NAME)
        except Exception as e:
            print(f"Cannot project: error fetching '{ZED_CAMERA_NAME}' image for intrinsics: {e}")
            return None
        print(f"[DEBUG] image intrinsics: width={image_struct.width}, height={image_struct.height}, "
              f"focalx={image_struct.focalx}, focaly={image_struct.focaly}, depth={depth:.1f}mm")

        u = image_struct.width / 2 - image_struct.focalx * (point_camera[1] / depth)
        v = image_struct.height / 2 - image_struct.focaly * (point_camera[2] / depth)

        if not (0 <= u < image_struct.width and 0 <= v < image_struct.height):
            print(f"Point projects outside the image frame: ({u:.1f}, {v:.1f}) vs {image_struct.width}x{image_struct.height}.")
            return None

        return int(u), int(v)


    def save_segmented_rgbd(self, mask, image_rgb, depth_m):
        """Save a segmented RGBD crop at the selected point: background zeroed out
        (both color and depth) and cropped to the mask's bounding box. No detection
        label - just the segmented RGB (.jpg) + depth in meters (.npy). Also updates
        the right-hand "segmented" viewer with the result.
        """
        os.makedirs(self.current_save_dir, exist_ok=True)

        mask_uint8 = (mask * 255).astype(np.uint8)
        if mask_uint8.shape != image_rgb.shape[:2]:
            mask_uint8 = cv2.resize(mask_uint8, (image_rgb.shape[1], image_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        x, y, w, h = cv2.boundingRect(mask_uint8)
        if w == 0 or h == 0:
            self.log("Cannot save: SAM mask is empty, nothing to crop.")
            return

        mask_bool = mask_uint8 > 0
        masked_rgb = image_rgb.copy()
        masked_rgb[~mask_bool] = 0
        masked_depth = depth_m.copy()
        masked_depth[~mask_bool] = 0.0

        rgb_crop = np.ascontiguousarray(masked_rgb[y:y + h, x:x + w])
        depth_crop = np.ascontiguousarray(masked_depth[y:y + h, x:x + w])

        basename = f"bump_{int(time.time())}"
        image_path = os.path.join(self.current_save_dir, f"{basename}_rgb.jpg")
        depth_path = os.path.join(self.current_save_dir, f"{basename}_depth.npy")

        cv2.imwrite(image_path, cv2.cvtColor(rgb_crop, cv2.COLOR_RGB2BGR))
        np.save(depth_path, depth_crop)

        crop_h, crop_w, crop_ch = rgb_crop.shape
        if crop_h > 0 and crop_w > 0:
            qimage = QImage(rgb_crop.data, crop_w, crop_h, crop_w * crop_ch, QImage.Format_RGB888).copy()
            self.ui.segmented_image_label.setPixmap(QPixmap.fromImage(qimage))

        self.log(f"Saved segmented RGBD: {image_path}, {depth_path}")

    def save_full_frame(self, label):
        """Save the raw, unmodified RGBD frame (no SAM, no crop) into <save_dir>/<label>/,
        for the presence/absence floor classifier: same image format either way, the only
        difference is whether the bump happens to be in view when this is clicked.
        """
        image_rgb, depth_m = self.get_current_rgbd()
        if image_rgb is None:
            self.log("No image available to save.")
            return

        save_dir = os.path.join(self.current_save_dir, label)
        os.makedirs(save_dir, exist_ok=True)

        basename = f"floor_{int(time.time())}"
        image_path = os.path.join(save_dir, f"{basename}_rgb.jpg")
        depth_path = os.path.join(save_dir, f"{basename}_depth.npy")

        cv2.imwrite(image_path, cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
        np.save(depth_path, depth_m)

        h, w, ch = image_rgb.shape
        qimage = QImage(image_rgb.data, w, h, w * ch, QImage.Format_RGB888).copy()
        self.ui.segmented_image_label.setPixmap(QPixmap.fromImage(qimage))

        self.log(f"Saved '{label}' frame: {image_path}, {depth_path}")


    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        QTimer.singleShot(200, QApplication.instance().quit)


    # =============== DSR SLOTS ===================
    # =============================================

    def maybe_project_problem_position(self, id: int) -> None:
        """Check node 'id' for a 'problem_position' attribute and trigger a SAM
        capture at its projected pixel, once per distinct value. Needed because
        DSR node creation (insert_node) only emits UPDATE_NODE, never
        UPDATE_NODE_ATTR (that one only fires on later updates, e.g. the RT
        edge's level/parent), so a fresh node's initial attributes must be
        picked up from the UPDATE_NODE signal too.
        """
        node = self.g.get_node(id)
        if node is None or "problem_position" not in node.attrs:
            return

        point_room_mm = list(node.attrs["problem_position"].value)
        if self._projected_problem_positions.get(id) == point_room_mm:
            return  # Already processed this exact value for this node.
        self._projected_problem_positions[id] = point_room_mm

        pixel = self.project_point_3d_to_2d(point_room_mm)
        if pixel is not None:
            x, y = pixel
            self.log(f"Projected 3D point {point_room_mm} to pixel ({x}, {y}) on node {id}. Processing SAM.")
            self.ui.image_sel_coords_label.setText(f"Selected point: ({x}, {y})")
            self.process_sam_on_point(x, y)

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        try:
            if "problem_position" in attribute_names:
                self.maybe_project_problem_position(id)
        except Exception as e:
            print(f"ERROR in update_node_att: {e}")


    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')
        self.current_node_id = id
        try:
            self.maybe_project_problem_position(id)
        except Exception as e:
            print(f"ERROR in update_node: {e}")

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')
        if self.current_node_id == id:
            self.current_node_id = None

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')
        # check if if its RT, get x-y position and process SAM on that position
        if type != "RT":
            return
    
        print(f"Processing SAM on edge from {fr} to {to} of type {type}")
        # add code to get x-y position from attributes
        x, y = 320, 240 # example
        self.process_sam_on_point(x, y)            
            

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')


    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # RoboCompCameraRGBDSimple.TRGBD self.camerargbdsimple_proxy.getAll(str camera)
    # RoboCompCameraRGBDSimple.TDepth self.camerargbdsimple_proxy.getDepth(str camera)
    # RoboCompCameraRGBDSimple.TImage self.camerargbdsimple_proxy.getImage(str camera)
    # RoboCompCameraRGBDSimple.TPoints self.camerargbdsimple_proxy.getPoints(str camera)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # ifaces.RoboCompCameraRGBDSimple.Point3D
    # ifaces.RoboCompCameraRGBDSimple.TPoints
    # ifaces.RoboCompCameraRGBDSimple.TImage
    # ifaces.RoboCompCameraRGBDSimple.TDepth
    # ifaces.RoboCompCameraRGBDSimple.TRGBD
