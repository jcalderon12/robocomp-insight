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
import cv2
from ultralytics import SAM
import torch
import sys
import threading
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

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
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # _b for base model, _l for large model
        self.sam = SAM("sam_b.pt")
        self.sam_masks = None

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)


    def __del__(self):
        """Destructor"""


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
            if self.sam_masks is None:
                spinner = AnimatedSpinner()
                spinner.start()
                results = self.sam(image_rgb, device=self.device, verbose=False)
                spinner.stop()
                if results[0].masks is not None:
                    self.sam_masks = results[0].masks.data.cpu().numpy()
            
            # overlay masks on the image with random colors
            if self.sam_masks is None:
                print("No masks detected by SAM.")
                return True

            for mask in self.sam_masks:
                mask = mask.astype(np.uint8)
                if mask.shape != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                color = np.random.randint(0, 255, (3,), dtype=np.uint8)
                image_rgb[mask > 0] = image_rgb[mask > 0] * 0.5 + color * 0.5

            # Show the image in the GUI
            qimage = QImage(image_rgb.data, w, h, w * ch, QImage.Format_RGB888).copy()
            self.ui.image_label.setPixmap(QPixmap.fromImage(qimage))

        except RuntimeError as e:
            if 'spinner' in locals():
                spinner.stop()
            if "CUDA out of memory" in str(e):
                print("CUDA out of memory error. Consider using a smaller model or reducing the image size.")
                torch.cuda.empty_cache()
                QApplication.instance().quit()
                return False
            else:
                print(f"RUNTIME ERROR ON COMPUTE: {e}")

        except Exception as e:
            if 'spinner' in locals():
                spinner.stop()
            print(f"ERROR ON COMPUTE: {e}")

        return True

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


