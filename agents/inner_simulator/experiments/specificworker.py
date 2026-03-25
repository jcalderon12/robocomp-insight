import os
import sys
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from ultralytics import YOLO
from genericworker import *
import pybullet as p
import pybullet_data as pd
import pybullet_utils.bullet_client as bc
from agent_training.yolo_dataset import YoloDataset
from agent_training.yolo_trainer import YoloTrainer
import cv2
import random
import multiprocessing as mp
import signal
import atexit

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
from pydsr import *
import numpy as np

CONCEPT_NAME = "bump"
MODEL_OUTPUT_DIR = "yolo_concept_model"


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        print("SpecificWorker initialized")

        # ================= PYBULLET MODELS LOADING  ================
        # ===========================================================

        self.dt = 1./62. # Simulation time step (60 Hz)

        # TImage getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);
        self.robot_img = self.camerargbdsimple_proxy.getImage("zed") # A bump in the way --- IGNORE ---
        self.print_time = self.actual_time = time.time()

        # ================ SIMULATION PARAM  ===============
        # ==================================================
        
        # Dataset initialization variables
        self.dataset_size = 5  # number of full row-scenes to generate
        self.augment_per_image = 0  # number of augmented copies per original image
        self.negative_sample_ratio = 1  # number of negative images generated per positive scene/thread
        self.dataset = YoloDataset(os.getcwd(), "concept_dataset", "concept")
        self.debug_save_bbox_images = True
        self.debug_bbox_dir = os.path.join(os.getcwd(), "debug_bboxes")
        os.makedirs(self.debug_bbox_dir, exist_ok=True)

        # Grid-based object positioning parameters
        self.grid_center = [0,0]  # [x, y] center of the map
        self.grid_spacing = 1.0  # distance between neighboring objects in the grid
        self.grid_size_n = 5  # NxN matrix size

        # Precompute grid positions; orientation is randomized per generated scene.
        self.grid_positions = self._generate_grid_positions()

        # Camera parameters
        self.camera_fixed_pos = np.array([0, 0, 0.94259881])
        self.camera_forward = [0, -1]  # Camera looking in -Y direction
        self.cam_near = 0.01
        self.cam_far = 100
        
        # Status variables
        self.has_concept_been_found = False

        # Multiprocessing count. Sets how many proccesses to use.
        self.instance_count = 12
        self._owner_pid = os.getpid()
        self._dataset_workers = []
        self._register_termination_handlers()
         
        # Initialize random objects dictionary for training variety
        self.init_random_obj_dict()
        self.random_objects_instances = []
        self.random_object_count_range = (3, 8)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            
        self.stage = "IDLE"

    def init_random_obj_dict(self):
        """Initialize a dictionary of ~50 random objects from pybullet_data for training.
        Each object has a label and path to the URDF/OBJ file."""
        datapath = pd.getDataPath()
        self.random_objects = {
            # Primitives and simple rigid bodies
            "sphere_small": {"label": "sphere", "path": os.path.join(datapath, "sphere_small.urdf")},
            "sphere_1cm": {"label": "sphere", "path": os.path.join(datapath, "sphere_1cm.urdf")},
            "sphere2": {"label": "sphere", "path": os.path.join(datapath, "sphere2.urdf")},
            "cube": {"label": "cube", "path": os.path.join(datapath, "cube.urdf")},
            "cube_small": {"label": "cube", "path": os.path.join(datapath, "cube_small.urdf")},
            "block": {"label": "block", "path": os.path.join(datapath, "block.urdf")},

            # Small objects and toys
            "duck": {"label": "duck", "path": os.path.join(datapath, "duck_vhacd.urdf")},
            "jenga": {"label": "jenga", "path": os.path.join(datapath, "jenga", "jenga.urdf")},
            "lego": {"label": "lego", "path": os.path.join(datapath, "lego", "lego.urdf")},
            "bunny": {"label": "animal", "path": os.path.join(datapath, "bunny.obj")},
            "stone": {"label": "stone", "path": os.path.join(datapath, "stone.obj")},
            "soccerball": {"label": "ball", "path": os.path.join(datapath, "soccerball.obj")},

            # Grippers
            "gripper_left": {"label": "gripper", "path": os.path.join(datapath, "gripper", "wsg50_one_motor_gripper_left_finger.urdf")},
            "gripper_right": {"label": "gripper", "path": os.path.join(datapath, "gripper", "wsg50_one_motor_gripper_right_finger.urdf")},
            "pr2_gripper": {"label": "gripper", "path": os.path.join(datapath, "pr2_gripper.urdf")},

            # Furniture and structures
            "table": {"label": "table", "path": os.path.join(datapath, "table", "table.urdf")},
            "table_square": {"label": "table", "path": os.path.join(datapath, "table_square", "table_square.urdf")},
            "tray": {"label": "tray", "path": os.path.join(datapath, "tray", "tray.urdf")},
            "concave_box": {"label": "box", "path": os.path.join(datapath, "toys", "concave_box.urdf")},

            # Robots and vehicles
            "r2d2": {"label": "robot", "path": os.path.join(datapath, "r2d2.urdf")},
            "humanoid": {"label": "humanoid", "path": os.path.join(datapath, "humanoid", "humanoid.urdf")},
            "cartpole": {"label": "cartpole", "path": os.path.join(datapath, "cartpole.urdf")},
            "husky": {"label": "robot", "path": os.path.join(datapath, "husky", "husky.urdf")},
            "kuka_iiwa": {"label": "robot_arm", "path": os.path.join(datapath, "kuka_iiwa", "model.urdf")},
            "racecar": {"label": "vehicle", "path": os.path.join(datapath, "racecar", "racecar.urdf")},
            "cone": {"label": "cone", "path": os.path.join(datapath, "racecar", "meshes", "cone.obj")},

            # Animals and deformables
            "teddy": {"label": "animal", "path": os.path.join(datapath, "teddy_vhacd.urdf")},
            "samurai": {"label": "object", "path": os.path.join(datapath, "samurai.urdf")},
            "cloth": {"label": "cloth", "path": os.path.join(datapath, "cloth_z_up.urdf")},

            # Environment
            "plane": {"label": "plane", "path": os.path.join(datapath, "plane.urdf")},
        }

    def __del__(self):
        """Destructor"""
        self._terminate_dataset_workers()


    @QtCore.Slot()
    def compute(self):
        self.show_compute_time_step()
        self.publish_status_to_dsr()

        match self.stage:
            
            case "IDLE":
                # If model already exists go directly to testing. Otherwise go to training.
                model = os.path.join(os.getcwd(), "runs", "detect", MODEL_OUTPUT_DIR, "weights", "best.pt")
                if os.path.exists(model):
                    console.print("Trained model found. Skipping training and going directly to analysis stage.", style="bold green")
                    self.trained_model = model
                    self.stage = "ANALYZE"
                else:
                    console.print("No trained model found. Starting training stage.", style="bold yellow")
                    self.stage = "TRAINING"


            case "TRAINING":
                total_positions = self.dataset_size
                console.print(
                    f"Generating dataset of {total_positions} grid-scenes "
                    f"({self.grid_size_n}x{self.grid_size_n}, objects per scene: {len(self.grid_positions)})...",
                    style="bold yellow",
                )
                
                # Distribute the object positions between the instances
                worker_count = max(1, min(self.instance_count, total_positions))
                points = []
                for i in range(worker_count):
                    points.append([])
                for i in range(total_positions):
                    points[i % worker_count].append(i)
                
                try:
                    # Start the processes to generate the dataset in parallel
                    self._dataset_workers = []
                    for i in range(worker_count):
                        if not points[i]:
                            continue
                        worker = mp.Process(target=self.render_images, args=(points[i],), daemon=True)
                        worker.start()
                        self._dataset_workers.append(worker)

                    # Wait for all processes to finish
                    for worker in self._dataset_workers:
                        worker.join()
                except KeyboardInterrupt:
                    console.print("Training interrupted. Terminating dataset workers...", style="bold red")
                    self._terminate_dataset_workers()
                    self.stage = "TERMINATE"
                    return True
                finally:
                    self._terminate_dataset_workers()

                console.print(f"Dataset complete: {self.dataset.get_dataset_size()} images (incl. augmentations)", style="bold green")

                # Build dataset YAML and train
                self.dataset.create_yaml()
                self.trainer = YoloTrainer(base_path=os.getcwd(), dataset_path=self.dataset.get_data_yaml_path(), model_name="yolov8n.pt", epochs=20, output_dir=MODEL_OUTPUT_DIR)
                self.trainer.init_training()
                print(self.trainer.get_trained_model_path())
                self.trained_model = self.trainer.get_trained_model_path()
                self.stage = "ANALYZE"
                
            
            case "ANALYZE":
                # Load the trained YOLO model
                model = YOLO(self.trained_model)
                while True:
                    # Capture image 60Hz at a time
                    start_time = time.time()
                    # Detect if CTRL+C has been pressed to break the loop
                    if QApplication.instance().closingDown():
                        break
                    
                    # Get image from robot camera
                    self.robot_img = self.camerargbdsimple_proxy.getImage("zed")
                    robot_img_array = np.frombuffer(self.robot_img.image, dtype=np.uint8)
                    self.robot_img_image = robot_img_array.reshape((self.robot_img.height, self.robot_img.width, 3))
                    # Analyze image (current robot image)
                    results = model.predict(self.robot_img_image, verbose=False)
                    if len(results[0].boxes) > 0:
                        # Set new status
                        self.has_concept_been_found = True
                        
                        # Draw the results on it
                        img_with_detections = self.robot_img_image.copy()
                        for result in results:
                            boxes = result.boxes.xyxy.cpu().numpy().astype(int)  # Bounding box coordinates
                            for box in boxes:
                                x1, y1, x2, y2 = box
                                cv2.rectangle(img_with_detections, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box
                                # draw the confidence score on top of the box                                
                                confidence = result.boxes.conf.cpu().numpy()
                                label = f"{confidence[0]:.2f}"
                                cv2.putText(img_with_detections, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                        self.robot_img_image = img_with_detections
                        
                    else:
                        # draw a small red cross inside a box on the image top left corner to indicate that the concept has not been found
                        img_with_detections = self.robot_img_image.copy()
                        cv2.line(img_with_detections, (10, 10), (30, 30), (0, 0, 255), 2)
                        cv2.line(img_with_detections, (30, 10), (10, 30), (0, 0, 255), 2)
                        cv2.rectangle(img_with_detections, (5, 5), (35, 35), (0, 0, 255), 2)
                        self.robot_img_image = img_with_detections
                        self.has_concept_been_found = False

                    # Publish the current status
                    self.publish_status_to_dsr()
                    
                    # Display the original image
                    cv2.imshow("Agent Vision (TM)", self.robot_img_image)
                    cv2.waitKey(1)
                    # wait for the remaining time to complete 1/60s
                    elapsed = time.time() - start_time
                    wait_time = max(0, self.dt - elapsed)
                    time.sleep(wait_time)
                    
                self.stage = "TERMINATE"

            case "TERMINATE":
                pass

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)



    # ================ PYBULLET SIMULATION RELATED  ================
    # ==============================================================

    def init_pybullet_sim(self):
        """
        Initializes a new PyBullet simulation instance with plane and concept.
        Configures lighting for good visibility.
        """
        client = bc.BulletClient(connection_mode=p.DIRECT)
        client.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)
        client.setGravity(0, 0, -9.81)
        
        # Configure rendering
        client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        client.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        client.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        client.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # LOAD PLANE IN THE SIMULATION
        client.loadURDF("../../../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0])

        return client

    def render_images(self, dataset_index_list):
        """
        Creates a new Pybullet instance and renders a set of images with the concept
        positioned in a centered NxN grid over the plane.
        Objects are converted to grayscale and processed for YOLO training.
        It's meant to be called in parallel using multiprocessing.
        """

        # Create a new pybullet instance
        client = self.init_pybullet_sim()

        # Create one concept instance per grid position once per worker process.
        scene_object_count = len(self.grid_positions)
        concept_ids = []
        for _ in range(scene_object_count):
            concept_id = client.loadURDF("../../../../etc/URDFs/bump/bump_100x5cm.urdf", [0, 0, 0])
            # Set the concept color to red
            client.changeVisualShape(concept_id, -1, rgbaColor=[1, 0, 0, 1])
            # Append concept to concept list
            concept_ids.append(concept_id)

        for dataset_index in dataset_index_list:
            if dataset_index >= self.dataset_size:
                continue

            # Randomize object orientation for this scene.
            scene_objects = self._generate_scene_objects_with_random_orientation()
            for concept_id, scene_obj in zip(concept_ids, scene_objects):
                client.resetBasePositionAndOrientation(concept_id, scene_obj["position"], scene_obj["orientation"])
            
            # Capture image from fixed camera position looking at the objects
            capture = self.capture_zed_image(
                self.camera_fixed_pos,
                self.camera_forward,
                light_direction=self._random_light_direction(),
            )

            # Convert RGBA to BGR
            img_bgr = cv2.cvtColor(capture, cv2.COLOR_RGBA2BGR)
            
            # Detect all objects and create labels
            labels = self._detect_and_label_all_objects(img_bgr)

            if self.debug_save_bbox_images:
                debug_img = self._draw_yolo_bboxes(img_bgr, labels)
                debug_name = f"scene_{dataset_index:04d}_pid_{os.getpid()}.png"
                cv2.imwrite(os.path.join(self.debug_bbox_dir, debug_name), debug_img)
            
            # Convert to grayscale (single channel black and white)
            img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            
            # Determine if training or validation split
            is_train = dataset_index < int(0.8 * self.dataset_size)
            
            # Add image and labels to dataset
            self.dataset.add_image(img_gray, labels, is_train)

            # Data augmentation (only for training images)
            if is_train:
                self._augment_and_add_grayscale(img_gray, labels, is_train=True)

            # Generate negative samples (images without the concept) based on negative_sample_ratio
            num_negative_samples = max(0, int(self.negative_sample_ratio))
            for _ in range(num_negative_samples):
                # Hide all concepts by moving them far below the plane.
                for concept_id in concept_ids:
                    hidden_concept_pos = [0, 0, -10.0]
                    client.resetBasePositionAndOrientation(concept_id, hidden_concept_pos, [0, 0, 0, 1])
                
                # Add random objects for visual variety
                self.add_random_objects(client, random.randint(*self.random_object_count_range))
                capture = self.capture_zed_image(
                    self.camera_fixed_pos,
                    self.camera_forward,
                    light_direction=self._random_light_direction(),
                )
                self.remove_random_objects(client)
                
                # Convert to grayscale
                img_bgr = cv2.cvtColor(capture, cv2.COLOR_RGBA2BGR)
                
                # Add as negative image
                self.dataset.add_negative_image(img_bgr, is_train)
            
            console.print(f"  [THREAD:ID{dataset_index}] finished generating image.", style="yellow")

        client.disconnect()
        
        
        
    # ====================  HELPERS  =====================
    # ====================================================

    def _register_termination_handlers(self):
        """Ensure child processes are terminated when this process receives a stop signal."""
        atexit.register(self._terminate_dataset_workers)
        signal.signal(signal.SIGINT, self._handle_termination_signal)
        signal.signal(signal.SIGTERM, self._handle_termination_signal)

    def _handle_termination_signal(self, signum, _frame):
        """Handle termination signals by terminating dataset workers and exiting."""
        if os.getpid() == self._owner_pid:
            console.print(f"Signal {signum} received. Terminating dataset workers...", style="bold red")
            self._terminate_dataset_workers()
        raise SystemExit(128 + signum)

    def _terminate_dataset_workers(self):
        """Terminate and join running dataset worker processes."""
        # Multiprocessing children created via fork inherit this object and the
        # atexit handler, but they must not manage Process objects created by
        # the parent process.
        if os.getpid() != self._owner_pid:
            return

        for worker in self._dataset_workers:
            if worker.is_alive():
                worker.terminate()
        for worker in self._dataset_workers:
            worker.join(timeout=2)
        self._dataset_workers = []

    def publish_status_to_dsr(self):
        """ Publishes the current status of the concept to DSR."""
        # check if we have found the concept in this compute step. if so, publish the possible_concept object node to DSR. If not, destroy it (if it exists)
        if not self.has_concept_been_found:
            # check if we have to destroy the node in DSR
            if self.g.get_node(f"object_{CONCEPT_NAME}") is not None:
                # destroy node (edge is implicitly destroyed)
                console.print(f"Destroying object node {CONCEPT_NAME}", style="bold red")
                self.g.delete_node(f"object_{CONCEPT_NAME}")
        else:
            # check if we have to create the node in DSR
            if self.g.get_node(f"object_{CONCEPT_NAME}") is None:
                # create node
                console.print(f"Creating object node {CONCEPT_NAME}", style="bold green")
                object_node = Node(agent_id=self.agent_id, name=f"object_{CONCEPT_NAME}", type="object")
                self.g.insert_node(object_node)                
                # create edge
                robot_node = self.g.get_node("robot")
                if robot_node is None:
                    console.print("Robot node not found in DSR graph. This should NOT happen. Exiting...", style="bold red")
                    exit()
                edge = Edge(object_node.id, robot_node.id, "thinks", self.agent_id)
                self.g.insert_or_assign_edge(edge)
                
    def show_compute_time_step(self):
        """
        Get the time step between compute calls
        :return: time
        """
        time_step = time.time() - self.actual_time
        self.actual_time = time.time()
        if time.time() - self.print_time > 5:
            self.print_time = time.time()
            console.print(f"Compute frequency: {1/time_step:.2f} Hz", style="bold blue")
            
        return time_step



    # ====================  RENDERING HELPERS  =====================
    # ==============================================================
    
    def capture_zed_image(self, cam_pos: list, forward: list, fov_v: float = 70.0, light_direction: list = None) -> np.ndarray:
        """
        Renders a perspective image from the ZED camera simulation using PyBullet.
        
        :param cam_pos: [x, y, z] world position of the camera
        :param forward: [fx, fy] normalized 2-D forward direction vector in the XY plane
        :param fov_v: Vertical field of view in degrees (default: 70° for f/2.0 lens)
        :return: numpy array (height, width, 4) RGBA image
        
        ZED Camera Specifications (dual lens configuration):
        
        Lens 1 (f/2.0) - Primary lens used:
        - Focal Length: 2.1mm
        - Aperture: ƒ/2.0
        - Field of View: 110°(H) x 70°(V) x 120°(D) max.
        - Resolution: 1280 x 720 pixels
        
        Lens 2 (f/1.8) - Alternative:
        - Focal Length: 4mm
        - Aperture: ƒ/1.8
        - Field of View: 72°(H) x 44°(V) x 81°(D) max.
        
        Wide-angle 8-element all-glass dual lens with optically corrected distortion
        """
        # Normalize the forward vector
        norm = np.sqrt(forward[0]**2 + forward[1]**2)
        if norm < 1e-6:
            forward = [1, 0]  # Default to +X if invalid
        else:
            forward = [forward[0]/norm, forward[1]/norm]
        
        fx, fy = forward
        
        # ZED camera resolution: 1280 x 720 pixels
        width = 1280
        height = 720
        
        # Build the up vector perpendicular to forward and world Z
        # Local frame: +X = (fx,fy,0), +Z = (0,0,1), +Y = (-fy,fx,0)
        up = [0, 0, 1]
        
        # ZED aspect ratio: 1280 / 720
        aspect_ratio = 1280.0 / 720.0
        
        # Target point in the direction the camera is looking
        target = [
            cam_pos[0] + fx,
            cam_pos[1] + fy,
            cam_pos[2]
        ]
        
        # Create view matrix
        view_matrix = p.computeViewMatrix(cameraEyePosition=cam_pos,
                                         cameraTargetPosition=target,
                                         cameraUpVector=up)
        
        # Create projection matrix with the specified FOV
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=fov_v,                    # PyBullet uses vertical FOV
            aspect=aspect_ratio,
            nearVal=self.cam_near,
            farVal=self.cam_far
        )
        
        # Render the image
        if light_direction is None:
            light_direction = [0.5, 0.5, 1.0]

        _, _, rgb, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            lightDirection=light_direction,
        )
        
        # Convert to numpy array with RGBA format
        image = np.array(rgb, dtype=np.uint8).reshape(height, width, 4)
        
        return image

    # ==================  DETECTION & AUGMENTATION ==================
    # ===============================================================

    def _generate_grid_positions(self):
        """
        Generate static object positions in an NxN grid centered at `grid_center`.
        The central cell is intentionally skipped (no object at the center).
        """
        positions = []
        center_idx = self.grid_size_n // 2

        for row_idx in range(self.grid_size_n):
            for col_idx in range(self.grid_size_n):
                # Skip central grid position.
                if row_idx == center_idx and col_idx == center_idx:
                    continue

                x = self.grid_center[0] + (col_idx - center_idx) * self.grid_spacing
                y = self.grid_center[1] + (row_idx - center_idx) * self.grid_spacing
                z = -0.05
                positions.append([x, y, z])

        return positions

    def _generate_scene_objects_with_random_orientation(self):
        """Create one full scene with random orientation only around Z axis."""
        scene_objects = []
        for pos in self.grid_positions:
            # PyBullet euler order is [roll(X), pitch(Y), yaw(Z)].
            # Keep random rotation exclusively on Z axis.
            roll = 0.0
            pitch = 0.0
            yaw = random.uniform(-np.pi, np.pi)
            orn = p.getQuaternionFromEuler([roll, pitch, yaw])
            scene_objects.append({"position": pos, "orientation": orn})
        return scene_objects

    def _random_light_direction(self):
        """Return a normalized random light direction with positive Z (overhead light)."""
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(0.35, 1.0)
        norm = np.sqrt(x * x + y * y + z * z)
        if norm < 1e-6:
            return [0.5, 0.5, 1.0]
        return [x / norm, y / norm, z / norm]
    
    def add_random_objects(self, client, num_objects=5):
        """Adds random objects to the simulation for visual variety."""
        self.random_objects_instances = []
        if not self.random_objects:
            return

        object_keys = list(self.random_objects.keys())
        num_objects = max(0, min(num_objects, len(object_keys)))
        if num_objects == 0:
            return

        selected_keys = random.sample(object_keys, num_objects)
        for key in selected_keys:
            obj_data = self.random_objects[key]
            obj_path = obj_data["path"]
            _, ext = os.path.splitext(obj_path)
            ext = ext.lower()

            # Place distractors around the concept while keeping the center mostly visible.
            angle = random.uniform(0, 2 * np.pi)
            radius = random.uniform(0.8, 3.2)
            pos = [
                radius * np.cos(angle),
                radius * np.sin(angle),
                random.uniform(0.02, 0.35),
            ]
            orn = client.getQuaternionFromEuler([0, 0, random.uniform(-np.pi, np.pi)])

            body_id = None
            try:
                if ext == ".urdf":
                    body_id = client.loadURDF(obj_path, basePosition=pos, baseOrientation=orn, useFixedBase=True)
                elif ext in {".obj", ".vtk"}:
                    visual_shape = client.createVisualShape(
                        shapeType=p.GEOM_MESH,
                        fileName=obj_path,
                        meshScale=[0.2, 0.2, 0.2],
                        rgbaColor=[random.random(), random.random(), random.random(), 1],
                    )
                    if visual_shape >= 0:
                        body_id = client.createMultiBody(
                            baseMass=0,
                            baseVisualShapeIndex=visual_shape,
                            baseCollisionShapeIndex=-1,
                            basePosition=pos,
                            baseOrientation=orn,
                        )
            except Exception:
                # Ignore unsupported/random assets and continue with the rest.
                continue

            if body_id is not None and body_id >= 0:
                self.random_objects_instances.append(body_id)

    def remove_random_objects(self, client):
        """Removes previously added random objects from the simulation."""
        for obj in self.random_objects_instances:
            try:
                client.removeBody(obj)
            except Exception:
                pass
        self.random_objects_instances = []



    # ==================  DETECTION & AUGMENTATION ==================
    # ===============================================================

    def _detect_and_label_all_objects(self, img_bgr: np.ndarray) -> str:
        """
        Detect all red bumps directly from the BGR color image and return a
        YOLO-format label string containing all bounding boxes.
        """
        # Mild blur reduces tiny aliasing artifacts from rendering.
        smooth = cv2.GaussianBlur(img_bgr, (5, 5), 0)
        hsv = cv2.cvtColor(smooth, cv2.COLOR_BGR2HSV)

        # Red wraps around HSV hue boundaries; combine both ranges.
        lower_red1 = np.array([0, 25, 20], dtype=np.uint8)
        upper_red1 = np.array([18, 255, 255], dtype=np.uint8)
        lower_red2 = np.array([160, 25, 20], dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        hsv_mask = cv2.bitwise_or(mask_red1, mask_red2)

        # Extra guard in BGR space: dominant red channel.
        b, g, r = cv2.split(smooth)
        red_dominance = ((r.astype(np.int16) - g.astype(np.int16) > 8) &
                 (r.astype(np.int16) - b.astype(np.int16) > 8) &
                 (r > 20)).astype(np.uint8) * 255

        binary = cv2.bitwise_and(hsv_mask, red_dominance)

        # Keep small objects while removing isolated noise.
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        img_h, img_w = img_bgr.shape[:2]
        labels = []
        
        if contours:
            # Lower minimum area so distant bumps are not discarded.
            min_area = (img_h * img_w) * 0.00005
            max_area = (img_h * img_w) * 0.25
            valid_contours = [c for c in contours if min_area < cv2.contourArea(c) < max_area]
            
            for contour in valid_contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 5 and h > 5:  # Minimum size filter
                    # Normalize to YOLO format (center_x, center_y, width, height) in [0, 1]
                    center_x = (x + w / 2) / img_w
                    center_y = (y + h / 2) / img_h
                    norm_w = w / img_w
                    norm_h = h / img_h
                    
                    # Class 0 is "bump".
                    labels.append(f"0 {center_x} {center_y} {norm_w} {norm_h}")

        # Return all labels as a single string separated by newlines
        if labels:
            return "\n".join(labels)
        else:
            return ""

    def _augment_and_add_grayscale(self, img_gray: np.ndarray, labels: str, is_train: bool) -> None:
        """
        Generate augmented copies of a grayscale image + labels and add them to the dataset.
        Augmentations:
          1) Brightness / contrast jitter
          2) Gaussian noise
        """
        for _ in range(self.augment_per_image):
            aug = img_gray.copy()

            # --- 1. Brightness / contrast jitter ---
            alpha = random.uniform(0.7, 1.3)   # contrast
            beta = random.randint(-30, 30)      # brightness
            aug = cv2.convertScaleAbs(aug, alpha=alpha, beta=beta)

            # --- 2. Gaussian noise ---
            if random.random() < 0.5:
                noise = np.random.normal(0, 8, aug.shape).astype(np.int16)
                aug = np.clip(aug.astype(np.int16) + noise, 0, 255).astype(np.uint8)

            self.dataset.add_image(aug, labels, is_train)

    def _draw_yolo_bboxes(self, img_bgr: np.ndarray, labels: str) -> np.ndarray:
        """Return a BGR copy with YOLO label boxes rendered for debug visualization."""
        debug_img = img_bgr.copy()
        if not labels:
            return debug_img

        img_h, img_w = debug_img.shape[:2]
        for line in labels.splitlines():
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            try:
                cls_id = int(float(parts[0]))
                cx = float(parts[1])
                cy = float(parts[2])
                w = float(parts[3])
                h = float(parts[4])
            except ValueError:
                continue

            x1 = int((cx - w / 2.0) * img_w)
            y1 = int((cy - h / 2.0) * img_h)
            x2 = int((cx + w / 2.0) * img_w)
            y2 = int((cy + h / 2.0) * img_h)

            x1 = max(0, min(img_w - 1, x1))
            y1 = max(0, min(img_h - 1, y1))
            x2 = max(0, min(img_w - 1, x2))
            y2 = max(0, min(img_h - 1, y2))

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_img, f"bump:{cls_id}", (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return debug_img