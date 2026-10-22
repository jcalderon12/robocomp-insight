#!/usr/bin/env python3

import ast
import math
import pybullet as p
import json
import argparse
import time
import sys
import numpy as np
import os
import json
from pybullet_imu import IMU
from draws import DRAW_WIDTHS, DRAWS, DrawControl
from uuid import uuid4
import pkgutil
import importlib
import inspect
from typing import Annotated, Union, List, Type
from causes.cause import Cause
from pydantic import BaseModel, Field, create_model
from simulation_scene import SimulationScene
from engines.engine_pybullet import EnginePybullet
from rich.console import Console
import os
import copy
import traceback

# Get the path to 'inner_simulator' (one level up from 'src')
agent_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if agent_root not in sys.path:
    sys.path.insert(0, agent_root)

# The scene and the physics are in metres. The recorded outputs (bottle and robot positions)
# stay in millimetres, as the verdict and the cause selection read them.
M_TO_MM = 1000
# The step is chosen for convergence (docs_output/resultados/31_arreglos_del_simulador, criterion 2'):
# against 1/500 s, neither 1/125 nor 1/250 gave the same outcome per repetition (89 % and 75 %) and
# the same IMU before the anomaly. The 1/62 s it replaces had been picked because the nominal replay of
# the four-wheel URDF did not knock the bottle off with it, and it did not converge either. A scene may
# set another step (physics_dt).
DEFAULT_PHYSICS_DT = 1. / 500.

# The scene is spawned slightly above the floor; before the episode starts, the bodies settle with
# the wheels held, until nothing moves faster than this (or for at most SETTLE_MAX_S of physics).
SETTLE_MIN_S = 0.5
SETTLE_MAX_S = 3.0
SETTLE_LINEAR_SPEED = 1e-3    # m/s
SETTLE_ANGULAR_SPEED = 1e-2   # rad/s
WHEEL_MAX_FORCE = 10          # N*m, the default maxTorque of a Webots motor

# The contacts of the Webots world. It declares ContactProperties only for the materials of the old
# mecanum wheels, so every pair here (drive wheels, casters, floor, bottle, tray, bumps) takes
# Webots' defaults: Coulomb friction 1, softERP 0.2, softCFM 0.001. An ODE soft contact is a spring
# k = ERP / (h CFM) with damping c = (1 - ERP) / CFM, h the world's basicTimeStep, on each contact
# point (see TWO_POINT_LINKS): the robot sinks 3.6 mm at 8 ms and 14.5 mm at 32 ms, as recorded.
# Rigid contacts, as PyBullet's default, leave the split of the weight
# between the drive wheels and the casters undetermined: the solver put 80 % on the casters, and the
# drive wheels slipped (1 m/s^2 instead of the recorded 3.5). Bullet multiplies the friction of the
# two bodies in a contact, puts their stiffnesses in series and adds their dampings: each body
# carries friction 1, stiffness 2k and damping c/2, so that every pair gets Webots' contact.
WEBOTS_FRICTION = 1.0
WEBOTS_SOFT_ERP = 0.2
WEBOTS_SOFT_CFM = 0.001
WEBOTS_DEFAULT_TIME_STEP_S = 0.032
# The casters of the URDF are spheres welded to the base (the URDF says why): without friction they
# are an ideal caster, which carries load and opposes no motion, like the Webots BallJoint.
FRICTIONLESS_LINKS = ("caster_front", "caster_back")
# ODE gives a cylinder lying on a plane two contact points, one at each rim, and a sphere one. The
# Webots drive wheels are cylinders, so each stands on two springs in parallel (2k, 2c) and each caster
# on one: at rest the drive wheels carry twice the casters' load and the robot sinks W / 6k. The URDF
# wheels are spheres, one Bullet point each: their links get a stiffness so large that, in series with
# the other body's 2k, the pair keeps 2k, and the damping that, added to the other's c/2, makes 2c.
# docs_output/resultados/34_forma_de_la_rueda (second part).
TWO_POINT_LINKS = ("wheel_left", "wheel_right")
TWO_POINT_STIFFNESS = 1e9

# How concept_bottle decides the bottle is lost (agents/concept_bottle: Period.Compute,
# bottle_lost_threshold, tilt_threshold and the detection box of specificworker.h). A frame every
# 100 ms of the recording's clock; a frame is lost when the bottle is tilted more than 45 deg from the
# robot's vertical, or out of the box the camera point cloud is cut to (millimetres there, relative
# to the zed camera, robot axes: x right, y forward, z up). The effect is 5 lost frames in a row.
# The bottle centre stands in for its point cloud.
PERCEPTION_PERIOD_S = 0.1
PERCEPTION_LOST_FRAMES = 5
PERCEPTION_TILT_DEG = 45.0
PERCEPTION_BOX_M = {"x": (-0.2, 0.2), "y": (0.0, 0.35), "z": (-0.17, float("inf"))}
CAMERA_LINK = "zed"
IMU_LINK = "imu"

from src.logger import Logger
from src.verdict import (DEFAULT_EFFECT_Z_FRACTION, DISTANCE, EFFECT_DELTA_S, EPS, STOPPED_EARLY, TAU_MAX_S,
                         TAU_STEP_S, channels_from_json, discrepancy, physical_problem)
# Fast repetitions of a seeded episode scene (doFastSimulations; docs_output/resultados/41_mas_simulaciones_
# por_segundo): the same repetitions as doSimulations, without simulating what cannot change what the verdict
# compares. The loop stops CUT_AFTER_OBS_S after t_obs: the verdict compares up to t_obs + TAU_MAX_S, and an
# effect within EFFECT_DELTA_S of t_obs is still classed on time.
CUT_AFTER_OBS_S = EFFECT_DELTA_S
# A bump cannot be touching a link of the robot that is farther, in the plane, than its radius plus
# REACH_MARGIN_M, or higher than its height plus HEIGHT_MARGIN_M (the robot sinks 3.6 mm and pitches a little
# on its casters; it moves ~5 mm in a recorded step).
REACH_MARGIN_M = 0.05
HEIGHT_MARGIN_M = 0.01
# Lazy ABC (Prangle, 2016, without its continuation draw): a repetition stops once its distance is sure to
# end where its kernel weight is below LAZY_ETA times the nominal's, d > sqrt(d_nom^2 + 2 eps^2 ln(1/LAZY_ETA));
# each stopped repetition biases the evidence by at most LAZY_ETA of the nominal's weight. Checked every
# LAZY_EVERY recorded samples.
LAZY_ETA = 1e-6
LAZY_EVERY = 25

console = Console(highlight=False)

# Keys of the IMU history dictionary
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
ADV_SPEED = "adv_speed"
ROT_SPEED = "rot_speed"

# Keys of the IMU historical dictionary
HISTORY = "history"
GENERATED_INSTANCES = "generated_instances"
BOTTLE_POSITION = "bottle_position"
BOTTLE_ORIENTATION = "bottle_orientation"
FALL_TIME = "fall_time"
ROBOT_POSITION_AT_FALL = "robot_position_at_fall"
ROBOT_FINAL_POSITION = "robot_final_position"
SPAWN_OVERLAP = "spawn_overlap"
EFFECT_TIME = "effect_time"
PERCEPTION = "perception"
SETTLE_TIME = "settle_s"
POSITION = "position"

# Constants for cause loading logic
# Ajustamos para que busque desde la carpeta actual del script
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

CAUSES_PACKAGE_PATH = os.path.join(CURRENT_DIR, "causes", "implementations")
CAUSES_BASE_CLASS = Cause
CAUSES_PACKAGE_NAME = "causes.implementations"

# =============== IMPORT CAUSES LOGIC  ================
# ======================================================

def import_and_create_dynamic_union(package_path: str, base_class: Type, package_name: str) -> Type:
    """
    Scans a directory (package), imports all modules and returns
    a Union type of all found subclasses of `base_class`.
    
    :param package_path: Path of the directory to scan.
    :param base_class: The base class that plugins must inherit from.
    :param package_name: The importable package name (e.g. 'src.cause_gen.instance_generator').
    """
    found_subclasses = []

    # Iterate all modules in the given path
    for _, module_name, _ in pkgutil.iter_modules([package_path]):
        # Build the full module name to import
        full_module_name = f"{package_name}.{module_name}"
        
        try:
            module = importlib.import_module(full_module_name)
        except ImportError as e:
            print(f"Error importing {full_module_name}: {e}")
            continue

        # Inspect the module looking for classes
        for name, obj in inspect.getmembers(module, inspect.isclass):
            # Check that it is a subclass of base_class, not the base itself
            if issubclass(obj, base_class) and obj is not base_class:
                
                # Relajamos la condición del módulo. 
                # Solo verificamos que el módulo de la clase empiece con el nombre del paquete
                # o que sea exactamente el módulo que acabamos de importar.
                if obj.__module__.startswith(package_name) or obj.__module__ == module.__name__:
                    found_subclasses.append(obj)

    if not found_subclasses:
        raise ValueError(f"No subclasses of {base_class.__name__} found in {package_path}")

    # Create the Union dynamically: Union[Class1, Class2, ...]
    return Union[tuple(found_subclasses)]


def make_discriminated_union(union_type: type, discriminator: str = "name"):
    """
    Converts Union[Cls1, Cls2, ...] into a discriminated union.
    """
    return Annotated[union_type, Field(discriminator=discriminator)]


def create_cause_model(union_type: type) -> type[BaseModel]:
    """
    Creates dynamically:

    class CauseWrapper(BaseModel):
        cause: <discriminated union>
    """
    discriminated = make_discriminated_union(union_type, "name")

    CauseWrapper = create_model(
        "CauseWrapper",
        cause=(discriminated, ...)
    )

    return CauseWrapper

DynamicUnion = import_and_create_dynamic_union(
    package_path=CAUSES_PACKAGE_PATH,
    base_class=CAUSES_BASE_CLASS,
    package_name=CAUSES_PACKAGE_NAME
)

CauseWrapper = create_cause_model(DynamicUnion)


class CausesSimulator:

    def __init__(self, cause, simulation_scene, pipe, logger, real_time=False, gui=False):


        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================

        # Maps
        self.initialze_wheel_simplified_names_map()
        self.initialize_wheels_movement_map()
        self.initialize_bodies_list()
        self.logger = logger

        # Cause-related parameters
        self.initial_cause:Cause = CauseWrapper.model_validate_json(cause).cause
        self.current_cause:Cause = copy.deepcopy(self.initial_cause)

        # Working parameters
        self.pipe = int(pipe)
        self.realTime = real_time
        self.simulationTime = 0.0
        self.angularSpeed = 0.0
        self.historical = []
        self.spawn_overlap = False
        self.engine_wrapper = EnginePybullet(self)
        self.forwardSpeed = 0.0
        # Small Z offset in meters to place robot slightly above the ground
        self.robot_z_offset = 0.005

        # Engine parameters. Headless (DIRECT) by default so many instances can run in parallel.
        self.physicsClient = p.connect(p.GUI if gui else p.DIRECT)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-50, cameraTargetPosition=[0.8, -0.9, 2.0])

        # Simulation scene parameters
        self.simulation_scene = self.load_simulation_scene_json(simulation_scene)
        self.dt = self.simulation_scene.physics_dt or DEFAULT_PHYSICS_DT
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1, numSolverIterations=20)
        self.flags = p.URDF_USE_INERTIA_FROM_FILE
        self.apply_simulation_params()

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD ROBOT IN THE SIMULATION. The initial orientation matters: episodes may
        # start with the robot rotated, and the bottle pose is derived from the robot
        # frame, so ignoring it would place the bottle off the tray.
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", [self.initial_position[0], self.initial_position[1], self.initial_position[2]], self._as_quaternion(self.initial_orientation), flags=self.flags)

        # LOAD BOTTLE IN THE SIMULATION
        self.bottle = p.loadURDF("../../etc/URDFs/bottle/bottle.urdf", [self.bottle_position[0], self.bottle_position[1], self.bottle_position[2]], self._as_quaternion(self.bottle_orientation), flags=self.flags)

        self.bodies_by_name = {
            "robot": self.robot,
            "bottle": self.bottle,
            "floor": self.plane,
            "plane": self.plane,
        }

        # ================ ROBOT PARAMETERS  ===============
        # ==================================================

        self.wheels_radius = 0.1                 # m
        self.wheels_width = 0.05                 # m, width of each wheel
        self.distance_between_wheels = 0.518     # m, between wheel centres (the driver's axesLength)
        self.distance_from_center_to_wheels = self.distance_between_wheels / 2
        self.base_width = 0.44                   # m, width of the chassis box (boundingObject)

        self.motors = ["wheel_right_joint", "wheel_left_joint"]
        self.joints_name = self.get_joints_info(self.robot)
        self.links_name = self.get_link_info(self.robot)

        # ================ CONTACTS AND IMU  ================
        # ====================================================

        self.world_time_step = self.simulation_scene.world_time_step_s or WEBOTS_DEFAULT_TIME_STEP_S
        self.contact_stiffness = WEBOTS_SOFT_ERP / (self.world_time_step * WEBOTS_SOFT_CFM)
        self.contact_damping = (1.0 - WEBOTS_SOFT_ERP) / WEBOTS_SOFT_CFM
        for body in (self.robot, self.bottle, self.plane):
            self.apply_contact_model(body)

        # Where the Webots Shadow carries its Accelerometer and Gyro; each recorded sample is the
        # mean over one step of the recorded world.
        self.imu = IMU(self.robot, self.dt, link_index=self.links_name.get(IMU_LINK, -1),
                       window_s=self.world_time_step, gravity=self.simulation_scene.gravity)
        self.history_stride = max(1, int(round(self.world_time_step / self.dt)))
        self.history_steps = 0
        self.settle_time = 0.0

        if gui:
            time.sleep(0.5)


    # ================ LOGGING  ===============
    # ==================================================

    def init_imu_record(self) -> None:
        """Initialize the real IMU history dictionary for recording sensor data during the mission.
        """
        self.imu_history = {}
        self.imu_history[TIMESTAMP] = []
        self.imu_history[ACCELEROMETER] = []
        self.imu_history[GYROSCOPE] = []
        self.imu_history[POSITION] = []

        # When the bottle first drops below the effect threshold, and where the robot was then.
        self.fall_time = None
        self.robot_position_at_fall = None

        # What concept_bottle would see: one frame per PERCEPTION_PERIOD_S, and when it would
        # report the bottle lost.
        self.perception = {TIMESTAMP: [], "tilt_deg": [], "in_view": []}
        self.next_frame_time = self.start_time + PERCEPTION_PERIOD_S
        self.lost_frames = 0
        self.effect_time = None

        # The bodies were just restored: the accelerometer must not difference against the
        # end of the previous repetition.
        self.imu.reset()

    def record_imu(self) -> None:
        """Measure the IMU (every step: the accelerometer differences the last ones) and keep one sample
        per physics step of the recorded world, as the recording has, with the robot's planar position."""
        acc, ang = self.imu.get_measurement()
        self.history_steps += 1
        if self.history_steps % self.history_stride:
            return
        self.imu_history[TIMESTAMP].append(self.simulationTime)
        self.imu_history[ACCELEROMETER].append(acc.tolist())
        self.imu_history[GYROSCOPE].append(ang.tolist())
        position = p.getBasePositionAndOrientation(self.robot)[0]
        self.imu_history[POSITION].append([position[0], position[1]])

    def record_fall(self) -> None:
        """Register the first instant the bottle is below the effect threshold (the same
        z fraction the verdict uses on the final pose)."""
        if self.fall_time is not None:
            return
        bottle_z = p.getBasePositionAndOrientation(self.bottle)[0][2]
        if bottle_z < DEFAULT_EFFECT_Z_FRACTION * self.bottle_position[2]:
            self.fall_time = self.simulationTime
            robot_pos_m = p.getBasePositionAndOrientation(self.robot)[0]
            self.robot_position_at_fall = [coord * M_TO_MM for coord in robot_pos_m]

    def bottle_as_perceived(self) -> tuple[float, bool]:
        """(tilt from the robot's vertical in degrees, whether the bottle is inside the detection
        box), as concept_bottle measures them."""
        _, robot_orientation = p.getBasePositionAndOrientation(self.robot)
        bottle_position, bottle_orientation = p.getBasePositionAndOrientation(self.bottle)
        robot_rotation = np.array(p.getMatrixFromQuaternion(robot_orientation)).reshape(3, 3)
        bottle_axis = np.array(p.getMatrixFromQuaternion(bottle_orientation)).reshape(3, 3)[:, 2]
        cosine = abs(float(np.dot(bottle_axis, robot_rotation[:, 2])))
        tilt_deg = float(np.degrees(np.arccos(min(1.0, cosine))))
        camera = p.getLinkState(self.robot, self.links_name[CAMERA_LINK], computeForwardKinematics=1)[4]
        relative = robot_rotation.T @ (np.array(bottle_position) - np.array(camera))
        in_view = all(low <= value <= high for value, (low, high) in zip(relative, PERCEPTION_BOX_M.values()))
        return tilt_deg, in_view

    def record_perception(self) -> None:
        """A concept_bottle frame when one is due; the effect is the first frame that completes
        PERCEPTION_LOST_FRAMES lost frames in a row, as the recording deletes the robot->bottle edge."""
        if self.simulationTime + 1e-9 < self.next_frame_time:
            return
        self.next_frame_time += PERCEPTION_PERIOD_S
        tilt_deg, in_view = self.bottle_as_perceived()
        self.perception[TIMESTAMP].append(self.simulationTime)
        self.perception["tilt_deg"].append(round(tilt_deg, 2))
        self.perception["in_view"].append(in_view)
        lost = tilt_deg > PERCEPTION_TILT_DEG or not in_view
        self.lost_frames = self.lost_frames + 1 if lost else 0
        if self.effect_time is None and self.lost_frames >= PERCEPTION_LOST_FRAMES:
            self.effect_time = self.simulationTime

    def save_imu(self) -> None:
        """Save the current iteration IMU history in the historical list.
        """
        obj = {}
        obj[HISTORY] = self.imu_history
        obj[GENERATED_INSTANCES] = self.current_cause.get_generated_instances()
        bottle_pos_m, bottle_orn = p.getBasePositionAndOrientation(self.bottle)
        obj[BOTTLE_POSITION] = [coord * M_TO_MM for coord in bottle_pos_m]
        obj[BOTTLE_ORIENTATION] = bottle_orn
        obj[FALL_TIME] = self.fall_time
        obj[SPAWN_OVERLAP] = self.spawn_overlap
        obj[ROBOT_POSITION_AT_FALL] = self.robot_position_at_fall
        obj[ROBOT_FINAL_POSITION] = [coord * M_TO_MM for coord in p.getBasePositionAndOrientation(self.robot)[0]]
        obj[EFFECT_TIME] = self.effect_time
        obj[PERCEPTION] = self.perception
        obj[SETTLE_TIME] = self.settle_time
        self.historical.append(obj)

    def send_history_to_parent(self) -> None:
        """Send the recorded IMU history to the parent process through the pipe.
        """
        with os.fdopen(self.pipe, "w") as channel:
            json.dump(self.historical, channel)




    # ================ SIMULATION  ===============
    # ==================================================
    @staticmethod
    def _as_quaternion(value) -> list:
        """[x, y, z, w] quaternion; degenerate values (all-zeros placeholder in
        old scene files) fall back to the identity."""
        try:
            quaternion = [float(component) for component in value]
        except (TypeError, ValueError):
            quaternion = []
        if len(quaternion) != 4 or sum(component * component for component in quaternion) < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return quaternion

    def load_simulation_scene_json(self, simulation_scene_file: str) -> SimulationScene:
        """Load the simulation scene parameters from a JSON file.
            Parameters:
                - simulation_scene_file (str): Path to the JSON file.
            Returns:
                - SimulationScene: The loaded simulation scene configuration.
        """
        return SimulationScene.model_validate_json(open(simulation_scene_file, 'r').read())

    def apply_simulation_params(self) -> None:
        """Apply the currently stored simulation scene parameters to the real scene.

        The scene stores positions in meters, as PyBullet uses them.
        """
        p.setGravity(0, 0, self.simulation_scene.gravity)
        # Apply small Z offset so robot appears slightly above the ground
        robot_init_pos = list(self.simulation_scene.initial_robot_position)
        # Ensure list has at least 3 elements
        while len(robot_init_pos) < 3:
            robot_init_pos.append(0.0)
        robot_init_pos[2] = robot_init_pos[2] + getattr(self, 'robot_z_offset', 0.005)
        self.initial_position = robot_init_pos
        self.initial_orientation = self.simulation_scene.initial_robot_orientation
        self.bottle_position = self.simulation_scene.bottle_position
        self.bottle_orientation = self.simulation_scene.bottle_orientation
        self.problem_position = self.simulation_scene.problem_position
        self.problem_orientation = self.simulation_scene.problem_orientation
        self.simulation_length = self.simulation_scene.simulation_length
        self.clock_rate = self.simulation_scene.clock_rate if self.simulation_scene.clock_rate > 0 else 1.0
        # If cause uses grid-based generation, derive repetitions from grid dimensions
        if hasattr(self.current_cause, 'grid_dimensions'):
            grid_dims = getattr(self.current_cause, 'grid_dimensions', [10, 10])
            self.num_of_repetitions = grid_dims[0] * grid_dims[1]
        else:
            self.num_of_repetitions = getattr(self.current_cause, 'num_of_repetitions', None) or self.simulation_scene.num_of_repetitions
        self.list_of_target_velocities = self.simulation_scene.list_of_target_velocities
        self.list_of_target_rot_speeds = self.simulation_scene.list_of_target_rot_speeds
        # The simulation may start mid-recording, from the recorded state (episode_scene, seeded scene).
        self.start_time = self.simulation_scene.start_time_s
        self.initial_speed = self.simulation_scene.initial_speed

    def simulate(self) -> None:
        """Simulate a step of the current iteration.
        """
        stepCount = 0
        self.simulationTime = self.start_time
        self.history_steps = 0
        # Simulate
        while (self.simulationTime) < float(self.simulation_length):
            
            it = time.time()
            self.do_acceleration_robot(self.simulationTime)
            self.current_cause.apply_compute(self.engine_wrapper)
            p.stepSimulation()
            stepCount += 1
            # A physics step lasts dt on the clock the recorded physics ran on (Webots
            # runs slower than the wall clock of the recording). The setpoints, the
            # horizon, the causes' windows and everything recorded here stay on the
            # recording's clock. What is recorded is the state after the step, stamped
            # with the instant the step ends.
            self.simulationTime = self.start_time + stepCount * self.dt / self.clock_rate
            self.record_imu()
            self.record_fall()
            self.record_perception()
            et = time.time()
            
            if self.realTime:
                time.sleep(max(0, self.dt - (et - it))) # Sleep to maintain real-time simulation
        return
    
    def doSimulations(self) -> None:
        """Simulate multiple iterations and collect all IMU recordings, sending them to the parent process.
        """
        self.settle_scene()
        self.set_initial_speed()
        self.intialState = p.saveState() # Save clean state: every repetition starts from the settled scene
        for i in range(self.num_of_repetitions):
            self.current_repetition = i
            self.current_cause.apply(self.engine_wrapper)
            for body in self.loaded_bodies:
                self.apply_contact_model(body)
            self.spawn_overlap = self.spawned_bodies_overlap()
            self.init_imu_record()
            self.simulate()
            self.save_imu()
            self.clean_bodies()
            self.reset_wheels_movement()
            self.current_cause = copy.deepcopy(self.initial_cause)
            p.restoreState(stateId=self.intialState) # Restore the clean state
        return
    
    # ================ FAST REPETITIONS  ================
    # ===================================================
    # docs_output/resultados/41_mas_simulaciones_por_segundo: same results as doSimulations (to the
    # repetition-order noise doSimulations already has: Bullet keeps state across restoreState), about
    # 8 times faster with thousands of repetitions:
    #   * the nominal is simulated once, saving the state at every recorded step; a repetition starts
    #     from the last state saved before its cause can act (a push: its onset; a bump: the first
    #     instant a link of the robot could touch it) and copies the nominal before;
    #   * a bump the robot cannot touch before the cut is the nominal; one spawned over the robot is
    #     invalid (verdict.physical_problem) without simulating it;
    #   * the loop stops at t_obs + CUT_AFTER_OBS_S; the IMU is read only on the recorded steps and the
    #     setpoints are sent only when they change;
    #   * each repetition is scored here with the verdict's discrepancy, and stopped early once it is sure
    #     to end beyond the lazy cutoff; only the nominal and the best repetition keep their history.

    def doFastSimulations(self, channels: list, first: int = 0, count: int | None = None) -> None:
        """Repetitions first .. first + count - 1 of the cause, scored against the real `channels` (the
        verdict's, verdict.scene_channels). The scene must carry observed_effect_time."""
        self.settle_scene()
        self.set_initial_speed()
        self.intialState = p.saveState()
        t_obs = float(self.simulation_scene.observed_effect_time)
        self.cut_time = min(float(self.simulation_length), t_obs + CUT_AFTER_OBS_S)
        self.robot_parts = self._robot_parts()
        self.lazy_cutoff = None
        self.snapshots = []
        self._start_fast_run()
        self._run_fast(cause=False, snapshots=self.snapshots)
        self.nominal_history = copy.deepcopy(self.imu_history)
        self.nominal_perception = copy.deepcopy(self.perception)
        self.nominal_repetition = self._fast_record({})
        nominal_distance = discrepancy(channels, self.nominal_history)
        if self.initial_cause.name == "none":
            self.historical.append({**self.nominal_repetition, DISTANCE: nominal_distance})
            return
        self._prepare_lower_bound(channels)
        if math.isfinite(nominal_distance):
            self.lazy_cutoff = math.sqrt(nominal_distance ** 2 + 2.0 * EPS ** 2 * math.log(1.0 / LAZY_ETA))
        last = self.num_of_repetitions if count is None else min(self.num_of_repetitions, first + count)
        best, best_distance = None, math.inf
        control = getattr(self, "draw_control", None)
        for i in range(first, last):
            if control is not None:
                control.start(i - first)
            rep, kind = self._fast_repetition(i)
            if control is not None:
                rep[DRAWS], rep[DRAW_WIDTHS] = list(control.units), list(control.widths)
            if kind == "stopped":
                rep[DISTANCE] = None
            elif physical_problem(rep) is not None:
                rep[DISTANCE] = None
            else:
                rep[DISTANCE] = nominal_distance if kind == "nominal" else discrepancy(channels, rep[HISTORY])
            if rep[DISTANCE] is not None and rep[DISTANCE] < best_distance:
                if best is not None:
                    self.historical[best].pop(HISTORY, None)
                    self.historical[best].pop(PERCEPTION, None)
                best, best_distance = len(self.historical), rep[DISTANCE]
            else:
                rep.pop(HISTORY, None)
                rep.pop(PERCEPTION, None)
            self.historical.append(rep)

    def _robot_parts(self) -> list:
        """Every robot link that can collide, as (x0, x1, y0, y1, z0): its outline in the plane, in the
        robot frame (x right, y forward), and its lowest point above the floor, from its bounding box at
        the settled pose. A bounding box is larger than the link, so both err on the safe side."""
        base, orientation = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(orientation)[2]
        cos, sin = math.cos(yaw), math.sin(yaw)
        parts = []
        for link in range(-1, p.getNumJoints(self.robot)):
            if not p.getCollisionShapeData(self.robot, link):
                continue
            low, high = p.getAABB(self.robot, link)
            us, vs = [], []
            for x in (low[0], high[0]):
                for y in (low[1], high[1]):
                    dx, dy = x - base[0], y - base[1]
                    us.append(cos * dx + sin * dy)
                    vs.append(-sin * dx + cos * dy)
            parts.append((min(us), max(us), min(vs), max(vs), low[2]))
        return parts

    def _can_touch(self, sample: dict, snapshot: dict) -> bool:
        """Whether a dome could be touching some link of the robot at a snapshot."""
        dx, dy = sample["x_m"] - snapshot["xy"][0], sample["y_m"] - snapshot["xy"][1]
        cos, sin = math.cos(snapshot["yaw"]), math.sin(snapshot["yaw"])
        u, v = cos * dx + sin * dy, -sin * dx + cos * dy
        reach = sample["diameter_m"] / 2.0 + REACH_MARGIN_M
        top = sample["height_m"] + HEIGHT_MARGIN_M
        return any(z0 < top and math.hypot(max(x0 - u, 0.0, u - x1), max(y0 - v, 0.0, v - y1)) < reach
                   for x0, x1, y0, y1, z0 in self.robot_parts)

    def _start_fast_run(self) -> None:
        self.init_imu_record()
        self.stepCount = 0
        self.simulationTime = self.start_time
        self.history_steps = 0
        self._last_command = None
        self._velocity_before = self.imu._state()[1]

    def _run_fast(self, cause: bool = True, snapshots: list | None = None) -> bool:
        """simulate()'s loop up to the cut, lean. False if the lazy cutoff stopped it."""
        lean_imu = self.imu.window_steps == self.history_stride
        while self.simulationTime < self.cut_time:
            if snapshots is not None and self.stepCount % self.history_stride == 0:
                snapshots.append(self._snapshot())
            command = (self.command_time(self.simulationTime), tuple(self.wheel_movement[m] for m in self.motors))
            if command != self._last_command:
                self._last_command = command
                self.do_acceleration_robot(self.simulationTime)
            if cause:
                self.current_cause.apply_compute(self.engine_wrapper)
            p.stepSimulation()
            self.stepCount += 1
            self.simulationTime = self.start_time + self.stepCount * self.dt / self.clock_rate
            if lean_imu:
                self._record_imu_lean()
            else:
                self.record_imu()
            self.record_fall()
            self.record_perception()
            if (cause and self.lazy_cutoff is not None and self.history_steps % (self.history_stride * LAZY_EVERY) == 0
                    and self._lower_bound() > self.lazy_cutoff):
                return False
        return True

    def _record_imu_lean(self) -> None:
        """record_imu, reading the IMU only on the recorded steps: the accelerometer differences the
        velocities one recorded step apart, its window, so the numbers are the same."""
        self.history_steps += 1
        if self.history_steps % self.history_stride:
            return
        rotation, linear, angular = self.imu._state()
        acceleration = (linear - self._velocity_before) / (self.imu.window_steps * self.imu.dt)
        self._velocity_before = linear
        world_to_device = rotation.T
        self.imu_history[TIMESTAMP].append(self.simulationTime)
        self.imu_history[ACCELEROMETER].append((world_to_device @ (acceleration - self.imu.gravity_world)).tolist())
        self.imu_history[GYROSCOPE].append((world_to_device @ angular).tolist())
        position = p.getBasePositionAndOrientation(self.robot)[0]
        self.imu_history[POSITION].append([position[0], position[1]])

    def _snapshot(self) -> dict:
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        return {"state": p.saveState(), "step": self.stepCount, "time": self.simulationTime,
                "history_steps": self.history_steps, "velocity_before": self._velocity_before.copy(),
                "n_history": len(self.imu_history[TIMESTAMP]), "n_perception": len(self.perception[TIMESTAMP]),
                "next_frame_time": self.next_frame_time, "lost_frames": self.lost_frames,
                "effect_time": self.effect_time, "fall_time": self.fall_time,
                "robot_position_at_fall": copy.deepcopy(self.robot_position_at_fall),
                "xy": (position[0], position[1]), "yaw": p.getEulerFromQuaternion(orientation)[2]}

    def _restore(self, snapshot: dict) -> None:
        p.restoreState(stateId=snapshot["state"])
        self.stepCount = snapshot["step"]
        self.simulationTime = snapshot["time"]
        self.history_steps = snapshot["history_steps"]
        self._velocity_before = snapshot["velocity_before"].copy()
        self._last_command = None
        self.imu_history = {key: list(values[:snapshot["n_history"]]) for key, values in self.nominal_history.items()}
        self.perception = {key: list(values[:snapshot["n_perception"]]) for key, values in self.nominal_perception.items()}
        self.next_frame_time = snapshot["next_frame_time"]
        self.lost_frames = snapshot["lost_frames"]
        self.effect_time = snapshot["effect_time"]
        self.fall_time = snapshot["fall_time"]
        self.robot_position_at_fall = copy.deepcopy(snapshot["robot_position_at_fall"])

    def _fast_record(self, instances: dict) -> dict:
        """What save_imu keeps of a repetition, with the given generated instances."""
        bottle_position, bottle_orientation = p.getBasePositionAndOrientation(self.bottle)
        return {HISTORY: self.imu_history, GENERATED_INSTANCES: instances,
                BOTTLE_POSITION: [c * M_TO_MM for c in bottle_position], BOTTLE_ORIENTATION: bottle_orientation,
                FALL_TIME: self.fall_time, SPAWN_OVERLAP: self.spawn_overlap,
                ROBOT_POSITION_AT_FALL: self.robot_position_at_fall,
                ROBOT_FINAL_POSITION: [c * M_TO_MM for c in p.getBasePositionAndOrientation(self.robot)[0]],
                EFFECT_TIME: self.effect_time, PERCEPTION: self.perception, SETTLE_TIME: self.settle_time}

    def _first_snapshot(self) -> int | None:
        """Index of the snapshot this repetition starts from; None when its cause cannot act before the
        cut (the repetition is the nominal). Causes other than a bump or a push start from the seed."""
        cause = self.current_cause
        if cause.name == "none":
            return None
        if cause.name == "bump_scaled":
            sample = cause.samples[self.current_repetition % len(cause.samples)]
            for index, snapshot in enumerate(self.snapshots):
                if self._can_touch(sample, snapshot):
                    return max(0, index - 1)
            return None
        if cause.name == "external_force":
            # the draws in the order apply_compute makes them, and its onset
            cause._draw_timing()
            length = self.get_simulation_length()
            start, end = cause.force_window_start * length, cause.force_window_end * length
            onset = start + cause._onset_fraction * (max(start, end - cause._duration) - start)
            if onset >= self.cut_time:
                return None
            return max([0] + [i for i, snapshot in enumerate(self.snapshots) if snapshot["time"] <= onset])
        return 0

    def _fast_repetition(self, i: int) -> tuple[dict, str]:
        """Repetition i and how it was obtained: 'nominal' (copied), 'overlap' (not simulated),
        'stopped' (lazy), 'start' or 'branch'."""
        self.current_repetition = i
        self.current_cause = copy.deepcopy(self.initial_cause)
        index = self._first_snapshot()
        if index is None:
            rep = dict(self.nominal_repetition)
            if self.current_cause.name == "bump_scaled":
                rep[GENERATED_INSTANCES] = {"sample": self.current_cause.samples[i % len(self.current_cause.samples)]}
            else:
                rep[GENERATED_INSTANCES] = self.current_cause.get_generated_instances()
            return rep, "nominal"
        self._restore(self.snapshots[index])
        self.current_cause.apply(self.engine_wrapper)
        for body in self.loaded_bodies:
            self.apply_contact_model(body)
        # only from the seed can a placed body overlap the robot: beyond its reach it cannot
        self.spawn_overlap = self.spawned_bodies_overlap() if index == 0 else False
        if self.spawn_overlap:
            rep = {**self.nominal_repetition, SPAWN_OVERLAP: True,
                   GENERATED_INSTANCES: self.current_cause.get_generated_instances()}
            kind = "overlap"
        else:
            finished = self._run_fast(cause=True)
            rep = self._fast_record(self.current_cause.get_generated_instances())
            rep[STOPPED_EARLY] = not finished
            kind = "stopped" if not finished else ("start" if index == 0 else "branch")
        self.spawn_overlap = False
        self.clean_bodies()
        self.reset_wheels_movement()
        return rep, kind

    def _prepare_lower_bound(self, channels: list) -> None:
        """The discrepancy pairs each real instant, at each shift, with the last simulated sample at or
        before it. Every repetition samples the instants the nominal does, so the pairings are fixed."""
        sim_times = np.asarray(self.nominal_history[TIMESTAMP], dtype=float)
        shifts = np.arange(-TAU_MAX_S, TAU_MAX_S + 1e-9, TAU_STEP_S)
        self._bound = []
        for c in channels:
            times = np.asarray(c["times"], dtype=float)
            index = np.clip(np.searchsorted(sim_times, times[None, :] + shifts[:, None], side="right") - 1,
                            0, sim_times.size - 1)
            values = np.asarray(c["values"], dtype=float).reshape(times.size, -1)
            self._bound.append({"key": c["key"], "index": index, "values": values, "scale": c["scale"]})

    def _lower_bound(self) -> float:
        """A lower bound of the repetition's final distance from what is simulated so far: the pairings
        not reached yet count as no error. Once everything is simulated, it is the distance."""
        available = len(self.imu_history[TIMESTAMP])
        total = 0.0
        for c in self._bound:
            simulated = np.asarray(self.imu_history[c["key"]], dtype=float)
            known = c["index"] < available
            error = simulated[np.where(known, c["index"], 0)] - c["values"][None, :, :]
            squared = np.where(known, np.sum(error ** 2, axis=2) / c["values"].shape[1], 0.0)
            total = total + np.sqrt(squared.sum(axis=1) / c["values"].shape[0]) / c["scale"]
        return float(np.min(total))

    def apply_contact_model(self, body: int) -> None:
        """Webots' default contact on every link of `body` (see WEBOTS_FRICTION); the robot's
        casters without friction and its drive wheels on two contact points (TWO_POINT_LINKS)."""
        for link in range(-1, p.getNumJoints(body)):
            name = p.getJointInfo(body, link)[12].decode() if link >= 0 else ""
            two_point = body == getattr(self, "robot", None) and name in TWO_POINT_LINKS
            p.changeDynamics(body, link, lateralFriction=0.0 if name in FRICTIONLESS_LINKS else WEBOTS_FRICTION,
                             contactStiffness=TWO_POINT_STIFFNESS if two_point else 2.0 * self.contact_stiffness,
                             contactDamping=(1.5 if two_point else 0.5) * self.contact_damping)

    def settle_scene(self) -> None:
        """Let the robot and the bottle come to rest with the wheels held, before the episode
        starts: they are spawned a few millimetres above their supports, and starting to drive
        while they still drop jolts the bottle."""
        for motor_name in self.motors:
            p.setJointMotorControl2(self.robot, self.joints_name[motor_name], p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=WHEEL_MAX_FORCE)
        steps = 0
        while steps * self.dt < SETTLE_MAX_S:
            p.stepSimulation()
            steps += 1
            if steps * self.dt >= SETTLE_MIN_S and self._at_rest():
                break
        self.settle_time = steps * self.dt
        self.logger.log(f"Scene settled in {self.settle_time:.2f} s of physics.")

    def set_initial_speed(self) -> None:
        """A seeded scene starts with the robot moving as recorded: the base and the bottle at the
        recorded forward speed along the robot's heading, the drive wheels rolling with it."""
        if self.initial_speed <= 0.0:
            return
        _, orientation = p.getBasePositionAndOrientation(self.robot)
        forward = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)[:, 1]
        velocity = (self.initial_speed * forward).tolist()
        for body in (self.robot, self.bottle):
            p.resetBaseVelocity(body, linearVelocity=velocity, angularVelocity=[0.0, 0.0, 0.0])
        for motor_name in self.motors:
            joint = self.joints_name[motor_name]
            p.resetJointState(self.robot, joint, p.getJointState(self.robot, joint)[0],
                              targetVelocity=self.initial_speed / self.wheels_radius)

    def _at_rest(self) -> bool:
        for body in (self.robot, self.bottle):
            linear, angular = p.getBaseVelocity(body)
            if np.linalg.norm(linear) > SETTLE_LINEAR_SPEED or np.linalg.norm(angular) > SETTLE_ANGULAR_SPEED:
                return False
        return True

    def spawned_bodies_overlap(self) -> bool:
        """Whether a body placed by the cause interpenetrates the robot or the bottle.
        The recording shows the robot standing there, so such a placement cannot be
        the real scene; the solver then pushes the bodies apart and the jolt alone
        can knock the bottle off."""
        for body_id in self.loaded_bodies:
            for target in (self.robot, self.bottle):
                if p.getClosestPoints(bodyA=target, bodyB=body_id, distance=0.0):
                    return True
        return False

    def clean_bodies(self) -> None:
        """Remove all bodies from the simulation except the plane and the robot.
        """
        count = 0
        for body_id in self.loaded_bodies:
            p.removeBody(body_id)
            count += 1
        self.loaded_bodies = []

    def reset_wheels_movement(self) -> None:
        """Set wheel movement to true for every wheel.
        """
        for wheel in self.wheel_movement:
            self.wheel_movement[wheel] = True
 
    # ================= ENGINE DATA ====================
    # ==================================================

    def initialize_wheels_movement_map(self) -> None:
        """Initialize the wheel movement map to track which wheels are currently moving.
        """
        self.wheel_movement = {}
        self.wheel_movement["wheel_right_joint"] = True
        self.wheel_movement["wheel_left_joint"] = True
    
    def initialze_wheel_simplified_names_map(self) -> None:
        """Initialize the mapping of wheel simplified names to full joint names.
        """
        self.wheel_names = {}
        self.wheel_names["L"] = "wheel_left_joint"
        self.wheel_names["R"] = "wheel_right_joint"
        
    def initialize_bodies_list(self) -> None:
        """Initialize the list to track all loaded bodies in the simulation.
        """
        self.loaded_bodies = []
    

 
    # ================= ENGINE "API" ===================
    # ==================================================

    def get_pybullet_instance(self):
        """ ENGINE: Get engine """
        return p
    
    def instantiate_body(self, body_file:str, body_position:tuple, identifier:str=str(uuid4())):
        """ ENGINE: Spawn body """
        body_position_m = body_position
        self.loaded_bodies.append(p.loadURDF(body_file, basePosition=body_position_m))

    def set_robot_wheel_moving(self, simplified_wheel_name:str, moving:bool):
        """ ENGINE: Set robot wheel moving ("L" or "R", the two drive wheels) """
        self.wheel_movement[self.wheel_names[simplified_wheel_name]] = moving
    
    def set_gravity(self, gravity:float=-9.81):
        """ ENGINE: Set gravity """
        p.setGravity(0, 0, gravity)

    def get_body_id_by_name(self, target_name:str) -> int:
        """ ENGINE: Resolve a semantic body name to its PyBullet body id """
        if target_name not in self.bodies_by_name:
            raise ValueError(f"Unknown simulation body '{target_name}'. Known bodies: {sorted(self.bodies_by_name)}")
        return self.bodies_by_name[target_name]

    def apply_external_force(self, target_name:str, force_vector:list, height:float=None):
        """ ENGINE: Apply a world-frame force (N) to a body for the next step: at its centre of mass,
        or, with `height`, at that height (m) above the body's base origin along its own vertical
        (the robot's base origin is on the floor). PyBullet clears external forces after each
        stepSimulation, so causes must re-apply it every step while the force lasts. """
        body_id = self.get_body_id_by_name(target_name)
        position, orientation = p.getBasePositionAndOrientation(body_id)
        if height is not None:
            up = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)[:, 2]
            position = (np.array(position) + height * up).tolist()
        p.applyExternalForce(body_id, -1, force_vector, position, p.WORLD_FRAME)

    def set_lateral_friction(self, target_name:str, value:float):
        """ ENGINE: Set the lateral friction coefficient of a body's base link """
        body_id = self.get_body_id_by_name(target_name)
        p.changeDynamics(body_id, -1, lateralFriction=value)

    def get_simulation_time(self):
        """ ENGINE: Get sim time """
        return self.simulationTime

    def get_simulation_length(self):
        """ENGINE: Get sim whole length"""
        return self.simulation_length

    # =============== ROBOT KINEMATICS  ================
    # ==================================================

    def do_acceleration_robot(self, simulationTime: float) -> None:
        """Apply velocity control to the robot wheels based on target velocity at given simulation time.
            Parameters:
                - simulationTime (float): Current simulation time in seconds.
        """
        command_time = self.command_time(simulationTime)
        self.angularSpeed = self.get_target_rot_speed(command_time)
        wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.get_target_velocity(command_time), self.angularSpeed)
        for motor_name in self.motors:
            if not self.wheel_movement[motor_name]: wheels_velocity[motor_name] = 0
        for motor_name in self.motors:
            p.setJointMotorControl2(bodyUniqueId=self.robot,
                                    jointIndex=self.joints_name[motor_name],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=wheels_velocity[motor_name],
                                    force=WHEEL_MAX_FORCE
            )

    def command_time(self, simulationTime: float) -> float:
        """The instant whose setpoint the base follows at `simulationTime`: the start of the current
        physics step of the recorded world. webots-bridge sets the wheel speeds once per step, with
        the last command received, so a setpoint that lasts less than a step (the 17 ms dip of the
        follow controller's derivative kick, at 32 ms) only acts if it is the last one of its step.
        The steps are counted from the first IMU sample, which Webots wrote at the end of one."""
        steps = math.floor(simulationTime * self.clock_rate / self.world_time_step + 1e-9)
        return steps * self.world_time_step / self.clock_rate

    def get_target_velocity(self, simulationTime: float) -> float:
        """Get the target forward velocity of the robot at a specific simulation timestamp.
            Parameters:
                - simulationTime (float): The simulation time in seconds.
            Returns:
                - float: Forward velocity value: the last setpoint at or before that time (0 before the first one).
        """
        index = int(np.searchsorted(self.list_of_target_velocities[TIMESTAMP], simulationTime, side="right")) - 1
        if index < 0:
            return 0
        return self.list_of_target_velocities[ADV_SPEED][index]

    def get_target_rot_speed(self, simulationTime: float) -> float:
        """Target angular velocity (rad/s) at a simulation time: the last sample at or
        before it, 0 when the scene carries no rotation profile.
        """
        timestamps = self.list_of_target_rot_speeds.get(TIMESTAMP, []) if self.list_of_target_rot_speeds else []
        index = int(np.searchsorted(timestamps, simulationTime, side="right")) - 1
        if index < 0:
            return 0.0
        return float(self.list_of_target_rot_speeds[ROT_SPEED][index])

    def get_forward_velocity(self) -> float:
        """Forward velocity of the robot (m/s) from its two drive wheels."""
        w_left = p.getJointState(self.robot, self.joints_name["wheel_left_joint"])[1]
        w_right = p.getJointState(self.robot, self.joints_name["wheel_right_joint"])[1]
        return (w_left + w_right) * self.wheels_radius / 2

    def get_angular_velocity(self) -> float:
        """Angular velocity of the robot (rad/s, positive counterclockwise) from its two drive wheels."""
        w_left = p.getJointState(self.robot, self.joints_name["wheel_left_joint"])[1]
        w_right = p.getJointState(self.robot, self.joints_name["wheel_right_joint"])[1]
        return (w_right - w_left) * self.wheels_radius / self.distance_between_wheels

    def get_wheels_velocity_from_forward_velocity_and_angular_velocity(self, forward_velocity: float = 0, angular_velocity: float = 0) -> dict:
        """Velocity (rad/s) of each drive wheel for a forward (m/s) and angular (rad/s) velocity."""
        return {
            "wheel_left_joint": (forward_velocity - self.distance_from_center_to_wheels * angular_velocity) / self.wheels_radius,
            "wheel_right_joint": (forward_velocity + self.distance_from_center_to_wheels * angular_velocity) / self.wheels_radius}

    # =============== PYBULLET MODELS INFO  ================
    # ======================================================


    def get_joints_info(self, robot_id: int) -> dict:
        """Get joint names and IDs from a robot model, initializing revolute joints with zero velocity.
            Parameters:
                - robot_id (int): ID of the robot model in the simulation.
            Returns:
                - dict: Dictionary mapping joint names to their joint IDs.
        """
        joint_name_to_id = {}
        # Get number of joints in the model
        num_joints = p.getNumJoints(robot_id)
        # self.logger.log("Num joints:", num_joints)

        # Populate the dictionary with joint nad
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            joint_name_to_id[joint_name] = i
            jid = joint_info[0]
            jtype = joint_info[2]
            if jtype == p.JOINT_REVOLUTE:
                p.setJointMotorControl2(bodyUniqueId=robot_id,
                                        jointIndex=jid,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=0,
                                        force=0)
        return joint_name_to_id

    def get_link_info(self, robot_id: int) -> dict:
        """Get link names and IDs from a robot model.
            Parameters:
                - robot_id (int): ID of the robot model in the simulation.
            Returns:
                - dict: Dictionary mapping link names to their link IDs.
        """
        link_name_to_id = {}
        # Get number of joints in the model
        num_links = p.getNumJoints(robot_id)
        # self.logger.log("Num links:", num_links)

        # Populate the dictionary with link names and IDs
        for i in range(num_links):
            link_info = p.getJointInfo(robot_id, i)
            link_name = link_info[12].decode("utf-8")
            link_name_to_id[link_name] = i
        return link_name_to_id

    def save_raw_data(self, output_dir: str = "logs/imu_raw") -> None:
        """Save raw IMU historical data to a JSON file before sending to parent process.
        """
        os.makedirs(output_dir, exist_ok=True)
        cause_name = self.initial_cause.name  # ya existe en todas las subclases de Cause
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(output_dir, f"{cause_name}_{timestamp}.json")
        
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.historical, f)
        
        self.logger.log(f"Raw data saved to '{filepath}'", style="bold cyan")


def main():
    # Init logger
    os.path.exists("logs/causes_simulator") or os.makedirs("logs/causes_simulator")
    logger = Logger(f"logs/causes_simulator/causes_simulator_{time.strftime('%Y%m%d_%H%M%S')}.log")
    logger.log("Starting causes simulator...")
    parser = argparse.ArgumentParser(
        prog = "Causes simulator",
        description = "Simulates multiple iterations of a scenary and a cause given its data.",
        epilog = "RoboLab - 2026"
    )
    
    parser.add_argument('-c', '--cause', required=True, help="JSON string defining the cause values")
    parser.add_argument('-s', '--simulation_scene', required=True, help="Path to the JSON file defining the simulation scene parameters")
    parser.add_argument('-p', '--pipe', required=True, help="Name of the pipe with which inner simulator will comunicate with the cause simulator instance")
    parser.add_argument('-rt', '--real_time', required=False, help="Run simulations at real-time speed. Not recommended outside of debugging.", action='store_true')
    parser.add_argument('-g', '--gui', required=False, help="Open the PyBullet GUI. Default is headless (DIRECT), required to run many instances in parallel.", action='store_true')
    parser.add_argument('--channels', required=False, help="JSON file with the verdict's real channels (verdict.channels_to_json): run the fast repetitions, scored here (doFastSimulations)")
    parser.add_argument('--first', type=int, default=0, help="With --channels: first repetition to run")
    parser.add_argument('--count', type=int, default=None, help="With --channels: how many repetitions to run (all by default)")
    parser.add_argument('--draws', required=False, help="With --channels: JSON file with the unit draws of each repetition run, in order (draws.DrawControl); without it they are drawn here and returned")
    parser.add_argument('--seed', type=int, default=None, help="With --channels: seed of the draws made here")
    args = parser.parse_args()

    try:
        logger.log("Arguments parsed successfully.")
        simulator = CausesSimulator(args.cause, args.simulation_scene, args.pipe, logger, args.real_time, args.gui)
        
        logger.log("Starting simulations...")
        if args.channels:
            with open(args.channels) as f:
                channels = channels_from_json(json.load(f))
            if simulator.initial_cause.name not in ("none", "bump_scaled"):
                # the generated causes draw with the random module: give or record their draws (a bump takes
                # its samples in the cause itself)
                given = None
                if args.draws:
                    with open(args.draws) as f:
                        given = json.load(f)
                simulator.draw_control = DrawControl(given, args.seed)
                simulator.draw_control.install()
            simulator.doFastSimulations(channels, args.first, args.count)
            logger.log(f"Fast simulations done: repetitions {args.first}.. ({len(simulator.historical)}), "
                       f"{sum(bool(h.get(STOPPED_EARLY)) for h in simulator.historical)} stopped early.")
        else:
            simulator.doSimulations()
            logger.log(f"""Simulations done.
        Historical data:
            Number of historicals: {len(simulator.historical)}
            Combined number of recordings: {sum(len(h[HISTORY][TIMESTAMP]) for h in simulator.historical)}""")
            simulator.save_raw_data()

        logger.log("Sending historicals to parent process...")
        simulator.send_history_to_parent()
        
        logger.log("Sent. Exiting...")
    except Exception as e:
        logger.log(f"Fatal exception occurred: {e}", style="bold red")
        logger.log(traceback.format_exc(), style="red")
        sys.exit(1)


if __name__ == "__main__":
    main()

