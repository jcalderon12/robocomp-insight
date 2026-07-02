#!/usr/bin/env python3

import ast
import pybullet as p
import json
import argparse
import time
import sys
import numpy as np
import os
import json
from pybullet_imu import IMU
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

MM_TO_M = 0.001
M_TO_MM = 1000

from src.logger import Logger

console = Console(highlight=False)

# Keys of the IMU history dictionary
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
ADV_SPEED = "adv_speed"

# Keys of the IMU historical dictionary
HISTORY = "history"
GENERATED_INSTANCES = "generated_instances"
BOTTLE_POSITION = "bottle_position"
BOTTLE_ORIENTATION = "bottle_orientation"

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

    def __init__(self, cause, simulation_scene, pipe, logger, real_time=False):
        

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
        self.engine_wrapper = EnginePybullet(self)
        self.forwardSpeed = 0.0
        # Small Z offset in millimeters to place robot slightly above the ground
        # Default 0.04 mm as requested (can be adjusted later)
        self.robot_z_offset_mm = 0.04

        # Engine parameters
        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-15, cameraTargetPosition=[0.8, -0.9, 0.2])
        self.dt = 1./120. 
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1, numSolverIterations=20)
        self.flags = p.URDF_USE_INERTIA_FROM_FILE

        # Simulation scene parameters
        self.simulation_scene = self.load_simulation_scene_json(simulation_scene)
        self.apply_simulation_params()

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD ROBOT IN THE SIMULATION
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", [self.initial_position[0], self.initial_position[1], self.initial_position[2]], flags=self.flags)        
        
        # LOAD BOTTLE IN THE SIMULATION
        self.bottle = p.loadURDF("../../etc/URDFs/bottle/bottle.urdf", [self.bottle_position[0], self.bottle_position[1], self.bottle_position[2]], flags=self.flags)
        
        # ================ IMU SENSOR SETUP  ================
        # ====================================================

        self.imu = IMU(self.robot, self.dt)

        # ================ ROBOT PARAMETERS  ===============
        # ==================================================

        self.wheels_radius = 0.1
        self.distance_between_wheels = 0.44
        self.distance_from_center_to_wheels = self.distance_between_wheels / 2

        self.motors = ["frame_back_right2motor_back_right", "frame_back_left2motor_back_left", "frame_front_right2motor_front_right", "frame_front_left2motor_front_left"]
        self.joints_name = self.get_joints_info(self.robot)
        self.links_name = self.get_link_info(self.robot)

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

    def record_imu(self) -> None:
        """Record the current real IMU measurements and store it at the IMU history dictionary.
        """
        acc, ang = self.imu.get_measurement()
        self.imu_history[TIMESTAMP].append(self.simulationTime)
        self.imu_history[ACCELEROMETER].append(acc.tolist())
        self.imu_history[GYROSCOPE].append(ang.tolist())
        
    def save_imu(self) -> None:
        """Save the current iteration IMU history in the historical list.
        """
        obj = {}
        obj[HISTORY] = self.imu_history
        obj[GENERATED_INSTANCES] = self.current_cause.get_generated_instances()
        bottle_pos_m, bottle_orn = p.getBasePositionAndOrientation(self.bottle)
        obj[BOTTLE_POSITION] = [coord * M_TO_MM for coord in bottle_pos_m]
        obj[BOTTLE_ORIENTATION] = bottle_orn
        self.historical.append(obj)

    def send_history_to_parent(self) -> None:
        """Send the recorded IMU history to the parent process through the pipe.
        """
        with os.fdopen(self.pipe, "w") as channel:
            json.dump(self.historical, channel)




    # ================ SIMULATION  ===============
    # ==================================================
    def load_simulation_scene_json(self, simulation_scene_file: str) -> SimulationScene:
        """Load the simulation scene parameters from a JSON file.
            Parameters:
                - simulation_scene_file (str): Path to the JSON file.
            Returns:
                - SimulationScene: The loaded simulation scene configuration.
        """
        return SimulationScene.model_validate_json(open(simulation_scene_file, 'r').read())

    def mm_to_m(self, value):
        """Convert a scalar or sequence from millimeters to meters."""
        if isinstance(value, (list, tuple)):
            return [v * MM_TO_M for v in value]
        return value * MM_TO_M

    def apply_simulation_params(self) -> None:
        """Apply the currently stored simulation scene parameters to the real scene.

        The JSON/simulation scene stores positions in millimeters, but PyBullet
        expects meters internally.
        """
        p.setGravity(0, 0, self.simulation_scene.gravity)
        # Apply small Z offset (in millimeters) so robot appears slightly above the ground
        robot_init_pos_mm = list(self.simulation_scene.initial_robot_position)
        # Ensure list has at least 3 elements
        while len(robot_init_pos_mm) < 3:
            robot_init_pos_mm.append(0.0)
        robot_init_pos_mm[2] = robot_init_pos_mm[2] + getattr(self, 'robot_z_offset_mm', 0.04)
        self.initial_position = self.mm_to_m(robot_init_pos_mm)
        self.initial_orientation = self.simulation_scene.initial_robot_orientation
        self.bottle_position = self.mm_to_m(self.simulation_scene.bottle_position)
        self.bottle_orientation = self.simulation_scene.bottle_orientation
        self.problem_position = self.simulation_scene.problem_position
        self.problem_orientation = self.simulation_scene.problem_orientation
        self.simulation_length = self.simulation_scene.simulation_length
        # If cause uses grid-based generation, derive repetitions from grid dimensions
        if hasattr(self.current_cause, 'grid_dimensions'):
            grid_dims = getattr(self.current_cause, 'grid_dimensions', [10, 10])
            self.num_of_repetitions = grid_dims[0] * grid_dims[1]
        else:
            self.num_of_repetitions = getattr(self.current_cause, 'num_of_repetitions', None) or self.simulation_scene.num_of_repetitions
        self.list_of_target_velocities = self.simulation_scene.list_of_target_velocities

    def simulate(self) -> None:
        """Simulate a step of the current iteration.
        """
        stepCount = 0
        self.simulationTime = 0.0
        # Simulate
        while (self.simulationTime) < float(self.simulation_length):
            
            it = time.time()
            self.do_acceleration_robot(self.simulationTime)
            self.current_cause.apply_compute(self.engine_wrapper)
            p.stepSimulation()
            self.record_imu()
            stepCount += 1
            self.simulationTime = stepCount * self.dt
            et = time.time()
            
            if self.realTime:
                time.sleep(max(0, self.dt - (et - it))) # Sleep to maintain real-time simulation
        return
    
    def doSimulations(self) -> None:
        """Simulate multiple iterations and collect all IMU recordings, sending them to the parent process.
        """
        self.intialState = p.saveState() # Save clean state
        for i in range(self.num_of_repetitions):
            self.current_repetition = i
            self.current_cause.apply(self.engine_wrapper)
            self.init_imu_record()
            self.simulate()
            self.save_imu()
            self.clean_bodies()
            self.reset_wheels_movement()
            self.current_cause = copy.deepcopy(self.initial_cause)
            p.restoreState(stateId=self.intialState) # Restore the clean state
        return
    
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
        self.wheel_movement["frame_front_right2motor_front_right"] = True
        self.wheel_movement["frame_back_right2motor_back_right"] = True
        self.wheel_movement["frame_front_left2motor_front_left"] = True
        self.wheel_movement["frame_back_left2motor_back_left"] = True
    
    def initialze_wheel_simplified_names_map(self) -> None:
        """Initialize the mapping of wheel simplified names to full joint names.
        """
        self.wheel_names = {}
        self.wheel_names["FL"] = "frame_front_left2motor_front_left"
        self.wheel_names["FR"] = "frame_front_right2motor_front_right"
        self.wheel_names["BL"] = "frame_back_left2motor_back_left"
        self.wheel_names["BR"] = "frame_back_right2motor_back_right"
        
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
        body_position_m = self.mm_to_m(body_position)
        self.loaded_bodies.append(p.loadURDF(body_file, basePosition=body_position_m))

    def set_robot_wheel_moving(self, simplified_wheel_name:str, moving:bool):
        """ ENGINE: Set robot wheel moving """
        self.wheel_movement[self.wheel_names[simplified_wheel_name]] = moving
    
    def set_gravity(self, gravity:float=-9.81):
        """ ENGINE: Set gravity """
        p.setGravity(0, 0, gravity)
        
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
        wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.get_target_velocity(simulationTime), self.angularSpeed)
        for motor_name in self.motors:
            if not self.wheel_movement[motor_name]: wheels_velocity[motor_name] = 0
        for motor_name in self.motors:
            p.setJointMotorControl2(bodyUniqueId=self.robot,
                                    jointIndex=self.joints_name[motor_name],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=wheels_velocity[motor_name],
                                    force=10
            )
            
    def get_target_velocity(self, simulationTime: float) -> float:
        """Get the target forward velocity of the robot at a specific simulation timestamp.
            Parameters:
                - simulationTime (float): The simulation time in seconds.
            Returns:
                - float: Forward velocity value (0 if no target velocity found for that timestamp).
        """
        # Get velocity from closest timestamp in the list BEFORE the given one
        closest_timestamp = min([h for h in self.list_of_target_velocities[TIMESTAMP] if h <= simulationTime], key=lambda h: abs(h - simulationTime), default=None)
        # Get index of the closest timestamp
        if closest_timestamp is not None:
            index = self.list_of_target_velocities[TIMESTAMP].index(closest_timestamp)
            return self.list_of_target_velocities[ADV_SPEED][index]
        else:
            self.logger.log(f"No target velocity found for simulation time {simulationTime}. Returning default stop (0) speed.", style="yellow")
            return 0

    def get_forward_velocity(self) -> float:
        """Get the forward velocity of the robot based on current wheel velocities.
            Returns:
                - float: Forward velocity in m/s.
        """
        wheel_velocities = {}
        for motor_name in self.motors:
            wheel_velocities[motor_name] = p.getJointState(self.robot, self.joints_name[motor_name])[1]
        forward_velocity = (wheel_velocities["frame_front_left2motor_front_left"] +
                            wheel_velocities["frame_front_right2motor_front_right"] +
                            wheel_velocities["frame_back_left2motor_back_left"] +
                            wheel_velocities["frame_back_right2motor_back_right"]) * self.wheels_radius / 4
        return forward_velocity

    def get_angular_velocity(self) -> float:
        """Get the angular velocity of the robot based on current wheel velocities.
            Returns:
                - float: Angular velocity in rad/s.
        """
        wheel_velocities = {}
        for motor_name in self.motors:
            wheel_velocities[motor_name] = p.getJointState(self.robot, self.joints_name[motor_name])[1]
        angular_velocity = ((wheel_velocities["frame_front_right2motor_front_right"] +
                            wheel_velocities["frame_back_right2motor_back_right"] -
                            wheel_velocities["frame_front_left2motor_front_left"] -
                            wheel_velocities["frame_back_left2motor_back_left"]) * self.wheels_radius /
                            2 * self.distance_between_wheels)
        return angular_velocity

    def get_wheels_velocity_from_forward_velocity_and_angular_velocity(self, forward_velocity: float = 0, angular_velocity: float = 0) -> dict:
        """Calculate required wheel velocities from forward and angular velocities for a differential drive robot.
            Parameters:
                - forward_velocity (float): Forward velocity of the robot (default: 0).
                - angular_velocity (float): Angular velocity of the robot (default: 0).
            Returns:
                - dict: Dictionary mapping motor names to their required velocities.
        """
        wheels_velocity = {
            "frame_front_left2motor_front_left": forward_velocity / self.wheels_radius - self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_front_right2motor_front_right": forward_velocity / self.wheels_radius + self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_back_left2motor_back_left": forward_velocity / self.wheels_radius - self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_back_right2motor_back_right": forward_velocity / self.wheels_radius + self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius}
        return wheels_velocity

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
    args = parser.parse_args()

    try:
        logger.log("Arguments parsed successfully.")
        simulator = CausesSimulator(args.cause, args.simulation_scene, args.pipe, logger, args.real_time)
        
        logger.log("Starting simulations...")
        simulator.doSimulations()    
        
        logger.log(f"""Simulations done.
        Historical data:
            Number of historicals: {len(simulator.historical)}
            Combined number of recordings: {sum(len(h[HISTORY][TIMESTAMP]) for h in simulator.historical)}""")
        
        logger.log("Sending historicals to parent process...")
        simulator.save_raw_data() 
        simulator.send_history_to_parent()
        
        logger.log("Sent. Exiting...")
    except Exception as e:
        logger.log(f"Fatal exception occurred: {e}", style="bold red")
        logger.log(traceback.format_exc(), style="red")
        sys.exit(1)


if __name__ == "__main__":
    main()

