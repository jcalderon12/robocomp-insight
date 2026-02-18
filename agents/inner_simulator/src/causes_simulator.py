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
import random as rnd
from uuid import uuid4

# Keys of 
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"

class CausesSimulator:

    def __init__(self, num_of_repetitions, simulation_length, initial_position, final_position, cause_file_path, pipe, real_time=False):
        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================
        
        # Maps
        self.initialze_wheel_simplified_names_map()
        self.initialize_wheels_movement_map()
        self.initialize_bodies_map()

        # Simulation-related parameters
        self.num_of_repetitions = int(num_of_repetitions)
        self.simulation_length = ast.literal_eval(simulation_length)
        self.initial_position = ast.literal_eval(initial_position)
        self.final_position = final_position
        self.cause_file_path = cause_file_path
        self.cause = None
        
        # Import the cause class from the cause file path. The cause class name is always "Cause".
        spec = None
        cause_module = None
        try:
            from importlib.util import spec_from_file_location, module_from_spec
            spec = spec_from_file_location("cause_module", self.cause_file_path)
            cause_module = module_from_spec(spec)
            spec.loader.exec_module(cause_module)
            self.cause = cause_module.Cause()
        except Exception as e:
            print(f"Error importing cause from file {self.cause_file_path}: {e}")
            sys.exit(1)
        
        
        # Working parameters
        self.pipe = int(pipe)
        self.real_time = real_time
        self.simulationTime = 0.0
        self.forwardSpeed = 0.7
        self.angularSpeed = 0.0
        self.historical = []
        
        # Engine parameters
        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-15,
                                        cameraTargetPosition=[-1.3, -0.5, 0.2])

        self.dt = 1./62. # Simulation time step (60 Hz)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)
        self.flags = p.URDF_USE_INERTIA_FROM_FILE

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD ROBOT IN THE SIMULATION
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", [self.initial_position[0][0], self.initial_position[0][1], self.initial_position[0][2]], flags=self.flags)        
        
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

    def init_imu_record(self):
        """ Initialize the real IMU history dictionary for recording sensor data during the mission. """
        self.imu_history = {}
        self.imu_history[TIMESTAMP] = []
        self.imu_history[ACCELEROMETER] = []
        self.imu_history[GYROSCOPE] = []

    def record_imu(self):
        """" Record the current real IMU measurements and store it at the IMU history dictionary. """       
        acc, ang = self.imu.get_measurement()
        self.imu_history[TIMESTAMP].append(self.simulationTime)
        self.imu_history[ACCELEROMETER].append(acc.tolist())
        self.imu_history[GYROSCOPE].append(ang.tolist())
        
    def save_imu(self):
        """ Save the current iteration IMU history in the historical list. """
        self.historical.append(self.imu_history)
        
    def send_history_to_parent(self):
        """ Send the recorded IMU history to the parent process through the pipe. """
        with os.fdopen(self.pipe, "w") as channel:
            json.dump(self.historical, channel)




    # ================ SIMULATION  ===============
    # ==================================================

    def simulate(self):
        """ Simulates a step of the current iteration. """
        stepCount = 0
        self.simulationTime = 0.0
        # Simulate
        while (self.simulationTime) < float(self.simulation_length):
            
            it = time.time()
            self.apply_compute_actions()
            p.stepSimulation()
            self.record_imu()
            stepCount += 1
            self.simulationTime = stepCount * self.dt
            et = time.time()
            
            if self.realTime:
                time.sleep(max(0, self.dt - (et - it))) # Sleep to maintain real-time simulation
        return
    
    def doSimulations(self):
        """ Simulates as many iterations as the number of repetitions specified,collecting all
        IMU recordings from each iteration and sending them to the parent process at the end. """
        self.intialState = p.saveState() # Save clean state
        for i in range(self.num_of_repetitions):
            self.apply_actions()
            self.init_imu_record()
            self.simulate()
            self.save_imu()
            self.clean_bodies()
            p.restoreState(stateId=self.intialState) # Restore the clean state
        return
    
    def clean_bodies(self):
        """ Remove all bodies from the simulation except the plane and the robot. """
        for body_id in self.loaded_bodies.values():
            p.removeBody(body_id)
        self.loaded_bodies = {}

    # ================= ENGINE DATA ====================
    # ==================================================

    def initialize_wheels_movement_map(self):
        self.wheel_movement = {}
        self.wheel_movement["frame_front_right2motor_front_right"] = True
        self.wheel_movement["frame_back_right2motor_back_right"] = True
        self.wheel_movement["frame_front_left2motor_front_left"] = True
        self.wheel_movement["frame_back_left2motor_back_left"] = True
    
    def initialze_wheel_simplified_names_map(self):
        self.wheel_names = {}
        self.wheel_names["FL"] = "frame_front_left2motor_front_left"
        self.wheel_names["FR"] = "frame_front_right2motor_front_right"
        self.wheel_names["BL"] = "frame_back_left2motor_back_left"
        self.wheel_names["BR"] = "frame_back_right2motor_back_right"
        

    def initialize_bodies_map(self):
        self.loaded_bodies = {}
    

 
    # ================= ENGINE "API" ===================
    # ==================================================

    def get_pybullet_instance(self):
        return p
    
    def instantiate_body(self, body_file:str, body_position:tuple, identifier:str=str(uuid4())):
        self.loaded_bodies[identifier] = p.loadURDF(body_file, basePosition=body_position)

    def set_robot_wheel_moving(self, simplified_wheel_name:str, moving:bool):
        self.wheel_movement[self.wheel_names[simplified_wheel_name]] = moving
    
    def set_gravity(self, gravity:float=-9.81):
        p.setGravity(0, 0, gravity)
        
    def get_simulation_time(self):
        return self.simulationTime

    # =============== ROBOT KINEMATICS  ================
    # ==================================================

    def do_acceleration_robot(self, sim_time : float):
        wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.forwardSpeed, self.angularSpeed)
        for motor_name in self.motors:
        
            if (self.wheel == motor_name) and (sim_time >= self.wheel_malfunction_timeout):
                p.setJointMotorControl2(bodyUniqueId=self.robot,
                                        jointIndex=self.joints_name[motor_name],
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=0,
                                        force=10)            
            else:
                p.setJointMotorControl2(bodyUniqueId=self.robot,
                                        jointIndex=self.joints_name[motor_name],
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=wheels_velocity[motor_name],
                                        force=10)
            

    def get_forward_velocity(self):
        """
        Get the forward velocity of the robot

        :return: Forward velocity
        """
        wheel_velocities = {}
        for motor_name in self.motors:
            wheel_velocities[motor_name] = p.getJointState(self.robot, self.joints_name[motor_name])[1]
        forward_velocity = (wheel_velocities["frame_front_left2motor_front_left"] +
                            wheel_velocities["frame_front_right2motor_front_right"] +
                            wheel_velocities["frame_back_left2motor_back_left"] +
                            wheel_velocities["frame_back_right2motor_back_right"]) * self.wheels_radius / 4
        return forward_velocity

    def get_angular_velocity(self):
        """
        Get the angular velocity of the robot

        :return: Angular velocity
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

    def get_wheels_velocity_from_forward_velocity_and_angular_velocity(self, forward_velocity=0, angular_velocity=0):
        """
        Get the velocity of each wheel from the forward velocity of the robot

        :param forward_velocity: Forward velocity of the robot
        :param angular_velocity: Angular velocity of the robot
        :return: Dictionary with the velocity of each wheel
        """
        wheels_velocity = {
            "frame_front_left2motor_front_left": forward_velocity / self.wheels_radius - self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_front_right2motor_front_right": forward_velocity / self.wheels_radius + self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_back_left2motor_back_left": forward_velocity / self.wheels_radius - self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius,
            "frame_back_right2motor_back_right": forward_velocity / self.wheels_radius + self.distance_from_center_to_wheels * angular_velocity / self.wheels_radius}
        return wheels_velocity

    # =============== PYBULLET MODELS INFO  ================
    # ======================================================


    def get_joints_info(self, robot_id):
        """
        Get joint names and IDs from a robot model

        @param robot_id: ID of the robot model in the simulation
        @return: Dictionary with joint names as keys and joint IDs as values
        """
        joint_name_to_id = {}
        # Get number of joints in the model
        num_joints = p.getNumJoints(robot_id)
        # print("Num joints:", num_joints)

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

    def get_link_info(self, robot_id):
        """
        Get link names and IDs from a robot model

        @param robot_id: ID of the robot model in the simulation
        @return: Dictionary with link names as keys and link IDs as values
        """
        link_name_to_id = {}
        # Get number of joints in the model
        num_links = p.getNumJoints(robot_id)
        # print("Num links:", num_links)

        # Populate the dictionary with link names and IDs
        for i in range(num_links):
            link_info = p.getJointInfo(robot_id, i)
            link_name = link_info[12].decode("utf-8")
            link_name_to_id[link_name] = i
        return link_name_to_id

def main():
    print("Starting causes simulator...")
    parser = argparse.ArgumentParser(
        prog = "Causes simulator",
        description = "Simulates multiple iterations of a scenary and a cause given its data.",
        epilog = "RoboLab - 2026"
    )
    
    parser.add_argument('-n', '--num_of_repetitions', required=True, help="Number of times the simulation will repeat (int)")
    parser.add_argument('-l', '--length', required=True, help="The length of the simulation (float)")
    parser.add_argument('-i', '--initial_position', required=True, help="The initial position of the robot (string quaternion in list format)")
    parser.add_argument('-f', '--final_position', required=True, help="The final position of the robot (string quaternion in list format)")
    parser.add_argument('-p', '--pipe', required=True, help="Name of the pipe with which inner simulator will comunicate with the cause simulator instance")
    parser.add_argument('-rt', '--real_time', required=False, help="Run simulations at real-time speed. Not recommended outside of debugging.", action='store_true')
    args = parser.parse_args()
    
    print("Arguments parsed successfully.")
    simulator = CausesSimulator(args.num_of_repetitions, args.length, args.initial_position, args.final_position, args.pipe, args.real_time)
    
    print("Starting simulations...")
    simulator.doSimulations()    
    
    print(f"""Simulations done.
    Historical data:
        Number of historicals: {len(simulator.historical)}
        Combined number of recordings: {sum(len(h[TIMESTAMP]) for h in simulator.historical)}""")
    
    print("Sending historicals to parent process...")
    simulator.send_history_to_parent()
    
    print("Sent. Exiting...")


if __name__ == "__main__":
    main()

