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

# Keys of 
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"

class CausesSimulator:

    def __init__(self, num_of_repetitions, simulation_length, initial_position, final_position, cause_type, coordinates, cause_range, pipe, wheel=None, real_time=False):
        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================
        self.num_of_repetitions = int(num_of_repetitions)
        self.simulation_length = ast.literal_eval(simulation_length)
        self.initial_position = ast.literal_eval(initial_position)
        self.final_position = final_position
        self.cause_type = cause_type
        self.coordinates = ast.literal_eval(coordinates)
        self.cause_range = cause_range
        self.pipe = int(pipe)
        self.wheel = wheel
        self.real_time = real_time
        self.physicsClient = p.connect(p.GUI)
        self.stepCount = 0
        self.currentObject = None
        self.forwardSpeed = 0.7
        self.angularSpeed = 0.0
        self.historical = []
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        # p.setRealTimeSimulation(1) # Enable real-time simulation
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
        self.imu_history = {}
        self.imu_history[TIMESTAMP] = []
        self.imu_history[ACCELEROMETER] = []
        self.imu_history[GYROSCOPE] = []

    def record_imu(self, current_time):       
        acc, ang = self.imu.get_measurement()
        self.imu_history[TIMESTAMP].append(current_time)
        self.imu_history[ACCELEROMETER].append(acc.tolist())
        self.imu_history[GYROSCOPE].append(ang.tolist())
        
    def send_history_to_parent(self):
        with os.fdopen(self.pipe, "w") as channel:
            json.dump(self.historical, channel)


    def simulate(self) -> str | None:
        #print("Simulated simulation...")
        stepCount = 0
        # Simulate
        while (stepCount * self.dt) < float(self.simulation_length):
            it = time.time()
            wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.forwardSpeed, self.angularSpeed)
            for motor_name in self.motors:
                p.setJointMotorControl2(bodyUniqueId=self.robot,
                                        jointIndex=self.joints_name[motor_name],
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=wheels_velocity[motor_name],
                                        force=10)
            p.stepSimulation()
            self.record_imu(stepCount * self.dt)
            et = time.time()
            stepCount += 1 
            if self.real_time:
                time.sleep(max(0, self.dt - (et - it))) # Sleep to maintain real-time simulation
        #print("Simulation finished.")
        return None
    
    def doSimulation(self) -> str | None:
        self.intialState = p.saveState()
        for i in range(self.num_of_repetitions):
            
            self.placeObject()
            self.init_imu_record()
            res = self.simulate()
            if res is not None:
                return res
            if self.currentObject: p.removeBody(self.currentObject)
            p.restoreState(stateId=self.intialState)
            self.historical.append(self.imu_history)
        self.send_history_to_parent()


        return None
    
    def calculateRandomObjectPosition(self):
        coordinate = []
        
        # Generate random number with uniform
        dx, dy = np.random.uniform(-float(self.cause_range), float(self.cause_range), 2)
        
        # Calculate new position
        coordinate.append(self.coordinates[0][0] + dx)
        coordinate.append(self.coordinates[0][1] + dy)
        coordinate.append(self.coordinates[0][2])
        
        return coordinate
        

    def placeObject(self):
        match self.cause_type:
            case "bump":
                position:list = self.calculateRandomObjectPosition()
                print(f"Generating bump at {position}")
                self.currentObject = p.loadURDF("../../etc/URDFs/bump/bump_100x5cm.urdf", position)
                
            case "wheel":
                
                pass
            case _:
                print(f"Unknown cause type {self.cause_type}")
                pass

    # =============== ROBOT KINEMATICS  ================
    # ==================================================

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

        # Populate the dictionary with joint names and IDs
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
        description = "Initis a simulation of a cause given its data.",
        epilog = "RoboLab - 2026"
    )
    
    parser.add_argument('-n', '--num_of_repetitions', required=True, help="Number of times the simulation will repeat (int)")
    parser.add_argument('-l', '--length', required=True, help="The length of the simulation (float)")
    parser.add_argument('-i', '--initial_position', required=True, help="The initial position of the robot (string quaternion in list format)")
    parser.add_argument('-f', '--final_position', required=True, help="The final position of the robot (string quaternion in list format)")
    parser.add_argument('-t', '--cause_type', required=True, help="The type of the cause (string)")
    parser.add_argument('-c', '--coordinates', required=True, help="The coordinates of the cause (string quaternion in list format)")
    parser.add_argument('-r', '--cause_range', required=True, help="The range of the cause (float)")
    parser.add_argument('-w', '--wheel', required=False, help="The wheel of the robot that is affected by the cause (string)")
    parser.add_argument('-rt', '--real_time', required=False, action='store_true')
    parser.add_argument('-p',  '--pipe', required=True, help="Name of the pipe with which inner simulator will comunicate with the cause simulator instance")
    args = parser.parse_args()
    
    print("Arguments parsed successfully.")
    simulator = CausesSimulator(args.num_of_repetitions, args.length, args.initial_position, args.final_position, args.cause_type, args.coordinates, args.cause_range, args.pipe, args.wheel, args.real_time)
    print("Simulator initialized.")
    res = simulator.doSimulation()
    print("Simulation done.")
    if res is not None:
        print("Cause location:", res)
        #devolver quat
    else:
        print("No cause detected.")


if __name__ == "__main__":
    main()

