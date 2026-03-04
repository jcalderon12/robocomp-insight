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
from rich.console import Console
from genericworker import *
import interfaces as ifaces
from src.simulation_scene import SimulationScene

import json
import pybullet as p
import numpy as np
import locale
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import time
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

# causes.json constants
JSON_FILE = "src/causes.json"

# Keys of IMU history dictionary
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
BOTTLE_POSITION = "bottle_position"

# Keys of IMU historical dictionary (recieved from causes simulator)
HISTORY = "history"
GENERATED_INSTANCES = "generated_instances"

# Positions of the bodies in the scene
ROBOT_POS = [-3.7, -0.7, 0.01]
BOTTLE_POS = [-3.65, -0.59, 0.7725]


matplotlib.use("TkAgg")

sys.path.append('/opt/robocomp/lib')

dir_name = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(dir_name)
sys.path.append(parent_dir + "/src/")
console = Console(highlight=False)

from pydsr import *

from pybullet_imu import IMU

from causes_simulator import CausesSimulator

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)
    
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        locale.setlocale(locale.LC_NUMERIC, 'en_US.UTF-8')

        self.print_dsr_signals = False # PONER A TRUE SI SE QUIERE VER EN CONSOLA LAS SEÑALES DSR

        self.state = "IDLE" # Possible states: "IDLE", "INNER_SIMULATOR"


        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================

        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
        # p.setRealTimeSimulation(1) # Enable real-time simulation
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-15,
                                     cameraTargetPosition=[-1.3, -0.5, 0.2])
        
        self.dt = 1./62. # Simulation time step (60 Hz)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)
        
        p.setGravity(0, 0, -9.81)

        # ================= PYBULLET MODELS LOADING  ================
        # ===========================================================
# Keys of 


        flags = p.URDF_USE_INERTIA_FROM_FILE

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD OBSTACLES IN THE SIMULATION
        # self.bump_100x5cm = p.loadURDF("./URDFs/bump/bump_100x5cm.urdf", [0, -0.33, 0.001], flags=flags)
        # self.bump_1000x10cm = p.loadURDF("./URDFs/bump/bump_100x10cm.urdf", [0, -0.33, 0.001], flags=flags)
        # self.cylinder_bump_10m = p.loadURDF("./URDFs/bump/cylinder_bump_10m.urdf", [0, -0.8, 0.001], p.getQuaternionFromEuler([0, 0, np.pi/2]), flags=flags)

        # LOAD ROBOT IN THE SIMULATION
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", ROBOT_POS, flags=flags)

        # LOAD BOTTLE IN THE SIMULATION
        self.bottle = p.loadURDF("../../etc/URDFs/bottle/bottle.urdf", BOTTLE_POS, flags=flags)

        time.sleep(0.5)


        # ================ ROBOT PARAMETERS  ===============
        # ==================================================

        self.wheels_radius = 0.1
        self.distance_between_wheels = 0.44
        self.distance_from_center_to_wheels = self.distance_between_wheels / 2

        self.motors = ["frame_back_right2motor_back_right", "frame_back_left2motor_back_left", "frame_front_right2motor_front_right", "frame_front_left2motor_front_left"]
        self.joints_name = self.get_joints_info(self.robot)
        self.links_name = self.get_link_info(self.robot)

        self.init_pos, self.init_orn = p.getBasePositionAndOrientation(self.robot)
        self.forward_vel, self.angular_vel = 0, 0

        self.print_time = self.actual_time = self.initial_time = time.time()

        # ================ IMU SENSOR SETUP  ================
        # ====================================================

        self.imu = IMU(self.robot, self.dt)
        self.last_imu_measurement = self.imu.get_measurement()
        self.publish_imu_to_dsr(self.last_imu_measurement[0], self.last_imu_measurement[1])

        # ================ CAUSE SIMULATOR  ================
        # ====================================================
        # Declare causes data dict
        self.sim_scene = SimulationScene.model_construct()
        # Set initial pose (Debugging purposes)
        self.sim_scene.gravity = -9.81
        self.sim_scene.initial_robot_position, self.sim_scene.initial_robot_orientation = p.getBasePositionAndOrientation(self.robot)
        self.sim_scene.robot_velocity = 0


    def init_imu_record(self):
        """ Initialize the real IMU history dictionary for recording sensor data during the mission. """
        self.imu_history = {}
        self.imu_history[TIMESTAMP] = []
        self.imu_history[ACCELEROMETER] = []
        self.imu_history[GYROSCOPE] = []


    def record_imu(self, current_time):
        """" Record the current real IMU measurements and store it at the IMU history dictionary. """ 
        # Get real IMU node data from DSR
        real_imu_node = self.g.get_node("imu")
        if real_imu_node is not None:
            acc = real_imu_node.attrs["imu_accelerometer"].value
            ang = real_imu_node.attrs["imu_gyroscope"].value
        self.imu_history[TIMESTAMP].append(current_time)
        self.imu_history[ACCELEROMETER].append(acc.tolist())
        self.imu_history[GYROSCOPE].append(ang.tolist())


    # DEBUG: Write fake JSON file
    def writeSimulationScene(self):
       """ DEBUG: Write fake JSON file """
       # Fake SIM_SCENE JSON file
       self.sim_scene.problem_position, self.sim_scene.problem_orientation = p.getBasePositionAndOrientation(self.robot)
       self.sim_scene.simulation_length = self.actual_time - self.initial_time
       self.sim_scene.num_of_repetitions = 300
       self.sim_scene.bottle_position = BOTTLE_POS
       self.sim_scene.bottle_orientation = [0,0,0,0]
       self.sim_scene.model_validate(self.sim_scene.__dict__)
       file = open("src/sim_scene.json", "w")
       file.write(self.sim_scene.model_dump_json(indent=4))
       file.close()


    def loadCausesJson(self):
        """ Loads the causes JSON file """
        try:
            file = open(JSON_FILE, "r")
            causes_data = json.loads(file.read())
            self.causes_data = causes_data
            file.close()
            print(f"> [loadCausesJson] {JSON_FILE} content: \n{causes_data}")
        except Exception as e:
            print(f"> [loadCausesJson] Error while reading from {JSON_FILE}: " + str(e))


    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        self.show_compute_time_step()
        actual_imu_measurement = self.imu.get_measurement()

        if (not np.allclose(self.last_imu_measurement[0], actual_imu_measurement[0], atol=0.001) 
            and 
            not np.allclose(self.last_imu_measurement[1], actual_imu_measurement[1], atol=0.001)):
            self.publish_imu_to_dsr(actual_imu_measurement[0], actual_imu_measurement[1])

        p.stepSimulation()
        match self.state:
            case "IDLE":
                follow_node = self.g.get_node("follow_me")
                if follow_node is not None and follow_node.attrs["aff_interacting"].value:
                    self.state = "INNER_SIMULATOR"
                    self.init_imu_record()
                    self.initial_time = time.time()
                    self.actual_time = time.time()
                pass

            case "INNER_SIMULATOR":
                self.forward_vel, self.angular_vel = self.get_velocities_from_dsr()
                wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.forward_vel, self.angular_vel)
                for motor_name in self.motors:
                    p.setJointMotorControl2(bodyUniqueId=self.robot,
                                            jointIndex=self.joints_name[motor_name],
                                            controlMode=p.VELOCITY_CONTROL,
                                            targetVelocity=wheels_velocity[motor_name],
                                            force=10)
                self.sim_scene.final_robot_position, self.sim_scene.final_robot_orientation = p.getBasePositionAndOrientation(self.robot)
                self.actual_time = time.time()
                self.sim_scene.robot_velocity = max(self.get_velocities_from_dsr()[0], self.sim_scene.robot_velocity)
                self.record_imu(self.actual_time - self.initial_time)
                
                if self.check_for_problem():
                    # Stop moving the robot
                    for motor_name in self.motors:
                        p.setJointMotorControl2(bodyUniqueId=self.robot,
                                                jointIndex=self.joints_name[motor_name],
                                                controlMode=p.VELOCITY_CONTROL,
                                                targetVelocity=0,
                                                force=10)
                        
                    self.state = "SIMULATE_REASON"
                
            case "SIMULATE_REASON":
                # Read JSON to get causes
                self.loadCausesJson()

                # Launch subprocesses for simulation
                print("Launching subprocesses...")                
                pids = []
                historicals = {}
                for cause in self.causes_data:
                    rpipe, wpipe = os.pipe()
                    json_data = {"cause": cause}
                    json_data = json.dumps(json_data)

                    pids.append([subprocess.Popen(["python", "src/causes_simulator.py", "-c", json_data, "-s", "src/sim_scene.json", "-p", str(wpipe)], pass_fds=[wpipe]), rpipe])
                    # Close wpipe descriptor to prevent deadlocks
                    os.close(wpipe)
                
                # Wait for subprocesses and collect pipe data
                print("Waiting for subprocesses...")
                while (True):
                    # pid[0] is the Popen class itself, and pid[1] is the (read) pipe which the process uses to communicate with inner simulator
                    for pid in pids:
                        if pid[0] not in historicals:
                            pipe = os.fdopen(pid[1])
                            print(f"Now reading pipe of process {pid[0]}...")
                            historicals[pid[0]] = pipe.read()
                            pipe.close()
                            print(f"Pipe for {pid[0]} has been read!")
                            
                    if len(historicals) == len(pids): break

                # Transform pipe data from JSON string to dictionary
                for pid in pids:
                    historicals[pid[0]] = json.loads(historicals[pid[0]])  
    
                print(f"Received historicals from {len(historicals)} simulations.")
                i = 0
                for h in historicals:
                    print(f"    Historical {i} recordings: {len(historicals[h])}")
                    i += 1
                print("Historical INNER frames:", len(self.imu_history[TIMESTAMP]))

                # Check for any possible matches between causes and real IMU's
                threads = []
                sim_out = {}
                sim_out["sim_scene"] = self.sim_scene.model_dump()
                sim_out["registers"] = []
                # {cause_definition: "asdasdasdas", top_five:[]}, {cause_definition: "asdas", top_five:[]}
                with ProcessPoolExecutor(max_workers=2) as executor:
                    # Launch proccesses
                    for h in historicals:
                        threads.append(executor.submit(SpecificWorker.find_matching_imu_recordings, self.imu_history, historicals[h]))
                    # Wait for processes and check results
                    i = 0
                    for h in historicals:
                        res = threads[i].result()
                        items = []
                        print("Top 5 best recordings for cause", self.causes_data[i]["name"], ":")
                        for rec in res:
                            print("\tRecording", rec[0], "with score", rec[1])
                            items.append(historicals[h][rec[0]]) #stores the whole recording (ts, acc and gyro) of the historical with id rec[0]
                        
                        row = {"cause_definition": self.causes_data[i], "top_five": items}
                        sim_out["registers"].append(row)
                        i += 1

                # Write results to JSON file
                
                # Sim_output structure:
                # sim_out = {
                #     "sim_scene": {JSON data from sim_scene.json},
                #     "registers": [
                #         {
                #             "cause_definition": {JSON data from causes.json},
                #             "top_five": [
                #                 {
                #                     "timestamp": [ts1, ts2, ts3, ...],
                #                     "accelerometer": [[acc_x1, acc_y1, acc_z1
                #                                      [acc_x2, acc_y2, acc_z2],
                #                                      [acc_x3, acc_y3, acc_z3],
                #                                      ...],
                #                     "gyroscope": [[gyro_x1, gyro_y1, gyro_z1
                #                                    [gyro_x2, gyro_y2, gyro_z2],
                #                                    [gyro_x3, gyro_y3, gyro_z3],
                #                                    ...]
                #                 },
                #                 ... (up to 5 recordings)
                #             ]
                #         },
                #         ... (one for each cause)
                #     ]
                # }
                
                output = open(f"sim_output.json", "w")
                output.write(json.dumps(sim_out, indent=4))
                output.close()
                
                print("Simulations finished. Results written to sim_output.json!")
                
                # Calculate the aproximate bottle position from the best 5 recordings for each cause
                for register in sim_out["registers"]:
                    list_of_positions = []
                    list_of_positions.extend([recording[BOTTLE_POSITION] for recording in register["top_five"]])
                    # Average position (in the three axis)
                    avg_position = np.mean(list_of_positions, axis=0)
                
                    print(f"Approximate bottle position for {register['cause_definition']['name']}: {avg_position}")
                    
                self.state = "IDLE"                
                
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)
        

    # =============== SIMULATION HELPERS  ================
    # ====================================================

    def check_for_problem(self):
        follow_node = self.g.get_node("follow_me")
        if follow_node is not None:
                if not follow_node.attrs["aff_interacting"].value:
                    self.writeSimulationScene()
                    return True
        return False

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

    @staticmethod
    def find_matching_imu_recordings(rimu, simu):
        """
        Find if there is any match between the real IMU recordings and the simulated ones.
        If there is a match, it means that the cause being simulated could be the reason behind the problem detected in the real robot.
        :param rimu: Dictionary with frames of the real IMU recordings (timestamp, accelerometer and gyroscope)
        :param simu: List of dictioraries, each one with frames of the simulated IMU recordings (timestamp, accelerometer, gyroscope)
        :param margin: (Default=0.05) Margin of error to consider a match between real and simulated IMU measurements
        :return: int, the ID of the simulation that matched the real IMU. If no match, return None.
        """
        
        # RIMU structure:
        # rimu = {
        #     "timestamp": [ts1, ts2, ts3, ...],
        #     "accelerometer": [[acc_x1, acc_y1, acc_z1
        #                      [acc_x2, acc_y2, acc_z2],
        #                      [acc_x3, acc_y3, acc_z3],
        #                      ...],
        #     "gyroscope": [[gyro_x1, gyro_y1, gyro_z1
        #                    [gyro_x2, gyro_y2, gyro_z2],
        #                    [gyro_x3, gyro_y3, gyro_z3],
        #                    ...]
        # }
        #
        # SIMU structure:
        # simu = [
        #     {
        #         "history": {
        #             "timestamp": [ts1, ts2, ts3, ...],
        #             "accelerometer": [[acc_x1, acc_y1, acc_z1
        #                              [acc_x2, acc_y2, acc_z2],
        #                              [acc_x3, acc_y3, acc_z3],
        #                              ...],
        #             "gyroscope": [[gyro_x1, gyro_y1, gyro_z1
        #                            [gyro_x2, gyro_y2, gyro_z2],
        #                            [gyro_x3, gyro_y3, gyro_z3],
        #                            ...]
        #         },
        #
        #         "generated_instances": {
        #             "generatorA": { value1, value2, ...},
        #             "generatorB": { value1, value2, ...},
        #             ...
        #         }
        #
        
        scores = []
        # for each simu simulation...
        for s in range(len(simu)):
            print(f"Now sorting {len(simu[s][HISTORY][TIMESTAMP])} simu (id={s}) frames against {len(rimu[TIMESTAMP])} rimu frames.")
            # for each frame of the simulation...
            for i in range(len(simu[s][HISTORY])):

                # Find closest timestamp value of simu to rimu's timestamp
                ts = min(rimu[TIMESTAMP], key=lambda v: abs(v - simu[s][HISTORY][TIMESTAMP][i]))
                frame_id = rimu[TIMESTAMP].index(ts)

                diff_acc = np.linalg.norm(np.array(simu[s][HISTORY][ACCELEROMETER][i]) - np.array(rimu[ACCELEROMETER][frame_id]))
                diff_gyro = np.linalg.norm(np.array(simu[s][HISTORY][GYROSCOPE][i]) - np.array(rimu[GYROSCOPE][frame_id]))
                total_diff = (diff_acc + diff_gyro) / 2
                scores.append([s, total_diff])

        print(f"Sorting {len(scores)} entries.")
        scores.sort(key=lambda x: x[1])
        return scores[:5]
    
    

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
    
    # ================= DSR INTERACTION  ================
    # ==================================================

    def get_velocities_from_dsr(self):
        """
        Get the forward and angular velocities from the DSR graph

        :return: Forward and angular velocities
        """
        forward_velocity = 0
        angular_velocity = 0

        robot_node = self.g.get_node("robot")
        if robot_node is not None:
            robot_node = robot_node
            if robot_node.attrs["robot_ref_adv_speed"].value is not None:
                forward_velocity = robot_node.attrs["robot_ref_adv_speed"].value
            if robot_node.attrs["robot_ref_rot_speed"].value is not None:
                angular_velocity = robot_node.attrs["robot_ref_rot_speed"].value

        return forward_velocity, angular_velocity
    

    def create_imu_node_in_dsr(self, acc_measured, angular_vel):
        """
        Create the IMU node in the DSR graph

        :param acc_measured: Measured acceleration
        :param angular_vel: Angular velocity
        """
        imu_node = Node(agent_id=self.agent_id, name="imu_sintetic", type="imu")

        robot_node = self.g.get_node("robot")
        if robot_node is None:
            console.print("Robot node not found in DSR graph", style="bold red")
            return

        imu_node.attrs["pos_x"] = Attribute(float(robot_node.attrs["pos_x"].value) - 100,
                                                         self.agent_id)
        imu_node.attrs["pos_y"] = Attribute(float(robot_node.attrs["pos_y"].value) + 100,
                                                         self.agent_id)
        imu_node.attrs["level"] = Attribute(robot_node.attrs['level'].value + 1,
                                                         self.agent_id)
        imu_node.attrs["parent"] = Attribute(int(robot_node.id), self.agent_id)
        imu_node.attrs["imu_accelerometer"] = Attribute(acc_measured, self.agent_id)
        imu_node.attrs["imu_gyroscope"] = Attribute(angular_vel, self.agent_id)

        self.g.insert_node(imu_node)


    def update_imu_node_in_dsr(self, imu_node, acc_measured, angular_vel):
        """
        Update the IMU node in the DSR graph

        :param imu_node: IMU node in the DSR graph
        :param acc_measured: Measured acceleration
        :param angular_vel: Angular velocity
        """
        if imu_node.attrs["imu_accelerometer"].value is not None:
            imu_node.attrs["imu_accelerometer"].value = acc_measured.tolist()
        if imu_node.attrs["imu_gyroscope"].value is not None:
            imu_node.attrs["imu_gyroscope"].value = angular_vel.tolist()
        self.g.update_node(imu_node)

    
    def create_edge_in_dsr(self, fr_node, to_node, edge_type):
        """
        Create an edge in the DSR graph

        :param fr_node: From node
        :param to_node: To node
        :param edge_type: Type of the edge
        """
        edge = Edge(to_node.id, fr_node.id, edge_type, self.agent_id)
        self.g.insert_or_assign_edge(edge)


    def publish_imu_to_dsr(self, acc_measured, angular_vel):
        """

        Publish the IMU measurements to the DSR graph or create the node if it does not exist

        :param acc_measured: Measured acceleration
        :param angular_vel: Angular velocity
        """
        
        imu_node = self.g.get_node("imu_sintetic")
        if imu_node is not None:
            self.update_imu_node_in_dsr(imu_node, acc_measured, angular_vel)
        else:
            self.create_imu_node_in_dsr(acc_measured, angular_vel)
            self.create_edge_in_dsr(self.g.get_node("robot"), self.g.get_node("imu_sintetic"), "has")



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        if self.print_dsr_signals:
            console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        if self.print_dsr_signals:
            console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        if self.print_dsr_signals:
            console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        if self.print_dsr_signals:
            console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
