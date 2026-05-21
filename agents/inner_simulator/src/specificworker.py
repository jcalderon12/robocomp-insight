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
# File constants
JSON_FILE = "src/causes.json"
AGENTS_FOLDER = "agents/"
EM_HISTORY_FILE = "src/mission_Follow Path_25032026_120505.txt"

# Keys of IMU history dictionary
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
ADV_SPEED = "adv_speed"

# Keys of IMU historical dictionary (recieved from causes simulator)
HISTORY = "history"

# Positions of the bodies in the scene
ROBOT_POS = [-3.7, -0.3, 0.0325]
PROBLEM_POS = [0.0, 0.04, 0.001]
BOTTLE_POS = [-3.65, -0.19, 0.795]

import os
import sys
import matplotlib.pyplot as plt
import threading

from PySide6 import QtCore
from rich.console import Console
from genericworker import *
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import time
import subprocess

matplotlib.use("TkAgg")

sys.path.append('/opt/robocomp/lib')

dir_name = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(dir_name)
sys.path.append(parent_dir + "/src/")
console = Console(highlight=False)

# Get the path to 'inner_simulator' (one level up from 'src')
agent_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if agent_root not in sys.path:
    sys.path.insert(0, agent_root)

from src.simulation_scene import SimulationScene
from src.logger import Logger

import pybullet as p
import json
import episodic_memory_api as mem
import numpy as np
import locale

from concurrent.futures import ProcessPoolExecutor
from .agent_generator import *

from pybullet_imu import IMU
from pydsr import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)

        self.Period = configData["Period"]["Compute"]
    
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        locale.setlocale(locale.LC_NUMERIC, 'en_US.UTF-8')

        # Init logger
        os.path.exists("logs/specific_worker") or os.makedirs("logs/specific_worker")
        self.logger = Logger(f"logs/specific_worker/specific_worker_{time.strftime('%Y%m%d_%H%M%S')}.log")

        self.state = "IDLE" # Possible states: "IDLE", "SIMULATE_REASON", "TERMINATED"

        # Clear terminal
        os.system('cls' if os.name == 'nt' else 'clear')

        self.print_time = self.actual_time = time.time()

        # ================ CAUSE SIMULATOR  ================
        # ====================================================
        # Declare causes data dict
        self.sim_scene = SimulationScene.model_construct()
        # Set initial pose (Debugging purposes)
        self.sim_scene.gravity = -9.81
        self.sim_scene.initial_robot_position, self.sim_scene.initial_robot_orientation = ROBOT_POS, [0,0,0,1]

        # ================ EPISODIC MEMORY API =================
        # ======================================================
        # Loaded dynamically at runtime from Follow Person filepath.

        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================

        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        # p.setRealTimeSimulation(1) # Enable real-time simulation
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-15,
                                     cameraTargetPosition=[-1.3, -0.5, 0.2])
        
        self.dt = 1./62. # Simulation time step (60 Hz)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)
        
        # ================= PYBULLET MODELS LOADING  ================
        # ===========================================================

        flags = p.URDF_USE_INERTIA_FROM_FILE

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD OBSTACLES IN THE SIMULATION
        # self.bump_100x5cm = p.loadURDF("./URDFs/bump/bump_100x5cm.urdf", [0, -0.33, 0.001], flags=flags)
        # self.bump_1000x10cm = p.loadURDF("./URDFs/bump/bump_100x10cm.urdf", [0, -0.33, 0.001], flags=flags)
        # self.cylinder_bump_10m = p.loadURDF("./URDFs/bump/cylinder_bump_10m.urdf", [0, -0.8, 0.001], p.getQuaternionFromEuler([0, 0, np.pi/2]), flags=flags)

        # LOAD ROBOT IN THE SIMULATION
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", [-3.7, -0.7, 0.01], flags=flags)

        # LOAD A CYLINDER IN THE SIMULATION
        # self.cylinder = p.loadURDF("../../etc/URDFs/cylinder/cylinder.urdf", [1.3, -0.7, 0.0], flags=flags)

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

        self.print_time = time.time()
        self.actual_time = time.time()
        self.initial_time = time.time()

        # ================ IMU SENSOR SETUP  ================
        # ====================================================

        self.imu = IMU(self.robot, self.dt)
        self.last_imu_measurement = self.imu.get_measurement()
        self.publish_imu_to_dsr(self.last_imu_measurement[0], self.last_imu_measurement[1])

        
    def get_robot_adv_speed_history(self) -> dict | None:
        """Get a dict of the robot's target speed for each timestamp in the episodic memory, by looking for the "robot_ref_adv_speed" attribute in the "robot" node history.
            Returns:
                - dict: with keys "timestamp" and "adv_speed", each one with a list of values for each timestamp found in the episodic memory.
                - None: if episodic memory is not ready.
        """
        self.robot_adv_speed_history = {}
        self.robot_adv_speed_history[TIMESTAMP] = []
        self.robot_adv_speed_history[ADV_SPEED] = []
        if self.mem_api.is_ready():
            robot_events = self.mem_api.get_node_history_by_name("robot")
            initial_ts = robot_events[0].timestamp
            
            # Print statistics of type of events in robot history
            event_types = {}
            for event in robot_events:
                if event.modification_type not in event_types:
                    event_types[event.modification_type] = 1
                else:
                    event_types[event.modification_type] += 1
            self.logger.log(f"Robot events in episodic memory by modification type:", style="bold green")
            for mod_type in event_types:
                self.logger.log(f"    Type {mod_type}: {event_types[mod_type]} events", style="green")
                      
            # Filter by modification type "K" (Keyframe)
            robot_events = [event for event in robot_events if event.modification_type == "K" and "robot_ref_adv_speed" in event.attributes]
            self.logger.log(f"Found {len(robot_events)} robot events in episodic memory with modification type K related to robot_ref_adv_speed.", style="bold green")
            for event in robot_events:
                corrected_ts = event.timestamp - initial_ts
                event.timestamp = corrected_ts
                if event.attributes.get("robot_ref_adv_speed") is not None:
                    self.robot_adv_speed_history[TIMESTAMP].append(corrected_ts * 1e-9)
                    self.robot_adv_speed_history[ADV_SPEED].append(event.attributes["robot_ref_adv_speed"].value)
                break
            
            # Filter by modification type "MNA" (Modified Node Attribute)
            robot_events = self.mem_api.get_node_history_by_name("robot")
            robot_events = [event for event in robot_events if event.modification_type == "MNA" and "robot_ref_adv_speed" in event.attributes]
            self.logger.log(f"Found {len(robot_events)} robot events in episodic memory with modification type MNA related to robot_ref_adv_speed.", style="bold green")
            for event in robot_events:
                corrected_ts = event.timestamp - initial_ts
                event.timestamp = corrected_ts
                if event.attributes.get("robot_ref_adv_speed") is not None:
                    self.robot_adv_speed_history[TIMESTAMP].append(corrected_ts * 1e-9)
                    self.robot_adv_speed_history[ADV_SPEED].append(event.attributes["robot_ref_adv_speed"].value)
                    
            return self.robot_adv_speed_history
        else:
            self.logger.log("Episodic Memory API is not ready!", style="bold red")
            return None

    
    def convert_episodic_to_imu_history(self, list_of_ts: list) -> dict | None:
        """Convert the episodic memory mission to the IMU history format given a list of timestamps to fill.
            Parameters:
                - list_of_ts (list): List of timestamps in seconds to convert from episodic memory.
            Returns:
                - dict: with keys "timestamp", "accelerometer" and "gyroscope", each one with a list of values for each timestamp in the list_of_ts.
                - None: if episodic memory is not ready.
        """
        self.imu_history = {}
        self.imu_history[TIMESTAMP] = []
        self.imu_history[ACCELEROMETER] = []
        self.imu_history[GYROSCOPE] = []
        
        if self.mem_api.is_ready():
            # Download the event list
            imu_events = self.mem_api.get_node_history_by_name("imu")
            initial_ts = imu_events[0].timestamp
            # Filter by modification type "MNA" (Modified Node Attribute)
            imu_events = [event for event in imu_events if event.modification_type == "MNA"]
            initial_acc = [0.0,0.0,0.0]
            initial_gyro = [0.0,0.0,0.0]
            
            self.logger.log(f"Found {len(imu_events)} IMU events in episodic memory with modification type MNA.", style="bold green")

            # Correct each ts of the event list
            for event in imu_events:
                corrected_ts = event.timestamp - initial_ts
                event.timestamp = corrected_ts


            # Recreate event history to imy history format
            for ts in list_of_ts:
                # Convert ts (seconds) to nanoseconds
                ts = int(ts * 1e9)
                # Get the value of acc and gyro from the closest timestamp before ts (using lambda function)
                candidates = [event for event in imu_events if event.timestamp <= ts]
                closest_event = min(candidates, key=lambda event: abs(event.timestamp - ts), default=None)
                if closest_event is not None:
                    acc = closest_event.attributes["imu_accelerometer"].value if "imu_accelerometer" in closest_event.attributes else initial_acc
                    gyro = closest_event.attributes["imu_gyroscope"].value if "imu_gyroscope" in closest_event.attributes else initial_gyro
                    self.logger.log(f"Bonding IMU history ts:{ts} <==> episodic event ts:{closest_event.timestamp+initial_ts} USING acc {acc} // gyro {gyro} (there were {len(candidates)} candidates).", style="purple")
                    initial_acc = acc
                    initial_gyro = gyro
                    self.imu_history[TIMESTAMP].append(ts * 1e-9) # Convert back to seconds for easier handling
                    self.imu_history[ACCELEROMETER].append(acc)
                    self.imu_history[GYROSCOPE].append(gyro)
                else:
                    acc = initial_acc
                    gyro = initial_gyro
                    self.logger.log(f"No IMU event found in episodic memory for or before timestamp {ts}. Using last known values: acc {acc} and gyro {gyro}.", style="yellow")
             
    
            print(f"Converted {len(imu_events)} imu events to IMU history format ({len(self.imu_history[TIMESTAMP])} frames).")
        else:
            self.logger.log("Episodic Memory API is not ready! WTF?", style="bold red")
            return


    def get_simulation_length_from_episodic_memory(self) -> float | None:
        """Get the length of the simulation from the episodic memory, by looking for the last timestamp of the "imu" node history.
            Returns:
                - float: length of the simulation in seconds.
                - None: if not available.
        """
        if self.mem_api.is_ready():
            imu_events = self.mem_api.get_node_history_by_name("imu")
            if len(imu_events) > 0:
                last_ts = imu_events[-1].timestamp
                initial_ts = imu_events[0].timestamp
                simulation_length = (last_ts - initial_ts) * 1e-9 # Convert from nanoseconds to seconds
                print(f"Simulation length obtained from episodic memory: lts:{last_ts} - its:{initial_ts} = {simulation_length} seconds.")
                return simulation_length
            else:
                print("No IMU events found in episodic memory!")
                return None
        else:
            print("Episodic Memory API is not ready! WTF?")
            return None    


    # DEBUG: Write fake JSON file
    def writeSimulationScene(self) -> None:
       # Fake SIM_SCENE JSON file
       self.logger.log("Writing simulation scene to sim_scene.json...", style="bold purple")
       self.sim_scene.problem_position, self.sim_scene.problem_orientation = PROBLEM_POS, [0,0,0,1]
       self.sim_scene.simulation_length = self.get_simulation_length_from_episodic_memory()
       self.sim_scene.list_of_target_velocities = self.get_robot_adv_speed_history() if self.get_robot_adv_speed_history() is not None else []
       self.sim_scene.num_of_repetitions = 200
       self.sim_scene.bottle_position = BOTTLE_POS # TODO: Constant
       self.sim_scene.bottle_orientation = [0,0,0,0] # TODO: Constant
       self.sim_scene.model_validate(self.sim_scene.__dict__)
       file = open("src/sim_scene.json", "w")
       file.write(self.sim_scene.model_dump_json(indent=4))
       file.close()


    def loadCausesJson(self) -> None:
        """Load the causes JSON file and store it in self.causes_data."""
        try:
            file = open(JSON_FILE, "r")
            causes_data = json.loads(file.read())
            self.causes_data = causes_data
            file.close()
            print(f"> [loadCausesJson] {JSON_FILE} content: \n{causes_data}")
        except Exception as e:
            print(f"> [loadCausesJson] Error while reading from {JSON_FILE}: " + str(e))


    def __del__(self) -> None:
        """Destructor for cleaning up resources."""


    @QtCore.Slot()
    def compute(self) -> bool:
        """Main computation loop that handles state machine and simulation logic.
            Returns:
                - bool: True if computation completed successfully, False otherwise.
        """
        print(flush=True, end="")
        self.show_compute_time_step()
        try:
            match self.state:
        
                case "IDLE":
                    self.wait_for_mission_start()
        
                case "INNER_SIMULATOR":
                    if self.check_for_problems():
                        console.print("Problem detected, trying to find a solution...", style="bold red")
                        self.state = "SIMULATE_REASON"
                    
                    self.inner_simulator()

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
                        pids.append([subprocess.Popen([str(sys.executable), "src/causes_simulator.py", "-c", json_data, "-s", "src/sim_scene.json", "-p", str(wpipe)], pass_fds=(wpipe,)), rpipe])
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
                        if pid[0] in historicals and historicals[pid[0]] is not None and historicals[pid[0]] != "":
                            historicals[pid[0]] = json.loads(historicals[pid[0]])
                        else:
                            raise ValueError(f"There was no data received from finalized process {pid[0]} or data was empty! Did the process crash?")
        
                    print(f"Received historicals from {len(historicals)} simulations.")
                    i = 0
                    for h in historicals:
                        print(f"    Historical {i} recordings: {len(historicals[h])}")
                        i += 1

                    # Convert episodic memory history to IMU history given one of the historicals ts list as an example.
                    list_of_ts = historicals[pids[0][0]][0][HISTORY][TIMESTAMP]
                    self.convert_episodic_to_imu_history(list_of_ts)
                    print("Historical INNER frames:", len(self.imu_history[TIMESTAMP]))

                    # Graph an example of the real IMU history (accelerometer and gyro)
                    axis_labels = ["X", "Y", "Z"]
                    axis_colors = ["tab:red", "tab:green", "tab:blue"]
                    plt.figure(figsize=(12, 5))
                    plt.suptitle("Real IMU history from Episodic Memory", fontsize=16)
                    # Accelerometer
                    plt.subplot(1, 2, 1)
                    plt.title("Accelerometer")
                    for j, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
                        real_acc = [v[j] for v in self.imu_history[ACCELEROMETER]]
                        plt.plot(self.imu_history[TIMESTAMP], real_acc, color=color, linestyle="-", label=f"Real {axis}")
                    plt.xlabel("Time (s)")
                    plt.ylabel("Acceleration (m/s^2)")
                    plt.legend()
                    # Gyroscope
                    plt.subplot(1, 2, 2)
                    plt.title("Gyroscope")
                    for j, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
                        real_gyro = [v[j] for v in self.imu_history[GYROSCOPE]]
                        plt.plot(self.imu_history[TIMESTAMP], real_gyro, color=color, linestyle="-", label=f"Real {axis}")
                    plt.xlabel("Time (s)")
                    plt.ylabel("Angular Velocity (rad/s)")
                    plt.legend()
                    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
                    plt.show()

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
                    
                    # Show a graph comparing the real IMU history with the best recording of the top for each cause (a graph per cause)
                    axis_labels = ["X", "Y", "Z"]
                    axis_colors = ["tab:red", "tab:green", "tab:blue"]

                    # debug: make imu_history a flat line to easily compare with the simulated ones
                    #self.imu_history[ACCELEROMETER] = [[0,0,0] for _ in self.imu_history[ACCELEROMETER]]
                    #self.imu_history[GYROSCOPE] = [[0,0,0] for _ in self.imu_history[GYROSCOPE]]

                    for i, cause in enumerate(self.causes_data):
                        best_recording = sim_out["registers"][i]["top_five"][0]  # Best recording for this cause

                        plt.figure(figsize=(12, 5))
                        plt.suptitle(f"Comparison of real IMU history with best recording of cause: {cause['name']}", fontsize=16)

                        # Accelerometer
                        plt.subplot(1, 2, 1)
                        plt.title("Accelerometer")
                        for j, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
                            real_acc = [v[j] for v in self.imu_history[ACCELEROMETER]]
                            sim_acc = [v[j] for v in best_recording[HISTORY][ACCELEROMETER]]
                            plt.plot(self.imu_history[TIMESTAMP], real_acc, color=color, linestyle="-", label=f"Real {axis}")
                            plt.plot(best_recording[HISTORY][TIMESTAMP], sim_acc, color=color, linestyle="--", label=f"Sim {axis}")
                        plt.xlabel("Time (s)")
                        plt.ylabel("Acceleration (m/s^2)")
                        plt.legend()

                        # Gyroscope
                        plt.subplot(1, 2, 2)
                        plt.title("Gyroscope")
                        for j, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
                            real_gyro = [v[j] for v in self.imu_history[GYROSCOPE]]
                            sim_gyro = [v[j] for v in best_recording[HISTORY][GYROSCOPE]]
                            plt.plot(self.imu_history[TIMESTAMP], real_gyro, color=color, linestyle="-", label=f"Real {axis}")
                            plt.plot(best_recording[HISTORY][TIMESTAMP], sim_gyro, color=color, linestyle="--", label=f"Sim {axis}")
                        plt.xlabel("Time (s)")
                        plt.ylabel("Angular Velocity (rad/s)")
                        plt.legend()

                        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
                        plt.show()

                    output = open(f"sim_output.json", "w")
                    output.write(json.dumps(sim_out, indent=4))
                    output.close()
                    
                    print("Simulations finished. Results written to sim_output.json!")
                    
                    
                    # Create agent template (CDSL) for each cause
                    for cause in self.causes_data:
                        if not generate_agent(cause["name"], AGENTS_FOLDER):
                            print("Error while generating agent template for cause", cause["name"])
                        
                    print("Agents templated generated at folder", AGENTS_FOLDER)

                    # TODO: Usamos terminated porque ahora mismo la misión de buscar causas no se termina.
                    self.state = "TERMINATED"      

                case "TERMINATED":
                    pass
                
        except Exception as e:            
            self.logger.log(f"Fatal exception occurred: {e} at line {sys.exc_info()[-1].tb_lineno}", style="bold red")
            # print traceback
            import traceback
            traceback_str = traceback.format_exc()
            self.logger.log(traceback_str, style="red")
            exit(1)
            
        return True
    
        

    def startup_check(self) -> None:
        """Perform startup checks and quit the application after a short delay."""
        

    # =============== SIMULATION HELPERS  ================
    # ====================================================

    def show_compute_time_step(self) -> float:
        """Get the time step between compute calls.
            Returns:
                - float: Time elapsed since last call in seconds.
        """
        time_step = time.time() - self.actual_time
        self.actual_time = time.time()
        if time.time() - self.print_time > 5:
            self.print_time = time.time()
            self.logger.log(f"Compute frequency: {1/time_step:.2f} Hz", style="bold blue")
            
        return time_step
    

    def inner_simulator(self):
        """
        Inner simulator method. It try to simulate the real robot moving with the same target speed that is published in graph.
        """

        actual_imu_measurement = self.imu.get_measurement()

        if (not np.allclose(self.last_imu_measurement[0], actual_imu_measurement[0], atol=0.001) 
            and 
            not np.allclose(self.last_imu_measurement[1], actual_imu_measurement[1], atol=0.001)):
            self.publish_imu_to_dsr(actual_imu_measurement[0], actual_imu_measurement[1])

        self.forward_vel, self.angular_vel = self.get_velocities_from_dsr()
        wheels_velocity = self.get_wheels_velocity_from_forward_velocity_and_angular_velocity(self.forward_vel, self.angular_vel)
        for motor_name in self.motors:
            p.setJointMotorControl2(bodyUniqueId=self.robot,
                                    jointIndex=self.joints_name[motor_name],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=wheels_velocity[motor_name],
                                    force=10)
            
        p.stepSimulation()


    
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

    def wait_for_mission_start(self):
        follow_person_node = self.graphs["work"].get_node("follow_me")
        if follow_person_node is not None and follow_person_node.attrs["aff_interacting"].value == True:
            console.print("Mission start detected! Transitioning to INNER_SIMULATOR state...", style="bold green")
            self.state = "INNER_SIMULATOR"

    def check_for_problems(self) -> bool:
        """Checks for problems in the episodic graph and initializes simulation if needed.
        Returns True if a problem was detected and the state should transition to SIMULATE_REASON.
        """
        spc_node = None
        fp_node = None
        for node in self.graphs["episodic"].get_nodes():
            if node.name.startswith("Search Problem Cause"):  # TODO: Should be a better way to identify the correct node.
                spc_node = node
            if node.name.startswith("Follow Person"):  # TODO: Should be a better way to identify the correct node.
                fp_node = node

        if (spc_node is not None 
                and "status" in spc_node.attrs 
                and spc_node.attrs["status"].value == "running" 
                and fp_node is not None 
                and "filepath" in fp_node.attrs 
                and fp_node.attrs["filepath"].value is not None):
            
            console.print(
                "Search Problem Cause node found with status 'running' and Follow Person node found with non-empty filepath.",
                style="bold green"
            )
            self.actual_time = time.time()
            console.print("MemoryAPI will be initialized with filepath: " + fp_node.attrs["filepath"].value, style="green")
            self.mem_api = mem.EpisodicMemoryAPI(fp_node.attrs["filepath"].value)

            # Wait for mem_api.is_ready() to be True, with a timeout of 10 seconds
            timeout = 10
            start_time = time.time()
            while not self.mem_api.is_ready():
                if time.time() - start_time > timeout:
                    console.print(
                        f"Timeout of {timeout} seconds reached while waiting for Episodic Memory API to be ready!",
                        style="bold red"
                    )
                    break
                time.sleep(0.1)

            self.writeSimulationScene()
            return True

        return False

    def get_velocities_from_dsr(self):
        """
        Get the forward and angular velocities from the DSR graph

        :return: Forward and angular velocities
        """
        forward_velocity = 0
        angular_velocity = 0

        robot_node = self.graphs["work"].get_node("robot")
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

        robot_node = self.graphs["work"].get_node("robot")
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

        self.graphs["work"].insert_node(imu_node)


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
        self.graphs["work"].update_node(imu_node)

    
    def create_edge_in_dsr(self, fr_node, to_node, edge_type):
        """
        Create an edge in the DSR graph

        :param fr_node: From node
        :param to_node: To node
        :param edge_type: Type of the edge
        """
        edge = Edge(to_node.id, fr_node.id, edge_type, self.agent_id)
        self.graphs["work"].insert_or_assign_edge(edge)


    def publish_imu_to_dsr(self, acc_measured, angular_vel):
        """

        Publish the IMU measurements to the DSR graph or create the node if it does not exist

        :param acc_measured: Measured acceleration
        :param angular_vel: Angular velocity
        """
        
        imu_node = self.graphs["work"].get_node("imu_sintetic")
        if imu_node is not None:
            self.update_imu_node_in_dsr(imu_node, acc_measured, angular_vel)
        else:
            self.create_imu_node_in_dsr(acc_measured, angular_vel)
            self.create_edge_in_dsr(self.graphs["work"].get_node("robot"), self.graphs["work"].get_node("imu_sintetic"), "has")



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


    # ===============    CHILDS PART    ================
    # ==================================================
        

    @staticmethod
    def find_matching_imu_recordings(rimu: dict, simu: list) -> list[tuple]:
        """Find matches between real IMU recordings and simulated ones. If there is a match, it means that the cause being simulated could be the reason behind the problem detected in the real robot.
            Parameters:
                - rimu (dict): Dictionary with real IMU recordings containing keys "timestamp", "accelerometer" and "gyroscope".
                - simu (list): List of dictionaries with simulated IMU recordings.
            Returns:
                - list[tuple]: Top 5 best matches as tuples of (simulation_id, score), sorted by score (lowest first).
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
        logger = Logger(f"logs/specific_worker/thread{threading.current_thread().ident}_{time.strftime('%Y%m%d_%H%M%S')}.log")

        # Prepare score list
        scores = {}
        for s in range(len(simu)):
            scores[s] = 0

        # for each simu simulation...
        for s in range(len(simu)):
            logger.log(f"Now matching {len(simu[s][HISTORY][TIMESTAMP])} simu (id={s}) frames against {len(rimu[TIMESTAMP])} rimu frames.")
            # for each frame of the simulation...
            for i in range(len(simu[s][HISTORY][TIMESTAMP])):

                # Find closest timestamp value of simu to rimu's timestamp
                ts = min(rimu[TIMESTAMP], key=lambda v: abs(v - simu[s][HISTORY][TIMESTAMP][i]))
                frame_id = rimu[TIMESTAMP].index(ts)

                diff_acc = np.linalg.norm(np.array(simu[s][HISTORY][ACCELEROMETER][i]) - np.array(rimu[ACCELEROMETER][frame_id]))
                diff_gyro = np.linalg.norm(np.array(simu[s][HISTORY][GYROSCOPE][i]) - np.array(rimu[GYROSCOPE][frame_id]))
                total_diff = (diff_acc + diff_gyro) / 2

                logger.log(f"Bonded simu frame {i} (ts={simu[s][HISTORY][TIMESTAMP][i]}) to rimu frame {frame_id} (ts={ts}) with acc diff {diff_acc:.2f} and gyro diff {diff_gyro:.2f} (total diff: {total_diff:.2f}), earning score: {total_diff:.2f}", style="dim")

                scores[s] += total_diff # Update score of this simu frame

            logger.log(f"Finished matching simu (id={s}) frames. Total score was: {scores[s]:.2f}", style="bold green")

        logger.log(f"Finished matching all simu IDs. Now sorting {len(scores)} entries (lowest first).")
        sorted_scores = sorted(scores.items(), key=lambda item: item[1])
        return sorted_scores[:5] # Return the top 5 best matches as (simulation_id, score)