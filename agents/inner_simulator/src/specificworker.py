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

# Positions of the bodies in the scene (millimeters)
ROBOT_POS = [-3700.0, -300.0, 32.5]
PROBLEM_POS = [0.0, 40.0, 1.0]
BOTTLE_POS = [0.0, 50.0, 795.0]

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
from dtaidistance import dtw

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

# Get the path to 'agents' (one level up from 'inner_simulator') to reach the
# shared agent_generation package (agent scaffolding + templates + training).
agents_root = os.path.dirname(agent_root)
if agents_root not in sys.path:
    sys.path.insert(0, agents_root)

from src.simulation_scene import SimulationScene
from src.logger import Logger

import pybullet as p
import json
import episodic_memory_api as mem
import numpy as np
import locale

from concurrent.futures import ProcessPoolExecutor
from agent_generation.agent_generator import *

from pybullet_imu import IMU
from pydsr import *


# ===================== CAUSE-SELECTION PIPELINE (Phase 1) =====================
# Config for the gates / ranking / confidence. Per-cause "t_abs" can be overridden
# from causes.json (field "t_abs"). See docstring of select_best_cause().
SELECTION_CFG = {
    "t_abs_default": 0.6,     # normalized-DTW ceiling for "a known cause explains it"
    "t_keep": 1.5,            # coarse per-recording prune (looser than t_abs)
    "top_k": 3,               # robust representative score = mean of k lowest survivors
    "k_margin": 1.0,          # runner_up/winner ratio that counts as full margin confidence
    "scale_abs": 0.15,        # steepness of c_abs sigmoid
    "frac_ref": 0.05,         # frac_good expected when a (spatial) cause is right
    "w_disp_m": 1.0,          # bottle-disturbance weight: displacement (meters)
    "w_tilt_rad": 1.0,        # bottle-disturbance weight: tilt (radians)
    "d_max_traj_m": 1.0,      # max distance (m) from a grid cell to the robot trajectory
    "enforce_position_gate": False,  # hard-reject grid cells far from the robot path
    "enforce_verdict": False,        # withhold problem_position / cause_confirmed on "unknown"
    "c_floor": 0.0,                  # inner's own min confidence to bother confirming (enforce mode)
}


def _znorm(arr: np.ndarray) -> np.ndarray:
    """Z-normalize each column (axis) independently: (x - mean) / std, std guarded."""
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] < 2:
        return arr
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)
    std = np.where(std < 1e-9, 1.0, std)
    return (arr - mean) / std


def _axis_dtw_mean(a: np.ndarray, b: np.ndarray) -> float:
    """Mean over the 3 axes of the DTW distance, normalized by warping-path length so
    the value is a per-step average (comparable across runs of different duration)."""
    n = float(max(len(a), len(b))) or 1.0
    vals = []
    for axis in range(3):
        d = dtw.distance_fast(np.ascontiguousarray(a[:, axis]),
                              np.ascontiguousarray(b[:, axis]))
        vals.append(d / n)
    return float(np.mean(vals))


def score_all_recordings_worker(rimu: dict, simu: list) -> list:
    """Picklable pool worker. Returns the normalized DTW score of EVERY recording,
    indexed by recording id. Non-finite / malformed recordings get float('inf')."""
    r_acc = _znorm(np.array(rimu[ACCELEROMETER], dtype=np.float64))
    r_gyro = _znorm(np.array(rimu[GYROSCOPE], dtype=np.float64))
    scores = []
    for sim in simu:
        try:
            s_acc = np.array(sim[HISTORY][ACCELEROMETER], dtype=np.float64)
            s_gyro = np.array(sim[HISTORY][GYROSCOPE], dtype=np.float64)
            if s_acc.ndim != 2 or s_acc.shape[1] != 3 or s_gyro.shape[1] != 3 or len(s_acc) < 2:
                scores.append(float("inf"))
                continue
            score = 0.5 * (_axis_dtw_mean(r_acc, _znorm(s_acc)) + _axis_dtw_mean(r_gyro, _znorm(s_gyro)))
            scores.append(score if np.isfinite(score) else float("inf"))
        except Exception:
            scores.append(float("inf"))
    return scores


def _quat_tilt_rad(quat) -> float:
    """Angle (rad) between the body's local +Z axis and world +Z, from an [x,y,z,w] quaternion.
    0 = upright, pi/2 = lying on its side."""
    try:
        x, y, z, w = [float(v) for v in quat]
    except Exception:
        return 0.0
    # world-Z component of the rotated local-Z axis (rotation matrix element R[2,2])
    zz = 1.0 - 2.0 * (x * x + y * y)
    zz = max(-1.0, min(1.0, zz))
    return float(np.arccos(zz))


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
                                     cameraTargetPosition=[0.8, -0.9, 0.2])
        
        self.dt = 1.0 / 240.0  # Simulation time step    
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
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", [0, 0.0, 0.04], flags=flags)

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


    def set_simulation_scene(self,
                             simulation_length, 
                             list_of_target_velocities, 
                             num_of_repetitions=200) -> None:
        """
            Set the simulation scene as the real one when the problem was detected, 
            with the same target velocities that the robot had in the real world, 
            and other parameters like the bottle position.

            Args:
                simulation_length (float): The length of the simulation in seconds.
                list_of_target_velocities (list[tuple[float, float]]): A list of tuples with the forward and angular velocities of the robot at each timestamp in the real world.
                num_of_repetitions (int): The number of times that each cause will be simulated to check for consistency in the results.
        """
        robot_positions = self.get_robot_positions_relative_to_problem()
        if robot_positions is not None and robot_positions.get("last_position_before_problem") is not None:
            problem_position_fixed = [robot_positions["last_position_before_problem"][1], robot_positions["last_position_before_problem"][0], robot_positions["last_position_before_problem"][2]]
            self.sim_scene.problem_position = problem_position_fixed
            self.logger.log(f"Problem position set for simulation: {problem_position_fixed}", style="green")
        else:
            self.sim_scene.problem_position = PROBLEM_POS
        self.sim_scene.problem_orientation = [0,0,0,1]
        
        if robot_positions is not None and robot_positions.get("first_position") is not None:
            robot_position_fixed = [robot_positions["first_position"][1], robot_positions["first_position"][0], robot_positions["first_position"][2]]
            self.sim_scene.initial_robot_position = robot_position_fixed
            self.logger.log(f"Initial robot position set for simulation: {robot_position_fixed}", style="green")
        else:
            self.logger.log("Could not read initial robot position from episodic memory, using default ROBOT_POS", style="yellow")
            self.sim_scene.initial_robot_position = ROBOT_POS
        
        self.sim_scene.initial_robot_orientation = [0,0,0,1]
        self.sim_scene.simulation_length = simulation_length
        self.sim_scene.list_of_target_velocities = list_of_target_velocities
        self.sim_scene.num_of_repetitions = num_of_repetitions
        self.sim_scene.bottle_position = BOTTLE_POS
        self.sim_scene.bottle_orientation = [0,0,0,0]
        self.sim_scene.model_validate(self.sim_scene.__dict__)
        file = open("src/sim_scene.json", "w")
        file.write(self.sim_scene.model_dump_json(indent=4))
        file.close()


    # DEBUG: Write fake JSON file
    def writeSimulationScene(self) -> None:
       # Fake SIM_SCENE JSON file
       self.logger.log("Writing simulation scene to sim_scene.json...", style="bold purple")
       
       robot_positions = self.get_robot_positions_relative_to_problem()
       if robot_positions is not None and robot_positions.get("last_position_before_problem") is not None:
           problem_position_fixed = [robot_positions["last_position_before_problem"][1], robot_positions["last_position_before_problem"][0], robot_positions["last_position_before_problem"][2]]
           self.sim_scene.problem_position = problem_position_fixed
       else:
           self.sim_scene.problem_position = PROBLEM_POS
       
       self.sim_scene.problem_orientation = [0,0,0,1]
       
       if robot_positions is not None and robot_positions.get("first_position") is not None:
           robot_position_fixed = [robot_positions["first_position"][1], robot_positions["first_position"][0], robot_positions["first_position"][2]]
           self.sim_scene.initial_robot_position = robot_position_fixed
       else:
           self.sim_scene.initial_robot_position = ROBOT_POS
       
       self.sim_scene.initial_robot_orientation = [0,0,0,1]
       self.sim_scene.simulation_length = self.get_simulation_length_from_episodic_memory()
       self.sim_scene.list_of_target_velocities = self.get_robot_adv_speed_history() if self.get_robot_adv_speed_history() is not None else []
       self.sim_scene.num_of_repetitions = 1
       self.sim_scene.bottle_position = BOTTLE_POS
       self.sim_scene.bottle_orientation = [0,0,0,0]
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
            self.logger.log(f"> [loadCausesJson] {JSON_FILE} content: \n{causes_data}", style="bold blue")
        except Exception as e:
            self.logger.log(f"> [loadCausesJson] Error while reading from {JSON_FILE}: " + str(e), style="bold blue")


    def __del__(self) -> None:
        """Destructor for cleaning up resources."""


    @QtCore.Slot()
    def compute(self) -> bool:
        """Main computation loop that handles state machine and simulation logic.
            Returns:
                - bool: True if computation completed successfully, False otherwise.
        """
        self.show_compute_time_step()
        try:
            match self.state:
        
                case "IDLE":
                    self.wait_for_mission_start()
        
                case "INNER_SIMULATOR":
                    if self.check_for_problems():
                        self.logger.log("Problem detected, trying to find a solution...", style="bold red")
                        self.state = "SIMULATE_REASON"
                    
                    self.inner_simulator()

                case "SIMULATE_REASON":

                    self.set_simulation_scene(self.get_simulation_length_from_episodic_memory(), self.get_robot_adv_speed_history(), num_of_repetitions=200)

                    # return True # We return here to let the simulation run for a while and the robot to reach the problem position, so we can get a more accurate history from the episodic memory. We will return to SIMULATE_REASON state in the next compute calls.

                    # Read JSON to get causes
                    self.loadCausesJson()

                    # Launch subprocesses for simulation
                    self.logger.log("Launching subprocesses...", style="bold blue")                
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
                    self.logger.log("Waiting for subprocesses...", style="blue")
                    while (True):
                        # pid[0] is the Popen class itself, and pid[1] is the (read) pipe which the process uses to communicate with inner simulator
                        for pid in pids:
                            if pid[0] not in historicals:
                                pipe = os.fdopen(pid[1])
                                self.logger.log(f"Now reading pipe of process {pid[0]}...", style="blue")
                                historicals[pid[0]] = pipe.read()
                                pipe.close()
                                self.logger.log(f"Pipe for {pid[0]} has been read!")
                                
                        if len(historicals) == len(pids): break

                    # Transform pipe data from JSON string to dictionary
                    for pid in pids:
                        if pid[0] in historicals and historicals[pid[0]] is not None and historicals[pid[0]] != "":
                            historicals[pid[0]] = json.loads(historicals[pid[0]])
                        else:
                            raise ValueError(f"There was no data received from finalized process {pid[0]} or data was empty! Did the process crash?")
        
                    self.logger.log(f"Received historicals from {len(historicals)} simulations.", style="bold blue")
                    i = 0
                    for h in historicals:
                        self.logger.log(f"    Historical {i} recordings: {len(historicals[h])}", style=" blue")
                        i += 1

                    # Convert episodic memory history to IMU history given one of the historicals ts list as an example.
                    list_of_ts = historicals[pids[0][0]][0][HISTORY][TIMESTAMP]
                    self.convert_episodic_to_imu_history(list_of_ts)
                    print("Historical INNER frames:", len(self.imu_history[TIMESTAMP]))

                    # Graph an example of the real IMU history (accelerometer and gyro)
                    axis_labels = ["X", "Y", "Z"]
                    axis_colors = ["tab:red", "tab:blue", "tab:blue"]
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

                    # ---- score EVERY recording of EVERY cause (normalized DTW), in parallel ----
                    cause_defs = list(self.causes_data)
                    recordings_by_cause = [historicals[pids[i][0]] for i in range(len(pids))]
                    with ProcessPoolExecutor(max_workers=2) as executor:
                        score_futures = [executor.submit(score_all_recordings_worker, self.imu_history, recs)
                                         for recs in recordings_by_cause]
                        all_scores = [f.result() for f in score_futures]

                    sim_out = {"sim_scene": self.sim_scene.model_dump(), "registers": []}
                    causes_for_selection = []
                    for i, recs in enumerate(recordings_by_cause):
                        scores = all_scores[i]
                        order = sorted(range(len(scores)), key=lambda r: scores[r])
                        top_ids = order[:5]
                        top_five = [recs[r] for r in top_ids]
                        print(f"Top 5 recordings for cause {cause_defs[i]['name']}: "
                              + ", ".join(f"{r}({scores[r]:.4f})" for r in top_ids))
                        sim_out["registers"].append({
                            "cause_definition": cause_defs[i],
                            "scores": scores,                       # normalized DTW per recording id
                            "top": [[r, scores[r]] for r in top_ids],
                            "top_five": top_five,
                        })
                        causes_for_selection.append({"cause_definition": cause_defs[i],
                                                     "recordings": recs, "scores": scores})

                    # ---- gates -> ranking -> confidence -> location ----
                    selection = self.select_best_cause(causes_for_selection, SELECTION_CFG)
                    sim_out["selection"] = selection
                    self.logger.log(
                        "[selection] "
                        + json.dumps({k: v for k, v in selection.items() if k != "per_cause"},
                                     default=str),
                        style="bold magenta")

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
                    axis_colors = ["tab:red", "tab:blue", "tab:blue"]

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

                    # Write the full result to a timestamped file (+ refresh stable sim_output.json),
                    # fully flushed before we touch the DSR so semantic never opens a half file.
                    sim_output_path = self._write_sim_output_file(sim_out)
                    self.logger.log(f"Simulations finished. Results -> {sim_output_path}", style="bold blue")

                    # Persist onto 'problem': always sim_output_path (semantic's trigger; verdict +
                    # detail live in the file). Phase-1 back-compat: also problem_position +
                    # cause_confirmed unless enforce_verdict withholds them on an "unknown" verdict.
                    self.persist_selection_in_dsr(selection, sim_output_path)

                    self.state = "IDLE"

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
            # self.logger.log(f"Compute frequency: {1/time_step:.2f} Hz", style="bold blue")
            
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
            self.logger.log("Mission start detected! Transitioning to INNER_SIMULATOR state...", style="bold blue")
            self.state = "INNER_SIMULATOR"

    def check_for_problems(self) -> bool:
        """Checks for problems in the episodic graph and initializes simulation if needed.
        Returns True if a problem was detected and the state should transition to SIMULATE_REASON.
        """
        spc_node = None
        fp_node = None
        for node in self.graphs["episodic"].get_nodes():
            # mission_controller names episodic mission nodes after the mission's
            # customName ("search_cause_attempt_N" / "follow_person_attempt_N"),
            # not its type string - match on that instead.
            if node.name.startswith("search_cause_attempt"):  # TODO: Should be a better way to identify the correct node.
                spc_node = node
            if node.name.startswith("follow_person_attempt"):  # TODO: Should be a better way to identify the correct node.
                fp_node = node

        if (spc_node is not None 
                and "status" in spc_node.attrs 
                and spc_node.attrs["status"].value == "running" 
                and fp_node is not None 
                and "filepath" in fp_node.attrs 
                and fp_node.attrs["filepath"].value is not None):
            
            self.logger.log(
                "Search Problem Cause node found with status 'running' and Follow Person node found with non-empty filepath.",
                style="bold blue"
            )
            self.actual_time = time.time()
            self.logger.log("MemoryAPI will be initialized with filepath: " + fp_node.attrs["filepath"].value, style="blue")
            self.mem_api = mem.EpisodicMemoryAPI(fp_node.attrs["filepath"].value)

            # Wait for mem_api.is_ready() to be True, with a timeout of 10 seconds
            timeout = 10
            start_time = time.time()
            while not self.mem_api.is_ready():
                if time.time() - start_time > timeout:
                    self.logger.log(
                        f"Timeout of {timeout} seconds reached while waiting for Episodic Memory API to be ready!",
                        style="bold red"
                    )
                    break
                time.sleep(0.1)

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
            self.logger.log("Robot node not found in DSR graph", style="bold red")
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


    def update_problem_position_in_dsr(self, best_recording: dict, position_index: int) -> None:
        """
        Persist the estimated 3D position of the detected problem (e.g. a bump) onto
        the 'problem' node, using the winning grid cell from the best-matching simulation.

        :param best_recording: The best-matching simulation recording (includes 'generated_instances').
        :param position_index: Id/repetition index of the best-matching simulation, which
                                doubles as the index into 'distributed_positions'.
        """
        distributed_positions = best_recording.get("generated_instances", {}).get("distributed_positions")
        if not distributed_positions or position_index >= len(distributed_positions):
            return  # This cause has no spatial grid (e.g. "wheel"); nothing to store.

        position = distributed_positions[position_index]

        problem_node = self.graphs["work"].get_node("problem")
        if problem_node is None:
            self.logger.log("'problem' node not found in DSR graph, cannot store problem_position.", style="bold red")
            return

        problem_node.attrs["problem_position"] = Attribute(list(position), self.agent_id)
        self.graphs["work"].update_node(problem_node)
        self.logger.log(f"Stored problem_position {position} (mm) on 'problem' node.", style="bold green")


    def mark_cause_confirmed_in_dsr(self) -> None:
        """
        Flag on 'problem' that the causal search concluded, for "semantic" to react to.
        """
        problem_node = self.graphs["work"].get_node("problem")
        if problem_node is None:
            self.logger.log("'problem' node not found in DSR graph, cannot set cause_confirmed.", style="bold red")
            return

        problem_node.attrs["cause_confirmed"] = Attribute(True, self.agent_id)
        self.graphs["work"].update_node(problem_node)
        self.logger.log("Stored cause_confirmed=True on 'problem' node.", style="bold green")


    # ===================== CAUSE-SELECTION PIPELINE (Phase 1) =====================

    def _recording_disturbance(self, recording: dict) -> float | None:
        """How much the bottle was disturbed in a simulated recording: weighted sum of
        its horizontal displacement (m) from the scene start and its tilt (rad)."""
        bp = recording.get("bottle_position")
        bo = recording.get("bottle_orientation")
        if bp is None or len(bp) < 3:
            return None
        disp_mm = float(np.hypot(float(bp[0]) - BOTTLE_POS[0], float(bp[1]) - BOTTLE_POS[1]))
        tilt = _quat_tilt_rad(bo) if bo is not None else 0.0
        return SELECTION_CFG["w_disp_m"] * (disp_mm / 1000.0) + SELECTION_CFG["w_tilt_rad"] * tilt

    def _real_bottle_disturbance(self) -> float | None:
        """Real bottle disturbance at problem onset (from episodic memory). Not evaluated
        in Phase 1 (manual knocks / weak bump) -> None keeps c_outcome neutral."""
        return None

    def _robot_trajectory(self) -> list | None:
        """Best-effort (x, y) of the robot during follow_person, from episodic 'room->robot'
        RT history. Units follow that edge (concept_robot writes meters). None if missing.
        Only used by the (opt-in, default-off) position-plausibility gate."""
        try:
            room = self.graphs["work"].get_node("room")
            robot = self.graphs["work"].get_node("robot")
            if room is None or robot is None:
                return None
            pts = []
            for pos in self.mem_api.get_edge_history(room.id, robot.id, "RT"):
                if pos.modification_type != "MEA" or "rt_translation" not in pos.attributes:
                    continue
                t = list(pos.attributes["rt_translation"].value)
                if len(t) >= 2:
                    pts.append((float(t[0]), float(t[1])))
            return pts or None
        except Exception as e:
            self.logger.log(f"[selection] robot trajectory unavailable: {e}", style="yellow")
            return None

    def select_best_cause(self, causes: list, cfg: dict) -> dict:
        """Pick the winning cause from the simulation results.

        causes: [{'cause_definition': {...}, 'recordings': [...], 'scores': [float per rec id]}]

        Pipeline:
          1. per-recording gates (validity, coarse score prune, opt-in position gate)
             -> survivors per cause; a cause with 0 survivors is 'excluded'.
          2. per-cause representative score: min (spatial) or mean of the k lowest (non-spatial).
          3. cross-cause ranking = argmin representative score among non-excluded causes.
          4. absolute gate: representative score > t_abs -> verdict 'unknown'.
          5. confidence = c_abs * c_margin * c_consistency * c_outcome  (about the winner only).
          6. location = winning grid cell (spatial winner) or None.
        Per-cause 't_abs' can be overridden in causes.json; else cfg['t_abs_default'].
        """
        traj = self._robot_trajectory()
        per_cause, candidates = {}, {}
        for c in causes:
            name = c["cause_definition"]["name"]
            t_abs = float(c["cause_definition"].get("t_abs", cfg["t_abs_default"]))
            recs, scores = c["recordings"], c["scores"]
            grid = recs[0].get("generated_instances", {}).get("distributed_positions") if recs else None
            spatial = bool(grid)

            survivors = []
            for rid, sc in enumerate(scores):
                if not np.isfinite(sc) or sc > cfg["t_keep"]:          # validity + coarse prune
                    continue
                pos = grid[rid] if (spatial and rid < len(grid)) else None
                d_traj = None
                if pos is not None and traj:
                    d_traj = min(float(np.hypot(pos[0] - tx, pos[1] - ty)) for tx, ty in traj)
                    if cfg["enforce_position_gate"] and d_traj > cfg["d_max_traj_m"]:
                        continue                                       # position-plausibility gate
                survivors.append({
                    "rec_id": rid, "score": float(sc), "d_traj": d_traj,
                    "position": list(pos) if pos is not None else None,
                    "disturbance": self._recording_disturbance(recs[rid] if rid < len(recs) else {}),
                })

            finite = sorted(s["score"] for s in survivors)
            k = cfg["top_k"]
            top = finite[:k]
            s_repr = (finite[0] if spatial else float(np.mean(top))) if finite else float("inf")
            spread = float(np.std(top) / np.mean(top)) if len(top) >= 2 and np.mean(top) > 0 else 0.0
            frac_good = (sum(1 for x in scores if np.isfinite(x) and x < t_abs) / len(scores)) if scores else 0.0
            per_cause[name] = {
                "spatial": spatial, "n": len(scores), "n_survivors": len(survivors),
                "s_best": finite[0] if finite else float("inf"), "s_repr": s_repr,
                "spread": spread, "frac_good": frac_good, "t_abs": t_abs,
                "excluded": len(survivors) == 0,
            }
            candidates[name] = survivors

        ranked = sorted(((n, m["s_repr"]) for n, m in per_cause.items() if not m["excluded"]),
                        key=lambda kv: kv[1])

        # fallback location = best grid cell of the best-ranked spatial cause, regardless of verdict
        fb_loc, fb_src = None, None
        for n, _ in ranked:
            with_pos = [s for s in candidates[n] if s["position"] is not None]
            if with_pos:
                fb_loc = min(with_pos, key=lambda s: s["score"])["position"]
                fb_src = "grid_cell"
                break

        selection = {
            "cause": "unknown", "score": None, "runner_up": None,
            "confidence": 0.0, "confidence_breakdown": {},
            "location": None, "location_source": None,
            "fallback_location": fb_loc, "fallback_location_source": fb_src,
            "enforce_verdict": bool(cfg["enforce_verdict"]),
            "per_cause": per_cause,
        }
        if not ranked:
            selection["reason"] = "all causes excluded (no surviving recordings)"
            return selection

        winner, s_win = ranked[0]
        runner = ranked[1] if len(ranked) > 1 else None
        selection["score"] = s_win
        selection["runner_up"] = list(runner) if runner else None
        wm = per_cause[winner]

        if s_win > wm["t_abs"]:
            selection["reason"] = f"best cause '{winner}' s_repr={s_win:.4f} > t_abs={wm['t_abs']:.4f}"
            return selection                                            # verdict stays 'unknown'

        c_abs = float(1.0 / (1.0 + np.exp(-(wm["t_abs"] - s_win) / cfg["scale_abs"])))
        ratio = (runner[1] / s_win) if (runner and s_win > 0) else float("inf")
        c_margin = float(np.clip((ratio - 1.0) / cfg["k_margin"], 0.0, 1.0)) if np.isfinite(ratio) else 1.0
        c_consistency = (float(np.clip(wm["frac_good"] / max(cfg["frac_ref"], 1e-9), 0.0, 1.0))
                         * (1.0 - float(np.clip(wm["spread"], 0.0, 1.0))))
        c_outcome = 1.0
        real_dist = self._real_bottle_disturbance()
        if real_dist is not None and real_dist > 1e-6:
            wsurv = candidates[winner]
            best = min(wsurv, key=lambda s: s["score"]) if wsurv else None
            if best and best["disturbance"] is not None:
                c_outcome = float(np.clip(1.0 - abs(best["disturbance"] - real_dist) / real_dist, 0.0, 1.0))
        confidence = float(c_abs * c_margin * c_consistency * c_outcome)

        with_pos = [s for s in candidates[winner] if s["position"] is not None]
        location = min(with_pos, key=lambda s: s["score"])["position"] if with_pos else None

        selection.update({
            "cause": winner, "confidence": confidence,
            "confidence_breakdown": {"c_abs": c_abs, "c_margin": c_margin,
                                     "c_consistency": c_consistency, "c_outcome": c_outcome},
            "location": list(location) if location is not None else None,
            "location_source": "grid_cell" if location is not None else None,
        })
        return selection

    @staticmethod
    def _json_safe(o):
        """Recursively replace non-finite floats (inf/nan) with None so the dumped file
        is strict, portable JSON for any consumer."""
        if isinstance(o, float):
            return o if np.isfinite(o) else None
        if isinstance(o, dict):
            return {k: SpecificWorker._json_safe(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [SpecificWorker._json_safe(v) for v in o]
        return o

    def _write_sim_output_file(self, sim_out: dict) -> str:
        """Dump sim_out to a timestamped JSON (history) and refresh the stable
        'sim_output.json'. Fully flushed/fsynced before returning. Returns the abs path
        of the timestamped file."""
        payload = json.dumps(
            self._json_safe(sim_out), indent=4, allow_nan=False,
            default=lambda o: (o.item() if hasattr(o, "item") else float(o)))
        path = os.path.abspath(f"sim_output_{time.strftime('%Y%m%d_%H%M%S')}.json")
        for p in (path, os.path.abspath("sim_output.json")):
            with open(p, "w") as f:
                f.write(payload)
                f.flush()
                os.fsync(f.fileno())
        return path

    def persist_selection_in_dsr(self, selection: dict, sim_output_path: str) -> None:
        """Write the causal-search outputs onto the 'problem' node.

        Always: 'sim_output_path' (its update is semantic's trigger; verdict + all detail
        live in that file).
        Phase-1 back-compat (current semantic still reads these): 'problem_position' +
        'cause_confirmed'. Withheld only when cfg['enforce_verdict'] and the verdict is
        'unknown' (or confidence < cfg['c_floor']).
        """
        problem_node = self.graphs["work"].get_node("problem")
        if problem_node is None:
            self.logger.log("'problem' node not found; cannot persist selection.", style="bold red")
            return

        problem_node.attrs["sim_output_path"] = Attribute(str(sim_output_path), self.agent_id)

        cfg = SELECTION_CFG
        withhold = cfg["enforce_verdict"] and (
            selection["cause"] == "unknown" or selection["confidence"] < cfg["c_floor"]
        )
        if not withhold:
            loc = selection.get("location") or selection.get("fallback_location")
            if loc is not None:
                problem_node.attrs["problem_position"] = Attribute([float(v) for v in loc], self.agent_id)
            problem_node.attrs["cause_confirmed"] = Attribute(True, self.agent_id)

        self.graphs["work"].update_node(problem_node)
        self.logger.log(
            f"[selection] cause={selection['cause']} conf={selection['confidence']:.3f} "
            f"loc={selection.get('location') or selection.get('fallback_location')} "
            f"withhold={withhold} sim_output_path={sim_output_path}",
            style="bold green")


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


    # == EPISODIC MEMORY INTERACTION HELPERS  ================
    # =========================================================

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
            
            # # Print statistics of type of events in robot history
            # event_types = {}
            # for event in robot_events:
            #     if event.modification_type not in event_types:
            #         event_types[event.modification_type] = 1
            #     else:
            #         event_types[event.modification_type] += 1
            # self.logger.log(f"Robot events in episodic memory by modification type:", style="bold blue")
            # for mod_type in event_types:
            #     self.logger.log(f"    Type {mod_type}: {event_types[mod_type]} events", style="blue")
                      
            # # Filter by modification type "K" (Keyframe)
            # robot_events = [event for event in robot_events if event.modification_type == "K" and "robot_ref_adv_speed" in event.attributes]
            # self.logger.log(f"Found {len(robot_events)} robot events in episodic memory with modification type K related to robot_ref_adv_speed.", style="bold blue")
            # for event in robot_events:
            #     corrected_ts = event.timestamp - initial_ts
            #     event.timestamp = corrected_ts
            #     if event.attributes.get("robot_ref_adv_speed") is not None:
            #         self.robot_adv_speed_history[TIMESTAMP].append(corrected_ts * 1e-9)
            #         self.robot_adv_speed_history[ADV_SPEED].append(event.attributes["robot_ref_adv_speed"].value)
            #     break
            
            # Filter by modification type "MNA" (Modified Node Attribute)
            robot_events = self.mem_api.get_node_history_by_name("robot")
            robot_events = [event for event in robot_events if event.modification_type == "MNA" and "robot_ref_adv_speed" in event.attributes]
            for event in robot_events:
                corrected_ts = event.timestamp - initial_ts
                event.timestamp = corrected_ts
                if event.attributes.get("robot_ref_adv_speed") is not None:
                    self.robot_adv_speed_history[TIMESTAMP].append(corrected_ts * 1e-9)
                    self.robot_adv_speed_history[ADV_SPEED].append(event.attributes["robot_ref_adv_speed"].value)

            
            self.logger.log(f"Robot speeds loaded from epidodic memory",style="bold blue")
                    
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
            
            self.logger.log(f"Found {len(imu_events)} IMU events in episodic memory with modification type MNA.", style="bold blue")
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
             
    
            os.makedirs("logs/imu_raw", exist_ok=True)
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filepath = f"logs/imu_raw/episodic_imu_{timestamp}.json"
            history_to_save = {
                k: [v.tolist() if isinstance(v, np.ndarray) else v for v in values]
                for k, values in self.imu_history.items()
            }
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(history_to_save, f)
            self.logger.log(f"Episodic IMU history saved to '{filepath}'", style="bold cyan")


            self.logger.log(f"Converted {len(imu_events)} imu events to IMU history format ({len(self.imu_history[TIMESTAMP])} frames).", style="bold blue")
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
                self.logger.log(f"Simulation length obtained from episodic memory:{simulation_length} seconds.",style="bold blue")
                return simulation_length
            else:
                self.logger.log("No IMU events found in episodic memory!",style="bold red")

                return None
        else:
            self.logger.log("Episodic Memory API is not ready! WTF?", style="bold red")
            return None   
        
    def get_robot_positions_relative_to_problem(self) -> dict[str, list[float]] | None:
        """
            Get robot position data from episodic memory.
            Returns both:
                - the first robot position recorded in the history,
                - the last robot position before the first problget_robot_positions_relative_to_problemem appearance.
        """
        if self.mem_api.is_ready():
            room_node = self.graphs["work"].get_node("room")
            robot_node = self.graphs["work"].get_node("robot")
            if room_node is None or robot_node is None:
                self.logger.log("Room or robot node not found in work graph!", style="bold red")
                return None

            robot_positions = [pos for pos in self.mem_api.get_edge_history(room_node.id, robot_node.id, "RT") if pos.modification_type == "MEA"]
            if not robot_positions:
                self.logger.log("No robot position measurements found in episodic memory!", style="bold red")
                return None

            first_position_data = robot_positions[0]
            while "rt_translation" not in first_position_data.attributes and robot_positions:
                robot_positions.pop(0)
                if robot_positions:
                    first_position_data = robot_positions[0]
                else:
                    self.logger.log("No robot position measurements with 'rt_translation' found in episodic memory!", style="bold red")
                    return None
                
            first_position = list(first_position_data.attributes["rt_translation"].value)
            self.logger.log(f"[DEBUG] RAW first rt_translation (room->robot) before conversion: {first_position}", style="bold yellow")


            problem_events = self.mem_api.get_node_history_by_name("problem")
            if not problem_events:
                self.logger.log("No problem events found in episodic memory!", style="bold red")
                return None

            first_problem_appereance_ts = problem_events[0].timestamp
            positions_before_problem = [pos for pos in robot_positions if pos.timestamp <= first_problem_appereance_ts]
            if not positions_before_problem:
                self.logger.log("No robot positions found before first problem appearance!", style="bold red")
                return None

            last_position_before_problem_data = positions_before_problem[-1]
            last_position_before_problem = list(last_position_before_problem_data.attributes["rt_translation"].value)
            self.logger.log(f"[DEBUG] RAW last rt_translation (room->robot) before conversion: {last_position_before_problem}", style="bold yellow")

            self.logger.log("Robot positions loaded from episodic memory", style="bold blue")
            return {
                "first_position": first_position,
                "last_position_before_problem": last_position_before_problem,
            }
        else:
            self.logger.log("Episodic Memory API is not ready!", style="bold red")
            return None

    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        if self.print_dsr_signals:
            self.logger.log(f"UPDATE NODE ATT: {id} {attribute_names}", style='blue')

    def update_node(self, id: int, type: str):
        if self.print_dsr_signals:
            self.logger.log(f"UPDATE NODE: {id} {type}", style='blue')

    def delete_node(self, id: int):
        if self.print_dsr_signals:
            self.logger.log(f"DELETE NODE:: {id} ", style='blue')

    def update_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            self.logger.log(f"UPDATE EDGE: {fr} to {type}", type, style='blue')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        if self.print_dsr_signals:
            self.logger.log(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='blue')

    def delete_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            self.logger.log(f"DELETE EDGE: {fr} to {type} {type}", style='blue')


    # ===============    CHILDS PART    ================
    # ==================================================
        

    # @staticmethod
    # def find_matching_imu_recordings(rimu: dict, simu: list) -> list[tuple]:
    #     """Find matches between real IMU recordings and simulated ones. If there is a match, it means that the cause being simulated could be the reason behind the problem detected in the real robot.
    #         Parameters:
    #             - rimu (dict): Dictionary with real IMU recordings containing keys "timestamp", "accelerometer" and "gyroscope".
    #             - simu (list): List of dictionaries with simulated IMU recordings.
    #         Returns:
    #             - list[tuple]: Top 5 best matches as tuples of (simulation_id, score), sorted by score (lowest first).
    #     """
        
    #     # RIMU structure:
    #     # rimu = {
    #     #     "timestamp": [ts1, ts2, ts3, ...],
    #     #     "accelerometer": [[acc_x1, acc_y1, acc_z1
    #     #                      [acc_x2, acc_y2, acc_z2],
    #     #                      [acc_x3, acc_y3, acc_z3],
    #     #                      ...],
    #     #     "gyroscope": [[gyro_x1, gyro_y1, gyro_z1
    #     #                    [gyro_x2, gyro_y2, gyro_z2],
    #     #                    [gyro_x3, gyro_y3, gyro_z3],
    #     #                    ...]
    #     # }
    #     #
    #     # SIMU structure:
    #     # simu = [
    #     #     {
    #     #         "history": {
    #     #             "timestamp": [ts1, ts2, ts3, ...],
    #     #             "accelerometer": [[acc_x1, acc_y1, acc_z1
    #     #                              [acc_x2, acc_y2, acc_z2],
    #     #                              [acc_x3, acc_y3, acc_z3],
    #     #                              ...],
    #     #             "gyroscope": [[gyro_x1, gyro_y1, gyro_z1
    #     #                            [gyro_x2, gyro_y2, gyro_z2],
    #     #                            [gyro_x3, gyro_y3, gyro_z3],
    #     #                            ...]
    #     #         },
    #     #
    #     #         "generated_instances": {
    #     #             "generatorA": { value1, value2, ...},
    #     #             "generatorB": { value1, value2, ...},
    #     #             ...
    #     #         }
    #     #
    #     logger = Logger(f"logs/specific_worker/thread{threading.current_thread().ident}_{time.strftime('%Y%m%d_%H%M%S')}.log")

    #     # Prepare score list
    #     scores = {}
    #     for s in range(len(simu)):
    #         scores[s] = 0

    #     # for each simu simulation...
    #     for s in range(len(simu)):
    #         logger.log(f"Now matching {len(simu[s][HISTORY][TIMESTAMP])} simu (id={s}) frames against {len(rimu[TIMESTAMP])} rimu frames.")
    #         # for each frame of the simulation...
    #         for i in range(len(simu[s][HISTORY][TIMESTAMP])):

    #             # Find closest timestamp value of simu to rimu's timestamp
    #             ts = min(rimu[TIMESTAMP], key=lambda v: abs(v - simu[s][HISTORY][TIMESTAMP][i]))
    #             frame_id = rimu[TIMESTAMP].index(ts)

    #             diff_acc = np.linalg.norm(np.array(simu[s][HISTORY][ACCELEROMETER][i]) - np.array(rimu[ACCELEROMETER][frame_id]))
    #             diff_gyro = np.linalg.norm(np.array(simu[s][HISTORY][GYROSCOPE][i]) - np.array(rimu[GYROSCOPE][frame_id]))
    #             total_diff = (diff_acc + diff_gyro) / 2

    #             logger.log(f"Bonded simu frame {i} (ts={simu[s][HISTORY][TIMESTAMP][i]}) to rimu frame {frame_id} (ts={ts}) with acc diff {diff_acc:.2f} and gyro diff {diff_gyro:.2f} (total diff: {total_diff:.2f}), earning score: {total_diff:.2f}", style="dim")

    #             scores[s] += total_diff # Update score of this simu frame

    #         logger.log(f"Finished matching simu (id={s}) frames. Total score was: {scores[s]:.2f}", style="bold blue")

    #     logger.log(f"Finished matching all simu IDs. Now sorting {len(scores)} entries (lowest first).")
    #     sorted_scores = sorted(scores.items(), key=lambda item: item[1])
    #     return sorted_scores[:5] # Return the top 5 best matches as (simulation_id, score)

    def extract_signals(self, imu_history: dict) -> tuple[np.ndarray, np.ndarray]:
        """Extract accelerometer and gyroscope as (N, 3) numpy arrays."""
        acc = np.array(imu_history[ACCELEROMETER], dtype=np.float64)   # (N, 3)
        gyro = np.array(imu_history[GYROSCOPE], dtype=np.float64)      # (N, 3)
        return acc, gyro


    def dtw_score(self, a: np.ndarray, b: np.ndarray) -> float:
        """Compute mean DTW distance across the 3 axes between two (N, 3) signals."""
        return np.mean([
            dtw.distance_fast(a[:, axis], b[:, axis])
            for axis in range(3)
        ])


    def find_matching_imu_recordings(self, rimu: dict, simu: list) -> list[tuple]:
        """Find matches between real IMU recordings and simulated ones using DTW.
            Parameters:
                - rimu (dict): Real IMU recordings.
                - simu (list): List of simulated IMU recordings.
            Returns:
                - list[tuple]: Top 5 best matches as (simulation_id, score), sorted by score (lowest first).
        """
        rimu_acc, rimu_gyro = self.extract_signals(rimu)

        print(f"Rimu acc shape: {rimu_acc.shape}, gyro shape: {rimu_gyro.shape}")

        scores = {}
        for s, sim in enumerate(simu):
            sim_acc, sim_gyro = self.extract_signals(sim[HISTORY])

            score_acc  = self.dtw_score(rimu_acc,  sim_acc)
            score_gyro = self.dtw_score(rimu_gyro, sim_gyro)
            scores[s]  = (score_acc + score_gyro) / 2

            self.logger.log(f"Simu {s}: acc_dtw={score_acc:.4f} gyro_dtw={score_gyro:.4f} total={scores[s]:.4f}", style="bold blue")

        self.logger.log(f"Sorting {len(scores)} entries (lowest = best match).")
        sorted_scores = sorted(scores.items(), key=lambda item: item[1])
        return sorted_scores[:5]