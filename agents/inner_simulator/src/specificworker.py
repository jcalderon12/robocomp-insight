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

# DSR contract with the semantic agent (work graph)
UNEXPLAINED_NODE = "unexplained"
HYPOTHESES_FILEPATH_ATTR = "hypotheses_filepath"
VERDICT_FILEPATH_ATTR = "verdict_filepath"

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

MM_TO_M = 0.001
M_TO_MM = 1000.0

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
from src.episode_scene import (
    build_simulation_scene,
    commanded_speed_history,
    episode_length_s,
    real_imu_history,
)
from src.hypothesis_compiler import compile_batch
from src.verdict import build_verdict, write_verdict, save_comparison_plots, save_real_imu_plot

import pybullet as p
import json
import episodic_memory_api as mem
import numpy as np
import locale

from concurrent.futures import ProcessPoolExecutor
from .agent_generator import *

from pybullet_imu import IMU
from pydsr import *


def extract_signals_worker(imu_history: dict) -> tuple[np.ndarray, np.ndarray]:
    acc = np.array(imu_history[ACCELEROMETER], dtype=np.float64)
    gyro = np.array(imu_history[GYROSCOPE], dtype=np.float64)
    return acc, gyro


def dtw_score_worker(a: np.ndarray, b: np.ndarray) -> float:
    return np.mean([dtw.distance_fast(a[:, axis], b[:, axis]) for axis in range(3)])


def find_matching_imu_recordings_worker(rimu: dict, simu: list) -> list[tuple]:
    """Module-level worker function for multiprocessing (picklable).
    Returns top-5 matches as (simulation_id, score).
    """
    rimu_acc, rimu_gyro = extract_signals_worker(rimu)

    print(f"[worker] Rimu acc shape: {rimu_acc.shape}, gyro shape: {rimu_gyro.shape}")

    scores = {}
    for s, sim in enumerate(simu):
        sim_acc, sim_gyro = extract_signals_worker(sim[HISTORY])

        score_acc = dtw_score_worker(rimu_acc, sim_acc)
        score_gyro = dtw_score_worker(rimu_gyro, sim_gyro)
        scores[s] = (score_acc + score_gyro) / 2

        print(f"[worker] Simu {s}: acc_dtw={score_acc:.4f} gyro_dtw={score_gyro:.4f} total={scores[s]:.4f}")

    sorted_scores = sorted(scores.items(), key=lambda item: item[1])
    return sorted_scores[:5]


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

        # Simulation visualization (config: Simulation.Gui / Simulation.RealTime).
        # Off by default so the pipeline stays headless and runs every cause in
        # parallel; enable to watch each cause in its own PyBullet window.
        sim_cfg = configData.get("Simulation", {})
        self.sim_gui = bool(sim_cfg.get("Gui", False))
        self.sim_real_time = bool(sim_cfg.get("RealTime", False))

        # Hypothesis-driven mode state (set when the semantic agent publishes a batch)
        self.hypotheses_compiled = None
        self.hypotheses_path = None
        self.processed_hypotheses_paths = set()
        self.processed_episode_paths = set()
        self.mem_api_path = None
        # When the recording saw the bottle leave the robot (seconds, IMU time base).
        self.observed_effect_time = None

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
        else:
            self.sim_scene.problem_position = PROBLEM_POS
        self.sim_scene.problem_orientation = [0,0,0,1]
        
        if robot_positions is not None and robot_positions.get("first_position") is not None:
            robot_position_fixed = [robot_positions["first_position"][1], robot_positions["first_position"][0], robot_positions["first_position"][2]]
            self.sim_scene.initial_robot_position = robot_position_fixed
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


    def writeSimulationScene(self) -> None:
       # Build the scene from the episodic recording; scene constants remain as fallback.
       self.logger.log("Writing simulation scene to sim_scene.json...", style="bold purple")
       scene, info = build_simulation_scene(self.mem_api, ROBOT_POS, PROBLEM_POS)
       self.logger.log(f"Scene poses reconstructed from episode: {info['pose_sources']}", style="purple")
       if len(info["effect_times"]) > 1:
           self.logger.log(
               f"Bottle detached {len(info['effect_times'])} times at "
               f"{[round(t, 2) for t in info['effect_times']]} s; using the first.", style="yellow")
       self.observed_effect_time = scene["observed_effect_time"]
       self.logger.log(
           f"Observed effect at {self.observed_effect_time} s; simulating {scene['simulation_length']} s "
           f"of a {info['episode_length']} s recording.", style="purple")
       for field_name, value in scene.items():
           setattr(self.sim_scene, field_name, value)
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

    def load_hypotheses_causes(self) -> bool:
        """Compile the hypotheses batch published on the intention node of the work
        graph. False means there is nothing new: fall back to the static causes.json."""
        self.hypotheses_compiled = None
        try:
            unexplained_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
        except Exception as e:
            self.logger.log(f"Cannot read work graph for hypotheses: {e}", style="yellow")
            return False
        if unexplained_node is None or HYPOTHESES_FILEPATH_ATTR not in unexplained_node.attrs:
            return False

        hypotheses_path = unexplained_node.attrs[HYPOTHESES_FILEPATH_ATTR].value
        if not hypotheses_path or hypotheses_path in self.processed_hypotheses_paths:
            return False
        if not os.path.exists(hypotheses_path):
            self.logger.log(f"Hypotheses path '{hypotheses_path}' not readable.", style="yellow")
            return False

        try:
            with open(hypotheses_path, "r") as f:
                batch = json.load(f)
            compiled = compile_batch(batch)
        except Exception as e:
            self.logger.log(f"Failed to compile hypotheses batch '{hypotheses_path}': {e}", style="bold red")
            return False

        self.hypotheses_compiled = compiled
        self.hypotheses_path = hypotheses_path
        self.causes_data = [entry["cause"] for entry in compiled["entries"]]
        self.logger.log(
            f"Hypotheses batch '{compiled.get('case_id', '')}' compiled: "
            f"{len(compiled['entries'])} causes (incl. nominal), {len(compiled['skipped'])} skipped.",
            style="bold green",
        )
        return True

    def publish_verdict_filepath(self, verdict_path: str) -> bool:
        """Expose the verdict path on the intention node so the semantic agent
        can close the loop."""
        try:
            unexplained_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
            if unexplained_node is None:
                self.logger.log("Cannot publish verdict: intention node not found.", style="yellow")
                return False
            unexplained_node.attrs[VERDICT_FILEPATH_ATTR] = Attribute(str(verdict_path), self.agent_id)
            self.graphs["work"].update_node(unexplained_node)
            self.logger.log(f"Published {VERDICT_FILEPATH_ATTR}='{verdict_path}' on intention node.", style="bold green")
            return True
        except Exception as e:
            self.logger.log(f"Cannot publish {VERDICT_FILEPATH_ATTR}: {e}", style="bold red")
            return False

    def retire_batch(self) -> None:
        """Forget the current batch so it is not retried, and re-arm IDLE."""
        if self.hypotheses_path:
            self.processed_hypotheses_paths.add(self.hypotheses_path)
        self.hypotheses_compiled = None
        self.hypotheses_path = None
        self.state = "IDLE"


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
                    # Pending episodes / hypotheses batches are handled even when the live
                    # mirror is not running: the semantic agent may publish its batch after
                    # the follow mission has already stopped.
                    if self.check_for_problems():
                        self.logger.log("Problem detected, trying to find a solution...", style="bold red")
                        self.state = "SIMULATE_REASON"
                    else:
                        self.wait_for_mission_start()

                case "INNER_SIMULATOR":
                    if self.check_for_problems():
                        self.logger.log("Problem detected, trying to find a solution...", style="bold red")
                        self.state = "SIMULATE_REASON"

                    self.inner_simulator()

                case "SIMULATE_REASON":
                    # Causes were already selected in check_for_problems: hypothesis-driven if
                    # available, static causes.json otherwise.
                    if self.hypotheses_compiled is None:
                        self.loadCausesJson()

                    # Launch subprocesses for simulation
                    self.logger.log("Launching subprocesses...", style="bold blue")                
                    pids = []
                    historicals = {}
                    # Watch the simulations in the PyBullet GUI (one window per cause)
                    # when enabled in etc/config; headless otherwise (see __init__).
                    gui_flags = []
                    if self.sim_gui:
                        gui_flags = ["--gui"]
                        if self.sim_real_time:
                            gui_flags.append("--real_time")
                    for cause in self.causes_data:
                        rpipe, wpipe = os.pipe()
                        json_data = {"cause": cause}
                        json_data = json.dumps(json_data)
                        pids.append([subprocess.Popen([str(sys.executable), "src/causes_simulator.py", "-c", json_data, "-s", "src/sim_scene.json", "-p", str(wpipe), *gui_flags], pass_fds=(wpipe,)), rpipe])
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

                    plots_dir = os.path.join(
                        "logs/plots",
                        (self.hypotheses_compiled or {}).get("case_id") or "static_causes",
                    )
                    os.makedirs(plots_dir, exist_ok=True)
                    save_real_imu_plot(self.imu_history, os.path.join(plots_dir, "real_imu_history.png"))

                    if self.hypotheses_compiled is None:
                        # Legacy matching (static causes mode): top-5 DTW ranking into sim_output.json
                        threads = []
                        sim_out = {}
                        sim_out["sim_scene"] = self.sim_scene.model_dump()
                        sim_out["registers"] = []
                        with ProcessPoolExecutor(max_workers=2) as executor:
                            for h in historicals:
                                threads.append(executor.submit(find_matching_imu_recordings_worker, self.imu_history, historicals[h]))
                            i = 0
                            for h in historicals:
                                res = threads[i].result()
                                items = []
                                print("Top 5 best recordings for cause", self.causes_data[i]["name"], ":")
                                for rec in res:
                                    self.logger.log(f"\tRecording {rec[0]} with score {rec[1]}", style="blue")
                                    items.append(historicals[h][rec[0]])
                                row = {"cause_definition": self.causes_data[i], "top_five": items}
                                sim_out["registers"].append(row)
                                i += 1

                        with open("sim_output.json", "w") as output:
                            output.write(json.dumps(sim_out, indent=4, default=lambda o: o.item() if hasattr(o, 'item') else float(o)))
                        self.logger.log("Simulations finished. Results written to sim_output.json!", style="bold blue")

                    # ============ CONTRASTIVE VERDICT ============
                    # Nominal baseline + symbolic effect (bottle off the tray) + IMU score.
                    if self.hypotheses_compiled is not None:
                        entries = self.hypotheses_compiled["entries"]
                        skipped = self.hypotheses_compiled["skipped"]
                        case_id = self.hypotheses_compiled.get("case_id", "") or "hypotheses_case"
                    else:
                        entries = [
                            {"hypothesis_id": f"static_{c['name']}", "title": c["name"], "cause": c}
                            for c in self.causes_data
                        ]
                        skipped = []
                        case_id = "static_causes"

                    ordered_historicals = [historicals[pid[0]] for pid in pids]
                    verdict = build_verdict(
                        case_id=case_id,
                        real_imu=self.imu_history,
                        entries=entries,
                        historicals=ordered_historicals,
                        initial_bottle_z=self.sim_scene.bottle_position[2],
                        skipped=skipped,
                        observed_effect_time=self.observed_effect_time,
                    )
                    verdict_path = os.path.abspath(write_verdict(verdict, "logs/verdicts"))
                    save_comparison_plots(self.imu_history, verdict["hypotheses"], ordered_historicals, plots_dir)

                    accepted_id = verdict.get("accepted_hypothesis_id")
                    self.logger.log(
                        f"Verdict written to {verdict_path}. Accepted hypothesis: {accepted_id}. "
                        f"Nominal score: {verdict.get('nominal_best_score')}. "
                        f"Window: {verdict['anomaly_window']}.",
                        style="bold green" if accepted_id else "bold yellow",
                    )

                    # Synthesize detector agent templates only for accepted causes
                    accepted_causes = {
                        e["cause"]["name"] for e in verdict["hypotheses"] if e.get("accepted")
                    }
                    for cause_name in accepted_causes:
                        if not generate_agent(cause_name, AGENTS_FOLDER):
                            print("Error while generating agent template for cause", cause_name)
                    if accepted_causes:
                        print("Agent templates generated at folder", AGENTS_FOLDER)

                    if self.hypotheses_compiled is not None:
                        self.publish_verdict_filepath(verdict_path)
                    # Back to IDLE: the processed-episode guard avoids re-simulating the same
                    # episode, while a later hypotheses batch for it is still picked up.
                    self.retire_batch()

                case "TERMINATED":
                    pass
                
        except Exception as e:
            self.logger.log(f"Exception during '{self.state}': {e} at line {sys.exc_info()[-1].tb_lineno}", style="bold red")
            import traceback
            self.logger.log(traceback.format_exc(), style="red")
            self.retire_batch()

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
                            (2 * self.distance_between_wheels))
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
        Returns True when there is something new to simulate (an unprocessed episode or a new
        hypotheses batch from the semantic agent) and the state should transition to SIMULATE_REASON.
        """
        spc_node = None
        fp_node = None
        for node in self.graphs["episodic"].get_nodes():
            if node.name.startswith("Search Problem Cause"):  # TODO: Should be a better way to identify the correct node.
                spc_node = node
            if node.name.startswith("Follow Person"):  # TODO: Should be a better way to identify the correct node.
                fp_node = node

        if not (spc_node is not None
                and "status" in spc_node.attrs
                and spc_node.attrs["status"].value == "running"
                and fp_node is not None
                and "filepath" in fp_node.attrs
                and fp_node.attrs["filepath"].value is not None):
            return False

        episode_path = fp_node.attrs["filepath"].value
        has_new_hypotheses = self.load_hypotheses_causes()
        if not has_new_hypotheses and episode_path in self.processed_episode_paths:
            return False

        self.logger.log(
            "Search Problem Cause node found with status 'running' and Follow Person node found with non-empty filepath.",
            style="bold blue"
        )
        self.actual_time = time.time()
        if self.mem_api_path != episode_path:
            self.logger.log("MemoryAPI will be initialized with filepath: " + episode_path, style="blue")
            self.mem_api = mem.EpisodicMemoryAPI(episode_path)
            self.mem_api_path = episode_path

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
        self.processed_episode_paths.add(episode_path)

        if not self.mem_api.is_ready():
            self.logger.log(
                f"Episode recording '{episode_path}' could not be indexed "
                f"(no keyframes / malformed). Skipping this episode. "
                f"Hint: decimal commas in the file indicate a locale problem "
                f"in the recorder agent.",
                style="bold red",
            )
            self.retire_batch()
            return False

        self.writeSimulationScene()
        return True

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
        self.robot_adv_speed_history = commanded_speed_history(self.mem_api)
        if self.robot_adv_speed_history is None:
            self.logger.log("Episodic Memory API is not ready!", style="bold red")
            return None
        self.logger.log(f"Robot speeds loaded from epidodic memory",style="bold blue")
        return self.robot_adv_speed_history

    
    def convert_episodic_to_imu_history(self, list_of_ts: list) -> dict | None:
        """Convert the episodic memory mission to the IMU history format given a list of timestamps to fill.
            Parameters:
                - list_of_ts (list): List of timestamps in seconds to convert from episodic memory.
            Returns:
                - dict: with keys "timestamp", "accelerometer" and "gyroscope", each one with a list of values for each timestamp in the list_of_ts.
                - None: if episodic memory is not ready.
        """
        self.imu_history = real_imu_history(self.mem_api, list_of_ts)
        if self.imu_history is None:
            self.imu_history = {TIMESTAMP: [], ACCELEROMETER: [], GYROSCOPE: []}
            self.logger.log("Episodic Memory API is not ready! WTF?", style="bold red")
            return

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
        self.logger.log(f"Converted episodic IMU to IMU history format ({len(self.imu_history[TIMESTAMP])} frames).", style="bold blue")
        return self.imu_history


    def get_simulation_length_from_episodic_memory(self) -> float | None:
        """Get the length of the simulation from the episodic memory, by looking for the last timestamp of the "imu" node history.
            Returns:
                - float: length of the simulation in seconds.
                - None: if not available.
        """
        if not self.mem_api.is_ready():
            self.logger.log("Episodic Memory API is not ready! WTF?", style="bold red")
            return None
        simulation_length = episode_length_s(self.mem_api)
        if simulation_length is None:
            self.logger.log("No IMU events found in episodic memory!",style="bold red")
            return None
        self.logger.log(f"Simulation length obtained from episodic memory:{simulation_length} seconds.",style="bold blue")
        return simulation_length
        
    def get_robot_positions_relative_to_problem(self) -> dict[str, list[float]] | None:
        """
            Get robot position data from episodic memory.
            Returns both:
                - the first robot position recorded in the history,
                - the last robot position before the first problem appearance.
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