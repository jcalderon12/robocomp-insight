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
AGENTS_FOLDER = "agents/"
EM_HISTORY_FILE = "src/mission_Follow Path_25032026_120505.txt"

# DSR contract with the semantic agent (work graph): its node of the open case, where it publishes
# the hypotheses batch and this agent the verdict, or the mark that it gave up waiting for the batch.
UNEXPLAINED_NODE = "unexplained"
HYPOTHESES_FILEPATH_ATTR = "hypotheses_filepath"
VERDICT_FILEPATH_ATTR = "verdict_filepath"
SEARCH_ABANDONED_ATTR = "search_abandoned"
# Default of Simulation.BatchWaitSeconds: past it without a batch the search ends unsimulated.
# The local LLM took ~85 s per batch in the flows of 11/2026.
BATCH_WAIT_DEFAULT_S = 180.0

# Keys of IMU history dictionary
TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
ADV_SPEED = "adv_speed"

# Keys of IMU historical dictionary (recieved from causes simulator)
HISTORY = "history"

# Positions of the bodies in the scene (millimeters)
ROBOT_POS = [-3.700, -0.300, 0.0325]
PROBLEM_POS = [0.0, 0.040, 0.001]
BOTTLE_POS = [0.0, 0.050, 0.795]

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
import hashlib
import zlib
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

# Get the path to 'agents' (one level up from 'inner_simulator') to reach the
# shared agent_generation package (agent scaffolding + templates + training).
agents_root = os.path.dirname(agent_root)
if agents_root not in sys.path:
    sys.path.insert(0, agents_root)

from src.simulation_scene import SimulationScene
from src.logger import Logger
from src.episode_scene import (
    DEFAULT_REPETITIONS,
    build_simulation_scene,
    commanded_speed_history,
    episode_length_s,
    real_imu_history,
    real_record,
)
from src.hypothesis_compiler import anchor_to_footprint, compile_batch
from src.verdict import (build_verdict, channels_to_json, save_comparison_plots, save_real_imu_plot,
                         scene_channels, simulated_runs, write_simulations, write_verdict)
from src.hypothesis_compiler import NOMINAL_HYPOTHESIS_ID
from src import search_rounds

import pybullet as p
import json
import episodic_memory_api as mem
import numpy as np
import locale
import math

from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from agent_generation.agent_generator import *

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

        # Simulation visualization (config: Simulation.Gui / Simulation.RealTime).
        # Off by default so the pipeline stays headless and runs every cause in
        # parallel; enable to watch each cause in its own PyBullet window.
        sim_cfg = configData.get("Simulation", {})
        self.sim_gui = bool(sim_cfg.get("Gui", False))
        self.sim_real_time = bool(sim_cfg.get("RealTime", False))
        self.sim_processes = int(sim_cfg.get("Processes", 0)) or os.cpu_count() or 1
        # How long to wait for the semantic agent's batch before the static causes (see waiting_for_batch).
        self.batch_wait_s = float(sim_cfg.get("BatchWaitSeconds", BATCH_WAIT_DEFAULT_S))
        # Repeticiones por causa (0: DEFAULT_REPETITIONS de episode_scene, 4000); menos acorta la busqueda.
        self.sim_repetitions = int(sim_cfg.get("Repetitions", 0)) or DEFAULT_REPETITIONS
        # Si el veredicto se abstiene, confirmar igualmente la hipotesis mas probable (provisional).
        self.accept_most_probable = bool(sim_cfg.get("AcceptMostProbable", False))
        self.batch_wait_started = {}

        # Hypothesis-driven mode state (set when the semantic agent publishes a batch)
        self.hypotheses_compiled = None
        self.hypotheses_batch = None
        self.hypotheses_path = None
        self.processed_hypotheses_paths = set()
        self.processed_episode_paths = set()
        self.mem_api_path = None
        # When the recording saw the bottle leave the robot (seconds, IMU time base).
        self.observed_effect_time = None
        # The instant the current batch was found and how long each stage took, for its verdict (stage_done).
        self.case_timings = None
        # Where the verdicts and their plots go (the agent runs from agents/inner_simulator).
        self.output_root = "logs"

        # Clear terminal
        os.system('cls' if os.name == 'nt' else 'clear')

        self.print_time = self.actual_time = time.time()

        # ================ CAUSE SIMULATOR  ================
        # ====================================================
        # Declare causes data dict
        self.sim_scene = SimulationScene.model_construct()
        # Set initial pose (Debugging purposes)
        self.sim_scene.gravity = -9.81

        # ================ EPISODIC MEMORY API =================
        # ======================================================
        # Loaded dynamically at runtime from Follow Person filepath.

        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================

        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-50,
                                     cameraTargetPosition=[0.8, -0.9, 2.])
        
        self.dt = 1.0 / 240.0  # Simulation time step    
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)

        # ================= PYBULLET MODELS LOADING  ================
        # ===========================================================

        flags = p.URDF_USE_INERTIA_FROM_FILE

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 

        # LOAD ROBOT IN THE SIMULATION
        self.robot_initial_position = [0, 0, 0.005]  # Small Z offset in meters to place robot slightly above the ground
        self.robot_initial_orientation = p.getQuaternionFromEuler([0, 0, -math.pi / 2])
        self.robot = p.loadURDF("../../etc/URDFs/shadow/shadow.urdf", self.robot_initial_position, self.robot_initial_orientation, flags=flags)

        self.bottle_position_offset = [0.15, 0.11, 0.802] # Position of the bottle relative to the robot's base_link frame (in meters)

        # LOAD BOTTLE IN THE SIMULATION
        self.bottle = p.loadURDF("../../etc/URDFs/bottle/bottle.urdf", [self.robot_initial_position[0] + self.bottle_position_offset[0], 
                                                                        self.robot_initial_position[1] + self.bottle_position_offset[1], 
                                                                        self.robot_initial_position[2] + self.bottle_position_offset[2]], 
                                 self.robot_initial_orientation, flags=flags)

        time.sleep(1)  # Wait for a second to ensure the models are loaded properly

        
        # ================ START SIMULATION ================
        # ==================================================

        p.setRealTimeSimulation(1) # Enable real-time simulation

        # ================ ROBOT PARAMETERS  ===============
        # ==================================================

        self.wheels_radius = 0.1                 # m (igual que antes)
        self.wheels_width = 0.05                 # m, ancho de cada rueda
        self.distance_between_wheels = 0.518     # m, entre centros de rueda (axesLength del driver)
        self.distance_from_center_to_wheels = self.distance_between_wheels / 2   # 0.259 m

        self.base_width = 0.44                   # m, ancho de la caja del chasis (boundingObject)

        self.motors = ["wheel_right_joint", "wheel_left_joint"]
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
                             list_of_target_velocities) -> None:
        """
            Set the simulation scene as the real one when the problem was detected, 
            with the same target velocities that the robot had in the real world, 
            and other parameters like the bottle position.

            Args:
                simulation_length (float): The length of the simulation in seconds.
                list_of_target_velocities (list[tuple[float, float]]): A list of tuples with the forward and angular velocities of the robot at each timestamp in the real world.
        """
        robot_positions = self.get_robot_positions_relative_to_problem()
        if robot_positions is not None and robot_positions.get("last_position_before_problem") is not None:
            lpbp = robot_positions["last_position_before_problem"]
            # lpbp comes from episodic's root->robot RT, in METERS, and that is what sim_scene
            # and CauseBump's grid expect: no conversion. The RT also has x/y swapped by
            # auto_localization(), which is what the [1],[0],[2] undoes.
            problem_position_fixed = [lpbp[1] , lpbp[0] , lpbp[2]]
            self.sim_scene.problem_position = problem_position_fixed
            self.logger.log(f"Problem position set for simulation: {problem_position_fixed} (m)", style="green")
        else:
            self.sim_scene.problem_position = PROBLEM_POS
        self.sim_scene.problem_orientation = [0,0,0,1]

        if robot_positions is not None and robot_positions.get("first_position") is not None:
            fp = robot_positions["first_position"]
            robot_position_fixed = [fp[1] , fp[0] , fp[2]]
            self.sim_scene.initial_robot_position = robot_position_fixed
            self.sim_scene.initial_robot_orientation = self.robot_initial_orientation
            self.sim_scene.bottle_position = [fp[1] + self.bottle_position_offset[1], fp[0] + self.bottle_position_offset[0], fp[2] + self.bottle_position_offset[2]]
            self.sim_scene.bottle_orientation = [self.robot_initial_orientation[0], self.robot_initial_orientation[1], self.robot_initial_orientation[2], self.robot_initial_orientation[3]]

            self.logger.log(f"Initial robot position set for simulation: {robot_position_fixed} (m)", style="green")
        else:
            self.logger.log("Could not read initial robot position from episodic memory, using default ROBOT_POS", style="yellow")
            self.sim_scene.initial_robot_position = ROBOT_POS
        
        self.sim_scene.simulation_length = simulation_length
        self.sim_scene.list_of_target_velocities = list_of_target_velocities



        print(type(self.sim_scene.simulation_length), repr(self.sim_scene.simulation_length))


        self.sim_scene.model_validate(self.sim_scene.__dict__)
        file = open("src/sim_scene.json", "w")
        file.write(self.sim_scene.model_dump_json(indent=4))
        file.close()


    def writeSimulationScene(self) -> None:
       # Build the scene from the episodic recording; scene constants remain as fallback.
       self.logger.log("Writing simulation scene to sim_scene.json...", style="bold purple")
       # The anomaly interval comes from the episode the semantic memory built (the batch links it
       # with its sha256); without a batch the scene applies the same rule to the recording.
       scene, info = build_simulation_scene(self.mem_api, ROBOT_POS, PROBLEM_POS,
                                            num_of_repetitions=self.sim_repetitions,
                                            episode_intervals=self.episode_intervals())
       self.logger.log(f"Simulation seeded at {scene['start_time_s']:.2f} s, anomaly interval "
                       f"{scene['anomaly_interval']} ({info['anomaly_interval_source']}).", style="purple")
       if self.hypotheses_compiled is not None:
           # Obstacles only where they can touch the robot during the anomaly interval.
           self.hypotheses_compiled = anchor_to_footprint(self.hypotheses_compiled, scene.get("anomaly_footprint"))
           self.causes_data = [entry["cause"] for entry in self.hypotheses_compiled["entries"]]
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


    def episode_intervals(self) -> list | None:
        """The intervals of the episode the current batch was generated from (contract 1), read from
        the episode file the batch links, if its sha256 matches; None otherwise. The batch cites the
        sha256 of the episode's canonical JSON (sorted keys, no spaces: hypothesis_pipeline.episode_sha256),
        not of the indented file."""
        episode = (getattr(self, "hypotheses_batch", None) or {}).get("episode") or {}
        path = episode.get("path")
        if not path or not os.path.exists(path):
            return None
        try:
            recorded = json.loads(open(path, "rb").read())
            canonical = json.dumps(recorded, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
            if episode.get("sha256") and hashlib.sha256(canonical.encode("utf-8")).hexdigest() != episode["sha256"]:
                self.logger.log(f"Episode file {path} does not match the batch's sha256; not used.", style="yellow")
                return None
            return recorded.get("intervals")
        except Exception as e:
            self.logger.log(f"Cannot read the episode file {path}: {e}", style="yellow")
            return None

    def load_hypotheses_causes(self) -> bool:
        """Compile the hypotheses batch published on the intention node of the work
        graph. False means there is nothing new to simulate."""
        self.hypotheses_compiled = None
        self.hypotheses_batch = None
        self.case_timings = None
        try:
            problem_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
        except Exception as e:
            self.logger.log(f"Cannot read work graph for hypotheses: {e}", style="yellow")
            return False
        if problem_node is None or HYPOTHESES_FILEPATH_ATTR not in problem_node.attrs:
            return False

        hypotheses_path = problem_node.attrs[HYPOTHESES_FILEPATH_ATTR].value
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
        self.hypotheses_batch = batch
        self.case_timings = {"batch_found_at": time.time(), "stages_s": {}}
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
            problem_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
            if problem_node is None:
                self.logger.log("Cannot publish verdict: intention node not found.", style="yellow")
                return False
            problem_node.attrs[VERDICT_FILEPATH_ATTR] = Attribute(str(verdict_path), self.agent_id)
            self.graphs["work"].update_node(problem_node)
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
        self.hypotheses_batch = None
        self.hypotheses_path = None
        self.state = "IDLE"

    def stage_done(self, stage: str, started: float) -> None:
        """Keep how long a stage of the current batch took since `started` (time.perf_counter)."""
        if self.case_timings is not None:
            self.case_timings["stages_s"][stage] = round(time.perf_counter() - started, 3)

    def verdict_timings(self) -> dict:
        """The timings the verdict carries (docs_output/diseno/evaluacion.md, 4.3): when the batch was found
        (epoch seconds), each stage in seconds, when the verdict is written and the total between both."""
        timings = dict(self.case_timings or {"batch_found_at": None, "stages_s": {}})
        written_at = time.time()
        found_at = timings.get("batch_found_at")
        timings.update(batch_found_at=round(found_at, 3) if found_at is not None else None,
                       verdict_written_at=round(written_at, 3),
                       total_s=round(written_at - found_at, 3) if found_at is not None else None,
                       processes=self.sim_processes)
        return timings


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
                    # Only a hypotheses batch of the semantic agent gets here (check_for_problems).
                    # A batch of the semantic memory, headless, on a seeded episode scene: the fast
                    # repetitions, scored in the simulator processes (docs_output/resultados/41).
                    scene_data = self.sim_scene.model_dump()
                    fast = (not self.sim_gui and scene_data.get("observed_effect_time") is not None
                            and scene_data.get("anomaly_interval"))
                    started = time.perf_counter()
                    record = real_record(self.mem_api)
                    self.stage_done("record", started)
                    started = time.perf_counter()
                    if fast:
                        ordered_historicals = self.run_fast_simulations(record, scene_data)
                    else:
                        ordered_historicals = self.run_simulations()
                    self.stage_done("simulation", started)
                    simulated_at = time.perf_counter()

                    # Convert episodic memory history to IMU history given one of the historicals ts list as an example.
                    list_of_ts = ordered_historicals[0][0][HISTORY][TIMESTAMP]
                    self.convert_episodic_to_imu_history(list_of_ts)
                    print("Historical INNER frames:", len(self.imu_history[TIMESTAMP]))

                    # Hypotheses of the semantic memory: the contrastive verdict, published for the memory.
                    self.write_batch_verdict(record, ordered_historicals, simulated_at)

                case "TERMINATED":
                    pass
                
        except Exception as e:
            self.logger.log(f"Exception during '{self.state}': {e} at line {sys.exc_info()[-1].tb_lineno}", style="bold red")
            import traceback
            self.logger.log(traceback.format_exc(), style="red")
            self.retire_batch()

        return True



    def write_batch_verdict(self, record: dict, historicals: list, simulated_at: float) -> str:
        """The verdict on the batch of the semantic memory (verdict.py), its plots and the agent templates of the
        accepted causes. What the campaign keeps with it (docs_output/diseno/evaluacion.md, 4.3): the simulations
        file (the scene, the record and the repetition of each hypothesis closest to it) and the timings of every
        stage, so the verdict is written last, then published. `simulated_at` (time.perf_counter) is when the
        simulations ended. Returns the verdict's path."""
        case_id = self.hypotheses_compiled.get("case_id", "") or "hypotheses_case"
        plots_dir = os.path.join(self.output_root, "plots", case_id)
        verdicts_dir = os.path.join(self.output_root, "verdicts")
        os.makedirs(plots_dir, exist_ok=True)
        save_real_imu_plot(self.imu_history, os.path.join(plots_dir, "real_imu_history.png"))

        # ============ CONTRASTIVE VERDICT ============
        # Nominal baseline + symbolic effect (bottle off the tray) + IMU score.
        scene = self.sim_scene.model_dump()
        started = time.perf_counter()
        verdict = build_verdict(
            case_id=case_id,
            record=record,
            entries=self.hypotheses_compiled["entries"],
            historicals=historicals,
            scene=scene,
            skipped=self.hypotheses_compiled["skipped"],
        )
        verdict_s = time.perf_counter() - started
        verdict["simulations_path"] = os.path.abspath(
            write_simulations(simulated_runs(verdict, historicals, scene, record), verdicts_dir))
        save_comparison_plots(self.imu_history, verdict["hypotheses"], historicals, plots_dir)

        # Synthesize detector agent templates only for accepted causes
        accepted_causes = {
            e["cause"]["name"] for e in verdict["hypotheses"] if e.get("accepted")
        }
        for cause_name in accepted_causes:
            if not generate_agent(cause_name, AGENTS_FOLDER):
                print("Error while generating agent template for cause", cause_name)
        if accepted_causes:
            print("Agent templates generated at folder", AGENTS_FOLDER)

        if self.case_timings is not None:
            self.case_timings["stages_s"]["verdict"] = round(verdict_s, 3)
            self.case_timings["stages_s"]["outputs"] = round(time.perf_counter() - simulated_at - verdict_s, 3)
        verdict["timings"] = self.verdict_timings()
        # Abstencion con AcceptMostProbable: la mas probable queda como provisional, sin tocar
        # accepted_hypothesis_id, asi semantic no la consolida como explicacion verificada.
        if (self.accept_most_probable and verdict.get("accepted_hypothesis_id") is None
                and verdict.get("most_probable_hypothesis_id") not in (None, NOMINAL_HYPOTHESIS_ID)):
            verdict["provisional_hypothesis_id"] = verdict["most_probable_hypothesis_id"]
        verdict_path = os.path.abspath(write_verdict(verdict, verdicts_dir))

        accepted_id = verdict.get("accepted_hypothesis_id")
        top_id = verdict.get("most_probable_hypothesis_id")
        top = next((h for h in verdict["hypotheses"] if h["hypothesis_id"] == top_id), {})
        self.logger.log(
            f"Verdict written to {verdict_path}. Accepted hypothesis: {accepted_id}. "
            f"Most probable: {top_id} (posterior {top.get('posterior')}). "
            f"Robust to the Monte Carlo error: {verdict.get('acceptance_robust')}. "
            f"90 % set: {[m['mechanism'] for m in (verdict.get('credible_set') or {}).get('mechanisms', [])]}. "
            f"Anomaly interval: {verdict.get('anomaly_interval')}. "
            f"{verdict['timings']['total_s']} s since the batch was found.",
            style="bold green" if accepted_id else "bold yellow",
        )
        if verdict.get("abstention_reason"):
            self.logger.log(f"Verdict abstained: {verdict['abstention_reason']}.", style="bold red")

        # La provisional va en el veredicto: semantic crea el concepto a partir de el.
        if not accepted_id and verdict.get("provisional_hypothesis_id"):
            self.logger.log(f"Provisionally accepting {verdict['provisional_hypothesis_id']} (AcceptMostProbable).",
                            style="bold yellow")

        self.publish_verdict_filepath(verdict_path)
        # Back to IDLE: the processed-episode guard avoids re-simulating the same
        # episode, while a later hypotheses batch for it is still picked up.
        self.retire_batch()
        return verdict_path

    def run_simulations(self) -> list:
        """One causes_simulator process per cause, every repetition with its history: the static causes,
        and a batch watched in the PyBullet GUI. Returns the repetitions of every cause, in order."""
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
        return [historicals[pid[0]] for pid in pids]

    def run_fast_simulations(self, record: dict, scene: dict) -> list:
        """The batch's causes searched in rounds (search_rounds.search: draws from each cause's prior, then rounds
        around the best ones, with importance weights), every repetition through causes_simulator --channels
        (doFastSimulations), Simulation.Processes processes at a time, each scoring its repetitions against the
        real channels of the verdict. Returns the repetitions of every cause, in order: the nominal and each
        chunk's best keep their history (docs_output/resultados/41, 46)."""
        work = os.path.abspath("logs/search")
        os.makedirs(work, exist_ok=True)
        channels_path = os.path.abspath("src/sim_channels.json")
        with open(channels_path, "w") as f:
            json.dump(channels_to_json(scene_channels(record, scene)), f)
        case_id = (self.hypotheses_compiled or {}).get("case_id") or "case"
        seed = zlib.crc32(case_id.encode()) % (2 ** 31)

        def run(spec):
            rpipe, wpipe = os.pipe()
            command = [str(sys.executable), "src/causes_simulator.py", "-c", json.dumps({"cause": spec["cause"]}),
                       "-s", "src/sim_scene.json", "-p", str(wpipe), "--channels", channels_path,
                       "--first", str(spec["first"]), "--count", str(spec["count"])]
            if spec.get("seed") is not None:
                command += ["--seed", str(spec["seed"])]
            if spec.get("draws") is not None:
                draws_path = os.path.join(work, f"draws_{os.getpid()}_{id(spec)}.json")
                with open(draws_path, "w") as f:
                    json.dump(spec["draws"], f)
                command += ["--draws", draws_path]
            process = subprocess.Popen(command, pass_fds=(wpipe,))
            os.close(wpipe)
            with os.fdopen(rpipe) as pipe:
                data = pipe.read()
            process.wait()
            if spec.get("draws") is not None:
                os.remove(draws_path)
            if not data:
                raise ValueError(f"No data from cause {spec['index']}, repetitions {spec['first']}-"
                                 f"{spec['first'] + spec['count'] - 1}: did the process crash?")
            return json.loads(data)

        def launch(specs):
            with ThreadPoolExecutor(max_workers=self.sim_processes) as pool:
                return list(pool.map(run, specs))

        started = time.time()
        ordered = search_rounds.search(
            self.causes_data,
            lambda cause: int(cause.get("num_of_repetitions") or scene["num_of_repetitions"]),
            launch, self.sim_processes, seed, log=lambda line: self.logger.log(line, style="blue"))
        self.logger.log(f"{sum(len(r) for r in ordered)} repetitions of {len(ordered)} causes in "
                        f"{time.time() - started:.1f} s ({self.sim_processes} processes).", style="bold blue")
        return ordered

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



    # =============== PLOTTING GRAPHS  ================
    # ==================================================

    def plot_imu_series(self, timestamps, accelerometer, gyroscope,
                        sim_timestamps=None, sim_accelerometer=None, sim_gyroscope=None,
                        title="Real IMU history from Episodic Memory",
                        save_path=None, show=False):
        """
        Plot an IMU time series (accelerometer and gyroscope), optionally overlaying a
        simulated series (dashed lines, same color per axis).

        :param timestamps: Sequence of N timestamps in seconds (real)
        :param accelerometer: Sequence of N samples [ax, ay, az] in m/s^2 (real)
        :param gyroscope: Sequence of N samples [gx, gy, gz] in rad/s (real)
        :param sim_timestamps: Optional sequence of M timestamps in seconds (sim)
        :param sim_accelerometer: Optional sequence of M samples [ax, ay, az] (sim)
        :param sim_gyroscope: Optional sequence of M samples [gx, gy, gz] (sim)
        :param title: Figure title
        :param save_path: If given, save the figure to this path
        :param show: If True, call plt.show()
        :return: The matplotlib figure
        """

        def _check(t, acc, gyro, name):
            t = np.asarray(t, dtype=float)
            acc = np.asarray(acc, dtype=float)
            gyro = np.asarray(gyro, dtype=float)
            if acc.ndim != 2 or acc.shape[1] != 3 or gyro.ndim != 2 or gyro.shape[1] != 3:
                raise ValueError(f"{name}: accelerometer and gyroscope must have shape (N, 3)")
            if not (len(t) == len(acc) == len(gyro)):
                raise ValueError(f"{name}: timestamps, accelerometer and gyroscope must have the same length")
            return t, acc, gyro

        t, acc, gyro = _check(timestamps, accelerometer, gyroscope, "real")

        sim = None
        if sim_timestamps is not None:
            if sim_accelerometer is None or sim_gyroscope is None:
                raise ValueError("sim_timestamps requires sim_accelerometer and sim_gyroscope")
            sim = _check(sim_timestamps, sim_accelerometer, sim_gyroscope, "sim")

        axis_labels = ["X", "Y", "Z"]
        axis_colors = ["tab:red", "tab:blue", "tab:green"]

        fig = plt.figure(figsize=(12, 5))
        plt.suptitle(title, fontsize=16)

        panels = [("Accelerometer", acc, None if sim is None else sim[1], "Acceleration (m/s^2)"),
                  ("Gyroscope", gyro, None if sim is None else sim[2], "Angular Velocity (rad/s)")]
        for k, (name, real_data, sim_data, ylabel) in enumerate(panels):
            plt.subplot(1, 2, k + 1)
            plt.title(name)
            for j, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
                plt.plot(t, real_data[:, j], color=color, linestyle="-", label=f"Real {axis}")
                if sim_data is not None:
                    plt.plot(sim[0], sim_data[:, j], color=color, linestyle="--", label=f"Sim {axis}")
            plt.xlabel("Time (s)")
            plt.ylabel(ylabel)
            plt.legend()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if save_path is not None:
            fig.savefig(save_path, dpi=150)
        if show:
            plt.show()
        return fig

    
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

        :return: Forward velocity (m/s)
        """
        w_left = p.getJointState(self.robot, self.joints_name["wheel_left_joint"])[1]
        w_right = p.getJointState(self.robot, self.joints_name["wheel_right_joint"])[1]
        forward_velocity = (w_left + w_right) * self.wheels_radius / 2
        return forward_velocity

    def get_angular_velocity(self):
        """
        Get the angular velocity of the robot

        :return: Angular velocity (rad/s), positive = counterclockwise
        """
        w_left = p.getJointState(self.robot, self.joints_name["wheel_left_joint"])[1]
        w_right = p.getJointState(self.robot, self.joints_name["wheel_right_joint"])[1]
        angular_velocity = (w_right - w_left) * self.wheels_radius / self.distance_between_wheels
        return angular_velocity

    def get_wheels_velocity_from_forward_velocity_and_angular_velocity(self, forward_velocity=0, angular_velocity=0):
        """
        Get the velocity of each wheel from the forward and angular velocity of the robot

        :param forward_velocity: Forward velocity of the robot (m/s)
        :param angular_velocity: Angular velocity of the robot (rad/s)
        :return: Dictionary with the velocity (rad/s) of each wheel
        """
        wheels_velocity = {
            "wheel_left_joint": (forward_velocity - self.distance_from_center_to_wheels * angular_velocity) / self.wheels_radius,
            "wheel_right_joint": (forward_velocity + self.distance_from_center_to_wheels * angular_velocity) / self.wheels_radius}
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
            # mission_controller names episodic mission nodes after the mission's customName:
            # "Search Problem Cause" / "Follow Person" in older runs, "search_cause_attempt_N" /
            # "follow_person_attempt_N" since 08/10. The semantic agent matches them the same way
            # (episode_memory_reader.mission_kind), so both read the same recording.
            normalized = node.name.strip().lower().replace(" ", "_")
            if normalized.startswith(("search_problem_cause", "search_cause")):
                spc_node = node
            if normalized.startswith("follow_person"):
                fp_node = node

        if not (spc_node is not None
                and "status" in spc_node.attrs
                and spc_node.attrs["status"].value == "running"
                and fp_node is not None
                and "filepath" in fp_node.attrs
                and fp_node.attrs["filepath"].value is not None):
            return False

        episode_path = fp_node.attrs["filepath"].value
        # Sin lote del semantic no se simula nada (no hay causas de reserva): se espera hasta
        # BatchWaitSeconds y, pasado, se abandona esta grabacion. Un lote que llegue despues se simula.
        if not self.load_hypotheses_causes():
            if episode_path not in self.processed_episode_paths and not self.waiting_for_batch(episode_path):
                self.processed_episode_paths.add(episode_path)
            return False

        self.logger.log(
            "Search Problem Cause node found with status 'running' and Follow Person node found with non-empty filepath.",
            style="bold blue"
        )
        self.actual_time = time.time()
        started = time.perf_counter()
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
        self.stage_done("episodic_memory", started)

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

        started = time.perf_counter()
        self.writeSimulationScene()
        self.stage_done("scene", started)
        return True

    def waiting_for_batch(self, episode_path: str) -> bool:
        """Whether to keep waiting for the semantic agent's hypotheses batch for this closed
        recording: its 'unexplained' node is in the work graph and the wait, counted
        from the first time the closed recording is seen, is under batch_wait_s. There are no
        static causes to fall back to: past the wait (or without the node) nothing is simulated."""
        try:
            explaining = self.graphs["work"].get_node(UNEXPLAINED_NODE) is not None
        except Exception:
            explaining = False
        if not explaining:
            self.logger.log("Recording closed with no 'unexplained' node: nothing to simulate.", style="yellow")
            return False
        if episode_path not in self.batch_wait_started:
            self.batch_wait_started[episode_path] = time.time()
            self.logger.log(
                f"Recording closed while the semantic agent explains it: waiting up to "
                f"{self.batch_wait_s:.0f} s for its hypotheses batch.", style="blue")
        if time.time() - self.batch_wait_started[episode_path] < self.batch_wait_s:
            return True
        self.logger.log(
            f"No hypotheses batch after {self.batch_wait_s:.0f} s: the search ends without simulating.", style="bold yellow")
        self.mark_search_abandoned()
        return False

    def mark_search_abandoned(self) -> None:
        """Marca el nodo del caso para que el semantic lo cierre (y descarte un lote que llegue tarde):
        sin el, la busqueda no termina."""
        try:
            problem_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
            if problem_node is None:
                return
            problem_node.attrs[SEARCH_ABANDONED_ATTR] = Attribute(True, self.agent_id)
            self.graphs["work"].update_node(problem_node)
        except Exception as e:
            self.logger.log(f"Cannot mark the case as abandoned: {e}", style="bold red")

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
        the 'unexplained' node, using the winning grid cell from the best-matching simulation.

        :param best_recording: The best-matching simulation recording (includes 'generated_instances').
        :param position_index: Id/repetition index of the best-matching simulation, which
                                doubles as the index into 'distributed_positions'.
        """
        distributed_positions = best_recording.get("generated_instances", {}).get("distributed_positions")
        if not distributed_positions or position_index >= len(distributed_positions):
            return  # This cause has no spatial grid (e.g. "wheel"); nothing to store.

        position = distributed_positions[position_index]

        problem_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
        if problem_node is None:
            self.logger.log("'unexplained' node not found in DSR graph, cannot store problem_position.", style="bold red")
            return

        problem_node.attrs["problem_position"] = Attribute(list(position), self.agent_id)
        self.graphs["work"].update_node(problem_node)
        self.logger.log(f"Stored problem_position {position} (m) on 'unexplained' node.", style="bold green")


    def mark_cause_confirmed_in_dsr(self) -> None:
        """
        Flag on 'unexplained' that the causal search concluded, for "semantic" to react to.
        """
        problem_node = self.graphs["work"].get_node(UNEXPLAINED_NODE)
        if problem_node is None:
            self.logger.log("'unexplained' node not found in DSR graph, cannot set cause_confirmed.", style="bold red")
            return

        problem_node.attrs["cause_confirmed"] = Attribute(True, self.agent_id)
        self.graphs["work"].update_node(problem_node)
        self.logger.log("Stored cause_confirmed=True on 'unexplained' node.", style="bold green")


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
                - the last robot position before the first problget_robot_positions_relative_to_problemem appearance.
        """
        if self.mem_api.is_ready():
            root_node = self.graphs["work"].get_node("root")
            robot_node = self.graphs["work"].get_node("robot")
            if root_node is None or robot_node is None:
                self.logger.log("Root or robot node not found in work graph!", style="bold red")
                return None

            robot_positions = [pos for pos in self.mem_api.get_edge_history(root_node.id, robot_node.id, "RT") if pos.modification_type == "MEA"]
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


            problem_events = self.mem_api.get_node_history_by_name(UNEXPLAINED_NODE)
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