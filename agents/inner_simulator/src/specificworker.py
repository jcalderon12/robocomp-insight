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

import threading

from PySide6 import QtCore
from pydsr import Attribute
from rich.console import Console
from genericworker import *
from src.simulation_scene import SimulationScene
from src.logger import Logger
from src.episode_scene import extract_scene_poses
from src.hypothesis_compiler import compile_batch
from src.verdict import build_verdict, write_verdict, save_comparison_plots, save_real_imu_plot


import json
import episodic_memory_api as mem
import numpy as np
import locale
import time
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from .agent_generator import *

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

# Positions of the bodies in the scene
ROBOT_POS = [-3.7, -0.3, 0.0325]
PROBLEM_POS = [0.0, 0.04, 0.001]
BOTTLE_POS = [-3.65, -0.19, 0.795]


sys.path.append('/opt/robocomp/lib')

dir_name = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(dir_name)
sys.path.append(parent_dir + "/src/")
console = Console(highlight=False)

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
            self.logger.log("Episodic Memory API is not ready! WTF?", style="bold red")
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


    def writeSimulationScene(self) -> None:
       # Build the scene from the episodic recording; scene constants remain as fallback.
       self.logger.log("Writing simulation scene to sim_scene.json...", style="bold purple")
       poses = extract_scene_poses(
           self.mem_api,
           fallback_robot_position=ROBOT_POS,
           fallback_bottle_position=BOTTLE_POS,
           fallback_problem_position=PROBLEM_POS,
           tray_offset=[b - r for b, r in zip(BOTTLE_POS, ROBOT_POS)],
       )
       self.logger.log(f"Scene poses reconstructed from episode: {poses.sources}", style="purple")
       self.sim_scene.initial_robot_position = poses.initial_robot_position
       self.sim_scene.initial_robot_orientation = poses.initial_robot_orientation
       self.sim_scene.problem_position = poses.problem_position
       self.sim_scene.problem_orientation = poses.problem_orientation
       self.sim_scene.bottle_position = poses.bottle_position
       self.sim_scene.bottle_orientation = poses.bottle_orientation
       self.sim_scene.simulation_length = self.get_simulation_length_from_episodic_memory()
       adv_speed_history = self.get_robot_adv_speed_history()
       self.sim_scene.list_of_target_velocities = adv_speed_history if adv_speed_history is not None else []
       self.sim_scene.num_of_repetitions = 10
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
        print(flush=True, end="")
        self.show_compute_time_step()
        try:
            match self.state:
        
                case "IDLE":
                    
                    spc_node = None
                    fp_node = None
                    for node in self.graphs["episodic"].get_nodes():
                        if node.name.startswith("Search Problem Cause"): # TODO: Should be a better way to identify the correct node.
                            spc_node = node
                        if node.name.startswith("Follow Person"): # TODO: Should be a better way to identify the correct node.
                            fp_node = node
                    
                    if spc_node is not None and "status" in spc_node.attrs and spc_node.attrs["status"].value == "running" and fp_node is not None and "filepath" in fp_node.attrs and fp_node.attrs["filepath"].value is not None:
                        episode_path = fp_node.attrs["filepath"].value
                        has_new_hypotheses = self.load_hypotheses_causes()
                        if has_new_hypotheses or episode_path not in self.processed_episode_paths:
                            console.print(f"Search Problem Cause node found with status 'running' and Follow Person node found with non-empty filepath.", style="bold green")
                            self.state = "SIMULATE_REASON"
                            self.actual_time = time.time()
                            if self.mem_api_path != episode_path:
                                console.print("MemoryAPI will be initialized with filepath: " + episode_path, style="green")
                                self.mem_api = mem.EpisodicMemoryAPI(episode_path)
                                self.mem_api_path = episode_path

                            timeout = 10
                            start_time = time.time()
                            while not self.mem_api.is_ready():
                                if time.time() - start_time > timeout:
                                    console.print(f"Timeout of {timeout} seconds reached while waiting for Episodic Memory API to be ready!", style="bold red")
                                    break
                                time.sleep(0.1)
                            self.processed_episode_paths.add(episode_path)
                            if not self.mem_api.is_ready():
                                console.print(
                                    f"Episode recording '{episode_path}' could not be indexed "
                                    f"(no keyframes / malformed). Skipping this episode. "
                                    f"Hint: decimal commas in the file indicate a locale problem "
                                    f"in the recorder agent.",
                                    style="bold red",
                                )
                                self.retire_batch()
                            else:
                                self.writeSimulationScene()


                case "SIMULATE_REASON":
                    # Causes were already selected in IDLE: hypothesis-driven if available,
                    # static causes.json otherwise.
                    if self.hypotheses_compiled is None:
                        self.loadCausesJson()

                    # Launch subprocesses for simulation
                    print("Launching subprocesses...")                
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

                    plots_dir = os.path.join(
                        "logs/plots",
                        (self.hypotheses_compiled or {}).get("case_id") or "static_causes",
                    )
                    os.makedirs(plots_dir, exist_ok=True)
                    save_real_imu_plot(self.imu_history, os.path.join(plots_dir, "real_imu_history.png"))

                    if self.hypotheses_compiled is None:
                        # Legacy matching (static causes mode): top-5 ranking into sim_output.json
                        threads = []
                        sim_out = {}
                        sim_out["sim_scene"] = self.sim_scene.model_dump()
                        sim_out["registers"] = []
                        with ProcessPoolExecutor(max_workers=2) as executor:
                            for h in historicals:
                                threads.append(executor.submit(SpecificWorker.find_matching_imu_recordings, self.imu_history, historicals[h]))
                            i = 0
                            for h in historicals:
                                res = threads[i].result()
                                items = []
                                print("Top 5 best recordings for cause", self.causes_data[i]["name"], ":")
                                for rec in res:
                                    print("\tRecording", rec[0], "with score", rec[1])
                                    items.append(historicals[h][rec[0]])
                                row = {"cause_definition": self.causes_data[i], "top_five": items}
                                sim_out["registers"].append(row)
                                i += 1

                        with open("sim_output.json", "w") as output:
                            output.write(json.dumps(sim_out, indent=4))
                        print("Simulations finished. Results written to sim_output.json!")

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
                    )
                    verdict_path = os.path.abspath(write_verdict(verdict, "logs/verdicts"))
                    save_comparison_plots(self.imu_history, verdict["hypotheses"], ordered_historicals, plots_dir)

                    accepted_id = verdict.get("accepted_hypothesis_id")
                    self.logger.log(
                        f"Verdict written to {verdict_path}. Accepted hypothesis: {accepted_id}. "
                        f"Nominal score: {verdict.get('nominal_best_score')}.",
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
                        self.retire_batch()
                    else:
                        # Legacy static-causes mode keeps the one-shot behavior.
                        self.state = "TERMINATED"

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