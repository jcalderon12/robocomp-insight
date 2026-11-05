"""Offline end-to-end test: the verdict with injected ground truth (~1 min).

A simulated push on the robot plays the real episode: its IMU and path are the record. The hypotheses
are the nominal, a bump, a blocked wheel, a push on the robot (from ranges around the true one, as the
memory grounds it) and a slippery patch, all simulated by the causes simulator. The verdict
(docs_output/diseno/veredicto.md) must give the push the highest posterior and accept it, and the
semantic ingestor must turn the verdict into causal triples.

Run from the repo root:  python3 tests/offline/test_verdict_ground_truth.py
(requires pybullet; runs headless)
"""
import json
import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"

os.chdir(INNER)
sys.path.insert(0, str(INNER))
sys.path.insert(0, str(INNER / "src"))

import pybullet as p
from causes_simulator import CausesSimulator
from src.logger import Logger
from src.verdict import NOMINAL_HYPOTHESIS_ID, build_verdict, write_verdict

WORKDIR = Path(tempfile.mkdtemp(prefix="insight_verdict_test_"))

# Positions in meters, as the causes simulator expects (see SimulationScene). The differential
# shadow URDF drives along its +y: yaw -90 deg heads it along world +x, as in the recordings, and
# the bottle sits where the Webots world puts it (episode_scene.DEFAULT_TRAY_OFFSET, rotated).
SCENE = {
    "gravity": -9.81,
    "initial_robot_position": [-3.7, -0.3, 0.0],
    "initial_robot_orientation": [0.0, 0.0, -0.7071067811865476, 0.7071067811865476],
    "problem_position": [-1.7, -0.3, 0.03],
    "problem_orientation": [0.0, 0.0, 0.0, 1.0],
    "simulation_length": 4.0,
    "list_of_target_velocities": {"timestamp": [0.0], "adv_speed": [0.5]},
    "num_of_repetitions": 4,
    "bottle_position": [-3.545, -0.19, 0.846],
    "bottle_orientation": [0.0, 0.0, 0.0, 0.0],
}

# The true push: 150 N to the robot's left for 0.2 s from 1.6 s, at a person's hand height. The
# robot drives along world +x, so its left is world +y.
GROUND_TRUTH_CAUSE = {
    "name": "external_force", "force_target": "robot",
    "force_window_start": 0.4, "force_window_end": 0.45, "force_duration_range_s": [0.2, 0.2],
    "force_x_range": 0.0, "force_y_range": 0.0, "force_z_range": 0.0,
    "force_x_origin": 0.0, "force_y_origin": 150.0, "force_z_origin": 0.0,
}
# What the verdict compares on: the anomaly interval, and the effect at its end.
VERDICT_SCENE = {"anomaly_interval": [1.4, 3.0], "start_time_s": 0.0, "observed_effect_time": 3.0}

ENTRIES = [
    {"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "Nominal", "mechanism": "nominal",
     "cause": {"name": "none", "num_of_repetitions": 1}},
    {"hypothesis_id": "EXT_bump", "title": "Bump", "mechanism": "obstacle_traversed", "cause": {
        "name": "bump", "bump_file": str(REPO / "etc/URDFs/bump/bump_100x5cm.urdf"),
        "bump_x_range": 600.0, "bump_y_range": 200.0, "bump_z_range": 0.0,
        "bump_x_origin": -2.9, "bump_y_origin": -0.3, "bump_z_origin": 0.001,
        "grid_dimensions": [2, 2]}},
    {"hypothesis_id": "INT_wheel", "title": "Wheel", "mechanism": "wheel_failure", "cause": {
        "name": "wheel", "wheel_wheel": "R", "wheel_min": 0.35, "wheel_max": 0.6}},
    {"hypothesis_id": "EXT_push", "title": "Push on the robot", "mechanism": "robot_push", "cause": {
        "name": "external_force", "force_target": "robot", "force_window_start": 0.35, "force_window_end": 0.5,
        "force_x_range": 40.0, "force_y_range": 100.0, "force_z_range": 0.0,
        "force_x_origin": 0.0, "force_y_origin": 150.0, "force_z_origin": 0.0}},
    {"hypothesis_id": "EXT_friction", "title": "Slippery floor", "mechanism": "slippery_floor", "cause": {
        "name": "friction", "friction_target": "floor", "friction_min": 0.02, "friction_max": 0.2}},
]


def run_cause(payload, scene_path, logger, reps):
    scene = dict(SCENE)
    scene["num_of_repetitions"] = reps
    scene_path.write_text(json.dumps(scene))
    # In-process run: read sim.historical directly instead of the IPC pipe (a
    # single process writing and later reading the same pipe would deadlock on
    # the pipe buffer once histories exceed ~64KB).
    rpipe, wpipe = os.pipe()
    try:
        sim = CausesSimulator(json.dumps({"cause": payload}), str(scene_path), wpipe, logger,
                              real_time=False, gui=False)
        sim.doSimulations()
        data = json.loads(json.dumps(sim.historical))
    finally:
        os.close(rpipe)
        os.close(wpipe)
        p.disconnect()
    return data


def main():
    import random
    random.seed(0)
    logger = Logger(str(WORKDIR / "test.log"))
    scene_path = WORKDIR / "scene.json"

    truth = run_cause(GROUND_TRUTH_CAUSE, scene_path, logger, reps=1)[0]["history"]
    record = {"accelerometer": {"timestamp": truth["timestamp"], "values": truth["accelerometer"]},
              "gyroscope": {"timestamp": truth["timestamp"], "values": truth["gyroscope"]},
              "path": {"timestamp": truth["timestamp"], "xy": truth["position"]}}

    historicals = [run_cause(e["cause"], scene_path, logger, reps=36) for e in ENTRIES]
    # The kernel width is the simulator's model error. Here the record is itself a simulation, so
    # there is none, and the width is small; production uses the one calibrated against Webots.
    import src.verdict as verdict_module
    verdict_module.EPS = 0.5
    verdict = build_verdict(case_id="ground_truth_test", record=record, entries=ENTRIES,
                            historicals=historicals, scene={**SCENE, **VERDICT_SCENE})
    verdict_path = write_verdict(verdict, str(WORKDIR))

    for h in verdict["hypotheses"]:
        print(f"  {h['hypothesis_id']:>13}: posterior={h['posterior']:.3f} distance={h['distance']} "
              f"reps={h['repetitions']} accepted={h['accepted']}")
    assert verdict["accepted_hypothesis_id"] == "EXT_push", (verdict["accepted_hypothesis_id"], verdict["abstention_reason"])
    push = next(h for h in verdict["hypotheses"] if h["hypothesis_id"] == "EXT_push")
    # The weighted force points to the robot's left, as the true one.
    assert push["parameter_estimate"]["random_range_coordinates.1"]["mean"] > 100.0, push["parameter_estimate"]

    # Semantic side: verdict -> causal triples
    sys.path.insert(0, str(REPO / "agents" / "semantic"))
    from src.verdict_ingestor import ingest_verdict

    ingestion = ingest_verdict(verdict_path)
    assert ingestion.ok and ingestion.accepted_intervention == "external_force", ingestion
    assert any("explainedBy" in t[1] for t in ingestion.triples)

    print("test_verdict_ground_truth OK")


if __name__ == "__main__":
    main()
