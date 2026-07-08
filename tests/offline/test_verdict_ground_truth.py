"""Offline end-to-end test: verdict with injected ground truth (slow, ~1-2 min).

A repetition of the 'external_force' cause plays the role of the real episode;
the verdict must accept the external_force hypothesis and reject the others,
and the semantic ingestor must turn the verdict into causal triples.

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

SCENE = {
    "gravity": -9.81,
    "initial_robot_position": [-3.7, -0.3, 0.0325],
    "initial_robot_orientation": [0.0, 0.0, 0.0, 1.0],
    "problem_position": [-1.7, -0.3, 0.03],
    "problem_orientation": [0.0, 0.0, 0.0, 1.0],
    "simulation_length": 4.0,
    "list_of_target_velocities": {"timestamp": [0.0], "adv_speed": [0.5]},
    "num_of_repetitions": 4,
    "bottle_position": [-3.65, -0.19, 0.795],
    "bottle_orientation": [0.0, 0.0, 0.0, 0.0],
}

GROUND_TRUTH_CAUSE = {
    "name": "external_force", "force_target": "bottle",
    "force_window_start": 0.4, "force_window_end": 0.5,
    "force_x_range": 4.0, "force_y_range": 4.0, "force_z_range": 0.0,
    "force_x_origin": 22.0, "force_y_origin": 0.0, "force_z_origin": 0.0,
}

ENTRIES = [
    {"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "Nominal", "cause": {"name": "none"}},
    {"hypothesis_id": "EXT_bump", "title": "Bump", "cause": {
        "name": "bump", "bump_file": str(REPO / "etc/URDFs/bump/bump_100x5cm.urdf"),
        "bump_x_range": 0.6, "bump_y_range": 0.2, "bump_z_range": 0.0,
        "bump_x_origin": -2.9, "bump_y_origin": -0.3, "bump_z_origin": 0.004}},
    {"hypothesis_id": "INT_wheel", "title": "Wheel", "cause": {
        "name": "wheel", "wheel_wheel": "BR", "wheel_min": 0.3, "wheel_max": 0.6}},
    {"hypothesis_id": "EXT_force", "title": "Force on bottle", "cause": dict(GROUND_TRUTH_CAUSE)},
    {"hypothesis_id": "EXT_friction", "title": "Slippery floor", "cause": {
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
    logger = Logger(str(WORKDIR / "test.log"))
    scene_path = WORKDIR / "scene.json"

    truth = run_cause(GROUND_TRUTH_CAUSE, scene_path, logger, reps=1)[0]
    assert truth["bottle_position"][2] < 0.4, "ground truth should knock the bottle down"

    historicals = [run_cause(e["cause"], scene_path, logger, reps=4) for e in ENTRIES]
    verdict = build_verdict(
        case_id="ground_truth_test",
        real_imu=truth["history"],
        entries=ENTRIES,
        historicals=historicals,
        initial_bottle_z=SCENE["bottle_position"][2],
    )
    verdict_path = write_verdict(verdict, str(WORKDIR))

    for h in verdict["hypotheses"]:
        best_effect = f"{h['best_effect_score']:.3f}" if h["best_effect_score"] is not None else "-"
        print(f"  {h['hypothesis_id']:>13}: best={h['best_score']:.3f} best_effect={best_effect} "
              f"effect_rate={h['effect_rate']:.2f} accepted={h['accepted']}")
    assert verdict["accepted_hypothesis_id"] == "EXT_force", verdict["accepted_hypothesis_id"]

    # Semantic side: verdict -> causal triples
    sys.path.insert(0, str(REPO / "agents" / "semantic"))
    from src.verdict_ingestor import ingest_verdict

    ingestion = ingest_verdict(verdict_path)
    assert ingestion.ok and ingestion.accepted_intervention == "external_force", ingestion
    assert any("explainedBy" in t[1] for t in ingestion.triples)

    print("test_verdict_ground_truth OK")


if __name__ == "__main__":
    main()
