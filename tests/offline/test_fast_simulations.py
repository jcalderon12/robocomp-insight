"""Offline test: the fast repetitions of a seeded scene (causes_simulator.doFastSimulations, ~30 s).

docs_output/resultados/41_mas_simulaciones_por_segundo. On a synthetic scene (the robot drives straight)
whose "record" is a simulated small bump under the right wheel:
  * the nominal scores exactly as doSimulations' nominal does;
  * a bump the robot never reaches is the nominal, copied; one spawned over the robot is invalid and not
    simulated; the true bump, simulated from the saved nominal, scores as doSimulations scores it;
  * a tall bump hit early is stopped by the lazy cutoff, and doSimulations puts it beyond the cutoff;
  * the lower bound of an unfinished repetition is its distance once it is finished;
  * only the nominal and the best repetition keep their history, and the verdict reads the distances
    (DISTANCE, STOPPED_EARLY); the channels survive the JSON the worker writes.

Run from the repo root:  python3 tests/offline/test_fast_simulations.py
"""
import json
import math
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

from causes_simulator import EPS, LAZY_ETA, CausesSimulator
from src.episode_scene import DEFAULT_TRAY_OFFSET
from src.logger import Logger
from src.verdict import (DISTANCE, HISTORY, STOPPED_EARLY, channels_from_json, channels_to_json, discrepancy,
                         physical_problem, repetition_distance, scene_channels)

G = 9.81
MESH = str(REPO / "etc" / "URDFs" / "bump" / "Bump_100x10cm.stl")
TRUE_BUMP = {"diameter_m": 0.25, "height_m": 0.02, "x_m": 0.45, "y_m": -0.259}   # under the right wheel
FAR_BUMP = {"diameter_m": 0.25, "height_m": 0.02, "x_m": 3.0, "y_m": 2.0}
OVER_ROBOT = {"diameter_m": 0.4, "height_m": 0.05, "x_m": 0.0, "y_m": 0.0}
TALL_EARLY = {"diameter_m": 0.3, "height_m": 0.1, "x_m": 0.45, "y_m": 0.0}     # hits the chassis


def _scene() -> dict:
    # Heading along world +x (yaw -90 deg), as in the recordings; the bottle on the tray; 0.5 m/s from 1 s.
    s = math.sin(-math.pi / 4)
    ox, oy, oz = DEFAULT_TRAY_OFFSET
    return {
        "gravity": -G,
        "initial_robot_position": [0.0, 0.0, 0.0], "initial_robot_orientation": [0.0, 0.0, s, math.cos(-math.pi / 4)],
        "problem_position": [0.0, 0.0, 0.0], "problem_orientation": [0.0, 0.0, 0.0, 1.0],
        "bottle_position": [oy, -ox, oz], "bottle_orientation": [0.0, 0.0, 0.0, 1.0],
        "simulation_length": 5.5, "num_of_repetitions": 1,
        "list_of_target_velocities": {"timestamp": [1.0], "adv_speed": [0.5]},
        "observed_effect_time": 2.5, "anomaly_interval": [1.0, 2.5], "world_time_step_s": 0.008,
    }


def _bump(samples: list) -> dict:
    return {"name": "bump_scaled", "mesh_file": MESH, "mesh_dimensions_m": [1.0, 1.0, 0.0994],
            "area_x": [0.0, 1.0], "area_y": [-0.5, 0.5], "diameter_range": [0.1, 1.0], "height_range": [0.01, 0.1],
            "samples": samples}


def _simulator(cause: dict, workdir: Path):
    scene_path = workdir / "scene.json"
    scene_path.write_text(json.dumps(_scene()), encoding="utf-8")
    read_fd, write_fd = os.pipe()
    os.close(read_fd)
    return CausesSimulator(json.dumps({"cause": cause}), str(scene_path), write_fd,
                           Logger(str(workdir / "sim.log")), real_time=False, gui=False), write_fd


def _legacy(cause: dict) -> list:
    workdir = Path(tempfile.mkdtemp(prefix="insight_fast_"))
    simulator, fd = _simulator(cause, workdir)
    try:
        simulator.doSimulations()
        return json.loads(json.dumps(simulator.historical))
    finally:
        os.close(fd)
        p.disconnect()


def _fast(cause: dict, channels: list):
    workdir = Path(tempfile.mkdtemp(prefix="insight_fast_"))
    simulator, fd = _simulator(cause, workdir)
    try:
        simulator.doFastSimulations(channels)
        historical = json.loads(json.dumps(simulator.historical))
        bound = None
        if cause["name"] != "none":
            # the lower bound, once a repetition is simulated to the cut, is its distance
            simulator.lazy_cutoff = None
            simulator.current_repetition = 0
            rep, kind = simulator._fast_repetition(0)
            bound = (simulator._lower_bound() if kind not in ("nominal", "overlap") else None,
                     discrepancy(channels, rep[HISTORY]) if HISTORY in rep else None)
        return historical, bound
    finally:
        os.close(fd)
        p.disconnect()


def _record(history: dict) -> dict:
    return {"accelerometer": {"timestamp": history["timestamp"], "values": history["accelerometer"]},
            "gyroscope": {"timestamp": history["timestamp"], "values": history["gyroscope"]},
            "path": {"timestamp": history["timestamp"], "xy": history["position"]}}


def main():
    # The "record": the true bump, simulated as production simulated it until now.
    truth = _legacy(_bump([TRUE_BUMP]))[0]
    channels = channels_from_json(json.loads(json.dumps(channels_to_json(scene_channels(_record(truth["history"]), _scene())))))
    assert channels and all(c["values"].ndim == 2 for c in channels)

    legacy_nominal = _legacy({"name": "none"})[0]
    d_nominal = discrepancy(channels, legacy_nominal["history"])
    fast_nominal, _ = _fast({"name": "none"}, channels)
    assert len(fast_nominal) == 1 and fast_nominal[0][DISTANCE] == d_nominal, (fast_nominal[0][DISTANCE], d_nominal)
    assert HISTORY in fast_nominal[0]

    samples = [FAR_BUMP, OVER_ROBOT, TRUE_BUMP, TALL_EARLY]
    legacy = _legacy(_bump(samples))
    fast, _ = _fast(_bump(samples), channels)
    assert len(fast) == len(samples)
    far, over, true, tall = fast
    # never reached: the nominal, copied
    assert far[DISTANCE] == d_nominal, (far[DISTANCE], d_nominal)
    # over the robot from the start: invalid, as doSimulations' repetition is
    assert physical_problem(over) and physical_problem(legacy[1]), (over.get("spawn_overlap"), legacy[1].get("spawn_overlap"))
    # the true bump: from the saved nominal, as doSimulations scores it (to the repetition-order noise)
    d_true_legacy = discrepancy(channels, legacy[2]["history"])
    assert abs(true[DISTANCE] - d_true_legacy) < 0.01, (true[DISTANCE], d_true_legacy)
    assert true[DISTANCE] < 0.5 * d_nominal, (true[DISTANCE], d_nominal)
    # the tall bump hit early: stopped, and production puts it beyond the cutoff
    cutoff = math.sqrt(d_nominal ** 2 + 2.0 * EPS ** 2 * math.log(1.0 / LAZY_ETA))
    assert tall[STOPPED_EARLY] and tall[DISTANCE] is None, tall.get(STOPPED_EARLY)
    assert discrepancy(channels, legacy[3]["history"]) > cutoff
    # only the best keeps its history
    assert [HISTORY in rep for rep in fast] == [False, False, True, False], [HISTORY in rep for rep in fast]
    # the verdict reads the distances
    assert repetition_distance(channels, true) == true[DISTANCE]
    assert repetition_distance(channels, tall) == math.inf
    assert repetition_distance(channels, legacy[2]) == d_true_legacy

    # the lower bound of the true bump, simulated to the cut, is its distance
    _, (bound, final) = _fast(_bump([TRUE_BUMP]), channels)
    assert bound is not None and abs(bound - final) < 1e-9, (bound, final)

    print("test_fast_simulations OK")


if __name__ == "__main__":
    main()
