"""Offline test: the bump of unknown size in the causes simulator (spawn_scaled_dome, contracts 1.5).

Checks:
  * the catalog entry compiles to the `bump_scaled` cause, which the simulator finds among its causes
    and validates;
  * its 36 draws (Latin hypercube, fixed by the seed) cover the size ranges and reach the area with
    the dome's edge: diameter and height in their strata, centre within the area dilated by half
    the drawn diameter;
  * each repetition spawns the catalog dome scaled to the drawn size, on the floor, as a body the
    simulator cleans up and checks for overlap;
  * run as production runs a cause (a separate process of causes_simulator.py that answers through
    a pipe), every repetition comes back with the size and centre it drew.

Run from the repo root:  python3 tests/offline/test_scaled_dome.py
(requires pybullet; runs headless)
"""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"
os.chdir(INNER)
sys.path.insert(0, str(INNER))
sys.path.insert(0, str(INNER / "src"))

import pybullet as p  # noqa: E402
from causes_simulator import CauseWrapper, CausesSimulator  # noqa: E402
from hypothesis_compiler import compile_hypothesis, load_catalog  # noqa: E402
from src.logger import Logger  # noqa: E402

WORKDIR = Path(tempfile.mkdtemp(prefix="insight_scaled_dome_test_"))
# Meters, as the causes simulator expects (see SimulationScene); the robot drives along y = -0.3 m
# (yaw -90 deg heads the differential URDF along world +x), the bottle on its tray.
SCENE = {
    "gravity": -9.81,
    "initial_robot_position": [-3.7, -0.3, 0.0],
    "initial_robot_orientation": [0.0, 0.0, -0.7071067811865476, 0.7071067811865476],
    "problem_position": [-1.7, -0.3, 0.03],
    "problem_orientation": [0.0, 0.0, 0.0, 1.0],
    "simulation_length": 1.0,
    "list_of_target_velocities": {"timestamp": [0.0], "adv_speed": [0.5]},
    "num_of_repetitions": 2,
    "bottle_position": [-3.545, -0.19, 0.846],
    "bottle_orientation": [0.0, 0.0, 0.0, 0.0],
}
AREA = {"x": [-3.0, -2.0], "y": [-0.65, 0.05], "z": [0.0, 0.0]}


def compiled_cause(**changes):
    hypothesis = {"hypothesis_id": "H01", "testable": True, "simulation_blueprint": {
        "intervention": "spawn_scaled_dome",
        "parameters": {"area": AREA, "diameter_range": [0.1, 1.0], "height_range": [0.01, 0.1]},
        "activation_window": None}}
    return {**compile_hypothesis(hypothesis, load_catalog()), **changes}


def check_compiled_and_drawn():
    payload = compiled_cause()
    assert payload["name"] == "bump_scaled" and Path(payload["mesh_file"]).exists(), payload
    cause = CauseWrapper.model_validate_json(json.dumps({"cause": payload})).cause
    assert type(cause).__name__ == "CauseBumpScaled" and cause.num_of_repetitions == 36
    samples = cause.samples
    # One draw per stratum of size: the 36 draws cover the ranges evenly, with no gap wider than two
    # strata (the draws are rounded to 0.1 mm, so one can sit on the edge of the next stratum).
    for key, (low, high) in (("diameter_m", (0.1, 1.0)), ("height_m", (0.01, 0.1))):
        values = sorted(s[key] for s in samples)
        stratum = (high - low) / 36
        assert low <= values[0] <= low + stratum and high - stratum <= values[-1] <= high, (key, values)
        assert max(b - a for a, b in zip(values, values[1:])) <= 2 * stratum, (key, values)
    for s in samples:
        half = s["diameter_m"] / 2
        assert AREA["x"][0] - half <= s["x_m"] <= AREA["x"][1] + half, s
        assert AREA["y"][0] - half <= s["y_m"] <= AREA["y"][1] + half, s
    # Fixed by the seed; another seed draws others.
    assert CauseWrapper.model_validate_json(json.dumps({"cause": payload})).cause.samples == samples
    assert CauseWrapper.model_validate_json(json.dumps({"cause": {**payload, "seed": 1}})).cause.samples != samples


def scene_file():
    path = WORKDIR / "sim_scene.json"
    path.write_text(json.dumps(SCENE), encoding="utf-8")
    return str(path)


def check_spawned_dome_has_the_drawn_size():
    samples = [{"diameter_m": 0.8, "height_m": 0.06, "x_m": -2.5, "y_m": -0.3},
               {"diameter_m": 0.2, "height_m": 0.02, "x_m": -2.5, "y_m": -0.9}]
    payload = compiled_cause(samples=samples)
    read_fd, write_fd = os.pipe()
    try:
        simulator = CausesSimulator(json.dumps({"cause": payload}), scene_file(), write_fd,
                                    Logger(str(WORKDIR / "sim.log")), real_time=False, gui=False)
        assert simulator.num_of_repetitions == 2
        for repetition, sample in enumerate(samples):
            simulator.current_repetition = repetition
            simulator.current_cause.apply(simulator.engine_wrapper)
            [body] = simulator.loaded_bodies
            low, high = p.getAABB(body)
            assert abs((high[0] - low[0]) - sample["diameter_m"]) < 0.01, (low, high)
            assert abs((high[1] - low[1]) - sample["diameter_m"]) < 0.01, (low, high)
            assert abs((high[2] - low[2]) - sample["height_m"]) < 0.005, (low, high)
            # On the floor (z = 1 mm); PyBullet's bounding box adds its collision margin, ~2 mm.
            assert abs(low[2] - 0.001) <= 0.0025, low
            centre = ((low[0] + high[0]) / 2, (low[1] + high[1]) / 2)
            assert abs(centre[0] - sample["x_m"]) < 0.01 and abs(centre[1] - sample["y_m"]) < 0.01, centre
            assert simulator.current_cause.get_generated_instances() == {"sample": sample}
            simulator.clean_bodies()
    finally:
        os.close(read_fd)
        os.close(write_fd)
        p.disconnect()


def check_as_production_runs_it():
    samples = [{"diameter_m": 0.6, "height_m": 0.05, "x_m": -2.6, "y_m": -0.3},
               {"diameter_m": 0.3, "height_m": 0.03, "x_m": -2.4, "y_m": 0.4}]
    payload = compiled_cause(samples=samples)
    read_fd, write_fd = os.pipe()
    process = subprocess.Popen([sys.executable, "src/causes_simulator.py", "-c", json.dumps({"cause": payload}),
                                "-s", scene_file(), "-p", str(write_fd)], pass_fds=(write_fd,))
    os.close(write_fd)
    with os.fdopen(read_fd) as pipe:
        historical = json.loads(pipe.read())
    assert process.wait(timeout=300) == 0
    assert [rep["generated_instances"] for rep in historical] == [{"sample": s} for s in samples], historical
    assert all(rep["history"]["timestamp"] for rep in historical)


def main():
    for check in (check_compiled_and_drawn, check_spawned_dome_has_the_drawn_size, check_as_production_runs_it):
        check()
        print(f"OK {check.__name__}")
    print("Scaled dome: all checks passed")


if __name__ == "__main__":
    main()
