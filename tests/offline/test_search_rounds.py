"""Offline test: the search in rounds and the verdict's adequacy rule (~40 s).

docs_output/resultados/45_busqueda_con_refinado and 46_refinado_con_lineas:
  * the densities the importance weights rest on: a round's proposal (gaussians around the elite, and a share of
    "line" draws that sweep one coordinate) integrates to 1, and so does the scaled bump's prior in metres, which is
    positive at every draw cause_bump_scaled makes;
  * search_rounds.search, driven by a stand-in for the simulator: every cause gets its budget (the prior's draws,
    then the rounds), in order, each repetition with its importance weight; the weights average 1, as an estimate of
    the prior's mass; the rounds come closer to the best than the prior did, and a bump right in its place but too
    large is brought to its size by a line draw;
  * the verdict: the importance weights enter the kernel (a repetition outside the prior counts for nothing), and the
    most probable hypothesis is accepted only if its best repetition leaves at most EXPLANATION_RATIO_MAX of the
    nominal's distance; otherwise it abstains as explanation_not_adequate;
  * the causes simulator, run as the worker runs it: a push returns the unit draws it made, and given back they make
    the same repetitions.

Run from the repo root:  python3 tests/offline/test_search_rounds.py
"""
import json
import math
import os
import random
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"
sys.path.insert(0, str(REPO / "tests" / "offline"))
import test_fast_simulations as tfs   # noqa: E402  (also moves to the inner simulator and sets its paths)

from causes.implementations.cause_bump_scaled import draw_samples   # noqa: E402
from src import search_rounds as sr   # noqa: E402
from src.draws import DRAW_WIDTHS, DRAWS   # noqa: E402
from src.verdict import (ABSTAIN_INADEQUATE, DISTANCE, EXPLANATION_RATIO_MAX, GENERATED_INSTANCES, LOG_WEIGHT,   # noqa: E402
                         NOMINAL_HYPOTHESIS_ID, build_verdict, channels_from_json, channels_to_json, scene_channels)

BUMP = {"name": "bump_scaled", "mesh_file": tfs.MESH, "mesh_dimensions_m": [1.0, 1.0, 0.0994],
        "area_x": [-2.0, -0.7], "area_y": [-0.65, 0.05], "diameter_range": [0.1, 1.0], "height_range": [0.01, 0.1],
        "bounds_x": [-6.0, 3.0], "bounds_y": [-3.0, 3.0], "num_of_repetitions": 1000, "seed": 0}
PUSH = {"name": "external_force", "force_target": "robot", "force_window_start": 0.3, "force_window_end": 0.6,
        "force_x_origin": 0.0, "force_x_range": 600.0, "force_y_origin": 0.0, "force_y_range": 600.0,
        "force_z_origin": 0.0, "force_z_range": 0.0, "num_of_repetitions": 1000}
TRUE_BUMP = np.array([0.25, 0.02, -1.5, -0.56])          # diameter, height, x, y
TRUE_PUSH = np.array([0.3, 0.7, 0.2, 0.8])               # unit draws: duration, onset, fx, fy


def check_densities():
    # a round's proposal integrates to 1 (a 2-D grid, fine against its narrowest gaussian)
    rng = np.random.default_rng(1)
    lows, highs = np.array([0.0, -2.0]), np.array([1.0, 3.0])
    centres = np.column_stack([rng.uniform(0, 1, 6), rng.uniform(-2, 3, 6)])
    proposal = {"kind": "line", "centres": centres, "lows": lows, "highs": highs, "width": 0.05}
    g0 = np.linspace(lows[0], highs[0], 801)
    g1 = np.linspace(lows[1], highs[1], 801)
    grid = np.array(np.meshgrid(g0, g1, indexing="ij")).reshape(2, -1).T
    mass = np.exp(sr.proposal_log_density(proposal, grid)).sum() * (g0[1] - g0[0]) * (g1[1] - g1[0])
    assert abs(mass - 1.0) < 0.01, mass
    # its draws stay within the ranges, and a share sweep a coordinate far from every centre
    draws = sr.propose(rng, centres, lows, highs, 0.05, 2000)
    assert ((draws >= lows) & (draws <= highs)).all()
    # the bump's prior in metres integrates to 1 over the widest box, and is positive at the production draws
    geo = sr.bump_geometry(BUMP)
    box = rng.uniform(geo["lows"], geo["highs"], size=(400_000, 4))
    volume = float(np.prod(geo["highs"] - geo["lows"]))
    mass = np.exp(sr.bump_log_prior(geo, box)).mean() * volume
    assert abs(mass - 1.0) < 0.02, mass
    theta = np.array([sr.theta_of({GENERATED_INSTANCES: {"sample": s}})
                      for s in draw_samples(BUMP["area_x"], BUMP["area_y"], BUMP["diameter_range"], BUMP["height_range"],
                                            500, 3, BUMP["bounds_x"], BUMP["bounds_y"])])
    assert np.isfinite(sr.bump_log_prior(geo, theta)).all()


def _stand_in(spec):
    """What causes_simulator --channels returns, with a distance that depends on how far a repetition is from a
    known answer: a bump right in its place but of another size still comes some way closer than nothing."""
    cause = spec["cause"]
    reps = []
    if cause["name"] == "none":
        return [{DISTANCE: 3.0}]
    if cause["name"] == "bump_scaled":
        samples = cause.get("samples") or draw_samples(cause["area_x"], cause["area_y"], cause["diameter_range"],
                                                       cause["height_range"], cause["num_of_repetitions"], cause["seed"],
                                                       cause["bounds_x"], cause["bounds_y"])
        for s in samples[spec["first"]:spec["first"] + spec["count"]]:
            t = np.array([s["diameter_m"], s["height_m"], s["x_m"], s["y_m"]])
            place = math.hypot(*(t[2:] - TRUE_BUMP[2:]))
            size = abs(t[0] - TRUE_BUMP[0]) / 0.9 + abs(t[1] - TRUE_BUMP[1]) / 0.09
            d = 3.0 if place > t[0] / 2 else 1.0 + 4 * place + 2 * size
            reps.append({DISTANCE: d, GENERATED_INSTANCES: {"sample": s}})
        return reps
    gen = random.Random(spec.get("seed"))
    for row in range(spec["count"]):
        units = spec["draws"][row] if spec.get("draws") is not None else [gen.random() for _ in range(5)]
        u = np.array(units[:4])
        reps.append({DISTANCE: 1.5 + 3 * float(np.abs(u - TRUE_PUSH).sum()), DRAWS: list(units),
                     DRAW_WIDTHS: [0.2, 1.0, 600.0, 600.0, 0.0]})
    return reps


def check_search():
    causes = [{"name": "none", "num_of_repetitions": 1}, BUMP, PUSH]
    calls = []

    def launch(specs):
        calls.append(len(specs))
        return [_stand_in(spec) for spec in specs]

    out = sr.search(causes, lambda c: c["num_of_repetitions"], launch, processes=16, seed=7, log=lambda line: None)
    n0, per_round = sr.plan(1000)
    assert (n0, per_round) == (500, 125) and sr.plan(4000) == (3000, 250)
    assert len(calls) == 1 + len(sr.WIDTHS)                       # round 0 and the rounds
    assert [len(r) for r in out] == [1, 1000, 1000]
    for reps in out[1:]:
        weights = np.array([math.exp(r[LOG_WEIGHT]) if r[LOG_WEIGHT] is not None else 0.0 for r in reps])
        # prior / mixture, averaged over draws from the mixture: the prior's mass, 1
        assert abs(weights.mean() - 1.0) < 0.2, weights.mean()
        d = np.array([r[DISTANCE] for r in reps])
        assert d[n0:].min() < d[:n0].min(), (d[n0:].min(), d[:n0].min())
    assert out[0][0][LOG_WEIGHT] == 0.0
    # the bump: found in its place and brought to its size
    bump = out[1]
    best = min(bump, key=lambda r: r[DISTANCE])[GENERATED_INSTANCES]["sample"]
    assert math.hypot(best["x_m"] + 1.5, best["y_m"] + 0.56) < 0.05, best
    assert abs(best["diameter_m"] - 0.25) < 0.08, best
    # the push: closer to its true draws than the prior got
    push = out[2]
    best = min(push, key=lambda r: r[DISTANCE])
    assert np.abs(np.array(best[DRAWS][:4]) - TRUE_PUSH).sum() < 0.3, best[DRAWS]


def _entries():
    return [{"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "mechanism": "nominal", "cause": {"name": "none"}},
            {"hypothesis_id": "H1", "mechanism": "obstacle_traversed", "cause": {"name": "bump_scaled"}},
            {"hypothesis_id": "H2", "mechanism": "robot_push", "cause": {"name": "external_force"}}]


def _record():
    t = np.arange(0.0, 10.0, 0.008).tolist()
    rng = np.random.default_rng(0)
    return {"accelerometer": {"timestamp": t, "values": (rng.normal(0, 0.3, (len(t), 3)) + [0, 0, 9.81]).tolist()},
            "gyroscope": {"timestamp": t, "values": rng.normal(0, 0.05, (len(t), 3)).tolist()},
            "path": {"timestamp": t, "xy": [[0.3 * x, 0.0] for x in t]}}


def _rep(d, log_weight=0.0):
    return {DISTANCE: d, LOG_WEIGHT: log_weight, GENERATED_INSTANCES: {}, "bottle_position": [0.0, 0.0, 800.0],
            "robot_final_position": [0.0, 0.0, 30.0], "effect_time": 7.5}


def check_verdict():
    scene = {"observed_effect_time": 7.5, "anomaly_interval": [5.5, 7.5], "start_time_s": 4.0}
    record = _record()
    # explains half the discrepancy: accepted
    historicals = [[_rep(3.0)], [_rep(1.5)] + [_rep(3.4)] * 99, [_rep(2.9)] * 100]
    verdict = build_verdict("c1", record, _entries(), historicals, scene)
    assert verdict["accepted_hypothesis_id"] == "H1" and abs(verdict["explained_ratio"] - 0.5) < 1e-9, verdict["explained_ratio"]
    # the most probable, but it leaves 70 % of the discrepancy: not adequate
    historicals = [[_rep(3.0)], [_rep(2.1)] + [_rep(3.4)] * 99, [_rep(2.9)] * 100]
    verdict = build_verdict("c2", record, _entries(), historicals, scene)
    assert verdict["most_probable_hypothesis_id"] == "H1" and verdict["accepted_hypothesis_id"] is None
    assert verdict["abstention_reason"] == ABSTAIN_INADEQUATE and verdict["explained_ratio"] > EXPLANATION_RATIO_MAX
    # the importance weights: H1's best lies outside its prior (weight 0), so H2 is the most probable
    historicals = [[_rep(3.0)], [_rep(1.0, None)] + [_rep(3.4)] * 99, [_rep(1.2)] + [_rep(2.9)] * 99]
    verdict = build_verdict("c3", record, _entries(), historicals, scene)
    assert verdict["most_probable_hypothesis_id"] == "H2", verdict["most_probable_hypothesis_id"]
    h1 = next(h for h in verdict["hypotheses"] if h["hypothesis_id"] == "H1")
    assert h1["ratio_to_nominal"] > 1.0     # its best supported repetition is 3.4


def check_simulator_draws():
    """A push through causes_simulator.py --channels, as the worker launches it."""
    truth = tfs._legacy(tfs._bump([tfs.TRUE_BUMP]))[0]
    scene = tfs._scene()
    workdir = Path(tempfile.mkdtemp(prefix="insight_search_"))
    scene_path, channels_path = workdir / "scene.json", workdir / "channels.json"
    scene_path.write_text(json.dumps(scene))
    channels = scene_channels(tfs._record(truth["history"]), scene)
    channels_path.write_text(json.dumps(channels_to_json(channels)))
    push = {**PUSH, "force_window_start": 1.2 / scene["simulation_length"],
            "force_window_end": 2.4 / scene["simulation_length"], "num_of_repetitions": 3}

    def run(extra):
        read_fd, write_fd = os.pipe()
        process = subprocess.Popen([sys.executable, "src/causes_simulator.py", "-c", json.dumps({"cause": push}),
                                    "-s", str(scene_path), "-p", str(write_fd), "--channels", str(channels_path),
                                    "--first", "0", "--count", "3", *extra], pass_fds=(write_fd,),
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.close(write_fd)
        with os.fdopen(read_fd) as pipe:
            reps = json.loads(pipe.read())
        assert process.wait(timeout=300) == 0
        return reps

    drawn = run(["--seed", "5"])
    assert all(len(r[DRAWS]) == 5 and np.allclose(r[DRAW_WIDTHS], [0.2, 1.0, 600.0, 600.0, 0.0]) for r in drawn), drawn[0][DRAW_WIDTHS]
    draws_path = workdir / "draws.json"
    draws_path.write_text(json.dumps([r[DRAWS] for r in drawn]))
    given = run(["--draws", str(draws_path)])
    assert [r[GENERATED_INSTANCES] for r in given] == [r[GENERATED_INSTANCES] for r in drawn]
    assert [r[DISTANCE] for r in given] == [r[DISTANCE] for r in drawn]
    assert [r[DRAWS] for r in given] == [r[DRAWS] for r in drawn]


def main():
    check_densities()
    check_search()
    check_verdict()
    check_simulator_draws()
    print("test_search_rounds OK")


if __name__ == "__main__":
    main()
