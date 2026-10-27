"""Offline test: what the final campaign needs the simulator to keep (plan, docs_output/diseno/evaluacion.md, 4.3
and 6.1), without robot, Webots or a simulation.

Criterion, fixed before:
  1. the simulator uses the episode's intervals: the batch cites the sha256 of the canonical JSON of the episode
     (sorted keys, no spaces) and the agent writes the file indented, so the file's own bytes never matched and the
     simulator recomputed the interval; now the canonical sha256 matches, the episode's intervals are used, and a file
     that is not the cited episode is still refused;
  2. next to the verdict, a file with the scene, the record the verdict compared against, and for the nominal and
     every hypothesis the repetition closest to the record (the one ratio_to_nominal uses), with its history: also
     when, as in the fast path, only each chunk's closest repetition keeps its history and the importance weights make
     another one the verdict's best_repetition_index;
  3. the verdict carries its timings (the instant the batch was found, each stage, the instant it was written and
     the total) and the path of that file, and is written after both, then published.

Checks 1 and 3 run the simulator's own methods on a simulated DSR (as test_inner_simulator_waits_for_batch.py), and
need PySide6, pydsr (/opt/robocomp/lib), Ice and RoboComp's ConfigLoader; without them they are skipped. Check 2
only needs the verdict.

Run from the repo root:  python3 tests/offline/test_campaign_records.py
"""
import hashlib
import json
import math
import os
import sys
import tempfile
import time
import types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"
for path in (Path("/home/javi/robocomp/core/classes/ConfigLoader"), INNER / "generated", Path("/opt/robocomp/lib"),
             INNER / "src", INNER):
    sys.path.insert(0, str(path))
os.chdir(INNER)

from src.hypothesis_compiler import NOMINAL_HYPOTHESIS_ID  # noqa: E402
from src.verdict import (DISTANCE, HISTORY, LOG_WEIGHT, build_verdict, simulated_runs,  # noqa: E402
                         write_simulations)

try:
    from PySide6 import QtCore  # noqa: F401  (before pydsr: both load Qt)
    import pydsr  # noqa: F401
    from src import specificworker
except ImportError as error:
    specificworker = None
    SKIPPED = str(error)


# --------------------------------------------------------------------------- #
# 1. The episode's intervals
# --------------------------------------------------------------------------- #
EPISODE = {"episode_id": "rec_test", "schema": "insight.episode/v0",
           "intervals": [{"id": "Interval_baseline", "start_s": 1.0, "end_s": 6.0},
                         {"id": "Interval_fall", "start_s": 12.25, "end_s": 14.5}],
           "time": {"observation_time_s": 14.5}, "evidence": [{"id": "Ev_x", "value": 0.1 + 0.2}]}


def canonical_sha256(episode: dict) -> str:
    """What the batch cites: hypothesis_pipeline.episode_sha256 (episode_rdf.record_sha256)."""
    return hashlib.sha256(json.dumps(episode, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode("utf-8")).hexdigest()


class Logger:
    def __init__(self):
        self.lines = []

    def log(self, message, style=None):
        self.lines.append(message)


def check_episode_intervals(tmp: Path):
    path = tmp / "case_episode.json"
    # As the semantic agent writes it (specificworker._generate_from_episode).
    path.write_text(json.dumps(EPISODE, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    assert hashlib.sha256(path.read_bytes()).hexdigest() != canonical_sha256(EPISODE)   # why it never matched
    worker = types.SimpleNamespace(logger=Logger(), hypotheses_batch={
        "episode": {"id": "rec_test", "path": str(path), "sha256": canonical_sha256(EPISODE)}})
    intervals = types.MethodType(specificworker.SpecificWorker.episode_intervals, worker)
    assert intervals() == EPISODE["intervals"], (intervals(), worker.logger.lines)
    # A file that is not the episode the batch cites is refused.
    changed = dict(EPISODE, intervals=[dict(EPISODE["intervals"][1], start_s=11.0)])
    path.write_text(json.dumps(changed, indent=1) + "\n", encoding="utf-8")
    assert intervals() is None and "does not match" in worker.logger.lines[-1], worker.logger.lines
    # No path, or no batch: None, as before.
    worker.hypotheses_batch = {"episode": {"id": "rec_test", "path": str(tmp / "missing.json"), "sha256": "x"}}
    assert intervals() is None
    worker.hypotheses_batch = None
    assert intervals() is None


# --------------------------------------------------------------------------- #
# 2. The simulations next to the verdict
# --------------------------------------------------------------------------- #
SCENE = {"observed_effect_time": 7.5, "anomaly_interval": [5.5, 7.5], "start_time_s": 4.0, "num_of_repetitions": 3}


def _record():
    t = np.arange(0.0, 10.0, 0.008).tolist()
    rng = np.random.default_rng(0)
    return {"accelerometer": {"timestamp": t, "values": (rng.normal(0, 0.3, (len(t), 3)) + [0, 0, 9.81]).tolist()},
            "gyroscope": {"timestamp": t, "values": rng.normal(0, 0.05, (len(t), 3)).tolist()},
            "path": {"timestamp": t, "xy": [[0.3 * x, 0.0] for x in t]}}


def _history(label: float) -> dict:
    t = np.arange(4.0, 8.0, 0.008).tolist()
    return {"timestamp": t, "accelerometer": [[label, 0.0, 9.81]] * len(t), "gyroscope": [[0.0, 0.0, label]] * len(t),
            "position": [[0.3 * x, 0.0] for x in t]}


def _rep(d, log_weight=0.0, history=None, sample=None):
    rep = {DISTANCE: d, LOG_WEIGHT: log_weight, "generated_instances": {"sample": sample or {"x_m": d}},
           "bottle_position": [0.0, 0.0, 800.0], "robot_final_position": [0.0, 0.0, 30.0], "effect_time": 7.5}
    if history is not None:
        rep[HISTORY] = history
        rep["perception"] = {"timestamp": [7.0], "tilt_deg": [1.0], "in_view": [True]}
    return rep


def _entries():
    return [{"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "Nominal", "mechanism": "nominal", "cause": {"name": "none"}},
            {"hypothesis_id": "H01", "title": "Bump", "mechanism": "obstacle_traversed", "cause": {"name": "bump_scaled"}},
            {"hypothesis_id": "H02", "title": "Push", "mechanism": "robot_push", "cause": {"name": "external_force"}}]


def _historicals():
    """As the fast path returns them: the nominal with its history; in each chunk only the closest repetition keeps
    it. H01's closest (1.5) has a lower importance weight than one a millimetre farther, which the kernel prefers."""
    return [[_rep(3.0, history=_history(0.0))],
            [_rep(1.501, log_weight=2.0), _rep(1.5, log_weight=-3.0, history=_history(1.5)), _rep(3.4)],
            [_rep(2.9), _rep(2.95), _rep(2.8, history=_history(2.8))]]


def check_simulations_file(tmp: Path):
    record, historicals = _record(), _historicals()
    verdict = build_verdict("case_1", record, _entries(), historicals, SCENE)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    # The verdict's best (the largest kernel term) and the closest repetition differ for H01.
    assert by_id["H01"]["best_repetition_index"] == 0 and by_id["H01"]["closest_repetition_index"] == 1, by_id["H01"]
    assert [by_id[h]["closest_repetition_index"] for h in (NOMINAL_HYPOTHESIS_ID, "H02")] == [0, 2]
    # The closest repetition is the one the ratio to the nominal uses.
    for hid, closest in (("H01", 1.5), ("H02", 2.8)):
        assert abs(by_id[hid]["ratio_to_nominal"] - closest / 3.0) < 1e-4, by_id[hid]["ratio_to_nominal"]

    runs = simulated_runs(verdict, historicals, SCENE, record)
    assert runs["case_id"] == "case_1" and runs["scene"] == SCENE and runs["record"] == record
    assert [r["hypothesis_id"] for r in runs["runs"]] == [NOMINAL_HYPOTHESIS_ID, "H01", "H02"]
    for run, label, distance in zip(runs["runs"], (0.0, 1.5, 2.8), (3.0, 1.5, 2.8)):
        assert run["has_history"] and run["repetition"][HISTORY]["gyroscope"][0][2] == label, run
        assert run["distance"] == distance, (run["hypothesis_id"], run["distance"])
        assert run["cause"] == by_id[run["hypothesis_id"]]["cause"] and run["mechanism"]
    assert runs["runs"][1]["repetition"]["generated_instances"] == {"sample": {"x_m": 1.5}}
    assert runs["runs"][1]["repetition"]["perception"]["in_view"] == [True]

    path = write_simulations(runs, str(tmp / "verdicts"))
    assert Path(path).name == "simulations_case_1.json"
    assert json.loads(Path(path).read_text(encoding="utf-8")) == json.loads(json.dumps(runs))
    # The episodic-memory API gives the record in float32 (found running a real case): written as numbers.
    as_api = dict(record, gyroscope={"timestamp": record["gyroscope"]["timestamp"][:2],
                                     "values": [list(np.float32([0.5, 0.25, 0.125]))] * 2})
    path = write_simulations(simulated_runs(verdict, historicals, SCENE, as_api), str(tmp / "verdicts"))
    assert json.loads(Path(path).read_text())["record"]["gyroscope"]["values"][0] == [0.5, 0.25, 0.125]

    # A repetition without history (should the closest have lost it) is said, not invented.
    historicals[2][2].pop(HISTORY)
    verdict = build_verdict("case_2", record, _entries(), historicals, SCENE)
    runs = simulated_runs(verdict, historicals, SCENE, record)
    assert not runs["runs"][2]["has_history"] and HISTORY not in runs["runs"][2]["repetition"]
    # No interval: no hypothesis is evaluated, the scene and the record are kept.
    verdict = build_verdict("case_3", record, _entries(), historicals, {**SCENE, "anomaly_interval": None})
    runs = simulated_runs(verdict, historicals, SCENE, record)
    assert runs["runs"] == [] and runs["record"] == record


# --------------------------------------------------------------------------- #
# 3. The verdict's timings, written with it
# --------------------------------------------------------------------------- #
WORKER_METHODS = ("write_batch_verdict", "retire_batch", "stage_done", "verdict_timings")


def check_verdict_timings(tmp: Path):
    # Templates of detector agents for the accepted causes would go to the repository: not here.
    specificworker.generate_agent = lambda cause_name, folder: True
    published = []
    record, historicals = _record(), _historicals()
    compiled = {"case_id": "case_t", "entries": _entries(), "skipped": []}
    found_at = time.time() - 5.0
    worker = types.SimpleNamespace(
        logger=Logger(), hypotheses_compiled=compiled, hypotheses_path="/batch.json", hypotheses_batch={},
        processed_hypotheses_paths=set(), state="SIMULATE_REASON", sim_processes=4, causes_data=[],
        imu_history={"timestamp": [0.0, 1.0], "accelerometer": [[0, 0, 9.81]] * 2, "gyroscope": [[0, 0, 0]] * 2},
        sim_scene=types.SimpleNamespace(model_dump=lambda: dict(SCENE)),
        case_timings={"batch_found_at": found_at, "stages_s": {"episodic_memory": 0.4, "scene": 0.2}},
        output_root=str(tmp))
    worker.publish_verdict_filepath = lambda path: published.append(path) or True
    for name in WORKER_METHODS:
        setattr(worker, name, types.MethodType(getattr(specificworker.SpecificWorker, name), worker))
    worker.case_timings["stages_s"].update(record=0.1, simulation=30.0)
    started = time.perf_counter()
    verdict_path = worker.write_batch_verdict(record, historicals, started)
    assert published == [verdict_path] and worker.state == "IDLE", (published, worker.state)
    verdict = json.loads(Path(verdict_path).read_text(encoding="utf-8"))
    timings = verdict["timings"]
    stages = timings["stages_s"]
    assert set(stages) == {"episodic_memory", "scene", "record", "simulation", "verdict", "outputs"}, stages
    assert stages["simulation"] == 30.0 and all(v >= 0.0 for v in stages.values())
    assert timings["batch_found_at"] == round(found_at, 3) and timings["verdict_written_at"] >= found_at + 5.0
    assert abs(timings["total_s"] - (timings["verdict_written_at"] - timings["batch_found_at"])) < 2e-3, timings
    assert timings["processes"] == 4
    # The simulations file, next to the verdict, written before it.
    simulations = Path(verdict["simulations_path"])
    assert simulations.parent == Path(verdict_path).parent and simulations.exists()
    assert simulations.stat().st_mtime <= Path(verdict_path).stat().st_mtime
    assert [r["hypothesis_id"] for r in json.loads(simulations.read_text())["runs"]] == [
        NOMINAL_HYPOTHESIS_ID, "H01", "H02"]
    # What the verdict decides is the same as before.
    assert verdict["accepted_hypothesis_id"] == build_verdict(
        "case_t", record, _entries(), historicals, SCENE)["accepted_hypothesis_id"]


def main():
    with tempfile.TemporaryDirectory() as tmp:
        check_simulations_file(Path(tmp))
        if specificworker is None:
            print(f"  (checks 1 and 3 skipped: the inner simulator cannot be imported here: {SKIPPED})")
        else:
            check_episode_intervals(Path(tmp))
            check_verdict_timings(Path(tmp))
    print("test_campaign_records OK")


if __name__ == "__main__":
    main()
