"""Offline test: the inner simulator waits for the semantic agent's batch (change 2 of 06/10, ~1 s).

The simulator and the semantic agent see the closed recording at the same time. Before this
change the simulator ran its static causes (src/causes.json) at once and the batch afterwards; in
Webots (06/10, 13:01) the batch came 4 s later and the static round took 34 s, for a verdict the
semantic agent ignores. Now, while the agent's "unexplained" intention node is in the work graph
and no batch is out, the simulator waits, up to Simulation.BatchWaitSeconds.

The simulator's own check_for_problems runs on simulated DSR graphs (the episodic memory API and
the scene writer are stubbed: what is checked is which causes get selected). Criterion, fixed
before (docs_output/resultados/14_esperar_lote/CRITERIO.md):
  1. intention node and batch in time: it waits without simulating, then simulates the batch
     alone, never the static causes;
  2. intention node and no batch: the static causes once the wait is over;
  3. no intention node: the static causes at once, as before;
  4. a batch that comes after the static causes is still simulated.

Needs PySide6, pydsr (/opt/robocomp/lib), Ice and RoboComp's ConfigLoader, as the simulator does;
without them the test is skipped. The batch of 06/10 13:01 is not in git
(agents/semantic/generated_hypotheses/): it travels in the hand-over package.

Run from the repo root:  python3 tests/offline/test_inner_simulator_waits_for_batch.py
"""
import os
import sys
import time
import types
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"
BATCH = REPO / "agents" / "semantic" / "generated_hypotheses" / "semantic_unexplained_20261006T110252Z.json"
for path in (Path("/home/javi/robocomp/core/classes/ConfigLoader"), INNER / "generated", Path("/opt/robocomp/lib"),
             INNER / "src", INNER):
    sys.path.insert(0, str(path))
os.chdir(INNER)

try:
    from PySide6 import QtCore  # noqa: F401  (before pydsr: both load Qt)
    import pydsr  # noqa: F401
    from src import specificworker
except ImportError as error:
    print(f"SKIP: the inner simulator cannot be imported here ({error})")
    sys.exit(0)

RECORDING = "/recordings/mission_Follow_Person_06102026_130107.txt"


# --------------------------------------------------------------------------- #
# A simulated DSR
# --------------------------------------------------------------------------- #
def node(name, node_id, **attrs):
    return types.SimpleNamespace(name=name, id=node_id,
                                 attrs={k: types.SimpleNamespace(value=v) for k, v in attrs.items()})


class FakeGraph:
    def __init__(self, *nodes):
        self.nodes = {n.name: n for n in nodes}

    def get_node(self, key):
        return next((n for n in self.nodes.values() if key in (n.name, n.id)), None)

    def get_nodes(self):
        return list(self.nodes.values())


class Logger:
    def __init__(self):
        self.lines = []

    def log(self, message, style=None):
        self.lines.append(message)


WORKER_METHODS = ("check_for_problems", "load_hypotheses_causes", "waiting_for_batch", "retire_batch")


def make_worker(batch_wait_s, with_intention_node):
    """The simulator's state once the follow mission has stopped and its recording is closed."""
    episodic = FakeGraph(node("Search Problem Cause_1", 50, status="running"),
                         node("Follow Person_1", 51, filepath=RECORDING))
    work = FakeGraph(node("robot", 1), *([node(specificworker.UNEXPLAINED_NODE, 60)] if with_intention_node else []))
    worker = types.SimpleNamespace(
        graphs={"work": work, "episodic": episodic}, logger=Logger(), state="IDLE",
        hypotheses_compiled=None, hypotheses_path=None, causes_data=None,
        processed_hypotheses_paths=set(), processed_episode_paths=set(), mem_api_path=None, mem_api=None,
        actual_time=time.time(), batch_wait_s=batch_wait_s, batch_wait_started={}, scenes_written=0)
    for name in WORKER_METHODS:
        setattr(worker, name, types.MethodType(getattr(specificworker.SpecificWorker, name), worker))
    worker.writeSimulationScene = lambda: setattr(worker, "scenes_written", worker.scenes_written + 1)
    return worker


def tick(worker, rounds):
    """One compute tick in IDLE; a selected round is recorded and closed as SIMULATE_REASON does."""
    if worker.check_for_problems():
        rounds.append("batch" if worker.hypotheses_compiled is not None else "static")
        worker.retire_batch()


def publish_batch(worker):
    unexplained = worker.graphs["work"].get_node(specificworker.UNEXPLAINED_NODE)
    unexplained.attrs[specificworker.HYPOTHESES_FILEPATH_ATTR] = types.SimpleNamespace(value=str(BATCH))


# --------------------------------------------------------------------------- #
# checks
# --------------------------------------------------------------------------- #
def check_batch_in_time():
    worker, rounds = make_worker(batch_wait_s=60.0, with_intention_node=True), []
    for _ in range(5):
        tick(worker, rounds)
    assert rounds == [] and worker.scenes_written == 0, rounds
    publish_batch(worker)
    for _ in range(5):
        tick(worker, rounds)
    assert rounds == ["batch"], rounds
    assert worker.scenes_written == 1
    assert any("waiting up to 60 s" in line for line in worker.logger.lines), worker.logger.lines
    print("OK check_batch_in_time")


def check_no_batch_then_late_batch():
    worker, rounds = make_worker(batch_wait_s=0.2, with_intention_node=True), []
    tick(worker, rounds)
    assert rounds == [], rounds
    time.sleep(0.25)
    for _ in range(3):
        tick(worker, rounds)
    assert rounds == ["static"], rounds
    assert any("No hypotheses batch after" in line for line in worker.logger.lines), worker.logger.lines
    # The batch comes after the static round: it is simulated too, as before the change.
    publish_batch(worker)
    for _ in range(3):
        tick(worker, rounds)
    assert rounds == ["static", "batch"], rounds
    print("OK check_no_batch_then_late_batch")


def check_without_intention_node():
    worker, rounds = make_worker(batch_wait_s=60.0, with_intention_node=False), []
    for _ in range(3):
        tick(worker, rounds)
    assert rounds == ["static"], rounds
    print("OK check_without_intention_node")


def check_config_default():
    config = (INNER / "etc" / "config").read_text(encoding="utf-8")
    assert "Simulation.BatchWaitSeconds = 120" in config
    assert specificworker.BATCH_WAIT_DEFAULT_S == 120.0
    print("OK check_config_default")


def main():
    if not BATCH.exists():
        print(f"SKIP: {BATCH} not found (it travels in the hand-over package)")
        return
    # The episodic memory API is not needed to choose the causes.
    specificworker.mem = types.SimpleNamespace(EpisodicMemoryAPI=lambda path: types.SimpleNamespace(is_ready=lambda: True))
    check_batch_in_time()
    check_no_batch_then_late_batch()
    check_without_intention_node()
    check_config_default()
    print("Inner simulator waits for the batch: all checks passed")


if __name__ == "__main__":
    main()
