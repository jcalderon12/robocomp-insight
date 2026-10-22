"""Offline test: scene reconstruction and verdict (~15 s).

  * the replay heading: the DSR robot frame is +y-forward, the URDF +x-forward;
  * the observed effect (robot->bottle RT deletion) and the horizon cut after it;
  * the anomaly interval (the episode's, or the same rule on the recording), the simulation seeded
    from the recorded state before it, and the footprint the robot covered in it;
  * the verdict (docs_output/diseno/veredicto.md): one discrepancy for every hypothesis, posterior
    with the same prior per mechanism, acceptance at P >= 0.9 or abstention (below the threshold,
    the nominal the most probable, the simulation not faithful), invalid repetitions counted as 0,
    the tolerated time shift, and the weighted parameters;
  * the semantic ingestor refusing to consolidate an abstained verdict;
  * the causes simulator following a rotation profile and recording the fall;
  * the episode clock: the rate fitted from the recording, and the simulator
    replaying on it while recording on the recording's clock.

Run from the repo root:  python3 tests/offline/test_scene_and_verdict_safeguards.py
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

import numpy as np

from src.episode_scene import (
    DEFAULT_TRAY_OFFSET,
    ROBOT_FOOTPRINT_RADIUS_M,
    SEED_LEAD_S,
    URDF_BASE_Z,
    build_simulation_scene,
    EFFECT_HORIZON_MARGIN_S,
    episode_clock_rate,
    episode_time_origin_ns,
    extract_scene_poses,
    observed_effect_times,
    simulation_horizon,
    to_urdf_orientation,
)
import src.verdict as verdict_module
from src.verdict import (
    ABSTAIN_BELOW_THRESHOLD,
    ABSTAIN_NOMINAL,
    ABSTAIN_NOT_FAITHFUL,
    NOMINAL_HYPOTHESIS_ID,
    build_verdict,
    write_verdict,
)

NS = 1_000_000_000


# --------------------------------------------------------------------------- #
# a minimal stand-in for episodic_memory_api
# --------------------------------------------------------------------------- #
class Attr:
    def __init__(self, value):
        self.value = value


class Event:
    def __init__(self, timestamp, modification_type, attributes=None):
        self.timestamp = timestamp
        self.modification_type = modification_type
        self.attributes = attributes or {}


class Keyframe:
    def __init__(self, nodes):
        self.nodes_list = nodes


class FakeMemApi:
    ROOM, ROBOT, BOTTLE = 100, 200, 203

    def __init__(self, yaw_deg, deletions_s):
        half = math.radians(yaw_deg) / 2.0
        quaternion = [0.0, 0.0, math.sin(half), math.cos(half)]
        self.robot_rt = [
            Event(1 * NS, "MEA", {"rt_translation": Attr([-3.7, -0.3, 0.02]), "rt_quaternion": Attr(quaternion)}),
            Event(9 * NS, "MEA", {"rt_translation": Attr([-0.4, -0.3, 0.04]), "rt_quaternion": Attr(quaternion)}),
        ]
        self.bottle_rt = [Event(2 * NS, "MEA")] + [Event(int((1 + t) * NS), "DE") for t in deletions_s]
        self.imu = [Event(1 * NS, "MN")] + [Event(int((1 + 0.1 * i) * NS), "MNA") for i in range(1, 200)]
        # Following at 0.4 m/s; the stop is issued 0.1 s after the first deletion.
        stop_s = (deletions_s[0] + 0.1) if deletions_s else 15.0
        self.robot_node = [Event(1 * NS, "MN"),
                           Event(int(1.2 * NS), "MNA", {"robot_ref_adv_speed": Attr(0.4)}),
                           Event(int((1 + stop_s) * NS), "MNA", {"robot_ref_adv_speed": Attr(0.0)})]

    def is_ready(self):
        return True

    def get_keyframe_count(self):
        return 1

    def get_keyframe(self, index):
        return Keyframe([{"name": "room", "id": self.ROOM}, {"name": "robot", "id": self.ROBOT},
                         {"name": "bottle", "id": self.BOTTLE}])

    def get_edge_history(self, from_id, to_id, edge_type):
        if (from_id, to_id) == (self.ROOM, self.ROBOT):
            return self.robot_rt
        if (from_id, to_id) == (self.ROBOT, self.BOTTLE):
            return self.bottle_rt
        return []

    def get_node_history_by_name(self, name):
        if name == "robot":
            return self.robot_node
        return self.imu if name == "imu" else []


def yaw_deg(quaternion):
    x, y, z, w = quaternion
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def test_heading_convention():
    # The differential shadow URDF drives along +y, like the DSR frame: a recorded yaw of -90 deg
    # (robot advancing along world +x) stays -90 deg. yaw_offset=pi/2 is what the four-wheel URDF needed.
    assert abs(yaw_deg(to_urdf_orientation([0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)])) + 90.0) < 1e-6
    assert abs(yaw_deg(to_urdf_orientation([0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)], math.pi / 2))) < 1e-6

    api = FakeMemApi(yaw_deg=-90.0, deletions_s=[])
    poses = extract_scene_poses(api, [0.0] * 3, [0.0] * 3, [0.0] * 3)
    assert abs(yaw_deg(poses.initial_robot_orientation) + 90.0) < 1e-6, poses.initial_robot_orientation
    # The URDF base rests on the floor whatever z the Webots body was recorded at (0.02 m).
    assert poses.initial_robot_position == [-3.7, -0.3, URDF_BASE_Z], poses.initial_robot_position
    # The tray offset is expressed in the URDF body frame: at yaw -90 deg the bottle ends ahead
    # (+x) and to the left (+y) of the robot, as the Webots world places it.
    ox, oy, oz = DEFAULT_TRAY_OFFSET
    expected = [-3.7 + oy, -0.3 - ox, URDF_BASE_Z + oz]
    assert all(abs(a - b) < 1e-9 for a, b in zip(poses.bottle_position, expected)), poses.bottle_position


def test_swapped_axes():
    """The UEx DSR of 08/10 hangs the robot from "root" and writes its position as (y, x) of the
    Webots world, with the Webots orientation: the scene recovers the room axes from the motion."""
    class RootMemApi(FakeMemApi):
        def get_keyframe(self, index):
            return Keyframe([{"name": "root", "id": self.ROOM}, {"name": "robot", "id": self.ROBOT},
                             {"name": "bottle", "id": self.BOTTLE}])

    plain = FakeMemApi(yaw_deg=-90.0, deletions_s=[])
    swapped = RootMemApi(yaw_deg=-90.0, deletions_s=[])
    for event in swapped.robot_rt:
        x, y, z = event.attributes["rt_translation"].value
        event.attributes["rt_translation"] = Attr([y, x, z])
    recorded = extract_scene_poses(plain, [0.0] * 3, [0.0] * 3, [0.0] * 3)
    corrected = extract_scene_poses(swapped, [0.0] * 3, [0.0] * 3, [0.0] * 3)
    assert recorded.sources["axes"] == "recorded" and corrected.sources["axes"] == "swapped_xy", corrected.sources
    for name in ("initial_robot_position", "problem_position", "bottle_position", "initial_robot_orientation"):
        assert getattr(corrected, name) == getattr(recorded, name), name


def test_observed_effect_and_horizon():
    api = FakeMemApi(yaw_deg=-90.0, deletions_s=[14.6, 18.6])
    origin = episode_time_origin_ns(api)
    times = observed_effect_times(api, origin)
    assert [round(t, 3) for t in times] == [14.6, 18.6], times
    assert simulation_horizon(118.0, times[0]) == 14.6 + EFFECT_HORIZON_MARGIN_S
    assert simulation_horizon(10.0, 14.6) == 10.0            # never beyond the recording
    assert simulation_horizon(118.0, None) == 118.0          # no observed effect: whole recording

    # The replay drops the stop the system issued after seeing the bottle go.
    scene, _ = build_simulation_scene(api, [-3.7, -0.3, 0.0325], [0.0, 0.04, 0.001])
    assert scene["observed_effect_time"] == times[0]
    assert scene["list_of_target_velocities"]["adv_speed"] == [0.4], scene["list_of_target_velocities"]
    assert scene["simulation_length"] == min(19.9, times[0] + EFFECT_HORIZON_MARGIN_S)


def test_anomaly_interval_and_seed():
    api = FakeMemApi(yaw_deg=-90.0, deletions_s=[14.6])
    # The episode's interval, when the semantic memory gives it.
    intervals = [{"id": "Interval_baseline", "start_s": 1.0, "end_s": 6.0}, {"id": "Interval_fall", "start_s": 6.0, "end_s": 14.6}]
    scene, info = build_simulation_scene(api, [-3.7, -0.3, 0.0], [0.0, 0.0, 0.001], episode_intervals=intervals)
    assert info["anomaly_interval_source"] == "episode" and scene["anomaly_interval"] == [6.0, 14.6]
    # Seeded SEED_LEAD_S before it, at the start of that step of the recorded world, from the recorded
    # robot (the pose of 0 s, the last one before 4.5 s), at the recorded speed over the clock rate; the
    # bottle on the tray.
    step = scene["world_time_step_s"] / scene["clock_rate"]
    assert 6.0 - SEED_LEAD_S - step < scene["start_time_s"] <= 6.0 - SEED_LEAD_S + 1e-9, scene["start_time_s"]
    assert abs(scene["start_time_s"] / step - round(scene["start_time_s"] / step)) < 1e-6
    assert scene["initial_robot_position"][:2] == [-3.7, -0.3]
    # In the interval the robot was at its pose of 8 s, (-0.4; -0.3): the footprint is that plus its reach.
    footprint = scene["anomaly_footprint"]
    assert all(abs(a - b) < 1e-9 for a, b in zip(footprint["x"], [-0.4 - ROBOT_FOOTPRINT_RADIUS_M, -0.4 + ROBOT_FOOTPRINT_RADIUS_M])), footprint
    assert all(abs(a - b) < 1e-9 for a, b in zip(footprint["y"], [-0.3 - ROBOT_FOOTPRINT_RADIUS_M, -0.3 + ROBOT_FOOTPRINT_RADIUS_M])), footprint
    # Without the episode, the same rule on the recording; without an observed effect, no seed.
    scene, info = build_simulation_scene(api, [-3.7, -0.3, 0.0], [0.0, 0.0, 0.001])
    assert info["anomaly_interval_source"] == "recording" and abs(scene["anomaly_interval"][1] - 14.6) < 1e-6
    scene, _ = build_simulation_scene(FakeMemApi(yaw_deg=-90.0, deletions_s=[]), [-3.7, -0.3, 0.0], [0.0, 0.0, 0.001])
    assert scene["start_time_s"] == 0.0 and scene["anomaly_interval"] is None


# --------------------------------------------------------------------------- #
# the verdict on synthetic records
# --------------------------------------------------------------------------- #
G = 9.81
DT = 0.01


def _signal(spike_at=None, spike=10.0, width=0.3, length=20.0, axis=1):
    """An IMU record: gravity on z, and a forward acceleration spike."""
    times = np.round(np.arange(0.0, length, DT), 4)
    acc = np.tile([0.0, 0.0, G], (times.size, 1))
    if spike_at is not None:
        acc[(times >= spike_at) & (times < spike_at + width), axis] += spike
    gyro = np.zeros((times.size, 3))
    gyro[:, 2] = 0.01 * np.sin(times)
    xy = np.column_stack([0.5 * times, np.full(times.size, -0.3)])
    return times, acc, gyro, xy


def _record(spike_at=12.0, **kw):
    times, acc, gyro, xy = _signal(spike_at, **kw)
    return {"accelerometer": {"timestamp": times.tolist(), "values": acc.tolist()},
            "gyroscope": {"timestamp": times.tolist(), "values": gyro.tolist()},
            "path": {"timestamp": times.tolist(), "xy": xy.tolist()}}


def _repetition(spike_at=None, effect=None, **kw):
    times, acc, gyro, xy = _signal(spike_at, **kw)
    return {"history": {"timestamp": times.tolist(), "accelerometer": acc.tolist(), "gyroscope": gyro.tolist(),
                        "position": xy.tolist()},
            "generated_instances": {"sample": {"x_m": 0.0 if spike_at is None else spike_at}},
            "bottle_position": [0.0, 0.0, 780.0], "fall_time": None, "effect_time": effect,
            "robot_position_at_fall": None, "robot_final_position": [0.0, 0.0, 30.0]}


SCENE = {"anomaly_interval": [11.0, 13.0], "start_time_s": 9.5, "observed_effect_time": 13.0}


def _entries(*mechanisms):
    entries = [{"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "Nominal", "mechanism": "nominal", "cause": {"name": "none"}}]
    for i, mechanism in enumerate(mechanisms):
        entries.append({"hypothesis_id": f"H{i}", "title": mechanism, "mechanism": mechanism, "cause": {"name": "bump_scaled"}})
    return entries


def test_verdict():
    # Synthetic records have no model error: a small kernel width, so that these checks test the
    # verdict's mechanics and not the provisional width production uses until it is calibrated.
    previous_eps = verdict_module.EPS
    verdict_module.EPS = 0.5
    try:
        return _verdict_checks()
    finally:
        verdict_module.EPS = previous_eps


def _verdict_checks():
    record = _record(12.0)
    nominal = [_repetition(None)]

    # 1. The hypothesis whose repetitions reproduce the record wins and is accepted; the posteriors
    #    add up to 1; the parameters are weighted by how well each repetition fits.
    match = [_repetition(12.0), _repetition(12.05), _repetition(15.0)]
    other = [_repetition(12.0, axis=0)]
    verdict = build_verdict("match", record, _entries("obstacle_traversed", "robot_push"), [nominal, match, other], SCENE)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert abs(sum(h["posterior"] for h in verdict["hypotheses"]) - 1.0) < 1e-9
    assert verdict["accepted_hypothesis_id"] == "H0" and verdict["abstention_reason"] is None, verdict
    assert by_id["H0"]["posterior"] >= 0.9 and by_id["H0"]["passes_rule"] and by_id["H0"]["accepted"]
    assert by_id["H0"]["bayes_factor_vs_nominal"] > 10
    assert abs(by_id["H0"]["parameter_estimate"]["sample.x_m"]["mean"] - 12.0) < 0.1, by_id["H0"]["parameter_estimate"]
    assert verdict["schema_version"] == "2.0" and verdict["anomaly_interval"]["seed_s"] == 9.5

    # 2. The same prior per mechanism: two hypotheses of one mechanism share it.
    verdict = build_verdict("priors", record, _entries("wheel_failure", "wheel_failure", "robot_push"),
                            [nominal, match, match, other], SCENE)
    priors = [h["prior"] for h in verdict["hypotheses"]]
    assert abs(priors[0] - 1 / 3) < 1e-6 and abs(priors[1] - 1 / 6) < 1e-6 and abs(priors[3] - 1 / 3) < 1e-6, priors

    # 3. Two hypotheses that explain the record equally well: neither reaches 0.9, the verdict abstains.
    verdict = build_verdict("tie", record, _entries("obstacle_traversed", "robot_push"), [nominal, match, match], SCENE)
    assert verdict["accepted_hypothesis_id"] is None and verdict["abstention_reason"] == ABSTAIN_BELOW_THRESHOLD

    # 4. A record with nothing anomalous: the nominal is the most probable; nothing is accepted.
    verdict = build_verdict("calm", _record(None), _entries("obstacle_traversed"), [nominal, [_repetition(12.0)]], SCENE)
    assert verdict["most_probable_hypothesis_id"] == NOMINAL_HYPOTHESIS_ID and verdict["accepted_hypothesis_id"] is None
    assert verdict["abstention_reason"] == ABSTAIN_NOMINAL

    # 5. Repetitions that are not physically meaningful count as 0 (and in N).
    overlapping = [dict(_repetition(12.0), spawn_overlap=True)]
    verdict = build_verdict("invalid", record, _entries("obstacle_traversed"), [nominal, overlapping], SCENE)
    hypothesis = verdict["hypotheses"][1]
    assert hypothesis["invalid_repetitions"] == 1 and hypothesis["posterior"] < 1e-6

    # 6. A clock offset of up to 0.3 s between recording and simulation (the whole simulated record
    #    shifted) is tolerated; 0.6 s is a different record.
    def shifted(offset):
        repetition = _repetition(12.0)
        repetition["history"]["timestamp"] = [s + offset for s in repetition["history"]["timestamp"]]
        return [repetition]
    near, far = shifted(0.2), shifted(0.6)
    verdict = build_verdict("shift", record, _entries("a", "b"), [nominal, near, far], SCENE)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert by_id["H0"]["distance"]["min"] < 0.2 < by_id["H1"]["distance"]["min"], (by_id["H0"]["distance"], by_id["H1"]["distance"])

    # 7. The nominal does not follow the record before the interval: the simulation is not faithful.
    previous = verdict_module.ADEQUACY_MAX
    verdict_module.ADEQUACY_MAX = 0.5
    try:
        drifting = [_repetition(10.2)]       # a spike in the lead-in the record does not have
        verdict = build_verdict("faithful", record, _entries("obstacle_traversed"), [drifting, match], SCENE)
    finally:
        verdict_module.ADEQUACY_MAX = previous
    assert verdict["abstention_reason"] == ABSTAIN_NOT_FAITHFUL and verdict["accepted_hypothesis_id"] is None
    assert verdict["adequacy"]["faithful"] is False
    return build_verdict("tie", record, _entries("obstacle_traversed", "robot_push"), [nominal, match, match], SCENE)


def test_ingestor_refuses_abstained_verdict(abstained_verdict):
    sys.path.insert(0, str(REPO / "agents" / "semantic"))
    for name in [m for m in sys.modules if m == "src" or m.startswith("src.")]:
        del sys.modules[name]                      # the semantic agent has its own `src`
    sys.path.remove(str(INNER))
    from src.verdict_ingestor import ingest_verdict

    workdir = Path(tempfile.mkdtemp(prefix="insight_safeguards_"))
    # The verdict abstained (no hypothesis reached P >= 0.9): nothing is consolidated.
    path = workdir / "verdict_abstained.json"
    path.write_text(json.dumps(abstained_verdict), encoding="utf-8")
    ingestion = ingest_verdict(path)
    assert ingestion.ok and not ingestion.triples and ingestion.accepted_hypothesis_id is None, ingestion
    assert abstained_verdict["abstention_reason"] in ingestion.reason, ingestion.reason
    # A verdict written before the simulator abstained on its own: flag set, id still there.
    legacy = dict(abstained_verdict, accepted_hypothesis_id="H0", abstention_reason=None, nominal_effect_warning=True)
    path = workdir / "verdict_legacy.json"
    path.write_text(json.dumps(legacy), encoding="utf-8")
    ingestion = ingest_verdict(path)
    assert ingestion.ok and not ingestion.triples and ingestion.accepted_hypothesis_id is None, ingestion


def test_simulator_rotation_and_fall():
    import pybullet as p
    from causes_simulator import CausesSimulator
    from src.logger import Logger

    workdir = Path(tempfile.mkdtemp(prefix="insight_safeguards_sim_"))
    scene = {
        "gravity": -9.81,
        "initial_robot_position": [0.0, 0.0, 0.0], "initial_robot_orientation": [0.0, 0.0, 0.0, 1.0],
        "problem_position": [0.0, 0.0, 0.0], "problem_orientation": [0.0, 0.0, 0.0, 1.0],
        "bottle_position": list(DEFAULT_TRAY_OFFSET), "bottle_orientation": [0.0, 0.0, 0.0, 1.0],
        "simulation_length": 2.0, "num_of_repetitions": 1,
        "list_of_target_velocities": {"timestamp": [0.0], "adv_speed": [0.0]},
        "list_of_target_rot_speeds": {"timestamp": [0.0], "rot_speed": [0.6]},
    }
    scene_path = workdir / "scene.json"
    scene_path.write_text(json.dumps(scene), encoding="utf-8")
    read_fd, write_fd = os.pipe()
    try:
        simulator = CausesSimulator(json.dumps({"cause": {"name": "none"}}), str(scene_path), write_fd,
                                    Logger(str(workdir / "sim.log")), real_time=False, gui=False)
        assert simulator.get_target_rot_speed(1.0) == 0.6
        # A 10 cm bump placed under the robot overlaps it; 2 m ahead it does not.
        bump = str(REPO / "etc/URDFs/bump/bump_100x10cm.urdf")
        simulator.instantiate_body(bump, [0.0, 0.0, 0.001])
        assert simulator.spawned_bodies_overlap()
        simulator.clean_bodies()
        simulator.instantiate_body(bump, [2.0, 0.0, 0.001])
        assert not simulator.spawned_bodies_overlap()
        simulator.clean_bodies()
        simulator.doSimulations()
        repetition = simulator.historical[0]
    finally:
        os.close(read_fd)
        os.close(write_fd)
        p.disconnect()
    # Commanded to turn in place at 0.6 rad/s: the simulated gyroscope must see a
    # sustained yaw rate (doSimulations restores the initial pose afterwards, so the
    # final orientation cannot be checked).
    history = repetition["history"]
    yaw_rates = [abs(sample[2]) for t, sample in zip(history["timestamp"], history["gyroscope"]) if t > 1.0]
    assert sum(yaw_rates) / len(yaw_rates) > 0.2, sum(yaw_rates) / len(yaw_rates)
    assert repetition["fall_time"] is None and "robot_final_position" in repetition


def _run_scene(scene: dict) -> dict:
    import pybullet as p
    from causes_simulator import CausesSimulator
    from src.logger import Logger

    workdir = Path(tempfile.mkdtemp(prefix="insight_clock_sim_"))
    scene_path = workdir / "scene.json"
    scene_path.write_text(json.dumps(scene), encoding="utf-8")
    read_fd, write_fd = os.pipe()
    try:
        simulator = CausesSimulator(json.dumps({"cause": {"name": "none"}}), str(scene_path), write_fd,
                                    Logger(str(workdir / "sim.log")), real_time=False, gui=False)
        simulator.doSimulations()
        return simulator.historical[0]
    finally:
        os.close(read_fd)
        os.close(write_fd)
        p.disconnect()


def test_episode_clock():
    # The recording's robot covers what the setpoint orders: rate ~1 (the real robot).
    api = FakeMemApi(yaw_deg=-90.0, deletions_s=[14.6])
    rate, windows = episode_clock_rate(api, episode_time_origin_ns(api), 14.6)
    assert abs(rate - 3.3 / (8 * 0.4)) < 0.01 and windows == 7, (rate, windows)
    # It covers half of it, as under the Webots clock; the scene carries the rate.
    api.robot_rt[1] = Event(9 * NS, "MEA", {"rt_translation": Attr([-3.7 + 0.5 * 0.4 * 8, -0.3, 0.04]),
                                            "rt_quaternion": Attr([0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)])})
    scene, info = build_simulation_scene(api, [-3.7, -0.3, 0.0325], [0.0, 0.04, 0.001])
    assert abs(scene["clock_rate"] - 0.5) < 1e-6 and info["clock_rate_windows"] == 7, (scene["clock_rate"], info)
    # A robot that is never told to move gives no window: the recording's own clock.
    api.robot_node = api.robot_node[:1]
    assert episode_clock_rate(api, episode_time_origin_ns(api), 14.6) == (1.0, 0)

    # Replayed at rate 0.5, the same horizon on the recording's clock takes half the
    # physics steps and the robot covers half the path.
    base = {
        "gravity": -9.81,
        # Heading along world +x (yaw -90 deg), with the bottle on the tray ahead of the robot.
        "initial_robot_position": [0.0, 0.0, 0.0], "initial_robot_orientation": [0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)],
        "problem_position": [0.0, 0.0, 0.0], "problem_orientation": [0.0, 0.0, 0.0, 1.0],
        "bottle_position": [DEFAULT_TRAY_OFFSET[1], -DEFAULT_TRAY_OFFSET[0], DEFAULT_TRAY_OFFSET[2]],
        "bottle_orientation": [0.0, 0.0, 0.0, 1.0],
        "simulation_length": 4.0, "num_of_repetitions": 1,
        "list_of_target_velocities": {"timestamp": [0.0], "adv_speed": [0.4]},
    }
    runs = {rate: _run_scene({**base, "clock_rate": rate}) for rate in (1.0, 0.5)}
    from causes_simulator import DEFAULT_PHYSICS_DT
    dt = DEFAULT_PHYSICS_DT
    # One sample per physics step of the recorded world (Webots' default 32 ms here), as the recording has.
    stride = round(0.032 / dt)
    for rate, repetition in runs.items():
        stamps = repetition["history"]["timestamp"]
        assert len(stamps) == math.ceil(4.0 * rate / dt) // stride, (rate, len(stamps))
        # Each sample is the state after a step, stamped with the instant the step ends.
        assert abs(stamps[0] - stride * dt / rate) < 1e-9, (rate, stamps[0])
        assert 4.0 - stride * dt / rate - 1e-9 <= stamps[-1] < 4.0 + dt / rate, (rate, stamps[-1])
    travelled = {rate: run["robot_final_position"][0] for rate, run in runs.items()}
    assert abs(travelled[0.5] / travelled[1.0] - 0.5) < 0.1, travelled


def main():
    test_heading_convention()
    test_swapped_axes()
    test_observed_effect_and_horizon()
    test_anomaly_interval_and_seed()
    abstained = test_verdict()
    test_simulator_rotation_and_fall()
    test_episode_clock()
    test_ingestor_refuses_abstained_verdict(abstained)   # last: it swaps `src` for the semantic agent's
    print("test_scene_and_verdict_safeguards OK")


if __name__ == "__main__":
    main()
