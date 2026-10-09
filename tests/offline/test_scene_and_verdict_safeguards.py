"""Offline test: scene reconstruction and verdict safeguards (~10 s).

  * the replay heading: the DSR robot frame is +y-forward, the URDF +x-forward;
  * the observed effect (robot->bottle RT deletion) and the horizon cut after it;
  * the anomaly window anchored on the observed effect, not on a later idle spike;
  * the verdict: abstention when the nominal run reproduces the effect, discarded
    blown-up repetitions and implausible landings, deterministic tie-break;
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

from src.episode_scene import (
    DEFAULT_TRAY_OFFSET,
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
from src.verdict import (
    ABSTAIN_NOMINAL_REPRODUCES_EFFECT,
    NOMINAL_HYPOTHESIS_ID,
    anomaly_window,
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


def _imu(spikes: dict[float, float], length_s: float = 50.0, dt: float = 0.1) -> dict:
    times = [round(i * dt, 3) for i in range(int(length_s / dt))]
    acc = [[0.0, 0.0, 9.81 + spikes.get(t, 0.0)] for t in times]
    return {"timestamp": times, "accelerometer": acc, "gyroscope": [[0.0, 0.0, 0.0]] * len(times)}


def test_window_anchored_on_observed_effect():
    # The fall shakes the IMU at 12 s; a bigger, unrelated spike happens at 40 s
    # while the robot waits stopped (episode 24092026_133323).
    imu = _imu({12.0: 3.0, 40.0: 6.0})
    start, end, source = anomaly_window(imu, observed_effect_time=12.4)
    assert source == "imu_peak_near_observed_effect" and abs((start + end) / 2 - 12.0) < 1e-6, (start, end)
    start, end, source = anomaly_window(imu)
    assert source == "imu_peak" and abs((start + end) / 2 - 40.0) < 1e-6, (start, end)
    # No IMU activity near the effect: centred on the observed instant.
    start, end, source = anomaly_window(_imu({40.0: 6.0}), observed_effect_time=12.4)
    assert source == "observed_effect_time" and abs((start + end) / 2 - 12.4) < 1e-6, (start, end)


def _repetition(imu, bottle_mm, fall_time=None, robot_at_fall=None, robot_final=(0.0, 0.0, 30.0)):
    return {"history": imu, "generated_instances": {}, "bottle_position": list(bottle_mm),
            "fall_time": fall_time, "robot_position_at_fall": list(robot_at_fall) if robot_at_fall else None,
            "robot_final_position": list(robot_final)}


ENTRIES = [
    {"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "Nominal", "cause": {"name": "none"}},
    {"hypothesis_id": "B_late", "title": "", "cause": {"name": "bump"}},
    {"hypothesis_id": "A_early", "title": "", "cause": {"name": "bump"}},
]
UPRIGHT = (0.0, 0.0, 780.0)
FALLEN_NEAR = (500.0, 0.0, 25.0)


def test_verdict_safeguards():
    imu = _imu({12.0: 3.0}, length_s=20.0)
    nominal = [_repetition(imu, UPRIGHT)]

    # 1. Tie-break: identical IMU (identical score); the fall closer to the
    #    observed instant wins, whatever the list order (both within the span).
    late = [_repetition(imu, FALLEN_NEAR, fall_time=12.75, robot_at_fall=(0.0, 0.0, 30.0))]
    early = [_repetition(imu, FALLEN_NEAR, fall_time=12.5, robot_at_fall=(0.0, 0.0, 30.0))]
    verdict = build_verdict("tie", imu, ENTRIES, [nominal, late, early], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    assert verdict["accepted_hypothesis_id"] == "A_early", verdict["accepted_hypothesis_id"]
    assert verdict["abstention_reason"] is None

    # 2. Blown-up and implausible repetitions reproduce no effect.
    through_floor = [_repetition(imu, (-15301.0, 3986.0, -280494.0), fall_time=5.0, robot_at_fall=(0.0, 0.0, 30.0))]
    flung = [_repetition(imu, (9548.0, 7591.0, 24.0), fall_time=5.0, robot_at_fall=(0.0, 0.0, 30.0))]
    verdict = build_verdict("physics", imu, ENTRIES, [nominal, through_floor, flung], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert by_id["B_late"]["invalid_repetitions"] == 1 and by_id["B_late"]["effect_rate"] == 0.0
    assert by_id["A_early"]["implausible_effects"] == 1 and by_id["A_early"]["effect_rate"] == 0.0
    assert verdict["accepted_hypothesis_id"] is None

    # 2b. An obstacle placed overlapping the robot: the repetition is discarded.
    overlapping = [dict(_repetition(imu, FALLEN_NEAR, fall_time=1.9, robot_at_fall=(0.0, 0.0, 30.0)),
                        spawn_overlap=True)]
    verdict = build_verdict("overlap", imu, ENTRIES, [nominal, overlapping, early], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert by_id["B_late"]["invalid_repetitions"] == 1 and by_id["B_late"]["effect_rate"] == 0.0
    assert verdict["accepted_hypothesis_id"] == "A_early"

    # 2c. The fall must happen when the recording says the bottle left: within
    #     [t_obs - 3, t_obs + 0.5]. Too early or too late, it is not the observed effect.
    too_late = [_repetition(imu, FALLEN_NEAR, fall_time=13.2, robot_at_fall=(0.0, 0.0, 30.0))]
    too_early = [_repetition(imu, FALLEN_NEAR, fall_time=9.0, robot_at_fall=(0.0, 0.0, 30.0))]
    verdict = build_verdict("timing", imu, ENTRIES, [nominal, too_late, too_early], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert all(by_id[h]["effect_rate"] == 0.0 and by_id[h]["mistimed_effects"] == 1 for h in ("A_early", "B_late"))
    assert verdict["accepted_hypothesis_id"] is None
    assert verdict["decision_rule"]["fall_span_s"] == [12.3 - 3.0, 12.3 + 0.5]
    in_span = [_repetition(imu, FALLEN_NEAR, fall_time=9.4, robot_at_fall=(0.0, 0.0, 30.0))]
    verdict = build_verdict("timing", imu, ENTRIES, [nominal, too_late, in_span], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    assert verdict["accepted_hypothesis_id"] == "A_early", verdict["accepted_hypothesis_id"]
    # Without an observed effect there is no span.
    verdict = build_verdict("no_time", imu, ENTRIES, [nominal, too_late, too_early], initial_bottle_z=780.0)
    assert all(h["mistimed_effects"] == 0 for h in verdict["hypotheses"]) and verdict["decision_rule"]["fall_span_s"] is None

    # 3. The nominal run reproduces the effect: abstain, but keep the per-hypothesis outcome.
    nominal_falls = [_repetition(imu, FALLEN_NEAR, fall_time=12.0, robot_at_fall=(0.0, 0.0, 30.0))]
    verdict = build_verdict("nominal", imu, ENTRIES, [nominal_falls, late, early], initial_bottle_z=780.0,
                            observed_effect_time=12.3)
    assert verdict["nominal_effect_warning"] is True
    assert verdict["abstention_reason"] == ABSTAIN_NOMINAL_REPRODUCES_EFFECT
    assert verdict["accepted_hypothesis_id"] is None
    assert not any(h["accepted"] for h in verdict["hypotheses"])
    assert any(h["passes_rule"] for h in verdict["hypotheses"])
    return verdict


def test_ingestor_refuses_abstained_verdict(abstained_verdict):
    sys.path.insert(0, str(REPO / "agents" / "semantic"))
    for name in [m for m in sys.modules if m == "src" or m.startswith("src.")]:
        del sys.modules[name]                      # the semantic agent has its own `src`
    sys.path.remove(str(INNER))
    from src.verdict_ingestor import ingest_verdict

    workdir = Path(tempfile.mkdtemp(prefix="insight_safeguards_"))
    # A verdict written before the simulator abstained on its own: flag set, id still there.
    legacy = dict(abstained_verdict, accepted_hypothesis_id="A_early", abstention_reason=None)
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
    yaw_rates = [abs(sample[2]) for sample in repetition["history"]["gyroscope"][-62:]]
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
    dt = 1.0 / 62.0
    for rate, repetition in runs.items():
        stamps = repetition["history"]["timestamp"]
        assert len(stamps) == math.ceil(4.0 * rate / dt), (rate, len(stamps))
        assert 4.0 - 2 * dt / rate < stamps[-1] + dt / rate <= 4.0 + 1e-9, (rate, stamps[-1])
    travelled = {rate: run["robot_final_position"][0] for rate, run in runs.items()}
    assert abs(travelled[0.5] / travelled[1.0] - 0.5) < 0.1, travelled


def main():
    test_heading_convention()
    test_swapped_axes()
    test_observed_effect_and_horizon()
    test_window_anchored_on_observed_effect()
    abstained = test_verdict_safeguards()
    test_simulator_rotation_and_fall()
    test_episode_clock()
    test_ingestor_refuses_abstained_verdict(abstained)   # last: it swaps `src` for the semantic agent's
    print("test_scene_and_verdict_safeguards OK")


if __name__ == "__main__":
    main()
