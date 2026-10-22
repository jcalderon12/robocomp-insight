"""Offline test: the simulator fixes of the 15/10 audit (~30 s).

  * the bodies: the robot weighs what the Webots Shadow weighs, the bottle what the MedicineBottle
    of the Webots world weighs;
  * the simulated IMU: robot frame, gravity rotated, mean acceleration over its window, no
    carry-over after a reset;
  * the scene settles before the episode: the robot does not move before its first setpoint,
    and two repetitions of the nominal run are identical;
  * the effect is concept_bottle's rule: 5 frames of 0.1 s with the bottle tilted or out of view;
  * the contacts: Webots' defaults, so the weight splits evenly between drive wheels and casters;
  * the scene from the recording: setpoints timed from the IMU origin, the rotation setpoints,
    the step of the recorded world, and no made-up zeros before the first accelerometer value;
  * the verdict: the effect from `effect_time`, and the IMU spread per sensor without the offsets
    between axes.

Run from the repo root:  python3 tests/offline/test_simulator_fixes.py
"""
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
INNER = REPO / "agents" / "inner_simulator"

os.chdir(INNER)
sys.path.insert(0, str(INNER))
sys.path.insert(0, str(INNER / "src"))

import pybullet as p

from causes_simulator import PERCEPTION_LOST_FRAMES, PERCEPTION_PERIOD_S, CausesSimulator
from pybullet_imu import IMU
from src.episode_scene import (
    DEFAULT_TRAY_OFFSET,
    build_simulation_scene,
    RECORDED_WORLD_TIME_STEP_S,
    commanded_speed_history,
    real_imu_history,
)
from src.logger import Logger
from src.verdict import (
    NOMINAL_HYPOTHESIS_ID,
    build_verdict,
    effect_reproduced,
    plausible_effect,
    sensor_spread,
)

NS = 1_000_000_000
G = 9.81


def _quaternion(axis, angle):
    s = math.sin(angle / 2.0)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)]


def test_bodies():
    p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(str(REPO / "etc/URDFs/shadow/shadow.urdf"), flags=p.URDF_USE_INERTIA_FROM_FILE)
        bottle = p.loadURDF(str(REPO / "etc/URDFs/bottle/bottle.urdf"), flags=p.URDF_USE_INERTIA_FROM_FILE)
        mass = sum(p.getDynamicsInfo(robot, link)[0] for link in range(-1, p.getNumJoints(robot)))
        # Webots: 50 kg body, two 2 kg drive wheels, two 0.2 kg casters; the devices have no mass.
        assert abs(mass - 54.4) < 0.05, mass
        assert abs(p.getDynamicsInfo(bottle, -1)[0] - 0.183) < 1e-6
    finally:
        p.disconnect()


def test_imu_model():
    p.connect(p.DIRECT)
    try:
        p.setGravity(0, 0, -G)
        box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
        dt = 1.0 / 240.0
        # At rest, pitched 10 deg nose up (+y forward, rotation about +x): the accelerometer reads
        # the reaction to gravity in the device frame, g sin(10 deg) on y. Before the fix it read
        # (0, 0, 9.81) whatever the attitude.
        pitch = math.radians(10.0)
        fixed = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box,
                                  baseOrientation=_quaternion([1, 0, 0], pitch))
        accelerometer, gyroscope = IMU(fixed, dt).get_measurement()
        expected = [0.0, G * math.sin(pitch), G * math.cos(pitch)]
        assert np.allclose(accelerometer, expected, atol=1e-6), accelerometer
        assert np.allclose(gyroscope, 0.0, atol=1e-9), gyroscope

        # Yawed 90 deg and spinning about world x: in the device frame that is -y. Before the fix
        # the gyroscope reported world axes.
        p.setGravity(0, 0, 0)
        free = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box, basePosition=[0, 0, 5],
                                 baseOrientation=_quaternion([0, 0, 1], math.pi / 2))
        imu = IMU(free, dt, window_s=2 * dt, gravity=0.0)
        p.resetBaseVelocity(free, linearVelocity=[0.2, 0.0, 0.0], angularVelocity=[1.0, 0.0, 0.0])
        accelerometer, gyroscope = imu.get_measurement()
        assert np.allclose(gyroscope, [0.0, -1.0, 0.0], atol=1e-9), gyroscope
        # The velocity jumped by 0.2 m/s along world x (device -y) inside a two-step window: the mean
        # acceleration over the window, with no low-pass filter delaying it.
        assert np.allclose(accelerometer, [0.0, -0.2 / (2 * dt), 0.0], atol=1e-6), accelerometer

        # After a reset the next sample does not difference against the previous state.
        imu.reset()
        accelerometer, _ = imu.get_measurement()
        assert np.allclose(accelerometer, 0.0, atol=1e-9), accelerometer
    finally:
        p.disconnect()


def _scene(**overrides) -> dict:
    # Heading along world +x (yaw -90 deg), as in the recordings; the bottle on the tray.
    yaw = -math.pi / 2
    ox, oy, oz = DEFAULT_TRAY_OFFSET
    scene = {
        "gravity": -G,
        "initial_robot_position": [0.0, 0.0, 0.0], "initial_robot_orientation": _quaternion([0, 0, 1], yaw),
        "problem_position": [0.0, 0.0, 0.0], "problem_orientation": [0.0, 0.0, 0.0, 1.0],
        "bottle_position": [oy, -ox, oz], "bottle_orientation": [0.0, 0.0, 0.0, 1.0],
        "simulation_length": 3.0, "num_of_repetitions": 2,
        "list_of_target_velocities": {"timestamp": [1.0], "adv_speed": [0.5]},
    }
    scene.update(overrides)
    return scene


def _run(scene: dict, cause: dict = None) -> list:
    workdir = Path(tempfile.mkdtemp(prefix="insight_fixes_"))
    scene_path = workdir / "scene.json"
    scene_path.write_text(json.dumps(scene), encoding="utf-8")
    read_fd, write_fd = os.pipe()
    try:
        simulator = CausesSimulator(json.dumps({"cause": cause or {"name": "none"}}), str(scene_path), write_fd,
                                    Logger(str(workdir / "sim.log")), real_time=False, gui=False)
        simulator.doSimulations()
        return json.loads(json.dumps(simulator.historical))
    finally:
        os.close(read_fd)
        os.close(write_fd)
        p.disconnect()


def test_settled_and_repeatable():
    first, second = _run(_scene())
    assert first["settle_s"] < 3.0, first["settle_s"]
    # Identical repetitions: nothing carries over from one to the next.
    assert first["history"] == second["history"] and first["bottle_position"] == second["bottle_position"]
    times = np.array(first["history"]["timestamp"])
    acc = np.array(first["history"]["accelerometer"])
    gyro = np.array(first["history"]["gyroscope"])
    still = times < 1.0
    # Before its first setpoint the settled robot stands still: no drop, no shaking.
    assert np.abs(acc[still] - [0.0, 0.0, G]).max() < 0.05, np.abs(acc[still] - [0.0, 0.0, G]).max()
    assert np.abs(gyro[still]).max() < 0.01, np.abs(gyro[still]).max()
    # Then it accelerates forward (+y of the robot) with what two 10 N*m motors give 54 kg.
    assert 2.5 < acc[(times > 1.0) & (times < 1.1), 1].max() < 4.5
    # Driving straight at 0.5 m/s does not lose the bottle.
    assert first["effect_time"] is None and first["fall_time"] is None


def test_contacts():
    """Webots' soft contacts, with two contact points per drive wheel (a cylinder in ODE) and one per
    caster, put a third of the weight on each drive wheel and a sixth on each caster, and the robot sinks
    W / 6k; with PyBullet's rigid ones the split was arbitrary (80 % on the casters), and the drive
    wheels slipped."""
    workdir = Path(tempfile.mkdtemp(prefix="insight_fixes_"))
    scene_path = workdir / "scene.json"
    scene_path.write_text(json.dumps(_scene(num_of_repetitions=1)), encoding="utf-8")
    read_fd, write_fd = os.pipe()
    try:
        simulator = CausesSimulator(json.dumps({"cause": {"name": "none"}}), str(scene_path), write_fd,
                                    Logger(str(workdir / "sim.log")), real_time=False, gui=False)
        simulator.settle_scene()
        names = {link: p.getJointInfo(simulator.robot, link)[12].decode() for link in range(p.getNumJoints(simulator.robot))}
        loads = {}
        for contact in p.getContactPoints(simulator.robot, simulator.plane):
            loads[names.get(contact[3], "base")] = loads.get(names.get(contact[3], "base"), 0.0) + contact[9]
        total = sum(loads.values())
        assert sorted(loads) == ["caster_back", "caster_front", "wheel_left", "wheel_right"], loads
        assert abs(total - 54.4 * G) < 0.02 * 54.4 * G, total
        share = {"wheel_left": 1 / 3, "wheel_right": 1 / 3, "caster_front": 1 / 6, "caster_back": 1 / 6}
        assert all(abs(load / total - share[name]) < 0.03 for name, load in loads.items()), loads
        # Webots' sink at rest (3.6 mm at 8 ms, 14.5 at 32): W over six springs of k = ERP / (h CFM).
        k = 0.2 / (simulator.world_time_step * 0.001)
        wheel_z = p.getLinkState(simulator.robot, simulator.links_name["wheel_left"])[0][2]
        assert abs((0.1 - wheel_z) - 54.4 * G / (6 * k)) < 0.0005, (0.1 - wheel_z, 54.4 * G / (6 * k))
        wheel = simulator.joints_name["wheel_left_joint"]
        assert p.getDynamicsInfo(simulator.robot, wheel)[1] * p.getDynamicsInfo(simulator.plane, -1)[1] == 1.0
    finally:
        os.close(read_fd)
        os.close(write_fd)
        p.disconnect()


def test_perception_rule():
    # A bottle lying on the floor next to the robot is out of the camera's box from the first
    # frame: the effect comes with the fifth frame, as concept_bottle deletes the edge.
    (repetition,) = _run(_scene(num_of_repetitions=1, simulation_length=1.0,
                                bottle_position=[0.0, 1.0, 0.03],
                                list_of_target_velocities={"timestamp": [0.0], "adv_speed": [0.0]}))
    perception = repetition["perception"]
    assert perception["in_view"][:PERCEPTION_LOST_FRAMES] == [False] * PERCEPTION_LOST_FRAMES
    expected = PERCEPTION_LOST_FRAMES * PERCEPTION_PERIOD_S
    assert abs(repetition["effect_time"] - expected) < 0.02, repetition["effect_time"]


# --------------------------------------------------------------------------- #
# the scene from a recording
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
    T0 = 10 * NS

    def __init__(self):
        yaw = [0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)]
        self.robot_rt = [Event(self.T0, "MEA", {"rt_translation": Attr([-3.7, -0.3, 0.02]), "rt_quaternion": Attr(yaw)}),
                         Event(self.T0 + 8 * NS, "MEA", {"rt_translation": Attr([-0.5, -0.3, 0.02]), "rt_quaternion": Attr(yaw)})]
        # The first IMU updates carry only the gyroscope; the accelerometer starts at 0.2 s and is
        # then updated every 40 ms, with a gap while it does not change.
        self.imu = [Event(self.T0 + int(0.04 * i * NS), "MNA", {"imu_gyroscope": Attr([0.0, 0.0, 0.01 * i])})
                    for i in range(5)]
        self.imu += [Event(self.T0 + int((0.2 + 0.04 * i) * NS), "MNA",
                           {"imu_accelerometer": Attr([0.0, 0.1 * i, G]), "imu_gyroscope": Attr([0.0, 0.0, 0.0])})
                     for i in range(50) if not 20 <= i < 30]
        # The robot node starts 80 ms after the IMU, as in the recordings.
        self.robot_node = [Event(self.T0 + int(0.08 * NS), "MN"),
                           Event(self.T0 + int(0.08 * NS), "MNA", {"robot_ref_adv_speed": Attr(0.6),
                                                                   "robot_ref_rot_speed": Attr(0.0)}),
                           Event(self.T0 + int(1.5 * NS), "MNA", {"robot_ref_adv_speed": Attr(0.6),
                                                                  "robot_ref_rot_speed": Attr(-0.3)})]

    def is_ready(self):
        return True

    def get_keyframe_count(self):
        return 1

    def get_keyframe(self, index):
        return Keyframe([{"name": "root", "id": self.ROOM}, {"name": "robot", "id": self.ROBOT},
                         {"name": "bottle", "id": self.BOTTLE}])

    def get_edge_history(self, from_id, to_id, edge_type):
        return self.robot_rt if (from_id, to_id) == (self.ROOM, self.ROBOT) else []

    def get_node_history_by_name(self, name):
        return {"robot": self.robot_node, "imu": self.imu}.get(name, [])


def test_scene_from_recording():
    api = FakeMemApi()
    scene, info = build_simulation_scene(api, [-3.7, -0.3, 0.0], [0.0, 0.0, 0.001])
    # Timed from the first IMU event, which the real IMU history and t_obs are timed from.
    assert abs(scene["list_of_target_velocities"]["timestamp"][0] - 0.08) < 1e-9, scene["list_of_target_velocities"]
    # The same setpoint timed from the first robot event (the static causes path) starts at 0.
    assert commanded_speed_history(api)["timestamp"][0] == 0.0
    rotation = scene["list_of_target_rot_speeds"]
    assert rotation["rot_speed"] == [0.0, -0.3] and abs(rotation["timestamp"][1] - 1.5) < 1e-9, rotation
    # The physics step of the recorded world, which the recording does not carry.
    assert scene["world_time_step_s"] == RECORDED_WORLD_TIME_STEP_S

    history = real_imu_history(api, [0.05 * i for i in range(1, 20)])
    # Nothing before the first accelerometer value (0.2 s): no made-up zeros.
    assert history["timestamp"][0] >= 0.2 - 1e-9, history["timestamp"][:3]
    assert all(sample[2] == G for sample in history["accelerometer"])


# --------------------------------------------------------------------------- #
# the verdict
# --------------------------------------------------------------------------- #
def test_verdict_effect_and_spread():
    # Gravity on z does not count as spread; only how each axis moves about its own mean.
    rng = np.random.default_rng(0)
    real = np.column_stack([np.zeros(500), rng.normal(0.0, 1.0, 500), np.full(500, G)])
    assert abs(sensor_spread(real) - math.sqrt(np.var(real[:, 1]) / 3.0)) < 1e-12
    assert sensor_spread(real) < 1.0 < float(np.std(real))

    upright, on_floor = [0.0, 0.0, 780.0], [500.0, 0.0, 25.0]
    # The effect is perception's, whether or not the bottle reached the floor.
    tipped_on_tray = {"bottle_position": upright, "effect_time": 12.0, "fall_time": None}
    assert effect_reproduced(tipped_on_tray, 780.0) and plausible_effect(tipped_on_tray, 780.0)
    seen_upright = {"bottle_position": on_floor, "effect_time": None, "fall_time": 12.0,
                    "robot_position_at_fall": [0.0, 0.0, 30.0]}
    assert not effect_reproduced(seen_upright, 780.0)
    # Repetitions from before the rule: the final height.
    assert effect_reproduced({"bottle_position": on_floor}, 780.0)
    # A bottle flung far away is still implausible.
    flung = {"bottle_position": [9000.0, 0.0, 25.0], "effect_time": 12.0, "fall_time": 11.9,
             "robot_position_at_fall": [0.0, 0.0, 30.0]}
    assert not plausible_effect(flung, 780.0)

    # The verdict counts each repetition's effect by when it happened: within 0.5 s of t_obs, at
    # another time, or not at all. With the provisional weights, which are equal, the effect does
    # not discriminate (the bench showed it does not predict the recorded loss); with informative
    # weights, the repetition that loses the bottle on time wins.
    times = [round(0.1 * i, 3) for i in range(200)]
    imu = {"timestamp": times, "accelerometer": [[0.0, 0.0, G]] * 200, "gyroscope": [[0.0, 0.0, 0.0]] * 200}
    record = {"accelerometer": {"timestamp": times, "values": imu["accelerometer"]},
              "gyroscope": {"timestamp": times, "values": imu["gyroscope"]}, "path": {"timestamp": [], "xy": []}}
    scene = {"anomaly_interval": [10.3, 12.3], "start_time_s": 8.8, "observed_effect_time": 12.3}

    def repetition(effect):
        return {"history": imu, "generated_instances": {}, "bottle_position": upright, "fall_time": None,
                "effect_time": effect, "robot_position_at_fall": None, "robot_final_position": [0.0, 0.0, 30.0]}
    entries = [{"hypothesis_id": NOMINAL_HYPOTHESIS_ID, "title": "", "mechanism": "nominal", "cause": {"name": "none"}},
               {"hypothesis_id": "late", "title": "", "mechanism": "a", "cause": {"name": "bump"}},
               {"hypothesis_id": "on_time", "title": "", "mechanism": "b", "cause": {"name": "bump"}}]
    historicals = [[repetition(None)], [repetition(13.0)], [repetition(12.2)]]
    verdict = build_verdict("effect", record, entries, historicals, scene)
    by_id = {h["hypothesis_id"]: h for h in verdict["hypotheses"]}
    assert by_id["late"]["mistimed_effects"] == 1 and by_id["on_time"]["effect_rate"] == 1.0
    assert abs(by_id["late"]["posterior"] - by_id["on_time"]["posterior"]) < 1e-9 and verdict["accepted_hypothesis_id"] is None
    import src.verdict as verdict_module
    previous = dict(verdict_module.EFFECT_WEIGHTS)
    verdict_module.EFFECT_WEIGHTS.update({"on_time": 0.9, "off_time": 0.01, "none": 0.01})
    try:
        verdict = build_verdict("effect", record, entries, historicals, scene)
    finally:
        verdict_module.EFFECT_WEIGHTS.update(previous)
    assert verdict["accepted_hypothesis_id"] == "on_time" and verdict["schema_version"] == "2.0", verdict["accepted_hypothesis_id"]


def main():
    test_bodies()
    test_imu_model()
    test_contacts()
    test_settled_and_repeatable()
    test_perception_rule()
    test_scene_from_recording()
    test_verdict_effect_and_spread()
    print("test_simulator_fixes OK")


if __name__ == "__main__":
    main()
