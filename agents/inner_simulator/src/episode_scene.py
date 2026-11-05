"""Reconstruction of the initial simulation scene from an episodic memory recording.

Replaces the hardcoded ROBOT_POS / BOTTLE_POS / PROBLEM_POS constants: the robot's
initial and final poses are read from the room->robot RT edge history (recorded in
the DSR room frame, which is also the PyBullet world frame), and the bottle pose is
derived as robot pose + tray offset (a physical constant of the robot model, since
the recording only registers the robot->bottle edge deletion when the bottle falls).

It also reads what the scene needs over time: the commanded speed profile, the
episode length, and the instant the bottle was observed to leave the robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Bottle resting on the tray, in the URDF robot frame (meters, +y forward). x and y are where
# SimpleWorld_Bump.wbt places the MedicineBottle relative to the Shadow (11.0 cm to the left,
# 15.5 cm ahead); z is where the bottle URDF comes to rest on the tray of the differential
# shadow URDF (0.841 m above its base origin), plus the 5 mm the simulator lifts the robot by.
DEFAULT_TRAY_OFFSET = [-0.110, 0.155, 0.846]

# The base origin of the shadow URDF rests on the floor. The recorded z (~0.0185 m) is the origin
# of the Webots body, not of the URDF: spawning the URDF there dropped it 2 cm at the start.
URDF_BASE_Z = 0.0

IDENTITY_QUATERNION = [0.0, 0.0, 0.0, 1.0]

# The DSR/RoboComp robot frame is +y-forward (concept_robot integrates odometry as
# x += v*sin(theta), y += v*cos(theta), and SimpleWorld_Bump.wbt starts the Shadow
# rotated -90 deg about z while it advances along world +x). The four-wheel shadow URDF drove
# along +x and needed +90 deg (without it, 43/43 replays drove 90 deg away from the recorded
# path). The differential shadow URDF (08/10) has its drive wheels on the x axis and drives
# along +y, like the DSR frame: no offset.
DSR_TO_URDF_YAW_OFFSET = 0.0

# Seconds simulated past the observed effect. The recording keeps going while the
# robot waits stopped (keyframes up to ~2 min after the fall); simulating that tail
# only costs time and lets fraction-based activation windows land after the event.
EFFECT_HORIZON_MARGIN_S = 3.0

TIMESTAMP = "timestamp"
ADV_SPEED = "adv_speed"
ROT_SPEED = "rot_speed"

# Recordings may be in millimeters (DSR convention) or meters (Webots bridge);
# coordinates beyond this magnitude mean the whole history is in millimeters.
MM_DETECTION_THRESHOLD = 100.0


@dataclass
class ScenePoses:
    initial_robot_position: list[float]
    initial_robot_orientation: list[float]
    problem_position: list[float]
    problem_orientation: list[float]
    bottle_position: list[float]
    bottle_orientation: list[float]
    sources: dict = field(default_factory=dict)  # per-field provenance, for logging


def _units_scale(events) -> float:
    """One unit decision for the whole edge history, not per sample."""
    peak = max(
        (abs(float(v)) for event in events for v in list(event.attributes["rt_translation"].value)[:2]),
        default=0.0,
    )
    return 0.001 if peak > MM_DETECTION_THRESHOLD else 1.0


def _quat_multiply(a: list[float], b: list[float]) -> list[float]:
    """Hamilton product a*b of [x, y, z, w] quaternions."""
    ax, ay, az, aw = (float(c) for c in a)
    bx, by, bz, bw = (float(c) for c in b)
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def to_urdf_orientation(dsr_quaternion: list[float], yaw_offset: float = DSR_TO_URDF_YAW_OFFSET) -> list[float]:
    """Orientation of the URDF body for a recorded DSR orientation: the recorded one composed
    with a rotation of yaw_offset about the body z axis (for a level robot, yaw + yaw_offset)."""
    half = yaw_offset / 2.0
    return _quat_multiply(dsr_quaternion, [0.0, 0.0, math.sin(half), math.cos(half)])


def _quat_rotate(quaternion: list[float], vector: list[float]) -> list[float]:
    """Rotate a vector by an [x, y, z, w] quaternion."""
    qx, qy, qz, qw = (float(c) for c in quaternion)
    q_vec = np.array([qx, qy, qz])
    v = np.array([float(c) for c in vector])
    t = 2.0 * np.cross(q_vec, v)
    rotated = v + qw * t + np.cross(q_vec, t)
    return rotated.tolist()


#: The world node the robot pose hangs from: "room" in older recordings, "root" since the UEx DSR
#: of 08/10. The semantic agent resolves it the same way (episode_memory_reader.WORLD_NODES).
WORLD_NODES = ("room", "root")


def _world_node_id(keyframe, name: Optional[str] = None):
    for candidate in ((name,) if name else WORLD_NODES):
        node_id = _find_node_id_by_name(keyframe, candidate)
        if node_id is not None:
            return node_id
    return None


def _find_node_id_by_name(keyframe, name: str):
    for node in keyframe.nodes_list:
        if node.get("name") == name:
            return node.get("id")
    return None


def _rt_pose_events(mem_api, from_id: int, to_id: int) -> list:
    """RT edge events that actually carry a translation, oldest first."""
    events = mem_api.get_edge_history(from_id, to_id, "RT")
    with_pose = []
    for event in events:
        try:
            attributes = event.attributes
        except TypeError:
            # DSR::Attribute not registered (pydsr not imported); treat as no data.
            return []
        if "rt_translation" in attributes:
            with_pose.append(event)
    return with_pose


#: Robot path (m) needed to tell from the motion which axes the recorded positions use.
AXES_MIN_PATH_M = 0.3
#: How much better the chosen axes must agree with the heading than the others. On a straight route at heading theta
#: from the room's x axis the wrong axes agree sin(2 theta): 0.1 tells them apart up to ~32 deg (0.3 stopped at 22).
AXES_MIN_MARGIN = 0.1


def _yaw(quaternion: list[float]) -> float:
    x, y, z, w = (float(c) for c in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def recorded_axes(positions: list[list[float]], quaternions: list[list[float]]) -> str:
    """"recorded" when the recorded positions agree with the recorded orientation, "swapped_xy"
    when they agree with x and y exchanged, "undetermined" otherwise (the recorded axes are kept).

    The UEx DSR of 08/10 writes root->robot as (y, x, z) of the Webots world and keeps the Webots
    orientation. A following robot moves forward, along +y of its DSR frame, so the axes under
    which the motion agrees with the heading are the right ones. The semantic agent decides the
    same way (episode_builder.recorded_axes).
    """
    xy = np.array([[float(p[0]), float(p[1])] for p in positions]) if positions else np.zeros((0, 2))
    steps = np.diff(xy, axis=0)
    path = float(np.linalg.norm(steps, axis=1).sum()) if len(steps) else 0.0
    if path < AXES_MIN_PATH_M:
        return "undetermined"
    yaw = np.array([_yaw(q) for q in quaternions[1:]])
    forward = np.column_stack([-np.sin(yaw), np.cos(yaw)])
    direct = float(np.sum(steps * forward)) / path
    swapped = float(np.sum(steps[:, ::-1] * forward)) / path
    if swapped > 0.5 and swapped - direct > AXES_MIN_MARGIN:
        return "swapped_xy"
    if direct > 0.5 and direct - swapped > AXES_MIN_MARGIN:
        return "recorded"
    return "undetermined"


def _event_pose(event, scale: float) -> tuple[list[float], list[float]]:
    attributes = event.attributes
    translation = [float(v) * scale for v in attributes["rt_translation"].value]
    if "rt_quaternion" in attributes:
        quaternion = [float(c) for c in attributes["rt_quaternion"].value]
    else:
        quaternion = list(IDENTITY_QUATERNION)
    return translation, quaternion


def extract_scene_poses(
    mem_api,
    fallback_robot_position: list[float],
    fallback_bottle_position: list[float],
    fallback_problem_position: list[float],
    tray_offset: list[float] = DEFAULT_TRAY_OFFSET,
    yaw_offset: float = DSR_TO_URDF_YAW_OFFSET,
    room_name: Optional[str] = None,
    robot_name: str = "robot",
) -> ScenePoses:
    """Build the scene poses from the episode, falling back to the provided
    constants for anything the recording does not contain."""
    poses = ScenePoses(
        initial_robot_position=list(fallback_robot_position),
        initial_robot_orientation=list(IDENTITY_QUATERNION),
        problem_position=list(fallback_problem_position),
        problem_orientation=list(IDENTITY_QUATERNION),
        bottle_position=list(fallback_bottle_position),
        bottle_orientation=list(IDENTITY_QUATERNION),
        sources={
            "initial_robot": "fallback_constant",
            "problem": "fallback_constant",
            "bottle": "fallback_constant",
        },
    )

    if not mem_api.is_ready() or mem_api.get_keyframe_count() == 0:
        return poses

    keyframe = mem_api.get_keyframe(0)
    room_id = _world_node_id(keyframe, room_name)
    robot_id = _find_node_id_by_name(keyframe, robot_name)
    if room_id is None or robot_id is None:
        return poses

    robot_events = _rt_pose_events(mem_api, room_id, robot_id)
    if not robot_events:
        return poses

    scale = _units_scale(robot_events)
    initial_position, initial_orientation = _event_pose(robot_events[0], scale)
    final_position, final_orientation = _event_pose(robot_events[-1], scale)
    recorded = [_event_pose(event, scale) for event in robot_events]
    axes = recorded_axes([position for position, _ in recorded], [quaternion for _, quaternion in recorded])
    poses.sources["axes"] = axes
    if axes == "swapped_xy":
        initial_position = [initial_position[1], initial_position[0], *initial_position[2:]]
        final_position = [final_position[1], final_position[0], *final_position[2:]]

    poses.initial_robot_position = [initial_position[0], initial_position[1], URDF_BASE_Z]
    poses.initial_robot_orientation = to_urdf_orientation(initial_orientation, yaw_offset)
    poses.sources["initial_robot"] = "episodic_rt_edge"

    poses.problem_position = final_position
    poses.problem_orientation = to_urdf_orientation(final_orientation, yaw_offset)
    poses.sources["problem"] = "episodic_rt_edge"

    # The tray offset is expressed in the URDF body frame, so it is rotated by the
    # orientation the URDF is actually loaded with.
    rotated_offset = _quat_rotate(poses.initial_robot_orientation, tray_offset)
    poses.bottle_position = [p + o for p, o in zip(poses.initial_robot_position, rotated_offset)]
    poses.sources["bottle"] = "robot_pose_plus_tray_offset"

    return poses


# --------------------------------------------------------------------------- #
# Time series of the episode
# --------------------------------------------------------------------------- #
def episode_time_origin_ns(mem_api) -> Optional[int]:
    """Timestamp (ns) of the first IMU event: time zero of the real IMU history the
    verdict compares against, and of the simulation."""
    if not mem_api.is_ready():
        return None
    imu_events = mem_api.get_node_history_by_name("imu")
    return int(imu_events[0].timestamp) if imu_events else None


def episode_length_s(mem_api) -> Optional[float]:
    """Seconds between the first and the last IMU event of the recording."""
    if not mem_api.is_ready():
        return None
    imu_events = mem_api.get_node_history_by_name("imu")
    if not imu_events:
        return None
    return (imu_events[-1].timestamp - imu_events[0].timestamp) * 1e-9


def _setpoint_history(mem_api, attribute: str, key: str, until_ns: Optional[int],
                      origin_ns: Optional[int]) -> Optional[dict]:
    """The `attribute` setpoints (MNA events of the robot node) as {timestamp, key}, oldest first,
    in seconds from `origin_ns` (the first robot event when None). Setpoints at or after
    `until_ns` are left out."""
    if not mem_api.is_ready():
        return None
    robot_events = mem_api.get_node_history_by_name("robot")
    history = {TIMESTAMP: [], key: []}
    if not robot_events:
        return history
    initial_ts = robot_events[0].timestamp if origin_ns is None else origin_ns
    setpoints = sorted(
        ((event.timestamp - initial_ts) * 1e-9, event.attributes[attribute].value)
        for event in robot_events
        if event.modification_type == "MNA" and attribute in event.attributes
        and (until_ns is None or event.timestamp < until_ns)
    )
    for time_s, value in setpoints:
        history[TIMESTAMP].append(time_s)
        history[key].append(value)
    return history


def commanded_speed_history(mem_api, until_ns: Optional[int] = None,
                            origin_ns: Optional[int] = None) -> Optional[dict]:
    """The robot_ref_adv_speed setpoints, in seconds from `origin_ns`. The replay passes the
    origin of the real IMU history (episode_time_origin_ns), which the verdict compares on:
    the first robot event comes ~80 ms later, and timing the setpoints from it ran them early."""
    return _setpoint_history(mem_api, "robot_ref_adv_speed", ADV_SPEED, until_ns, origin_ns)


def commanded_rot_speed_history(mem_api, until_ns: Optional[int] = None,
                                origin_ns: Optional[int] = None) -> Optional[dict]:
    """The robot_ref_rot_speed setpoints (rad/s), like commanded_speed_history. On the differential
    Shadow of Webots (08-09/10) they have the sign of the yaw rate the gyroscope measures (positive
    counterclockwise): the -1.57 and -1.16 rad/s ordered next to the bump at 09:25 and 09:48 of
    09/10 show as -0.24 and -0.18 rad/s, while the robot is held by the bump."""
    return _setpoint_history(mem_api, "robot_ref_rot_speed", ROT_SPEED, until_ns, origin_ns)


def _imu_samples(mem_api) -> list[tuple[int, Optional[list], Optional[list]]]:
    """(ns from the first IMU event, accelerometer or None, gyroscope or None) of each IMU
    update, oldest first."""
    imu_events = mem_api.get_node_history_by_name("imu")
    if not imu_events:
        return []
    initial_ts = imu_events[0].timestamp
    return sorted(
        ((event.timestamp - initial_ts,
          event.attributes["imu_accelerometer"].value if "imu_accelerometer" in event.attributes else None,
          event.attributes["imu_gyroscope"].value if "imu_gyroscope" in event.attributes else None)
         for event in imu_events if event.modification_type == "MNA"),
        key=lambda sample: sample[0])


def real_imu_history(mem_api, list_of_ts: list) -> Optional[dict]:
    """The recorded IMU resampled at the simulation timestamps (seconds from the first
    IMU event): each sample takes the last recorded value at or before it. Timestamps before
    both the accelerometer and the gyroscope have a value are left out: the first updates of a
    recording carry only the gyroscope, and filling the accelerometer with zeros there made up
    a signal the robot never reported."""
    if not mem_api.is_ready():
        return None
    history = {TIMESTAMP: [], "accelerometer": [], "gyroscope": []}
    samples = _imu_samples(mem_api)
    if not samples:
        return history
    times = [sample[0] for sample in samples]
    last_acc, last_gyro = None, None
    consumed = 0
    for ts in list_of_ts:
        index = int(np.searchsorted(times, int(ts * 1e9), side="right")) - 1
        while consumed <= index:
            _, acc, gyro = samples[consumed]
            last_acc = acc if acc is not None else last_acc
            last_gyro = gyro if gyro is not None else last_gyro
            consumed += 1
        if last_acc is None or last_gyro is None:
            continue
        history[TIMESTAMP].append(int(ts * 1e9) * 1e-9)
        history["accelerometer"].append(last_acc)
        history["gyroscope"].append(last_gyro)
    return history


#: Physics step (s) of the world the episodes are recorded in. The recording does not say it. Change it
#: with the world: the simulator derives from it the span each accelerometer sample averages over and
#: Webots' soft contacts. SimpleWorld_Bump.wbt has basicTimeStep 8 since 16/10/2026, as webots-bridge
#: asks for; before, it set none and ran at Webots' default, 32 ms (the episodes of 08-09/10). The
#: resting robot sinks 3.6 mm at 8 ms and 14.5 mm at 32 ms, as the soft contact predicts.
RECORDED_WORLD_TIME_STEP_S = 0.008


def _find_node_id_in_keyframes(mem_api, name: str):
    """Node id by name, looking through every keyframe (a node may appear late)."""
    for index in range(mem_api.get_keyframe_count()):
        node_id = _find_node_id_by_name(mem_api.get_keyframe(index), name)
        if node_id is not None:
            return node_id
    return None


def observed_effect_times(
    mem_api,
    origin_ns: Optional[int],
    from_name: str = "robot",
    to_name: str = "bottle",
    edge_type: str = "RT",
) -> list[float]:
    """Seconds (from `origin_ns`) at which the robot->bottle RT edge was deleted,
    i.e. when perception reported the bottle had left the robot. The first one is
    the observed effect; later ones come from the bottle being re-attached and lost
    again, so callers should keep them only for the record."""
    if origin_ns is None or not mem_api.is_ready() or mem_api.get_keyframe_count() == 0:
        return []
    from_id = _find_node_id_in_keyframes(mem_api, from_name)
    to_id = _find_node_id_in_keyframes(mem_api, to_name)
    if from_id is None or to_id is None:
        return []
    events = mem_api.get_edge_history(from_id, to_id, edge_type)
    return sorted((event.timestamp - origin_ns) * 1e-9 for event in events if event.modification_type == "DE")


def simulation_horizon(
    episode_length: Optional[float],
    observed_effect_time: Optional[float],
    margin_s: float = EFFECT_HORIZON_MARGIN_S,
) -> Optional[float]:
    """Simulate until shortly after the observed effect; the whole recording when
    no effect was observed."""
    if observed_effect_time is None or episode_length is None:
        return episode_length
    return min(episode_length, observed_effect_time + margin_s)


# --------------------------------------------------------------------------- #
# Clock of the episode
# --------------------------------------------------------------------------- #
# The recording is stamped with the wall clock, but the physics behind it may run on
# a clock of its own: Webots simulated 0.46-0.96 s per wall second depending on the
# load, and in its own time the base follows the setpoint 1:1 (webots-bridge has no
# ramp or clamp). Replayed on the wall clock, the setpoint made the simulated robot
# cover 1.6-2.3 times the recorded path and reach the obstacle seconds early. The
# rate is fitted from the episode itself, as travelled over ordered in free motion;
# on the real robot it comes out close to 1 (the base's ramp), so nothing has to
# say which platform recorded the episode.
CLOCK_FIT_START_S = 1.0     # skips the start-up turn (placeholder person pose)
CLOCK_FIT_WINDOW_S = 1.0
CLOCK_MIN_ORDERED_M = 0.1   # the robot was told to move
CLOCK_MIN_RATIO = 0.25      # below it the robot was stalled, e.g. against an obstacle


def _zoh_integral(times: np.ndarray, values: np.ndarray, at: np.ndarray) -> np.ndarray:
    """Integral, from the first sample, of a zero-order-hold signal at each of `at`."""
    cumulative = np.concatenate([[0.0], np.cumsum(np.diff(times) * values[:-1])])
    index = np.searchsorted(times, at, side="right") - 1
    safe = np.clip(index, 0, None)
    return np.where(index >= 0, cumulative[safe] + values[safe] * (at - times[safe]), 0.0)


def episode_clock_rate(mem_api, origin_ns: Optional[int], until_s: Optional[float]) -> tuple[float, int]:
    """(rate, windows): seconds of the recorded physics per second of the recording.

    The median, over windows of CLOCK_FIT_WINDOW_S from CLOCK_FIT_START_S to `until_s`,
    of the path the robot travelled (room->robot RT history) over the path the forward
    setpoint ordered. Windows where the robot was not told to move, or was stalled, do
    not count. Without any window the rate is 1: the recording's own clock."""
    if origin_ns is None or until_s is None or not mem_api.is_ready() or mem_api.get_keyframe_count() == 0:
        return 1.0, 0
    keyframe = mem_api.get_keyframe(0)
    room_id = _world_node_id(keyframe)
    robot_id = _find_node_id_by_name(keyframe, "robot")
    if room_id is None or robot_id is None:
        return 1.0, 0
    pose_events = _rt_pose_events(mem_api, room_id, robot_id)
    setpoints = [
        ((event.timestamp - origin_ns) * 1e-9, float(event.attributes["robot_ref_adv_speed"].value))
        for event in mem_api.get_node_history_by_name("robot")
        if event.modification_type == "MNA" and "robot_ref_adv_speed" in event.attributes
    ]
    if len(pose_events) < 2 or not setpoints:
        return 1.0, 0

    scale = _units_scale(pose_events)
    pose_times = np.array([(event.timestamp - origin_ns) * 1e-9 for event in pose_events])
    xy = np.array([_event_pose(event, scale)[0][:2] for event in pose_events])
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    setpoint_times, setpoint_values = (np.array(column) for column in zip(*setpoints))

    edges = np.arange(CLOCK_FIT_START_S, until_s + 1e-9, CLOCK_FIT_WINDOW_S)
    travelled = np.diff(np.interp(edges, pose_times, path))
    ordered = np.diff(_zoh_integral(setpoint_times, setpoint_values, edges))
    moving = ordered > CLOCK_MIN_ORDERED_M
    ratios = travelled[moving] / ordered[moving]
    ratios = ratios[ratios > CLOCK_MIN_RATIO]
    if ratios.size == 0:
        return 1.0, 0
    return float(np.median(ratios)), int(ratios.size)


#: Repetitions of every hypothesis but the nominal. The verdict rests on the simulations closest to the
#: record: with 36 there were too few (docs_output/resultados/35_banco_de_verificacion), and with 144 none
#: came near a small bump off the robot's path (resultados/36); with 4,000 the search reaches it
#: (resultados/38), and the fast repetitions (causes_simulator.doFastSimulations) run them in ~40 s per case
#: (resultados/41). Its EPS is calibrated for this number (resultados/43).
DEFAULT_REPETITIONS = 4000


# --------------------------------------------------------------------------- #
# The anomaly interval and the seeded start (docs_output/diseno/veredicto.md, 3.1-3.2)
# --------------------------------------------------------------------------- #
#: The episode's anomaly interval, as the semantic memory writes it (contract 1, episode_builder).
ANOMALY_INTERVAL_ID = "Interval_fall"
#: Without the episode, the same rule: the last ANOMALY_INTERVAL_S of motion before t_obs.
ANOMALY_INTERVAL_S = 2.0
MOVING_SPEED_M_S = 0.05
#: The simulation starts this long before the anomaly interval, from the recorded state: the replay
#: from the start of the recording drifted up to 28 cm sideways on the Webots bench (resultados/32).
SEED_LEAD_S = 1.5
#: Reach of the robot around its centre (wheels, casters and body), as the semantic memory dilates the
#: path segments (hypothesis_pipeline.ROBOT_FOOTPRINT_RADIUS_M).
ROBOT_FOOTPRINT_RADIUS_M = 0.35


def aligned_to_world_step(t: float, clock_rate: float) -> float:
    """The start of the step of the recorded world in which `t` (recording clock) falls. Webots applies
    the controller's commands and samples the IMU step by step, from the first IMU sample; a seeded
    start inside a step put every simulated IMU window across two steps and split the pulse of each
    setpoint change in half (docs_output/resultados/35_banco_de_verificacion)."""
    step = RECORDED_WORLD_TIME_STEP_S / clock_rate
    return math.floor(t / step + 1e-9) * step


def anomaly_footprint(mem_api, origin_ns: int, interval: tuple[float, float]) -> Optional[dict]:
    """{x: [min, max], y: [min, max]} (m) the robot covered during the anomaly interval: its recorded
    path then, dilated by its reach. A cause that touches the robot in that interval (a bump, a cable)
    lies within it, plus its own half size."""
    path = recorded_path(mem_api, origin_ns)
    if path is None:
        return None
    times, positions, _ = path
    inside = (times >= interval[0]) & (times <= interval[1])
    if not inside.any():
        return None
    xy = positions[inside, :2]
    return {axis: [float(xy[:, k].min() - ROBOT_FOOTPRINT_RADIUS_M), float(xy[:, k].max() + ROBOT_FOOTPRINT_RADIUS_M)]
            for k, axis in enumerate(("x", "y"))}


def recorded_path(mem_api, origin_ns: int):
    """(times s, positions m [N, 3], quaternions [N, 4]) of the root->robot RT history, in the room
    axes (recorded_axes), or None without two poses."""
    if not mem_api.is_ready() or mem_api.get_keyframe_count() == 0:
        return None
    keyframe = mem_api.get_keyframe(0)
    world_id, robot_id = _world_node_id(keyframe), _find_node_id_by_name(keyframe, "robot")
    if world_id is None or robot_id is None:
        return None
    events = _rt_pose_events(mem_api, world_id, robot_id)
    if len(events) < 2:
        return None
    scale = _units_scale(events)
    poses = [_event_pose(event, scale) for event in events]
    positions = np.array([position for position, _ in poses], dtype=float)
    if recorded_axes(list(positions), [quaternion for _, quaternion in poses]) == "swapped_xy":
        positions = positions[:, [1, 0, 2]]
    times = np.array([(event.timestamp - origin_ns) * 1e-9 for event in events])
    return times, positions, np.array([quaternion for _, quaternion in poses], dtype=float)


def anomaly_interval(mem_api, origin_ns: int, observed_effect_time: float,
                     episode_intervals: Optional[list] = None) -> tuple[float, float, str]:
    """(start, end, source) of the anomaly interval: the episode's Interval_fall when the semantic
    memory gave it, else the same rule on the recorded path (the last ANOMALY_INTERVAL_S of motion
    before t_obs, plus the final stop)."""
    for interval in episode_intervals or []:
        if interval.get("id") == ANOMALY_INTERVAL_ID:
            return float(interval["start_s"]), float(interval["end_s"]), "episode"
    path = recorded_path(mem_api, origin_ns)
    last_motion = observed_effect_time
    if path is not None:
        times, positions, _ = path
        speed = np.r_[0.0, np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1) / np.maximum(np.diff(times), 1e-6)]
        moving = times[(times <= observed_effect_time) & (speed > MOVING_SPEED_M_S)]
        if len(moving):
            last_motion = float(moving[-1])
    return max(0.0, last_motion - ANOMALY_INTERVAL_S), observed_effect_time, "recording"


def recorded_state_at(mem_api, origin_ns: int, at_s: float, yaw_offset: float = DSR_TO_URDF_YAW_OFFSET):
    """(position, URDF orientation, forward speed m/s) of the recorded robot at `at_s`: the last pose
    at or before it, and the speed over the 0.25 s before. None without a path."""
    path = recorded_path(mem_api, origin_ns)
    if path is None:
        return None
    times, positions, quaternions = path
    index = max(0, int(np.searchsorted(times, at_s, side="right")) - 1)
    earlier = max(0, int(np.searchsorted(times, times[index] - 0.25, side="right")) - 1)
    span = times[index] - times[earlier]
    speed = float(np.linalg.norm(positions[index, :2] - positions[earlier, :2]) / span) if span > 0 else 0.0
    return ([float(positions[index, 0]), float(positions[index, 1]), URDF_BASE_Z],
            to_urdf_orientation(list(quaternions[index]), yaw_offset), speed)


def real_record(mem_api) -> Optional[dict]:
    """What the verdict compares against: the recorded IMU updates (accelerometer and gyroscope,
    each with its own instants, from the first one that carries each) and the robot's planar path,
    in seconds from the first IMU event."""
    if not mem_api.is_ready():
        return None
    origin_ns = episode_time_origin_ns(mem_api)
    record = {}
    samples = _imu_samples(mem_api)
    for key, column in (("accelerometer", 1), ("gyroscope", 2)):
        rows = [(sample[0] * 1e-9, sample[column]) for sample in samples if sample[column] is not None]
        record[key] = {TIMESTAMP: [row[0] for row in rows], "values": [list(row[1]) for row in rows]}
    path = recorded_path(mem_api, origin_ns) if origin_ns is not None else None
    record["path"] = ({TIMESTAMP: path[0].tolist(), "xy": path[1][:, :2].tolist()} if path is not None
                      else {TIMESTAMP: [], "xy": []})
    return record


def build_simulation_scene(
    mem_api,
    fallback_robot_position: list[float],
    fallback_problem_position: list[float],
    num_of_repetitions: int = DEFAULT_REPETITIONS,
    gravity: float = -9.81,
    yaw_offset: float = DSR_TO_URDF_YAW_OFFSET,
    physics_dt: Optional[float] = None,
    episode_intervals: Optional[list] = None,
    seeded: bool = True,
) -> tuple[dict, dict]:
    """The scene the causes simulator replays (a SimulationScene dict, positions in
    meters, like the fallbacks), plus what was learned while building
    it, for logging. yaw_offset and physics_dt let experiments compare variants.

    Times stay on the recording's clock, from the first IMU event; `clock_rate` tells the
    simulator how fast the recorded physics ran against it (episode_clock_rate), and
    `world_time_step_s` the physics step of the recorded world (RECORDED_WORLD_TIME_STEP_S).

    With an observed effect and `seeded`, the simulation starts SEED_LEAD_S before the anomaly
    interval (the episode's, from `episode_intervals`, or the same rule on the recording) from the
    recorded state of the robot then, with the bottle on its tray; `start_time_s` says when, and the
    setpoints, the horizon and everything simulated stay on the recording's clock.

    The robot replays both setpoints: forward and rotation speed."""
    robot_fallback_m = list(fallback_robot_position)
    poses = extract_scene_poses(
        mem_api,
        fallback_robot_position=robot_fallback_m,
        fallback_bottle_position=[r + o for r, o in zip(robot_fallback_m, DEFAULT_TRAY_OFFSET)],
        fallback_problem_position=list(fallback_problem_position),
        yaw_offset=yaw_offset,
    )
    # First deletion of the robot->bottle RT edge; later ones come from the bottle
    # being re-attached and lost again.
    origin_ns = episode_time_origin_ns(mem_api)
    effect_times = observed_effect_times(mem_api, origin_ns)
    observed_effect_time = effect_times[0] if effect_times else None
    length = episode_length_s(mem_api)
    # The replay only uses setpoints issued before the effect was observed; the last
    # one is held until the horizon. Later ones are the system's reaction to the
    # anomaly (the semantic agent stops the robot ~0.1 s after the bottle is lost),
    # and replaying that hard stop can knock the bottle off by itself.
    until_ns = origin_ns + int(observed_effect_time * 1e9) if observed_effect_time is not None else None
    speeds = commanded_speed_history(mem_api, until_ns, origin_ns) or {TIMESTAMP: [], ADV_SPEED: []}
    rot_speeds = commanded_rot_speed_history(mem_api, until_ns, origin_ns) or {TIMESTAMP: [], ROT_SPEED: []}
    clock_rate, clock_windows = episode_clock_rate(
        mem_api, origin_ns, observed_effect_time if observed_effect_time is not None else length)

    start_time, initial_speed, interval, interval_source, footprint = 0.0, 0.0, None, None, None
    initial_position, initial_orientation = list(poses.initial_robot_position), poses.initial_robot_orientation
    bottle_position, bottle_orientation = list(poses.bottle_position), poses.bottle_orientation
    if seeded and observed_effect_time is not None and origin_ns is not None:
        start, end, interval_source = anomaly_interval(mem_api, origin_ns, observed_effect_time, episode_intervals)
        interval = [start, end]
        footprint = anomaly_footprint(mem_api, origin_ns, (start, end))
        seed = aligned_to_world_step(start - SEED_LEAD_S, clock_rate)
        state = recorded_state_at(mem_api, origin_ns, max(0.0, seed), yaw_offset)
        if state is not None and seed > 0.0:
            start_time = seed
            initial_position, initial_orientation, recorded_speed = state
            # The path is stamped with the recording's clock; the physics runs on the episode's.
            initial_speed = recorded_speed / clock_rate
            # Before the anomaly the bottle stood on the tray, where it starts.
            offset = _quat_rotate(initial_orientation, DEFAULT_TRAY_OFFSET)
            bottle_position = [a + b for a, b in zip(initial_position, offset)]
            bottle_orientation = initial_orientation

    scene = {
        "gravity": gravity,
        "initial_robot_position": initial_position,
        "initial_robot_orientation": initial_orientation,
        "problem_position": list(poses.problem_position),
        "problem_orientation": poses.problem_orientation,
        "bottle_position": bottle_position,
        "bottle_orientation": bottle_orientation,
        "simulation_length": simulation_horizon(length, observed_effect_time),
        "num_of_repetitions": num_of_repetitions,
        "list_of_target_velocities": speeds,
        "list_of_target_rot_speeds": rot_speeds,
        "observed_effect_time": observed_effect_time,
        "episode_length": length,
        "clock_rate": clock_rate,
        "physics_dt": physics_dt,
        "world_time_step_s": RECORDED_WORLD_TIME_STEP_S,
        "start_time_s": start_time,
        "initial_speed": initial_speed,
        "anomaly_interval": interval,
        "anomaly_footprint": footprint,
    }
    info = {"pose_sources": poses.sources, "effect_times": effect_times, "episode_length": length,
            "clock_rate": clock_rate, "clock_rate_windows": clock_windows,
            "anomaly_interval_source": interval_source}
    return scene, info
