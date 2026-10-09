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
    if swapped > 0.5 and swapped - direct > 0.3:
        return "swapped_xy"
    if direct > 0.5 and direct - swapped > 0.3:
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


def commanded_speed_history(mem_api, until_ns: Optional[int] = None) -> Optional[dict]:
    """The robot_ref_adv_speed setpoints (MNA events of the robot node), with their
    time relative to the first robot event. Setpoints at or after `until_ns` are
    left out."""
    if not mem_api.is_ready():
        return None
    robot_events = mem_api.get_node_history_by_name("robot")
    history = {TIMESTAMP: [], ADV_SPEED: []}
    if not robot_events:
        return history
    initial_ts = robot_events[0].timestamp
    for event in robot_events:
        if until_ns is not None and event.timestamp >= until_ns:
            continue
        if event.modification_type == "MNA" and "robot_ref_adv_speed" in event.attributes:
            history[TIMESTAMP].append((event.timestamp - initial_ts) * 1e-9)
            history[ADV_SPEED].append(event.attributes["robot_ref_adv_speed"].value)
    return history


def real_imu_history(mem_api, list_of_ts: list) -> Optional[dict]:
    """The recorded IMU resampled at the simulation timestamps (seconds from the first
    IMU event): each sample takes the last recorded value at or before it."""
    if not mem_api.is_ready():
        return None
    imu_events = mem_api.get_node_history_by_name("imu")
    history = {TIMESTAMP: [], "accelerometer": [], "gyroscope": []}
    if not imu_events:
        return history
    initial_ts = imu_events[0].timestamp
    samples = [
        (event.timestamp - initial_ts,
         event.attributes["imu_accelerometer"].value if "imu_accelerometer" in event.attributes else None,
         event.attributes["imu_gyroscope"].value if "imu_gyroscope" in event.attributes else None)
        for event in imu_events if event.modification_type == "MNA"
    ]
    samples.sort(key=lambda sample: sample[0])
    times = [sample[0] for sample in samples]
    last_acc, last_gyro = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for ts in list_of_ts:
        index = int(np.searchsorted(times, int(ts * 1e9), side="right")) - 1
        if index < 0:
            continue
        _, acc, gyro = samples[index]
        last_acc = acc if acc is not None else last_acc
        last_gyro = gyro if gyro is not None else last_gyro
        history[TIMESTAMP].append(int(ts * 1e9) * 1e-9)
        history["accelerometer"].append(last_acc)
        history["gyroscope"].append(last_gyro)
    return history


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


DEFAULT_REPETITIONS = 10


def build_simulation_scene(
    mem_api,
    fallback_robot_position: list[float],
    fallback_problem_position: list[float],
    num_of_repetitions: int = DEFAULT_REPETITIONS,
    gravity: float = -9.81,
    yaw_offset: float = DSR_TO_URDF_YAW_OFFSET,
    physics_dt: Optional[float] = None,
) -> tuple[dict, dict]:
    """The scene the causes simulator replays (a SimulationScene dict, positions in
    meters, like the fallbacks), plus what was learned while building
    it, for logging. yaw_offset and physics_dt let experiments compare variants.

    Times stay on the recording's clock; `clock_rate` tells the simulator how fast
    the recorded physics ran against it (episode_clock_rate).

    The rotation profile stays empty: where omega should come from (the pose
    history or the setpoint) is still to be decided, so the replay drives straight."""
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
    speeds = commanded_speed_history(mem_api, until_ns) or {TIMESTAMP: [], ADV_SPEED: []}
    clock_rate, clock_windows = episode_clock_rate(
        mem_api, origin_ns, observed_effect_time if observed_effect_time is not None else length)

    scene = {
        "gravity": gravity,
        "initial_robot_position": list(poses.initial_robot_position),
        "initial_robot_orientation": poses.initial_robot_orientation,
        "problem_position": list(poses.problem_position),
        "problem_orientation": poses.problem_orientation,
        "bottle_position": list(poses.bottle_position),
        "bottle_orientation": poses.bottle_orientation,
        "simulation_length": simulation_horizon(length, observed_effect_time),
        "num_of_repetitions": num_of_repetitions,
        "list_of_target_velocities": speeds,
        "list_of_target_rot_speeds": {},
        "observed_effect_time": observed_effect_time,
        "episode_length": length,
        "clock_rate": clock_rate,
        "physics_dt": physics_dt,
    }
    info = {"pose_sources": poses.sources, "effect_times": effect_times, "episode_length": length,
            "clock_rate": clock_rate, "clock_rate_windows": clock_windows}
    return scene, info
