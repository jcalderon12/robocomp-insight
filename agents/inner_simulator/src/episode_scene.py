"""Reconstruction of the initial simulation scene from an episodic memory recording.

Replaces the hardcoded ROBOT_POS / BOTTLE_POS / PROBLEM_POS constants: the robot's
initial and final poses are read from the room->robot RT edge history (recorded in
the DSR room frame, which is also the PyBullet world frame), and the bottle pose is
derived as robot pose + tray offset (a physical constant of the robot model, since
the recording only registers the robot->bottle edge deletion when the bottle falls).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Bottle resting on the tray, in the URDF robot frame (meters): BOTTLE_POS - ROBOT_POS
# with the shadow URDF loaded at identity orientation.
DEFAULT_TRAY_OFFSET = [0.05, 0.11, 0.7625]

IDENTITY_QUATERNION = [0.0, 0.0, 0.0, 1.0]

# The DSR/RoboComp robot frame is +y-forward (concept_robot integrates odometry as
# x += v*sin(theta), y += v*cos(theta), and SimpleWorld_Bump.wbt starts the Shadow
# rotated -90 deg about z while it advances along world +x); the shadow URDF drives
# along +x. Loading the recorded quaternion as is made every replay drive 90 deg
# away from the recorded path (43/43 recorded episodes).
DSR_TO_URDF_YAW_OFFSET = math.pi / 2.0

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


def to_urdf_orientation(dsr_quaternion: list[float]) -> list[float]:
    """Orientation of the URDF body whose +x points where the DSR body's +y points.

    Composes the recorded orientation with a +90 deg rotation about the body z
    axis (for a level robot this is yaw + 90 deg)."""
    half = DSR_TO_URDF_YAW_OFFSET / 2.0
    return _quat_multiply(dsr_quaternion, [0.0, 0.0, math.sin(half), math.cos(half)])


def _quat_rotate(quaternion: list[float], vector: list[float]) -> list[float]:
    """Rotate a vector by an [x, y, z, w] quaternion."""
    qx, qy, qz, qw = (float(c) for c in quaternion)
    q_vec = np.array([qx, qy, qz])
    v = np.array([float(c) for c in vector])
    t = 2.0 * np.cross(q_vec, v)
    rotated = v + qw * t + np.cross(q_vec, t)
    return rotated.tolist()


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
    room_name: str = "room",
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
    room_id = _find_node_id_by_name(keyframe, room_name)
    robot_id = _find_node_id_by_name(keyframe, robot_name)
    if room_id is None or robot_id is None:
        return poses

    robot_events = _rt_pose_events(mem_api, room_id, robot_id)
    if not robot_events:
        return poses

    scale = _units_scale(robot_events)
    initial_position, initial_orientation = _event_pose(robot_events[0], scale)
    final_position, final_orientation = _event_pose(robot_events[-1], scale)

    poses.initial_robot_position = initial_position
    poses.initial_robot_orientation = to_urdf_orientation(initial_orientation)
    poses.sources["initial_robot"] = "episodic_rt_edge"

    poses.problem_position = final_position
    poses.problem_orientation = to_urdf_orientation(final_orientation)
    poses.sources["problem"] = "episodic_rt_edge"

    # The tray offset is expressed in the URDF body frame, so it is rotated by the
    # orientation the URDF is actually loaded with.
    rotated_offset = _quat_rotate(poses.initial_robot_orientation, tray_offset)
    poses.bottle_position = [p + o for p, o in zip(initial_position, rotated_offset)]
    poses.sources["bottle"] = "robot_pose_plus_tray_offset"

    return poses
