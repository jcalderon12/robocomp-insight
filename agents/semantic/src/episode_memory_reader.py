"""RecordedSeries from the episodic memory: the production reader of the episode (task A2).

The semantic agent builds the episode from the recording of the mission whose path the "Follow
Person" node publishes, through the episodic-memory API: the same API and the same recording the
inner simulator opens when it searches for the cause (episode_scene.py). The experiments read the
same files with experiments/episode_series.py; both readers must give the same episode.

The API needs pydsr imported first, so that DSR attributes are registered; `open_recording` takes
care of the order. Recorded values come back as float32.

The API parses the numbers of the recording with the C library, which follows LC_NUMERIC. Inside
the agent, Qt takes the locale from the environment, and with a decimal comma (es_ES) "-3.699979"
reads as -3: the episode loses every decimal (first run in Webots, 06/10). `episode_from_memory`
reads under the C numeric locale, as the inner simulator does (it sets en_US when it starts).
"""

from __future__ import annotations

import hashlib
import locale
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Optional

import numpy as np

from src.episode_builder import (
    EpisodeError,
    RecordedSeries,
    build_episode,
    newest_rt_slot,
    to_meters,
    yaw_from_quaternion,
)

READER = "episodic_memory_api"
NANOSECONDS = 1e-9
READY_TIMEOUT_S = 10.0

ROOM, ROBOT, PERSON, BOTTLE, IMU = "room", "robot", "person", "bottle", "imu"


def open_recording(path: str | Path, timeout_s: float = READY_TIMEOUT_S):
    """An indexed EpisodicMemoryAPI over a recording (pydsr first, then the API)."""
    import pydsr  # noqa: F401  (registers the DSR attribute types the API returns)
    import episodic_memory_api

    api = episodic_memory_api.EpisodicMemoryAPI(str(path))
    start = time.time()
    while not api.is_ready() and time.time() - start < timeout_s:
        time.sleep(0.05)
    if not api.is_ready():
        raise EpisodeError(f"{path}: the episodic memory could not index the recording")
    return api


def _value(attribute) -> Any:
    return attribute.value if hasattr(attribute, "value") else attribute


def _attributes(event) -> dict[str, Any]:
    return {name: _value(attribute) for name, attribute in event.attributes.items()}


def _node_ids(api) -> dict[str, int]:
    """First id under which each node name appears, keyframe by keyframe."""
    ids: dict[str, int] = {}
    for index in range(api.get_keyframe_count()):
        for node in api.get_keyframe(index).nodes_list:
            if node.get("name") is not None and node.get("id") is not None:
                ids.setdefault(node["name"], node["id"])
    return ids


def read_series(api, path: Optional[str | Path] = None) -> RecordedSeries:
    """The series of the episode builder, read through an indexed EpisodicMemoryAPI."""
    path = Path(path if path is not None else api.get_filepath())
    ids = _node_ids(api)
    for name in (ROOM, ROBOT, IMU, BOTTLE):
        if name not in ids:
            raise EpisodeError(f"{path}: no '{name}' node in the recording")

    imu_events = api.get_node_history_by_name(IMU)
    if not imu_events:
        raise EpisodeError(f"{path}: no imu events")
    origin = int(imu_events[0].timestamp)
    file_start = int(api.get_keyframe_timestamp(0))

    def seconds(stamps) -> np.ndarray:
        return (np.asarray(stamps, dtype=np.int64) - origin) * NANOSECONDS

    acc, gyro = [], []
    for event in imu_events:
        if event.modification_type != "MNA":
            continue
        attributes = _attributes(event)
        if "imu_accelerometer" in attributes:
            acc.append((int(event.timestamp), [float(c) for c in attributes["imu_accelerometer"][:3]]))
        if "imu_gyroscope" in attributes:
            gyro.append((int(event.timestamp), [float(c) for c in attributes["imu_gyroscope"][:3]]))

    adv, rot = [], []
    for event in api.get_node_history_by_name(ROBOT):
        if event.modification_type != "MNA":
            continue
        attributes = _attributes(event)
        if "robot_ref_adv_speed" in attributes:
            adv.append((int(event.timestamp), float(attributes["robot_ref_adv_speed"])))
        if "robot_ref_rot_speed" in attributes:
            rot.append((int(event.timestamp), float(attributes["robot_ref_rot_speed"])))

    poses = []
    for event in api.get_edge_history(ids[ROOM], ids[ROBOT], "RT"):
        if event.modification_type != "MEA":
            continue
        attributes = _attributes(event)
        if "rt_translation" not in attributes:
            continue
        translation = attributes["rt_translation"]
        quaternion = attributes.get("rt_quaternion", [0.0, 0.0, 0.0, 1.0])
        poses.append((int(event.timestamp), float(translation[0]), float(translation[1]),
                      yaw_from_quaternion(quaternion)))
    if not poses:
        raise EpisodeError(f"{path}: no robot poses")

    person: dict[int, list[float]] = {}
    if PERSON in ids:
        state: dict[str, Any] = {}
        for event in api.get_edge_history(ids[ROBOT], ids[PERSON], "RT"):
            if event.modification_type != "MEA":
                continue
            state.update(_attributes(event))
            newest = newest_rt_slot(state.get("rt_translation"), state.get("rt_timestamps"))
            if newest is not None:
                stamp_ms, xyz = newest
                person[stamp_ms * 1_000_000 if stamp_ms else int(event.timestamp)] = xyz[:2]

    bottle_edge: list[tuple[int, bool]] = []
    for index in range(api.get_keyframe_count()):
        keyframe = api.get_keyframe(index)
        if any(edge.get("from") == ids[ROBOT] and edge.get("to") == ids[BOTTLE] and edge.get("type") == "RT"
               for edge in keyframe.edges_list):
            bottle_edge.append((int(keyframe.timestamp), True))
    for event in api.get_edge_history(ids[ROBOT], ids[BOTTLE], "RT"):
        bottle_edge.append((int(event.timestamp), event.modification_type != "DE"))
    bottle_edge.sort(key=lambda sample: sample[0])

    def vectors(samples):
        return (seconds([s[0] for s in samples]),
                np.asarray([s[1] for s in samples], dtype=float).reshape(-1, 3))

    def channel(samples):
        if not samples:
            return np.zeros((0, 2))
        return np.column_stack([seconds([s[0] for s in samples]), [s[1] for s in samples]])

    pose_array = np.asarray(poses, dtype=float)
    person_stamps = sorted(person)
    acc_t, acc_v = vectors(acc)
    gyro_t, gyro_v = vectors(gyro)
    return RecordedSeries(
        path=str(path),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else "",
        reader=READER,
        origin_ns=origin,
        origin_offset_from_file_start_s=(origin - file_start) * NANOSECONDS,
        length_s=(int(imu_events[-1].timestamp) - origin) * NANOSECONDS,
        pose_t=seconds([p[0] for p in poses]),
        pose_xy=to_meters(pose_array[:, 1:3]),
        pose_yaw=pose_array[:, 3],
        acc_t=acc_t, acc=acc_v,
        gyro_t=gyro_t, gyro=gyro_v,
        adv=channel(adv), rot=channel(rot),
        person_t=seconds(person_stamps),
        person_xy=to_meters(np.asarray([person[s] for s in person_stamps], dtype=float).reshape(-1, 2)),
        bottle_edge=[(float(t), present) for t, present in zip(seconds([b[0] for b in bottle_edge]),
                                                               [b[1] for b in bottle_edge])],
    )


#: Episodic-graph missions: the one recorded while following, and the one that starts when the
#: follow mission stops without the bottle (mission_controller).
FOLLOW_MISSION, SEARCH_MISSION = "Follow Person", "Search Problem Cause"


def recording_to_explain(episodic_nodes) -> Optional[str]:
    """Path of the recording to explain, or None while it is not ready.

    The same rule the inner simulator follows before simulating (check_for_problems), so that both
    agents read the same recording: a "Search Problem Cause" mission is running, and the last
    "Follow Person" mission has published its file path. By then the follow mission has stopped and
    its recording holds the fall and the robot's reaction.
    """
    search = follow = None
    for node in episodic_nodes:
        if node.name.startswith(SEARCH_MISSION):
            search = node
        if node.name.startswith(FOLLOW_MISSION):
            follow = node
    if search is None or follow is None or "status" not in search.attrs or "filepath" not in follow.attrs:
        return None
    if search.attrs["status"].value != "running":
        return None
    return follow.attrs["filepath"].value or None


@contextmanager
def c_numeric_locale():
    """LC_NUMERIC set to C (decimal point) while the episodic memory parses, and restored after."""
    previous = locale.setlocale(locale.LC_NUMERIC)
    locale.setlocale(locale.LC_NUMERIC, "C")
    try:
        yield
    finally:
        locale.setlocale(locale.LC_NUMERIC, previous)


def episode_from_memory(path: str | Path, episode_id: Optional[str] = None) -> dict[str, Any]:
    """Open the recording through the episodic memory and build its episode (under the C numeric
    locale, whatever the process has)."""
    with c_numeric_locale():
        return build_episode(read_series(open_recording(path), path), episode_id)
