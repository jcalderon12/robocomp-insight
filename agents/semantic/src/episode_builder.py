"""The episode of contract 1 (`insight.episode/1.0`), built from a mission recording.

The semantic memory does not copy the episodic one: it keeps an index with meaning over it (motion
phases, path segments, time intervals, the bottle held on the tray, the fall and a fixed list of
observable evidence), and points back to the recording it was built from.

This module is the reader-independent core. It takes the series already read from the recording
(`RecordedSeries`) and returns the episode as a JSON-ready dict. Two readers fill the series from the
same kind of recording: the episodic-memory API in production and a file reader in the experiments
and the offline tests.

Conventions (contract 1):
  * space: the DSR room frame, which is also the PyBullet world, in meters. Headings in degrees in
    that frame, 0 = +x and counter-clockwise: the DSR yaw + 90 deg, because the DSR robot frame is
    +y-forward.
  * IMU axes: x lateral, y forward, z up, gravity included.
  * time: seconds from the first event of the `imu` node, as the simulator and the verdict count it.
    t_obs is the first deletion of the robot->bottle RT edge; the horizon is min(L, t_obs + 3 s).
    The fall interval is the last 2 s of motion before t_obs, plus the final stop if the robot was
    stopped when the bottle fell; the free-motion baseline runs from 1 s to its start.
  * Nothing after t_obs enters the evidence except `bottle_reacquired` and what is explicitly marked
    as the system's reaction (the stop the semantic agent orders when the bottle is lost).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

SCHEMA = "insight.episode/1.0"
BUILDER = "episode_builder 1.2"  # contract 1, version 1.7 (the tray)

ROBOT = "Agent_Robot"
BOTTLE = "PhysicalObject_Bottle"
PERSON = "Agent_Person"
ROOM = "PhysicalPlace_Room"
#: The tray on top of the robot: a part of it, and what holds the bottle (not a DSR node).
TRAY = "PhysicalObject_Tray"

FALL_INTERVAL = "Interval_fall"
BASELINE_INTERVAL = "Interval_baseline"
REACTION_INTERVAL = "Interval_reaction"
REACTION_PHASE = "Phase_reaction"
FINAL_SEGMENT = "Segment_final"
ACCIDENT = "Accident_1"
SUPPORT = "Support_bottle"

#: The DSR robot frame is +y-forward; room headings count from +x.
DSR_TO_ROOM_HEADING_DEG = 90.0
#: concept_robot sends -robot_ref_rot_speed to the base: the robot turns against the setpoint sign.
ROT_SETPOINT_SIGN = -1.0
GRAVITY_M_S2 = 9.81

#: Motion: moving above this speed, turning above this yaw rate.
MOVING_SPEED_M_S = 0.05
TURN_RATE_RAD_S = 0.1
#: Shorter phases join the previous one.
MIN_PHASE_S = 1.0
#: Path segments, counted back from the pose at t_obs; a shorter first one joins the next.
SEGMENT_M = 1.0
MIN_SEGMENT_M = 0.1
#: Intervals.
FALL_WINDOW_S = 2.0
BASELINE_START_S = 1.0
EFFECT_HORIZON_MARGIN_S = 3.0
#: Forward setpoint filter: moving median over the zero-order-hold signal. Two writers alternate
#: 15 ms apart, which unfiltered looks like a step every half second.
SETPOINT_GRID_S = 0.01
SETPOINT_MEDIAN_S = 0.2
SETPOINT_STEP_WINDOW_S = 0.5
#: A sustained acceleration lasts this many consecutive IMU samples.
SUSTAINED_SAMPLES = 3

DECIMALS = 3


@dataclass
class RecordedSeries:
    """What the builder needs from one recording, already read and in production's time base.

    Times are seconds from the first `imu` event; positions are meters. Setpoint channels are
    (n, 2) arrays [t, value] with every recorded event, in recording order.
    """

    path: str                       #: the recording the episode points to
    sha256: str
    reader: str                     #: which reader filled the series
    origin_ns: int                  #: timestamp of the first imu event
    origin_offset_from_file_start_s: float
    length_s: float                 #: first to last imu event (L)
    pose_t: np.ndarray              #: room -> robot RT edge
    pose_xy: np.ndarray             #: (n, 2), room frame
    pose_yaw: np.ndarray            #: DSR yaw, rad
    acc_t: np.ndarray
    acc: np.ndarray                 #: (n, 3) m/s2
    gyro_t: np.ndarray
    gyro: np.ndarray                #: (n, 3) rad/s
    adv: np.ndarray                 #: robot_ref_adv_speed, m/s
    rot: np.ndarray                 #: robot_ref_rot_speed, rad/s
    person_t: np.ndarray            #: robot -> person RT edge
    person_xy: np.ndarray           #: (n, 2), robot frame
    #: (t, present): the robot->bottle RT edge seen in the graph (True) or deleted (False).
    bottle_edge: list[tuple[float, bool]] = field(default_factory=list)


class EpisodeError(ValueError):
    """Raised when a recording cannot yield an episode (no fall, no poses)."""


# --------------------------------------------------------------------------- #
# Shared by the readers: how a recorded value becomes a series sample
# --------------------------------------------------------------------------- #
#: Coordinates beyond this magnitude mean the whole edge history is in millimeters
#: (DSR convention) rather than meters (Webots bridge).
MM_DETECTION_THRESHOLD = 100.0


def yaw_from_quaternion(quaternion) -> float:
    """Yaw (rad) of an [x, y, z, w] quaternion."""
    x, y, z, w = (float(c) for c in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def newest_rt_slot(translation, timestamps) -> Optional[tuple[int, list[float]]]:
    """(timestamp_ms, xyz) of the newest pose of an RT edge.

    Some RT edges (robot -> person, robot -> bottle) are ring buffers of several poses with their
    `rt_timestamps` in ms; the newest slot is the current pose. A single-pose edge gives (0, xyz).
    """
    if translation is None or len(translation) < 3:
        return None
    if timestamps is None or len(timestamps) == 0 or len(translation) < 3 * len(timestamps):
        return 0, [float(c) for c in translation[:3]]
    slot = int(np.argmax(np.asarray(timestamps, dtype=np.uint64)))
    return int(timestamps[slot]), [float(c) for c in translation[3 * slot:3 * slot + 3]]


def to_meters(xy: np.ndarray) -> np.ndarray:
    """One unit decision for a whole edge history, not per sample."""
    if xy.size and np.abs(xy).max() > MM_DETECTION_THRESHOLD:
        return xy * 0.001
    return xy


# --------------------------------------------------------------------------- #
# Signal helpers
# --------------------------------------------------------------------------- #
def _r(value: Optional[float], decimals: int = DECIMALS) -> Optional[float]:
    if value is None:
        return None
    rounded = round(float(value), decimals)
    return 0.0 if rounded == 0.0 else rounded


def _xy(point: np.ndarray) -> list[float]:
    return [_r(point[0]), _r(point[1])]


def _zoh(samples: np.ndarray, times: np.ndarray, default: float = 0.0) -> np.ndarray:
    """Zero-order-hold value of [t, v] samples at `times`."""
    if samples.size == 0:
        return np.full(np.shape(times), default, dtype=float)
    index = np.searchsorted(samples[:, 0], times, side="right") - 1
    return np.where(index >= 0, samples[np.clip(index, 0, None), 1], default)


def _zoh_integral(samples: np.ndarray, start: float, end: float) -> float:
    """Integral over [start, end] of the zero-order-hold signal (0 before the first sample)."""
    if samples.size == 0 or end <= start:
        return 0.0
    t, v = samples[:, 0], samples[:, 1]
    edges = np.concatenate([[start], t[(t > start) & (t < end)], [end]])
    values = _zoh(samples, edges[:-1])
    return float(np.sum(np.diff(edges) * values))


def _moving_median(values: np.ndarray, width: int) -> np.ndarray:
    half = width // 2
    padded = np.pad(values, half, mode="edge")
    windows = np.lib.stride_tricks.sliding_window_view(padded, width)
    return np.median(windows, axis=1)


def _in(times: np.ndarray, interval: tuple[float, float]) -> np.ndarray:
    return (times >= interval[0]) & (times <= interval[1])


def _peak(times: np.ndarray, values: np.ndarray, interval: tuple[float, float]) -> Optional[tuple[float, float]]:
    """(signed value, instant) of the largest |value| inside the interval."""
    mask = _in(times, interval)
    if not mask.any():
        return None
    index = int(np.argmax(np.abs(values[mask])))
    return float(values[mask][index]), float(times[mask][index])


def _heading_deg(dsr_yaw: np.ndarray) -> np.ndarray:
    """Unwrapped room heading, degrees; the first sample is wrapped into (-180, 180]."""
    heading = np.degrees(np.unwrap(dsr_yaw)) + DSR_TO_ROOM_HEADING_DEG
    offset = heading[0] - _wrap_deg(heading[0])
    return heading - offset


def _wrap_deg(angle: float) -> float:
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


# --------------------------------------------------------------------------- #
# Motion: phases and path segments
# --------------------------------------------------------------------------- #
def _phase_runs(series: RecordedSeries, path: np.ndarray, t_obs: float) -> list[list[Any]]:
    """[kind, start, end] runs up to t_obs, from the motion between consecutive poses.

    Phases start when the robot first moves (faster than MOVING_SPEED_M_S); the wait before is not
    a phase. A run shorter than MIN_PHASE_S joins the previous one, and the first one the next.
    """
    t, yaw = series.pose_t, np.unwrap(series.pose_yaw)
    speeds = np.diff(path) / np.diff(t)
    moving = np.flatnonzero(speeds > MOVING_SPEED_M_S)
    runs: list[list[Any]] = []
    for i in range(int(moving[0]) if moving.size else 0, len(t) - 1):
        start, end = float(t[i]), float(min(t[i + 1], t_obs))
        if start >= t_obs or end <= start:
            break
        yaw_rate = (yaw[i + 1] - yaw[i]) / (t[i + 1] - t[i])
        if abs(yaw_rate) >= TURN_RATE_RAD_S:
            kind = "turn"
        elif speeds[i] > MOVING_SPEED_M_S:
            kind = "advance_straight"
        else:
            kind = "stopped"
        if runs and runs[-1][0] == kind:
            runs[-1][2] = end
        else:
            runs.append([kind, start, end])
    if runs and runs[-1][2] < t_obs:
        runs[-1][2] = t_obs  # the last pose before t_obs holds until t_obs

    while len(runs) > 1:
        short = next((i for i, run in enumerate(runs) if run[2] - run[1] < MIN_PHASE_S), None)
        if short is None:
            break
        if short == 0:
            runs[1][1] = runs[0][1]
        else:
            runs[short - 1][2] = runs[short][2]
        runs.pop(short)
        fused: list[list[Any]] = []
        for run in runs:
            if fused and fused[-1][0] == run[0]:
                fused[-1][2] = run[2]
            else:
                fused.append(run)
        runs = fused
    return runs


def _time_at_path(t: np.ndarray, path: np.ndarray, value: float) -> float:
    """First instant the cumulative path reaches `value` (path is non-decreasing)."""
    index = int(np.searchsorted(path, value, side="left"))
    if index <= 0:
        return float(t[0])
    if index >= len(path):
        return float(t[-1])
    span = path[index] - path[index - 1]
    fraction = (value - path[index - 1]) / span if span > 0 else 1.0
    return float(t[index - 1] + fraction * (t[index] - t[index - 1]))


def _pose_at(series: RecordedSeries, at: float) -> np.ndarray:
    return np.array([np.interp(at, series.pose_t, series.pose_xy[:, 0]),
                     np.interp(at, series.pose_t, series.pose_xy[:, 1])])


def _segments(series: RecordedSeries, path: np.ndarray, t_obs: float, t_start: float) -> list[dict[str, Any]]:
    """Segments of SEGMENT_M of path counted back from the pose at t_obs, latest first."""
    path_obs = float(np.interp(t_obs, series.pose_t, path))
    bounds: list[tuple[float, float]] = []
    end = path_obs
    while end > 0.0:
        start = max(0.0, end - SEGMENT_M)
        bounds.append((start, end))
        end = start
    if len(bounds) > 1 and bounds[-1][1] - bounds[-1][0] < MIN_SEGMENT_M:
        short = bounds.pop()
        bounds[-1] = (short[0], bounds[-1][1])
    if not bounds:  # the robot never moved: one empty segment at the fall pose
        bounds = [(path_obs, path_obs)]

    def time_at(value: float) -> float:
        if value >= path_obs:
            return t_obs
        return max(t_start, _time_at_path(series.pose_t, path, value))

    segments = []
    for order, (start, end) in enumerate(bounds, start=1):
        t0, t1 = time_at(start), time_at(end)
        inside = _in(series.pose_t, (t0, t1))
        points = np.vstack([_pose_at(series, t0), series.pose_xy[inside], _pose_at(series, t1)])
        origin, target = points[0], points[-1]
        segment_id = FINAL_SEGMENT if order == 1 else f"Segment_prev_{order - 1}"
        length = end - start
        segments.append({
            "id": segment_id,
            "order_back_from_fall": order,
            "interval": f"Interval_{segment_id}",
            "from_xy": _xy(origin),
            "to_xy": _xy(target),
            "bbox": {"x": [_r(points[:, 0].min()), _r(points[:, 0].max())],
                     "y": [_r(points[:, 1].min()), _r(points[:, 1].max())]},
            "length_m": _r(length),
            "heading_deg": _r(math.degrees(math.atan2(target[1] - origin[1], target[0] - origin[0])), 1),
            "mean_speed_mps": _r(length / (t1 - t0)) if t1 > t0 else 0.0,
            "_span": (t0, t1),
        })
    return segments


# --------------------------------------------------------------------------- #
# The episode
# --------------------------------------------------------------------------- #
def observed_effect_time(series: RecordedSeries) -> Optional[float]:
    """t_obs: the first deletion of the robot->bottle RT edge."""
    return next((t for t, present in series.bottle_edge if not present), None)


def build_episode(series: RecordedSeries, episode_id: Optional[str] = None) -> dict[str, Any]:
    """The episode of contract 1 as a JSON-ready dict."""
    t_obs = observed_effect_time(series)
    if t_obs is None:
        raise EpisodeError(f"{series.path}: the robot->bottle edge is never deleted; there is no fall.")
    if len(series.pose_t) < 2:
        raise EpisodeError(f"{series.path}: fewer than two robot poses.")

    length = float(series.length_s)
    horizon = min(length, t_obs + EFFECT_HORIZON_MARGIN_S)
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(series.pose_xy, axis=0), axis=1))])
    heading = _heading_deg(series.pose_yaw)

    # Filtered forward setpoint, as the simulator holds it.
    grid = np.arange(0.0, horizon + SETPOINT_GRID_S / 2, SETPOINT_GRID_S)
    setpoint = _moving_median(_zoh(series.adv, grid), int(round(SETPOINT_MEDIAN_S / SETPOINT_GRID_S)) + 1)

    # ---- intervals --------------------------------------------------------
    runs = _phase_runs(series, path, t_obs)
    t_start = runs[0][1] if runs else float(series.pose_t[0])
    segments = _segments(series, path, t_obs, t_start)

    # The fall interval covers the last FALL_WINDOW_S of motion before the fall. If the robot was
    # stopped when the bottle fell (e.g. stalled against an obstacle), that includes the stop:
    # the cause may have acted when the robot stopped, seconds before the fall.
    last_motion = runs[-1][1] if runs and runs[-1][0] == "stopped" else t_obs
    fall = (max(0.0, last_motion - FALL_WINDOW_S), t_obs)
    baseline = (BASELINE_START_S, fall[0]) if fall[0] > BASELINE_START_S else None
    after = series.adv[(series.adv[:, 0] > t_obs) & (series.adv[:, 1] == 0.0)] if series.adv.size else series.adv
    reaction_onset = float(after[0, 0]) if len(after) else None
    reaction = (reaction_onset, length) if reaction_onset is not None else None

    intervals = [{"id": FALL_INTERVAL, "start_s": _r(fall[0]), "end_s": _r(fall[1])}]
    if baseline:
        intervals.append({"id": BASELINE_INTERVAL, "start_s": _r(baseline[0]), "end_s": _r(baseline[1])})

    # ---- phases -----------------------------------------------------------
    phases = []
    phase_spans: dict[str, tuple[float, float]] = {}
    for number, (kind, start, end) in enumerate(runs, start=1):
        phase_id = f"Phase_{number}"
        phase_spans[phase_id] = (start, end)
        intervals.append({"id": f"Interval_{phase_id}", "start_s": _r(start), "end_s": _r(end)})
        # The setpoint held when the phase starts, and every one recorded during it.
        ordered = np.concatenate([_zoh(series.adv, np.array([start])),
                                  series.adv[_in(series.adv[:, 0], (start, end)), 1]]) if series.adv.size else []
        on_phase = _in(series.pose_t, (start, end))
        headings = np.concatenate([[np.interp(start, series.pose_t, heading)], heading[on_phase],
                                   [np.interp(end, series.pose_t, heading)]])
        travelled = float(np.interp(end, series.pose_t, path) - np.interp(start, series.pose_t, path))
        phases.append({
            "id": phase_id, "kind": kind, "interval": f"Interval_{phase_id}",
            "mean_speed_mps": _r(travelled / (end - start)),
            # Every recorded setpoint of the phase, both writers (descriptive; rules use the filter).
            "setpoint_mps": [_r(ordered.min(), 2), _r(ordered.max(), 2)] if len(ordered) else None,
            "heading_deg_range": [_r(headings.min(), 1), _r(headings.max(), 1)],
        })

    for segment in segments:
        start, end = segment.pop("_span")
        intervals.append({"id": segment["interval"], "start_s": _r(start), "end_s": _r(end)})

    if reaction:
        intervals.append({"id": REACTION_INTERVAL, "start_s": _r(reaction[0]), "end_s": _r(reaction[1])})
        stopped_at = _stopped_after(series, path, reaction_onset)
        note = (f"setpoint to 0 at {reaction_onset:.2f} s ({reaction_onset - t_obs:.2f} s after t_obs); "
                + (f"robot stopped at {stopped_at:.2f} s" if stopped_at is not None else "robot never stopped"))
        phases.append({"id": REACTION_PHASE, "kind": "system_reaction", "interval": REACTION_INTERVAL,
                       "is_system_reaction": True, "stopped_at_s": _r(stopped_at), "note": note})

    # ---- the bottle and the fall ------------------------------------------
    present = [t for t, seen in series.bottle_edge if seen and t <= t_obs]
    support = {
        "id": SUPPORT, "supporter": TRAY, "supported": BOTTLE,
        "start_s": _r(max(0.0, min(present))) if present else None,
        "end_s": _r(t_obs), "ended_by": ACCIDENT,
    }
    fall_xy = _pose_at(series, t_obs)
    accident = {
        "id": ACCIDENT, "time_s": _r(t_obs), "place": FINAL_SEGMENT,
        "robot_pose": {"xy": _xy(fall_xy),
                       "heading_deg": _r(_wrap_deg(float(np.interp(t_obs, series.pose_t, heading))), 1)},
        "participants": [ROBOT, BOTTLE],
        "cause": "unknown", "observed_as": "deletion of the robot->bottle RT edge",
    }

    evidence = _evidence(series, path, heading, grid, setpoint, t_obs, fall, baseline, reaction, phase_spans)

    origin_offset = float(series.origin_offset_from_file_start_s)
    return {
        "schema": SCHEMA,
        "episode_id": episode_id or _default_episode_id(series.path),
        "source": {"kind": "recording", "path": series.path, "sha256": series.sha256,
                   "builder": BUILDER, "reader": series.reader},
        "frame": {"name": "room", "units": "m", "heading": "deg, room frame, 0 = +x, counter-clockwise"},
        "time": {
            "origin": "first_imu_event",
            "origin_ns": int(series.origin_ns),
            "origin_offset_from_file_start_s": _r(origin_offset),
            "t_obs_s": _r(t_obs),
            "episode_length_s": _r(length),
            "simulation_horizon_s": _r(horizon),
        },
        "entities": {"robot": ROBOT, "tray": TRAY, "bottle": BOTTLE, "person": PERSON, "room": ROOM},
        "intervals": intervals,
        "phases": phases,
        "segments": segments,
        "support": support,
        "accident": accident,
        "evidence": evidence,
    }


def _default_episode_id(path: str) -> str:
    stem = path.replace("\\", "/").rsplit("/", 1)[-1]
    return "rec_" + (stem[:-4] if stem.endswith(".txt") else stem)


def _stopped_after(series: RecordedSeries, path: np.ndarray, since: float) -> Optional[float]:
    """First pose after `since` from which the robot moves slower than MOVING_SPEED_M_S."""
    t = series.pose_t
    for i in range(len(t) - 1):
        if t[i] < since:
            continue
        if (path[i + 1] - path[i]) / (t[i + 1] - t[i]) <= MOVING_SPEED_M_S:
            return float(t[i])
    return None


# --------------------------------------------------------------------------- #
# Evidence: a fixed list of observable properties
# --------------------------------------------------------------------------- #
def _peak_evidence(evidence_id: str, prop: str, unit: str, times: np.ndarray, values: np.ndarray,
                   fall: tuple[float, float], baseline: Optional[tuple[float, float]]) -> dict[str, Any]:
    """Largest |value| in the fall interval, and the same over the free-motion baseline."""
    peak = _peak(times, values, fall)
    entry = {"id": evidence_id, "property": prop, "interval": FALL_INTERVAL,
             "value": _r(abs(peak[0])) if peak else None, "unit": unit,
             "at_s": _r(peak[1]) if peak else None}
    if peak:
        entry["signed_value"] = _r(peak[0])
    base = _peak(times, values, baseline) if baseline else None
    entry["baseline"] = _r(abs(base[0])) if base else None
    return entry


def _evidence(series: RecordedSeries, path: np.ndarray, heading: np.ndarray, grid: np.ndarray,
              setpoint: np.ndarray, t_obs: float, fall: tuple[float, float],
              baseline: Optional[tuple[float, float]], reaction: Optional[tuple[float, float]],
              phase_spans: dict[str, tuple[float, float]]) -> list[dict[str, Any]]:
    acc_t, acc, gyro_t, gyro = series.acc_t, series.acc, series.gyro_t, series.gyro
    evidence = [
        _peak_evidence("Ev_pitch_rate_peak", "pitch_rate_peak", "rad/s", gyro_t, gyro[:, 0], fall, baseline),
        _peak_evidence("Ev_vertical_accel_peak", "vertical_accel_peak", "m/s2",
                       acc_t, acc[:, 2] - GRAVITY_M_S2, fall, baseline),
        _peak_evidence("Ev_longitudinal_accel_peak", "longitudinal_accel_peak", "m/s2",
                       acc_t, acc[:, 1], fall, baseline),
        _peak_evidence("Ev_lateral_accel_peak", "lateral_accel_peak", "m/s2", acc_t, acc[:, 0], fall, baseline),
    ]

    # Yaw: what the poses show against what the setpoint ordered (the robot turns against its sign).
    yaw_change = float(np.interp(fall[1], series.pose_t, heading) - np.interp(fall[0], series.pose_t, heading))
    commanded = math.degrees(ROT_SETPOINT_SIGN * _zoh_integral(series.rot, fall[0], fall[1]))
    evidence.append({"id": "Ev_yaw_change", "property": "yaw_change", "interval": FALL_INTERVAL,
                     "value": _r(yaw_change, 2), "unit": "deg", "commanded": _r(commanded, 2)})
    evidence.append(_peak_evidence("Ev_yaw_rate_peak", "yaw_rate_peak", "rad/s",
                                   gyro_t, gyro[:, 2], fall, baseline))

    # Largest change of the filtered forward setpoint within SETPOINT_STEP_WINDOW_S.
    lag = int(round(SETPOINT_STEP_WINDOW_S / SETPOINT_GRID_S))
    inside = np.flatnonzero(_in(grid, fall))
    steps = [abs(setpoint[i] - setpoint[i - lag]) for i in inside if i - lag >= inside[0]]
    evidence.append({"id": "Ev_setpoint_step", "property": "setpoint_step", "interval": FALL_INTERVAL,
                     "value": _r(max(steps)) if steps else None, "unit": "m/s"})

    # Travelled over ordered, relative to the episode's own free motion (the Webots clock).
    def own_ratio(interval: Optional[tuple[float, float]]) -> Optional[float]:
        if interval is None:
            return None
        ordered = _zoh_integral(series.adv, *interval)
        travelled = float(np.interp(interval[1], series.pose_t, path) - np.interp(interval[0], series.pose_t, path))
        return travelled / ordered if ordered > 0 else None

    ratio_fall, ratio_base = own_ratio(fall), own_ratio(baseline)
    evidence.append({"id": "Ev_speed_ratio", "property": "speed_ratio", "interval": FALL_INTERVAL,
                     "value": _r(ratio_fall / ratio_base) if ratio_fall is not None and ratio_base else None,
                     "unit": "1", "own_ratio_fall": _r(ratio_fall), "own_ratio_baseline": _r(ratio_base)})

    # Largest horizontal acceleration held over SUSTAINED_SAMPLES consecutive samples, before t_obs.
    before = acc_t <= t_obs
    if phase_spans:
        before &= acc_t >= min(start for start, _ in phase_spans.values())
    horizontal = np.hypot(acc[before, 0], acc[before, 1])
    sustained_entry = {"id": "Ev_sustained_accel", "property": "max_sustained_horizontal_accel",
                       "interval": None, "value": None, "unit": "m/s2", "at_s": None}
    if len(horizontal) >= SUSTAINED_SAMPLES:
        held = np.lib.stride_tricks.sliding_window_view(horizontal, SUSTAINED_SAMPLES).min(axis=1)
        index = int(np.argmax(held))
        at = float(acc_t[before][index])
        sustained_entry.update(value=_r(held[index]), at_s=_r(at),
                               interval=_containing_phase(phase_spans, at))
    evidence.append(sustained_entry)

    near = _in(series.person_t, fall)
    distances = np.linalg.norm(series.person_xy[near], axis=1) if near.any() else None
    evidence.append({"id": "Ev_person_distance", "property": "person_distance_min", "interval": FALL_INTERVAL,
                     "value": _r(distances.min()) if distances is not None else None, "unit": "m"})

    back = [t for t, seen in series.bottle_edge if seen and t > t_obs]
    reacquired = {"id": "Ev_bottle_reacquired", "property": "bottle_reacquired", "value": bool(back)}
    if back:
        reacquired["at_s"] = _r(back[0])
    evidence.append(reacquired)

    if reaction:
        evidence.append({"id": "Ev_reaction_onset", "property": "reaction_onset",
                         "value": _r(reaction[0]), "unit": "s"})
        braking = _peak(acc_t, acc[:, 1], reaction)
        evidence.append({"id": "Ev_reaction_braking", "property": "longitudinal_accel_peak",
                         "interval": REACTION_INTERVAL, "value": _r(braking[0]) if braking else None,
                         "unit": "m/s2", "at_s": _r(braking[1]) if braking else None,
                         "is_system_reaction": True})
    return evidence


def _containing_phase(phase_spans: dict[str, tuple[float, float]], at: float) -> Optional[str]:
    for phase_id, (start, end) in phase_spans.items():
        if start <= at <= end:
            return f"Interval_{phase_id}"
    return None
