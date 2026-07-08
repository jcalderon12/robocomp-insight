"""Contrastive verdict over simulated hypotheses.

A hypothesis is accepted iff one of its repetitions (a) reproduces the observed
effect (the bottle leaves the tray) and (b) fits the real IMU no worse than the
nominal run does, within a margin. Accepted hypotheses are ranked by IMU score
(normalized RMSE over the anomaly window of the real episode).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

import numpy as np

from src.hypothesis_compiler import NOMINAL_HYPOTHESIS_ID

TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
HISTORY = "history"
GENERATED_INSTANCES = "generated_instances"
BOTTLE_POSITION = "bottle_position"

DEFAULT_MARGIN = 0.15
DEFAULT_SCORE_TOLERANCE = 0.05
DEFAULT_WINDOW_PADDING_S = 1.5
DEFAULT_SETTLE_S = 2.0
DEFAULT_EFFECT_Z_FRACTION = 0.5
_EPS = 1e-9


def _safe_filename(text: str) -> str:
    return "".join(c if c.isalnum() or c in {"-", "_"} else "_" for c in str(text))


def estimate_anomaly_window(
    real_imu: dict,
    padding_s: float = DEFAULT_WINDOW_PADDING_S,
    settle_s: float = DEFAULT_SETTLE_S,
) -> tuple[float, float]:
    """Window around the strongest IMU deviation. The first `settle_s` seconds
    are excluded from the peak search: the spawn/settling transient can dwarf
    the actual anomaly."""
    times = np.asarray(real_imu[TIMESTAMP], dtype=float)
    acc = np.asarray(real_imu[ACCELEROMETER], dtype=float)
    if times.size == 0 or acc.size == 0:
        return 0.0, 0.0
    magnitude = np.linalg.norm(acc, axis=1)
    deviation = np.abs(magnitude - np.median(magnitude))
    searchable = times >= float(times[0]) + settle_s
    if not searchable.any():
        searchable = np.ones(times.size, dtype=bool)
    if float(deviation[searchable].max()) < 1e-6:
        return float(times[0]), float(times[-1])
    peak_index = int(np.argmax(np.where(searchable, deviation, -np.inf)))
    peak_time = float(times[peak_index])
    return (
        max(float(times[0]), peak_time - padding_s),
        min(float(times[-1]), peak_time + padding_s),
    )


def _windowed(real_imu: dict, window: tuple[float, float]) -> dict:
    times = np.asarray(real_imu[TIMESTAMP], dtype=float)
    mask = (times >= window[0]) & (times <= window[1])
    if not mask.any():
        mask = np.ones_like(times, dtype=bool)
    return {
        TIMESTAMP: times[mask],
        ACCELEROMETER: np.asarray(real_imu[ACCELEROMETER], dtype=float)[mask],
        GYROSCOPE: np.asarray(real_imu[GYROSCOPE], dtype=float)[mask],
    }


def score_repetition(real_window: dict, acc_std: float, gyro_std: float, sim_history: dict) -> float:
    """Normalized RMSE of the simulated IMU against the real one over the
    window, matching each real frame with the temporally closest simulated one."""
    sim_times = np.asarray(sim_history[TIMESTAMP], dtype=float)
    sim_acc = np.asarray(sim_history[ACCELEROMETER], dtype=float)
    sim_gyro = np.asarray(sim_history[GYROSCOPE], dtype=float)
    if sim_times.size == 0:
        return float("inf")

    indices = np.searchsorted(sim_times, real_window[TIMESTAMP])
    indices = np.clip(indices, 0, sim_times.size - 1)
    left = np.clip(indices - 1, 0, sim_times.size - 1)
    choose_left = np.abs(sim_times[left] - real_window[TIMESTAMP]) < np.abs(
        sim_times[indices] - real_window[TIMESTAMP]
    )
    nearest = np.where(choose_left, left, indices)

    acc_rmse = float(np.sqrt(np.mean(np.sum((sim_acc[nearest] - real_window[ACCELEROMETER]) ** 2, axis=1))))
    gyro_rmse = float(np.sqrt(np.mean(np.sum((sim_gyro[nearest] - real_window[GYROSCOPE]) ** 2, axis=1))))
    return acc_rmse / (acc_std + _EPS) + gyro_rmse / (gyro_std + _EPS)


def effect_reproduced(repetition: dict, initial_bottle_z: float, z_fraction: float = DEFAULT_EFFECT_Z_FRACTION) -> bool:
    """The bottle is off the tray when its final height dropped well below it."""
    final_position = repetition.get(BOTTLE_POSITION)
    if not final_position or initial_bottle_z <= 0.0:
        return False
    return float(final_position[2]) < z_fraction * initial_bottle_z


def build_verdict(
    case_id: str,
    real_imu: dict,
    entries: list[dict[str, Any]],
    historicals: list[list[dict[str, Any]]],
    initial_bottle_z: float,
    margin: float = DEFAULT_MARGIN,
    score_tolerance: float = DEFAULT_SCORE_TOLERANCE,
    window_padding_s: float = DEFAULT_WINDOW_PADDING_S,
    effect_z_fraction: float = DEFAULT_EFFECT_Z_FRACTION,
    skipped: Optional[list[dict[str, Any]]] = None,
) -> dict[str, Any]:
    """entries[i] is {'hypothesis_id', 'title', 'cause'} (nominal first);
    historicals[i] is the list of repetitions the causes simulator returned."""
    window = estimate_anomaly_window(real_imu, window_padding_s)
    real_window = _windowed(real_imu, window)
    acc_std = float(np.std(np.asarray(real_imu[ACCELEROMETER], dtype=float)))
    gyro_std = float(np.std(np.asarray(real_imu[GYROSCOPE], dtype=float)))

    evaluations = []
    nominal_best_score = None

    for entry, repetitions in zip(entries, historicals):
        rep_scores = []
        rep_effects = []
        for repetition in repetitions:
            rep_scores.append(score_repetition(real_window, acc_std, gyro_std, repetition[HISTORY]))
            rep_effects.append(effect_reproduced(repetition, initial_bottle_z, effect_z_fraction))

        best_index = int(np.argmin(rep_scores)) if rep_scores else -1
        effect_indices = [i for i, has_effect in enumerate(rep_effects) if has_effect]
        best_effect_index = (
            min(effect_indices, key=lambda i: rep_scores[i]) if effect_indices else None
        )
        report_index = best_effect_index if best_effect_index is not None else best_index

        evaluation = {
            "hypothesis_id": entry["hypothesis_id"],
            "title": entry.get("title", ""),
            "cause": entry["cause"],
            "repetitions": len(repetitions),
            "best_score": rep_scores[best_index] if rep_scores else None,
            "best_effect_score": rep_scores[best_effect_index] if best_effect_index is not None else None,
            "effect_rate": (sum(rep_effects) / len(rep_effects)) if rep_effects else 0.0,
            "best_repetition_index": report_index if repetitions else None,
            "best_generated_instances": repetitions[report_index][GENERATED_INSTANCES] if repetitions else {},
            "best_bottle_final_position": repetitions[report_index][BOTTLE_POSITION] if repetitions else None,
        }
        evaluations.append(evaluation)
        if entry["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID:
            nominal_best_score = evaluation["best_score"]

    nominal_effect_warning = False
    for evaluation in evaluations:
        if evaluation["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID:
            evaluation["accepted"] = False
            if evaluation["effect_rate"] > 0.0:
                # The effect appears with no intervention: no acceptance is trustworthy.
                nominal_effect_warning = True
            continue
        score = evaluation["best_effect_score"]
        evaluation["accepted"] = (
            score is not None
            and nominal_best_score is not None
            and score <= nominal_best_score * (1.0 + margin) + score_tolerance
        )

    accepted = [e for e in evaluations if e.get("accepted")]
    accepted_id = (
        min(accepted, key=lambda e: e["best_effect_score"])["hypothesis_id"] if accepted else None
    )

    return {
        "schema_version": "1.0",
        "case_id": case_id,
        "anomaly_window": {"start_s": window[0], "end_s": window[1]},
        "decision_rule": {
            "margin": margin,
            "score_tolerance": score_tolerance,
            "effect_z_fraction": effect_z_fraction,
            "initial_bottle_z": initial_bottle_z,
        },
        "nominal_best_score": nominal_best_score,
        "nominal_effect_warning": nominal_effect_warning,
        "hypotheses": evaluations,
        "skipped": skipped or [],
        "accepted_hypothesis_id": accepted_id,
    }


def write_verdict(verdict: dict[str, Any], output_dir: str) -> str:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_path = out_dir / f"verdict_{_safe_filename(verdict.get('case_id', 'case'))}.json"
    output_path.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    return str(output_path)


def _imu_figure(plt, real_imu: dict, sim_history: Optional[dict] = None, title: str = ""):
    axis_labels = ["X", "Y", "Z"]
    axis_colors = ["tab:red", "tab:green", "tab:blue"]
    figure = plt.figure(figsize=(12, 5))
    if title:
        plt.suptitle(title, fontsize=14)
    for subplot_index, (key, unit) in enumerate(
        [(ACCELEROMETER, "Acceleration (m/s^2)"), (GYROSCOPE, "Angular velocity (rad/s)")]
    ):
        plt.subplot(1, 2, subplot_index + 1)
        plt.title(key.capitalize())
        real_values = np.asarray(real_imu[key], dtype=float)
        sim_values = np.asarray(sim_history[key], dtype=float) if sim_history is not None else None
        for axis_index, (axis, color) in enumerate(zip(axis_labels, axis_colors)):
            plt.plot(real_imu[TIMESTAMP], real_values[:, axis_index], color=color, linestyle="-", label=f"Real {axis}")
            if sim_values is not None:
                plt.plot(sim_history[TIMESTAMP], sim_values[:, axis_index], color=color, linestyle="--", label=f"Sim {axis}")
        plt.xlabel("Time (s)")
        plt.ylabel(unit)
        plt.legend()
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    return figure


def save_real_imu_plot(real_imu: dict, output_path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure = _imu_figure(plt, real_imu, title="Real IMU history from Episodic Memory")
    figure.savefig(output_path, dpi=110)
    plt.close(figure)


def save_comparison_plots(real_imu: dict, entries: list[dict], historicals: list[list[dict]], output_dir: str) -> None:
    """One real-vs-simulated figure per hypothesis, using the repetition the
    verdict reported (entries may be the verdict evaluations)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for entry, repetitions in zip(entries, historicals):
        if not repetitions:
            continue
        repetition = repetitions[entry.get("best_repetition_index") or 0]
        title = f"Real vs simulated IMU — {entry['hypothesis_id']} ({entry['cause']['name']})"
        figure = _imu_figure(plt, real_imu, repetition[HISTORY], title)
        figure.savefig(out_dir / f"imu_comparison_{_safe_filename(entry['hypothesis_id'])}.png", dpi=110)
        plt.close(figure)
