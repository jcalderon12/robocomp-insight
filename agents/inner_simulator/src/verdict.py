"""Verdict over simulated hypotheses: approximate Bayesian model choice on the robot's own record.

Design and its justification: docs_output/diseno/veredicto.md. In short, for the episode's anomaly
interval W = [t_a, t_obs] (the semantic memory's Interval_fall) and every hypothesis H_k of the batch
plus the nominal H_0, each simulated from the recorded state shortly before W (episode_scene):

  * discrepancy, the same for every hypothesis: over W, the RMS error of the accelerometer, the
    gyroscope (robot frame) and the planar path, each over its scale, at the best common time shift
    |tau| <= TAU_MAX_S (the alignment uncertainty between the recording and the simulation):

        d(x, y) = min_tau  sum_c  RMS_W( x_c(t + tau) - y_c(t) ) / sigma_c

  * the observed effect (perception lost the bottle at t_obs) enters with the weight of its measured
    reliability, q(E | x): one value if the repetition loses the bottle within +-EFFECT_DELTA_S of
    t_obs, another if it loses it at another time, another if it does not lose it;
  * approximate likelihood, an ABC kernel whose width is the simulator's model error:

        p(y | H_k) ~= (1/N) sum_i exp(-d_i^2 / (2 eps^2)) q(E | x_i)

    repetitions that are not physically meaningful (blown-up physics, a body spawned overlapping
    the robot) contribute 0 and still count in N;
  * posterior with the same prior for every mechanism (the nominal is one), split among the
    hypotheses of a mechanism:  P(H_k | y) = pi_k p(y | H_k) / sum_j pi_j p(y | H_j);
  * decision: the most probable hypothesis is accepted if its posterior is at least P_MIN (Chow's
    reject rule with a reject cost of 1/10 of an error); otherwise the verdict abstains. If the
    nominal is the most probable, nothing simulated explains the episode better than no cause.
  * validity: before W, between the seed and t_a, nothing anomalous happened; if the nominal is
    farther from the record there than ADEQUACY_MAX, the simulation is not faithful for this
    episode and the verdict abstains without comparing hypotheses.

The Monte Carlo error of the posteriors is reported (bootstrap over the repetitions), and so are,
for every hypothesis, its parameters weighted by how well each repetition fits.

A repetition may carry its distance already computed (DISTANCE, by causes_simulator.doFastSimulations,
which keeps the history of the nominal and of its best repetition only); one it stopped early
(STOPPED_EARLY) is sure to lie where its kernel weight is negligible, and counts as 0 like an invalid one.
A repetition drawn by the search in rounds (search_rounds.py) carries LOG_WEIGHT, log(prior / the mixture of
the proposals it came from), which multiplies its kernel: the likelihood stays an estimate over the prior.

Adequacy of the explanation (docs_output/resultados/44-46): the most probable hypothesis is accepted only if
its best repetition leaves the distance at most EXPLANATION_RATIO_MAX times the nominal's. Otherwise it does
not explain the record well enough, whatever its probability among the hypotheses of the batch (the true cause
may not be in it), and the verdict abstains.

Calibration: EPS, PATH_SCALE_M, the effect weights and ADEQUACY_MAX are to be calibrated on the
Webots bench (docs_output/resultados/32) by the leave-one-out log score of the true cause; until
then they hold PROVISIONAL values and every verdict says so (`calibrated: false`).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Optional

import numpy as np

from src.hypothesis_compiler import NOMINAL_HYPOTHESIS_ID

TIMESTAMP = "timestamp"
ACCELEROMETER = "accelerometer"
GYROSCOPE = "gyroscope"
POSITION = "position"
HISTORY = "history"
GENERATED_INSTANCES = "generated_instances"
BOTTLE_POSITION = "bottle_position"
FALL_TIME = "fall_time"
ROBOT_POSITION_AT_FALL = "robot_position_at_fall"
ROBOT_FINAL_POSITION = "robot_final_position"
SPAWN_OVERLAP = "spawn_overlap"
EFFECT_TIME = "effect_time"
DISTANCE = "distance"
STOPPED_EARLY = "stopped_early"
LOG_WEIGHT = "log_weight"

SCHEMA_VERSION = "2.0"
DEFAULT_EFFECT_Z_FRACTION = 0.5
_EPS = 1e-12

# --------------------------------------------------------------------------- #
# Parameters (docs_output/diseno/veredicto.md, sections 3.4-3.6 and 4)
# --------------------------------------------------------------------------- #
#: Alignment uncertainty between the recording and the simulation: on the Webots bench the simulated
#: loss of the bottle came 0.19-0.29 s (median) from the recorded one.
TAU_MAX_S = 0.3
TAU_STEP_S = 0.02
#: An effect within this span of t_obs is the observed one.
EFFECT_DELTA_S = 0.5
#: Chow's reject rule with a reject cost of 1/10 of the cost of a wrong explanation.
P_MIN = 0.9
#: Bootstrap resamples of the repetitions, for the Monte Carlo error of the posteriors.
BOOTSTRAP = 200
BOOTSTRAP_SEED = 0
#: The kernel width, in units of d, acts as a temperature, not as the model error (part of the error is
#: common to every hypothesis). With 144 repetitions it was calibrated by the leave-one-out log score of the
#: true mechanism over the review bench (docs_output/resultados/35_banco_de_verificacion: 14 Webots cases
#: with the memory's anchored enumeration and the LLM batch of 19/10 13:26, and 31 synthetic cases of other
#: causes): 0.04. With DEFAULT_REPETITIONS = 4000 (resultados/43_epsilon_para_4000) the same bench no longer
#: bounds it from below: the log score is flat from 0.002 to 0.02, and every eps in 0.002-0.15 accepts the
#: 14 Webots cases and no wrong cause. It stays at 0.04, the smallest width at which the simulator's own
#: repetition-order noise (median 5e-4 on the best repetition's distance, resultados/41) moves a weight by
#: less than a factor e: a sharper kernel would let that noise decide. The path scale stays 5 cm. The effect
#: weights stay equal: the weights measured on the bench (on time 0.50, otherwise 0.21) did not improve the
#: log score on the Webots cases. No validity bound yet.
EPS = 0.04
#: The adequacy of the explanation: the accepted hypothesis' best repetition must leave the distance at most this
#: fraction of the nominal's. Calibrated on the review bench (resultados/46: 45 cases, each with and without the true
#: mechanism in its batch; 10 for a wrong acceptance, 1 for a correct one lost, the Chow cost of P_MIN), with the
#: search in rounds of search_rounds.py: no acceptance without the true cause, one Webots run lost (a 5 cm bump the
#: robot stalls against, 0.634). On the 20/10 evaluation it turns the two robot pushes the batch did not hold (0.73 and
#: 0.81, taken for bumps) into abstentions and keeps the 8 bumps (0.38-0.53). Depends on the search: recalibrate if it
#: changes.
EXPLANATION_RATIO_MAX = 0.615
PATH_SCALE_M = 0.05
EFFECT_WEIGHTS = {"on_time": 1.0, "off_time": 1.0, "none": 1.0}
ADEQUACY_MAX: Optional[float] = None
CALIBRATED = True

#: Mechanism of a cause, for the prior when the batch does not name it.
NOMINAL_MECHANISM = "nominal"

ABSTAIN_NOT_FAITHFUL = "simulation_not_faithful"
ABSTAIN_BELOW_THRESHOLD = "no_hypothesis_above_threshold"
ABSTAIN_NOMINAL = "nominal_most_probable"
ABSTAIN_INADEQUATE = "explanation_not_adequate"
ABSTAIN_NO_DATA = "no_record_in_interval"

# Physical plausibility of a repetition (millimeters, like the simulator output).
# A bottle lying on the floor sits at z ~ 25 mm; the robot base stays near it.
FLOOR_TOLERANCE_MM = 50.0
ROBOT_Z_RANGE_MM = (-100.0, 1000.0)
# A bottle knocked off the tray lands next to the robot; farther away means it was flung by a solver
# explosion or an unrealistic push.
MAX_LANDING_DISTANCE_MM = 3000.0


def _safe_filename(text: str) -> str:
    return "".join(c if c.isalnum() or c in {"-", "_"} else "_" for c in str(text))


# --------------------------------------------------------------------------- #
# Repetition checks
# --------------------------------------------------------------------------- #
def _finite(values) -> bool:
    return values is not None and len(values) >= 3 and all(math.isfinite(float(v)) for v in values)


def physical_problem(repetition: dict) -> Optional[str]:
    """Why a repetition is not physically meaningful, or None when it is."""
    if repetition.get(SPAWN_OVERLAP):
        return "obstacle placed overlapping the robot or the bottle"
    bottle = repetition.get(BOTTLE_POSITION)
    if not _finite(bottle):
        return "bottle pose missing or not finite"
    if float(bottle[2]) < -FLOOR_TOLERANCE_MM:
        return "bottle ended below the floor"
    robot = repetition.get(ROBOT_FINAL_POSITION)
    if robot is not None:
        if not _finite(robot):
            return "robot pose not finite"
        if not ROBOT_Z_RANGE_MM[0] <= float(robot[2]) <= ROBOT_Z_RANGE_MM[1]:
            return "robot base left the floor"
    if bottle_on_floor(repetition, None) and (landing_distance_mm(repetition) or 0.0) > MAX_LANDING_DISTANCE_MM:
        return "bottle flung beyond a plausible landing distance"
    return None


def bottle_on_floor(repetition: dict, initial_bottle_z: Optional[float], z_fraction: float = DEFAULT_EFFECT_Z_FRACTION) -> bool:
    """The bottle ended well below the tray (on the floor). Without an initial height, below 0.4 m."""
    final_position = repetition.get(BOTTLE_POSITION)
    if not final_position:
        return False
    limit = z_fraction * initial_bottle_z if initial_bottle_z else 400.0
    return float(final_position[2]) < limit


def landing_distance_mm(repetition: dict) -> Optional[float]:
    """Horizontal distance from the robot, when the bottle fell, to where it ended."""
    bottle = repetition.get(BOTTLE_POSITION)
    robot = repetition.get(ROBOT_POSITION_AT_FALL)
    if not _finite(bottle) or not _finite(robot):
        return None
    return float(math.hypot(float(bottle[0]) - float(robot[0]), float(bottle[1]) - float(robot[1])))


def effect_time(repetition: dict) -> Optional[float]:
    """When perception would have lost the bottle in the simulation (concept_bottle's rule); for
    repetitions from before that rule, the first instant the bottle was below the effect height."""
    value = repetition.get(EFFECT_TIME) if EFFECT_TIME in repetition else repetition.get(FALL_TIME)
    return float(value) if value is not None else None


def effect_reproduced(repetition: dict, initial_bottle_z: float, z_fraction: float = DEFAULT_EFFECT_Z_FRACTION) -> bool:
    """Perception would have lost the bottle; older repetitions: it ended on the floor."""
    if EFFECT_TIME in repetition:
        return repetition[EFFECT_TIME] is not None
    return bottle_on_floor(repetition, initial_bottle_z, z_fraction)


def plausible_effect(repetition: dict, initial_bottle_z: float, z_fraction: float = DEFAULT_EFFECT_Z_FRACTION) -> bool:
    """The effect, in a physically meaningful repetition (kept for the offline studies)."""
    return physical_problem(repetition) is None and effect_reproduced(repetition, initial_bottle_z, z_fraction)


# --------------------------------------------------------------------------- #
# The discrepancy
# --------------------------------------------------------------------------- #
def sensor_spread(values) -> float:
    """Spread of a three-axis signal: the root mean of its per-axis variances, each axis about its own
    mean, so that gravity on z is not spread and an axis that barely moves is not blown up."""
    values = np.asarray(values, dtype=float).reshape(-1, 3)
    return float(np.sqrt(np.mean(values.var(axis=0)))) if len(values) else 0.0


def _channels(record: dict, interval: tuple[float, float], until: float) -> list[dict]:
    """The real channels over the interval: times, values and scale of each."""
    channels = []
    for key in (ACCELEROMETER, GYROSCOPE):
        times = np.asarray(record[key][TIMESTAMP], dtype=float)
        values = np.asarray(record[key]["values"], dtype=float).reshape(-1, 3)
        mask = (times >= interval[0]) & (times <= interval[1])
        if not mask.any():
            continue
        scale = sensor_spread(values[times <= until])
        if scale <= 0.0:
            continue
        channels.append({"key": key, "times": times[mask], "values": values[mask], "scale": scale})
    path = record.get("path") or {}
    times = np.asarray(path.get(TIMESTAMP, []), dtype=float)
    if times.size:
        xy = np.asarray(path["xy"], dtype=float).reshape(-1, 2)
        mask = (times >= interval[0]) & (times <= interval[1])
        if mask.any():
            channels.append({"key": POSITION, "times": times[mask], "values": xy[mask], "scale": PATH_SCALE_M})
    return channels


def discrepancy(channels: list[dict], history: dict) -> float:
    """d(x, y): over the interval, the sum over channels of RMS(simulated - real) / scale, at the best
    common time shift within +-TAU_MAX_S. The simulated signal is held at each real instant (the last
    simulated sample at or before it)."""
    sim_times = np.asarray(history.get(TIMESTAMP, []), dtype=float)
    # Only the channels the simulation recorded (repetitions from before the path have none).
    channels = [c for c in channels if len(history.get(c["key"], [])) == sim_times.size]
    if sim_times.size == 0 or not channels:
        return float("inf")
    simulated = {c["key"]: np.asarray(history[c["key"]], dtype=float) for c in channels}
    best = float("inf")
    for shift in np.arange(-TAU_MAX_S, TAU_MAX_S + 1e-9, TAU_STEP_S):
        total = 0.0
        for channel in channels:
            index = np.clip(np.searchsorted(sim_times, channel["times"] + shift, side="right") - 1, 0, sim_times.size - 1)
            error = simulated[channel["key"]][index] - channel["values"]
            total += float(np.sqrt(np.mean(np.sum(error ** 2, axis=1) / error.shape[1]))) / channel["scale"]
        best = min(best, total)
    return best


def repetition_distance(channels: list[dict], repetition: dict) -> float:
    """d of a repetition: the one the simulator computed if it carries it (inf if it stopped the
    repetition early), its discrepancy from its history otherwise."""
    if DISTANCE in repetition:
        value = repetition[DISTANCE]
        return float("inf") if value is None or repetition.get(STOPPED_EARLY) else float(value)
    return discrepancy(channels, repetition.get(HISTORY, {}))


def scene_channels(record: dict, scene: dict) -> list[dict]:
    """The real channels the verdict compares on for a scene (its anomaly interval, up to t_obs); empty
    when the scene has no interval or no observed effect."""
    interval, t_obs = scene.get("anomaly_interval"), scene.get("observed_effect_time")
    if not interval or t_obs is None:
        return []
    return _channels(record, (interval[0], interval[1]), t_obs)


def channels_to_json(channels: list[dict]) -> list[dict]:
    return [{"key": c["key"], "times": np.asarray(c["times"]).tolist(), "values": np.asarray(c["values"]).tolist(),
             "scale": float(c["scale"])} for c in channels]


def channels_from_json(data: list[dict]) -> list[dict]:
    return [{"key": c["key"], "times": np.asarray(c["times"], dtype=float),
             "values": np.asarray(c["values"], dtype=float).reshape(len(c["times"]), -1), "scale": float(c["scale"])}
            for c in data]


def effect_weight(repetition: dict, observed_effect_time: Optional[float]) -> tuple[float, str]:
    """q(E_obs | x) and which case applies."""
    simulated = effect_time(repetition)
    if simulated is None:
        return EFFECT_WEIGHTS["none"], "none"
    if observed_effect_time is not None and abs(simulated - observed_effect_time) <= EFFECT_DELTA_S:
        return EFFECT_WEIGHTS["on_time"], "on_time"
    return EFFECT_WEIGHTS["off_time"], "off_time"


# --------------------------------------------------------------------------- #
# The verdict
# --------------------------------------------------------------------------- #
def _log_mean_exp(values: np.ndarray) -> float:
    if values.size == 0 or not np.isfinite(values).any():
        return float("-inf")
    top = float(np.max(values))
    return top + math.log(float(np.mean(np.exp(values - top))))


def _mechanism(entry: dict) -> str:
    if entry["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID:
        return NOMINAL_MECHANISM
    if entry.get("mechanism"):
        return str(entry["mechanism"])
    cause = entry.get("cause") or {}
    name = str(cause.get("name", ""))
    return f"{name}:{cause.get('force_target', '')}" if name == "external_force" else name


def priors(entries: list[dict]) -> list[float]:
    """The same prior for every mechanism (the nominal is one), split among its hypotheses."""
    mechanisms = [_mechanism(entry) for entry in entries]
    counts = {m: mechanisms.count(m) for m in set(mechanisms)}
    return [1.0 / len(counts) / counts[m] for m in mechanisms]


def _numeric_instances(instances: Any, prefix: str = "") -> dict[str, float]:
    """Flatten the numbers a repetition drew (generated_instances) into named values."""
    out = {}
    if isinstance(instances, dict):
        for key, value in instances.items():
            out.update(_numeric_instances(value, f"{prefix}{key}."))
    elif isinstance(instances, (list, tuple)):
        flat = list(np.ravel(np.asarray(instances, dtype=object)))
        if len(flat) == 1:
            out.update(_numeric_instances(flat[0], prefix))
        else:
            for i, value in enumerate(flat):
                out.update(_numeric_instances(value, f"{prefix}{i}."))
    elif isinstance(instances, (int, float)) and not isinstance(instances, bool) and math.isfinite(float(instances)):
        out[prefix.rstrip(".")] = float(instances)
    return out


def _parameter_estimate(repetitions: list[dict], weights: np.ndarray) -> dict[str, dict[str, float]]:
    """Weighted mean and standard deviation of each number the repetitions drew."""
    if weights.sum() <= 0.0:
        return {}
    values = [_numeric_instances(rep.get(GENERATED_INSTANCES) or {}) for rep in repetitions]
    names = sorted({name for v in values for name in v})
    out = {}
    for name in names:
        rows = [(v[name], w) for v, w in zip(values, weights) if name in v]
        if not rows:
            continue
        x, w = np.array([r[0] for r in rows]), np.array([r[1] for r in rows])
        if w.sum() <= 0.0:
            continue
        mean = float(np.sum(w * x) / w.sum())
        out[name] = {"mean": round(mean, 4), "sd": round(float(np.sqrt(np.sum(w * (x - mean) ** 2) / w.sum())), 4)}
    return out


def build_verdict(
    case_id: str,
    record: dict,
    entries: list[dict[str, Any]],
    historicals: list[list[dict[str, Any]]],
    scene: dict,
    skipped: Optional[list[dict[str, Any]]] = None,
) -> dict[str, Any]:
    """entries[i] is {'hypothesis_id', 'title', 'cause'[, 'mechanism']} (the nominal first);
    historicals[i] its repetitions as the causes simulator returned them; `record` the recording as
    episode_scene.real_record reads it; `scene` the simulation scene (its anomaly_interval,
    start_time_s and observed_effect_time)."""
    t_obs = scene.get("observed_effect_time")
    interval = scene.get("anomaly_interval")
    start = float(scene.get("start_time_s") or 0.0)
    method = {
        "discrepancy": "min over |tau| <= tau_max of sum_c RMS_W(sim_c - real_c) / sigma_c, c in "
                       "{accelerometer, gyroscope, planar path}",
        "likelihood": "mean_i exp(-d_i^2 / 2 eps^2) q(E | x_i); invalid repetitions count as 0",
        "prior": "equal per mechanism (the nominal is one), split among its hypotheses",
        "decision": f"accept the most probable hypothesis if its posterior >= {P_MIN} (Chow), else abstain",
        "tau_max_s": TAU_MAX_S, "eps": EPS, "path_scale_m": PATH_SCALE_M, "effect_delta_s": EFFECT_DELTA_S,
        "effect_weights": dict(EFFECT_WEIGHTS), "p_min": P_MIN, "adequacy_max": ADEQUACY_MAX,
        "explanation_ratio_max": EXPLANATION_RATIO_MAX,
        "importance_weights": "each repetition's log(prior / proposal mixture), when the search drew it in rounds",
        "calibrated": CALIBRATED,
    }
    base = {"schema_version": SCHEMA_VERSION, "case_id": case_id, "observed_effect_time": t_obs,
            "anomaly_interval": {"start_s": interval[0] if interval else None, "end_s": interval[1] if interval else None,
                                 "seed_s": start},
            "method": method, "skipped": skipped or []}
    if not interval or t_obs is None:
        return {**base, "hypotheses": [], "accepted_hypothesis_id": None, "abstention_reason": ABSTAIN_NO_DATA}
    channels = _channels(record, (interval[0], interval[1]), t_obs)
    if not channels:
        return {**base, "hypotheses": [], "accepted_hypothesis_id": None, "abstention_reason": ABSTAIN_NO_DATA}

    # Validity: before the anomaly interval nothing happened, so the nominal must follow the record.
    adequacy = None
    nominal_index = next((i for i, e in enumerate(entries) if e["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID), None)
    if nominal_index is not None and historicals[nominal_index] and interval[0] - start > TAU_MAX_S:
        lead = _channels(record, (start + TAU_MAX_S, interval[0]), t_obs)
        lead_d = discrepancy(lead, historicals[nominal_index][0].get(HISTORY, {})) if lead else None
        adequacy = {"lead_in_distance": lead_d, "max": ADEQUACY_MAX,
                    "faithful": ADEQUACY_MAX is None or (lead_d is not None and lead_d <= ADEQUACY_MAX)}

    prior = priors(entries)
    evaluations, log_terms, best_distances, closest = [], [], [], []
    for entry, repetitions, pi in zip(entries, historicals, prior):
        distances, weights_q, cases, problems = [], [], [], []
        for repetition in repetitions:
            problem = physical_problem(repetition)
            problems.append(problem)
            distances.append(float("inf") if problem else repetition_distance(channels, repetition))
            q, case = effect_weight(repetition, t_obs)
            weights_q.append(q)
            cases.append(case)
        d = np.asarray(distances, dtype=float)
        log_w = np.array([0.0 if LOG_WEIGHT not in r else (-math.inf if r[LOG_WEIGHT] is None else float(r[LOG_WEIGHT]))
                          for r in repetitions])
        terms = np.where(np.isfinite(d) & np.isfinite(log_w),
                         -(d ** 2) / (2 * EPS ** 2) + np.log(np.maximum(weights_q, _EPS)) + log_w, -np.inf)
        log_terms.append(terms)
        counted = np.flatnonzero(np.isfinite(terms))
        best_distances.append(float(d[counted].min()) if counted.size else math.inf)
        closest.append(int(counted[np.argmin(d[counted])]) if counted.size else None)
        timing = [abs(effect_time(r) - t_obs) for r, c in zip(repetitions, cases) if c != "none" and effect_time(r) is not None]
        evaluations.append({
            "hypothesis_id": entry["hypothesis_id"],
            "title": entry.get("title", ""),
            "cause": entry["cause"],
            "mechanism": _mechanism(entry),
            "prior": round(pi, 6),
            "repetitions": len(repetitions),
            "invalid_repetitions": sum(problem is not None for problem in problems),
            "stopped_early": sum(bool(r.get(STOPPED_EARLY)) for r in repetitions),
            "invalid_reasons": sorted({problem for problem in problems if problem}),
            "effect_rate": (sum(c != "none" for c in cases) / len(cases)) if cases else 0.0,
            "mistimed_effects": sum(c == "off_time" for c in cases),
            "median_timing_error_s": float(np.median(timing)) if timing else None,
            "distance": {"median": float(np.median(d[np.isfinite(d)])) if np.isfinite(d).any() else None,
                         "min": float(np.min(d)) if np.isfinite(d).any() else None},
        })

    log_evidence = np.array([_log_mean_exp(terms) for terms in log_terms])
    log_joint = log_evidence + np.log(prior)
    posterior = _normalise(log_joint)

    # Monte Carlo error: resample the repetitions of every hypothesis.
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    draws = []
    for _ in range(BOOTSTRAP):
        resampled = np.array([_log_mean_exp(t[rng.integers(0, t.size, t.size)]) if t.size else float("-inf")
                              for t in log_terms])
        draws.append(_normalise(resampled + np.log(prior)))
    draws = np.array(draws)

    nominal_log = log_evidence[nominal_index] if nominal_index is not None else None
    for i, (evaluation, repetitions, terms) in enumerate(zip(evaluations, historicals, log_terms)):
        weights = np.exp(terms - terms.max()) if terms.size and np.isfinite(terms).any() else np.zeros(terms.size)
        best = int(np.argmax(terms)) if terms.size and np.isfinite(terms).any() else None
        evaluation.update({
            "log_evidence": None if not np.isfinite(log_evidence[i]) else round(float(log_evidence[i]), 4),
            "posterior": round(float(posterior[i]), 6),
            "posterior_bootstrap": {"p10": round(float(np.percentile(draws[:, i], 10)), 6),
                                    "p90": round(float(np.percentile(draws[:, i], 90)), 6),
                                    "sd": round(float(np.std(draws[:, i])), 6)},
            "bayes_factor_vs_nominal": (None if nominal_log is None or not np.isfinite(nominal_log) or not np.isfinite(log_evidence[i])
                                        else float(math.exp(min(700.0, log_evidence[i] - nominal_log)))),
            "parameter_estimate": _parameter_estimate(repetitions, weights),
            "best_repetition_index": best,
            # The repetition closest to the record, the one ratio_to_nominal uses (it may differ from the best when
            # the search's importance weights or the effect weigh in); its history is in the simulations file.
            "closest_repetition_index": closest[i],
            "best_generated_instances": repetitions[best].get(GENERATED_INSTANCES, {}) if best is not None else {},
            "best_bottle_final_position": repetitions[best].get(BOTTLE_POSITION) if best is not None else None,
            "passes_rule": bool(posterior[i] >= P_MIN),
        })

    order = np.argsort(-posterior)
    top = int(order[0])
    # How much of the discrepancy with the nominal each hypothesis' best repetition leaves.
    nominal_best = best_distances[nominal_index] if nominal_index is not None else math.inf
    for evaluation, best in zip(evaluations, best_distances):
        evaluation["ratio_to_nominal"] = (round(best / nominal_best, 4) if math.isfinite(best) and math.isfinite(nominal_best)
                                          and nominal_best > 0 else None)
    explained_ratio = evaluations[top]["ratio_to_nominal"]
    if adequacy is not None and not adequacy["faithful"]:
        reason = ABSTAIN_NOT_FAITHFUL
    elif posterior[top] < P_MIN:
        reason = ABSTAIN_BELOW_THRESHOLD
    elif entries[top]["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID:
        reason = ABSTAIN_NOMINAL
    elif explained_ratio is not None and explained_ratio > EXPLANATION_RATIO_MAX:
        reason = ABSTAIN_INADEQUATE
    else:
        reason = None
    accepted_id = entries[top]["hypothesis_id"] if reason is None else None
    for evaluation in evaluations:
        evaluation["accepted"] = evaluation["hypothesis_id"] == accepted_id
    return {**base, "adequacy": adequacy, "hypotheses": evaluations,
            "most_probable_hypothesis_id": entries[top]["hypothesis_id"],
            "explained_ratio": explained_ratio,
            "accepted_hypothesis_id": accepted_id, "abstention_reason": reason,
            # With EPS so sharp the posterior rests on the few repetitions closest to the record: an
            # acceptance is robust to the Monte Carlo error when it holds in 90 % of the bootstrap draws.
            "acceptance_robust": bool(accepted_id is not None and np.percentile(draws[:, top], 10) >= P_MIN),
            "credible_set": credible_set(entries, posterior)}


def credible_set(entries: list[dict[str, Any]], posterior: np.ndarray, mass: float = P_MIN) -> dict[str, Any]:
    """The fewest mechanisms that together hold `mass` of the posterior, most probable first: what the
    record cannot tell apart. A push on the bottle, for one, leaves no trace on the robot's sensors and
    shares the set with the nominal (docs_output/resultados/35_banco_de_verificacion)."""
    by_mechanism: dict[str, float] = {}
    for entry, p in zip(entries, posterior):
        by_mechanism[_mechanism(entry)] = by_mechanism.get(_mechanism(entry), 0.0) + float(p)
    members, held = [], 0.0
    for name, p in sorted(by_mechanism.items(), key=lambda kv: -kv[1]):
        members.append({"mechanism": name, "posterior": round(p, 6)})
        held += p
        if held >= mass:
            break
    return {"mass": mass, "mechanisms": members}


def _normalise(log_values: np.ndarray) -> np.ndarray:
    if not np.isfinite(log_values).any():
        return np.full(log_values.shape, 1.0 / log_values.size)
    top = float(np.max(log_values[np.isfinite(log_values)]))
    weights = np.where(np.isfinite(log_values), np.exp(log_values - top), 0.0)
    return weights / weights.sum()


def write_verdict(verdict: dict[str, Any], output_dir: str) -> str:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_path = out_dir / f"verdict_{_safe_filename(verdict.get('case_id', 'case'))}.json"
    output_path.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    return str(output_path)


# --------------------------------------------------------------------------- #
# What the simulation looked like, next to the verdict
# --------------------------------------------------------------------------- #
SIMULATIONS_SCHEMA_VERSION = "1.0"


def simulated_runs(verdict: dict[str, Any], historicals: list[list[dict[str, Any]]], scene: dict,
                   record: Optional[dict]) -> dict[str, Any]:
    """What the figures of simulated against recorded, and a later replay of a simulation, need from a case
    (docs_output/diseno/evaluacion.md, 4.3): the scene, the record the verdict compared against, and for the
    nominal and every hypothesis the repetition closest to the record (closest_repetition_index, the one
    ratio_to_nominal uses) as the simulator returned it: its history (IMU, planar path), what perception saw, the
    numbers it drew (generated_instances, and the unit draws of a generated cause) and where the bottle ended. A
    replay runs the scene with that bump sample or those draws (causes_simulator.py --draws). The fast path keeps
    the history of each chunk's closest repetition only, which includes every hypothesis' closest one."""
    runs = []
    for evaluation, repetitions in zip(verdict.get("hypotheses", []), historicals):
        index = evaluation.get("closest_repetition_index")
        repetition = repetitions[index] if index is not None else None
        # Without the simulator's own distance (the slow path), every finite distance counts: its minimum.
        distance = (None if repetition is None else repetition[DISTANCE] if DISTANCE in repetition
                    else (evaluation.get("distance") or {}).get("min"))
        runs.append({"hypothesis_id": evaluation["hypothesis_id"], "mechanism": evaluation.get("mechanism"),
                     "cause": evaluation.get("cause"), "repetition_index": index, "distance": distance,
                     "has_history": bool(repetition) and HISTORY in repetition, "repetition": repetition})
    return {"schema_version": SIMULATIONS_SCHEMA_VERSION, "case_id": verdict.get("case_id"), "scene": scene,
            "record": record, "runs": runs}


def _plain(value: Any) -> Any:
    """A numpy number or array as JSON writes it: the episodic-memory API gives the record in float32."""
    if isinstance(value, (np.generic, np.ndarray)):
        return value.tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def write_simulations(runs: dict[str, Any], output_dir: str) -> str:
    """simulated_runs next to the verdict, as simulations_<case_id>.json (compact: it holds the record)."""
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_path = out_dir / f"simulations_{_safe_filename(runs.get('case_id') or 'case')}.json"
    output_path.write_text(json.dumps(runs, separators=(",", ":"), default=_plain), encoding="utf-8")
    return str(output_path)


# --------------------------------------------------------------------------- #
# Plots
# --------------------------------------------------------------------------- #
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
            if sim_values is not None and sim_values.size:
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
    """One real-vs-simulated figure per hypothesis, using the repetition the verdict reported
    (entries may be the verdict evaluations)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for entry, repetitions in zip(entries, historicals):
        if not repetitions:
            continue
        repetition = repetitions[entry.get("best_repetition_index") or 0]
        if HISTORY not in repetition:
            continue
        title = f"Real vs simulated IMU — {entry['hypothesis_id']} ({entry['cause']['name']})"
        figure = _imu_figure(plt, real_imu, repetition[HISTORY], title)
        figure.savefig(out_dir / f"imu_comparison_{_safe_filename(entry['hypothesis_id'])}.png", dpi=110)
        plt.close(figure)
