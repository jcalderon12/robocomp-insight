"""Contrast of the mechanisms with the recorded episode (contract 2, sections 3.3-3.6; task A3).

Before anything is simulated, each mechanism is checked against what the episode recorded:

  * its preconditions (`insight:requires`): if one is false, the hypothesis is incoherent;
  * its contrast rules (`insight:hasContrastRule`): they discard, inform about a recorded trace,
    or say that the memory already explains the effect (a commanded brake). They never confirm a cause.

What a mechanism is checked with comes from the INSIGHT TBox (agents/semantic/data/insight_tbox.ttl):
which preconditions and rules it has, their effect and status, and the observable properties each
rule reads. This module implements each check by its id, with the thresholds of contract 2, and
refuses a TBox whose checks it does not implement or whose properties differ from the ones the
implementation reads.

The rules read the episode of contract 1 (episode_builder.py) and nothing else:

  * the evidence measured over Interval_fall, compared with the same property in free motion
    (Interval_baseline) where the rule says so: relative to the episode itself, because of the
    Webots clock. The rules that compare the recorded turn with the ordered one scale the order by
    the episode's own clock (what the robot travelled over what it was ordered, in free motion);
  * never the evidence marked as the system's reaction (the stop after the fall), although the
    braking shares its property name with the longitudinal peak before the fall;
  * the preconditions read the anchored segment or interval, the phases and the bottle support.

A missing value never discards: the check is `undetermined`. A rule whose evidence was measured
over another interval than the one the hypothesis anchors is `not_applicable`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable, Optional

from rdflib import RDFS, Graph, Namespace

from src.explanation_context import mechanism_nodes

INSIGHT = Namespace("http://insight.local/ontology#")
DEFAULT_TBOX = Path(__file__).resolve().parents[1] / "data" / "insight_tbox.ttl"


def tbox_graph(tbox: str | Path | Graph) -> Graph:
    """The TBox as a graph: the one read from the semantic memory, or parsed from a file (offline).
    What is derived from it is cached by graph identity, so each read is a graph of its own."""
    return tbox if isinstance(tbox, Graph) else Graph().parse(str(tbox), format="turtle")

GRAVITY_M_S2 = 9.81
FALL_INTERVAL = "Interval_fall"
FINAL_SEGMENT = "Segment_final"

#: Thresholds of contract 2, section 3.6. Those of rules with status `to_calibrate` are provisional:
#: A3 sets them so that the obstacle is never discarded in the five recordings with a bump, and
#: the true mechanism is discarded in fewer than 2 % of the bench episodes.
THRESHOLDS: dict[str, dict[str, float]] = {
    "moving_on_segment": {"min_speed_mps": 0.05},
    "moving_in_interval": {"min_speed_mps": 0.05},
    "bottle_supported": {},
    "obstacle_jolt": {"times_baseline": 3.0},
    # mu_max is the upper end of `slippery` (table 3.4); 1.1 is the tolerance on the IMU.
    "traction_vs_friction": {"mu_max": 0.3, "tolerance": 1.1},
    # Blocking a drive wheel of the real robot turns it 6-11 deg.
    "turn_vs_command": {"max_heading_diff_deg": 3.0, "max_yaw_rate_rad_s": 0.3},
    "setpoint_step": {"min_step_mps": 0.2},
    "speed_ratio_and_jolt": {"max_ratio_deviation": 0.15, "times_baseline": 2.0},
    "robot_jolt": {"times_baseline": 2.0, "max_ratio_deviation": 0.15, "max_heading_diff_deg": 3.0},
    # 1 m of reach plus 0.5 m of margin.
    "person_within_reach": {"max_distance_m": 1.5},
    "bottle_reacquired": {},
}

#: Verdicts of one check, by its effect in the TBox.
HOLDS, FAILS = "holds", "fails"                      # precondition (effect `incoherent`)
DISCARDS, SURVIVES = "discards", "survives"          # effect `discard`
EXPLAINS = "explains"                                # effect `explain_or_discard` (or DISCARDS)
TRACE_SEEN, NO_TRACE = "trace_seen", "no_trace"      # effect `inform`
UNDETERMINED, NOT_APPLICABLE = "undetermined", "not_applicable"

#: Outcome of the contrast for one hypothesis (the part of contract 3's status it decides).
INCOHERENT, DISCARDED, EXPLAINED_BY_MEMORY, SURVIVES_CONTRAST = (
    "incoherent", "discarded", "explained_by_memory", "survives")


# --------------------------------------------------------------------------- #
# The mechanisms and their checks, read from the TBox
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Check:
    """A precondition or a contrast rule, as the TBox declares it."""

    rule_id: str
    kind: str                       #: "precondition" or "contrast_rule"
    effect: str                     #: incoherent, discard, explain_or_discard or inform
    status: str                     #: fixed or to_calibrate
    properties: tuple[str, ...]     #: observable properties it reads (contrast rules)
    comment: str


@dataclass(frozen=True)
class Mechanism:
    mechanism_id: str
    iri: str
    label: str
    anchors: tuple[str, ...]        #: "segment" and/or "interval"
    preconditions: tuple[Check, ...]
    rules: tuple[Check, ...]
    realized_by: tuple[str, ...]    #: the interventions that simulate it (local names, sorted); empty if none
    realization_experimental: bool  #: all of them only in experiments/ (override_setpoint)
    not_simulable_because: Optional[str]
    cost_in_simulations: int
    family: str


ANCHOR_KINDS = {INSIGHT.SegmentAnchor: "segment", INSIGHT.IntervalAnchor: "interval"}


def _local(iri) -> str:
    return str(iri).rsplit("#", 1)[-1]


def _check(graph: Graph, node, kind: str) -> Check:
    def text(prop) -> str:
        value = graph.value(node, prop)
        if value is None:
            raise ValueError(f"{node}: {kind} without {_local(prop)} in the TBox")
        return str(value)

    return Check(
        rule_id=text(INSIGHT.ruleId), kind=kind, effect=text(INSIGHT.ruleEffect), status=text(INSIGHT.ruleStatus),
        properties=tuple(sorted(_local(p) for p in graph.objects(node, INSIGHT.checksProperty))),
        comment=str(graph.value(node, RDFS.comment) or ""))


@lru_cache(maxsize=4)
def load_mechanisms(tbox_path: str | Path | Graph = DEFAULT_TBOX,
                    profiles: Optional[tuple[str, ...]] = None) -> dict[str, Mechanism]:
    """The mechanisms of the TBox and their checks, by mechanism id.

    Raises if the TBox has a check this module does not implement, or if a rule reads other
    properties than the ones its implementation reads.
    """
    graph = tbox_graph(tbox_path)
    mechanisms: dict[str, Mechanism] = {}
    for node in mechanism_nodes(graph, profiles):
        # A mechanism may be simulated in more than one way (the obstacle: a dome of unknown size for a
        # bump, a fixed asset for a cable); the grounding picks one per hypothesis.
        interventions = sorted(graph.objects(node, INSIGHT.realizedBy), key=str)
        experimental = [graph.value(i, INSIGHT.isExperimental) for i in interventions]
        reason = graph.value(node, INSIGHT.notSimulableBecause)
        mechanism = Mechanism(
            mechanism_id=str(graph.value(node, INSIGHT.mechanismId)),
            iri=str(node),
            label=str(graph.value(node, RDFS.label) or ""),
            anchors=tuple(sorted(ANCHOR_KINDS[a] for a in graph.objects(node, INSIGHT.requiresAnchor))),
            preconditions=tuple(sorted((_check(graph, p, "precondition") for p in graph.objects(node, INSIGHT.requires)),
                                       key=lambda c: c.rule_id)),
            rules=tuple(sorted((_check(graph, r, "contrast_rule") for r in graph.objects(node, INSIGHT.hasContrastRule)),
                               key=lambda c: c.rule_id)),
            realized_by=tuple(_local(i) for i in interventions),
            realization_experimental=bool(interventions) and all(e is not None and e.toPython() for e in experimental),
            not_simulable_because=(str(reason) if reason is not None else
                                   "No simulation realization is declared for this mechanism." if not interventions else None),
            cost_in_simulations=int(graph.value(node, INSIGHT.costInSimulations) or 0),
            family=str(graph.value(node, INSIGHT.family) or ""),
        )
        for check in mechanism.preconditions + mechanism.rules:
            implementation = CHECKS.get(check.rule_id)
            if implementation is None:
                raise ValueError(f"the TBox declares check '{check.rule_id}', which episode_contrast does not implement")
            if check.kind == "contrast_rule" and check.properties != implementation.reads:
                raise ValueError(f"check '{check.rule_id}': the TBox says it reads {check.properties}, "
                                 f"the implementation reads {implementation.reads}")
        if mechanism.mechanism_id in mechanisms:
            raise ValueError(f"duplicate active mechanism id '{mechanism.mechanism_id}' in the TBox")
        mechanisms[mechanism.mechanism_id] = mechanism
    if not mechanisms and profiles is None:
        raise ValueError(f"{tbox_path}: no mechanism in the TBox")
    return mechanisms


# --------------------------------------------------------------------------- #
# What the checks read from the episode
# --------------------------------------------------------------------------- #
class EpisodeView:
    """The parts of a contract-1 episode the checks read. The evidence of the system's reaction is
    left out here, once, so that no check can read it."""

    def __init__(self, episode: dict[str, Any]):
        self.episode = episode
        self.intervals = {i["id"]: (i["start_s"], i["end_s"]) for i in episode.get("intervals", [])}
        self.segments = {s["id"]: s for s in episode.get("segments", [])}
        self.phases = [p for p in episode.get("phases", []) if not p.get("is_system_reaction")]
        self.support = episode.get("support") or {}
        self.t_obs = (episode.get("time") or {}).get("t_obs_s", (episode.get("change") or {}).get("time_s"))
        self.evidence: dict[str, dict[str, Any]] = {}
        for entry in episode.get("evidence", []):
            if entry.get("is_system_reaction"):
                continue
            if entry["property"] in self.evidence:
                raise ValueError(f"two pieces of evidence for {entry['property']} outside the reaction")
            self.evidence[entry["property"]] = entry

    def value(self, prop: str) -> Any:
        entry = self.evidence.get(prop)
        return None if entry is None else entry.get("value")

    def baseline(self, prop: str) -> Optional[float]:
        entry = self.evidence.get(prop)
        return None if entry is None else entry.get("baseline")

    def mean_speed_in(self, interval_id: str) -> Optional[float]:
        """Time-weighted mean of the speeds of the motion phases over the part of the interval
        they cover (exact when the interval is made of whole phases)."""
        start, end = self.intervals[interval_id]
        covered = travelled = 0.0
        for phase in self.phases:
            p_start, p_end = self.intervals[phase["interval"]]
            overlap = min(end, p_end) - max(start, p_start)
            if overlap > 0:
                covered += overlap
                travelled += overlap * phase["mean_speed_mps"]
        return travelled / covered if covered > 0 else None


@dataclass
class Condition:
    """One comparison of a check: `quantity value op limit`."""

    quantity: str
    value: Any
    op: str
    limit: Any
    limit_expr: Optional[str] = None
    holds: Optional[bool] = None
    value_expr: Optional[str] = None

    def __post_init__(self):
        if self.value is None or self.limit is None:
            self.holds = None
        elif self.holds is None:
            self.holds = {"<": self.value < self.limit, "<=": self.value <= self.limit,
                          ">": self.value > self.limit, ">=": self.value >= self.limit,
                          "==": self.value == self.limit}[self.op]

    def text(self) -> str:
        mark = {True: "", False: " (no)", None: " (unknown)"}[self.holds]
        if self.op == "==":
            return f"{self.quantity}: {_fmt(self.value)}{mark}"
        limit = f"{self.limit_expr} = {_fmt(self.limit)}" if self.limit_expr else _fmt(self.limit)
        value = f"{self.value_expr} = {_fmt(self.value)}" if self.value_expr else _fmt(self.value)
        return f"{self.quantity} {value} {self.op} {limit}{mark}"

    def as_dict(self) -> dict[str, Any]:
        return {"quantity": self.quantity, "value": self.value, "value_expr": self.value_expr, "op": self.op,
                "limit": self.limit, "limit_expr": self.limit_expr, "holds": self.holds}


def _fmt(value: Any) -> str:
    if value is None:
        return "?"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".") if abs(value) < 1000 else f"{value:.0f}"
    return str(value)


def _r(value: Optional[float]) -> Optional[float]:
    return None if value is None else round(float(value), 4)


def _all(conditions: list[Condition]) -> Optional[bool]:
    if any(c.holds is False for c in conditions):
        return False
    return None if any(c.holds is None for c in conditions) else True


def _any(conditions: list[Condition]) -> Optional[bool]:
    if any(c.holds is True for c in conditions):
        return True
    return None if any(c.holds is None for c in conditions) else False


def _times_baseline(view: EpisodeView, prop: str, quantity: str, op: str, factor: float) -> Condition:
    base = view.baseline(prop)
    return Condition(quantity, view.value(prop), op, None if base is None else _r(factor * base),
                     f"{factor:g} x baseline {_fmt(base)}")


def _own_clock(view: EpisodeView) -> Optional[float]:
    """What the robot travelled over what it was ordered, in free motion (own_ratio_baseline of
    speed_ratio). Under the Webots clock it is about 0.5: the robot turns, like it advances, only
    that fraction of what it is ordered (in the free motion of the 24/09 falls that turn cleanly,
    12:29, 12:40 and 13:33, turned over ordered gives 0.43-0.61, and this ratio 0.46-0.59). On the
    real robot it is about 1."""
    entry = view.evidence.get("speed_ratio")
    return None if entry is None else entry.get("own_ratio_baseline")


def _heading_mismatch(view: EpisodeView, quantity: str, limit: float) -> Condition:
    """|recorded heading change - r x commanded one| over Interval_fall < limit (deg), with r the
    episode's own clock. The commanded change is already -integral(robot_ref_rot_speed): the
    episode turns the setpoint sign around. Without r (no free motion) it is unknown."""
    entry = view.evidence.get("yaw_change") or {}
    turned, ordered, clock = entry.get("value"), entry.get("commanded"), _own_clock(view)
    if turned is None or ordered is None or clock is None:
        return Condition(quantity, None, "<", limit)
    return Condition(quantity, _r(abs(turned - clock * ordered)), "<", limit,
                     value_expr=f"|{_fmt(turned)} - {_fmt(clock)} x {_fmt(ordered) if ordered >= 0 else f'({_fmt(ordered)})'}|")


def _speed_ratio_deviation(view: EpisodeView) -> Optional[float]:
    ratio = view.value("speed_ratio")
    return None if ratio is None else _r(abs(ratio - 1.0))


# --------------------------------------------------------------------------- #
# The checks, by rule id
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Implementation:
    reads: tuple[str, ...]          #: observable properties (sorted), as the TBox must declare them
    #: (view, anchors, thresholds) -> (verdict if every condition holds / any holds, conditions)
    evaluate: Callable[[EpisodeView, dict[str, Optional[str]], dict[str, float]], tuple[str, list[Condition]]]
    combine: str = "all"            #: how the conditions combine: "all" or "any"


def _moving_on_segment(view, anchors, th):
    segment = view.segments.get(anchors.get("segment") or "")
    speed = segment.get("mean_speed_mps") if segment else None
    return HOLDS, [Condition(f"mean speed on {anchors.get('segment')}", speed, ">", th["min_speed_mps"])]


def _moving_in_interval(view, anchors, th):
    interval = anchors.get("interval")
    speed = view.mean_speed_in(interval) if interval in view.intervals else None
    return HOLDS, [Condition(f"mean speed in {interval}", _r(speed), ">", th["min_speed_mps"])]


def _bottle_supported(view, anchors, th):
    interval = anchors.get("interval")
    start = view.intervals[interval][0] if interval in view.intervals else None
    return HOLDS, [Condition(f"start of {interval}", start, ">=", view.support.get("start_s"), "support start"),
                   Condition(f"start of {interval}", start, "<", view.support.get("end_s"), "support end")]


def _obstacle_jolt(view, anchors, th):
    k = th["times_baseline"]
    return TRACE_SEEN, [_times_baseline(view, "pitch_rate_peak", "pitch_rate_peak", ">", k),
                        _times_baseline(view, "vertical_accel_peak", "vertical_accel_peak", ">", k)]


def _traction_vs_friction(view, anchors, th):
    limit = th["tolerance"] * th["mu_max"] * GRAVITY_M_S2
    return DISCARDS, [Condition("max_sustained_horizontal_accel", view.value("max_sustained_horizontal_accel"),
                                ">", _r(limit), f"{th['tolerance']:g} x {th['mu_max']:g} x g")]


def _turn_vs_command(view, anchors, th):
    return DISCARDS, [_heading_mismatch(view, "|yaw_change - r x commanded|", th["max_heading_diff_deg"]),
                      Condition("yaw_rate_peak", view.value("yaw_rate_peak"), "<", th["max_yaw_rate_rad_s"])]


def _setpoint_step(view, anchors, th):
    return EXPLAINS, [Condition("setpoint_step", view.value("setpoint_step"), ">=", th["min_step_mps"])]


def _speed_ratio_and_jolt(view, anchors, th):
    return DISCARDS, [Condition("|speed_ratio - 1|", _speed_ratio_deviation(view), "<=", th["max_ratio_deviation"]),
                      _times_baseline(view, "longitudinal_accel_peak", "longitudinal_accel_peak", "<=",
                                      th["times_baseline"])]


def _robot_jolt(view, anchors, th):
    return DISCARDS, [_times_baseline(view, "lateral_accel_peak", "lateral_accel_peak", "<=", th["times_baseline"]),
                      Condition("|speed_ratio - 1|", _speed_ratio_deviation(view), "<=", th["max_ratio_deviation"]),
                      _heading_mismatch(view, "|yaw_change - r x commanded|", th["max_heading_diff_deg"])]


def _person_within_reach(view, anchors, th):
    return DISCARDS, [Condition("person_distance_min", view.value("person_distance_min"), ">", th["max_distance_m"])]


def _bottle_reacquired(view, anchors, th):
    # This records restoration of the carrying relation within the available episode, not
    # continuous physical support. Neither restoration nor its absence decides the cause.
    return TRACE_SEEN, [Condition("bottle_reacquired", view.value("bottle_reacquired"), "==", True)]


CHECKS: dict[str, Implementation] = {
    "moving_on_segment": Implementation((), _moving_on_segment),
    "moving_in_interval": Implementation((), _moving_in_interval),
    "bottle_supported": Implementation((), _bottle_supported),
    "obstacle_jolt": Implementation(("pitch_rate_peak", "vertical_accel_peak"), _obstacle_jolt, combine="any"),
    "traction_vs_friction": Implementation(("max_sustained_horizontal_accel",), _traction_vs_friction),
    "turn_vs_command": Implementation(("speed_ratio", "yaw_change", "yaw_rate_peak"), _turn_vs_command),
    "setpoint_step": Implementation(("setpoint_step",), _setpoint_step),
    "speed_ratio_and_jolt": Implementation(("longitudinal_accel_peak", "speed_ratio"), _speed_ratio_and_jolt),
    "robot_jolt": Implementation(("lateral_accel_peak", "speed_ratio", "yaw_change"), _robot_jolt),
    "person_within_reach": Implementation(("person_distance_min",), _person_within_reach),
    "bottle_reacquired": Implementation(("bottle_reacquired",), _bottle_reacquired),
}

#: Verdict when the conditions do not give the one the check is written for.
OTHERWISE = {HOLDS: FAILS, DISCARDS: SURVIVES, EXPLAINS: DISCARDS, TRACE_SEEN: NO_TRACE}


# --------------------------------------------------------------------------- #
# Contrast of one hypothesis
# --------------------------------------------------------------------------- #
def default_anchors(mechanism: Mechanism) -> dict[str, Optional[str]]:
    """The anchors of the fall: Segment_final and Interval_fall (what the anchored enumeration uses)."""
    return {"segment": FINAL_SEGMENT if "segment" in mechanism.anchors else None,
            "interval": FALL_INTERVAL if "interval" in mechanism.anchors else None}


def _applies(view: EpisodeView, mechanism: Mechanism, check: Check, anchors: dict[str, Optional[str]]) -> Optional[str]:
    """Why a contrast rule does not apply to these anchors (None if it applies).

    The evidence is measured over fixed intervals (Interval_fall for the peaks). A rule says
    nothing about a hypothesis anchored somewhere else: an interval other than the one the
    evidence covers, or a segment travelled outside it.
    """
    for prop in check.properties:
        entry = view.evidence.get(prop)
        measured_in = entry.get("interval") if entry else None
        if measured_in is None or measured_in not in view.intervals:
            continue
        if anchors.get("interval") and anchors["interval"] != measured_in:
            return f"{prop} is measured over {measured_in}, the hypothesis is anchored on {anchors['interval']}"
        segment = view.segments.get(anchors.get("segment") or "")
        if segment is not None:
            s_start, s_end = view.intervals[segment["interval"]]
            m_start, m_end = view.intervals[measured_in]
            if min(s_end, m_end) <= max(s_start, m_start):
                return f"{prop} is measured over {measured_in}, which {segment['id']} does not overlap"
    return None


def evaluate_check(view: EpisodeView, mechanism: Mechanism, check: Check,
                   anchors: dict[str, Optional[str]]) -> dict[str, Any]:
    implementation = CHECKS[check.rule_id]
    result = {"check": check.rule_id, "kind": check.kind, "effect": check.effect, "status": check.status,
              "reads": list(check.properties), "conditions": [], "verdict": None, "reason": ""}
    if check.kind == "contrast_rule":
        why_not = _applies(view, mechanism, check, anchors)
        if why_not:
            result.update(verdict=NOT_APPLICABLE, reason=why_not)
            return result
    target, conditions = implementation.evaluate(view, anchors, THRESHOLDS[check.rule_id])
    met = (_any if implementation.combine == "any" else _all)(conditions)
    verdict = UNDETERMINED if met is None else (target if met else OTHERWISE[target])
    joiner = " or " if implementation.combine == "any" else " and "
    result.update(conditions=[c.as_dict() for c in conditions], verdict=verdict,
                  reason=joiner.join(c.text() for c in conditions))
    if check.effect == "inform" and check.comment:
        result["reason"] += f". {check.comment}"
    return result


def contrast_hypothesis(episode: dict[str, Any] | EpisodeView, mechanism_id: str,
                        anchors: Optional[dict[str, Optional[str]]] = None,
                        mechanisms: Optional[dict[str, Mechanism]] = None) -> dict[str, Any]:
    """Preconditions and contrast rules of one mechanism with these anchors, over the episode.

    Without anchors, the anchors of the fall (Segment_final and Interval_fall, as admitted).
    The outcome is `incoherent` if a precondition fails, `discarded` if a rule discards,
    `explained_by_memory` if the memory explains the fall, and `survives` otherwise.
    """
    view = episode if isinstance(episode, EpisodeView) else EpisodeView(episode)
    mechanism = (mechanisms or load_mechanisms())[mechanism_id]
    anchors = dict(anchors) if anchors is not None else default_anchors(mechanism)
    checks = [evaluate_check(view, mechanism, c, anchors) for c in mechanism.preconditions + mechanism.rules]

    decided_by = None
    outcome = SURVIVES_CONTRAST
    for verdict, result_outcome in ((FAILS, INCOHERENT), (DISCARDS, DISCARDED), (EXPLAINS, EXPLAINED_BY_MEMORY)):
        hit = next((c for c in checks if c["verdict"] == verdict), None)
        if hit:
            outcome, decided_by = result_outcome, hit["check"]
            break
    return {"mechanism": mechanism_id, "mechanism_iri": mechanism.iri, "anchors": anchors,
            "checks": checks, "outcome": outcome, "decided_by": decided_by,
            "status_after_contrast": status_after_contrast(mechanism, outcome)}


def status_after_contrast(mechanism: Mechanism, outcome: str) -> str:
    """The status of contract 3 the contrast already fixes; `pending` means it still depends on the
    grounding and the budget (to_simulate or over_budget, task A4), and for an experimental
    realization (override_setpoint) on whether it enters the simulator at all."""
    if outcome != SURVIVES_CONTRAST:
        return outcome
    if not mechanism.realized_by:
        # Surviving a check is not confirmation; the mechanism has no simulation realization.
        return "checked_not_simulable" if mechanism.rules else "not_simulable"
    return "pending"


def contrast_episode(episode: dict[str, Any], mechanisms: Optional[dict[str, Mechanism]] = None) -> dict[str, Any]:
    """Every mechanism with the anchors of the fall: the contrast of the anchored enumeration."""
    mechanisms = mechanisms or load_mechanisms()
    view = EpisodeView(episode)
    return {
        "schema": "insight.contrast/0.1",
        "episode_id": episode["episode_id"],
        "episode_source": {k: episode["source"].get(k) for k in ("path", "sha256")},
        "thresholds": THRESHOLDS,
        "results": [contrast_hypothesis(view, mechanism_id, mechanisms=mechanisms)
                    for mechanism_id in sorted(mechanisms)],
    }
