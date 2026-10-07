"""From the hypotheses in symbols to the published batch (contract 3; task A4, subpaso 3.1).

The deterministic part of the new generation, with no LLM. It takes the episode (contract 1) and a
list of hypotheses in symbols (layer 3a: a mechanism, its anchors in the episode and its
qualitative parameters, never a number) and returns the batch of layer 3b (schema 2.0), in the four
steps of contract 3, section 4.3:

  1. coherence: the mechanism exists, its anchors exist in the episode and are the ones it admits,
     its qualitative parameters are in their lists, and its preconditions hold;
  2. contrast with the recording (episode_contrast.py): the rules only discard, or say that the
     memory already explains the fall;
  3. grounding in numbers (contract 2, section 3.5): a segment becomes a position range, an
     interval an activation window, a direction a force range. The result is a blueprint v2, so the
     compiler, the simulator and the verdict do not change;
  4. budget: every hypothesis that survives and can be simulated is simulated. With a budget k (the
     evaluation's comparisons), only in the proposed order while its compiled causes fit in k.

A list that breaks the schema (an unknown mechanism, a parameter outside its list or missing, a
number anywhere) is refused as a whole with HypothesisSchemaError, so that the generator retries. A
wrong anchor or a false precondition only makes that hypothesis `incoherent`.

The LLM chooses kinds, never magnitudes (contract 2, version 1.4): whether the obstacle is a bump
or a cable, the direction of a push, the side of a wheel. How big, how strong or how slippery is
not in the evidence without a model fitted to the scenario, so the grounding covers the whole range
the catalog can simulate: a magnitude is sampled by the simulator over its range. A bump is one
dome of unknown size (`spawn_scaled_dome`, contracts 1.5) whose repetitions draw its diameter and
height, and it costs what it simulates; with a catalog that only has fixed bump assets, it is one
cause per asset, all counted together in the budget. The rule depends on the catalog, not on the
scenario.

What each mechanism admits, how it is realized and how its title is written come from the INSIGHT
TBox. The numbers come from contract 2, section 3.4; a TBox whose values differ from the ones
translated here is refused. The anchored enumeration (task B4) is here too, as a layer-3a list.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import re
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Any, Optional

from rdflib import Graph, URIRef

from src.episode_contrast import (
    DEFAULT_TBOX, DISCARDED, EXPLAINED_BY_MEMORY, FALL_INTERVAL, INCOHERENT, INSIGHT, EpisodeView, Mechanism,
    contrast_hypothesis, default_anchors, load_mechanisms,
)
from src.intervention_catalog import InterventionCatalog

DEFAULT_CATALOG = Path(__file__).resolve().parents[3] / "etc" / "intervention_catalog.json"

SCHEMA_VERSION = "2.0"
ARMS = ("llm_full", "llm_json", "enum_anchored", "enum_anchored_contrast", "production")
#: No budget: everything that survives and can be simulated is simulated (Javi, 07/10). A number k is
#: the evaluation's budget (contract 3: k = 6 against the old 3 + 3, and k = 1 and 3).
DEFAULT_BUDGET: Optional[int] = None
NEW_MECHANISM = "new"

#: Contract 2, section 3.3, in its order (the anchored enumeration follows it).
MECHANISM_ORDER = (
    "obstacle_traversed", "bottle_push", "robot_push", "slippery_floor", "wheel_failure",
    "commanded_speed_change", "uncommanded_motion", "bottle_removed_by_person", "perception_failure",
    "suspension_failure",
)

DIRECTIONS = ("any", "forward", "backward", "left", "right")
#: Contract 2, section 3.4 (version 1.4): the qualitative parameters and their values, in its order.
#: Only kinds, which the evidence can tell in words. The TBox must declare exactly these.
PARAMETER_VALUES: dict[str, dict[str, tuple[str, ...]]] = {
    "obstacle_traversed": {"shape": ("bump", "cable")},
    "bottle_push": {"direction": DIRECTIONS},
    "robot_push": {"direction": DIRECTIONS},
    "wheel_failure": {"side": ("left", "right", "unknown")},
    "uncommanded_motion": {"kind": ("brake", "jerk")},
}

#: Section 3.4, the magnitudes, which the LLM does not choose. A bump is the dome of unknown size of
#: this intervention when the catalog has it (contracts 1.5); otherwise every asset of the catalog
#: whose id starts with the prefix, one compiled cause each, in the catalog's order.
SCALED_DOME = "spawn_scaled_dome"
BUMP_ASSET_PREFIX = "bump_"
CABLE_ASSET = "cylinder_bump_10m"
#: Force (N), one range the simulator samples: from a light to a strong push (study 2: the effect
#: appears from about 5 N on the bottle and from about 500 N on the robot). It depends on the bodies
#: pushed, not on the scenario.
PUSH_FORCE_N = {"bottle_push": (3.0, 30.0), "robot_push": (150.0, 500.0)}
PUSH_TARGET = {"bottle_push": "bottle", "robot_push": "robot"}
#: Floor friction, from very slippery to slippery (the PyBullet floor is about 1.0).
FRICTION_RANGE = (0.01, 0.3)
#: `unknown` simulates each drive wheel: two compiled causes, published as two hypotheses.
WHEEL_SIDES = {"left": ("left",), "right": ("right",), "unknown": ("left", "right")}

#: Section 3.5. The radius of the robot's footprint, as CORRIDOR_HALF_WIDTH_M of study 5.
ROBOT_FOOTPRINT_RADIUS_M = 0.35
#: The cable lies along x and cannot be oriented: it is only simulable on a segment that crosses
#: the x axis by at least 45 deg.
CABLE_MAX_ABS_COS = 0.71
#: A directed force range is widened by this fraction of the largest magnitude on each side.
FORCE_WIDENING = 0.2
DECIMALS = 3

NO_BLUEPRINT = {"intervention": None, "parameters": {}, "activation_window": None}

#: Status of layer 3b (contract 3, section 4.3), besides incoherent, discarded and explained_by_memory,
#: which the contrast gives. PENDING is internal: grounded, waiting for the budget.
TO_SIMULATE, OVER_BUDGET, CHECKED_NOT_SIMULABLE, NOT_SIMULABLE = (
    "to_simulate", "over_budget", "checked_not_simulable", "not_simulable")
PENDING = "pending"


class HypothesisSchemaError(ValueError):
    """The layer-3a list breaks the schema: the generator retries (contract 3, section 4.2)."""

    def __init__(self, problems: list[str]):
        super().__init__("; ".join(problems))
        self.problems = problems


# --------------------------------------------------------------------------- #
# The mechanisms: what they admit and how they are published, from the TBox
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MechanismTerms:
    mechanism: Mechanism                    #: id, anchors, checks, realization and cost (episode_contrast)
    parameters: dict[str, tuple[str, ...]]  #: qualitative parameter -> its values, in the contract's order
    title_template: str
    interventions: tuple[str, ...]          #: catalog ids of its realizations (sorted); empty if none is in the catalog
    prompt_description: str                 #: the sentence the LLM reads
    trace: tuple[str, ...]                  #: observable properties on which it leaves a trace


@lru_cache(maxsize=4)
def load_vocabulary(tbox_path: str | Path = DEFAULT_TBOX) -> dict[str, MechanismTerms]:
    """The mechanisms of the TBox, by id, with their parameters, title and catalog intervention.

    Raises if the TBox has other mechanisms than contract 2 lists, or if the values it admits for a
    parameter are not the ones this module translates into numbers.
    """
    mechanisms = load_mechanisms(tbox_path)
    if set(mechanisms) != set(MECHANISM_ORDER):
        raise ValueError(f"the TBox mechanisms {sorted(mechanisms)} are not those of contract 2 {sorted(MECHANISM_ORDER)}")
    graph = Graph().parse(str(tbox_path), format="turtle")
    vocabulary: dict[str, MechanismTerms] = {}
    for mechanism_id in MECHANISM_ORDER:
        mechanism = mechanisms[mechanism_id]
        node = URIRef(mechanism.iri)
        declared = {str(graph.value(p, INSIGHT.parameterName)): {str(v) for v in graph.objects(p, INSIGHT.allowedValue)}
                    for p in graph.objects(node, INSIGHT.admitsParameter)}
        translated = PARAMETER_VALUES.get(mechanism_id, {})
        if declared != {name: set(values) for name, values in translated.items()}:
            raise ValueError(f"{mechanism_id}: the TBox admits {declared}, the grounding translates {translated}")
        catalog_ids = sorted(str(c) for i in graph.objects(node, INSIGHT.realizedBy)
                             for c in graph.objects(i, INSIGHT.catalogId))
        vocabulary[mechanism_id] = MechanismTerms(
            mechanism=mechanism, parameters=dict(translated),
            title_template=str(graph.value(node, INSIGHT.titleTemplate) or mechanism.label),
            interventions=tuple(catalog_ids),
            prompt_description=str(graph.value(node, INSIGHT.promptDescription) or mechanism.label),
            trace=tuple(sorted(str(p).rsplit("#", 1)[-1] for p in graph.objects(node, INSIGHT.entailsObservation))))
    return vocabulary


# --------------------------------------------------------------------------- #
# Step 1a: the schema of layer 3a
# --------------------------------------------------------------------------- #
def _numbers(value: Any, where: str):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        yield where
    elif isinstance(value, dict):
        for key, item in value.items():
            yield from _numbers(item, f"{where}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            yield from _numbers(item, f"{where}[{index}]")


def _text(value: Any, where: str, problems: list[str]) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        problems.append(f"{where} must be a string")
        return ""
    return value.strip()


def check_schema(proposal: Any, vocabulary: dict[str, MechanismTerms]) -> list[dict[str, Any]]:
    """The layer-3a hypotheses, normalized; HypothesisSchemaError with every problem otherwise.

    Refused: an unknown mechanism (other than `new`), a qualitative parameter that the mechanism
    does not admit (a magnitude among them), a value outside its list or a missing one (there is no
    default), `new` without a description, and any number. The anchors are only checked to be strings here: whether they exist in the episode is
    coherence (step 1b), and a wrong one only makes that hypothesis incoherent.
    """
    hypotheses = proposal.get("hypotheses") if isinstance(proposal, dict) else None
    if not isinstance(hypotheses, list) or not hypotheses:
        raise HypothesisSchemaError(["the proposal needs a non-empty 'hypotheses' list"])
    problems: list[str] = []
    normalized = []
    for index, raw in enumerate(hypotheses, start=1):
        where = f"hypothesis {index}"
        if not isinstance(raw, dict):
            problems.append(f"{where} must be an object")
            continue
        problems.extend(f"{path}: numbers are not allowed in layer 3a" for path in _numbers(raw, where))
        mechanism_id = _text(raw.get("mechanism"), f"{where}.mechanism", problems)
        if mechanism_id != NEW_MECHANISM and mechanism_id not in vocabulary:
            problems.append(f"{where}: unknown mechanism '{mechanism_id}'")
            continue
        description = _text(raw.get("new_mechanism_description"), f"{where}.new_mechanism_description", problems)
        if mechanism_id == NEW_MECHANISM and not description:
            problems.append(f"{where}: a 'new' mechanism needs new_mechanism_description")

        parameters = raw.get("qualitative_parameters") or {}
        if not isinstance(parameters, dict):
            problems.append(f"{where}.qualitative_parameters must be an object")
            parameters = {}
        parameters = {str(k): v.strip() if isinstance(v, str) else v for k, v in parameters.items()}
        if mechanism_id != NEW_MECHANISM:
            admitted = vocabulary[mechanism_id].parameters
            for name, value in parameters.items():
                if name not in admitted:
                    problems.append(f"{where}: {mechanism_id} admits no parameter '{name}'")
                elif value not in admitted[name]:
                    problems.append(f"{where}: {mechanism_id}.{name} must be one of {list(admitted[name])}, got '{value}'")
            for name in admitted:
                if name not in parameters:
                    problems.append(f"{where}: {mechanism_id} needs the parameter '{name}'")
        elif any(not isinstance(v, str) for v in parameters.values()):
            problems.append(f"{where}.qualitative_parameters of a 'new' mechanism must be words")

        expected_trace = raw.get("expected_trace") or []
        if not isinstance(expected_trace, list) or not all(isinstance(t, str) for t in expected_trace):
            problems.append(f"{where}.expected_trace must be a list of strings")
            expected_trace = []
        normalized.append({
            "mechanism": mechanism_id,
            "new_mechanism_description": description if mechanism_id == NEW_MECHANISM else None,
            "segment": _text(raw.get("segment"), f"{where}.segment", problems) or None,
            "interval": _text(raw.get("interval"), f"{where}.interval", problems) or None,
            "qualitative_parameters": parameters,
            "expected_trace": [t.strip() for t in expected_trace if t.strip()],
            "rationale": _text(raw.get("rationale"), f"{where}.rationale", problems),
            "title": _text(raw.get("title"), f"{where}.title", problems),
        })
    if problems:
        raise HypothesisSchemaError(problems)
    return normalized


# --------------------------------------------------------------------------- #
# Step 1b: the anchors exist and are the ones the mechanism admits
# --------------------------------------------------------------------------- #
def _anchor_problems(hypothesis: dict[str, Any], terms: Optional[MechanismTerms], view: EpisodeView) -> list[str]:
    """Why the anchors are not coherent with the episode (empty if they are).

    A mechanism needs exactly the anchors it admits (a `new` one may take either). An interval that
    starts at or after the fall (the system's reaction) cannot hold its cause.
    """
    mechanism_id = hypothesis["mechanism"]
    admitted = set(terms.mechanism.anchors) if terms is not None else {"segment", "interval"}
    problems = []
    for kind, known in (("segment", view.segments), ("interval", view.intervals)):
        anchor = hypothesis[kind]
        if anchor is None:
            if terms is not None and kind in admitted:
                problems.append(f"{mechanism_id} needs a {kind} anchor")
        elif kind not in admitted:
            problems.append(f"{mechanism_id} admits no {kind} anchor, got {anchor}")
        elif anchor not in known:
            problems.append(f"{kind} {anchor} does not exist in the episode")
        elif kind == "interval" and known[anchor][0] >= view.t_obs:
            problems.append(f"{anchor} starts at {known[anchor][0]:g} s, after the fall (t_obs {view.t_obs:g} s)")
    return problems


# --------------------------------------------------------------------------- #
# Step 3: grounding in numbers (contract 2, section 3.5)
# --------------------------------------------------------------------------- #
def _round(value: float) -> float:
    return round(float(value), DECIMALS) + 0.0      # + 0.0: no -0.0 in the JSON


def _clip(low: float, high: float, bounds: Any) -> Optional[list[float]]:
    if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
        low, high = max(low, float(bounds[0])), min(high, float(bounds[1]))
    return [_round(low), _round(high)] if low <= high else None


def _fmt(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


def _parameter_bounds(catalog: InterventionCatalog, intervention: str, parameter: str) -> Any:
    return catalog.interventions[intervention]["parameters"][parameter].get("bounds")


def _activation_window(view: EpisodeView, interval_id: str, horizon_s: float) -> tuple[dict[str, float], str]:
    start, end = view.intervals[interval_id]
    window = {"start_fraction": _round(min(max(start / horizon_s, 0.0), 1.0)),
              "end_fraction": _round(min(max(end / horizon_s, 0.0), 1.0))}
    return window, f"{interval_id} / H = [{_fmt(start)}, {_fmt(end)}] / {_fmt(horizon_s)}"


def mean_heading_deg(view: EpisodeView, interval_id: str) -> tuple[Optional[float], list[str]]:
    """Heading of the robot over an interval: the circular mean of the headings of the segments
    it overlaps, weighted by the overlap. It is the heading of the path, which is the robot's while
    it drives forward. None if no segment overlaps the interval."""
    start, end = view.intervals[interval_id]
    sx = sy = 0.0
    used = []
    for segment in view.segments.values():
        s_start, s_end = view.intervals[segment["interval"]]
        weight = min(end, s_end) - max(start, s_start)
        if weight > 0:
            angle = math.radians(segment["heading_deg"])
            sx, sy = sx + weight * math.cos(angle), sy + weight * math.sin(angle)
            used.append(segment["id"])
    if not used or math.hypot(sx, sy) < 1e-9:
        return None, used
    return math.degrees(math.atan2(sy, sx)), used


def _ground_obstacle(params, anchors, view, catalog) -> tuple[Optional[dict], dict[str, str], str]:
    segment_id = anchors["segment"]
    segment = view.segments[segment_id]
    bbox = segment["bbox"]
    if params["shape"] == "bump" and SCALED_DOME in catalog.interventions:
        return _ground_scaled_dome(segment_id, bbox, catalog)
    bounds = catalog.interventions["spawn_static_object"]["parameters"]["position_range"].get("bounds", {})
    if params["shape"] == "cable":
        asset = CABLE_ASSET
        abs_cos = abs(math.cos(math.radians(segment["heading_deg"])))
        if abs_cos > CABLE_MAX_ABS_COS:
            return None, {"asset": f"cable = {asset}"}, (
                f"The catalog cannot orient the cable, which lies along x, and {segment_id} runs almost "
                f"parallel to it (heading {segment['heading_deg']:g} deg, |cos| = {abs_cos:.2f} > {CABLE_MAX_ABS_COS}).")
        margin_y = ROBOT_FOOTPRINT_RADIUS_M + catalog.assets[asset]["approx_dimensions_m"][1] / 2.0
        x = _clip(bbox["x"][0], bbox["x"][1], bounds.get("x"))
        y = _clip(bbox["y"][0] - margin_y, bbox["y"][1] + margin_y, bounds.get("y"))
        trace = {"asset": f"cable = {asset}",
                 "position_range": f"x = bbox({segment_id}); y = bbox({segment_id}) + {_fmt(margin_y)} m "
                                   f"({_fmt(ROBOT_FOOTPRINT_RADIUS_M)} robot footprint + half the cable width); "
                                   f"heading {segment['heading_deg']:g} deg crosses x"}
    else:
        asset = params["asset"]
        half = max(catalog.assets[asset]["approx_dimensions_m"][:2]) / 2.0
        margin = ROBOT_FOOTPRINT_RADIUS_M + half
        x = _clip(bbox["x"][0] - margin, bbox["x"][1] + margin, bounds.get("x"))
        y = _clip(bbox["y"][0] - margin, bbox["y"][1] + margin, bounds.get("y"))
        trace = {"asset": f"bump = {asset}",
                 "position_range": f"bbox({segment_id}) + {_fmt(margin)} m ({_fmt(ROBOT_FOOTPRINT_RADIUS_M)} robot "
                                   f"footprint + {_fmt(half)} half of {asset}), clipped to the catalog"}
    if x is None or y is None:
        return None, trace, f"{segment_id} lies outside the catalog bounds for the obstacle position."
    blueprint = {"intervention": "spawn_static_object",
                 "parameters": {"asset": asset, "position_range": {"x": x, "y": y, "z": [0.0, 0.0]}},
                 "activation_window": None}
    return blueprint, trace, ""


def _ground_scaled_dome(segment_id, bbox, catalog) -> tuple[Optional[dict], dict[str, str], str]:
    """A bump of unknown size: the area it must reach (the segment plus the robot footprint) and the
    whole size range the catalog simulates. The simulator adds half of each drawn diameter."""
    spec = catalog.interventions[SCALED_DOME]["parameters"]
    area_bounds = spec["area"].get("bounds", {})
    x = _clip(bbox["x"][0] - ROBOT_FOOTPRINT_RADIUS_M, bbox["x"][1] + ROBOT_FOOTPRINT_RADIUS_M, area_bounds.get("x"))
    y = _clip(bbox["y"][0] - ROBOT_FOOTPRINT_RADIUS_M, bbox["y"][1] + ROBOT_FOOTPRINT_RADIUS_M, area_bounds.get("y"))
    diameter = [_round(v) for v in spec["diameter_range"]["bounds"]]
    height = [_round(v) for v in spec["height_range"]["bounds"]]
    trace = {"area": f"bbox({segment_id}) + {_fmt(ROBOT_FOOTPRINT_RADIUS_M)} m (robot footprint); each repetition "
                     "adds half its drawn diameter",
             "size": f"not chosen: diameter {_fmt(diameter[0])}-{_fmt(diameter[1])} m and height "
                     f"{_fmt(height[0])}-{_fmt(height[1])} m, the whole range the catalog simulates"}
    if x is None or y is None:
        return None, trace, f"{segment_id} lies outside the catalog bounds for the bump area."
    blueprint = {"intervention": SCALED_DOME,
                 "parameters": {"area": {"x": x, "y": y, "z": [0.0, 0.0]}, "diameter_range": diameter,
                                "height_range": height},
                 "activation_window": None}
    return blueprint, trace, ""


def _ground_push(mechanism_id, params, anchors, view, horizon_s, catalog) -> tuple[Optional[dict], dict[str, str], str]:
    low, high = PUSH_FORCE_N[mechanism_id]
    direction = params["direction"]
    bounds = _parameter_bounds(catalog, "apply_external_force", "force_range") or {}
    trace = {"force_range": f"{_fmt(low)}-{_fmt(high)} N (from a light to a strong push), direction {direction}"}
    if direction == "any":
        ranges = {"x": (-high, high), "y": (-high, high)}
    else:
        heading, used = mean_heading_deg(view, anchors["interval"])
        if heading is None:
            return None, trace, f"No segment overlaps {anchors['interval']}: the robot heading there is unknown."
        h = math.radians(heading)
        forward, left = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
        unit = {"forward": forward, "backward": (-forward[0], -forward[1]),
                "left": left, "right": (-left[0], -left[1])}[direction]
        ranges = {}
        for axis, component in zip(("x", "y"), unit):
            a, b = sorted((component * low, component * high))
            ranges[axis] = (a - FORCE_WIDENING * high, b + FORCE_WIDENING * high)
        trace["force_range"] += (f", relative to h = {heading:.1f} deg (heading of {', '.join(used)} over "
                                 f"{anchors['interval']}), widened by {FORCE_WIDENING:g} x {_fmt(high)} N")
    force = {axis: _clip(a, b, bounds.get(axis)) for axis, (a, b) in ranges.items()}
    force["z"] = [0.0, 0.0]
    if any(r is None for r in force.values()):
        return None, trace, "The force range falls outside the catalog bounds."
    window, trace["activation_window"] = _activation_window(view, anchors["interval"], horizon_s)
    blueprint = {"intervention": "apply_external_force",
                 "parameters": {"target": PUSH_TARGET[mechanism_id], "force_range": force},
                 "activation_window": window}
    return blueprint, trace, ""


def _ground_friction(params, catalog) -> tuple[Optional[dict], dict[str, str], str]:
    low, high = FRICTION_RANGE
    friction = _clip(low, high, _parameter_bounds(catalog, "set_friction", "lateral_friction_range"))
    trace = {"lateral_friction_range": f"mu in [{_fmt(low)}, {_fmt(high)}] (from very slippery to slippery)"}
    if friction is None:
        return None, trace, "The friction range falls outside the catalog bounds."
    blueprint = {"intervention": "set_friction",
                 "parameters": {"target": "floor", "lateral_friction_range": friction},
                 "activation_window": None}
    return blueprint, trace, ""


def _ground_wheel(params, anchors, view, horizon_s) -> tuple[Optional[dict], dict[str, str], str]:
    trace = {"wheel_id": f"side {params['side']}"}
    window, trace["activation_window"] = _activation_window(view, anchors["interval"], horizon_s)
    return {"intervention": "disable_wheel", "parameters": {"wheel_id": params["side"]},
            "activation_window": window}, trace, ""


def ground(mechanism_id: str, params: dict[str, str], anchors: dict[str, Optional[str]], view: EpisodeView,
           horizon_s: float, catalog: InterventionCatalog) -> tuple[Optional[dict], dict[str, str], str]:
    """(blueprint v2, how each number was obtained, why it cannot be grounded) for one compiled cause.

    `params` is one variant: a single side for a wheel, and the asset for a bump (see variants). The
    blueprint is checked against the catalog; one it refuses is reported, never published.
    """
    if mechanism_id == "obstacle_traversed":
        blueprint, trace, reason = _ground_obstacle(params, anchors, view, catalog)
    elif mechanism_id in PUSH_FORCE_N:
        blueprint, trace, reason = _ground_push(mechanism_id, params, anchors, view, horizon_s, catalog)
    elif mechanism_id == "slippery_floor":
        blueprint, trace, reason = _ground_friction(params, catalog)
    elif mechanism_id == "wheel_failure":
        blueprint, trace, reason = _ground_wheel(params, anchors, view, horizon_s)
    else:
        raise ValueError(f"no grounding for mechanism '{mechanism_id}'")
    if blueprint is None:
        return None, trace, reason
    grounded = catalog.ground_blueprint(blueprint)
    if not grounded.testable:
        return None, trace, f"The catalog refuses the grounded blueprint: {grounded.reason}"
    return {**grounded.blueprint, "activation_window": blueprint["activation_window"]}, trace, ""


# --------------------------------------------------------------------------- #
# Titles, written from the mechanism and its anchors
# --------------------------------------------------------------------------- #
def segment_label(view: EpisodeView, segment_id: str) -> str:
    """'the last 1 m of the path before the fall', 'the path between 1 and 2 m before the fall'."""
    segment = view.segments.get(segment_id)
    if segment is None:
        return segment_id
    order = segment.get("order_back_from_fall", 0)
    start = sum(s["length_m"] for s in view.segments.values() if s.get("order_back_from_fall", 0) < order)
    end = start + segment["length_m"]
    if start == 0:
        return f"the last {_fmt(round(end, 1))} m of the path before the fall"
    return f"the path between {_fmt(round(start, 1))} and {_fmt(round(end, 1))} m before the fall"


def interval_label(view: EpisodeView, interval_id: str) -> str:
    """'the last 2 s before the fall', 'the drive over <segment>', or the times."""
    if interval_id not in view.intervals:
        return interval_id
    start, end = view.intervals[interval_id]
    if interval_id == FALL_INTERVAL:
        return f"the last {_fmt(round(end - start, 1))} s before the fall"
    for segment in view.segments.values():
        if segment["interval"] == interval_id:
            return f"the drive over {segment_label(view, segment['id'])}"
    return f"{start:.1f}-{end:.1f} s"


def write_title(template: str, params: dict[str, str], anchors: dict[str, Optional[str]], view: EpisodeView) -> str:
    values = {name: value.replace("_", " ") for name, value in params.items()}
    if anchors.get("segment"):
        values["segment"] = segment_label(view, anchors["segment"])
    if anchors.get("interval"):
        values["interval"] = interval_label(view, anchors["interval"])
    text = " ".join(re.sub(r"\{(\w+)\}", lambda m: values.get(m.group(1), ""), template).split())
    return text[:1].upper() + text[1:]


# --------------------------------------------------------------------------- #
# One hypothesis: coherence, contrast and grounding
# --------------------------------------------------------------------------- #
def bump_assets(catalog: InterventionCatalog) -> list[str]:
    """The bumps of the catalog, in its order."""
    return [asset for asset in catalog.assets if asset.startswith(BUMP_ASSET_PREFIX)]


def variants(hypothesis: dict[str, Any], catalog: InterventionCatalog) -> list[tuple[str, dict[str, str], str]]:
    """(id suffix, parameters, why it is split) of each compiled cause of a hypothesis: one per
    side for a wheel of unknown side; for a bump, one per bump asset when the catalog has no dome of
    unknown size; one otherwise."""
    params = hypothesis["qualitative_parameters"]
    if hypothesis["mechanism"] == "wheel_failure" and len(WHEEL_SIDES[params["side"]]) > 1:
        sides = WHEEL_SIDES[params["side"]]
        return [(f"_{side}", {**params, "side": side},
                 f"side {params['side']}: each drive wheel is one compiled cause; this one is the {side} one")
                for side in sides]
    if (hypothesis["mechanism"] == "obstacle_traversed" and params.get("shape") == "bump"
            and SCALED_DOME not in catalog.interventions):
        assets = bump_assets(catalog)
        return [(f"_{asset}", {**params, "asset": asset},
                 f"bump: its size is not chosen, so each of the {len(assets)} bumps of the catalog is one "
                 f"compiled cause; this one is {asset}") for asset in assets]
    return [("", dict(params), "")]


def _not_grounded_reason(terms: MechanismTerms) -> str:
    mechanism = terms.mechanism
    if mechanism.not_simulable_because:
        return mechanism.not_simulable_because
    if mechanism.realization_experimental:
        return (f"Its realization ({', '.join(mechanism.realized_by)}) is experimental, only in experiments/: it enters "
                "the catalog if task B1 shows on 16/10 that it tips the bottle (contract decision 10).")
    return (f"Its realization ({', '.join(mechanism.realized_by)}) is the nominal run, which is always simulated: "
            "there is no cause to add.")


def _status_reason(status: str, decided: Optional[dict[str, Any]], terms: MechanismTerms) -> str:
    """The untestable_reason of the status the contrast leaves (empty while pending)."""
    if status == INCOHERENT:
        return f"Incoherent: precondition {decided['check']} fails ({decided['reason']})."
    if status == DISCARDED:
        return f"Discarded before simulating by {decided['check']}: {decided['reason']}."
    if status == EXPLAINED_BY_MEMORY:
        return (f"Explained by memory ({decided['check']}: {decided['reason']}): the nominal run, always "
                "simulated, replays the commanded setpoint.")
    if status == CHECKED_NOT_SIMULABLE:
        return f"Compatible with the recording but not simulable: {_not_grounded_reason(terms)}"
    if status == NOT_SIMULABLE:
        return f"Not simulable: {_not_grounded_reason(terms)}"
    return ""


def process_hypothesis(hypothesis: dict[str, Any], rank: int, view: EpisodeView, horizon_s: float,
                       vocabulary: dict[str, MechanismTerms], catalog: InterventionCatalog) -> list[dict[str, Any]]:
    """The published entries of one layer-3a hypothesis (two for a wheel of unknown side), after
    coherence, contrast and grounding. Their status is final, or PENDING until the budget."""
    mechanism_id = hypothesis["mechanism"]
    terms = vocabulary.get(mechanism_id)
    anchors = {"segment": hypothesis["segment"], "interval": hypothesis["interval"]}
    base = {
        "hypothesis_id": f"H{rank:02d}", "rank": rank,
        "mechanism": mechanism_id, "mechanism_iri": terms.mechanism.iri if terms else None,
        "new_mechanism_description": hypothesis["new_mechanism_description"],
        "anchors": anchors, "qualitative_parameters": dict(hypothesis["qualitative_parameters"]),
        "expected_trace": hypothesis["expected_trace"], "rationale": hypothesis["rationale"],
        "llm_title": hypothesis["title"],
    }
    anchor_problems = _anchor_problems(hypothesis, terms, view)

    if terms is None:                       # `new`: kept to see whether the vocabulary must grow
        status = INCOHERENT if anchor_problems else NOT_SIMULABLE
        reason = ("; ".join(anchor_problems) if anchor_problems else
                  "New mechanism, not in the list: not simulable; kept to see whether the vocabulary must grow.")
        return [{**base, "title": f"Unlisted mechanism: {hypothesis['new_mechanism_description']}",
                 "family": None, "status": status,
                 "checks": {"coherence": {"passed": not anchor_problems, "reason": "; ".join(anchor_problems),
                                          "preconditions": []}, "contrast": []},
                 "grounding_trace": {}, "cost_in_simulations": 0, "untestable_reason": reason,
                 "simulation_blueprint": dict(NO_BLUEPRINT)}]

    mechanism = terms.mechanism
    if anchor_problems:
        coherence = {"passed": False, "reason": "; ".join(anchor_problems), "preconditions": []}
        contrast_checks, status, reason = [], INCOHERENT, f"Incoherent: {'; '.join(anchor_problems)}."
    else:
        contrast = contrast_hypothesis(view, mechanism_id, anchors, {m: t.mechanism for m, t in vocabulary.items()})
        preconditions = [c for c in contrast["checks"] if c["kind"] == "precondition"]
        contrast_checks = [c for c in contrast["checks"] if c["kind"] == "contrast_rule"]
        decided = next((c for c in contrast["checks"] if c["check"] == contrast["decided_by"]), None)
        coherence = {"passed": contrast["outcome"] != INCOHERENT,
                     "reason": "; ".join(f"{c['check']}: {c['reason']}" for c in preconditions),
                     "preconditions": preconditions}
        status = contrast["status_after_contrast"]
        reason = _status_reason(status, decided, terms)

    entries = []
    for suffix, params, split in variants(hypothesis, catalog):
        own = [value for name, value in params.items() if name not in hypothesis["qualitative_parameters"]]
        entry = {**base, "hypothesis_id": base["hypothesis_id"] + suffix,
                 "title": write_title(terms.title_template, params, anchors, view)
                          + (f" ({', '.join(own)})" if own else ""),
                 "family": mechanism.family, "status": status,
                 "checks": {"coherence": coherence, "contrast": contrast_checks},
                 "grounding_trace": {}, "cost_in_simulations": mechanism.cost_in_simulations,
                 "untestable_reason": reason, "simulation_blueprint": dict(NO_BLUEPRINT)}
        if split:
            entry["grounding_trace"]["split"] = split
        # Every coherent hypothesis with a catalog realization is grounded, also when it is discarded
        # or over budget: the blueprint says what was not simulated. Only to_simulate is testable.
        if status != INCOHERENT and terms.interventions:
            blueprint, trace, why_not = ground(mechanism_id, params, anchors, view, horizon_s, catalog)
            entry["grounding_trace"].update(trace)
            if blueprint is not None:
                entry["simulation_blueprint"] = blueprint
                # An intervention that simulates more than one cause's repetitions costs what it runs.
                entry["cost_in_simulations"] = int(catalog.interventions[blueprint["intervention"]].get(
                    "budget_cost", entry["cost_in_simulations"]))
            elif status == PENDING:
                entry.update(status=NOT_SIMULABLE, untestable_reason=f"Not simulable here: {why_not}")
            else:
                entry["grounding_trace"]["not_grounded"] = why_not
        elif status == PENDING:
            entry.update(status=NOT_SIMULABLE, untestable_reason=f"Not simulable: {_not_grounded_reason(terms)}")
        entries.append(entry)
    return entries


# --------------------------------------------------------------------------- #
# Step 4: the budget, and the batch
# --------------------------------------------------------------------------- #
def apply_budget(entries: list[dict[str, Any]], budget: Optional[int]) -> int:
    """Walk the hypotheses in the proposed order: one is simulated while its compiled causes fit in
    k (all of them or none: both wheels of an unknown side). The walk stops at the first one that
    does not fit, so a later, cheaper one never overtakes it. Without a budget (None) every one is
    simulated. The nominal run is free. Returns the causes used."""
    if budget is None:
        budget = math.inf
    used, exhausted = 0, False
    for _, group in itertools.groupby(entries, key=lambda e: e["rank"]):
        group = [e for e in group if e["status"] == PENDING]
        if not group:
            continue
        cost = sum(e["cost_in_simulations"] for e in group)
        if not exhausted and used + cost <= budget:
            used += cost
            for entry in group:
                entry.update(status=TO_SIMULATE, untestable_reason="")
        else:
            exhausted = True
            for entry in group:
                entry.update(status=OVER_BUDGET,
                             untestable_reason=f"Over budget: k = {budget} compiled causes, {used} already used.")
    return used


def episode_sha256(episode: dict[str, Any]) -> str:
    """Hash of the episode JSON, canonical (sorted keys, no spaces)."""
    canonical = json.dumps(episode, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _header(episode: dict[str, Any], *, arm: str, budget: Optional[int], used: int, status: str, errors: list[str],
            case_id: Optional[str] = None, trace_id: Optional[str] = None, generated_at: Optional[str] = None,
            trigger: Optional[dict] = None, context_summary: Optional[dict] = None, model: Optional[str] = None,
            prompt_path: Optional[str] = None, attempts: Optional[int] = None, episode_path: Optional[str] = None,
            tbox_path: str | Path = DEFAULT_TBOX) -> dict[str, Any]:
    """The fields of a layer-3b batch other than its hypotheses (contract 3, section 4.3)."""
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got '{arm}'")
    source = episode.get("source") or {}
    return {
        "schema_version": SCHEMA_VERSION,
        "case_id": case_id or episode["episode_id"],
        "trace_id": trace_id or f"trace_{uuid.uuid4().hex}",
        "generated_at": generated_at or datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": status,
        "errors": list(errors),
        "trigger": trigger or {},
        "context_summary": context_summary or {},
        "arm": arm,
        "episode": {"id": episode["episode_id"], "path": episode_path, "sha256": episode_sha256(episode),
                    "recording": {"path": source.get("path"), "sha256": source.get("sha256")}},
        "mechanisms_version": hashlib.sha256(Path(tbox_path).read_bytes()).hexdigest(),
        "budget": {"k": budget, "used": used, "unit": "compiled cause; the nominal run is free"},
        "model": model,
        "prompt_path": prompt_path,
        "attempts": attempts,
    }


def error_batch(episode: dict[str, Any], errors: list[str], *, arm: str, budget: Optional[int] = DEFAULT_BUDGET,
                **metadata: Any) -> dict[str, Any]:
    """The batch published when no proposal passed the schema: no hypothesis, and why."""
    return {**_header(episode, arm=arm, budget=budget, used=0, status="error", errors=errors, **metadata),
            "hypotheses": []}


def publish_batch(episode: dict[str, Any], proposal: Any, *, arm: str, budget: Optional[int] = DEFAULT_BUDGET,
                  tbox_path: str | Path = DEFAULT_TBOX, catalog: Optional[InterventionCatalog] = None,
                  **metadata: Any) -> dict[str, Any]:
    """The batch of layer 3b for a layer-3a proposal over the episode.

    `metadata` are the batch fields of _header: case_id, trace_id, generated_at, trigger,
    context_summary, model, prompt_path, attempts and episode_path. Raises HypothesisSchemaError if
    the proposal breaks the schema (the generator retries).
    """
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got '{arm}'")
    if budget is not None and budget < 0:
        raise ValueError("the budget k must be >= 0 (None: no budget)")
    vocabulary = load_vocabulary(tbox_path)
    catalog = catalog or InterventionCatalog.from_file(DEFAULT_CATALOG)
    # The fixed bump assets stand in for the dome of unknown size when the catalog lacks it.
    missing = {i for t in vocabulary.values() for i in t.interventions} - set(catalog.interventions) - {SCALED_DOME}
    if missing:
        raise ValueError(f"the TBox realizes mechanisms with interventions the catalog lacks: {sorted(missing)}")
    hypotheses = check_schema(proposal, vocabulary)

    view = EpisodeView(episode)
    horizon_s = float(episode["time"]["simulation_horizon_s"])
    entries = [entry for rank, hypothesis in enumerate(hypotheses, start=1)
               for entry in process_hypothesis(hypothesis, rank, view, horizon_s, vocabulary, catalog)]
    used = apply_budget(entries, budget)
    for entry in entries:
        entry["testable"] = entry["status"] == TO_SIMULATE
    ordered = ("hypothesis_id", "rank", "mechanism", "mechanism_iri", "new_mechanism_description", "anchors",
               "qualitative_parameters", "expected_trace", "rationale", "llm_title", "title", "family", "status",
               "checks", "grounding_trace", "cost_in_simulations", "testable", "untestable_reason",
               "simulation_blueprint")
    return {**_header(episode, arm=arm, budget=budget, used=used, status="success", errors=[], tbox_path=tbox_path,
                      **metadata),
            "hypotheses": [{key: entry[key] for key in ordered} for entry in entries]}


# --------------------------------------------------------------------------- #
# The anchored enumeration (task B4), as layer 3a
# --------------------------------------------------------------------------- #
def anchored_enumeration(tbox_path: str | Path = DEFAULT_TBOX) -> dict[str, Any]:
    """Every mechanism on the anchors of the fall (Segment_final, Interval_fall), with every value
    of its qualitative parameters, in the order of contract 2.

    Values that only gather others are dropped when they add no cause: `direction: any` is kept
    (one force range covers every direction) and the four directions are dropped; `side: unknown`
    is dropped, since `left` and `right` are the same two compiled causes. The magnitudes are not
    enumerated here: the grounding covers them.
    """
    vocabulary = load_vocabulary(tbox_path)
    hypotheses = []
    for mechanism_id in MECHANISM_ORDER:
        terms = vocabulary[mechanism_id]
        anchors = default_anchors(terms.mechanism)
        choices = {name: [v for v in values if not (name == "direction" and v != "any") and v != "unknown"]
                   for name, values in terms.parameters.items()}
        for combination in itertools.product(*choices.values()):
            params = dict(zip(choices, combination))
            hypotheses.append({"mechanism": mechanism_id, "new_mechanism_description": None,
                               "segment": anchors["segment"], "interval": anchors["interval"],
                               "qualitative_parameters": params, "expected_trace": [], "rationale": "",
                               "title": ""})
    return {"hypotheses": hypotheses}
