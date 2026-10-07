"""The LLM in front of the deterministic part of the new generation (contract 3, section 4.2; task A4, subpaso 3.2).

The prompt gives the LLM, once each:

  * the episode (contract 1) in readable form: what happened, the phases, the path segments and
    the time intervals a hypothesis may be anchored to, and the evidence, each property explained
    with its comment in the TBox. The robot's reaction after the fall is not offered as evidence
    nor as an anchor;
  * the mechanisms (contract 2), from the TBox: id, description, anchors, qualitative parameters
    (kinds only: the magnitudes are the memory's), precondition, the properties on which they leave
    a trace, whether they can be simulated and what they cost;
  * the robot's self-model (description.md).

The LLM answers in symbols (layer 3a) and hypothesis_pipeline turns the answer into the published
batch (layer 3b). An answer that is not a JSON object, or breaks the schema, is sent back with its
problems and asked again, up to `max_attempts`: at temperature 0, asking again without them would
bring back the same answer. A failure to reach the LLM is retried as it was. The batch is saved
next to its prompt and to the whole conversation with the LLM. Nothing here reads the truth of a
case.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable, Optional

import requests
from rdflib import RDF, RDFS, Graph, Namespace

from src.episode_contrast import DEFAULT_TBOX, FALL_INTERVAL, EpisodeView
from src.hypothesis_pipeline import (
    DEFAULT_BUDGET, DEFAULT_CATALOG, NEW_MECHANISM, SCALED_DOME, WHEEL_SIDES, HypothesisSchemaError, bump_assets,
    error_batch, interval_label, load_vocabulary, publish_batch, segment_label,
)
from src.intervention_catalog import InterventionCatalog
from src.hypothesis_service import _extract_json_object, _normalize_message_content, _ollama_native_metrics

REPO = Path(__file__).resolve().parents[3]
DEFAULT_SELF_MODEL = REPO / "description.md"
SOSA = Namespace("http://www.w3.org/ns/sosa/")

ARM = "llm_full"
DEFAULT_MAX_ATTEMPTS = 3
MAX_HYPOTHESES = 10
#: DescriptionCharLimit of agents/semantic/etc/config: the self-model goes whole if it fits.
SELF_MODEL_CHAR_LIMIT = 12000

#: The LLM: chat messages -> (answer, metrics). It raises when the LLM cannot be reached.
LLM = Callable[[list[dict[str, str]]], tuple[str, dict[str, Any]]]


def ollama_chat(model: str, base_url: str = "http://localhost:11434", timeout_s: float = 300.0) -> LLM:
    """The Ollama chat endpoint at temperature 0, as production calls it (_invoke_ollama_http)."""
    session = requests.Session()
    endpoint = base_url.rstrip("/") + "/api/chat"

    def chat(messages: list[dict[str, str]]) -> tuple[str, dict[str, Any]]:
        response = session.post(endpoint, timeout=timeout_s, json={
            "model": model, "messages": messages, "stream": False, "options": {"temperature": 0}})
        response.raise_for_status()
        data = response.json()
        return _normalize_message_content((data.get("message") or {}).get("content", "")), _ollama_native_metrics(data)

    return chat


# --------------------------------------------------------------------------- #
# The prompt
# --------------------------------------------------------------------------- #
def _n(value: Any) -> str:
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".")
    return str(value)


@lru_cache(maxsize=4)
def observable_properties(tbox_path: str | Path = DEFAULT_TBOX) -> dict[str, tuple[str, str]]:
    """Each observable property of the TBox, by local name: (label, comment)."""
    graph = Graph().parse(str(tbox_path), format="turtle")
    return {str(p).rsplit("#", 1)[-1]: (str(graph.value(p, RDFS.label) or ""), str(graph.value(p, RDFS.comment) or ""))
            for p in graph.subjects(RDF.type, SOSA.ObservableProperty)}


def _anchor_intervals(view: EpisodeView) -> list[str]:
    """The intervals a hypothesis may be anchored to: those that start before the fall."""
    return [i for i, (start, _) in view.intervals.items() if start < view.t_obs]


def _interval_meaning(view: EpisodeView, interval_id: str) -> str:
    if interval_id == "Interval_baseline":
        return f"free motion before {FALL_INTERVAL}: the reference the peaks are compared with"
    for phase in view.phases:
        if phase["interval"] == interval_id:
            return f"the whole of {phase['id']} ({phase['kind']})"
    return interval_label(view, interval_id)


def _evidence_line(entry: dict[str, Any], properties: dict[str, tuple[str, str]]) -> str:
    prop = entry["property"]
    unit = entry.get("unit")
    unit = "" if unit in (None, "1") else f" {unit}"
    text = f"{prop} = {_n(entry.get('value'))}{unit if not isinstance(entry.get('value'), bool) else ''}"
    if entry.get("at_s") is not None:
        text += f" at {_n(entry['at_s'])} s"
    if entry.get("interval"):
        text += f", over {entry['interval']}"
    if entry.get("baseline") is not None:
        text += f"; in free motion: {_n(entry['baseline'])}{unit}"
    if entry.get("commanded") is not None:
        text += f"; commanded: {_n(entry['commanded'])}{unit}"
    if entry.get("own_ratio_fall") is not None:
        text += (f"; travelled over commanded: {_n(entry['own_ratio_fall'])} there, "
                 f"{_n(entry.get('own_ratio_baseline'))} in free motion")
    label, comment = properties.get(prop, ("", ""))
    return f"- {text}." + (f" ({label}: {comment})" if comment else "")


def describe_episode(episode: dict[str, Any], tbox_path: str | Path = DEFAULT_TBOX) -> str:
    """The episode of contract 1 in readable form, with the ids the LLM anchors to."""
    view = EpisodeView(episode)
    properties = observable_properties(tbox_path)
    support, accident = episode["support"], episode["accident"]
    pose = accident["robot_pose"]
    onset = next((e.get("value") for e in episode["evidence"] if e["property"] == "reaction_onset"), None)
    lines = [
        f"## The episode {episode['episode_id']}",
        "",
        "Room frame: x and y in metres, headings in degrees (0 = +x, counter-clockwise). Times in seconds "
        "from the start of the recording.",
        "",
        "### What happened",
        f"- The robot was following a person with a bottle on its tray. {support['id']}: the "
        f"{'tray' if support['supporter'] == episode['entities'].get('tray') else 'robot'} held the "
        f"bottle from {_n(support['start_s'])} s to {_n(support['end_s'])} s.",
        f"- {accident['id']}: the bottle was lost at t_obs = {_n(view.t_obs)} s, with the robot at "
        f"({_n(pose['xy'][0])}, {_n(pose['xy'][1])}), heading {_n(pose['heading_deg'])} deg, on {accident['place']}. "
        "Its cause is unknown.",
        "- After t_obs the robot reacted to the loss" + (f" (its speed setpoint went to 0 at {_n(onset)} s)" if onset else "")
        + ". That reaction is a consequence, not a cause: it is left out below.",
        "",
        "### Phases of the motion before the fall",
    ]
    for phase in view.phases:
        start, end = view.intervals[phase["interval"]]
        details = [f"mean speed {_n(phase['mean_speed_mps'])} m/s"]
        if phase.get("setpoint_mps"):
            details.append(f"commanded {_n(phase['setpoint_mps'][0])}-{_n(phase['setpoint_mps'][1])} m/s")
        if phase.get("heading_deg_range"):
            details.append(f"heading from {_n(phase['heading_deg_range'][0])} to {_n(phase['heading_deg_range'][1])} deg")
        lines.append(f"- {phase['id']} ({phase['kind']}), {phase['interval']} = [{_n(start)}, {_n(end)}] s: "
                     + ", ".join(details) + ".")
    lines += ["", "### Path segments, counted back from the fall (the anchors for a place: `segment`)"]
    for segment in sorted(view.segments.values(), key=lambda s: s.get("order_back_from_fall", 0)):
        start, end = view.intervals[segment["interval"]]
        lines.append(f"- {segment['id']}: {segment_label(view, segment['id'])}, from ({_n(segment['from_xy'][0])}, "
                     f"{_n(segment['from_xy'][1])}) to ({_n(segment['to_xy'][0])}, {_n(segment['to_xy'][1])}), heading "
                     f"{_n(segment['heading_deg'])} deg, mean speed {_n(segment['mean_speed_mps'])} m/s, driven during "
                     f"{segment['interval']} = [{_n(start)}, {_n(end)}] s.")
    lines += ["", "### Time intervals (the anchors for a time: `interval`)"]
    for interval_id in _anchor_intervals(view):
        start, end = view.intervals[interval_id]
        lines.append(f"- {interval_id} = [{_n(start)}, {_n(end)}] s: {_interval_meaning(view, interval_id)}.")
    lines += ["", "### Evidence (what the recording shows; \"in free motion\" is the same quantity over Interval_baseline)"]
    for entry in episode["evidence"]:
        if entry.get("is_system_reaction") or entry["property"] == "reaction_onset":
            continue
        lines.append(_evidence_line(entry, properties))
    return "\n".join(lines)


def _cost(mechanism_id: str, terms, catalog: InterventionCatalog) -> str:
    """What a hypothesis of the mechanism takes from the budget, in words."""
    unit = terms.mechanism.cost_in_simulations
    if not terms.interventions or unit == 0:
        return "none (not simulated)"
    if mechanism_id == "obstacle_traversed":
        if SCALED_DOME in catalog.interventions:
            cost = catalog.interventions[SCALED_DOME].get("budget_cost", unit)
            return f"a bump, {cost} simulations (its size is drawn over the whole range); a cable, {unit}"
        return f"a bump, {len(bump_assets(catalog)) * unit} simulations (one per bump size); a cable, {unit}"
    if mechanism_id == "wheel_failure":
        return f"{unit} per side; side unknown, {len(WHEEL_SIDES['unknown']) * unit}"
    return f"{unit} simulation" + ("s" if unit > 1 else "")


def describe_mechanisms(tbox_path: str | Path = DEFAULT_TBOX, catalog: Optional[InterventionCatalog] = None,
                        with_costs: bool = True) -> str:
    """The closed list of mechanisms of the TBox, as the LLM reads it. The costs only matter with a
    budget."""
    catalog = catalog or InterventionCatalog.from_file(DEFAULT_CATALOG)
    lines = ["## The mechanisms", "",
             "The closed list of ways the bottle can fall. Use their ids. Their parameters are kinds only: how big, "
             "how strong or how slippery is not chosen, because the memory simulates the whole range it can.", ""]
    for mechanism_id, terms in load_vocabulary(tbox_path).items():
        mechanism = terms.mechanism
        parameters = "; ".join(f"{name}: {' | '.join(values)}" for name, values in terms.parameters.items()) or "none"
        if terms.interventions:
            simulable = "yes."
        elif mechanism.not_simulable_because:
            simulable = f"no. {mechanism.not_simulable_because}"
        elif mechanism.realization_experimental:
            simulable = "not yet: its simulation is experimental."
        else:
            simulable = "the nominal run, which replays the recorded setpoints, already reproduces it."
        lines += [f"- {mechanism_id}: {terms.prompt_description}",
                  f"  Anchors: {', '.join(mechanism.anchors) or 'none'}. Parameters: {parameters}.",
                  f"  Precondition: {' '.join(c.comment for c in mechanism.preconditions) or 'none.'}",
                  f"  Leaves a trace on: {', '.join(terms.trace) or 'nothing the recording measures'}.",
                  f"  Simulable: {simulable}" + (f" Cost: {_cost(mechanism_id, terms, catalog)}." if with_costs else "")]
    lines.append(f"- {NEW_MECHANISM}: a mechanism that is not in this list. Describe it in new_mechanism_description; "
                 "its anchors are optional. It is kept to extend the list, not simulated.")
    return "\n".join(lines)


def read_self_model(path: Path = DEFAULT_SELF_MODEL, char_limit: int = SELF_MODEL_CHAR_LIMIT) -> str:
    text = path.read_text(encoding="utf-8").strip()
    return text if len(text) <= char_limit else text[:char_limit].rsplit("\n", 1)[0]


ANSWER_FORMAT = {"hypotheses": [{
    "mechanism": "<a mechanism id, or new>",
    "new_mechanism_description": "<only with new; otherwise null>",
    "segment": "<a segment id, or null>",
    "interval": "<an interval id, or null>",
    "qualitative_parameters": {"<parameter>": "<one of its values>"},
    "expected_trace": ["<what the recording should show if this is the cause>"],
    "rationale": "<why, from the evidence>",
    "title": "<a short title>",
}]}


def build_prompt(episode: dict[str, Any], self_model: str, budget: Optional[int] = DEFAULT_BUDGET,
                 tbox_path: str | Path = DEFAULT_TBOX) -> str:
    if budget is None:
        simulated = ("Each one is checked against the recording, and every one that survives and can be simulated "
                     "is simulated;")
    else:
        simulated = (f"Each one is checked against the recording, and up to {budget} simulations are run, in your "
                     "order, with the ones that survive. Each hypothesis takes the cost of its mechanism, all of it "
                     "or nothing: the first one that does not fit in what is left ends the simulations;")
    task = [
        "## Your task",
        "",
        "The bottle fell from the tray and the robot perceived no cause. Propose the mechanisms that could "
        "explain the fall, as hypotheses written in symbols:",
        "- each hypothesis names one mechanism id from the list (or `new`), the anchors that mechanism takes "
        "(segment and interval ids from the episode), and its qualitative parameters. Every parameter of the "
        "mechanism is required, with one of its listed values;",
        "- never write a number: no coordinates, times, fractions, forces nor probabilities. The memory turns "
        "the symbols into numbers;",
        "- read the evidence: a hypothesis should explain what the recording shows;",
        f"- order the hypotheses from the most to the least plausible; propose at most {MAX_HYPOTHESES}. {simulated}",
        "- propose a mechanism that explains the evidence even if it cannot be simulated;",
        "- the bottle leaving the tray is the effect, not a cause, and the robot's reaction after t_obs is not a "
        "cause either.",
        "",
        "Answer with exactly one JSON object and nothing else (no markdown, no prose), in this format:",
        json.dumps(ANSWER_FORMAT, indent=2),
    ]
    return "\n\n".join([
        "You explain anomalies of a mobile robot. You propose hypotheses; the robot's memory checks them "
        "against the recording and simulates the ones that survive.",
        describe_episode(episode, tbox_path),
        describe_mechanisms(tbox_path, with_costs=budget is not None),
        "## The robot (its self-model)\n\n" + self_model,
        "\n".join(task),
    ])


def feedback(problems: list[str]) -> str:
    """What the LLM is told when its answer is refused."""
    return ("Your answer was refused and nothing was simulated:\n"
            + "\n".join(f"- {problem}" for problem in problems)
            + "\nAnswer again with the whole JSON object, following the rules of the task.")


# --------------------------------------------------------------------------- #
# The generation, with retries
# --------------------------------------------------------------------------- #
@dataclass
class GenerationResult:
    ok: bool
    batch: dict[str, Any]
    batch_path: Path
    prompt_path: Path
    transcript_path: Path


def _shown(path: Path) -> str:
    """A path relative to the repository when it is inside it."""
    try:
        return str(path.resolve().relative_to(REPO))
    except ValueError:
        return str(path)


def generate_batch(episode: dict[str, Any], llm: LLM, *, model: str, output_dir: Path,
                   case_id: Optional[str] = None, budget: Optional[int] = DEFAULT_BUDGET,
                   max_attempts: int = DEFAULT_MAX_ATTEMPTS, self_model_path: Path = DEFAULT_SELF_MODEL,
                   tbox_path: str | Path = DEFAULT_TBOX, episode_path: Optional[str] = None,
                   trigger: Optional[dict] = None, log: Optional[Callable[[str], None]] = None) -> GenerationResult:
    """Ask the LLM for a layer-3a proposal over the episode and publish its layer-3b batch.

    Writes, in output_dir: {case_id}.json (the batch), {case_id}_prompt.md (the prompt) and
    {case_id}_transcript.json (every message of the conversation). If no answer passes the schema in
    max_attempts, the batch has status `error`, no hypotheses and the problems of each attempt.
    """
    log = log or (lambda message: None)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    case_id = case_id or f"generation_v2_{episode['episode_id']}_{stamp}"
    prompt = build_prompt(episode, read_self_model(self_model_path), budget, tbox_path)
    prompt_path = output_dir / f"{case_id}_prompt.md"
    prompt_path.write_text(prompt + "\n", encoding="utf-8")
    metadata = {"case_id": case_id, "model": model, "prompt_path": _shown(prompt_path), "episode_path": episode_path,
                "trigger": trigger, "tbox_path": tbox_path}

    messages = [{"role": "user", "content": prompt}]
    attempts: list[dict[str, Any]] = []
    errors: list[str] = []
    batch = None
    started = time.monotonic()
    for number in range(1, max_attempts + 1):
        attempt: dict[str, Any] = {"attempt": number, "model": model}
        attempts.append(attempt)
        clock = time.monotonic()
        try:
            answer, metrics = llm(messages)
        except Exception as error:          # the LLM was not reached: ask again, as it was
            attempt.update(status="llm_error", error=str(error), elapsed_seconds=round(time.monotonic() - clock, 3))
            errors.append(f"attempt {number}: the LLM failed: {error}")
            log(f"attempt {number}: the LLM failed: {error}")
            continue
        attempt.update(elapsed_seconds=round(time.monotonic() - clock, 3), **({"ollama_native": metrics} if metrics else {}))
        messages.append({"role": "assistant", "content": answer})
        try:
            proposal = _extract_json_object(answer)
            batch = publish_batch(episode, proposal, arm=ARM, budget=budget, attempts=number, **metadata)
        except HypothesisSchemaError as error:
            problems = error.problems
        except ValueError as error:          # _extract_json_object: no JSON object in the answer
            problems = [f"the answer is not a JSON object ({error})"]
        else:
            attempt["status"] = "success"
            log(f"attempt {number}: {len(batch['hypotheses'])} hypotheses published")
            break
        attempt.update(status="refused", problems=problems)
        errors.append(f"attempt {number}: " + "; ".join(problems))
        log(f"attempt {number} refused: " + "; ".join(problems))
        messages.append({"role": "user", "content": feedback(problems)})

    if batch is None:
        batch = error_batch(episode, errors, arm=ARM, budget=budget, attempts=len(attempts), **metadata)
    batch["generation_metrics"] = {"llm_elapsed_seconds": round(time.monotonic() - started, 3), "attempts": attempts}
    batch_path = output_dir / f"{case_id}.json"
    batch_path.write_text(json.dumps(batch, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    transcript_path = output_dir / f"{case_id}_transcript.json"
    transcript_path.write_text(json.dumps({"case_id": case_id, "model": model, "messages": messages},
                                          indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    return GenerationResult(ok=batch["status"] == "success", batch=batch, batch_path=batch_path,
                            prompt_path=prompt_path, transcript_path=transcript_path)
