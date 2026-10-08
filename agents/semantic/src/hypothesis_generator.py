"""Case-derived prompts in front of the deterministic hypothesis pipeline.

The prompt gives the LLM, once each:

  * the episode (contract 1) in readable form: what happened, the phases, the path segments and
    the time intervals a hypothesis may be anchored to, and the evidence, each property explained
    with its comment in the TBox. The robot's reaction after the observation is not offered as evidence
    nor as an anchor;
  * the mechanisms selected by the TBox's explanation profiles: id, description, anchors, qualitative parameters
    (kinds only: the magnitudes are the memory's), precondition, the properties on which they leave
    a trace, whether they can be simulated and what they cost;
  * the robot's self-model (description.md).

The observed delta, entity roles and selected profiles are saved in context_summary. No physical
event is assumed from a structural retraction; unknown profiles never inherit a scenario's catalog.
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

from src.episode_contrast import DEFAULT_TBOX
from src.explanation_context import build_explanation_context, render_context_template
from src.hypothesis_pipeline import (
    DEFAULT_BUDGET, DEFAULT_CATALOG, NEW_MECHANISM, HypothesisSchemaError,
    error_batch, load_vocabulary, publish_batch, simulation_cost_description,
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
    if value is None:
        return "unknown"
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


def describe_episode(episode: dict[str, Any], tbox_path: str | Path = DEFAULT_TBOX,
                     *, context: Optional[dict[str, Any]] = None) -> str:
    """Render recorded facts and symbolic anchors without assuming a particular physical event."""
    context = context if context is not None else build_explanation_context(
        episode, Graph().parse(str(tbox_path), format="turtle"))
    properties = observable_properties(tbox_path)
    clock, frame = episode.get("time") or {}, episode.get("frame") or {}
    observed_at = context["observed_at_s"]
    intervals = {i["id"]: i for i in episode.get("intervals", [])}
    reaction_intervals = {p.get("interval") for p in episode.get("phases", []) if p.get("is_system_reaction")}
    phases = [p for p in episode.get("phases", []) if not p.get("is_system_reaction")]
    support = episode.get("support") or {}
    event = episode.get("observation") or episode.get("accident") or {}
    lines = [f"## The episode {episode.get('episode_id', context['observation_id'])}", "",
             f"Reference frame: {frame.get('name', 'not recorded')}. Position units: "
             f"{frame.get('units', 'not recorded')}. Heading convention: {frame.get('heading', 'not recorded')}.",
             f"Times in seconds; origin: {clock.get('origin', 'not recorded')}.",
             "Identifiers are recorded labels; their names do not establish physical events.", "",
             "### Observed discrepancy",
             f"- {context['observation_id']}: {context['summary']}; observation time = {_n(observed_at)} s."]
    for change in context["changes"]:
        lines.append(f"- Recorded change ({change['operation']}): subject={change.get('subject', 'unknown')}; "
                     f"relation={change.get('predicate', 'unknown')}; object={change.get('object', 'unknown')}.")
    if context["mission"] is not None:
        mission = context["mission"]
        lines.append("- Recorded mission context: " + (mission if isinstance(mission, str) else
                     json.dumps(mission, ensure_ascii=False)) + ".")
    if episode.get("entities"):
        lines.append("- Recorded entity roles: " + "; ".join(
            f"{role}={entity}" for role, entity in episode["entities"].items()) + ".")
    lines.append("- Applicable ontology profiles: " + (", ".join(context["profile_labels"]) or "none declared") + ".")
    lines.extend(f"- {note}" for note in context["profile_notes"])
    lines.extend(f"- {note}" for note in context["selection_notes"])
    lines.extend(f"- Unknown: {note}" for note in context["unknowns"])
    pose = event.get("robot_pose") or {}
    if pose:
        lines.append(f"- Recorded observer pose at the observation: position={pose.get('xy', 'unknown')}, "
                     f"heading={_n(pose.get('heading_deg'))} deg. This does not locate the affected entity.")
    if support:
        lines.append(f"- Recorded support estimate {support.get('id', 'unnamed')}: "
                     f"{support.get('supporter', 'unknown')} supporting {support.get('supported', 'unknown')}, "
                     f"represented over [{_n(support.get('start_s'))}, {_n(support.get('end_s'))}] s. "
                     "The end of this estimate is not an independently measured end of physical support.")
    lines += ["- The cause of the discrepancy is not established. Marked system reactions are excluded "
              "as candidate causes and anchors.", "", "### Recorded phases before the observation"]
    for phase in phases:
        span = intervals.get(phase.get("interval"), {})
        if observed_at is not None and span.get("start_s") is not None and span["start_s"] >= observed_at:
            continue
        details = []
        if phase.get("mean_speed_mps") is not None:
            details.append(f"mean speed {_n(phase['mean_speed_mps'])} m/s")
        if phase.get("setpoint_mps"):
            details.append(f"commanded {_n(phase['setpoint_mps'][0])}-{_n(phase['setpoint_mps'][1])} m/s")
        if phase.get("heading_deg_range"):
            details.append(f"heading from {_n(phase['heading_deg_range'][0])} to {_n(phase['heading_deg_range'][1])} deg")
        lines.append(f"- {phase['id']} ({phase.get('kind', 'unspecified')}), {phase.get('interval', 'unknown')} = "
                     f"[{_n(span.get('start_s'))}, {_n(span.get('end_s'))}] s: " + ", ".join(details) + ".")
    lines += ["", "### Recorded path segments (anchors for a place: `segment`)"]
    for segment in sorted(episode.get("segments", []), key=lambda item: item.get("order_back_from_fall", 0)):
        if segment.get("interval") in reaction_intervals:
            continue
        span = intervals.get(segment.get("interval"), {})
        if observed_at is not None and span.get("start_s") is not None and span["start_s"] >= observed_at:
            continue
        lines.append(f"- {segment['id']}: recorded path from {segment.get('from_xy', 'unknown')} to "
                     f"{segment.get('to_xy', 'unknown')}, heading {_n(segment.get('heading_deg'))} deg, "
                     f"mean speed {_n(segment.get('mean_speed_mps'))} m/s, during "
                     f"{segment.get('interval', 'unknown')} = [{_n(span.get('start_s'))}, {_n(span.get('end_s'))}] s.")
    lines += ["", "### Time intervals (anchors for a time: `interval`)"]
    for interval_id, span in intervals.items():
        if interval_id in reaction_intervals or span.get("is_system_reaction"):
            continue
        if observed_at is not None and span.get("start_s") is not None and span["start_s"] >= observed_at:
            continue
        lines.append(f"- {interval_id} = [{_n(span.get('start_s'))}, {_n(span.get('end_s'))}] s.")
    lines += ["", "### Evidence (recorded measurements; baseline values refer to their recorded reference)"]
    for entry in episode.get("evidence", []):
        if entry.get("is_system_reaction") or entry["property"] == "reaction_onset":
            continue
        lines.append(_evidence_line(entry, properties))
    if any(entry.get("at_s", observed_at or 0) > (observed_at or 0)
           for entry in episode.get("evidence", []) if not entry.get("is_system_reaction")
           and entry.get("at_s") is not None):
        lines.append("- Later observations describe state after the discrepancy; they are not earlier causes.")
    if clock.get("episode_length_s") is not None:
        lines.append(f"- Recording available through {_n(clock['episode_length_s'])} s. Lack of a later "
                     "observation refers only to this recorded interval.")
    return "\n".join(lines)


def describe_mechanisms(tbox_path: str | Path = DEFAULT_TBOX, catalog: Optional[InterventionCatalog] = None,
                        with_costs: bool = True, context: Optional[dict[str, Any]] = None) -> str:
    """The active vocabulary, with role-bound descriptions and optional grounding costs."""
    catalog = catalog or InterventionCatalog.from_file(DEFAULT_CATALOG)
    profiles = tuple(context["profile_ids"]) if context is not None else None
    vocabulary = load_vocabulary(tbox_path, profiles)
    lines = ["## The mechanisms", "",
             "These are candidate mechanisms declared applicable to the observed change by the ontology. "
             "Their descriptions are hypotheses, not observations. Use their ids. Parameters are qualitative "
             "kinds only; the memory supplies numerical magnitudes and simulation ranges.", ""]
    if not vocabulary:
        lines += ["No applicable mechanisms are declared for this change. Do not borrow mechanisms from "
                  "a different case. An unlisted proposal is kept as `new` and is not simulated.", ""]
    for mechanism_id, terms in vocabulary.items():
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
        description = render_context_template(terms.prompt_description, context or {})
        precondition = " ".join(render_context_template(c.comment, context or {})
                                for c in mechanism.preconditions) or "none."
        lines += [f"- {mechanism_id}: {description}",
                  f"  Anchors: {', '.join(mechanism.anchors) or 'none'}. Parameters: {parameters}.",
                  f"  Precondition: {precondition}",
                  f"  Leaves a trace on: {', '.join(terms.trace) or 'nothing the recording measures'}.",
                  f"  Simulable: {simulable}" + (f" Cost: {simulation_cost_description(mechanism_id, terms, catalog)}."
                                                 if with_costs else "")]
    lines.append(f"- {NEW_MECHANISM}: a mechanism that is not in this list. Describe it in new_mechanism_description; "
                 "its anchors are optional. It is recorded as unlisted and is not simulated; "
                 "it does not modify the ontology.")
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
                 tbox_path: str | Path = DEFAULT_TBOX, *, trigger: Optional[dict[str, Any]] = None,
                 context: Optional[dict[str, Any]] = None) -> str:
    context = context if context is not None else build_explanation_context(
        episode, Graph().parse(str(tbox_path), format="turtle"), trigger)
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
        f"Explain the observed discrepancy {context['observation_id']} described above. "
        "Propose candidate mechanisms as hypotheses written in symbols:",
        "- each hypothesis names one mechanism id from the list (or `new`), the anchors that mechanism takes "
        "(segment and interval ids from the episode), and its qualitative parameters. Every parameter of the "
        "mechanism is required, with one of its listed values;",
        "- never write a number: no coordinates, times, fractions, forces nor probabilities. The memory turns "
        "the symbols into numbers;",
        "- read the evidence: a hypothesis should explain what the recording shows;",
        "- distinguish observations, the robot's previous estimates and hypothetical physical events. "
        "Do not turn a change in the robot's representation into an observed physical event;",
        "- missing observations are unknown. Absence of redetection in a finite recording is not proof "
        "of a physical displacement, and redetection alone does not establish continuous support;",
        f"- order the hypotheses from the most to the least plausible; propose at most {MAX_HYPOTHESES}. {simulated}",
        "- propose a mechanism that explains the evidence even if it cannot be simulated;",
        "- the observed discrepancy is what needs explanation, not a cause. System reactions after the "
        "observation must not be used as earlier causes.",
        "",
        "Answer with exactly one JSON object and nothing else (no markdown, no prose), in this format:",
        json.dumps(ANSWER_FORMAT, indent=2),
    ]
    return "\n\n".join([
        "You explain anomalies of a mobile robot. You propose hypotheses; the robot's memory checks them "
        "against the recording and simulates the ones that survive.",
        describe_episode(episode, tbox_path, context=context),
        describe_mechanisms(tbox_path, with_costs=budget is not None, context=context),
        "## The robot (its self-model)\n\nThis supplied description specifies capabilities and prior model "
        "information; it does not establish what occurred in this episode.\n\n" + self_model,
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
    context = build_explanation_context(episode, Graph().parse(str(tbox_path), format="turtle"), trigger)
    profiles = tuple(context["profile_ids"])
    prompt = build_prompt(episode, read_self_model(self_model_path), budget, tbox_path, context=context)
    prompt_path = output_dir / f"{case_id}_prompt.md"
    prompt_path.write_text(prompt + "\n", encoding="utf-8")
    metadata = {"case_id": case_id, "model": model, "prompt_path": _shown(prompt_path), "episode_path": episode_path,
                "trigger": trigger, "tbox_path": tbox_path,
                "context_summary": {"explanation_context": context,
                                    "mechanism_ids": list(load_vocabulary(tbox_path, profiles))}}

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
            batch = publish_batch(episode, proposal, arm=ARM, budget=budget, attempts=number,
                                  profiles=profiles, **metadata)
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
