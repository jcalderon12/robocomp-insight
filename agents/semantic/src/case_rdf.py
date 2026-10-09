"""The case in RDF: what the memory decided about each hypothesis, and what the simulation made of it
(contracts 1.11).

Both graphs go to the episode's named graph, next to the episode (episode_rdf.py), so that the
semantic memory answers by itself which hypotheses the LLM proposed, why each was kept or dropped,
and which explanation the simulator supported:

  * decision_graph(batch, episode), when the batch is published: the anomaly case, the request to
    the LLM, and each hypothesis with its mechanism, anchors, kinds, status and the checks applied
    to it (with the observations they read);
  * simulation_graph(batch, verdict), when the verdict on that batch arrives: one simulation run
    per simulated cause and one for the nominal replay, with what the verdict made of them.

The supported explanation is verdict_ingestor.mechanism_graph. The vocabulary is the INSIGHT TBox;
nothing here changes the batch, the verdict or what the live graph receives.
"""

from __future__ import annotations

from decimal import Decimal
from functools import lru_cache
from pathlib import Path
from typing import Any, Optional

from rdflib import OWL, RDF, RDFS, XSD, Graph, Literal, URIRef

from src.episode_contrast import DEFAULT_TBOX, tbox_graph
from src.episode_rdf import DUL, INSIGHT, INST, episode_namespace, observation_iri

NOMINAL_ID = "__nominal__"
CASE_PREFIX = "Case_"

#: What each cause reports of its best repetition (best_generated_instances), as named quantities.
SAMPLED_LISTS = {
    "external_force": ("random_range_coordinates", ("force_x_n", "force_y_n", "force_z_n")),
    "friction": ("random_uniform_range", ("friction_mu",)),
    "wheel": ("random_uniform_range", ("stop_time_fraction",)),
}


def case_iri(case_id: str) -> URIRef:
    return INST[f"{CASE_PREFIX}{case_id}"]


def hypothesis_iri(batch: dict, hypothesis_id: str) -> URIRef:
    return episode_namespace(batch["episode"]["id"])[f"Hypothesis_{hypothesis_id}"]


def run_iri(batch: dict, hypothesis_id: str) -> URIRef:
    name = "nominal" if hypothesis_id == NOMINAL_ID else hypothesis_id
    return episode_namespace(batch["episode"]["id"])[f"Run_{name}"]


@lru_cache(maxsize=4)
def _tbox_terms(tbox_path: str | Graph) -> tuple[dict[str, URIRef], frozenset[URIRef]]:
    """The episode checks by rule id, and the kinds the TBox declares."""
    tbox = tbox_graph(tbox_path)
    checks = {str(rule_id): check for check, rule_id in tbox.subject_objects(INSIGHT.ruleId)}
    kinds = frozenset(tbox.subjects(RDF.type, INSIGHT.Kind))
    return checks, kinds


def _decimal(value: float) -> Literal:
    return Literal(Decimal(repr(float(value))), datatype=XSD.decimal)


def _new_graph(batch: dict) -> Graph:
    graph = Graph()
    for prefix, namespace in (("dul", DUL), ("insight", INSIGHT), ("inst", INST),
                              ("ep", episode_namespace(batch["episode"]["id"])), ("owl", OWL)):
        graph.bind(prefix, namespace)
    return graph


def _individual(graph: Graph, iri: URIRef, rdf_type: URIRef, label: str) -> URIRef:
    graph.add((iri, RDF.type, OWL.NamedIndividual))
    graph.add((iri, RDF.type, rdf_type))
    graph.add((iri, RDFS.label, Literal(label)))
    return iri


def _observations_by_property(episode: dict) -> dict[str, str]:
    """property -> id of its observation in the episode; the system reaction is left out (no check reads it)."""
    return {entry["property"]: entry["id"] for entry in episode.get("evidence", []) if not entry.get("is_system_reaction")}


def is_v2(batch: Optional[dict]) -> bool:
    return bool(batch) and str(batch.get("schema_version")) == "2.0" and bool(batch.get("episode"))


def decision_graph(batch: dict, episode: dict, batch_path: Optional[str] = None,
                   tbox_path: str | Path | Graph = DEFAULT_TBOX) -> Graph:
    """The anomaly case, the request to the LLM, and what the memory decided about each hypothesis."""
    if not is_v2(batch):
        raise ValueError("the decision trail needs a v2 batch (schema 2.0, with its episode)")
    checks, kinds = _tbox_terms(tbox_path if isinstance(tbox_path, Graph) else str(tbox_path))
    ep = episode_namespace(batch["episode"]["id"])
    graph = _new_graph(batch)
    observations = _observations_by_property(episode)
    anchors_in_episode = {entry["id"] for entry in episode.get("segments", []) + episode.get("intervals", [])}

    case = _individual(graph, case_iri(batch["case_id"]), INST.AnomalyCase, batch["case_id"])
    graph.add((case, INST.concernsEvent, observation_iri(episode)))
    graph.add((ep.Episode, DUL.isSettingFor, case))

    generation = _individual(graph, ep.Generation, INSIGHT.HypothesisGeneration, "hypothesis generation")
    graph.add((generation, INSIGHT.llmModel, Literal(str(batch.get("model", "")))))
    graph.add((generation, INSIGHT.llmAttempts, Literal(int(batch.get("attempts") or 0), datatype=XSD.nonNegativeInteger)))
    graph.add((generation, INSIGHT.batchStatus, Literal(str(batch.get("status", "")))))
    if batch_path:
        graph.add((generation, INSIGHT.batchPath, Literal(str(batch_path), datatype=XSD.anyURI)))
    graph.add((case, DUL.isSettingFor, generation))

    for hypothesis in batch.get("hypotheses", []):
        hid = hypothesis["hypothesis_id"]
        h = _individual(graph, hypothesis_iri(batch, hid), INSIGHT.Hypothesis, hypothesis["title"])
        graph.add((case, DUL.isSettingFor, h))
        graph.add((h, INSIGHT.hypothesisId, Literal(hid)))
        graph.add((h, INSIGHT.rank, Literal(int(hypothesis["rank"]), datatype=XSD.positiveInteger)))
        graph.add((h, INSIGHT.proposedIn, generation))
        if hypothesis.get("rationale"):
            graph.add((h, INSIGHT.llmRationale, Literal(hypothesis["rationale"])))
        if hypothesis.get("mechanism_iri"):
            graph.add((h, DUL.satisfies, URIRef(hypothesis["mechanism_iri"])))
        if hypothesis.get("new_mechanism_description"):
            graph.add((h, INSIGHT.newMechanismDescription, Literal(hypothesis["new_mechanism_description"])))
        for anchor in (hypothesis.get("anchors") or {}).values():
            if anchor in anchors_in_episode:
                graph.add((h, INSIGHT.anchoredTo, ep[anchor]))
        for name, value in (hypothesis.get("qualitative_parameters") or {}).items():
            kind = INSIGHT[f"Kind_{name}_{value}"]
            if kind in kinds:
                graph.add((h, DUL.isClassifiedBy, kind))
        graph.add((h, DUL.isClassifiedBy, INSIGHT[f"Status_{hypothesis['status']}"]))
        if hypothesis.get("untestable_reason"):
            graph.add((h, INSIGHT.statusReason, Literal(hypothesis["untestable_reason"])))

        coherence = (hypothesis.get("checks") or {}).get("coherence") or {}
        applied = list(coherence.get("preconditions") or []) + list((hypothesis.get("checks") or {}).get("contrast") or [])
        for entry in applied:
            outcome = _individual(graph, ep[f"Check_{hid}_{entry['check']}"], INSIGHT.CheckOutcome,
                                  f"{entry['check']} on {hid}")
            graph.add((h, INSIGHT.checkedBy, outcome))
            if entry["check"] in checks:
                graph.add((outcome, INSIGHT.appliesCheck, checks[entry["check"]]))
            graph.add((outcome, INSIGHT.checkVerdict, Literal(entry["verdict"])))
            if entry.get("reason"):
                graph.add((outcome, RDFS.comment, Literal(entry["reason"])))
            for prop in entry.get("reads") or []:
                if prop in observations:
                    graph.add((outcome, INSIGHT.readsObservation, ep[observations[prop]]))
        if not coherence.get("passed", True) and not coherence.get("preconditions"):
            # Incoherent without a precondition: an anchor or a kind the episode or the TBox lacks.
            outcome = _individual(graph, ep[f"Check_{hid}_coherence"], INSIGHT.CheckOutcome, f"coherence of {hid}")
            graph.add((h, INSIGHT.checkedBy, outcome))
            graph.add((outcome, INSIGHT.checkVerdict, Literal("fails")))
            if coherence.get("reason"):
                graph.add((outcome, RDFS.comment, Literal(coherence["reason"])))
    return graph


def _sampled_values(cause_name: str, instances: dict[str, Any]) -> list[tuple[str, float]]:
    """The named numbers a cause reports of its best repetition."""
    if isinstance(instances.get("sample"), dict):
        return [(str(k), float(v)) for k, v in instances["sample"].items() if isinstance(v, (int, float))]
    key, names = SAMPLED_LISTS.get(cause_name, (None, ()))
    values = instances.get(key) if key else None
    if not values:
        return []
    first = values[0] if isinstance(values[0], (list, tuple)) else values
    return [(name, float(value)) for name, value in zip(names, first) if isinstance(value, (int, float))]


def simulation_graph(batch: dict, verdict: dict, verdict_path: Optional[str] = None) -> Graph:
    """One simulation run per simulated cause and one for the nominal, with what the verdict made of them."""
    if not is_v2(batch):
        raise ValueError("the simulation runs need a v2 batch (schema 2.0, with its episode)")
    ep = episode_namespace(batch["episode"]["id"])
    graph = _new_graph(batch)
    case = case_iri(batch["case_id"])
    in_batch = {h["hypothesis_id"] for h in batch.get("hypotheses", [])}
    if verdict_path:
        graph.add((case, INSIGHT.verdictPath, Literal(str(verdict_path), datatype=XSD.anyURI)))
    if verdict.get("abstention_reason"):
        graph.add((case, INSIGHT.abstentionReason, Literal(str(verdict["abstention_reason"]))))

    for entry in verdict.get("hypotheses", []):
        hid = entry.get("hypothesis_id")
        if hid != NOMINAL_ID and hid not in in_batch:
            continue
        nominal = hid == NOMINAL_ID
        run = _individual(graph, run_iri(batch, hid), INSIGHT.SimulationRun,
                          "nominal run" if nominal else f"simulation of {hid}")
        graph.add((case, DUL.isSettingFor, run))
        graph.add((run, INSIGHT.isNominal, Literal(nominal)))
        if not nominal:
            graph.add((run, INSIGHT.testsHypothesis, hypothesis_iri(batch, hid)))
        cause_name = str((entry.get("cause") or {}).get("name", ""))
        if cause_name:
            graph.add((run, INSIGHT.causeName, Literal(cause_name)))
        for key, prop in (("repetitions", INSIGHT.repetitions), ("mistimed_effects", INSIGHT.mistimedEffects)):
            if entry.get(key) is not None:
                graph.add((run, prop, Literal(int(entry[key]), datatype=XSD.nonNegativeInteger)))
        for key, prop in (("effect_rate", INSIGHT.effectRate), ("median_timing_error_s", INSIGHT.medianTimingErrorS)):
            if entry.get(key) is not None:
                graph.add((run, prop, _decimal(entry[key])))
        for key, prop in (("passes_rule", INSIGHT.passesRule), ("accepted", INSIGHT.acceptedBySimulation)):
            if entry.get(key) is not None:
                graph.add((run, prop, Literal(bool(entry[key]))))
        for name, value in _sampled_values(cause_name, entry.get("best_generated_instances") or {}):
            sample = _individual(graph, ep[f"Run_{'nominal' if nominal else hid}_{name}"], INSIGHT.SampledValue, name)
            graph.add((sample, INSIGHT.quantityName, Literal(name)))
            graph.add((sample, DUL.hasDataValue, _decimal(value)))
            graph.add((run, DUL.hasRegion, sample))
    return graph
