"""Turns an accepted verdict into the episodic explanation triples for GraphDB:

    insight:Case_<case_id>  rdf:type               insight:AnomalyCase
    insight:Case_<case_id>  insight:concernsEvent  insight:Event_BottleLocationChange
    insight:Case_<case_id>  insight:explainedBy    insight:Event_<Intervention>

With generation v2 they go to the case's named graph, next to the episode (contracts 1.12; before,
to the live graph, which each case restores to the mirror of the working memory). The episode
graph also records a simulation-supported explanation linking the
selected hypothesis, the simulation run and the observed representation change. It does not
instantiate a physical causal event or assert soma:causes. Legacy live-graph identifiers and
links stay available; their types now distinguish representation changes from explanations.

Since contracts 1.13, verification_graph also keeps what the verification knows, accepted or not:
the probability of each simulated mechanism, the 90 % credible set and, on an abstention, which
alternatives remain standing and with which evidence of the episode. It never asserts a cause.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from rdflib import OWL, RDF, RDFS, Graph, Literal, URIRef

from src.case_rdf import (NOMINAL_ID, _decimal, _individual, _new_graph, _observations_by_property, case_iri,
                          hypothesis_iri, is_v2, run_iri)
from src.episode_contrast import DEFAULT_TBOX
from src.episode_rdf import DUL, INSIGHT as INSIGHT_TBOX, episode_namespace, observation_iri
from src.semantic_memory import CONTRACTS_VERSION, code_commit, tbox_version

from src.ontology_mapping import (
    ANOMALY_CASE_CLASS,
    CASE_PREFIX,
    CONCERNS_EVENT,
    EVENT_BOTTLE_LOCATION_CHANGE,
    EXPLAINED_BY,
    INSIGHT,
    INTERVENTION_EVENT_PREFIX,
)

Triple = tuple[str, str, str]

#: A coherent hypothesis with these statuses has no simulation realization: it stays unresolved.
NON_SIMULABLE = frozenset({"checked_not_simulable", "not_simulable"})
#: Abstentions in which the simulation says nothing about the causes (verdict.py).
UNINFORMATIVE_ABSTENTIONS = frozenset({"simulation_not_faithful", "no_record_in_interval"})


@dataclass(frozen=True)
class VerdictIngestion:
    ok: bool
    case_id: str = ""
    accepted_hypothesis_id: Optional[str] = None
    accepted_intervention: Optional[str] = None
    triples: frozenset = field(default_factory=frozenset)
    reason: str = ""


def _intervention_event_iri(cause_name: str) -> str:
    token = "".join(part.capitalize() for part in str(cause_name).split("_")) or "Unknown"
    return str(INSIGHT[f"{INTERVENTION_EVENT_PREFIX}{token}"])


def build_case_triples(cause_name: str, case_id: str) -> set[Triple]:
    case_iri = str(INSIGHT[f"{CASE_PREFIX}{case_id}"])
    event_iri = _intervention_event_iri(cause_name)
    effect_iri = str(EVENT_BOTTLE_LOCATION_CHANGE)
    return {
        (case_iri, str(RDF.type), str(ANOMALY_CASE_CLASS)),
        (case_iri, str(CONCERNS_EVENT), effect_iri),
        (case_iri, str(EXPLAINED_BY), event_iri),
        (event_iri, str(RDF.type), str(INSIGHT_TBOX.SimulationSupportedExplanation)),
        (effect_iri, str(RDF.type), str(INSIGHT_TBOX.ObservedAnomaly)),
    }


def case_triples_graph(triples) -> Graph:
    """The legacy consolidation triples (all IRIs) as a graph, to go to the case's named graph."""
    graph = Graph()
    for s, p, o in triples:
        graph.add((URIRef(s), URIRef(p), URIRef(o)))
    return graph


def load_verdict(verdict_path: str | Path) -> Optional[dict]:
    path = Path(verdict_path)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def ingest_verdict(verdict_path: str | Path) -> VerdictIngestion:
    verdict = load_verdict(verdict_path)
    if verdict is None:
        return VerdictIngestion(ok=False, reason=f"Verdict file '{verdict_path}' not readable.")

    case_id = str(verdict.get("case_id", ""))
    accepted_id = verdict.get("accepted_hypothesis_id")
    # The simulator abstained (since verdict 2.0: no hypothesis reached its probability threshold, the
    # nominal was the most probable, or the simulation was not faithful to the episode). Verdicts
    # written before the simulator abstained on its own carry the flag and still an accepted id.
    if verdict.get("nominal_effect_warning") or verdict.get("abstention_reason"):
        return VerdictIngestion(
            ok=True,
            case_id=case_id,
            reason=(
                f"Verdict abstained ({verdict.get('abstention_reason') or 'nominal_effect_warning: the nominal run reproduces the effect'}); "
                "anomaly remains unexplained."
            ),
        )
    if not accepted_id:
        return VerdictIngestion(
            ok=True,
            case_id=case_id,
            reason="No hypothesis accepted; anomaly remains unexplained.",
        )
    if not case_id:
        return VerdictIngestion(ok=False, reason="Verdict has no case_id.")

    accepted = next(
        (h for h in verdict.get("hypotheses", []) if h.get("hypothesis_id") == accepted_id),
        None,
    )
    if accepted is None:
        return VerdictIngestion(
            ok=False,
            case_id=case_id,
            reason=f"Accepted hypothesis '{accepted_id}' not found in verdict payload.",
        )

    cause_name = str(accepted.get("cause", {}).get("name", "")).strip()
    if not cause_name or cause_name == "none":
        return VerdictIngestion(
            ok=False,
            case_id=case_id,
            reason=f"Accepted hypothesis '{accepted_id}' has an invalid cause '{cause_name}'.",
        )

    return VerdictIngestion(
        ok=True,
        case_id=case_id,
        accepted_hypothesis_id=str(accepted_id),
        accepted_intervention=cause_name,
        triples=frozenset(build_case_triples(cause_name, case_id)),
    )


def mechanism_graph(batch: dict, accepted_hypothesis_id: str, episode: Optional[dict] = None) -> Optional[Graph]:
    """A simulation-supported explanation of a v2 case, never an asserted physical cause.

    The description selects the hypothesis (whose anchors remain hypothetical), links its run
    and explains the recorded anomaly. Non-simulable alternatives remain explicitly unresolved.
    None for an unknown or non-simulated hypothesis, or a batch without a v2 episode.
    """
    if str(batch.get("schema_version")) != "2.0" or not batch.get("episode"):
        return None
    hypothesis = next((h for h in batch.get("hypotheses", []) if h.get("hypothesis_id") == accepted_hypothesis_id), None)
    if hypothesis is None or not hypothesis.get("mechanism_iri") or hypothesis.get("status") != "to_simulate":
        return None
    ep = episode_namespace(batch["episode"]["id"])
    explanation = ep[f"Explanation_{accepted_hypothesis_id}"]
    graph = Graph()
    graph.bind("ep", ep)
    graph.bind("dul", DUL)
    graph.bind("insight", INSIGHT_TBOX)
    graph.add((explanation, RDF.type, OWL.NamedIndividual))
    graph.add((explanation, RDF.type, INSIGHT_TBOX.SimulationSupportedExplanation))
    graph.add((explanation, RDFS.label, Literal(hypothesis["title"])))
    graph.add((explanation, INSIGHT_TBOX.hypothesisId, Literal(accepted_hypothesis_id)))
    graph.add((explanation, DUL.isDescribedBy, URIRef(hypothesis["mechanism_iri"])))
    graph.add((explanation, INSIGHT_TBOX.selectedHypothesis, hypothesis_iri(batch, accepted_hypothesis_id)))
    graph.add((explanation, INSIGHT_TBOX.supportedBySimulation, run_iri(batch, accepted_hypothesis_id)))
    if episode is not None:
        graph.add((explanation, INSIGHT_TBOX.explainsObservation, observation_iri(episode)))
    for alternative in _non_simulable(batch):
        if alternative["hypothesis_id"] != accepted_hypothesis_id:
            graph.add((explanation, INSIGHT_TBOX.hasUnresolvedAlternative,
                       hypothesis_iri(batch, alternative["hypothesis_id"])))
    graph.add((ep.Episode, DUL.isSettingFor, explanation))
    graph.add((INSIGHT[f"{CASE_PREFIX}{batch['case_id']}"], EXPLAINED_BY, explanation))
    return graph


def _non_simulable(batch: dict) -> list[dict]:
    """The coherent hypotheses of the batch that have no simulation realization."""
    return [h for h in batch.get("hypotheses", [])
            if h.get("status") in NON_SIMULABLE and (h.get("checks", {}).get("coherence") or {}).get("passed", True)]


def _abstained(verdict: dict) -> bool:
    """As ingest_verdict: an abstention reason, the old nominal warning, or nothing accepted."""
    return bool(verdict.get("nominal_effect_warning") or verdict.get("abstention_reason")
                or not verdict.get("accepted_hypothesis_id"))


def verification_graph(batch: dict, verdict: dict, episode: Optional[dict] = None) -> Optional[Graph]:
    """What the verification knows of a v2 case, accepted or not (contracts 1.13); never a cause.

    Always: the decision and the thresholds the verdict took it with, the most probable run, the
    smallest ratio to the nominal among the simulated causes, and the probability of each simulated
    mechanism (the nominal included) with whether it is in the credible set. Each run's own
    probability and ratio are in case_rdf.simulation_graph.

    On an abstention, the alternatives that remain standing: every simulated hypothesis (none was
    refuted) and every coherent one without a simulation realization. When every simulated cause
    leaves more than the verdict's limit of the nominal's discrepancy, the outcome is also a
    NoSimulatedCauseExplainsBetter. An accepted verdict links its explanation instead.

    The alternatives (on acceptance, the unresolved ones of mechanism_graph) are linked to the episode
    observations in which an informing rule saw their trace. None for a batch without a v2 episode.
    """
    if not is_v2(batch):
        return None
    ep = episode_namespace(batch["episode"]["id"])
    in_batch = {h["hypothesis_id"]: h for h in batch.get("hypotheses", [])}
    entries = [e for e in verdict.get("hypotheses", []) if e.get("hypothesis_id") == NOMINAL_ID
               or e.get("hypothesis_id") in in_batch]
    causes = [e for e in entries if e["hypothesis_id"] != NOMINAL_ID]
    method = verdict.get("method") or {}
    abstained = _abstained(verdict)

    graph = _new_graph(batch)
    outcome = _individual(graph, ep.Verification, INSIGHT_TBOX.VerificationOutcome, f"verification of {batch['case_id']}")
    graph.add((case_iri(batch["case_id"]), DUL.isSettingFor, outcome))
    graph.add((outcome, INSIGHT_TBOX.verdictDecision, Literal("abstained" if abstained else "accepted")))
    # The outcome may be written after the rest of the case (a re-consolidation): its own provenance.
    for prop, value in ((INSIGHT_TBOX.tboxVersion, tbox_version(DEFAULT_TBOX)),
                        (INSIGHT_TBOX.contractsVersion, CONTRACTS_VERSION), (INSIGHT_TBOX.codeCommit, code_commit())):
        graph.add((outcome, prop, Literal(value)))
    ratios = [e["ratio_to_nominal"] for e in causes if e.get("ratio_to_nominal") is not None]
    for prop, value in ((INSIGHT_TBOX.acceptanceThreshold, method.get("p_min")),
                        (INSIGHT_TBOX.explanationRatioMax, method.get("explanation_ratio_max")),
                        (INSIGHT_TBOX.credibleMass, (verdict.get("credible_set") or {}).get("mass")),
                        (INSIGHT_TBOX.explainedRatio, verdict.get("explained_ratio")),
                        (INSIGHT_TBOX.bestCauseRatio, min(ratios) if ratios else None)):
        if value is not None:
            graph.add((outcome, prop, _decimal(value)))
    most_probable = verdict.get("most_probable_hypothesis_id")
    if any(e["hypothesis_id"] == most_probable for e in entries):
        graph.add((outcome, INSIGHT_TBOX.mostProbableRun, run_iri(batch, most_probable)))

    # The probability of each mechanism: the sum over the runs of its hypotheses.
    credible = {m.get("mechanism") for m in (verdict.get("credible_set") or {}).get("mechanisms", [])}
    by_mechanism: dict[str, list[dict]] = {}
    for entry in entries:
        if entry.get("posterior") is not None:
            name = entry.get("mechanism") or in_batch.get(entry["hypothesis_id"], {}).get("mechanism") or "unknown"
            by_mechanism.setdefault(str(name), []).append(entry)
    for name, group in by_mechanism.items():
        probability = _individual(graph, ep[f"Probability_{name}"], INSIGHT_TBOX.MechanismProbability,
                                  f"probability of {name}")
        graph.add((outcome, INSIGHT_TBOX.assignsProbability, probability))
        graph.add((probability, INSIGHT_TBOX.mechanismName, Literal(name)))
        graph.add((probability, INSIGHT_TBOX.posteriorProbability,
                   _decimal(round(sum(float(e["posterior"]) for e in group), 6))))
        if verdict.get("credible_set") is not None:
            graph.add((probability, INSIGHT_TBOX.inCredibleSet, Literal(name in credible)))
        for entry in group:
            graph.add((probability, INSIGHT_TBOX.aggregatesRun, run_iri(batch, entry["hypothesis_id"])))
        mechanism = next((in_batch[e["hypothesis_id"]].get("mechanism_iri") for e in group
                          if in_batch.get(e["hypothesis_id"], {}).get("mechanism_iri")), None)
        if mechanism:
            graph.add((probability, INSIGHT_TBOX.forMechanism, URIRef(mechanism)))

    non_simulable = _non_simulable(batch)
    if abstained:
        limit = method.get("explanation_ratio_max")
        if (limit is not None and ratios and min(ratios) > float(limit)
                and str(verdict.get("abstention_reason")) not in UNINFORMATIVE_ABSTENTIONS):
            graph.add((outcome, RDF.type, INSIGHT_TBOX.NoSimulatedCauseExplainsBetter))
        simulated = [in_batch[e["hypothesis_id"]] for e in causes]
        for prop, hypotheses in ((INSIGHT_TBOX.openSimulatedAlternative, simulated),
                                 (INSIGHT_TBOX.openNonSimulableAlternative, non_simulable)):
            for hypothesis in hypotheses:
                graph.add((outcome, prop, hypothesis_iri(batch, hypothesis["hypothesis_id"])))
        alternatives = simulated + non_simulable
    else:
        accepted_id = verdict.get("accepted_hypothesis_id")
        if mechanism_graph(batch, accepted_id, episode) is not None:
            graph.add((outcome, INSIGHT_TBOX.selectsExplanation, ep[f"Explanation_{accepted_id}"]))
        alternatives = [h for h in non_simulable if h["hypothesis_id"] != accepted_id]

    # The evidence: an informing rule saw, in the episode, the trace the alternative's mechanism entails.
    observations = _observations_by_property(episode) if episode is not None else {}
    for hypothesis in alternatives:
        for entry in (hypothesis.get("checks") or {}).get("contrast") or []:
            if entry.get("verdict") == "trace_seen":
                for prop in entry.get("reads") or []:
                    if prop in observations:
                        graph.add((hypothesis_iri(batch, hypothesis["hypothesis_id"]),
                                   INSIGHT_TBOX.supportedByObservation, ep[observations[prop]]))
    return graph
