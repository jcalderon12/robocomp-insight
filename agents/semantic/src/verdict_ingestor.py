"""Turns an accepted verdict into the episodic explanation triples for GraphDB:

    insight:Case_<case_id>  rdf:type               insight:AnomalyCase
    insight:Case_<case_id>  insight:concernsEvent  insight:Event_BottleLocationChange
    insight:Case_<case_id>  insight:explainedBy    insight:Event_<Intervention>

With generation v2, the episode graph records a simulation-supported explanation linking the
selected hypothesis, the simulation run and the observed representation change. It does not
instantiate a physical causal event or assert soma:causes. Legacy live-graph identifiers and
links stay available; their types now distinguish representation changes from explanations.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from rdflib import OWL, RDF, RDFS, Graph, Literal, URIRef

from src.case_rdf import hypothesis_iri, run_iri
from src.episode_rdf import DUL, INSIGHT as INSIGHT_TBOX, episode_namespace, observation_iri

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
    # When the nominal run already reproduces the effect, no hypothesis was told
    # apart from "nothing happened"; verdicts written before the simulator
    # abstained on its own still carry an accepted id, so check the flag here too.
    if verdict.get("nominal_effect_warning") or verdict.get("abstention_reason"):
        return VerdictIngestion(
            ok=True,
            case_id=case_id,
            reason=(
                "Verdict abstained: the nominal run reproduces the effect "
                f"({verdict.get('abstention_reason') or 'nominal_effect_warning'}); anomaly remains unexplained."
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
    for alternative in batch.get("hypotheses", []):
        if (alternative["hypothesis_id"] != accepted_hypothesis_id
                and alternative.get("status") in {"checked_not_simulable", "not_simulable"}
                and (alternative.get("checks", {}).get("coherence") or {}).get("passed", True)):
            graph.add((explanation, INSIGHT_TBOX.hasUnresolvedAlternative,
                       hypothesis_iri(batch, alternative["hypothesis_id"])))
    graph.add((ep.Episode, DUL.isSettingFor, explanation))
    graph.add((INSIGHT[f"{CASE_PREFIX}{batch['case_id']}"], EXPLAINED_BY, explanation))
    return graph
