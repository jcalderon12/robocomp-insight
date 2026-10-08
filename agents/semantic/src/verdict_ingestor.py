"""Turns an accepted verdict into the episodic explanation triples for GraphDB:

    insight:Case_<case_id>  rdf:type               insight:AnomalyCase
    insight:Case_<case_id>  insight:concernsEvent  insight:Event_BottleLocationChange
    insight:Case_<case_id>  insight:explainedBy    insight:Event_<Intervention>

With the generation v2 (contract 3), the verified mechanism is consolidated too, in the named
graph of the episode (mechanism_graph): the accepted hypothesis, found in the semantic agent's own
batch, becomes a verified cause described by its mechanism and located at its anchors. The triples
above stay as they were: other consumers read them.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from rdflib import OWL, RDF, RDFS, Graph, Literal, URIRef

from src.case_rdf import hypothesis_iri, run_iri
from src.episode_rdf import DUL, INSIGHT as INSIGHT_TBOX, SOMA, episode_namespace

from src.ontology_mapping import (
    ANOMALY_CASE_CLASS,
    CASE_PREFIX,
    CONCERNS_EVENT,
    EVENT_BOTTLE_LOCATION_CHANGE,
    EVENT_CLASS,
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
        (event_iri, str(RDF.type), str(EVENT_CLASS)),
        (effect_iri, str(RDF.type), str(EVENT_CLASS)),
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
    """The verified mechanism of an accepted hypothesis of a v2 batch, for the episode's named graph.

        ep:Cause_<id>  a insight:VerifiedCause ; rdfs:label <title from the mechanism> ;
                       insight:hypothesisId <id> ; dul:isDescribedBy <mechanism> ;
                       dul:hasLocation ep:<segment> ; dul:hasTimeInterval ep:<interval> ;
                       dul:hasSetting ep:Hypothesis_<id> ; insight:verifiedBy ep:Run_<id> .
        ep:Episode     dul:isSettingFor ep:Cause_<id> .
        insight:Case_<case_id>  insight:explainedBy  ep:Cause_<id> .

    With the episode, the cause also causes its accident (soma:causes): the bottle fell because of
    it. The hypothesis and the run are those of case_rdf (contracts 1.8). The title is the one
    written from the mechanism and its anchors, not the LLM's. None if the batch is not a v2 one or
    does not hold the hypothesis.
    """
    if str(batch.get("schema_version")) != "2.0" or not batch.get("episode"):
        return None
    hypothesis = next((h for h in batch.get("hypotheses", []) if h.get("hypothesis_id") == accepted_hypothesis_id), None)
    if hypothesis is None or not hypothesis.get("mechanism_iri"):
        return None
    ep = episode_namespace(batch["episode"]["id"])
    cause = ep[f"Cause_{accepted_hypothesis_id}"]
    graph = Graph()
    graph.bind("ep", ep)
    graph.bind("dul", DUL)
    graph.bind("insight", INSIGHT_TBOX)
    graph.add((cause, RDF.type, OWL.NamedIndividual))
    graph.add((cause, RDF.type, INSIGHT_TBOX.VerifiedCause))
    graph.add((cause, RDFS.label, Literal(hypothesis["title"])))
    graph.add((cause, INSIGHT_TBOX.hypothesisId, Literal(accepted_hypothesis_id)))
    graph.add((cause, DUL.isDescribedBy, URIRef(hypothesis["mechanism_iri"])))
    anchors = hypothesis.get("anchors") or {}
    if anchors.get("segment"):
        graph.add((cause, DUL.hasLocation, ep[anchors["segment"]]))
    if anchors.get("interval"):
        graph.add((cause, DUL.hasTimeInterval, ep[anchors["interval"]]))
    graph.add((cause, DUL.hasSetting, hypothesis_iri(batch, accepted_hypothesis_id)))
    graph.add((cause, INSIGHT_TBOX.verifiedBy, run_iri(batch, accepted_hypothesis_id)))
    if episode is not None:
        graph.add((cause, SOMA.causes, ep[episode["accident"]["id"]]))
    graph.add((ep.Episode, DUL.isSettingFor, cause))
    graph.add((INSIGHT[f"{CASE_PREFIX}{batch['case_id']}"], EXPLAINED_BY, cause))
    return graph
