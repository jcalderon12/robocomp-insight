"""Turns an accepted verdict into the episodic explanation triples for GraphDB:

    insight:Case_<case_id>  rdf:type               insight:AnomalyCase
    insight:Case_<case_id>  insight:concernsEvent  insight:Event_BottleLocationChange
    insight:Case_<case_id>  insight:explainedBy    insight:Event_<Intervention>
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from rdflib import RDF

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
