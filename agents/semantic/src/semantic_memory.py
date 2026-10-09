"""The case as the semantic memory holds it: what the generation reads (plan_memoria_semantica.md, change B).

The prompt, the active vocabulary, the validation of the answer, the contrast with the recording,
the grounding and the anchored enumeration read one snapshot of the semantic memory: the TBox's
named graph and the case's (the episode's) named graph, read together once.

  * In production the agent writes the case to GraphDB and reads the two graphs back
    (snapshot_from_dataset over GraphDBClient.read_graphs).
  * Offline (tests and experiments/), local_snapshot builds the same two graphs in memory, with the
    same functions the agent uses to write them.

read_case answers, with the fixed queries of agents/semantic/data/queries/episode/, the episode
(the contract-1 fields those steps read), the trigger the live validator recorded, and the
provenance of the case. The contract-1 JSON is still written and kept, but it is no longer what
the prompt is built from.
"""

from __future__ import annotations

import hashlib
import subprocess
from dataclasses import dataclass
from decimal import Decimal
from functools import lru_cache
from pathlib import Path
from typing import Any, Optional

from rdflib import OWL, RDF, RDFS, Dataset, Graph, Literal, URIRef
from rdflib.compare import to_isomorphic

from src.case_rdf import case_iri
from src.episode_contrast import DEFAULT_TBOX, tbox_graph
from src.episode_rdf import DUL, INSIGHT, INST, episode_graph, episode_namespace, graph_iri, observation_iri

QUERIES_DIR = Path(__file__).resolve().parents[1] / "data" / "queries" / "episode"
REPO = Path(__file__).resolve().parents[3]
ONTOLOGY_GRAPH = "http://insight.local/ontology"
ONTOLOGY_IRI = URIRef("http://insight.local/ontology")
#: Version of the contracts between the semantic memory and the evaluation
#: (docs_output/implementacion/contratos.md) that this code writes.
CONTRACTS_VERSION = "1.12"


class SemanticMemoryError(RuntimeError):
    """The semantic memory does not hold the case it was asked for (or could not be read)."""


# --------------------------------------------------------------------------- #
# The snapshot
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MemorySnapshot:
    """The two named graphs the generation reads, as they were read."""

    tbox: Graph                 #: the TBox's graph
    case: Graph                 #: the case's graph: the episode, the trigger and the provenance
    graphs: tuple[str, str]     #: their IRIs (TBox, case)
    sha256: str                 #: digest of both, independent of blank-node labels
    source: str                 #: "graphdb" or "local"

    def summary(self) -> dict[str, Any]:
        """What the batch records of the memory it was built from."""
        return {"source": self.source, "graphs": list(self.graphs), "snapshot_sha256": self.sha256}


def _copy(graph) -> Graph:
    """A graph of its own (the generation caches what it derives from a TBox by graph identity)."""
    copy = Graph()
    for prefix, namespace in graph.namespaces():
        copy.bind(prefix, namespace)
    for triple in graph:
        copy.add(triple)
    return copy


def snapshot_digest(graphs: dict[str, Graph]) -> str:
    """sha256 over each graph's IRI and canonical digest: the same for the same triples, whatever the
    blank-node labels (the TBox has blank nodes, and GraphDB relabels them)."""
    lines = [f"{iri} {to_isomorphic(graph).graph_digest()}" for iri, graph in sorted(graphs.items())]
    return hashlib.sha256("\n".join(lines).encode("utf-8")).hexdigest()


def snapshot_from_dataset(dataset: Dataset, tbox_iri: str, case_iri: str, source: str = "graphdb") -> MemorySnapshot:
    """The snapshot of the two graphs of a dataset (what GraphDBClient.read_graphs returns)."""
    tbox, case = _copy(dataset.graph(URIRef(tbox_iri))), _copy(dataset.graph(URIRef(case_iri)))
    if not len(tbox):
        raise SemanticMemoryError(f"the semantic memory has no TBox in <{tbox_iri}>")
    if not len(case):
        raise SemanticMemoryError(f"the semantic memory has no case in <{case_iri}>")
    return MemorySnapshot(tbox=tbox, case=case, graphs=(tbox_iri, case_iri),
                          sha256=snapshot_digest({tbox_iri: tbox, case_iri: case}), source=source)


def local_snapshot(episode: dict[str, Any], trigger: Optional[dict[str, Any]] = None,
                   tbox: str | Path | Graph = DEFAULT_TBOX, case_id: Optional[str] = None) -> MemorySnapshot:
    """The snapshot the agent would read after writing this case, built without GraphDB."""
    tbox_triples = _copy(tbox_graph(tbox))
    case = case_graph(episode, trigger, case_id or episode["episode_id"], tbox)
    tbox_iri, case_iri = ONTOLOGY_GRAPH, str(graph_iri(episode["episode_id"]))
    return MemorySnapshot(tbox=tbox_triples, case=case, graphs=(tbox_iri, case_iri),
                          sha256=snapshot_digest({tbox_iri: tbox_triples, case_iri: case}), source="local")


# --------------------------------------------------------------------------- #
# What the agent writes for a case before generating: the episode and the provenance
# --------------------------------------------------------------------------- #
@lru_cache(maxsize=1)
def code_commit() -> str:
    """The git commit of this code, with '+uncommitted' when tracked files have changes."""
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True, text=True, timeout=10)
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO,
                               capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    if head.returncode != 0:
        return "unknown"
    return head.stdout.strip() + ("+uncommitted" if dirty.returncode == 0 and dirty.stdout.strip() else "")


def tbox_version(tbox: str | Path | Graph) -> str:
    return str(tbox_graph(tbox).value(ONTOLOGY_IRI, OWL.versionInfo) or "unknown")


def tbox_sha256(tbox: str | Path | Graph) -> str:
    """sha256 of the TBox file; for a graph, its canonical digest."""
    if isinstance(tbox, Graph):
        return hashlib.sha256(str(to_isomorphic(tbox).graph_digest()).encode("utf-8")).hexdigest()
    return hashlib.sha256(Path(tbox).read_bytes()).hexdigest()


def provenance_graph(episode: dict[str, Any], case_id: str, tbox: str | Path | Graph = DEFAULT_TBOX) -> Graph:
    """The case, set in its episode, with the TBox, contracts and code it is written with."""
    graph = Graph()
    case = case_iri(case_id)
    graph.add((case, RDF.type, OWL.NamedIndividual))
    graph.add((case, RDF.type, INST.AnomalyCase))
    graph.add((case, RDFS.label, Literal(case_id)))
    graph.add((case, INST.concernsEvent, observation_iri(episode)))
    graph.add((episode_namespace(episode["episode_id"]).Episode, DUL.isSettingFor, case))
    for prop, value in ((INSIGHT.tboxVersion, tbox_version(tbox)), (INSIGHT.tboxSha256, tbox_sha256(tbox)),
                        (INSIGHT.contractsVersion, CONTRACTS_VERSION), (INSIGHT.codeCommit, code_commit())):
        graph.add((case, prop, Literal(value)))
    return graph


def case_graph(episode: dict[str, Any], trigger: Optional[dict[str, Any]], case_id: str,
               tbox: str | Path | Graph = DEFAULT_TBOX) -> Graph:
    """What the agent writes to the case's graph before generating."""
    graph = episode_graph(episode, trigger)
    for triple in provenance_graph(episode, case_id, tbox):
        graph.add(triple)
    return graph


# --------------------------------------------------------------------------- #
# Reading the case
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CaseView:
    """What the generation reads of a case, answered by the semantic memory."""

    episode: dict[str, Any]             #: the contract-1 fields the generation reads
    trigger: Optional[dict[str, Any]]   #: the live validator's delta and reason, if it recorded one
    record_sha256: Optional[str]        #: the contract-1 record the episode graph was written from
    provenance: dict[str, Optional[str]]


@lru_cache(maxsize=None)
def _query(name: str) -> str:
    return (QUERIES_DIR / f"{name}.rq").read_text(encoding="utf-8")


def _value(term: Any) -> Any:
    """A query term as a plain value: decimals as float, integers as int, the rest as text."""
    if term is None:
        return None
    if isinstance(term, Literal):
        value = term.toPython()
        if isinstance(value, Decimal):
            return float(value)
        if isinstance(value, (bool, int, float)):
            return value
        return str(term)
    return str(term)


def _rows(graph: Graph, name: str, **bindings) -> list[dict[str, Any]]:
    result = graph.query(_query(name), initBindings={k: v for k, v in bindings.items() if v is not None})
    names = [str(v) for v in result.vars]
    return [{n: _value(row[i]) for i, n in enumerate(names)} for row in result]


def _present(**values) -> dict[str, Any]:
    return {key: value for key, value in values.items() if value is not None}


def _entity_id(iri: Optional[str], ids: dict[str, str], namespace: str) -> Optional[str]:
    """The id the episode gives an entity: its role binding's, else its local name in the namespace."""
    if iri is None:
        return None
    if iri in ids:
        return ids[iri]
    return iri[len(namespace):] if iri.startswith(namespace) else iri


def read_episode(graph: Graph, episode_iri: URIRef) -> tuple[dict[str, Any], Optional[dict[str, Any]], Optional[str]]:
    """(episode, trigger, record sha256) of the episode individual of a case graph."""
    found = _rows(graph, "episode", episode=episode_iri)
    if not found:
        raise SemanticMemoryError(f"no episode <{episode_iri}> in the case graph")
    head = found[0]
    namespace = head["namespace"] or str(INST)
    episode: dict[str, Any] = _present(schema=head["schema"], episode_id=head["id"])
    source = _present(kind=head["kind"], path=head["path"], sha256=head["recording_sha256"],
                      builder=head["builder"], reader=head["reader"], pose_axes=head["pose_axes"])
    if source:
        episode["source"] = source
    frame = _present(name=head["frame"], units=head["units"], heading=head["heading"])
    if frame:
        episode["frame"] = frame
    clock = _present(origin=head["origin"], origin_ns=head["origin_ns"],
                     origin_offset_from_file_start_s=head["origin_offset"], t_obs_s=head["t_obs"],
                     episode_length_s=head["length"], simulation_horizon_s=head["horizon"])
    if clock:
        episode["time"] = clock
    if head["namespace"] is not None:
        episode["entity_namespace"] = head["namespace"]
    if head["mission"] is not None:
        episode["mission"] = head["mission"]

    roles = _rows(graph, "roles", episode=episode_iri)
    if roles:
        episode["entities"] = {row["role"]: row["entity_id"] for row in roles}
    ids = {row["entity"]: row["entity_id"] for row in reversed(_rows(graph, "labels", episode=episode_iri))
           if row["entity_id"] is not None}
    ids.update({str(o): str(graph.value(b, INSIGHT.entityId)) for b, o in graph.subject_objects(INSIGHT.boundEntity)
                if str(o) not in ids})
    labels = {_entity_id(row["entity"], ids, namespace): row["label"]
              for row in _rows(graph, "labels", episode=episode_iri)}
    if labels:
        episode["entity_labels"] = labels

    intervals = [_present(id=r["id"], start_s=r["start"], end_s=r["end"], is_system_reaction=r["reaction"])
                 for r in _rows(graph, "intervals", episode=episode_iri)]
    if intervals:
        episode["intervals"] = intervals

    phases = []
    for r in _rows(graph, "phases", episode=episode_iri):
        if r["reaction"]:
            phase = _present(id=r["id"], kind="system_reaction", interval=r["interval"], is_system_reaction=True,
                             stopped_at_s=r["stopped_at"], note=r["note"])
        else:
            phase = _present(id=r["id"], kind=r["kind"], interval=r["interval"], mean_speed_mps=r["speed"])
            if r["setpoint_min"] is not None:
                phase["setpoint_mps"] = [r["setpoint_min"], r["setpoint_max"]]
            if r["heading_min"] is not None:
                phase["heading_deg_range"] = [r["heading_min"], r["heading_max"]]
        phases.append(phase)
    if phases:
        episode["phases"] = phases

    segments = [{"id": r["id"], "order_back_from_fall": r["order"], "interval": r["interval"],
                 "from_xy": [r["from_x"], r["from_y"]], "to_xy": [r["to_x"], r["to_y"]],
                 "bbox": {"x": [r["x_min"], r["x_max"]], "y": [r["y_min"], r["y_max"]]},
                 "length_m": r["length"], "heading_deg": r["heading"], "mean_speed_mps": r["speed"]}
                for r in _rows(graph, "segments", episode=episode_iri)]
    if segments:
        episode["segments"] = segments

    support = _rows(graph, "support", episode=episode_iri)
    if support:
        r = support[0]
        episode["support"] = _present(id=r["id"], supporter=_entity_id(r["supporter"], ids, namespace),
                                      supported=_entity_id(r["supported"], ids, namespace),
                                      start_s=r["start"], end_s=r["end"], ended_by=r["ended_by"])

    observed = _rows(graph, "observation", episode=episode_iri)
    if len(observed) != 1:
        raise SemanticMemoryError(f"the episode <{episode_iri}> needs one observed change, it has {len(observed)}")
    o = observed[0]
    record = _present(id=o["recorded_id"], time_s=o["time"], place=o["segment"])
    if o["x"] is not None or o["heading"] is not None:
        record["robot_pose"] = _present(xy=[o["x"], o["y"]] if o["x"] is not None else None, heading_deg=o["heading"])
    record.update(_present(observed_as=o["observed_as"], summary=o["summary"], affected_entity=o["affected_entity"],
                           mission=o["mission"]))
    unknowns = [r["statement"] for r in _rows(graph, "unknowns", episode=episode_iri)]
    if unknowns:
        record["unknowns"] = unknowns
    profiles = [r["profile"] for r in _rows(graph, "profiles", episode=episode_iri)]
    if profiles:
        record["profile_ids"] = profiles
    trigger: dict[str, Any] = {}
    if o["validator_reason"] is not None:
        trigger["unexplained_reason"] = o["validator_reason"]
    for r in _rows(graph, "changes", episode=episode_iri):
        change = _present(operation=r["operation"], subject=r["subject"], predicate=r["predicate"], object=r["object"])
        if r["source"] == "change":
            record.update(change)
        elif r["source"] == "changes":
            record.setdefault("changes", []).append(change)
        else:
            trigger.setdefault(r["source"], []).append(
                {key: change[key] for key in ("subject", "predicate", "object") if key in change})
    if trigger:
        trigger = {"unexplained_reason": trigger.get("unexplained_reason"),
                   "removed_triples": trigger.get("removed_triples", []),
                   "added_triples": trigger.get("added_triples", [])}
    episode[o["section"]] = record

    evidence = []
    for r in _rows(graph, "evidence", episode=episode_iri):
        entry = _present(id=r["id"], property=r["property"].rsplit("#", 1)[-1], interval=r["interval"])
        entry["value"] = r["value"]
        entry.update(_present(unit=r["unit"], at_s=r["at"], signed_value=r["signed"], baseline=r["baseline"],
                              commanded=r["commanded"], own_ratio_fall=r["ratio"],
                              own_ratio_baseline=r["ratio_baseline"],
                              is_system_reaction=True if r["reaction"] else None))
        evidence.append(entry)
    if evidence:
        episode["evidence"] = evidence
    return episode, (trigger or None), head["record_sha256"]


def read_case(snapshot: MemorySnapshot, case_id: Optional[str] = None) -> CaseView:
    """The case of a snapshot: its episode (there is one per case graph), trigger and provenance."""
    episodes = list(snapshot.case.subjects(RDF.type, INSIGHT.Episode))
    if len(episodes) != 1:
        raise SemanticMemoryError(f"the case graph holds {len(episodes)} episodes, not one")
    episode, trigger, record = read_episode(snapshot.case, episodes[0])
    provenance = {"tbox_version": None, "tbox_sha256": None, "contracts_version": None, "code_commit": None}
    if case_id is not None:
        rows = _rows(snapshot.case, "provenance", case=case_iri(case_id))
        if rows:
            provenance.update(rows[0])
    in_memory = str(snapshot.tbox.value(ONTOLOGY_IRI, OWL.versionInfo) or "")
    if provenance["tbox_version"] is not None and provenance["tbox_version"] != in_memory:
        raise SemanticMemoryError(f"the case was written with TBox {provenance['tbox_version']}, "
                                  f"the memory holds TBox {in_memory or 'without a version'}")
    return CaseView(episode=episode, trigger=trigger, record_sha256=record, provenance=provenance)
