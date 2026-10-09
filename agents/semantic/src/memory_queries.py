"""The competency questions of the semantic memory (contracts 1.11), and its current RDF case model.

Each question is a SPARQL file in agents/semantic/data/queries/ whose first comment line states it.
They are written to give the same rows with and without inference (classes are matched on what the
agent writes, never on what a reasoner would add), so that they run on GraphDB, which infers with
the TBox, and offline on rdflib, which does not.

case_dataset() rebuilds, without GraphDB, the named graphs the semantic agent leaves after a case:
the TBox in its own graph, the live graph (the mirror of the working memory) and the case's graph
(the episode with its trigger and provenance, the decisions, the simulation runs, the supported
explanation and the legacy consolidation triples), from the same functions the agent calls. GraphDB
keeps every case (contracts 1.12): called again on the same dataset, it adds another case, and
every question answers with the case each row belongs to.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

from rdflib import Dataset, Graph, Literal, URIRef

from src.case_rdf import decision_graph, simulation_graph
from src.episode_contrast import DEFAULT_TBOX
from src.episode_rdf import graph_iri
from src.semantic_memory import ONTOLOGY_GRAPH, case_graph
from src.verdict_ingestor import build_case_triples, case_triples_graph, mechanism_graph

QUERIES_DIR = Path(__file__).resolve().parents[1] / "data" / "queries"
LIVE_GRAPH = "urn:insight:semantic:live"


@dataclass(frozen=True)
class Question:
    name: str
    question: str
    text: str


def load_questions(directory: Path = QUERIES_DIR) -> list[Question]:
    questions = []
    for path in sorted(directory.glob("cq*.rq")):
        text = path.read_text(encoding="utf-8")
        first = text.splitlines()[0].lstrip("# ").strip()
        questions.append(Question(path.stem, first, text))
    return questions


def python_value(term: Any) -> Any:
    """A query term as a plain value: numbers as float (rounded), the rest as text."""
    if term is None:
        return None
    if isinstance(term, Literal):
        value = term.toPython()
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) or type(value).__name__ == "Decimal":
            return round(float(value), 6)
        return str(value)
    return str(term)


def run(graph: Graph | Dataset, question: Question) -> list[dict[str, Any]]:
    result = graph.query(question.text)
    names = [str(v) for v in result.vars]
    return [{name: python_value(row[i]) for i, name in enumerate(names)} for row in result]


def case_dataset(*, episode: dict, batch: dict, verdict: Optional[dict], mirror: Iterable[tuple[str, str, str]],
                 batch_path: Optional[str] = None, verdict_path: Optional[str] = None,
                 tbox_path: str | Path = DEFAULT_TBOX, trigger: Optional[dict] = None,
                 dataset: Optional[Dataset] = None) -> Dataset:
    """The named graphs the agent leaves in GraphDB after a case, built without GraphDB. With a
    dataset, the case is added to the ones it holds, as GraphDB keeps them."""
    if dataset is None:
        dataset = Dataset(default_union=True)
        dataset.graph(URIRef(ONTOLOGY_GRAPH)).parse(str(tbox_path), format="turtle")

    live = dataset.graph(URIRef(LIVE_GRAPH))
    live.remove((None, None, None))
    for s, p, o in mirror:
        live.add((URIRef(s), URIRef(p), URIRef(o)))

    accepted = (verdict or {}).get("accepted_hypothesis_id")
    abstained = (verdict or {}).get("nominal_effect_warning") or (verdict or {}).get("abstention_reason")
    case = dataset.graph(graph_iri(episode["episode_id"]))
    pieces = [case_graph(episode, trigger if trigger is not None else batch.get("trigger"), batch["case_id"], tbox_path),
              decision_graph(batch, episode, batch_path, tbox_path)]
    if verdict is not None:
        pieces.append(simulation_graph(batch, verdict, verdict_path))
        if accepted and not abstained:
            cause = next(h for h in verdict["hypotheses"] if h["hypothesis_id"] == accepted)["cause"]["name"]
            pieces += [case_triples_graph(build_case_triples(cause, batch["case_id"])),
                       mechanism_graph(batch, accepted, episode)]
    for piece in pieces:
        if piece is not None:
            for triple in piece:
                case.add(triple)
    return dataset
