"""Offline test: the competency questions of the semantic memory, on a real case (contracts 1.8).

The case is the Webots run of 2026-10-07 at 10:00 (a 25 x 5 cm bump): its episode and batch, as
the semantic agent wrote them, and the verdict of the inner simulator with the fall counted only
at its time (verdict 1.2), which accepts the bump (docs_output/resultados/19). The named graphs
are rebuilt as the agent leaves them in GraphDB (memory_queries.case_dataset), and each question
of agents/semantic/data/queries/ must give the answer the files give. Criterion in
docs_output/resultados/20_memoria_en_graphdb/CRITERIO.md:
  * every hypothesis is there with the batch's status; the discarded ones with the check and the
    observation that decided them (H05: person_within_reach on person_distance_min = 3.269; H07:
    bottle_reacquired), and H06 with why it is not simulable;
  * H02 (obstacle_traversed on Segment_final) supports an explanation of the observed anomaly, with 7 of 36 repetitions at the
    right time, and the nominal run reproduced it in 0 of 10;
  * the episode's graph only uses TBox terms, and HermiT finds the TBox plus the whole case
    consistent (if owlready2 is installed).

The same questions on the real GraphDB, with its inference: experiments/semantic_memory_graphdb.py.

The historical batch is preserved. A separate current-pipeline check verifies that carrying
relation restoration only informs, that perception failure is not simulable rather than
discarded, and that both the informative check and the reason are stored in RDF (contracts 1.10).

Run from the repo root:  python3 tests/offline/test_semantic_memory_queries.py
"""
import copy
import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SEMANTIC = REPO / "agents" / "semantic"
sys.path.insert(0, str(SEMANTIC))

from rdflib import OWL, RDF, RDFS, Graph, Literal, Namespace, URIRef  # noqa: E402

from src.case_rdf import hypothesis_iri  # noqa: E402
from src.episode_rdf import graph_iri  # noqa: E402
from src.hypothesis_pipeline import publish_batch  # noqa: E402
from src.memory_queries import ONTOLOGY_GRAPH, case_dataset, load_questions, run  # noqa: E402

CASE = SEMANTIC / "generated_hypotheses" / "semantic_unexplained_20261007T080222Z"
VERDICT = Path(__file__).resolve().parent / "data" / "verdict_semantic_unexplained_20261007T080222Z.json"
TBOX = SEMANTIC / "data" / "insight_tbox.ttl"

INST = "http://insight.local/instances#"
DUL = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#"
TYPE = str(RDF.type)
#: What the agent mirrors of the working memory once the bottle is lost and the cycle is closed.
MIRROR = {(INST + "Agent_Robot", TYPE, DUL + "PhysicalAgent"), (INST + "Agent_Person", TYPE, DUL + "PhysicalAgent"),
          (INST + "PhysicalObject_Bottle", TYPE, DUL + "PhysicalObject"),
          (INST + "PhysicalPlace_Room", TYPE, DUL + "PhysicalPlace"),
          (INST + "Agent_Robot", DUL + "hasLocation", INST + "PhysicalPlace_Room"),
          (INST + "Agent_Person", DUL + "hasLocation", INST + "PhysicalPlace_Room")}


def load_case():
    episode = json.loads(Path(f"{CASE}_episode.json").read_text(encoding="utf-8"))
    batch = json.loads(Path(f"{CASE}.json").read_text(encoding="utf-8"))
    verdict = json.loads(VERDICT.read_text(encoding="utf-8"))
    dataset = case_dataset(episode=episode, batch=batch, verdict=verdict, mirror=MIRROR,
                           batch_path=f"{CASE}.json", verdict_path=str(VERDICT))
    return episode, batch, verdict, dataset


def answers(dataset):
    return {question.name: run(dataset, question) for question in load_questions()}


def check_answers(episode, batch, verdict, dataset):
    got = answers(dataset)
    assert len(got) == 9, sorted(got)

    fall = got["cq1_fall"]
    assert len(fall) == 1, fall
    assert (fall[0]["t_obs"], fall[0]["segment"], fall[0]["phase"], fall[0]["motion"], fall[0]["reaction"]) == (
        episode["accident"]["time_s"], "Segment_final", "Phase_2", "advance_straight", "Phase_reaction"), fall
    assert fall[0]["observation"] == "Accident_1" and fall[0]["estimate"] == "Support_bottle", fall
    assert (fall[0]["estimate_start"], fall[0]["estimate_end"]) == (0.0, 13.691), fall

    proposals = {row["id"]: row for row in got["cq2_proposals"]}
    assert {i: row["status"] for i, row in proposals.items()} == {
        h["hypothesis_id"]: h["status"] for h in batch["hypotheses"]}, proposals
    assert proposals["H02"]["mechanism"] == "obstacle_traversed" and proposals["H02"]["anchors"] == "Segment_final"
    assert proposals["H02"]["kinds"] == "shape=bump" and proposals["H01"]["kinds"] == "direction=left"

    why = {row["id"]: row for row in got["cq3_why_not_simulated"]}
    assert set(why) == {"H05", "H06", "H07"}, why
    assert (why["H05"]["check"], why["H05"]["verdict"], why["H05"]["property"], why["H05"]["value"]) == (
        "person_within_reach", "discards", "person_distance_min", 3.269), why["H05"]
    assert (why["H07"]["check"], why["H07"]["property"], why["H07"]["value"]) == (
        "bottle_reacquired", "bottle_reacquired", False), why["H07"]
    assert why["H06"]["status"] == "not_simulable" and why["H06"]["check"] is None
    assert "experimental" in why["H06"]["reason"], why["H06"]

    traces = {}
    for row in got["cq4_traces_by_sensor"]:
        traces.setdefault(row["mechanism"], set()).update({row["sensor"]} - {None})
    assert len(traces) == 10 and {m for m, s in traces.items() if not s} == {"bottle_push", "suspension_failure"}
    assert traces["obstacle_traversed"] == {"IMU (WT901B AHRS)"}, traces

    cause = got["cq5_cause"]
    assert len(cause) == 1, cause
    cause = cause[0]
    assert (cause["mechanism"], cause["anchor"], cause["observation"]) == ("obstacle_traversed", "Segment_final", "Accident_1")
    assert cause["hypothesis"] == "H02" and cause["unresolved_alternatives"] == "H06", cause
    assert cause["repetitions"] == 36 and round(cause["effect_rate"] * 36) == 7 and cause["mistimed"] == 5, cause
    assert (cause["nominal_repetitions"], cause["nominal_effect_rate"]) == (10, 0.0), cause

    magnitudes = {row["quantity"]: row["value"] for row in got["cq6_magnitudes"]}
    accepted = next(h for h in verdict["hypotheses"] if h["accepted"])
    assert magnitudes == {k: round(v, 6) for k, v in accepted["best_generated_instances"]["sample"].items()}

    runs = got["cq7_simulations"]
    simulated = {h["hypothesis_id"] for h in batch["hypotheses"] if h["status"] == "to_simulate"}
    assert {row["hypothesis"] for row in runs} == simulated | {None}, runs
    assert [row["hypothesis"] for row in runs if row["accepted"]] == ["H02"], runs

    assert {row["mechanism"] for row in got["cq8_not_simulable"]} == {
        "bottle_removed_by_person", "perception_failure", "suspension_failure", "uncommanded_motion"}

    provenance = got["cq9_provenance"]
    assert len(provenance) == 1 and provenance[0]["model"] == batch["model"], provenance
    assert provenance[0]["recording"].endswith("mission_Follow_Person_07102026_100057.txt"), provenance


def check_only_tbox_terms(episode, batch, verdict, dataset):
    tbox = Graph().parse(TBOX)
    stored = dataset.graph(graph_iri(episode["episode_id"]))
    properties = {s for kind in (OWL.ObjectProperty, OWL.DatatypeProperty) for s in tbox.subjects(RDF.type, kind)}
    classes = set(tbox.subjects(RDF.type, OWL.Class))
    used_properties = {p for _, p, _ in stored} - {RDF.type, RDFS.label, RDFS.comment}
    used_classes = {o for _, p, o in stored if p == RDF.type} - {OWL.NamedIndividual}
    assert used_properties <= properties, used_properties - properties
    assert used_classes <= classes, used_classes - classes
    # Every individual the decisions point to in the TBox exists there (statuses, kinds, checks, mechanisms).
    referenced = {o for _, p, o in stored if isinstance(o, URIRef) and str(o).startswith("http://insight.local/ontology#")}
    missing = {o for o in referenced if not any(tbox.triples((o, None, None)))}
    assert not missing, missing


def check_consistency_if_possible(episode, batch, verdict, dataset):
    try:
        import owlready2
    except ImportError:
        print("  (owlready2 not installed: HermiT consistency check skipped)")
        return
    merged = Graph()
    for name in (ONTOLOGY_GRAPH, str(graph_iri(episode["episode_id"]))):
        for triple in dataset.graph(URIRef(name)):
            merged.add(triple)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "case_with_tbox.owl"
        merged.serialize(path, format="xml")
        world = owlready2.World()
        ontology = world.get_ontology(path.as_uri()).load()
        with ontology:
            owlready2.sync_reasoner_hermit(world, infer_property_values=False, debug=0)
        unsatisfiable = list(world.inconsistent_classes())
    assert not unsatisfiable, unsatisfiable
    print("  HermiT: the TBox plus the whole case (episode, decisions, runs, explanation) is consistent")


def check_current_perception_failure_stays_unresolved(episode, historical_batch, verdict, dataset):
    insight = Namespace("http://insight.local/ontology#")
    dul = Namespace(DUL)
    proposal = {"hypotheses": [{
        "mechanism": "perception_failure", "new_mechanism_description": None,
        "segment": None, "interval": "Interval_fall", "qualitative_parameters": {},
        "expected_trace": [], "rationale": "The carrying representation may have failed.",
        "title": "Perception failure",
    }]}
    for value, expected in ((False, "no_trace"), (True, "trace_seen"), (None, "undetermined")):
        current_episode = copy.deepcopy(episode)
        entry = next(e for e in current_episode["evidence"] if e["property"] == "bottle_reacquired")
        entry["value"] = value
        batch = publish_batch(current_episode, proposal, arm="production", tbox_path=TBOX,
                              profiles=("carried_object_relation_loss",))
        hypothesis = batch["hypotheses"][0]
        assert hypothesis["status"] == "checked_not_simulable" and not hypothesis["testable"], hypothesis
        assert hypothesis["cost_in_simulations"] == 0, hypothesis
        assert hypothesis["checks"]["contrast"][0]["verdict"] == expected, hypothesis
        current = case_dataset(episode=current_episode, batch=batch, verdict=None, mirror=MIRROR)
        graph = current.graph(graph_iri(episode["episode_id"]))
        h = hypothesis_iri(batch, hypothesis["hypothesis_id"])
        assert (h, dul.isClassifiedBy, insight.Status_checked_not_simulable) in graph
        check = graph.value(h, insight.checkedBy)
        assert (check, insight.checkVerdict, Literal(expected)) in graph
        assert "within the available recording" in str(graph.value(check, RDFS.comment))
        row, = answers(current)["cq3_why_not_simulated"]
        assert row["status"] == "checked_not_simulable" and row["check"] is None, row
        assert "not simulable" in row["reason"], row
        assert not list(graph.subjects(RDF.type, insight.SimulationRun))
    # A new policy must not rewrite decisions that were recorded under the previous policy.
    historical = next(h for h in historical_batch["hypotheses"] if h["mechanism"] == "perception_failure")
    assert historical["status"] == "discarded", historical


def main():
    case = load_case()
    for check in (check_answers, check_only_tbox_terms, check_consistency_if_possible,
                  check_current_perception_failure_stays_unresolved):
        check(*case)
        print(f"OK {check.__name__}")
    print("Semantic memory queries: all checks passed")


if __name__ == "__main__":
    main()
