"""Observed changes, estimates and simulation explanations stay distinct in RDF (contracts 1.11).

Checks a saved episode and verdict, another explicit change without motion data, unresolved
perception alternatives, abstention, and DUL entailments that could silently turn a description
into an actual event. No new LLM call or physical simulation. Run from the repository root.
"""
import copy
import json
import sys
import tempfile
from pathlib import Path

from rdflib import Graph, RDF

REPO = Path(__file__).resolve().parents[2]
SEMANTIC = REPO / "agents/semantic"
sys.path.insert(0, str(SEMANTIC))

from src.case_rdf import decision_graph  # noqa: E402
from src.episode_rdf import DUL, INSIGHT, INST, SOMA, entity_iri, episode_graph, episode_namespace  # noqa: E402
from src.hypothesis_pipeline import publish_batch  # noqa: E402
from src.memory_queries import case_dataset, load_questions, run  # noqa: E402
from src.verdict_ingestor import build_case_triples, mechanism_graph  # noqa: E402

CASE = SEMANTIC / "generated_hypotheses/semantic_unexplained_20261007T080222Z"
TBOX = SEMANTIC / "data/insight_tbox.ttl"
EPISODE = json.loads(Path(f"{CASE}_episode.json").read_text())
BATCH = json.loads(Path(f"{CASE}.json").read_text())
VERDICT = json.loads((REPO / "tests/offline/data/verdict_semantic_unexplained_20261007T080222Z.json").read_text())


def check_observation_does_not_assert_physical_loss():
    graph = episode_graph(EPISODE)
    ep = episode_namespace(EPISODE["episode_id"])
    assert (ep.Accident_1, RDF.type, INSIGHT.ObservedAnomaly) in graph
    assert (ep.Accident_1, INSIGHT.observedBy, INST.Agent_Robot) in graph
    assert (ep.Accident_1, INSIGHT.affectedEntity, INST.PhysicalObject_Bottle) in graph
    assert (ep.Accident_1, INSIGHT.observedAtSegment, ep.Segment_final) in graph
    assert (ep.Accident_1, INSIGHT.invalidatesEstimate, ep.Support_bottle) in graph
    assert (ep.Support_bottle, RDF.type, INSIGHT.SupportEstimate) in graph
    assert (ep.Support_bottle, DUL.usesConcept, ep.EstimatedSupportKind) in graph
    assert (ep.EstimatedSupportKind, RDF.type, SOMA.SupportState) in graph
    assert (ep.Support_bottle, INSIGHT.estimatedSupporter, INST.PhysicalObject_Tray) in graph
    assert not list(graph.subjects(RDF.type, SOMA.Accident))
    assert not list(graph.subjects(RDF.type, SOMA.State))
    assert not list(graph.triples((None, INSIGHT.ends, None)))
    assert not list(graph.triples((None, SOMA.causes, None)))
    assert not list(graph.triples((ep.Accident_1, DUL.hasLocation, None)))
    assert not list(graph.triples((INST.PhysicalObject_Bottle, DUL.hasLocation, None)))
    assert not list(graph.triples((ep.Support_bottle, DUL.hasTimeInterval, None)))
    assert not list(graph.triples((ep.Support_bottle, DUL.hasParticipant, None)))


def check_explanation_is_a_description_with_provenance():
    graph = mechanism_graph(BATCH, "H02", EPISODE)
    ep = episode_namespace(EPISODE["episode_id"])
    explanation = ep.Explanation_H02
    assert (explanation, RDF.type, INSIGHT.SimulationSupportedExplanation) in graph
    assert (explanation, INSIGHT.selectedHypothesis, ep.Hypothesis_H02) in graph
    assert (explanation, INSIGHT.supportedBySimulation, ep.Run_H02) in graph
    assert (explanation, INSIGHT.explainsObservation, ep.Accident_1) in graph
    assert (explanation, INSIGHT.hasUnresolvedAlternative, ep.Hypothesis_H06) in graph
    assert not list(graph.triples((None, SOMA.causes, None)))
    assert not list(graph.subjects(RDF.type, INSIGHT.VerifiedCause))
    assert not list(graph.triples((explanation, DUL.hasLocation, None)))
    assert not list(graph.triples((explanation, DUL.hasTimeInterval, None)))
    assert not list(graph.triples((explanation, INSIGHT.verifiedBy, None)))
    legacy = build_case_triples("bump_scaled", BATCH["case_id"])
    assert not any(p == str(RDF.type) and o == str(DUL.Event) for _, p, o in legacy)
    assert any(o == str(INSIGHT.SimulationSupportedExplanation) for _, _, o in legacy)
    assert mechanism_graph(BATCH, "unknown", EPISODE) is None
    assert mechanism_graph(BATCH, "H06", EPISODE) is None  # Not simulated, so cannot support an explanation.


def check_current_perception_alternative_remains_visible():
    def idea(mechanism, **parameters):
        return {"mechanism": mechanism, "segment": "Segment_final" if parameters else None,
                "interval": None if parameters else "Interval_fall", "qualitative_parameters": parameters,
                "new_mechanism_description": None, "expected_trace": [], "rationale": "Test proposal", "title": "Test"}
    batch = publish_batch(EPISODE, {"hypotheses": [idea("obstacle_traversed", shape="bump"), idea("perception_failure")]},
                          arm="production", case_id="test_epistemic_rdf", profiles=("carried_object_relation_loss",))
    assert batch["hypotheses"][1]["status"] == "checked_not_simulable"
    graph = mechanism_graph(batch, "H01", EPISODE) + decision_graph(batch, EPISODE)
    ep = episode_namespace(EPISODE["episode_id"])
    assert (ep.Explanation_H01, INSIGHT.hasUnresolvedAlternative, ep.Hypothesis_H02) in graph
    assert (ep.Hypothesis_H02, DUL.isClassifiedBy, INSIGHT.Status_checked_not_simulable) in graph
    assert not list(graph.triples((None, SOMA.causes, None)))


def check_abstention_keeps_runs_without_an_explanation():
    for changes in ({"accepted_hypothesis_id": None}, {"nominal_effect_warning": True},
                    {"abstention_reason": "The nominal replay reproduces the effect."}):
        verdict = {**copy.deepcopy(VERDICT), **changes}
        dataset = case_dataset(episode=EPISODE, batch=BATCH, verdict=verdict, mirror=[])
        assert list(dataset.subjects(RDF.type, INSIGHT.SimulationRun))
        assert not list(dataset.subjects(RDF.type, INSIGHT.SimulationSupportedExplanation))
        assert not list(dataset.triples((None, SOMA.causes, None)))


def check_another_change_does_not_borrow_object_loss_facts():
    episode = {"episode_id": "target_tracking_change", "entities": {"robot": "Agent_Robot", "person": "Agent_Target"},
               "change": {"id": "Observed_target_loss", "summary": "The target association disappeared.",
                          "time_s": 7.0, "subject": "Agent_Target"}}
    graph = episode_graph(episode)
    ep = episode_namespace(episode["episode_id"])
    assert (ep.Observed_target_loss, INSIGHT.affectedEntity, INST.Agent_Target) in graph
    assert not list(graph.subjects(RDF.type, INSIGHT.SupportEstimate))
    assert not list(graph.triples((None, INSIGHT.observerX, None)))
    assert not list(graph.triples((None, INSIGHT.observedAtSegment, None)))
    assert not list(graph.triples((None, DUL.isSettingFor, INST.PhysicalObject_Bottle)))
    question = next(q for q in load_questions() if q.name == "cq1_fall")
    row, = run(graph, question)
    assert row["observation"] == "Observed_target_loss" and row["t_obs"] == 7.0, row
    assert row["robot_x"] is None and row["estimate"] is None, row
    medicine = copy.deepcopy(EPISODE)
    medicine["entities"]["bottle"] = medicine["support"]["supported"] = "PhysicalObject_Medicine"
    changed = episode_graph(medicine)
    assert (episode_namespace(medicine["episode_id"]).Accident_1,
            INSIGHT.affectedEntity, INST.PhysicalObject_Medicine) in changed
    assert entity_iri(medicine, "http://example.org/medicine") == entity_iri({}, "http://example.org/medicine")


def check_inferred_types_if_possible():
    try:
        import owlready2
    except ImportError:
        print("  (owlready2 not installed: inferred-type check skipped)")
        return
    dataset = case_dataset(episode=EPISODE, batch=BATCH, verdict=VERDICT, mirror=[])
    graph = Graph()
    for triple in dataset.triples((None, None, None)):
        graph.add(triple)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "epistemic_case.owl"
        graph.serialize(path, format="xml")
        world = owlready2.World()
        ontology = world.get_ontology(path.as_uri()).load()
        with ontology:
            owlready2.sync_reasoner_hermit(world, infer_property_values=False, debug=0)
        ep = episode_namespace(EPISODE["episode_id"])
        for iri in (ep.Support_bottle, ep.Explanation_H02):
            types = world[str(iri)].INDIRECT_is_a
            assert world[str(DUL.Description)] in types, iri
            assert world[str(DUL.Event)] not in types, (iri, types)
        types = world[str(ep.Accident_1)].INDIRECT_is_a
        assert world[str(DUL.Event)] in types and world[str(SOMA.Accident)] not in types, types
        assert not list(world.inconsistent_classes())
    print("  HermiT: descriptions do not infer actual causal events or physical support states")


def main():
    for check in (check_observation_does_not_assert_physical_loss, check_explanation_is_a_description_with_provenance,
                  check_current_perception_alternative_remains_visible, check_abstention_keeps_runs_without_an_explanation,
                  check_another_change_does_not_borrow_object_loss_facts, check_inferred_types_if_possible):
        check()
        print(f"OK {check.__name__}")
    print("Epistemic RDF: all checks passed")


if __name__ == "__main__":
    main()
