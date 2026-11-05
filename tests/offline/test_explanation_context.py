"""Offline regression of case-derived prompts and ontology-selected vocabularies; no network.

Run: python3 tests/offline/test_explanation_context.py
"""

import copy
import json
import sys
import tempfile
from pathlib import Path

from rdflib import Graph, Literal, Namespace, RDF, RDFS, URIRef

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.explanation_context import build_explanation_context  # noqa: E402
from src.hypothesis_generator import build_prompt, feedback, generate_batch  # noqa: E402
from src.hypothesis_pipeline import MECHANISM_ORDER, anchored_enumeration, load_vocabulary  # noqa: E402
from test_hypothesis_generator import ScriptedLLM  # noqa: E402

TBOX = REPO / "agents" / "semantic" / "data" / "insight_tbox.ttl"
INSIGHT = Namespace("http://insight.local/ontology#")
DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
TRACKING = URIRef("urn:test:tracked_target")
PROFILE = INSIGHT.Profile_TargetRelationLoss
MECHANISM = INSIGHT.Mechanism_TargetOcclusion
REAL = REPO / "agents" / "semantic" / "generated_hypotheses" / "semantic_unexplained_20261007T080222Z_episode.json"


def target_episode():
    return {
        "episode_id": "target_discrepancy", "source": {},
        "time": {"t_obs_s": 2.0, "episode_length_s": 3.0, "simulation_horizon_s": 3.0},
        "entities": {"robot": "Observer_R1", "person": "Target_A7"},
        "entity_labels": {"Target_A7": "the followed target A7"},
        "change": {"id": "TargetChange_9", "operation": "removed", "subject": "Target_A7",
                   "predicate": str(TRACKING), "object": "Observer_R1",
                   "summary": "The recorded target association was removed",
                   "mission": "Follow Target_A7", "unknowns": ["The target's current visibility"]},
        "intervals": [{"id": "Interval_observation", "start_s": 1.0, "end_s": 2.0}],
        "segments": [], "phases": [], "evidence": [],
    }


def broader_tbox():
    graph = Graph().parse(TBOX)
    for predicate, value in (
        (RDF.type, INSIGHT.ExplanationProfile), (INSIGHT.profileId, Literal("target_relation_loss")),
        (RDFS.label, Literal("perceived loss of target association")),
        (INSIGHT.matchesOperation, Literal("removed")), (INSIGHT.matchesPredicate, TRACKING),
        (INSIGHT.requiresAffectedRole, Literal("person")),
    ):
        graph.add((PROFILE, predicate, value))
    for predicate, value in (
        (RDF.type, INSIGHT.Mechanism), (INSIGHT.mechanismId, Literal("target_occlusion")),
        (INSIGHT.appliesToProfile, PROFILE), (RDFS.label, Literal("target visibility interruption")),
        (INSIGHT.promptDescription, Literal("Visibility of {affected_entity} could be obstructed.")),
        (INSIGHT.family, Literal("external")), (INSIGHT.costInSimulations, Literal(0)),
        (INSIGHT.notSimulableBecause, Literal("No visual sensor model is available.")),
        (INSIGHT.admitsParameter, INSIGHT.Param_TestExtent),
    ):
        graph.add((MECHANISM, predicate, value))
    graph.add((INSIGHT.Param_TestExtent, INSIGHT.parameterName, Literal("extent")))
    for value in ("partial", "full"):
        graph.add((INSIGHT.Param_TestExtent, INSIGHT.allowedValue, Literal(value)))
    # An inactive profile deliberately has a check that this implementation cannot execute.
    graph.add((INSIGHT.Profile_Future, INSIGHT.profileId, Literal("future_profile")))
    graph.add((INSIGHT.Mechanism_Future, INSIGHT.mechanismId, Literal("future_mechanism")))
    graph.add((INSIGHT.Mechanism_Future, INSIGHT.appliesToProfile, INSIGHT.Profile_Future))
    graph.add((INSIGHT.Mechanism_Future, INSIGHT.hasContrastRule, INSIGHT.Rule_Future))
    graph.add((INSIGHT.Rule_Future, INSIGHT.ruleId, Literal("not_implemented")))
    graph.add((INSIGHT.Rule_Future, INSIGHT.ruleEffect, Literal("inform")))
    graph.add((INSIGHT.Rule_Future, INSIGHT.ruleStatus, Literal("experimental")))
    return graph


def check_existing_delta_and_legacy():
    episode = json.loads(REAL.read_text())
    graph = Graph().parse(TBOX)
    trigger = {"removed_triples": [{"subject": "http://insight.local/instances#" + episode["support"]["supported"],
                                    "predicate": str(DUL.hasLocation),
                                    "object": "http://insight.local/instances#" + episode["entities"]["robot"]}]}
    legacy = build_explanation_context(episode, graph)
    context = build_explanation_context(episode, graph, trigger)
    assert legacy["profile_ids"] == context["profile_ids"] == ["carried_object_relation_loss"]
    assert context["changes"][0]["operation"] == "removed"
    unrelated_namespace = copy.deepcopy(trigger)
    unrelated_namespace["removed_triples"][0]["subject"] = "urn:other#" + episode["support"]["supported"]
    assert build_explanation_context(episode, graph, unrelated_namespace)["profile_ids"] == []
    prompt = build_prompt(episode, "Capabilities only", trigger=trigger)
    task = prompt[prompt.index("## Your task"):]
    for phrase in ("The bottle fell", "explain the fall", "before the fall", "the bottle leaving the tray"):
        assert phrase not in prompt, phrase
    # Facts and declared knowledge only: no instruction about what to conclude.
    for phrase in ("does not establish", "not establish", "is not established", "not an independently measured",
                   "does not locate", "Do not turn", "not proof", "free motion", "Identifiers are recorded labels",
                   "proximity alone", "Neither absent nor renewed", "Do not borrow", "physical fall"):
        assert phrase not in prompt, phrase
    # Ids that interpret the change are shown neutral; entities by label, not by IRI.
    assert context["display_ids"] == {"Accident_1": "Observation_1", "Interval_fall": "Interval_before_observation"}
    assert "Accident_1" not in prompt and "Interval_fall" not in prompt
    assert "- Interval_before_observation = [" in prompt and "over Interval_before_observation" in prompt
    assert "subject=PhysicalObject_Bottle; relation=hasLocation; object=Agent_Robot." in prompt
    assert "http://" not in prompt[:prompt.index("## The mechanisms")]
    assert "PhysicalObject_Tray (supporter)" in prompt and "PhysicalObject_Bottle (supported, affected entity)" in prompt
    assert "deletion of the robot->bottle RT edge" in prompt
    assert "PhysicalObject_Bottle" in prompt and "PhysicalObject_Tray" in prompt
    assert "Explain the observed discrepancy Observation_1" in task
    assert "bottle" not in task.lower() and "tray" not in task.lower()
    assert "Interval_reaction" not in prompt
    assert "No removal intervention is implemented" in prompt
    assert "Not physical" not in prompt

    # Roles, not object names in Python, determine the description on another carried payload.
    renamed = copy.deepcopy(episode)
    renamed["support"]["supported"] = "Medicine_Box_42"
    renamed["entities"]["payload"] = renamed["entities"].pop("bottle")
    renamed["entities"]["payload"] = "Medicine_Box_42"
    renamed["entity_labels"] = {"Medicine_Box_42": "the transported medicine box"}
    rendered = build_prompt(renamed, "Capabilities only")
    cards = rendered[rendered.index("## The mechanisms"):rendered.index("## The robot")]
    assert "the transported medicine box" in cards and "PhysicalObject_Bottle" not in cards


def check_other_change_and_active_validation():
    graph = broader_tbox()
    episode = target_episode()
    context = build_explanation_context(episode, graph)
    assert context["profile_ids"] == ["target_relation_loss"], context
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "broader.ttl"
        path.write_text(graph.serialize(format="turtle"))
        prompt = build_prompt(episode, "Observer can track targets", tbox_path=path)
        assert "TargetChange_9" in prompt and "Follow Target_A7" in prompt
        assert "Visibility of the followed target A7" in prompt
        assert "bottle" not in prompt.lower() and "tray" not in prompt.lower()
        assert "- target_occlusion:" in prompt and "- obstacle_traversed:" not in prompt
        assert "future_mechanism" not in prompt
        assert list(load_vocabulary(path, ("target_relation_loss",))) == ["target_occlusion"]
        assert list(load_vocabulary(path, ("carried_object_relation_loss",))) == list(MECHANISM_ORDER)
        assert {h["mechanism"] for h in anchored_enumeration(path, ("target_relation_loss",))["hypotheses"]} == {
            "target_occlusion"}

        borrowed = {"hypotheses": [{"mechanism": "robot_push", "qualitative_parameters": {"direction": "any"}}]}
        valid = {"hypotheses": [{"mechanism": "target_occlusion", "new_mechanism_description": None,
                                "segment": None, "interval": None, "qualitative_parameters": {"extent": "partial"},
                                "expected_trace": [], "rationale": "The target association disappeared", "title": "Occlusion"}]}
        llm = ScriptedLLM(json.dumps(borrowed), json.dumps(valid))
        result = generate_batch(episode, llm, model="scripted", output_dir=Path(tmp), tbox_path=path)
        assert result.ok and result.batch["attempts"] == 2
        assert "unknown mechanism 'robot_push'" in llm.seen[1][-1]["content"]
        assert result.batch["hypotheses"][0]["status"] == "not_simulable"
        assert result.batch["context_summary"]["mechanism_ids"] == ["target_occlusion"]
        assert result.batch["context_summary"]["explanation_context"]["profile_ids"] == ["target_relation_loss"]
        assert result.prompt_path.read_text().strip() == llm.seen[0][0]["content"].strip()

        # Unsupported checks fail when their profile is actually selected; no fabricated execution.
        try:
            load_vocabulary(path, ("future_profile",))
        except ValueError as error:
            assert "does not implement" in str(error), error
        else:
            raise AssertionError("an active check with no implementation was accepted")


def check_unknown_and_missing_context():
    graph = Graph().parse(TBOX)
    episode = target_episode()
    context = build_explanation_context(episode, graph)
    assert context["profile_ids"] == []
    prompt = build_prompt(episode, "Observer")
    assert "The ontology declares no mechanism for this kind of change." in prompt and "- robot_push:" not in prompt
    assert "- new:" in prompt

    # An explicit unrelated observation must not inherit the legacy object's profile.
    recording = json.loads(REAL.read_text())
    recording["change"] = episode["change"]
    assert build_explanation_context(recording, graph)["profile_ids"] == []
    minimal = {"episode_id": "generic_change", "change": {"summary": "A measured state changed"}}
    minimal_prompt = build_prompt(minimal, "Observer")
    assert "A measured state changed" in minimal_prompt and "observation time = unknown" in minimal_prompt
    assert "bottle" not in minimal_prompt.lower()
    unlisted = {"hypotheses": [{"mechanism": "new", "new_mechanism_description": "An unmodeled state transition",
                               "segment": None, "interval": None, "qualitative_parameters": {},
                               "expected_trace": [], "rationale": "The recorded state changed", "title": "State change"}]}
    with tempfile.TemporaryDirectory() as tmp:
        result = generate_batch(minimal, ScriptedLLM(json.dumps(unlisted)), model="scripted", output_dir=Path(tmp))
        assert result.ok and result.batch["hypotheses"][0]["status"] == "not_simulable"
        assert result.batch["budget"]["used"] == 0

    # Missing role data is unknown, not a contradiction; applicability stays provisional.
    incomplete = {"episode_id": "incomplete_delta", "change": {
        "operation": "removed", "subject": "Object_X", "predicate": str(DUL.hasLocation), "object": "Robot_R"}}
    context = build_explanation_context(incomplete, graph)
    assert context["profile_ids"] == ["carried_object_relation_loss"] and context["selection_notes"]
    # Selection notes are provenance of the batch, not text for the LLM.
    assert "provisional" not in build_prompt(incomplete, "Observer")


def check_neutral_ids_read_back():
    """An answer with the neutral ids reaches the batch with the recorded ones; feedback shows the neutral."""
    episode = json.loads(REAL.read_text())
    answer = {"hypotheses": [
        {"mechanism": "bottle_push", "new_mechanism_description": None, "segment": None,
         "interval": "Interval_before_observation", "qualitative_parameters": {"direction": "any"},
         "expected_trace": [], "rationale": "A push", "title": "Push"},
        {"mechanism": "slippery_floor", "new_mechanism_description": None, "segment": None,
         "interval": "Interval_before_observation", "qualitative_parameters": {},
         "expected_trace": [], "rationale": "Slip", "title": "Slip"}]}
    with tempfile.TemporaryDirectory() as tmp:
        result = generate_batch(episode, ScriptedLLM(json.dumps(answer)), model="scripted", output_dir=Path(tmp))
    assert result.ok and result.batch["attempts"] == 1, result.batch.get("errors")
    push, slip = result.batch["hypotheses"]
    assert push["anchors"]["interval"] == "Interval_fall" and push["status"] != "incoherent", push
    # Read back before the checks: the memory's reasons name the recorded ids.
    assert slip["status"] == "incoherent" and "got Interval_fall" in slip["checks"]["coherence"]["reason"], slip
    assert result.batch["context_summary"]["explanation_context"]["observation_id"] == "Accident_1"
    shown = feedback(["slippery_floor admits no interval anchor, got Interval_fall"],
                     result.batch["context_summary"]["explanation_context"]["display_ids"])
    assert "got Interval_before_observation" in shown and "Interval_fall" not in shown, shown


def main():
    for check in (check_existing_delta_and_legacy, check_other_change_and_active_validation,
                  check_unknown_and_missing_context, check_neutral_ids_read_back):
        check()
        print(f"OK {check.__name__}")
    print("Explanation context: all checks passed")


if __name__ == "__main__":
    main()
