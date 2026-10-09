"""Offline test: the prompt is built from the semantic memory (change B of plan_memoria_semantica.md).

The agent writes the case to the semantic memory (the episode, the change the live validator found
unexplained and the provenance, in the case's named graph) and the generation reads it back with
the TBox's graph. Criterion, fixed before (plan, section 3):
  * the episode read from the memory is the contract-1 record it was written from, field by field
    (absent and null fields aside, and the legacy `participants` and `cause` of an accident, which
    the memory does not assert), and so is the trigger;
  * the prompt is the same, character by character, as the one built from the JSON, on the real
    case of 07/10 at 10:02 with its trigger, the five falls of 24/09 and the synthetic episodes of
    test_explanation_context (the followed target, the minimal one and the incomplete delta);
  * with the same scripted answer, the batch has the same hypotheses (statuses, anchors and
    numbers) as the one published straight from the JSON;
  * written as Turtle into a store and read back as a dataset, as GraphDB does, nothing changes; the
    snapshot's digest does not depend on blank-node labels;
  * the memory refuses to give a case it does not hold, or one written with another TBox.

Needs the recordings of 24/09 (agents/mission_controller/recorded_missions/, in the hand-over
package), as test_episode_contrast.

Run from the repo root:  python3 tests/offline/test_prompt_from_memory.py
"""
import copy
import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SEMANTIC = REPO / "agents" / "semantic"
for path in (SEMANTIC, Path(__file__).resolve().parent, REPO / "experiments"):
    sys.path.insert(0, str(path))

from rdflib import OWL, Dataset, Graph, Literal, URIRef  # noqa: E402

from src.episode_contrast import DEFAULT_TBOX  # noqa: E402
from src.episode_rdf import graph_iri  # noqa: E402
from src.hypothesis_generator import build_prompt, generate_batch, generate_from_memory  # noqa: E402
from src.hypothesis_pipeline import anchored_enumeration, publish_batch  # noqa: E402
from src.semantic_memory import (ONTOLOGY_GRAPH, ONTOLOGY_IRI, SemanticMemoryError, case_graph,  # noqa: E402
                                 local_snapshot, read_case, snapshot_from_dataset)

CASE = SEMANTIC / "generated_hypotheses" / "semantic_unexplained_20261007T080222Z"
TRACKED = [SEMANTIC / "generated_hypotheses" / f"semantic_unexplained_{stamp}"
           for stamp in ("20261006T073328Z", "20261006T110252Z", "20261007T080222Z")]
INST = "http://insight.local/instances#"
DUL = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#"
LOST_BOTTLE = {"unexplained_reason": "Bottle lost location on robot without explicit causal evidence.",
               "removed_triples": [{"subject": INST + "PhysicalObject_Bottle", "predicate": DUL + "hasLocation",
                                    "object": INST + "Agent_Robot"}],
               "added_triples": []}
#: What the memory does not assert of a legacy accident record.
NOT_IN_MEMORY = {"accident": ("participants", "cause")}
SELF_MODEL = "The robot's self-model."


class Scripted:
    def __init__(self, *answers):
        self.answers = list(answers)

    def __call__(self, messages):
        return self.answers.pop(0), {}


def normalized(value):
    """Without absent-equivalent fields: None, {} and []."""
    if isinstance(value, dict):
        return {k: normalized(v) for k, v in value.items() if v is not None and v != {} and v != []}
    if isinstance(value, list):
        return [normalized(v) for v in value]
    return value


def recorded(episode):
    expected = normalized(copy.deepcopy(episode))
    for section, fields in NOT_IN_MEMORY.items():
        for field in fields:
            expected.get(section, {}).pop(field, None)
    return expected


def real_cases():
    """(name, episode, trigger) of the tracked cases of the semantic agent, with their trigger."""
    for case in TRACKED:
        episode = json.loads(Path(f"{case}_episode.json").read_text(encoding="utf-8"))
        trigger = json.loads(Path(f"{case}.json").read_text(encoding="utf-8")).get("trigger") or None
        yield case.name, episode, trigger


def falls_2409():
    from test_episode_contrast import episodes_2409
    return [(f"falls_2409_{stamp}", episode, LOST_BOTTLE) for stamp, episode in episodes_2409().items()]


def synthetic():
    import test_explanation_context as context
    return [("target", context.target_episode(), None),
            ("minimal", {"episode_id": "generic_change", "change": {"summary": "A measured state changed"}}, None),
            ("incomplete", {"episode_id": "incomplete_delta", "change": {
                "operation": "removed", "subject": "Object_X", "predicate": DUL + "hasLocation", "object": "Robot_R"}},
             None)]


def through_store(episode, trigger, case_id, tbox=DEFAULT_TBOX):
    """The case written as Turtle into a store and read back as one dataset, as GraphDB does."""
    store, case = Dataset(), str(graph_iri(episode["episode_id"]))
    store.graph(URIRef(ONTOLOGY_GRAPH)).parse(data=Path(tbox).read_text(encoding="utf-8"), format="turtle")
    store.graph(URIRef(case)).parse(data=case_graph(episode, trigger, case_id, tbox).serialize(format="turtle"),
                                    format="turtle")
    read = Dataset()
    for name in (ONTOLOGY_GRAPH, case):
        read.graph(URIRef(name)).parse(data=store.graph(URIRef(name)).serialize(format="nt"), format="nt")
    return snapshot_from_dataset(read, ONTOLOGY_GRAPH, case)


def check_records_read_back(cases):
    for name, episode, trigger in cases:
        case = read_case(local_snapshot(episode, trigger, case_id=name), name)
        assert normalized(case.episode) == recorded(episode), name
        assert case.trigger == trigger, (name, case.trigger, trigger)
        assert case.record_sha256 and case.provenance["tbox_version"] == "0.3", case.provenance
        assert case.provenance["contracts_version"] == "1.12" and case.provenance["code_commit"], case.provenance


def check_same_prompt(cases):
    for name, episode, trigger in cases:
        from_json = build_prompt(episode, SELF_MODEL, None, trigger=trigger)
        case = read_case(local_snapshot(episode, trigger, case_id=name), name)
        snapshot = local_snapshot(episode, trigger, case_id=name)
        assert build_prompt(case.episode, SELF_MODEL, None, snapshot.tbox, trigger=case.trigger) == from_json, name
        stored = read_case(through_store(episode, trigger, name), name)
        assert build_prompt(stored.episode, SELF_MODEL, None, trigger=stored.trigger) == from_json, name


def check_same_prompt_broader_tbox():
    import test_explanation_context as context
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "broader.ttl"
        path.write_text(context.broader_tbox().serialize(format="turtle"), encoding="utf-8")
        episode = context.target_episode()
        from_json = build_prompt(episode, SELF_MODEL, None, path)
        snapshot = through_store(episode, None, "target", path)
        case = read_case(snapshot, "target")
        assert build_prompt(case.episode, SELF_MODEL, None, snapshot.tbox, trigger=case.trigger) == from_json
        assert "- target_occlusion:" in from_json


def check_same_batch(cases):
    """The batch from the memory, with the anchored enumeration as the answer, against the one
    published straight from the JSON."""
    enumeration = anchored_enumeration()
    with tempfile.TemporaryDirectory() as tmp:
        for name, episode, trigger in cases:
            if not episode.get("segments"):
                continue
            result = generate_from_memory(through_store(episode, trigger, name), Scripted(json.dumps(enumeration)),
                                          model="scripted", output_dir=Path(tmp), case_id=name, budget=None)
            context = result.batch["context_summary"]["explanation_context"]
            direct = publish_batch(episode, copy.deepcopy(enumeration), arm="llm_full", budget=None,
                                   profiles=tuple(context["profile_ids"]))
            assert result.batch["hypotheses"] == direct["hypotheses"], name
            assert result.batch["episode"]["sha256"] == direct["episode"]["sha256"], name
            assert result.batch["mechanisms_version"] == direct["mechanisms_version"], name
            assert result.batch["trigger"] == (trigger or {}), name
            memory = result.batch["semantic_memory"]
            assert memory["graphs"] == [ONTOLOGY_GRAPH, str(graph_iri(episode["episode_id"]))], memory
            assert result.episode is not None and normalized(result.episode) == recorded(episode)
        # generate_batch, which tests and experiments call with an episode, goes through the memory too.
        name, episode, trigger = cases[0]
        result = generate_batch(episode, Scripted(json.dumps(enumeration)), model="scripted", output_dir=Path(tmp),
                                case_id=name, trigger=trigger)
        assert result.batch["semantic_memory"]["source"] == "local"


def check_digest_ignores_blank_node_labels():
    episode, trigger = json.loads(Path(f"{CASE}_episode.json").read_text()), LOST_BOTTLE
    one, two = through_store(episode, trigger, "c"), through_store(episode, trigger, "c")
    assert one.sha256 == two.sha256 == local_snapshot(episode, trigger, case_id="c").sha256
    changed = copy.deepcopy(episode)
    changed["evidence"][0]["value"] += 0.001
    assert through_store(changed, trigger, "c").sha256 != one.sha256


def check_refusals():
    episode = json.loads(Path(f"{CASE}_episode.json").read_text())
    try:
        snapshot_from_dataset(Dataset(), ONTOLOGY_GRAPH, str(graph_iri(episode["episode_id"])))
    except SemanticMemoryError as error:
        assert "no TBox" in str(error), error
    else:
        raise AssertionError("an empty memory gave a case")
    snapshot = local_snapshot(episode, LOST_BOTTLE, case_id="c")
    older = Graph()
    for triple in snapshot.tbox:
        older.add(triple)
    older.set((ONTOLOGY_IRI, OWL.versionInfo, Literal("0.2")))
    try:
        read_case(type(snapshot)(tbox=older, case=snapshot.case, graphs=snapshot.graphs, sha256="", source="test"), "c")
    except SemanticMemoryError as error:
        assert "TBox 0.3" in str(error) and "TBox 0.2" in str(error), error
    else:
        raise AssertionError("a case written with another TBox was read")


def main():
    cases = list(real_cases()) + falls_2409()
    for check in (check_records_read_back, check_same_prompt, check_same_batch):
        check(cases)
        print(f"OK {check.__name__} ({len(cases)} episodes)")
    check_same_prompt(synthetic())
    print("OK check_same_prompt (synthetic: target, minimal, incomplete)")
    for check in (check_same_prompt_broader_tbox, check_digest_ignores_blank_node_labels, check_refusals):
        check()
        print(f"OK {check.__name__}")
    print("Prompt from the semantic memory: all checks passed")


if __name__ == "__main__":
    main()
