"""Offline test: the semantic agent with the generation v2 (task A4, subpaso 3.3), without robot.

The agent's own code (agents/semantic/src/specificworker.py) runs on a simulated DSR (the working
and the episodic graphs), a simulated GraphDB and a scripted LLM, over the 12:29 recording of
2026-09-24. Criterion of subpaso 3.3, fixed before:
  1. the agent builds, with the episodic-memory API, the same episode as the file reader;
  2. it publishes a valid 3b batch and leaves its path where the inner simulator reads it (the
     `hypotheses_filepath` attribute of the `unexplained` intention node);
  3. the simulator's compiler takes that batch;
  4. with a verdict that accepts the bump (a dome of unknown size), GraphDB gets the case explained by obstacle_traversed on
     Segment_final, with the title written from the mechanism; what the consolidation wrote before
     in the live graph is written as it was, and nothing is removed from it.
Also: the agent waits for the recording as the inner simulator does (a running "Search Problem
Cause" mission and the "Follow Person" file path), keeps the episode in its own named graph, and
ignores a verdict that does not answer its batch (the simulator's static causes, when it finds
the recording before the batch is out).

GraphDB is written off the agent's main thread (09/10): with GraphDB slow to answer (5 s timeouts,
with the repository's inference on), the robot stopped 5 s after losing the bottle. The lost bottle
is now found unexplained while GraphDB has not answered; the writes land in the order they were
queued, a failed one is retried, and the next sync of the mirror makes up for one given up.

GraphDB does not pile up one case per run: when a case starts, the live graph goes back to the
mirror of the working memory alone and the episode graphs of previous cases are dropped; what the
case writes stays until the next one (a second case, on the 12:40 recording, finds none of the
first). Other named graphs are left alone.

What the memory decides is kept in the memory (contracts 1.8; criterion in
docs_output/resultados/20_memoria_en_graphdb/CRITERIO.md):
  * the INSIGHT TBox is loaded into its own graph at start-up, and again at the next case if
    GraphDB was not reachable; no reset touches it;
  * when the batch is out, every hypothesis is in the episode's graph with the batch's status, and
    a discarded or incoherent one with the check that decided it (and the observation it read);
  * the verdict adds the nominal run and each simulated one, also when nothing is accepted, and
    the supported explanation links the observation, hypothesis and simulation;
  * the episode's graph only uses TBox terms.

Two faults of the first run in Webots (06/10) are checked too:
  * the loop runs under a decimal-comma numeric locale, as in the agent, where Qt takes the
    locale from the environment: the episodic-memory API then read "-3.699979" as -3, and the
    episode lost every decimal;
  * every slot the agent connects to a DSR signal is annotated as pydsr requires: pydsr picks the
    signal a callback serves from its annotations, and update_node_att, with an unannotated
    `attribute_names`, was taken for a node-deletion handler, which killed the agent when it
    deleted the `unexplained` node.

Needs PySide6, pydsr and the episodic-memory API (/opt/robocomp/lib), Ice and RoboComp's
ConfigLoader; without them the test is skipped. The recordings are not in git
(agents/mission_controller/recorded_missions/), nor is experiments/: both travel in the hand-over
package.

Run from the repo root:  python3 tests/offline/test_semantic_agent_v2.py
"""
import inspect
import json
import locale
import re
import sys
import tempfile
import threading
import time
import types
from decimal import Decimal
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SEMANTIC = REPO / "agents" / "semantic"
RECORDINGS = REPO / "agents" / "mission_controller" / "recorded_missions"
RECORDING_1229 = RECORDINGS / "mission_Follow_Person_24092026_122924.txt"
RECORDING_1240 = RECORDINGS / "mission_Follow_Person_24092026_124030.txt"
TBOX = SEMANTIC / "data" / "insight_tbox.ttl"
for path in (Path("/home/javi/robocomp/core/classes/ConfigLoader"), SEMANTIC / "generated", Path("/opt/robocomp/lib"),
             REPO / "agents" / "inner_simulator" / "src", REPO / "experiments", Path(__file__).resolve().parent,
             SEMANTIC):
    sys.path.insert(0, str(path))

try:
    from PySide6 import QtCore  # noqa: F401  (before pydsr: both load Qt)
    import pydsr  # noqa: F401
    from src import specificworker
except ImportError as error:
    print(f"SKIP: the semantic agent cannot be imported here ({error})")
    sys.exit(0)

from hypothesis_compiler import compile_batch  # noqa: E402
from rdflib import OWL, RDF, RDFS, Dataset, Graph, Literal, Namespace, URIRef  # noqa: E402
from src.episode_memory_reader import recording_to_explain  # noqa: E402
from src.episode_rdf import EPISODES, graph_iri  # noqa: E402
from src.memory_queries import ONTOLOGY_GRAPH  # noqa: E402
from src.graphdb_client import GraphDBClient, GraphDBConfig  # noqa: E402
from src.graphdb_writer import GraphDBWriter  # noqa: E402
from src.live_causal_validator import LiveCausalValidator  # noqa: E402
from src.ontology_mapping import AGENT_ROBOT, PHYSICAL_OBJECT_BOTTLE  # noqa: E402
from src.hypothesis_config import ConfigError, HypothesisGeneratorConfig  # noqa: E402
from src.verdict_ingestor import build_case_triples  # noqa: E402
from test_episode_builder import differences  # noqa: E402

DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
INSIGHT = Namespace("http://insight.local/ontology#")
INST = Namespace("http://insight.local/instances#")


# --------------------------------------------------------------------------- #
# A simulated DSR, GraphDB and LLM
# --------------------------------------------------------------------------- #
def node(name, node_id, **attrs):
    return types.SimpleNamespace(name=name, type="intention", id=node_id,
                                 attrs={k: types.SimpleNamespace(value=v) for k, v in attrs.items()})


class FakeGraph:
    """The calls the agent makes on a DSR graph."""

    def __init__(self, *nodes):
        self.nodes = {n.name: n for n in nodes}
        self.next_id = 100

    def get_node(self, key):
        return next((n for n in self.nodes.values() if key in (n.name, n.id)), None)

    def get_nodes(self):
        return list(self.nodes.values())

    def insert_node(self, dsr_node):
        self.next_id += 1
        self.nodes[dsr_node.name] = types.SimpleNamespace(name=dsr_node.name, type=dsr_node.type, id=self.next_id,
                                                          attrs={})
        return self.next_id

    def update_node(self, updated):
        self.nodes[updated.name] = updated
        return True

    def insert_or_assign_edge(self, edge):
        return True

    def delete_node(self, node_id):
        self.nodes = {name: n for name, n in self.nodes.items() if n.id != node_id}
        return True


class FakeGraphDB:
    """The live graph as a set of triples, and the named graphs in a dataset."""

    def __init__(self):
        self.live, self.removed, self.dataset = set(), set(), Dataset()
        self.down = False

    def _reachable(self):
        if self.down:
            raise ConnectionError("GraphDB is not reachable")

    def apply_delta(self, *, added, removed):
        self._reachable()
        self.live |= set(added)
        self.live -= set(removed)
        self.removed |= set(removed)

    def replace_graph(self, turtle_payload, graph=None):
        self._reachable()
        context = self.dataset.graph(URIRef(graph))
        context.remove((None, None, None))
        context.parse(data=turtle_payload, format="turtle")

    def add_to_graph(self, turtle_payload, graph):
        self._reachable()
        self.dataset.graph(URIRef(graph)).parse(data=turtle_payload, format="turtle")

    def replace_with_triples(self, triples, graph=None):
        self._reachable()
        assert graph is None, "only the live graph is replaced with triples"
        self.live = set(triples)

    def drop_graphs(self, prefix):
        self._reachable()
        for context in list(self.dataset.contexts()):
            if str(context.identifier).startswith(prefix):
                self.dataset.remove_graph(context)

    def episode_graphs(self):
        return {str(c.identifier) for c in self.dataset.contexts() if str(c.identifier).startswith(EPISODES) and len(c)}


#: What the agent mirrors of the working memory (a few of the 9 triples).
MIRROR = {(str(INST.Agent_Robot), str(DUL.hasLocation), str(INST.PhysicalPlace_Room)),
          (str(INST.PhysicalObject_Bottle), str(DUL.hasLocation), str(INST.Agent_Robot))}


class ScriptedLLM:
    def __init__(self, answer):
        self.answer, self.calls = answer, 0

    def __call__(self, messages):
        self.calls += 1
        return self.answer, {}


ANSWER = json.dumps({"hypotheses": [
    {"mechanism": "obstacle_traversed", "segment": "Segment_final", "interval": None,
     "qualitative_parameters": {"shape": "bump"}, "expected_trace": ["pitch_rate_peak"],
     "rationale": "A pitch peak just before the loss.", "title": "Bump"},
    {"mechanism": "bottle_push", "segment": None, "interval": "Interval_fall",
     "qualitative_parameters": {"direction": "any"}, "expected_trace": [], "rationale": "", "title": "Push"},
    # The person never came within reach (contrast), and a segment the episode does not have (coherence).
    {"mechanism": "bottle_removed_by_person", "segment": None, "interval": "Interval_fall",
     "qualitative_parameters": {}, "expected_trace": ["person_distance_min"], "rationale": "", "title": "Taken"},
    {"mechanism": "obstacle_traversed", "segment": "Segment_prev_9", "interval": None,
     "qualitative_parameters": {"shape": "bump"}, "expected_trace": [], "rationale": "", "title": "Far bump"},
]})

WORKER_METHODS = ("compute", "deactivate_follow_affordance", "insert_intention_hanging_for_robot",
                  "generate_hypotheses_json", "_generate_from_episode", "_store_episode", "_trigger_summary",
                  "_publish_path_attribute", "_check_and_ingest_verdict", "_resolve_unexplained_cycle",
                  "_log_hypothesis_event", "_reset_previous_cases", "_load_ontology", "_store_decisions")


def make_worker(output_dir):
    """The agent's state after an unexplained change, with its own methods bound to it."""
    config = HypothesisGeneratorConfig(
        enabled=True, output_dir=Path(output_dir), description_path=REPO / "description.md",
        catalog_path=REPO / "etc" / "intervention_catalog.json", primary_model="scripted", fallback_model="",
        ollama_base_url="", request_timeout_seconds=1.0, internal_count=3, external_count=3,
        preferred_client="ollama_http", description_char_limit=12000, generation="v2", budget=6, max_attempts=3)
    db = FakeGraphDB()
    worker = types.SimpleNamespace(
        g=FakeGraph(node("robot", 1)), episodic_g=FakeGraph(), agent_id=9, graphdb_client=db,
        graphdb_writer=GraphDBWriter(db, lambda message, style: None, retry_seconds=0.01),
        hypothesis_config=config, hypothesis_service=None, llm=ScriptedLLM(ANSWER),
        unexplained=True, unexplained_reason="Bottle lost location on robot without explicit causal evidence.",
        trigger_added=frozenset(), trigger_removed=frozenset(), stop_inserted=False,
        hypothesis_generation_done=False, last_hypotheses_path=None, hypotheses_path_published=False,
        ingested_verdict_paths=set(), _parsed_ingestions={},
        current_batch=None, current_episode=None, _waiting_for_recording=False, _foreign_verdict_paths=set(),
        _ontology_loaded=False, graphdb_config=types.SimpleNamespace(ontology_graph=ONTOLOGY_GRAPH),
        mapper=types.SimpleNamespace(get_state=lambda: types.SimpleNamespace(
            triples=frozenset(MIRROR), signature=tuple(sorted(MIRROR)))),
        _sync_lock=threading.Lock(), last_triples=set(), last_signature=None)
    for name in WORKER_METHODS:
        setattr(worker, name, types.MethodType(getattr(specificworker.SpecificWorker, name), worker))
    return worker


def compute(worker):
    """One cycle of the agent, and then its GraphDB writes, which it does not wait for."""
    worker.compute()
    worker.graphdb_writer.join()


def publish_verdict(worker, path, verdict):
    Path(path).write_text(json.dumps(verdict), encoding="utf-8")
    intention = worker.g.get_node("unexplained")
    intention.attrs["verdict_filepath"] = types.SimpleNamespace(value=str(path))


# --------------------------------------------------------------------------- #
# Checks
# --------------------------------------------------------------------------- #
def check_recording_rule():
    follow = node("Follow Person-1", 10, status="stopped", filepath="/rec/a.txt")
    assert recording_to_explain([follow]) is None                                   # no search mission
    assert recording_to_explain([follow, node("Search Problem Cause-1", 11, status="pending")]) is None
    running = node("Search Problem Cause-1", 11, status="running")
    assert recording_to_explain([follow, running]) == "/rec/a.txt"
    later = node("Follow Person-2", 12, status="stopped", filepath="/rec/b.txt")
    assert recording_to_explain([follow, later, running]) == "/rec/b.txt"            # the last one, as the simulator
    assert recording_to_explain([node("Follow Person-3", 13, status="stopped", filepath=""), running]) is None
    # Since 08/10 mission_controller names the automatic missions "..._attempt_N"; both schemes count.
    renamed = [node("follow_person_attempt_1-20261008-1", 14, status="stopped", filepath="/rec/c.txt"),
               node("search_cause_attempt_1-20261008-2", 15, status="running")]
    assert recording_to_explain(renamed) == "/rec/c.txt"
    try:
        HypothesisGeneratorConfig.from_config({"hypothesisGenerator": {
            "PrimaryModel": "m", "OllamaBaseUrl": "u", "PreferredClient": "c", "OutputDir": "o",
            "DescriptionPath": "d", "CatalogPath": "c", "RequestTimeoutSeconds": 1, "InternalCount": 3,
            "ExternalCount": 3, "DescriptionCharLimit": 12000, "Generation": "v3"}}, REPO)
    except ConfigError:
        pass
    else:
        raise AssertionError("an unknown generation was accepted")


#: A numeric locale with a decimal comma, as the agent runs on Javi's machine (es_ES).
COMMA_LOCALES = ("es_ES.UTF-8", "es_ES.utf8", "de_DE.UTF-8", "fr_FR.UTF-8")


def comma_locale():
    """Switch LC_NUMERIC to a decimal-comma locale; the previous one, or None if none is installed."""
    previous = locale.setlocale(locale.LC_NUMERIC)
    for name in COMMA_LOCALES:
        try:
            locale.setlocale(locale.LC_NUMERIC, name)
            return previous
        except locale.Error:
            continue
    return None


def check_agent_loop():
    from episode_series import episode_from_recording

    previous = comma_locale()
    if previous is None:
        print("  (no decimal-comma locale installed: the loop runs under the current one)")
    try:
        run_agent_loop(episode_from_recording)
    finally:
        if previous is not None:
            locale.setlocale(locale.LC_NUMERIC, previous)


#: Signal -> annotations pydsr needs to route a callback to it (signal_function_caster.h).
SLOT_ANNOTATIONS = {
    "UPDATE_NODE": ["<class 'int'>", "<class 'str'>"],
    "UPDATE_NODE_ATTR": ["<class 'int'>", "[<class 'str'>]"],
    "UPDATE_EDGE": ["<class 'int'>", "<class 'int'>", "<class 'str'>"],
    "UPDATE_EDGE_ATTR": ["<class 'int'>", "<class 'int'>", "<class 'str'>", "[<class 'str'>]"],
    "DELETE_EDGE": ["<class 'int'>", "<class 'int'>", "<class 'str'>"],
    "DELETE_NODE": ["<class 'int'>"],
}


def check_slot_annotations():
    source = inspect.getsource(specificworker.SpecificWorker.__init__)
    connections = re.findall(r"^\s*signals\.connect\(self\.g, signals\.(\w+), self\.(\w+)\)", source, re.MULTILINE)
    assert connections, "no DSR signal connected"
    for signal, slot in connections:
        annotations = getattr(specificworker.SpecificWorker, slot).__annotations__
        got = [str(value) for name, value in annotations.items() if name != "return"]
        assert got == SLOT_ANNOTATIONS[signal], (signal, slot, got)
    # The attribute slots, connected or not, are annotated for their own signal.
    for slot, signal in (("update_node_att", "UPDATE_NODE_ATTR"), ("update_edge_att", "UPDATE_EDGE_ATTR")):
        annotations = getattr(specificworker.SpecificWorker, slot).__annotations__
        assert [str(v) for k, v in annotations.items() if k != "return"] == SLOT_ANNOTATIONS[signal], slot


def run_agent_loop(episode_from_recording):
    with tempfile.TemporaryDirectory() as tmp:
        worker = make_worker(tmp)
        follow = node("Follow Person-24092026", 10, status="running", filepath=str(RECORDING_1229))
        worker.episodic_g = FakeGraph(follow)
        # GraphDB still holds a previous case, and a graph that is not the agent's.
        db = worker.graphdb_client
        db.live = MIRROR | build_case_triples("wheel", "old_case")
        db.dataset.graph(URIRef(EPISODES + "rec_old")).add((URIRef(EPISODES + "rec_old#Episode"), RDF.type,
                                                             INSIGHT.Episode))
        other = db.dataset.graph(URIRef("urn:other:graph"))
        other.add((INST.Agent_Robot, RDF.type, DUL.PhysicalAgent))
        # At start-up GraphDB is not reachable: the TBox is not loaded, and the agent goes on.
        db.down = True
        worker._load_ontology()
        worker.graphdb_writer.join()
        db.down = False
        assert not worker._ontology_loaded and not len(db.dataset.graph(URIRef(ONTOLOGY_GRAPH)))
        tbox_size = len(Graph().parse(TBOX))

        # The change is unexplained, but the follow mission has not stopped: the agent waits.
        compute(worker)
        assert worker.g.get_node("unexplained") is not None
        assert not worker.hypothesis_generation_done and worker.last_hypotheses_path is None
        assert worker.llm.calls == 0
        assert db.episode_graphs() == {EPISODES + "rec_old"}                # nothing cleared yet

        # The mission stops and the search starts: the episode, the batch and its path on the DSR.
        follow.attrs["status"].value = "stopped"
        worker.episodic_g.nodes["Search Problem Cause-1"] = node("Search Problem Cause-1", 11, status="running")
        compute(worker)
        assert worker.llm.calls == 1
        intention = worker.g.get_node("unexplained")
        published = intention.attrs["hypotheses_filepath"].value
        batch = json.loads(Path(published).read_text(encoding="utf-8"))
        assert batch["schema_version"] == "2.0" and batch["status"] == "success" and batch["arm"] == "llm_full"
        assert batch["case_id"].startswith("semantic_unexplained_") and batch["attempts"] == 1
        assert batch["trigger"]["unexplained_reason"].startswith("Bottle lost location")

        # 1. The episode is the one of the file reader.
        episode = json.loads(Path(batch["episode"]["path"]).read_text(encoding="utf-8"))
        assert episode["source"]["reader"] == "episodic_memory_api"
        from_file = episode_from_recording(RECORDING_1229)
        for one in (episode, from_file):
            one["source"].pop("reader")
            one["source"].pop("path")
        found = differences(from_file, episode)
        assert not found, found[:5]
        assert batch["episode"]["id"] == episode["episode_id"] == "rec_mission_Follow_Person_24092026_122924"

        # The case starts from a clean GraphDB: the mirror alone, and no previous episode. The TBox,
        # which could not be loaded at start-up, is there now, in its own graph.
        assert worker._ontology_loaded and len(db.dataset.graph(URIRef(ONTOLOGY_GRAPH))) == tbox_size
        assert db.live == MIRROR, db.live - MIRROR
        assert worker.graphdb_writer._mirror == MIRROR
        assert len(db.dataset.graph(URIRef("urn:other:graph"))) == 1        # not the agent's: left alone
        # The episode is kept in the semantic memory, in its own named graph.
        episode_iri = graph_iri(episode["episode_id"])
        assert db.episode_graphs() == {str(episode_iri)}
        stored = worker.graphdb_client.dataset.graph(episode_iri)
        ep = Namespace(str(episode_iri) + "#")
        assert (ep.Accident_1, INSIGHT.observedAtSegment, ep.Segment_final) in stored
        assert (ep.Accident_1, RDF.type, INSIGHT.ObservedAnomaly) in stored
        assert (ep.Support_bottle, RDF.type, INSIGHT.SupportEstimate) in stored
        # Next to it, what the memory decided about each hypothesis: the batch's status ...
        statuses = {str(i): str(s) for i, s in (Graph().parse(TBOX) + stored).query("""
            PREFIX dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
            PREFIX insight: <http://insight.local/ontology#>
            SELECT ?i ?s WHERE { ?h a insight:Hypothesis ; insight:hypothesisId ?i ; dul:isClassifiedBy/insight:statusId ?s }""")}
        assert statuses == {h["hypothesis_id"]: h["status"] for h in batch["hypotheses"]}, statuses
        assert statuses == {"H01": "to_simulate", "H02": "to_simulate", "H03": "discarded", "H04": "incoherent"}
        # ... and the check that decided it, with the observation it read.
        assert (ep.Hypothesis_H03, INSIGHT.checkedBy, ep.Check_H03_person_within_reach) in stored
        assert (ep.Check_H03_person_within_reach, INSIGHT.appliesCheck, INSIGHT.rule_person_within_reach) in stored
        assert (ep.Check_H03_person_within_reach, INSIGHT.checkVerdict, Literal("discards")) in stored
        assert (ep.Check_H03_person_within_reach, INSIGHT.readsObservation, ep.Ev_person_distance) in stored
        assert any(stored.triples((ep.Hypothesis_H04, INSIGHT.checkedBy, None)))
        assert {o for c in stored.objects(ep.Hypothesis_H04, INSIGHT.checkedBy)
                for o in stored.objects(c, INSIGHT.checkVerdict)} == {Literal("fails")}
        assert (ep.Hypothesis_H01, DUL.satisfies, INSIGHT.Mechanism_ObstacleTraversed) in stored
        assert (ep.Hypothesis_H01, INSIGHT.anchoredTo, ep.Segment_final) in stored
        assert (ep.Hypothesis_H01, DUL.isClassifiedBy, INSIGHT.Kind_shape_bump) in stored

        # 2 and 3. The simulator's compiler takes the published batch.
        compiled = compile_batch(batch)
        to_simulate = [h["hypothesis_id"] for h in batch["hypotheses"] if h["status"] == "to_simulate"]
        assert [e["hypothesis_id"] for e in compiled["entries"][1:]] == to_simulate, compiled["skipped"]
        # The bump is one dome of unknown size (cost 4) and the push one cause: 5 of 6.
        assert to_simulate == ["H01", "H02"] and batch["budget"]["used"] == 5, to_simulate
        assert compiled["entries"][1]["cause"]["name"] == "bump_scaled"
        assert db.live == MIRROR

        # A verdict that does not answer our batch (the simulator's static causes) is ignored.
        publish_verdict(worker, Path(tmp) / "static_verdict.json",
                        {"case_id": "static_causes", "accepted_hypothesis_id": None, "hypotheses": []})
        compute(worker)
        assert worker.unexplained and worker.g.get_node("unexplained") is not None
        assert db.live == MIRROR

        # 4. The verdict on our batch accepts the true bump.
        verdict = {"case_id": batch["case_id"], "accepted_hypothesis_id": "H01",
                   "hypotheses": [{"hypothesis_id": e["hypothesis_id"], "cause": e["cause"]}
                                  for e in compiled["entries"]]}
        publish_verdict(worker, Path(tmp) / "verdict.json", verdict)
        compute(worker)
        # What the consolidation wrote before, as it was; nothing removed.
        assert db.live == MIRROR | build_case_triples("bump_scaled", batch["case_id"])
        assert not db.removed
        # The simulation-supported explanation, in the episode's graph.
        stored = worker.graphdb_client.dataset.graph(episode_iri)
        case = INST[f"Case_{batch['case_id']}"]
        cause = ep["Explanation_H01"]
        assert (case, INST.explainedBy, cause) in stored
        assert (cause, DUL.isDescribedBy, INSIGHT.Mechanism_ObstacleTraversed) in stored
        assert (cause, RDF.type, INSIGHT.SimulationSupportedExplanation) in stored
        assert (ep.Hypothesis_H01, INSIGHT.anchoredTo, ep.Segment_final) in stored
        assert (cause, RDFS.label, Literal("Undetected bump on the last 1 m of the path before the fall")) in stored
        # It selects a hypothesis and links its run, without asserting a physical causal event.
        assert (cause, INSIGHT.explainsObservation, ep.Accident_1) in stored
        assert (cause, INSIGHT.selectedHypothesis, ep.Hypothesis_H01) in stored
        assert (cause, INSIGHT.supportedBySimulation, ep.Run_H01) in stored
        assert not any(stored.triples((None, URIRef("http://www.ease-crc.org/ont/SOMA.owl#causes"), None)))
        runs = set(stored.subjects(RDF.type, INSIGHT.SimulationRun))
        assert runs == {ep.Run_nominal, ep.Run_H01, ep.Run_H02}, runs
        assert (ep.Run_H01, INSIGHT.testsHypothesis, ep.Hypothesis_H01) in stored
        assert (ep.Run_nominal, INSIGHT.isNominal, Literal(True)) in stored
        # Asked as a question, with the TBox: which mechanism explains the case, and where.
        merged = Graph().parse(TBOX)
        for triple in stored:
            merged.add(triple)
        rows = list(merged.query("""
            PREFIX dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
            PREFIX insight: <http://insight.local/ontology#>
            PREFIX inst: <http://insight.local/instances#>
            SELECT ?mechanism ?where ?t WHERE {
              ?case inst:explainedBy ?explanation .
              ?explanation a insight:SimulationSupportedExplanation ; dul:isDescribedBy ?m ;
                    insight:selectedHypothesis ?hypothesis ; insight:explainsObservation ?observed .
              ?hypothesis insight:anchoredTo ?place . ?m insight:mechanismId ?mechanism .
              ?place dul:hasRegion ?region . ?region insight:toX ?where .
              ?observed a insight:ObservedAnomaly ; insight:timeS ?t }"""))
        assert [(str(m), float(x), float(t)) for m, x, t in rows] == [("obstacle_traversed", -0.507, 13.685)], rows
        # Only TBox terms in the whole episode graph: the episode, the decisions, the runs, the cause.
        tbox = Graph().parse(TBOX)
        properties = {s for kind in (OWL.ObjectProperty, OWL.DatatypeProperty) for s in tbox.subjects(RDF.type, kind)}
        classes = set(tbox.subjects(RDF.type, OWL.Class))
        assert {p for _, p, _ in stored} - {RDF.type, RDFS.label, RDFS.comment} <= properties, (
            {p for _, p, _ in stored} - properties)
        assert {o for _, p, o in stored if p == RDF.type} - {OWL.NamedIndividual} <= classes
        # The cycle is closed; what the case wrote stays until the next one.
        assert not worker.unexplained and worker.g.get_node("unexplained") is None and worker.current_batch is None
        assert db.episode_graphs() == {str(episode_iri)} and case_triples_in(db.live) == {str(case)}

        # A second case, on the 12:40 recording: it finds nothing of the first.
        worker.unexplained = True
        worker.llm = ScriptedLLM(ANSWER)
        follow.attrs["filepath"].value = str(RECORDING_1240)
        compute(worker)
        second = json.loads(Path(worker.g.get_node("unexplained").attrs["hypotheses_filepath"].value)
                            .read_text(encoding="utf-8"))
        assert second["episode"]["id"] == "rec_mission_Follow_Person_24092026_124030"
        assert db.episode_graphs() == {str(graph_iri(second["episode"]["id"]))}
        assert db.live == MIRROR and not case_triples_in(db.live)
        assert len(db.dataset.graph(URIRef(ONTOLOGY_GRAPH))) == tbox_size      # no reset touches the TBox

        # A verdict that accepts nothing: the live graph is not touched, but what was simulated stays.
        publish_verdict(worker, Path(tmp) / "verdict_none.json",
                        {"case_id": second["case_id"], "accepted_hypothesis_id": None,
                         "hypotheses": [{"hypothesis_id": "__nominal__", "cause": {"name": "none"},
                                         "repetitions": 10, "effect_rate": 0.0, "accepted": False}]})
        compute(worker)
        assert not worker.unexplained and db.live == MIRROR
        second_ep = Namespace(str(graph_iri(second["episode"]["id"])) + "#")
        second_stored = db.dataset.graph(graph_iri(second["episode"]["id"]))
        assert (second_ep.Run_nominal, INSIGHT.effectRate, Literal(Decimal("0.0"))) in second_stored
        assert not any(second_stored.subjects(RDF.type, INSIGHT.SimulationSupportedExplanation))


def case_triples_in(live):
    return {s for s, p, o in live if p == str(RDF.type) and o == str(INST.AnomalyCase)}


def check_graphdb_client_requests():
    """The real client: the live graph replaced with the mirror, and only the episode graphs dropped."""
    calls = []

    class Session:
        def post(self, url, **kwargs):
            calls.append(("post", kwargs))
            return types.SimpleNamespace(raise_for_status=lambda: None)

        def put(self, url, **kwargs):
            calls.append(("put", kwargs))
            return types.SimpleNamespace(raise_for_status=lambda: None)

    client = GraphDBClient(GraphDBConfig(enabled=True, endpoint="http://db", repository="insight",
                                         named_graph="urn:insight:semantic:live", timeout_seconds=1.0))
    client.session = Session()
    client.replace_with_triples(MIRROR)
    method, request = calls[-1]
    assert method == "put" and request["params"] == {"context": "<urn:insight:semantic:live>"}, request
    sent = Graph().parse(data=request["data"].decode("utf-8"), format="nt")
    assert {(str(s), str(p), str(o)) for s, p, o in sent} == MIRROR

    client.drop_graphs(EPISODES)
    method, request = calls[-1]
    assert method == "post" and request["headers"]["Content-Type"].startswith("application/sparql-update")
    dataset = Dataset()
    for name in (EPISODES + "rec_a", EPISODES + "rec_b", "urn:insight:semantic:live"):
        dataset.graph(URIRef(name)).add((INST.Agent_Robot, RDF.type, DUL.PhysicalAgent))
    dataset.update(request["data"].decode("utf-8"))
    left = {str(c.identifier) for c in dataset.contexts() if len(c)}
    assert left == {"urn:insight:semantic:live"}, left


def check_graphdb_off_main_thread():
    """GraphDB slow to answer does not hold the agent; its writes land in order, and a mirror sync
    given up is made up for by the next one."""
    located = (str(PHYSICAL_OBJECT_BOTTLE), str(DUL.hasLocation), str(AGENT_ROBOT))
    with_bottle = {(str(AGENT_ROBOT), str(DUL.hasLocation), str(INST.PhysicalPlace_Room)), located,
                   (str(PHYSICAL_OBJECT_BOTTLE), str(RDF.type), str(DUL.PhysicalObject))}
    lost = with_bottle - {located}
    other = (str(INST.Case_old), str(RDF.type), str(INST.AnomalyCase))        # not the mirror's
    release = threading.Event()

    class SlowGraphDB(FakeGraphDB):
        def apply_delta(self, *, added, removed):
            release.wait(10)
            super().apply_delta(added=added, removed=removed)

    db = SlowGraphDB()
    db.live = with_bottle | {other}
    writer = GraphDBWriter(db, lambda message, style: None, mirror=with_bottle, retry_seconds=0.01)
    states = [with_bottle]
    stub = types.SimpleNamespace(
        mapper=types.SimpleNamespace(get_state=lambda: types.SimpleNamespace(
            triples=frozenset(states[-1]), signature=tuple(sorted(states[-1])))),
        _sync_lock=threading.Lock(), last_triples=set(with_bottle), last_signature=tuple(sorted(with_bottle)),
        _last_validated_signature=None, bootstrap_sync=True, causal_validator=LiveCausalValidator(),
        unexplained=False, unexplained_reason="", hypothesis_generation_done=False,
        trigger_added=frozenset(), trigger_removed=frozenset(), graphdb_writer=writer)
    sync = types.MethodType(specificworker.SpecificWorker.sync_semantic_to_graphdb, stub)
    sync()                                                     # start-up: as GraphDB holds it
    states.append(lost)                                        # the bottle is lost
    started = time.monotonic()
    sync()
    assert time.monotonic() - started < 0.5 and stub.unexplained, stub.unexplained_reason
    assert stub.trigger_removed == {located} and db.live == with_bottle | {other}   # GraphDB not answered yet
    release.set()
    writer.join()
    assert db.live == lost | {other}, db.live

    # In order, with retries: the reset fails once, the next sync every time (given up); the one
    # after it is computed against what GraphDB holds, not against the sync given up.
    outcomes = iter([False, True, True, True, False, False, True])

    class FlakyGraphDB(FakeGraphDB):
        def _reachable(self):
            if not next(outcomes, True):
                raise TimeoutError("Read timed out. (read timeout=5.0)")

    db, logs = FlakyGraphDB(), []
    db.dataset.graph(URIRef(EPISODES + "rec_old")).add((INST.Episode_old, RDF.type, INSIGHT.Episode))
    writer = GraphDBWriter(db, lambda message, style: logs.append(message), attempts=2, retry_seconds=0.01)
    extra = {(str(INST.A), str(RDF.type), str(INST.B)), (str(INST.C), str(RDF.type), str(INST.D))}
    writer.reset(MIRROR, EPISODES)
    writer.submit("store the episode", lambda client: client.replace_graph(
        f"<{EPISODES}rec_new#Episode> a <{INSIGHT.Episode}> .", EPISODES + "rec_new"))
    writer.sync_mirror(MIRROR | set(list(extra)[:1]))
    writer.sync_mirror(MIRROR | extra)
    writer.join()
    assert db.episode_graphs() == {EPISODES + "rec_new"}       # stored after the reset dropped the old one
    assert db.live == MIRROR | extra, db.live
    assert sum("retrying" in m for m in logs) == 2 and sum("gave up" in m for m in logs) == 1, logs


def check_non_semantic_signals():
    """With the UEx stack of 08/10 the DSR changes ~350 times/s (IMU at 60 Hz, poses at 38 Hz): the
    agent drops the updates that carry no fact of the mirror before any DSR query or print, so it
    reacts to a lost bottle in time (it lagged 5 s, resultados/27)."""
    calls = []

    class Graph:
        def get_node(self, key):
            calls.append(("get_node", key))
            return types.SimpleNamespace(id=203, type="bottle") if key in ("bottle", 203) else \
                types.SimpleNamespace(id=key, type="affordance")

    class Mapper:
        def updated_node(self, g, node_type):
            calls.append(("updated_node", node_type))
            return False

        def updated_edge(self, g, edge_type):
            calls.append(("updated_edge", edge_type))
            return False

    stub = types.SimpleNamespace(g=Graph(), mapper=Mapper(), _sync_lock=threading.Lock(), _bottle_id=None,
                                 _request_sync=lambda: calls.append(("sync",)))
    for name in ("update_node_att", "update_node", "update_edge", "update_edge_att", "_bottle_node_id"):
        setattr(stub, name, types.MethodType(getattr(specificworker.SpecificWorker, name), stub))
    stub.update_node_att(300, ["imu_accelerometer", "imu_gyroscope"])     # imu, imu_sintetic
    stub.update_node_att(200, ["robot_ref_adv_speed", "robot_ref_rot_speed"])
    stub.update_node(300, "imu")
    stub.update_edge_att(100, 200, "RT", ["rt_translation", "rt_quaternion"])  # root->robot pose
    assert calls == [], calls
    stub.update_edge(100, 200, "RT")                                      # root->robot re-assigned
    stub.update_edge(200, 300, "RT")                                      # robot->person re-assigned
    assert calls == [("get_node", "bottle")], calls                       # the bottle id, looked up once
    stub.update_edge(200, 203, "RT")                                      # robot->bottle: the fact
    stub.update_node_att(401, ["aff_interacting"])                        # follow_me: a fact too
    assert ("updated_edge", "RT") in calls and ("updated_node", "affordance") in calls, calls


def main():
    for check in (check_recording_rule, check_slot_annotations, check_non_semantic_signals, check_graphdb_off_main_thread,
                  check_graphdb_client_requests, check_agent_loop):
        check()
        print(f"OK {check.__name__}")
    print("Semantic agent v2: all checks passed")


if __name__ == "__main__":
    main()
