"""Offline test: the INSIGHT TBox (agents/semantic/data/insight_tbox.ttl).

Checks that the file matches its sources, that the imported SOMA/DUL/SOSA hierarchy is the one the
contracts rely on, that the mechanism layer is complete and agrees with the intervention catalog,
that every term production writes is declared, and that the TBox answers queries over the triples
production mirrored in a real run. If owlready2 is installed, HermiT also checks consistency.

Run from the repo root:  python3 tests/offline/test_insight_tbox.py
"""
import json
import re
import sys
import tempfile
from pathlib import Path

from rdflib import Graph, Namespace, URIRef
from rdflib.compare import to_isomorphic

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "agents" / "semantic" / "data"
TBOX = DATA / "insight_tbox.ttl"
sys.path.insert(0, str(DATA))

from build_insight_tbox import build  # noqa: E402

DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
INST = Namespace("http://insight.local/instances#")

PREFIXES = """
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
PREFIX owl: <http://www.w3.org/2002/07/owl#>
PREFIX dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
PREFIX sosa: <http://www.w3.org/ns/sosa/>
PREFIX insight: <http://insight.local/ontology#>
PREFIX inst: <http://insight.local/instances#>
"""

# Contract 2, section 3.3.
MECHANISM_IDS = {
    "obstacle_traversed", "bottle_push", "robot_push", "slippery_floor", "wheel_failure",
    "commanded_speed_change", "uncommanded_motion", "bottle_removed_by_person",
    "perception_failure", "suspension_failure",
}

# Batch written by production on 2026-09-24 (12:29 episode); its context holds the mirrored triples.
REAL_BATCH = REPO / "agents" / "semantic" / "generated_hypotheses" / "semantic_unexplained_20260924T102944Z.json"


def ask(graph, query):
    return bool(graph.query(PREFIXES + query).askAnswer)


def select(graph, query):
    return [tuple(row) for row in graph.query(PREFIXES + query)]


def check_up_to_date(tbox):
    assert to_isomorphic(build()) == to_isomorphic(tbox), (
        "insight_tbox.ttl is out of date: run python3 agents/semantic/data/build_insight_tbox.py")


def check_hierarchy(tbox):
    # Physical states remain available, but carrying-relation observations use a support estimate.
    assert ask(tbox, "ASK { soma:SupportState rdfs:subClassOf* dul:Concept }")
    assert not ask(tbox, "ASK { soma:SupportState rdfs:subClassOf* dul:Event }")
    for event_class in ("soma:State", "soma:Accident", "sosa:Observation", "insight:SystemReaction"):
        assert ask(tbox, f"ASK {{ {event_class} rdfs:subClassOf* dul:Event }}"), event_class
    assert ask(tbox, "ASK { insight:ObservedAnomaly rdfs:subClassOf* dul:Event }")
    for description in ("insight:SupportEstimate", "insight:SimulationSupportedExplanation"):
        assert ask(tbox, f"ASK {{ {description} rdfs:subClassOf* dul:Description }}")
        assert not ask(tbox, f"ASK {{ {description} rdfs:subClassOf* dul:Event }}")
    assert ask(tbox, "ASK { dul:PhysicalAgent rdfs:subClassOf dul:Agent, dul:PhysicalObject }")
    assert ask(tbox, "ASK { insight:Mechanism rdfs:subClassOf* dul:Description }")
    assert ask(tbox, "ASK { insight:PathSegment rdfs:subClassOf* dul:PhysicalPlace }")
    assert ask(tbox, "ASK { insight:Episode rdfs:subClassOf* dul:Situation }")
    orphans = select(tbox, """
        SELECT ?c WHERE {
          ?c a owl:Class . FILTER(isIRI(?c) && ?c NOT IN (owl:Thing, dul:Entity))
          FILTER NOT EXISTS { ?c rdfs:subClassOf* dul:Entity }
        }""")
    assert not orphans, orphans


def check_mechanisms(tbox):
    ids = {str(row[0]) for row in select(tbox, "SELECT ?id WHERE { ?m a insight:Mechanism ; insight:mechanismId ?id }")}
    assert ids == MECHANISM_IDS, ids ^ MECHANISM_IDS

    for (mechanism,) in select(tbox, "SELECT ?m WHERE { ?m a insight:Mechanism }"):
        m = f"<{mechanism}>"
        for field in ("family", "inOldCatalog", "costInSimulations", "promptDescription", "titleTemplate"):
            assert ask(tbox, f"ASK {{ {m} insight:{field} ?v }}"), (mechanism, field)
        realized = ask(tbox, f"ASK {{ {m} insight:realizedBy ?i }}")
        explained = ask(tbox, f"ASK {{ {m} insight:notSimulableBecause ?r }}")
        assert realized != explained, f"{mechanism}: needs exactly one of realizedBy / notSimulableBecause"

        # A contrast rule may only read properties on which the mechanism leaves a trace.
        unexpected = select(tbox, f"""
            SELECT ?p WHERE {{ {m} insight:hasContrastRule ?r . ?r insight:checksProperty ?p .
                               FILTER NOT EXISTS {{ {m} insight:entailsObservation ?p }} }}""")
        assert not unexpected, (mechanism, unexpected)

        # The title template can only use what the mechanism admits.
        template = str(select(tbox, f"SELECT ?t WHERE {{ {m} insight:titleTemplate ?t }}")[0][0])
        names = {str(r[0]) for r in select(tbox, f"""
            SELECT ?n WHERE {{ {m} insight:admitsParameter ?p . ?p insight:parameterName ?n }}""")}
        if ask(tbox, f"ASK {{ {m} insight:requiresAnchor insight:SegmentAnchor }}"):
            names.add("segment")
        if ask(tbox, f"ASK {{ {m} insight:requiresAnchor insight:IntervalAnchor }}"):
            names.add("interval")
        placeholders = set(re.findall(r"\{(\w+)\}", template))
        assert placeholders <= names, (mechanism, placeholders - names)


def check_checks_and_evidence(tbox):
    incomplete = select(tbox, """
        SELECT ?r WHERE { ?r a/rdfs:subClassOf* insight:EpisodeCheck .
          FILTER NOT EXISTS { ?r insight:ruleId ?i ; insight:ruleEffect ?e ; insight:ruleStatus ?s } }""")
    assert not incomplete, incomplete
    unobserved = select(tbox, """
        SELECT ?p WHERE { ?p a sosa:ObservableProperty .
          FILTER NOT EXISTS { ?sensor a sosa:Sensor ; sosa:observes ?p } }""")
    assert not unobserved, unobserved
    assert len(select(tbox, "SELECT ?p WHERE { ?p a sosa:ObservableProperty }")) == 12


def check_catalog(tbox):
    """The catalog is checked from the ontology: every production intervention realizes some
    mechanism, and every non-experimental realization exists in the catalog."""
    catalog = json.loads((REPO / "etc" / "intervention_catalog.json").read_text(encoding="utf-8"))
    in_catalog = set(catalog["interventions"])
    realized = {str(r[0]) for r in select(tbox, """
        SELECT ?id WHERE { ?m insight:realizedBy ?i . ?i insight:catalogId ?id ; insight:isExperimental false }""")}
    assert realized == in_catalog, realized ^ in_catalog


def check_production_terms(tbox):
    """Every class and property production writes is declared (individuals are not)."""
    sources = [REPO / "agents" / "semantic" / "src" / name for name in ("ontology_mapping.py", "verdict_ingestor.py")]
    used = set()
    for source in sources:
        for namespace, name in re.findall(r"\b(DUL|INSIGHT)\.([A-Za-z]+)\b", source.read_text(encoding="utf-8")):
            if not (namespace == "DUL" and name == "owl"):
                used.add((DUL if namespace == "DUL" else INST)[name])
    missing = [term for term in used if not ask(tbox, f"ASK {{ <{term}> a ?kind }}")]
    assert not missing, missing


def check_queries_over_real_mirror(tbox):
    """With the TBox, the mirror of a real run answers questions it could not before."""
    batch = json.loads(REAL_BATCH.read_text(encoding="utf-8"))
    graph = Graph()
    for triple in batch["context_summary"]["current_triples"]:
        graph.add(tuple(URIRef(triple[key]) for key in ("subject", "predicate", "object")))
    graph += tbox
    agents = {str(r[0]) for r in select(graph, "SELECT ?x WHERE { ?x a/rdfs:subClassOf* dul:Agent }")}
    assert agents == {str(INST.Agent_Robot), str(INST.Agent_Person)}, agents
    assert ask(graph, "ASK { inst:PhysicalObject_Bottle a/rdfs:subClassOf* dul:Object }")
    assert ask(graph, "ASK { inst:Action_Follow a/rdfs:subClassOf* dul:Event }")


def check_consistency_if_possible(tbox):
    try:
        import owlready2
    except ImportError:
        print("  (owlready2 not installed: HermiT consistency check skipped)")
        return
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "insight_tbox.owl"
        tbox.serialize(path, format="xml")
        world = owlready2.World()
        ontology = world.get_ontology(path.as_uri()).load()
        with ontology:
            owlready2.sync_reasoner_hermit(world, infer_property_values=False, debug=0)
        unsatisfiable = list(world.inconsistent_classes())
    assert not unsatisfiable, unsatisfiable
    print("  HermiT: consistent, no unsatisfiable classes")


def main():
    assert TBOX.exists(), f"missing {TBOX}: run python3 agents/semantic/data/build_insight_tbox.py"
    tbox = Graph()
    tbox.parse(TBOX, format="turtle")
    for check in (check_up_to_date, check_hierarchy, check_mechanisms, check_checks_and_evidence,
                  check_catalog, check_production_terms, check_queries_over_real_mirror,
                  check_consistency_if_possible):
        check(tbox)
        print(f"OK {check.__name__}")
    print("INSIGHT TBox: all checks passed")


if __name__ == "__main__":
    main()
