"""Builds the INSIGHT TBox: agents/semantic/data/insight_tbox.ttl.

It merges, into one self-contained file:
  * T_op.owl, the IPM's pruned module (DUL core and the SOMA causal relations);
  * the DUL and SOMA classes the episode and the mechanisms need, imported with MIREOT: each term
    with its label, comment and named superclasses up to dul:Entity, and no other axiom;
  * the SOSA terms the evidence uses, with their alignment to DUL from the W3C SSN-DUL alignment;
  * insight_layer.ttl, the hand-written INSIGHT layer.

The sources are in sources/ (SOMA and DUL copied from the IPM repository, SOSA and SSN-DUL
downloaded from w3.org). Run from the repo root:

    python3 agents/semantic/data/build_insight_tbox.py
"""

from __future__ import annotations

import hashlib
from pathlib import Path

from rdflib import Graph, Literal, Namespace, URIRef
from rdflib.namespace import OWL, RDF, RDFS

HERE = Path(__file__).resolve().parent
SOURCES = HERE / "sources"
OUTPUT = HERE / "insight_tbox.ttl"

DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
SOMA = Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
SOSA = Namespace("http://www.w3.org/ns/sosa/")
INSIGHT = Namespace("http://insight.local/ontology#")
INST = Namespace("http://insight.local/instances#")
OCRA = Namespace("http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#")

ONTOLOGY_IRI = URIRef("http://insight.local/ontology")
VERSION = "0.1"

# Design section 3.1, plus soma:State, which contract 1 needs for the support of the bottle
# (soma:SupportState is a concept that classifies states, not a state), and soma:DesignedComponent,
# the tray: a part of the robot that holds the bottle (contract 1, version 1.7).
SOMA_CLASSES = [
    "Simulation_Reasoner", "SupportState", "Supporter", "SupportedObject", "Collision",
    "ForceInteraction", "Feature", "Accident", "Locomotion", "FrictionAttribute",
    "HardwareDiagnosis", "SoftwareDiagnosis", "State", "DesignedComponent",
]
# PhysicalAgent is what production types the robot and the person with; the rest are used by
# contract 1 (SpaceRegion), the SSN-DUL alignment (Quality) and the INSIGHT layer (Parameter).
DUL_CLASSES = ["PhysicalAgent", "SpaceRegion", "Quality", "Parameter"]

SOSA_CLASSES = ["Sensor", "ObservableProperty", "Observation"]
SOSA_OBJECT_PROPERTIES = ["observes", "madeBySensor", "observedProperty", "phenomenonTime"]
SOSA_DATATYPE_PROPERTIES = ["hasSimpleResult"]
# sosa:ObservableProperty is a ssn:Property, which the alignment puts under dul:Quality.
SOSA_EXTRA_ALIGNMENT = [(SOSA.ObservableProperty, RDFS.subClassOf, DUL.Quality)]

ANNOTATIONS = (RDFS.label, RDFS.comment)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path, fmt: str) -> Graph:
    graph = Graph()
    graph.parse(path, format=fmt)
    return graph


def named_superclasses(graph: Graph, term: URIRef) -> list[URIRef]:
    return [o for o in graph.objects(term, RDFS.subClassOf) if isinstance(o, URIRef)]


def import_class(target: Graph, term: URIRef, sources: dict[str, tuple[Graph, URIRef]]) -> None:
    """MIREOT: copy the class, its annotations and its named superclasses, recursively.

    Each term comes from the ontology of its namespace: SOMA re-declares some DUL classes without
    their superclasses, so taking the first graph that declares a term would cut the hierarchy."""
    namespace = str(term).rsplit("#", 1)[0] + "#"
    if namespace not in sources:
        raise KeyError(f"no source for the namespace of {term}")
    source, source_iri = sources[namespace]
    if (term, RDF.type, OWL.Class) not in source:
        raise KeyError(f"{term} is not declared in {source_iri}")

    target.add((term, RDF.type, OWL.Class))
    target.add((term, RDFS.isDefinedBy, source_iri))
    for annotation in ANNOTATIONS:
        if next(target.objects(term, annotation), None) is None:
            for value in source.objects(term, annotation):
                target.add((term, annotation, value))
    for parent in named_superclasses(source, term):
        target.add((term, RDFS.subClassOf, parent))
        if parent != OWL.Thing:
            import_class(target, parent, sources)


def import_sosa(target: Graph, sosa: Graph, alignment: Graph) -> None:
    sosa_iri = URIRef(str(SOSA))
    terms = (
        [(SOSA[n], OWL.Class) for n in SOSA_CLASSES]
        + [(SOSA[n], OWL.ObjectProperty) for n in SOSA_OBJECT_PROPERTIES]
        + [(SOSA[n], OWL.DatatypeProperty) for n in SOSA_DATATYPE_PROPERTIES]
    )
    for term, kind in terms:
        if (term, None, None) not in sosa:
            raise KeyError(f"{term} is not in sosa.ttl")
        target.add((term, RDF.type, kind))
        target.add((term, RDFS.isDefinedBy, sosa_iri))
        for annotation in ANNOTATIONS:
            for value in sosa.objects(term, annotation):
                if not isinstance(value, Literal) or value.language in (None, "en"):
                    target.add((term, annotation, value))
        for predicate in (RDFS.subClassOf, RDFS.subPropertyOf):
            for parent in alignment.objects(term, predicate):
                if isinstance(parent, URIRef):
                    target.add((term, predicate, parent))
    for triple in SOSA_EXTRA_ALIGNMENT:
        target.add(triple)


def add_kinds(graph: Graph) -> None:
    """One insight:Kind per allowed value of each qualitative parameter (contracts 1.8): the
    concepts a hypothesis is classified by, so that the kinds the LLM chose are individuals of the
    TBox and not strings. Named after the parameter and the value (Kind_shape_bump)."""
    for parameter in set(graph.subjects(RDF.type, INSIGHT.QualitativeParameter)):
        name = graph.value(parameter, INSIGHT.parameterName)
        for value in graph.objects(parameter, INSIGHT.allowedValue):
            kind = kind_iri(str(name), str(value))
            graph.add((kind, RDF.type, INSIGHT.Kind))
            graph.add((kind, RDFS.label, Literal(f"{name}: {value}", lang="en")))
            graph.add((kind, INSIGHT.ofParameter, parameter))
            graph.add((kind, INSIGHT.kindValue, Literal(str(value))))


def kind_iri(parameter_name: str, value: str) -> URIRef:
    return INSIGHT[f"Kind_{parameter_name}_{value}"]


def drop_ontology_headers(graph: Graph) -> None:
    for ontology in list(graph.subjects(RDF.type, OWL.Ontology)):
        graph.remove((ontology, None, None))


def declared(graph: Graph, term: URIRef) -> bool:
    return next(graph.objects(term, RDF.type), None) is not None


def check(graph: Graph) -> list[str]:
    """Every class reaches dul:Entity, and every term the layer uses is declared."""
    problems = []
    for cls in set(graph.subjects(RDF.type, OWL.Class)):
        if not isinstance(cls, URIRef) or cls in (OWL.Thing, DUL.Entity):
            continue
        seen, frontier = set(), [cls]
        while frontier:
            current = frontier.pop()
            if current in seen:
                continue
            seen.add(current)
            frontier.extend(named_superclasses(graph, current))
        if DUL.Entity not in seen:
            problems.append(f"class without a path to dul:Entity: {cls}")

    own = (str(INSIGHT), str(INST), str(SOMA), str(SOSA))
    for s, p, o in graph:
        used = [p] + ([o] if p == RDF.type else [])
        for term in used:
            if isinstance(term, URIRef) and str(term).startswith(own) and not declared(graph, term):
                problems.append(f"undeclared term: {term}")
    return sorted(set(problems))


def build() -> Graph:
    t_op_path = HERE / "T_op.owl"
    dul_path, soma_path = SOURCES / "DUL.owl", SOURCES / "SOMA.owl.rdf"
    sosa_path, alignment_path = SOURCES / "sosa.ttl", SOURCES / "ssn-dul.ttl"
    layer_path = HERE / "insight_layer.ttl"

    tbox = load(t_op_path, "xml")
    drop_ontology_headers(tbox)
    dul, soma = load(dul_path, "turtle"), load(soma_path, "xml")
    sosa, alignment = load(sosa_path, "turtle"), load(alignment_path, "turtle")

    dul_iri, soma_iri = URIRef("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl"), URIRef(str(SOMA)[:-1])
    sources = {str(DUL): (dul, dul_iri), str(SOMA): (soma, soma_iri)}
    for name in DUL_CLASSES:
        import_class(tbox, DUL[name], sources)
    for name in SOMA_CLASSES:
        import_class(tbox, SOMA[name], sources)
    import_sosa(tbox, sosa, alignment)

    tbox.parse(layer_path, format="turtle")
    add_kinds(tbox)

    tbox.add((ONTOLOGY_IRI, RDF.type, OWL.Ontology))
    tbox.add((ONTOLOGY_IRI, OWL.versionInfo, Literal(VERSION)))
    tbox.add((ONTOLOGY_IRI, RDFS.label, Literal("INSIGHT TBox", lang="en")))
    tbox.add((ONTOLOGY_IRI, RDFS.comment, Literal(
        "Generated by build_insight_tbox.py; do not edit by hand. Sources (sha256): "
        + "; ".join(f"{p.name} {sha256(p)[:12]}" for p in
                    (t_op_path, dul_path, soma_path, sosa_path, alignment_path, layer_path)),
        lang="en")))

    for prefix, namespace in (("dul", DUL), ("soma", SOMA), ("sosa", SOSA), ("insight", INSIGHT),
                              ("inst", INST), ("ocra", OCRA), ("owl", OWL)):
        tbox.bind(prefix, namespace, replace=True)
    return tbox


def main() -> None:
    tbox = build()
    problems = check(tbox)
    for problem in problems:
        print("PROBLEM:", problem)
    if problems:
        raise SystemExit(1)
    tbox.serialize(OUTPUT, format="turtle")
    count = lambda kind: len(set(s for s in tbox.subjects(RDF.type, kind) if isinstance(s, URIRef)))
    print(f"Wrote {OUTPUT.relative_to(HERE.parents[2])}: {len(tbox)} triples, "
          f"{count(OWL.Class)} classes, {count(OWL.ObjectProperty)} object properties, "
          f"{count(OWL.DatatypeProperty)} datatype properties, "
          f"{len(set(tbox.subjects(RDF.type, INSIGHT.Mechanism)))} mechanisms.")


if __name__ == "__main__":
    main()
