"""The episode of contract 1 in RDF: the ABox the semantic memory keeps of an episode (task A2).

Each episode goes in a named graph of its own, <http://insight.local/episodes/{episode_id}>, and
its individuals live in <http://insight.local/episodes/{episode_id}#> under their short JSON ids
(Segment_final, Interval_fall, Ev_pitch_rate_peak, ...), so two episodes never share an
individual. The entities and the sensors keep the IRIs production already uses (inst:Agent_Robot,
inst:Sensor_IMU, ...). The vocabulary is the INSIGHT TBox (agents/semantic/data/insight_tbox.ttl).

The graph only adds: the triples production mirrors from the working memory stay as they are.

What the TBox has no term for stays only in the JSON: the setpoint and heading ranges of a phase,
the signed value of a peak and the two ratios behind speed_ratio. The robot pose at the fall is the
end point of Segment_final.
"""

from __future__ import annotations

from decimal import Decimal
from typing import Any

from rdflib import RDF, RDFS, XSD, Dataset, Graph, Literal, Namespace, URIRef
from rdflib.namespace import OWL

DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
SOMA = Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
SOSA = Namespace("http://www.w3.org/ns/sosa/")
INSIGHT = Namespace("http://insight.local/ontology#")
INST = Namespace("http://insight.local/instances#")
EPISODES = "http://insight.local/episodes/"

#: Short entity ids of the JSON -> the IRIs production writes.
ENTITY_IRIS = {
    "Agent_Robot": INST.Agent_Robot,
    "PhysicalObject_Bottle": INST.PhysicalObject_Bottle,
    "Agent_Person": INST.Agent_Person,
    "PhysicalPlace_Room": INST.PhysicalPlace_Room,
    "PhysicalObject_Tray": INST.PhysicalObject_Tray,
}

#: Which source of evidence makes each observation (the TBox says what each sensor observes).
SENSORS_BY_PROPERTY = {
    "pitch_rate_peak": [INST.Sensor_IMU],
    "vertical_accel_peak": [INST.Sensor_IMU],
    "longitudinal_accel_peak": [INST.Sensor_IMU],
    "lateral_accel_peak": [INST.Sensor_IMU],
    "yaw_rate_peak": [INST.Sensor_IMU],
    "max_sustained_horizontal_accel": [INST.Sensor_IMU],
    "yaw_change": [INST.Sensor_Localization, INST.Sensor_BaseCommand],
    "speed_ratio": [INST.Sensor_Localization, INST.Sensor_BaseCommand],
    "setpoint_step": [INST.Sensor_BaseCommand],
    "reaction_onset": [INST.Sensor_BaseCommand],
    "person_distance_min": [INST.Sensor_PersonTracker],
    "bottle_reacquired": [INST.Sensor_BottleTracker],
}

#: Kinds of motion that are locomotion; a stopped phase is not classified by any.
LOCOMOTION = {"advance_straight": INST.Locomotion_advance_straight, "turn": INST.Locomotion_turn}
SUPPORT_STATE = INST.SupportState_bottle_on_tray


def graph_iri(episode_id: str) -> URIRef:
    return URIRef(EPISODES + episode_id)


def episode_namespace(episode_id: str) -> Namespace:
    return Namespace(EPISODES + episode_id + "#")


def _decimal(value: float) -> Literal:
    return Literal(Decimal(repr(float(value))), datatype=XSD.decimal)


def episode_graph(episode: dict[str, Any]) -> Graph:
    """The triples of one episode (to be stored in its named graph)."""
    ep = episode_namespace(episode["episode_id"])
    g = Graph()
    for prefix, namespace in (("dul", DUL), ("soma", SOMA), ("sosa", SOSA), ("insight", INSIGHT),
                              ("inst", INST), ("ep", ep), ("owl", OWL)):
        g.bind(prefix, namespace)

    episode_iri = ep.Episode
    individuals: list[URIRef] = []

    def individual(local_id: str, *types: URIRef) -> URIRef:
        iri = ep[local_id]
        g.add((iri, RDF.type, OWL.NamedIndividual))
        for rdf_type in types:
            g.add((iri, RDF.type, rdf_type))
        g.add((iri, RDFS.label, Literal(local_id)))
        individuals.append(iri)
        return iri

    def interval(local_id: str, start: float, end: float) -> URIRef:
        iri = individual(local_id, DUL.TimeInterval)
        g.add((iri, INSIGHT.startS, _decimal(start)))
        g.add((iri, INSIGHT.endS, _decimal(end)))
        return iri

    robot = ENTITY_IRIS[episode["entities"]["robot"]]
    bottle = ENTITY_IRIS[episode["entities"]["bottle"]]
    # The tray is a part of the robot (not a DSR node, so production's mirror does not write it).
    if "tray" in episode["entities"]:
        tray = ENTITY_IRIS[episode["entities"]["tray"]]
        g.add((tray, RDF.type, OWL.NamedIndividual))
        g.add((tray, RDF.type, SOMA.DesignedComponent))
        g.add((tray, RDFS.label, Literal("tray")))
        g.add((robot, DUL.hasComponent, tray))

    # The episode and where it was recorded.
    g.add((episode_iri, RDF.type, OWL.NamedIndividual))
    g.add((episode_iri, RDF.type, INSIGHT.Episode))
    g.add((episode_iri, RDFS.label, Literal(episode["episode_id"])))
    g.add((episode_iri, INSIGHT.recordedIn, Literal(episode["source"]["path"], datatype=XSD.anyURI)))
    g.add((episode_iri, INSIGHT.timeOriginNs, Literal(int(episode["time"]["origin_ns"]), datatype=XSD.integer)))

    intervals = {}
    for entry in episode["intervals"]:
        intervals[entry["id"]] = interval(entry["id"], entry["start_s"], entry["end_s"])

    # Phases: what the robot was doing; the reaction is the system's, never a cause.
    for phase in episode["phases"]:
        if phase.get("is_system_reaction"):
            iri = individual(phase["id"], INSIGHT.SystemReaction)
            g.add((iri, INSIGHT.isSystemReaction, Literal(True)))
            if phase.get("note"):
                g.add((iri, RDFS.comment, Literal(phase["note"])))
        else:
            iri = individual(phase["id"], DUL.Action)
            g.add((iri, INSIGHT.motionKind, Literal(phase["kind"])))
            g.add((iri, INSIGHT.meanSpeedMps, _decimal(phase["mean_speed_mps"])))
            if phase["kind"] in LOCOMOTION:
                g.add((iri, DUL.isClassifiedBy, LOCOMOTION[phase["kind"]]))
        g.add((iri, DUL.hasTimeInterval, intervals[phase["interval"]]))
        g.add((iri, DUL.hasParticipant, robot))
    for kind, concept in LOCOMOTION.items():
        g.add((concept, RDF.type, OWL.NamedIndividual))
        g.add((concept, RDF.type, SOMA.Locomotion))
        g.add((concept, RDFS.label, Literal(kind)))

    # Path segments and their regions.
    for segment in episode["segments"]:
        iri = individual(segment["id"], INSIGHT.PathSegment)
        region = individual(f"Region_{segment['id']}", DUL.SpaceRegion)
        g.add((iri, DUL.hasRegion, region))
        g.add((iri, INSIGHT.traversedDuring, intervals[segment["interval"]]))
        g.add((iri, INSIGHT.orderBackFromFall, Literal(segment["order_back_from_fall"], datatype=XSD.positiveInteger)))
        g.add((iri, INSIGHT.lengthM, _decimal(segment["length_m"])))
        g.add((iri, INSIGHT.meanSpeedMps, _decimal(segment["mean_speed_mps"])))
        for prop, value in ((INSIGHT.fromX, segment["from_xy"][0]), (INSIGHT.fromY, segment["from_xy"][1]),
                            (INSIGHT.toX, segment["to_xy"][0]), (INSIGHT.toY, segment["to_xy"][1]),
                            (INSIGHT.xMin, segment["bbox"]["x"][0]), (INSIGHT.xMax, segment["bbox"]["x"][1]),
                            (INSIGHT.yMin, segment["bbox"]["y"][0]), (INSIGHT.yMax, segment["bbox"]["y"][1]),
                            (INSIGHT.headingDeg, segment["heading_deg"])):
            g.add((region, prop, _decimal(value)))

    # The bottle held on the tray: a state classified as support, with its two roles.
    support = episode["support"]
    support_iri = individual(support["id"], SOMA.State)
    g.add((SUPPORT_STATE, RDF.type, OWL.NamedIndividual))
    g.add((SUPPORT_STATE, RDF.type, SOMA.SupportState))
    g.add((SUPPORT_STATE, RDFS.label, Literal("bottle on the tray")))
    g.add((support_iri, DUL.isClassifiedBy, SUPPORT_STATE))
    supporter = individual("Role_supporter", SOMA.Supporter)
    supported = individual("Role_supported", SOMA.SupportedObject)
    g.add((supporter, DUL.classifies, ENTITY_IRIS[support["supporter"]]))
    g.add((supported, DUL.classifies, ENTITY_IRIS[support["supported"]]))
    g.add((support_iri, DUL.hasParticipant, ENTITY_IRIS[support["supporter"]]))
    g.add((support_iri, DUL.hasParticipant, ENTITY_IRIS[support["supported"]]))
    if support.get("start_s") is not None:
        g.add((support_iri, DUL.hasTimeInterval, interval("Interval_support", support["start_s"], support["end_s"])))

    # The fall.
    accident = episode["accident"]
    accident_iri = individual(accident["id"], SOMA.Accident)
    g.add((accident_iri, INSIGHT.timeS, _decimal(accident["time_s"])))
    g.add((accident_iri, DUL.hasLocation, ep[accident["place"]]))
    g.add((accident_iri, INSIGHT.headingDeg, _decimal(accident["robot_pose"]["heading_deg"])))
    for participant in accident["participants"]:
        g.add((accident_iri, DUL.hasParticipant, ENTITY_IRIS[participant]))
    g.add((accident_iri, INSIGHT.ends, support_iri))
    g.add((accident_iri, RDFS.comment, Literal(f"Cause {accident['cause']}; observed as {accident['observed_as']}.")))

    # Evidence: one observation per entry of the fixed list.
    for entry in episode["evidence"]:
        iri = individual(entry["id"], SOSA.Observation)
        g.add((iri, SOSA.observedProperty, INSIGHT[entry["property"]]))
        for sensor in SENSORS_BY_PROPERTY[entry["property"]]:
            g.add((iri, SOSA.madeBySensor, sensor))
        value = entry.get("value")
        if isinstance(value, bool):
            g.add((iri, SOSA.hasSimpleResult, Literal(value)))
        elif value is not None:
            g.add((iri, SOSA.hasSimpleResult, _decimal(value)))
        if entry.get("interval"):
            g.add((iri, SOSA.phenomenonTime, intervals[entry["interval"]]))
        for key, prop in (("at_s", INSIGHT.atTimeS), ("baseline", INSIGHT.baselineValue),
                          ("commanded", INSIGHT.commandedValue)):
            if entry.get(key) is not None:
                g.add((iri, prop, _decimal(entry[key])))
        if entry.get("unit"):
            g.add((iri, INSIGHT.unit, Literal(entry["unit"])))
        if entry.get("is_system_reaction"):
            g.add((iri, INSIGHT.isSystemReaction, Literal(True)))

    for iri in individuals + list(ENTITY_IRIS.values()):
        g.add((episode_iri, DUL.isSettingFor, iri))
    return g


def episode_dataset(episode: dict[str, Any]) -> Dataset:
    """The episode in its named graph, ready to be written as TriG or sent to GraphDB."""
    graph = episode_graph(episode)
    dataset = Dataset()
    named = dataset.graph(graph_iri(episode["episode_id"]))
    for prefix, namespace in graph.namespaces():
        dataset.bind(prefix, namespace)
    for triple in graph:
        named.add(triple)
    return dataset
