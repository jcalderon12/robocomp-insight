"""The episode of contract 1 in RDF: the ABox the semantic memory keeps of an episode (task A2).

Each episode goes in a named graph of its own, <http://insight.local/episodes/{episode_id}>, and
its individuals live in <http://insight.local/episodes/{episode_id}#> under their short JSON ids
(Segment_final, Interval_fall, Ev_pitch_rate_peak, ...), so two episodes never share an
individual. The entities and the sensors keep the IRIs production already uses (inst:Agent_Robot,
inst:Sensor_IMU, ...). The vocabulary is the INSIGHT TBox (agents/semantic/data/insight_tbox.ttl).

The graph only adds: the triples production mirrors from the working memory stay as they are.

Since contracts 1.12 the graph keeps every field of contract 1 that the prompt, the vocabulary, the
contrast and the grounding read, so that they read the case from the semantic memory
(semantic_memory.py) and not from the JSON: the frame and the clock, the roles of the entities and
their labels, the order of every list, the setpoint and heading ranges of each phase, the signed
value of a peak, the two ratios behind speed_ratio, the recorded change that the live validator
found unexplained (with its reason), and the sha256 of the contract-1 record. The legacy
`participants` and `cause` of the accident record are left out: they describe the change as an
accident, which the memory does not assert. The observation's coordinates belong to the robot, not
the object. Legacy accident/support JSON fields describe a representation change and an estimate:
they never assert a physical fall or the end of physical support.
"""

from __future__ import annotations

import hashlib
import json
from decimal import Decimal
from typing import Any, Optional

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


def graph_iri(episode_id: str) -> URIRef:
    return URIRef(EPISODES + episode_id)


def episode_namespace(episode_id: str) -> Namespace:
    return Namespace(EPISODES + episode_id + "#")


def entity_iri(episode: dict[str, Any], identifier: str) -> URIRef:
    """Resolve a recorded entity identifier without assuming the current payload's name."""
    if ":" in identifier:
        return URIRef(identifier)
    return URIRef(episode.get("entity_namespace", str(INST)) + identifier)


#: The sections of contract 1 that can hold the observed change, in the order they are looked for.
OBSERVATION_SECTIONS = ("change", "observation", "accident")
#: Local name of an observed change whose record gives it no id.
UNIDENTIFIED_OBSERVATION = "ObservedChange"
#: Where a recorded change comes from: the record of the change, or the live validator's delta.
CHANGE_SOURCES = ("change", "changes", "removed_triples", "added_triples")


def observation_section(episode: dict[str, Any]) -> str:
    """The section that holds the observed change: change, observation or the legacy accident."""
    section = next((name for name in OBSERVATION_SECTIONS if episode.get(name)), None)
    if section is None:
        raise ValueError("the RDF episode needs an observed change")
    return section


def observation_record(episode: dict[str, Any]) -> dict[str, Any]:
    """Explicit observed change, or the legacy field whose name does not establish an accident."""
    return episode[observation_section(episode)]


def observation_id(episode: dict[str, Any]) -> str:
    return observation_record(episode).get("id") or UNIDENTIFIED_OBSERVATION


def observation_iri(episode: dict[str, Any]) -> URIRef:
    return episode_namespace(episode["episode_id"])[observation_id(episode)]


def record_sha256(episode: dict[str, Any]) -> str:
    """sha256 of the contract-1 record, canonical (sorted keys, no spaces): what a batch cites."""
    canonical = json.dumps(episode, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _decimal(value: float) -> Literal:
    return Literal(Decimal(repr(float(value))), datatype=XSD.decimal)


def _number(value: Any) -> Literal:
    """A recorded value with its type: a boolean, an integer or a decimal."""
    if isinstance(value, bool):
        return Literal(value)
    if isinstance(value, int):
        return Literal(value, datatype=XSD.integer)
    return _decimal(value)


def _index(value: int) -> Literal:
    return Literal(value, datatype=XSD.nonNegativeInteger)


def episode_graph(episode: dict[str, Any], trigger: Optional[dict[str, Any]] = None) -> Graph:
    """The triples of one episode (to be stored in its named graph).

    `trigger` is the delta the live validator found unexplained (the batch's trigger: its reason and
    the removed and added triples of the working memory); it goes with the observed change."""
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

    entities = episode.get("entities") or {}
    robot = entity_iri(episode, entities["robot"]) if entities.get("robot") else None
    # The tray is a part of the robot (not a DSR node, so production's mirror does not write it).
    if "tray" in entities and robot is not None:
        tray = entity_iri(episode, entities["tray"])
        g.add((tray, RDF.type, OWL.NamedIndividual))
        g.add((tray, RDF.type, SOMA.DesignedComponent))
        g.add((tray, RDFS.label, Literal("tray")))
        g.add((robot, DUL.hasComponent, tray))

    # The episode and where it was recorded.
    g.add((episode_iri, RDF.type, OWL.NamedIndividual))
    g.add((episode_iri, RDF.type, INSIGHT.Episode))
    g.add((episode_iri, RDFS.label, Literal(episode["episode_id"])))
    if (episode.get("source") or {}).get("path"):
        g.add((episode_iri, INSIGHT.recordedIn, Literal(episode["source"]["path"], datatype=XSD.anyURI)))
    if (episode.get("time") or {}).get("origin_ns") is not None:
        g.add((episode_iri, INSIGHT.timeOriginNs, Literal(int(episode["time"]["origin_ns"]), datatype=XSD.integer)))
    g.add((episode_iri, INSIGHT.recordSha256, Literal(record_sha256(episode))))
    texts = [(episode, "schema", INSIGHT.episodeSchema), (episode, "entity_namespace", INSIGHT.entityNamespace),
             (episode, "mission", INSIGHT.missionDescription)]
    source, frame, clock = episode.get("source") or {}, episode.get("frame") or {}, episode.get("time") or {}
    texts += [(source, "kind", INSIGHT.sourceKind), (source, "sha256", INSIGHT.recordingSha256),
              (source, "builder", INSIGHT.builtBy), (source, "reader", INSIGHT.readBy),
              (source, "pose_axes", INSIGHT.poseAxes), (frame, "name", INSIGHT.frameName),
              (frame, "units", INSIGHT.positionUnits), (frame, "heading", INSIGHT.headingConvention),
              (clock, "origin", INSIGHT.timeOrigin)]
    for record, key, prop in texts:
        if isinstance(record.get(key), (str, dict)):
            value = record[key]
            g.add((episode_iri, prop, Literal(value if isinstance(value, str) else json.dumps(value, ensure_ascii=False))))
    for key, prop in (("origin_offset_from_file_start_s", INSIGHT.originOffsetS), ("t_obs_s", INSIGHT.observationTimeS),
                      ("episode_length_s", INSIGHT.episodeLengthS), ("simulation_horizon_s", INSIGHT.simulationHorizonS)):
        if clock.get(key) is not None:
            g.add((episode_iri, prop, _number(clock[key])))

    # Which entity plays each role, in the episode's order, and how the episode labels them.
    for index, (role, entity) in enumerate(entities.items()):
        if not isinstance(entity, str):
            continue
        binding = individual(f"Role_{role}", INSIGHT.RoleBinding)
        g.add((binding, INSIGHT.listIndex, _index(index)))
        g.add((binding, INSIGHT.roleName, Literal(role)))
        g.add((binding, INSIGHT.entityId, Literal(entity)))
        g.add((binding, INSIGHT.boundEntity, entity_iri(episode, entity)))
    for key, label in (episode.get("entity_labels") or {}).items():
        g.add((entity_iri(episode, key), INSIGHT.displayLabel, Literal(str(label))))

    intervals = {}
    for index, entry in enumerate(episode.get("intervals", [])):
        intervals[entry["id"]] = interval(entry["id"], entry["start_s"], entry["end_s"])
        g.add((intervals[entry["id"]], INSIGHT.listIndex, _index(index)))
        if entry.get("is_system_reaction"):
            g.add((intervals[entry["id"]], INSIGHT.isSystemReaction, Literal(True)))

    # Phases: what the robot was doing; the reaction is the system's, never a cause.
    for index, phase in enumerate(episode.get("phases", [])):
        if phase.get("is_system_reaction"):
            iri = individual(phase["id"], INSIGHT.SystemReaction)
            g.add((iri, INSIGHT.isSystemReaction, Literal(True)))
            if phase.get("note"):
                g.add((iri, RDFS.comment, Literal(phase["note"])))
            if phase.get("stopped_at_s") is not None:
                g.add((iri, INSIGHT.stoppedAtS, _number(phase["stopped_at_s"])))
        else:
            iri = individual(phase["id"], DUL.Action)
            g.add((iri, INSIGHT.motionKind, Literal(phase["kind"])))
            g.add((iri, INSIGHT.meanSpeedMps, _decimal(phase["mean_speed_mps"])))
            if phase["kind"] in LOCOMOTION:
                g.add((iri, DUL.isClassifiedBy, LOCOMOTION[phase["kind"]]))
            for key, props in (("setpoint_mps", (INSIGHT.setpointMinMps, INSIGHT.setpointMaxMps)),
                               ("heading_deg_range", (INSIGHT.headingMinDeg, INSIGHT.headingMaxDeg))):
                for prop, value in zip(props, phase.get(key) or ()):
                    g.add((iri, prop, _number(value)))
        g.add((iri, INSIGHT.listIndex, _index(index)))
        g.add((iri, DUL.hasTimeInterval, intervals[phase["interval"]]))
        if robot is not None:
            g.add((iri, DUL.hasParticipant, robot))
    for kind, concept in LOCOMOTION.items():
        g.add((concept, RDF.type, OWL.NamedIndividual))
        g.add((concept, RDF.type, SOMA.Locomotion))
        g.add((concept, RDFS.label, Literal(kind)))

    # Path segments and their regions.
    for index, segment in enumerate(episode.get("segments", [])):
        iri = individual(segment["id"], INSIGHT.PathSegment)
        g.add((iri, INSIGHT.listIndex, _index(index)))
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

    # The estimated support is a description. Using hasParticipant or hasTimeInterval here
    # would infer dul:Event through their domains, undoing the distinction from a physical state.
    support = episode.get("support") or {}
    support_iri = None
    if support:
        support_iri = individual(support["id"], INSIGHT.SupportEstimate)
        support_kind = individual("EstimatedSupportKind", SOMA.SupportState)
        g.add((support_iri, DUL.usesConcept, support_kind))
        for key, prop in (("supporter", INSIGHT.estimatedSupporter), ("supported", INSIGHT.estimatedSupportedObject)):
            if support.get(key):
                g.add((support_iri, prop, entity_iri(episode, support[key])))
        g.add((support_iri, RDFS.comment, Literal(
            "Estimate from the represented carrying relation and robot self-model; not a measured physical support state.")))
        if support.get("start_s") is not None and support.get("end_s") is not None:
            g.add((support_iri, INSIGHT.estimateValidDuring,
                   interval("Interval_support", support["start_s"], support["end_s"])))

    # A recorded change in representation, without inferring a physical accident or object pose.
    section = observation_section(episode)
    observed = episode[section]
    observed_iri = individual(observation_id(episode), INSIGHT.ObservedAnomaly)
    g.add((observed_iri, INSIGHT.contractSection, Literal(section)))
    if observed.get("id"):
        g.add((observed_iri, INSIGHT.recordedId, Literal(observed["id"])))
    for key, prop in (("observed_as", INSIGHT.observedAs), ("summary", INSIGHT.changeSummary),
                      ("affected_entity", INSIGHT.declaredAffectedEntityId)):
        if isinstance(observed.get(key), str):
            g.add((observed_iri, prop, Literal(observed[key])))
    if observed.get("mission") is not None:
        mission = observed["mission"]
        g.add((observed_iri, INSIGHT.missionDescription,
               Literal(mission if isinstance(mission, str) else json.dumps(mission, ensure_ascii=False))))
    for profile_id in observed.get("profile_ids") or []:
        g.add((observed_iri, INSIGHT.declaredProfileId, Literal(str(profile_id))))
    for index, statement in enumerate(observed.get("unknowns") or []):
        unknown = individual(f"Unknown_{index + 1}", INSIGHT.StatedUnknown)
        g.add((unknown, INSIGHT.listIndex, _index(index)))
        g.add((unknown, RDFS.comment, Literal(str(statement))))
        g.add((observed_iri, INSIGHT.hasStatedUnknown, unknown))

    # The changes of the working memory in which it was observed: those of its record, and the
    # delta the live validator found unexplained, in their order.
    changes = []
    if isinstance(observed.get("changes"), list):
        changes += [("changes", change) for change in observed["changes"] if isinstance(change, dict)]
    elif observed.get("operation"):
        changes.append(("change", observed))
    if trigger:
        if trigger.get("unexplained_reason") is not None:
            g.add((observed_iri, INSIGHT.validatorReason, Literal(str(trigger["unexplained_reason"]))))
        for key, operation in (("removed_triples", "removed"), ("added_triples", "added")):
            changes += [(key, {"operation": operation, **triple}) for triple in trigger.get(key) or []]
    for index, (origin, change) in enumerate(changes):
        recorded = individual(f"Change_{index + 1}", INSIGHT.RecordedChange)
        g.add((recorded, INSIGHT.listIndex, _index(index)))
        g.add((recorded, INSIGHT.changeSource, Literal(origin)))
        for key, prop in (("operation", INSIGHT.changeOperation), ("subject", INSIGHT.changeSubject),
                          ("predicate", INSIGHT.changePredicate), ("object", INSIGHT.changeObject)):
            if change.get(key) is not None:
                g.add((recorded, prop, Literal(str(change[key]))))
        g.add((observed_iri, INSIGHT.recordedAs, recorded))
    time_s = observed.get("time_s", (episode.get("time") or {}).get("t_obs_s"))
    if time_s is not None:
        g.add((observed_iri, INSIGHT.timeS, _decimal(time_s)))
    affected = observed.get("affected_entity") or observed.get("subject")
    if not affected and not episode.get("change") and not episode.get("observation"):
        affected = support.get("supported")
    if affected:
        g.add((observed_iri, INSIGHT.affectedEntity, entity_iri(episode, affected)))
    if robot is not None:
        g.add((observed_iri, INSIGHT.observedBy, robot))
    if observed.get("place") in {s["id"] for s in episode.get("segments", [])}:
        g.add((observed_iri, INSIGHT.observedAtSegment, ep[observed["place"]]))
    pose = observed.get("robot_pose") or {}
    xy = pose.get("xy")
    if xy is not None:
        for prop, value in zip((INSIGHT.observerX, INSIGHT.observerY), xy):
            if value is not None:
                g.add((observed_iri, prop, _decimal(value)))
    if pose.get("heading_deg") is not None:
        g.add((observed_iri, INSIGHT.observerHeadingDeg, _decimal(pose["heading_deg"])))
    if support_iri is not None and support.get("ended_by") == observed["id"]:
        g.add((observed_iri, INSIGHT.invalidatesEstimate, support_iri))
    summary = observed.get("summary") or observed.get("observed_as") or "Recorded representation change"
    g.add((observed_iri, RDFS.comment, Literal(f"Observed as {summary}. The physical outcome is not established by this change.")))
    # The system reacted to the perceived change; the physical event remains unknown.
    for phase in episode.get("phases", []):
        if phase.get("is_system_reaction"):
            g.add((ep[phase["id"]], SOMA.isReactionTo, observed_iri))

    # Evidence: one observation per entry of the fixed list.
    for index, entry in enumerate(episode.get("evidence", [])):
        iri = individual(entry["id"], SOSA.Observation)
        g.add((iri, INSIGHT.listIndex, _index(index)))
        g.add((iri, SOSA.observedProperty, INSIGHT[entry["property"]]))
        for sensor in SENSORS_BY_PROPERTY.get(entry["property"], []):
            g.add((iri, SOSA.madeBySensor, sensor))
        value = entry.get("value")
        if value is not None:
            g.add((iri, SOSA.hasSimpleResult, _number(value)))
        if entry.get("interval"):
            g.add((iri, SOSA.phenomenonTime, intervals[entry["interval"]]))
        for key, prop in (("at_s", INSIGHT.atTimeS), ("baseline", INSIGHT.baselineValue),
                          ("commanded", INSIGHT.commandedValue), ("signed_value", INSIGHT.signedValue),
                          ("own_ratio_fall", INSIGHT.travelledOverCommanded),
                          ("own_ratio_baseline", INSIGHT.travelledOverCommandedBaseline)):
            if entry.get(key) is not None:
                g.add((iri, prop, _number(entry[key])))
        if entry.get("unit"):
            g.add((iri, INSIGHT.unit, Literal(entry["unit"])))
        if entry.get("is_system_reaction"):
            g.add((iri, INSIGHT.isSystemReaction, Literal(True)))

    for iri in individuals + [entity_iri(episode, value) for value in entities.values() if isinstance(value, str)]:
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
