"""Offline test: the episode of contract 1, built from a recording (agents/semantic/src/episode_builder.py).

Checks:
  * the core alone, on a synthetic recording: segments counted back from the fall (a short first one
    joins the next), phases from the first motion, peaks in the fall interval, nothing after t_obs
    except what is marked as the reaction, and the bottle re-acquired;
  * the 12:29 recording of 2026-09-24 read with experiments/episode_series.py gives the figures of
    the worked example in the contract (section 2.4);
  * the other four falls of that session build a consistent episode, and in all five the largest
    pitch rate before the fall lies in the fall interval (subpaso 1.3; the robot stalls against the
    bump in two of them);
  * the production reader (episodic_memory_api) and the file reader give the same episode for the
    five falls (skipped if pydsr / episodic_memory_api cannot be imported);
  * the episode in RDF (agents/semantic/src/episode_rdf.py) only uses terms of the INSIGHT TBox,
    answers "when and where did the bottle fall?", and, with owlready2, HermiT finds the TBox plus
    the episode consistent (~20 s).

The recordings are not in git (agents/mission_controller/recorded_missions/), nor is experiments/:
both travel in the hand-over package.

Run from the repo root:  python3 tests/offline/test_episode_builder.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RECORDINGS = REPO / "agents" / "mission_controller" / "recorded_missions"
sys.path.insert(0, str(REPO / "agents" / "semantic"))
sys.path.insert(0, str(REPO / "experiments"))

from src.episode_builder import RecordedSeries, build_episode  # noqa: E402
from src.episode_rdf import SENSORS_BY_PROPERTY, episode_dataset, episode_graph, graph_iri  # noqa: E402

CASE_1229 = RECORDINGS / "mission_Follow_Person_24092026_122924.txt"
OTHER_FALLS_2409 = ["111727", "124030", "125706", "133323"]
TBOX = REPO / "agents" / "semantic" / "data" / "insight_tbox.ttl"

PREFIXES = """
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
PREFIX dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
PREFIX sosa: <http://www.w3.org/ns/sosa/>
PREFIX insight: <http://insight.local/ontology#>
PREFIX inst: <http://insight.local/instances#>
"""


def close(actual, expected, tolerance, what):
    assert actual is not None and abs(actual - expected) <= tolerance, f"{what}: {actual} != {expected} ± {tolerance}"


def by_id(items):
    return {item["id"]: item for item in items}


# --------------------------------------------------------------------------- #
# The core on a synthetic recording
# --------------------------------------------------------------------------- #
def synthetic_series(bottle_back_at=None, stalls_at=None) -> RecordedSeries:
    """Straight along +x at 0.25 m/s from 0.5 s; the bottle edge is deleted at 12.8 s; the
    setpoint goes to 0 at 12.9 s and the robot brakes. With `stalls_at`, the robot stops there
    while still ordered to move, and the bottle falls later."""
    t_obs, start, speed = 12.8, 0.5, 0.25
    pose_t = np.round(np.arange(0.0, 20.0, 0.1), 3)
    moving_until = stalls_at if stalls_at is not None else 13.2
    travelled = speed * (np.clip(pose_t, start, moving_until) - start)
    pose_xy = np.column_stack([-3.0 + travelled, np.zeros_like(pose_t)])
    pose_yaw = np.full_like(pose_t, -math.pi / 2)          # DSR yaw -90 deg: heading +x

    acc_t = np.round(np.arange(0.0, 20.0, 0.08), 3)
    acc = np.tile([0.0, 0.0, 9.81], (len(acc_t), 1))
    acc[np.argmin(abs(acc_t - 12.4)), 2] += 2.0           # vertical bump inside the fall interval
    acc[np.argmin(abs(acc_t - 13.12)), 1] = -4.0          # braking of the reaction
    gyro_t = np.round(np.arange(0.0, 20.0, 0.02), 3)
    gyro = np.zeros((len(gyro_t), 3))
    gyro[np.argmin(abs(gyro_t - 5.0)), 0] = 0.1           # baseline
    gyro[np.argmin(abs(gyro_t - 12.0)), 0] = -0.8         # pitch peak before the fall
    gyro[np.argmin(abs(gyro_t - 13.0)), 0] = 5.0          # after t_obs: must not count

    adv = np.array([[t, 0.5] for t in np.arange(0.2, t_obs, 0.5)] + [[12.9, 0.0]])
    bottle_edge = [(0.0, True), (5.0, True), (t_obs, False)]
    if bottle_back_at is not None:
        bottle_edge.append((bottle_back_at, True))
    return RecordedSeries(
        path="synthetic.txt", sha256="0" * 64, reader="test", origin_ns=0,
        origin_offset_from_file_start_s=0.0, length_s=19.9,
        pose_t=pose_t, pose_xy=pose_xy, pose_yaw=pose_yaw,
        acc_t=acc_t, acc=acc, gyro_t=gyro_t, gyro=gyro,
        adv=adv, rot=np.array([[0.0, 0.0]]),
        person_t=pose_t, person_xy=np.tile([0.0, 3.0], (len(pose_t), 1)),
        bottle_edge=bottle_edge)


def check_core_on_synthetic_series():
    episode = build_episode(synthetic_series())
    json.dumps(episode)
    assert episode["schema"] == "insight.episode/1.0"
    assert episode["episode_id"] == "rec_synthetic"
    close(episode["time"]["t_obs_s"], 12.8, 1e-9, "t_obs")
    close(episode["time"]["simulation_horizon_s"], 15.8, 1e-9, "horizon")

    # 0.25 m/s for 12.3 s = 3.075 m: two 1 m segments, and the 0.075 m start joins the third.
    segments = episode["segments"]
    assert [s["id"] for s in segments] == ["Segment_final", "Segment_prev_1", "Segment_prev_2"], segments
    for segment, length in zip(segments, (1.0, 1.0, 1.075)):
        close(segment["length_m"], length, 1e-3, f"{segment['id']} length")
        close(segment["heading_deg"], 0.0, 1e-6, f"{segment['id']} heading")
        close(segment["mean_speed_mps"], 0.25, 1e-3, f"{segment['id']} speed")
    assert segments[0]["to_xy"] == episode["accident"]["robot_pose"]["xy"] == [0.075, 0.0]
    assert episode["accident"]["place"] == "Segment_final"
    close(episode["accident"]["robot_pose"]["heading_deg"], 0.0, 1e-6, "heading at the fall")

    # One phase from the first motion; the wait before it is not a phase.
    phases = by_id(episode["phases"])
    assert phases["Phase_1"]["kind"] == "advance_straight"
    intervals = by_id(episode["intervals"])
    assert (intervals["Interval_Phase_1"]["start_s"], intervals["Interval_Phase_1"]["end_s"]) == (0.5, 12.8)
    assert intervals["Interval_Segment_prev_2"]["start_s"] == 0.5
    assert (intervals["Interval_fall"]["start_s"], intervals["Interval_baseline"]["end_s"]) == (10.8, 10.8)
    assert phases["Phase_reaction"]["is_system_reaction"] is True
    close(phases["Phase_reaction"]["stopped_at_s"], 13.2, 1e-9, "robot stopped")

    evidence = by_id(episode["evidence"])
    pitch = evidence["Ev_pitch_rate_peak"]
    assert (pitch["value"], pitch["signed_value"], pitch["at_s"], pitch["baseline"]) == (0.8, -0.8, 12.0, 0.1), pitch
    close(evidence["Ev_vertical_accel_peak"]["value"], 2.0, 1e-9, "vertical peak")
    close(evidence["Ev_speed_ratio"]["value"], 1.0, 1e-3, "speed ratio")
    close(evidence["Ev_person_distance"]["value"], 3.0, 1e-9, "person distance")
    assert evidence["Ev_setpoint_step"]["value"] == 0.0
    assert evidence["Ev_bottle_reacquired"]["value"] is False
    close(evidence["Ev_reaction_onset"]["value"], 12.9, 1e-9, "reaction onset")
    braking = evidence["Ev_reaction_braking"]
    assert (braking["value"], braking["at_s"], braking["is_system_reaction"]) == (-4.0, 13.12, True), braking

    back = by_id(build_episode(synthetic_series(bottle_back_at=15.0))["evidence"])["Ev_bottle_reacquired"]
    assert back == {"id": "Ev_bottle_reacquired", "property": "bottle_reacquired", "value": True, "at_s": 15.0}, back

    # Stalled from 9 s, fall at 12.8 s: the fall interval starts 2 s before the stop, and the
    # free-motion baseline ends there.
    stalled = build_episode(synthetic_series(stalls_at=9.0))
    phases = [(p["kind"], p["interval"]) for p in stalled["phases"] if not p.get("is_system_reaction")]
    assert [kind for kind, _ in phases] == ["advance_straight", "stopped"], phases
    intervals = by_id(stalled["intervals"])
    assert intervals["Interval_Phase_2"]["start_s"] == 9.0
    assert (intervals["Interval_fall"]["start_s"], intervals["Interval_fall"]["end_s"]) == (7.0, 12.8)
    assert intervals["Interval_baseline"]["end_s"] == 7.0


# --------------------------------------------------------------------------- #
# The 12:29 recording against the contract's worked example
# --------------------------------------------------------------------------- #
def check_episode_1229():
    from episode_series import episode_from_recording

    assert CASE_1229.exists(), f"missing {CASE_1229}"
    episode = episode_from_recording(CASE_1229)
    json.dumps(episode)
    assert episode["episode_id"] == "rec_mission_Follow_Person_24092026_122924"
    assert episode["source"]["path"] == str(CASE_1229.relative_to(REPO))
    assert episode["source"]["sha256"] == "1d90942bb8b5ed2ab207829888e4ecdaa495532ef5d2a287ff1f0f076325df9e"

    time = episode["time"]
    close(time["origin_offset_from_file_start_s"], 0.020, 0.001, "time origin")
    close(time["t_obs_s"], 13.685, 0.001, "t_obs")
    close(time["episode_length_s"], 65.056, 0.001, "L")
    close(time["simulation_horizon_s"], 16.685, 0.001, "horizon")

    intervals = by_id(episode["intervals"])
    expected_intervals = {
        "Interval_fall": (11.685, 13.685), "Interval_baseline": (1.0, 11.685),
        "Interval_Phase_1": (0.30, 13.685), "Interval_Segment_final": (9.19, 13.685),
        "Interval_Segment_prev_1": (5.07, 9.19), "Interval_Segment_prev_2": (1.20, 5.07),
        "Interval_Segment_prev_3": (0.30, 1.20), "Interval_reaction": (13.79, 65.056),
    }
    assert set(intervals) == set(expected_intervals), sorted(intervals)
    for interval_id, (start, end) in expected_intervals.items():
        close(intervals[interval_id]["start_s"], start, 0.01, f"{interval_id} start")
        close(intervals[interval_id]["end_s"], end, 0.01, f"{interval_id} end")

    phases = by_id(episode["phases"])
    assert list(phases) == ["Phase_1", "Phase_reaction"], list(phases)
    phase = phases["Phase_1"]
    assert phase["kind"] == "advance_straight"
    close(phase["mean_speed_mps"], 0.24, 0.005, "Phase_1 speed")
    assert phase["setpoint_mps"] == [0.35, 0.59], phase["setpoint_mps"]
    close(phase["heading_deg_range"][0], -12, 1.0, "Phase_1 min heading")
    close(phase["heading_deg_range"][1], 2, 1.0, "Phase_1 max heading")
    assert phases["Phase_reaction"]["is_system_reaction"] is True

    segments = episode["segments"]
    expected_segments = [
        ("Segment_final", (-1.51, -0.41), (-0.51, -0.39), 1.0, 1.2, 0.22),
        ("Segment_prev_1", (-2.51, -0.42), (-1.51, -0.41), 1.0, 0.4, 0.24),
        ("Segment_prev_2", (-3.50, -0.34), (-2.51, -0.42), 1.0, -4.4, 0.26),
        ("Segment_prev_3", (-3.70, -0.30), (-3.50, -0.34), 0.20, -11.4, 0.22),
    ]
    assert [s["id"] for s in segments] == [e[0] for e in expected_segments]
    for segment, (segment_id, start, end, length, heading, speed) in zip(segments, expected_segments):
        for axis in (0, 1):
            close(segment["from_xy"][axis], start[axis], 0.01, f"{segment_id} from")
            close(segment["to_xy"][axis], end[axis], 0.01, f"{segment_id} to")
        close(segment["length_m"], length, 0.01, f"{segment_id} length")
        close(segment["heading_deg"], heading, 0.2, f"{segment_id} heading")
        close(segment["mean_speed_mps"], speed, 0.01, f"{segment_id} speed")

    support, accident = episode["support"], episode["accident"]
    assert (support["start_s"], support["end_s"], support["ended_by"]) == (0.0, 13.685, "Accident_1"), support
    # The tray holds the bottle (contract 1, version 1.7); where the lost bottle went is not said.
    assert episode["entities"]["tray"] == support["supporter"] == "PhysicalObject_Tray", support
    close(accident["time_s"], 13.685, 0.001, "accident time")
    assert accident["place"] == "Segment_final"
    assert accident["participants"] == ["Agent_Robot", "PhysicalObject_Bottle"]
    close(accident["robot_pose"]["xy"][0], -0.51, 0.01, "fall x")
    close(accident["robot_pose"]["xy"][1], -0.39, 0.01, "fall y")
    close(accident["robot_pose"]["heading_deg"], 1.6, 0.1, "fall heading")

    evidence = by_id(episode["evidence"])
    # (id, value, tolerance, at_s, baseline)
    peaks = [
        ("Ev_pitch_rate_peak", 0.95, 0.005, 12.93, 0.009),
        ("Ev_vertical_accel_peak", 2.39, 0.005, 13.63, 0.018),
        ("Ev_longitudinal_accel_peak", 3.13, 0.005, 13.31, 2.05),
        ("Ev_lateral_accel_peak", 0.37, 0.005, 13.63, 0.13),
        ("Ev_yaw_rate_peak", 0.15, 0.01, None, None),
    ]
    for evidence_id, value, tolerance, at_s, baseline in peaks:
        entry = evidence[evidence_id]
        assert entry["interval"] == "Interval_fall", entry
        close(entry["value"], value, tolerance, evidence_id)
        if at_s is not None:
            close(entry["at_s"], at_s, 0.005, f"{evidence_id} instant")
        if baseline is not None:
            close(entry["baseline"], baseline, 0.005, f"{evidence_id} baseline")
    close(evidence["Ev_yaw_change"]["value"], 0.3, 0.1, "yaw change")
    close(evidence["Ev_yaw_change"]["commanded"], 0.1, 0.05, "commanded yaw change")
    # The contract says "about 0.01": the filtered setpoint falls ~0.01 m/s every 0.5 s, so a
    # 0.5 s window spans two of those steps (0.0197). Either way, no step was ordered.
    assert evidence["Ev_setpoint_step"]["value"] <= 0.02, evidence["Ev_setpoint_step"]
    ratio = evidence["Ev_speed_ratio"]
    close(ratio["value"], 1.08, 0.005, "speed ratio")
    close(ratio["own_ratio_fall"], 0.49, 0.005, "own ratio, fall")
    close(ratio["own_ratio_baseline"], 0.46, 0.005, "own ratio, baseline")
    sustained = evidence["Ev_sustained_accel"]
    assert sustained["interval"] == "Interval_Phase_1", sustained
    close(sustained["value"], 3.41, 0.01, "sustained acceleration")
    close(sustained["at_s"], 0.30, 0.01, "sustained acceleration instant")
    close(evidence["Ev_person_distance"]["value"], 3.81, 0.005, "person distance")
    assert evidence["Ev_bottle_reacquired"]["value"] is False
    close(evidence["Ev_reaction_onset"]["value"], 13.79, 0.005, "reaction onset")
    braking = evidence["Ev_reaction_braking"]
    assert braking["is_system_reaction"] is True and braking["interval"] == "Interval_reaction"
    close(braking["value"], -3.92, 0.005, "reaction braking")
    close(braking["at_s"], 13.98, 0.005, "reaction braking instant")
    check_consistency(episode)


def check_consistency(episode):
    """What every episode must satisfy, whatever the recording."""
    t_obs = episode["time"]["t_obs_s"]
    intervals = by_id(episode["intervals"])
    segments = episode["segments"]
    assert segments[0]["id"] == "Segment_final" and episode["accident"]["place"] == "Segment_final"
    assert segments[0]["to_xy"] == episode["accident"]["robot_pose"]["xy"]
    for later, earlier in zip(segments, segments[1:]):
        assert earlier["to_xy"] == later["from_xy"], (earlier["id"], later["id"])
        assert intervals[earlier["interval"]]["end_s"] == intervals[later["interval"]]["start_s"]
    assert intervals["Interval_Segment_final"]["end_s"] == t_obs
    for entry in episode["evidence"]:
        if entry.get("is_system_reaction") or entry["property"] in {"bottle_reacquired", "reaction_onset"}:
            continue
        assert intervals[entry["interval"]]["end_s"] <= t_obs, entry
        assert entry.get("at_s") is None or entry["at_s"] <= t_obs, entry


def check_fall_interval_holds_the_pitch_peak_2409():
    """Subpaso 1.3, criterion fixed before running: in the five falls of 2026-09-24 (bump), the
    largest pitch rate of the whole episode before the fall lies inside Interval_fall. In 12:40 and
    13:33 the robot stalls against the bump seconds before the bottle falls."""
    from episode_series import read_series

    for stamp in ["122924"] + OTHER_FALLS_2409:
        series = read_series(RECORDINGS / f"mission_Follow_Person_24092026_{stamp}.txt")
        episode = build_episode(series)
        t_obs = episode["time"]["t_obs_s"]
        before = series.gyro_t <= t_obs
        at = float(series.gyro_t[before][np.argmax(np.abs(series.gyro[before, 0]))])
        fall = by_id(episode["intervals"])["Interval_fall"]
        assert fall["start_s"] <= at <= fall["end_s"], (stamp, at, fall)
        pitch = by_id(episode["evidence"])["Ev_pitch_rate_peak"]
        assert pitch["value"] > pitch["baseline"], (stamp, pitch)


def check_other_falls_2409():
    from episode_series import episode_from_recording

    for stamp in OTHER_FALLS_2409:
        recording = RECORDINGS / f"mission_Follow_Person_24092026_{stamp}.txt"
        assert recording.exists(), f"missing {recording}"
        episode = episode_from_recording(recording)
        json.dumps(episode)
        check_consistency(episode)


# --------------------------------------------------------------------------- #
# The production reader
# --------------------------------------------------------------------------- #
def differences(a, b, path="", tolerance=2e-3):
    """Paths where two JSON-like values differ (numbers within `tolerance`: the API gives float32)."""
    if isinstance(a, dict) and isinstance(b, dict):
        found = [f"{path}/{k}" for k in set(a) ^ set(b)]
        return found + [d for k in set(a) & set(b) for d in differences(a[k], b[k], f"{path}/{k}", tolerance)]
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"{path}: {len(a)} != {len(b)} items"]
        return [d for i, (x, y) in enumerate(zip(a, b)) for d in differences(x, y, f"{path}[{i}]", tolerance)]
    numbers = (int, float)
    if isinstance(a, numbers) and isinstance(b, numbers) and not isinstance(a, bool) and not isinstance(b, bool):
        return [] if abs(a - b) <= tolerance else [f"{path}: {a} != {b}"]
    return [] if a == b else [f"{path}: {a!r} != {b!r}"]


def check_readers_agree():
    """The episodic-memory API (production) and the file reader give the same episode."""
    from episode_series import episode_from_recording

    try:
        from src.episode_memory_reader import episode_from_memory, open_recording
        open_recording(CASE_1229)
    except ImportError as error:
        print(f"  SKIP: the episodic-memory API cannot be imported ({error})")
        return
    for stamp in ["122924"] + OTHER_FALLS_2409:
        recording = RECORDINGS / f"mission_Follow_Person_24092026_{stamp}.txt"
        from_file, from_memory = episode_from_recording(recording), episode_from_memory(recording)
        assert from_memory["source"]["reader"] == "episodic_memory_api"
        for episode in (from_file, from_memory):
            episode["source"].pop("reader")
            episode["source"].pop("path")
        found = differences(from_file, from_memory)
        assert not found, (stamp, found[:5])


# --------------------------------------------------------------------------- #
# The episode in RDF
# --------------------------------------------------------------------------- #
def check_rdf_1229():
    from episode_series import episode_from_recording
    from rdflib import OWL, RDF, RDFS, Graph, URIRef

    episode = episode_from_recording(CASE_1229)
    graph = episode_graph(episode)
    tbox = Graph().parse(TBOX)

    # Only TBox vocabulary.
    properties = {s for kind in (OWL.ObjectProperty, OWL.DatatypeProperty) for s in tbox.subjects(RDF.type, kind)}
    classes = set(tbox.subjects(RDF.type, OWL.Class))
    used_properties = set(graph.predicates()) - {RDF.type, RDFS.label, RDFS.comment}
    used_classes = set(graph.objects(None, RDF.type)) - {OWL.NamedIndividual}
    assert used_properties <= properties, used_properties - properties
    assert used_classes <= classes, used_classes - classes

    # Each observation is made by sensors that observe its property, as the TBox says.
    sosa = "http://www.w3.org/ns/sosa/"
    for prop, sensors in SENSORS_BY_PROPERTY.items():
        target = URIRef("http://insight.local/ontology#" + prop)
        for sensor in sensors:
            assert (sensor, URIRef(sosa + "observes"), target) in tbox, (sensor, prop)

    # The observation locates the robot, not the object; the invalidated support is an estimate.
    dataset = episode_dataset(episode)
    rows = list(dataset.query(PREFIXES + """
        SELECT ?g ?t ?place ?x ?y ?state WHERE { GRAPH ?g {
          ?observed a insight:ObservedAnomaly ; insight:affectedEntity inst:PhysicalObject_Bottle ; insight:timeS ?t ;
                insight:observedAtSegment ?place ; insight:invalidatesEstimate ?state ;
                insight:observerX ?x ; insight:observerY ?y .
          ?state a insight:SupportEstimate } }"""))
    assert len(rows) == 1, rows
    g, t, place, x, y, state = rows[0]
    assert g == graph_iri(episode["episode_id"])
    assert (float(t), float(x), float(y)) == (13.685, -0.507, -0.388), rows[0]
    assert str(place).endswith("#Segment_final") and str(state).endswith("#Support_bottle")

    # With the TBox: what the contrast reads, and what it must leave out.
    merged = tbox + graph
    commanded = list(merged.query(PREFIXES + """
        SELECT ?recorded ?commanded WHERE {
          ?o sosa:observedProperty insight:yaw_change ; sosa:hasSimpleResult ?recorded ;
             insight:commandedValue ?commanded ; sosa:phenomenonTime ?i .
          ?i rdfs:label "Interval_fall" }"""))
    assert [(float(a), float(b)) for a, b in commanded] == [(0.35, 0.11)], commanded
    reaction = {str(r[0]).split("#")[1] for r in merged.query(PREFIXES + """
        SELECT ?x WHERE { ?x insight:isSystemReaction true }""")}
    assert reaction == {"Phase_reaction", "Ev_reaction_braking"}, reaction
    # The stop is a reaction to the perceived change, not proof of a physical fall.
    reacts_to = [tuple(str(x).split("#")[1] for x in row) for row in merged.query(PREFIXES + """
        SELECT ?x ?fall WHERE { ?x soma:isReactionTo ?fall }""")]
    assert reacts_to == [("Phase_reaction", "Accident_1")], reacts_to
    events ={str(r[0]).split("#")[1] for r in merged.query(PREFIXES + """
        SELECT ?x WHERE { ?x a/rdfs:subClassOf* dul:Event . FILTER(STRSTARTS(STR(?x), "http://insight.local/episodes/")) }""")}
    assert {"Accident_1", "Phase_1", "Phase_reaction"} <= events, events
    assert "Support_bottle" not in events, events
    assert not bool(merged.query(PREFIXES + "ASK { ?x a/rdfs:subClassOf* soma:Accident }").askAnswer)
    # The tray is a known component; its support of the bottle is an estimate, not a physical state.
    assert bool(merged.query(PREFIXES + """
        ASK { inst:Agent_Robot dul:hasComponent inst:PhysicalObject_Tray .
              inst:PhysicalObject_Tray a/rdfs:subClassOf* dul:PhysicalObject .
              ?estimate a insight:SupportEstimate ; insight:estimatedSupporter inst:PhysicalObject_Tray ;
                     insight:estimatedSupportedObject inst:PhysicalObject_Bottle ;
                     insight:estimateValidDuring ?i }""").askAnswer)


def check_rdf_consistency_if_possible():
    try:
        import owlready2
    except ImportError:
        print("  (owlready2 not installed: HermiT consistency check skipped)")
        return
    import tempfile

    from episode_series import episode_from_recording
    from rdflib import Graph

    merged = Graph().parse(TBOX) + episode_graph(episode_from_recording(CASE_1229))
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "episode_with_tbox.owl"
        merged.serialize(path, format="xml")
        world = owlready2.World()
        ontology = world.get_ontology(path.as_uri()).load()
        with ontology:
            owlready2.sync_reasoner_hermit(world, infer_property_values=False, debug=0)
        unsatisfiable = list(world.inconsistent_classes())
    assert not unsatisfiable, unsatisfiable
    print("  HermiT: the TBox plus the 12:29 episode is consistent")


def main():
    for check in (check_core_on_synthetic_series, check_episode_1229, check_other_falls_2409,
                  check_fall_interval_holds_the_pitch_peak_2409, check_readers_agree, check_rdf_1229,
                  check_rdf_consistency_if_possible):
        check()
        print(f"OK {check.__name__}")
    print("Episode builder: all checks passed")


if __name__ == "__main__":
    main()
