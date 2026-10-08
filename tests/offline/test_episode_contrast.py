"""Offline test: the contrast of the mechanisms with the episode (agents/semantic/src/episode_contrast.py).

Checks:
  * the mechanisms and their checks come from the INSIGHT TBox as contract 2 lists them (section
    3.3 and 3.6), and a TBox with a check the module does not implement, or whose properties differ
    from the ones the implementation reads, is refused;
  * on a hand-made episode, each check lands on the right side of its threshold; a missing value
    never discards; the evidence of the system's reaction is never read, although the braking shares
    its property name with the longitudinal peak before the fall; a rule says nothing about
    anchors its evidence does not cover; the recorded turn is compared with the ordered one scaled
    by the episode's own clock (contract 2, version 1.2), so a robot turning in free motion under
    the Webots clock does not keep a wheel failure alive;
  * carrying-relation restoration, its absence within the recording and missing evidence only
    inform: perception failure remains unresolved and not simulable (contracts 1.10);
  * the 12:29 recording of 2026-09-24 gives the column "12:29" of the contract's table 3.6;
  * in the five falls of that session (all with the bump), obstacle_traversed is never discarded
    nor found incoherent (criterion of A3), and the rest of the table is the one in
    docs_output/resultados/05_contraste_giro_2409/, except for the informative restoration check.

The recordings are not in git (agents/mission_controller/recorded_missions/), nor is experiments/:
both travel in the hand-over package.

Run from the repo root:  python3 tests/offline/test_episode_contrast.py
"""
import copy
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
RECORDINGS = REPO / "agents" / "mission_controller" / "recorded_missions"
TBOX = REPO / "agents" / "semantic" / "data" / "insight_tbox.ttl"
sys.path.insert(0, str(REPO / "agents" / "semantic"))
sys.path.insert(0, str(REPO / "experiments"))

from src.episode_contrast import (  # noqa: E402
    DISCARDED, EXPLAINED_BY_MEMORY, INCOHERENT, SURVIVES_CONTRAST,
    contrast_episode, contrast_hypothesis, load_mechanisms,
)

# Contract 2, sections 3.3 and 3.6: preconditions, then (rule, effect, status).
EXPECTED_CHECKS = {
    "obstacle_traversed": (["moving_on_segment"], [("obstacle_jolt", "inform", "to_calibrate")]),
    "bottle_push": (["bottle_supported"], []),
    "robot_push": ([], [("robot_jolt", "discard", "to_calibrate")]),
    "slippery_floor": ([], [("traction_vs_friction", "discard", "fixed")]),
    "wheel_failure": (["moving_in_interval"], [("turn_vs_command", "discard", "to_calibrate")]),
    "commanded_speed_change": ([], [("setpoint_step", "explain_or_discard", "to_calibrate")]),
    "uncommanded_motion": (["moving_in_interval"], [("speed_ratio_and_jolt", "discard", "to_calibrate")]),
    "bottle_removed_by_person": ([], [("person_within_reach", "discard", "fixed")]),
    "perception_failure": ([], [("bottle_reacquired", "inform", "fixed")]),
    "suspension_failure": ([], []),
}


def by_mechanism(contrast):
    return {result["mechanism"]: result for result in contrast["results"]}


def check_of(result, rule_id):
    return next(c for c in result["checks"] if c["check"] == rule_id)


# --------------------------------------------------------------------------- #
# The TBox
# --------------------------------------------------------------------------- #
def check_mechanisms_from_tbox():
    mechanisms = load_mechanisms(TBOX)
    assert set(mechanisms) == set(EXPECTED_CHECKS), sorted(mechanisms)
    for mechanism_id, (preconditions, rules) in EXPECTED_CHECKS.items():
        mechanism = mechanisms[mechanism_id]
        assert [c.rule_id for c in mechanism.preconditions] == preconditions, mechanism_id
        assert all(c.effect == "incoherent" for c in mechanism.preconditions), mechanism_id
        assert [(c.rule_id, c.effect, c.status) for c in mechanism.rules] == rules, mechanism_id
    assert mechanisms["obstacle_traversed"].anchors == ("segment",)
    assert mechanisms["slippery_floor"].anchors == ()
    assert mechanisms["uncommanded_motion"].realization_experimental
    assert mechanisms["suspension_failure"].realized_by == ()
    assert mechanisms["obstacle_traversed"].realized_by == ("Intervention_spawn_scaled_dome",
                                                            "Intervention_spawn_static_object")


def check_tbox_mismatch_is_refused():
    text = TBOX.read_text(encoding="utf-8")
    unknown = text.replace('insight:ruleId "bottle_reacquired"', 'insight:ruleId "bottle_seen_again"')
    other_property = text.replace("insight:checksProperty insight:setpoint_step ;",
                                  "insight:checksProperty insight:speed_ratio ;")
    assert unknown != text and other_property != text, "the TBox text changed: update this check"
    with tempfile.TemporaryDirectory() as tmp:
        for name, content, message in (("unknown.ttl", unknown, "does not implement"),
                                       ("other.ttl", other_property, "the TBox says it reads")):
            path = Path(tmp) / name
            path.write_text(content, encoding="utf-8")
            try:
                load_mechanisms(path)
            except ValueError as error:
                assert message in str(error), error
            else:
                raise AssertionError(f"{name}: the mismatch was not refused")


# --------------------------------------------------------------------------- #
# The checks on a hand-made episode
# --------------------------------------------------------------------------- #
def synthetic_episode():
    """The 12:29 figures (contract 1, section 2.4), with only what the checks read."""
    def evidence(prop, value, interval="Interval_fall", **extra):
        return {"id": f"Ev_{prop}", "property": prop, "interval": interval, "value": value, **extra}

    return {
        "episode_id": "rec_synthetic",
        "source": {"path": "synthetic.txt", "sha256": "0" * 64},
        "time": {"t_obs_s": 13.685},
        "intervals": [
            {"id": "Interval_fall", "start_s": 11.685, "end_s": 13.685},
            {"id": "Interval_baseline", "start_s": 1.0, "end_s": 11.685},
            {"id": "Interval_Phase_1", "start_s": 0.3, "end_s": 13.685},
            {"id": "Interval_Segment_final", "start_s": 9.19, "end_s": 13.685},
            {"id": "Interval_Segment_prev_1", "start_s": 5.07, "end_s": 9.19},
            {"id": "Interval_reaction", "start_s": 13.79, "end_s": 65.0},
        ],
        "phases": [
            {"id": "Phase_1", "kind": "advance_straight", "interval": "Interval_Phase_1", "mean_speed_mps": 0.24},
            {"id": "Phase_reaction", "kind": "system_reaction", "interval": "Interval_reaction",
             "is_system_reaction": True},
        ],
        "segments": [
            {"id": "Segment_final", "interval": "Interval_Segment_final", "mean_speed_mps": 0.22},
            {"id": "Segment_prev_1", "interval": "Interval_Segment_prev_1", "mean_speed_mps": 0.24},
        ],
        "support": {"start_s": 0.0, "end_s": 13.685},
        "evidence": [
            evidence("pitch_rate_peak", 0.95, baseline=0.009),
            evidence("vertical_accel_peak", 2.39, baseline=0.018),
            evidence("longitudinal_accel_peak", 3.13, baseline=2.05),
            evidence("lateral_accel_peak", 0.37, baseline=0.13),
            evidence("yaw_change", 0.3, commanded=0.1),
            evidence("yaw_rate_peak", 0.15),
            evidence("setpoint_step", 0.01),
            evidence("speed_ratio", 1.08, own_ratio_fall=0.49, own_ratio_baseline=0.457),
            evidence("max_sustained_horizontal_accel", 3.41, interval="Interval_Phase_1"),
            evidence("person_distance_min", 3.81),
            {"id": "Ev_bottle_reacquired", "property": "bottle_reacquired", "value": False},
            {"id": "Ev_reaction_onset", "property": "reaction_onset", "value": 13.79},
            evidence("longitudinal_accel_peak", -3.92, interval="Interval_reaction", is_system_reaction=True),
        ],
    }


def with_evidence(episode, prop, **changes):
    changed = copy.deepcopy(episode)
    entry = next(e for e in changed["evidence"] if e["property"] == prop and not e.get("is_system_reaction"))
    entry.update(changes)
    return changed


def outcome(episode, mechanism_id, anchors=None):
    return contrast_hypothesis(episode, mechanism_id, anchors)["outcome"]


def check_thresholds_on_synthetic_episode():
    episode = synthetic_episode()
    contrast = by_mechanism(contrast_episode(episode))
    expected = {
        "obstacle_traversed": SURVIVES_CONTRAST, "bottle_push": SURVIVES_CONTRAST,
        "robot_push": SURVIVES_CONTRAST, "slippery_floor": DISCARDED, "wheel_failure": DISCARDED,
        "commanded_speed_change": DISCARDED, "uncommanded_motion": DISCARDED,
        "bottle_removed_by_person": DISCARDED, "perception_failure": SURVIVES_CONTRAST,
        "suspension_failure": SURVIVES_CONTRAST,
    }
    assert {m: r["outcome"] for m, r in contrast.items()} == expected, contrast
    assert check_of(contrast["obstacle_traversed"], "obstacle_jolt")["verdict"] == "trace_seen"
    assert contrast["perception_failure"]["status_after_contrast"] == "checked_not_simulable"
    assert contrast["suspension_failure"]["status_after_contrast"] == "not_simulable"
    assert contrast["obstacle_traversed"]["status_after_contrast"] == "pending"

    # Each check on the other side of its threshold.
    slow = copy.deepcopy(episode)
    slow["segments"][0]["mean_speed_mps"] = 0.04
    assert outcome(slow, "obstacle_traversed") == INCOHERENT
    assert outcome(with_evidence(episode, "max_sustained_horizontal_accel", value=3.2), "slippery_floor") == SURVIVES_CONTRAST
    assert outcome(with_evidence(episode, "yaw_change", value=5.0), "wheel_failure") == SURVIVES_CONTRAST
    assert outcome(with_evidence(episode, "yaw_rate_peak", value=0.4), "wheel_failure") == SURVIVES_CONTRAST
    assert outcome(with_evidence(episode, "setpoint_step", value=0.25), "commanded_speed_change") == EXPLAINED_BY_MEMORY
    assert outcome(with_evidence(episode, "speed_ratio", value=0.8), "uncommanded_motion") == SURVIVES_CONTRAST
    assert outcome(with_evidence(episode, "longitudinal_accel_peak", value=4.2), "uncommanded_motion") == SURVIVES_CONTRAST
    calm = with_evidence(episode, "lateral_accel_peak", value=0.25)
    assert outcome(calm, "robot_push") == DISCARDED
    assert outcome(with_evidence(calm, "yaw_change", value=4.0), "robot_push") == SURVIVES_CONTRAST
    near = contrast_hypothesis(with_evidence(episode, "person_distance_min", value=1.2), "bottle_removed_by_person")
    assert near["outcome"] == SURVIVES_CONTRAST and near["status_after_contrast"] == "checked_not_simulable"
    assert outcome(with_evidence(episode, "bottle_reacquired", value=True), "perception_failure") == SURVIVES_CONTRAST
    flat = with_evidence(with_evidence(episode, "pitch_rate_peak", value=0.02), "vertical_accel_peak", value=0.05)
    jolt = check_of(contrast_hypothesis(flat, "obstacle_traversed"), "obstacle_jolt")
    assert jolt["verdict"] == "no_trace", jolt
    assert outcome(flat, "obstacle_traversed") == SURVIVES_CONTRAST     # it only informs

    # A stall: the interval is mostly a stop, and the mean speed is weighted by time.
    stalled = copy.deepcopy(episode)
    stalled["intervals"][2]["end_s"] = 11.885
    stalled["intervals"].append({"id": "Interval_Phase_2", "start_s": 11.885, "end_s": 13.685})
    stalled["phases"].insert(1, {"id": "Phase_2", "kind": "stopped", "interval": "Interval_Phase_2",
                                 "mean_speed_mps": 0.0})
    result = contrast_hypothesis(stalled, "wheel_failure")
    assert result["outcome"] == INCOHERENT, result                          # 0.2 s at 0.24 over 2 s
    assert abs(check_of(result, "moving_in_interval")["conditions"][0]["value"] - 0.024) < 1e-9


def check_missing_values_never_discard():
    episode = synthetic_episode()
    for prop, mechanism_id in (("max_sustained_horizontal_accel", "slippery_floor"),
                               ("yaw_rate_peak", "wheel_failure"), ("speed_ratio", "uncommanded_motion"),
                               ("person_distance_min", "bottle_removed_by_person")):
        result = contrast_hypothesis(with_evidence(episode, prop, value=None), mechanism_id)
        assert result["outcome"] == SURVIVES_CONTRAST, (prop, result)
        assert result["checks"][-1]["verdict"] == "undetermined", (prop, result)
    no_baseline = with_evidence(episode, "longitudinal_accel_peak", baseline=None)
    assert outcome(no_baseline, "uncommanded_motion") == SURVIVES_CONTRAST


def check_restoration_only_informs():
    episode = synthetic_episode()
    for value, verdict in ((True, "trace_seen"), (False, "no_trace"), (None, "undetermined")):
        result = contrast_hypothesis(with_evidence(episode, "bottle_reacquired", value=value), "perception_failure")
        check = check_of(result, "bottle_reacquired")
        assert check["verdict"] == verdict and check["effect"] == "inform", result
        assert check["conditions"][0]["value"] is value, check
        assert "within the available recording" in check["reason"], check
        assert result["outcome"] == SURVIVES_CONTRAST and result["decided_by"] is None, result
        assert result["status_after_contrast"] == "checked_not_simulable", result
    missing = copy.deepcopy(episode)
    missing["evidence"] = [e for e in missing["evidence"] if e["property"] != "bottle_reacquired"]
    result = contrast_hypothesis(missing, "perception_failure")
    assert check_of(result, "bottle_reacquired")["verdict"] == "undetermined", result
    assert result["outcome"] == SURVIVES_CONTRAST, result


def check_reaction_is_never_read():
    episode = synthetic_episode()
    reference = contrast_episode(episode)
    loud = copy.deepcopy(episode)
    loud["evidence"][-1]["value"] = 30.0          # the braking after the fall, longitudinal_accel_peak
    loud["phases"][-1]["mean_speed_mps"] = 0.0
    assert contrast_episode(loud)["results"] == reference["results"]
    # Unmarked, it would be a second longitudinal peak: the view refuses to guess which one.
    unmarked = copy.deepcopy(episode)
    del unmarked["evidence"][-1]["is_system_reaction"]
    try:
        contrast_episode(unmarked)
    except ValueError as error:
        assert "longitudinal_accel_peak" in str(error)
    else:
        raise AssertionError("two longitudinal peaks outside the reaction were accepted")


def check_turn_relative_to_the_episode():
    episode = synthetic_episode()
    # Turning in free motion at the fall: 20 deg ordered, 9.1 executed (the Webots clock, r = 0.457).
    turning = with_evidence(episode, "yaw_change", value=9.1, commanded=20.0)
    result = contrast_hypothesis(turning, "wheel_failure")
    assert result["outcome"] == DISCARDED, result
    condition = check_of(result, "turn_vs_command")["conditions"][0]
    assert abs(condition["value"] - abs(9.1 - 0.457 * 20.0)) < 1e-6, condition
    # The same turn on the real robot (r = 1) is 11 deg short of the order: it stays.
    real_robot = with_evidence(turning, "speed_ratio", own_ratio_baseline=1.0)
    assert outcome(real_robot, "wheel_failure") == SURVIVES_CONTRAST
    # robot_jolt compares the turn the same way.
    calm = with_evidence(turning, "lateral_accel_peak", value=0.25)
    assert outcome(calm, "robot_push") == DISCARDED
    assert outcome(with_evidence(calm, "speed_ratio", own_ratio_baseline=1.0), "robot_push") == SURVIVES_CONTRAST
    # Without the episode's clock (no free motion) the turn is unknown: nothing is discarded.
    no_clock = with_evidence(turning, "speed_ratio", own_ratio_baseline=None)
    assert check_of(contrast_hypothesis(no_clock, "wheel_failure"), "turn_vs_command")["verdict"] == "undetermined"
    assert outcome(no_clock, "wheel_failure") == SURVIVES_CONTRAST


def check_rules_only_speak_for_their_interval():
    episode = synthetic_episode()
    # The yaw evidence is over Interval_fall: it says nothing of a wheel blocked in Segment_prev_1.
    result = contrast_hypothesis(episode, "wheel_failure", {"segment": None, "interval": "Interval_Segment_prev_1"})
    assert result["outcome"] == SURVIVES_CONTRAST, result
    assert check_of(result, "turn_vs_command")["verdict"] == "not_applicable"
    # Nor are the jolt peaks of Interval_fall about a segment travelled before it.
    result = contrast_hypothesis(episode, "obstacle_traversed", {"segment": "Segment_prev_1", "interval": None})
    assert check_of(result, "obstacle_jolt")["verdict"] == "not_applicable", result
    # The floor has no anchor: its rule reads the whole run.
    assert outcome(episode, "slippery_floor", {"segment": None, "interval": None}) == DISCARDED


# --------------------------------------------------------------------------- #
# The recordings of 2026-09-24
# --------------------------------------------------------------------------- #
FALLS_2409 = ["111727", "122924", "124030", "125706", "133323"]
D, S = DISCARDED, SURVIVES_CONTRAST
#: docs_output/resultados/05_contraste_giro_2409/tablas.md, with the provisional thresholds of 3.6.
EXPECTED_2409 = {
    #                            11:17 12:29 12:40 12:57 13:33
    "obstacle_traversed":       (S,    S,    S,    S,    S),
    "bottle_push":              (S,    S,    S,    S,    S),
    "robot_push":               (S,    S,    S,    S,    S),
    "slippery_floor":           (S,    D,    D,    D,    D),
    "wheel_failure":            (S,    D,    S,    S,    S),
    "commanded_speed_change":   (D,    D,    D,    D,    D),
    "uncommanded_motion":       (S,    D,    S,    S,    S),
    "bottle_removed_by_person": (D,    D,    D,    D,    D),
    "perception_failure":       (S,    S,    S,    S,    S),
    "suspension_failure":       (S,    S,    S,    S,    S),
}


def episodes_2409():
    from episode_series import read_series
    from src.episode_builder import build_episode

    episodes = {}
    for stamp in FALLS_2409:
        recording = RECORDINGS / f"mission_Follow_Person_24092026_{stamp}.txt"
        assert recording.exists(), f"missing {recording}"
        episodes[stamp] = build_episode(read_series(recording))
    return episodes


def check_contract_example_1229(episodes):
    contrast = by_mechanism(contrast_episode(episodes["122924"]))
    # Contract 2, table 3.6, column "12:29".
    assert check_of(contrast["obstacle_traversed"], "moving_on_segment")["verdict"] == "holds"
    assert check_of(contrast["obstacle_traversed"], "obstacle_jolt")["verdict"] == "trace_seen"
    traction = check_of(contrast["slippery_floor"], "traction_vs_friction")["conditions"][0]
    assert traction["value"] > traction["limit"] and abs(traction["limit"] - 3.237) < 1e-3, traction
    for mechanism_id, rule_id in (("slippery_floor", "traction_vs_friction"), ("wheel_failure", "turn_vs_command"),
                                  ("commanded_speed_change", "setpoint_step"),
                                  ("uncommanded_motion", "speed_ratio_and_jolt"),
                                  ("bottle_removed_by_person", "person_within_reach")):
        assert contrast[mechanism_id]["outcome"] == DISCARDED, mechanism_id
        assert contrast[mechanism_id]["decided_by"] == rule_id, mechanism_id
    assert contrast["robot_push"]["outcome"] == SURVIVES_CONTRAST     # lateral 0.37 is 2.8 x 0.13
    assert contrast["perception_failure"]["status_after_contrast"] == "checked_not_simulable"
    assert check_of(contrast["perception_failure"], "bottle_reacquired")["verdict"] == "no_trace"


def check_five_falls_2409(episodes):
    for column, stamp in enumerate(FALLS_2409):
        contrast = by_mechanism(contrast_episode(episodes[stamp]))
        # Criterion of A3, fixed before: the bump is never ruled out.
        assert contrast["obstacle_traversed"]["outcome"] == SURVIVES_CONTRAST, (stamp, contrast["obstacle_traversed"])
        assert check_of(contrast["obstacle_traversed"], "obstacle_jolt")["verdict"] == "trace_seen", stamp
        got = {m: r["outcome"] for m, r in contrast.items()}
        expected = {m: outcomes[column] for m, outcomes in EXPECTED_2409.items()}
        assert got == expected, (stamp, {m: (got[m], expected[m]) for m in got if got[m] != expected[m]})


def main():
    for check in (check_mechanisms_from_tbox, check_tbox_mismatch_is_refused, check_thresholds_on_synthetic_episode,
                  check_missing_values_never_discard, check_restoration_only_informs, check_reaction_is_never_read,
                  check_turn_relative_to_the_episode, check_rules_only_speak_for_their_interval):
        check()
        print(f"OK {check.__name__}")
    episodes = episodes_2409()
    for check in (check_contract_example_1229, check_five_falls_2409):
        check(episodes)
        print(f"OK {check.__name__}")
    print("Episode contrast: all checks passed")


if __name__ == "__main__":
    main()
