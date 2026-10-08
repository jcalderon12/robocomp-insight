"""Offline test: from the hypotheses in symbols to the published batch (agents/semantic/src/hypothesis_pipeline.py).

Checks:
  * the qualitative parameters (kinds only, no magnitudes) and their values, the title templates
    and the catalog interventions come from the INSIGHT TBox as contract 2 lists them, and a TBox
    whose values differ from the ones the grounding translates into numbers is refused;
  * a layer-3a list that breaks the schema (unknown mechanism, parameter outside its list or
    missing, a magnitude, a number anywhere, `new` without description) is refused as a whole,
    naming every problem; a wrong anchor or a false precondition only makes that hypothesis
    incoherent;
  * on a hand-made episode with the 12:29 figures, the grounding gives the numbers of contract 2,
    section 3.5: a bump is every bump of the catalog, one cause each, and the box of the wide high
    one on Segment_final holds the true bump's centre; the cable only when the segment crosses x;
    the window of Interval_fall; the whole force range of a push, by direction relative to the
    robot heading; a wheel of unknown side is two compiled causes; and the budget walks the
    proposed order, takes each hypothesis whole, and stops at the first one that does not fit;
  * the six ideas of the real 12:29 batch, in layer 3a, give the states and the window of contract
    3, section 4.4 (criterion of subpaso 3.1);
  * with the production catalog, which has the dome of unknown size (`spawn_scaled_dome`,
    contracts 1.5), a bump is one hypothesis that costs 4: the area it must reach and the whole size
    range, which the simulator's cause draws per repetition; in the five falls a 1 m dome can reach
    the true bump's centre. With a catalog of fixed bumps only, the bump is one cause per asset, as
    before;
  * in the five falls of 2026-09-24, the bump of the anchored enumeration includes the true one
    (bump_100x10cm), whose box on Segment_final holds its centre, (0; -0.1) (criterion of subpaso
    3.1, kept when the magnitudes moved to the memory);
  * every blueprint passes the validation of hypothesis_service (the one test_blueprint_grounding
    exercises), and the simulator's compiler takes the batch unchanged: the nominal run plus one
    cause per hypothesis to simulate (and the causes simulator accepts them, if pybullet is here).

The recordings are not in git (agents/mission_controller/recorded_missions/), nor is experiments/:
both travel in the hand-over package.

Run from the repo root:  python3 tests/offline/test_hypothesis_pipeline.py
"""
import copy
import json
import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TBOX = REPO / "agents" / "semantic" / "data" / "insight_tbox.ttl"
TRUE_BUMP_CENTRE = (0.0, -0.1)          # experiments/ground_truth.py, session of 2026-09-24
sys.path.insert(0, str(REPO / "agents" / "semantic"))
sys.path.insert(0, str(REPO / "agents" / "inner_simulator" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "experiments"))

from hypothesis_compiler import NOMINAL_HYPOTHESIS_ID, compile_batch  # noqa: E402
from src.hypothesis_pipeline import (  # noqa: E402
    MECHANISM_ORDER, HypothesisSchemaError, anchored_enumeration, load_vocabulary, publish_batch,
)
from src.hypothesis_service import _ground_blueprint  # noqa: E402
from src.intervention_catalog import InterventionCatalog  # noqa: E402
from test_episode_contrast import FALLS_2409, episodes_2409, with_evidence  # noqa: E402
from test_episode_contrast import synthetic_episode as contrast_fixture  # noqa: E402

CATALOG = InterventionCatalog.from_file(REPO / "etc" / "intervention_catalog.json")
#: The catalog before contracts 1.5: fixed bump assets, no dome of unknown size.
_fixed = json.loads((REPO / "etc" / "intervention_catalog.json").read_text(encoding="utf-8"))
del _fixed["interventions"]["spawn_scaled_dome"]
FIXED_CATALOG = InterventionCatalog(_fixed)


def synthetic_episode(heading_deg=None):
    """The 12:29 figures of contract 1 (section 2.4), with the geometry the grounding reads."""
    episode = contrast_fixture()
    episode["time"].update(simulation_horizon_s=16.685, episode_length_s=65.0)
    geometry = {"Segment_final": (1, [-1.51, -0.51], [-0.41, -0.39], 1.2),
                "Segment_prev_1": (2, [-2.51, -1.51], [-0.42, -0.41], 0.4)}
    for segment in episode["segments"]:
        order, x, y, heading = geometry[segment["id"]]
        segment.update(order_back_from_fall=order, bbox={"x": x, "y": y}, length_m=1.0,
                       heading_deg=heading if heading_deg is None else heading_deg)
    return episode


def idea(mechanism, segment=None, interval=None, **parameters):
    return {"mechanism": mechanism, "new_mechanism_description": None, "segment": segment, "interval": interval,
            "qualitative_parameters": parameters, "expected_trace": [], "rationale": "", "title": ""}


def publish(episode, *ideas, budget=6, catalog=None):
    return publish_batch(episode, {"hypotheses": list(ideas)}, arm="llm_full", budget=budget, tbox_path=TBOX,
                         catalog=catalog)


def single(episode, *ideas, budget=6):
    hypotheses = publish(episode, *ideas, budget=budget)["hypotheses"]
    assert len(hypotheses) == 1, hypotheses
    return hypotheses[0]


def contains(position_range, point):
    return all(low <= value <= high for (low, high), value in zip((position_range["x"], position_range["y"]), point))


# --------------------------------------------------------------------------- #
# The TBox
# --------------------------------------------------------------------------- #
def check_vocabulary_from_tbox():
    vocabulary = load_vocabulary(TBOX)
    assert list(vocabulary) == list(MECHANISM_ORDER)
    assert {m: t.interventions for m, t in vocabulary.items()} == {
        "obstacle_traversed": ("spawn_scaled_dome", "spawn_static_object"),
        "bottle_push": ("apply_external_force",), "robot_push": ("apply_external_force",),
        "slippery_floor": ("set_friction",), "wheel_failure": ("disable_wheel",),
        "commanded_speed_change": (), "uncommanded_motion": (), "bottle_removed_by_person": (),
        "perception_failure": (), "suspension_failure": ()}
    # Kinds only: no size, height, strength nor slipperiness (contract 2, version 1.4).
    assert {m: t.parameters for m, t in vocabulary.items() if t.parameters} == {
        "obstacle_traversed": {"shape": ("bump", "cable")},
        "bottle_push": {"direction": ("any", "forward", "backward", "left", "right")},
        "robot_push": {"direction": ("any", "forward", "backward", "left", "right")},
        "wheel_failure": {"side": ("left", "right", "unknown")},
        "uncommanded_motion": {"kind": ("brake", "jerk")}}
    assert vocabulary["obstacle_traversed"].title_template == "Undetected {shape} on {segment}"

    text = TBOX.read_text(encoding="utf-8")
    other_values = text.replace('insight:allowedValue "bump",\n        "cable" ;',
                                'insight:allowedValue "bump",\n        "rope" ;')
    assert other_values != text, "the TBox text changed: update this check"
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "other.ttl"
        path.write_text(other_values, encoding="utf-8")
        try:
            load_vocabulary(path)
        except ValueError as error:
            assert "the grounding translates" in str(error), error
        else:
            raise AssertionError("a TBox with other parameter values was accepted")


# --------------------------------------------------------------------------- #
# Coherence: schema (the whole list is refused) and anchors (the hypothesis is incoherent)
# --------------------------------------------------------------------------- #
def schema_problems(*ideas):
    try:
        publish(synthetic_episode(), *ideas)
    except HypothesisSchemaError as error:
        return error.problems
    raise AssertionError(f"accepted: {ideas}")


def check_schema_is_refused():
    assert any("unknown mechanism 'gremlins'" in p for p in schema_problems(idea("gremlins")))
    assert any("must be one of" in p for p in schema_problems(idea("obstacle_traversed", "Segment_final", shape="rug")))
    assert any("needs the parameter 'direction'" in p for p in schema_problems(
        idea("bottle_push", interval="Interval_fall")))
    assert any("admits no parameter 'colour'" in p for p in schema_problems(idea("slippery_floor", colour="blue")))
    # The magnitudes are not the LLM's to choose.
    for magnitude in (idea("obstacle_traversed", "Segment_final", shape="bump", size="wide"),
                      idea("bottle_push", interval="Interval_fall", direction="any", strength="light"),
                      idea("slippery_floor", slipperiness="very_slippery")):
        assert any("admits no parameter" in p for p in schema_problems(magnitude)), magnitude
    assert any("needs new_mechanism_description" in p for p in schema_problems(idea("new")))
    numeric = idea("bottle_push", interval="Interval_fall", direction="any")
    numeric["confidence"] = 0.7
    assert any("confidence: numbers are not allowed" in p for p in schema_problems(numeric))
    window = idea("wheel_failure", interval="Interval_fall", side="left")
    window["activation_window"] = {"start_fraction": 0.3, "end_fraction": 0.7}
    assert len(schema_problems(window)) == 2
    # Every problem is reported, not only the first one.
    assert len(schema_problems(idea("gremlins"), idea("new"), numeric)) == 3
    try:
        publish_batch(synthetic_episode(), {"hypotheses": []}, arm="llm_full", tbox_path=TBOX)
    except HypothesisSchemaError:
        pass
    else:
        raise AssertionError("an empty list was accepted")


def check_wrong_anchors_are_incoherent():
    episode = synthetic_episode()
    for wrong, why in ((idea("obstacle_traversed", "Segment_prev_9", shape="cable"), "does not exist in the episode"),
                       (idea("obstacle_traversed", shape="cable"), "needs a segment anchor"),
                       (idea("slippery_floor", interval="Interval_fall"), "admits no interval anchor"),
                       (idea("bottle_push", interval="Interval_reaction", direction="any"), "after the observation")):
        result = single(episode, wrong)
        assert result["status"] == "incoherent" and why in result["untestable_reason"], (why, result)
        assert result["checks"]["coherence"]["passed"] is False and not result["testable"]
        assert result["simulation_blueprint"]["intervention"] is None
    # A precondition that fails: the robot did not move on the segment.
    slow = copy.deepcopy(episode)
    slow["segments"][0]["mean_speed_mps"] = 0.04
    result = single(slow, idea("obstacle_traversed", "Segment_final", shape="cable"))
    assert result["status"] == "incoherent" and "moving_on_segment" in result["untestable_reason"], result
    # A new mechanism is kept, not simulated; with an anchor that does not exist it is incoherent.
    new = idea("new", interval="Interval_fall")
    new["new_mechanism_description"] = "The tray tilts by itself."
    result = single(episode, new)
    assert result["status"] == "not_simulable" and result["title"] == "Unlisted mechanism: The tray tilts by itself."
    new["interval"] = "Interval_nowhere"
    assert single(episode, new)["status"] == "incoherent"


# --------------------------------------------------------------------------- #
# Grounding in numbers (contract 2, section 3.5)
# --------------------------------------------------------------------------- #
def bump_entries(episode, budget=6):
    return publish(episode, idea("obstacle_traversed", "Segment_final", shape="bump"), budget=budget,
                   catalog=FIXED_CATALOG)["hypotheses"]


def check_obstacle_box():
    episode = synthetic_episode()
    # With a catalog of fixed bumps only (before contracts 1.5), a bump is every bump of the catalog,
    # one compiled cause each, in the catalog's order.
    bumps = bump_entries(episode)
    assert [h["hypothesis_id"] for h in bumps] == [
        "H01_bump_25x2cm", "H01_bump_25x5cm", "H01_bump_100x5cm", "H01_bump_100x10cm"], bumps
    assert {h["rank"] for h in bumps} == {1} and all(h["status"] == "to_simulate" for h in bumps)
    assert all(h["qualitative_parameters"] == {"shape": "bump"} for h in bumps)
    wide_high = bumps[3]
    blueprint = wide_high["simulation_blueprint"]
    assert blueprint["parameters"]["asset"] == "bump_100x10cm" and blueprint["activation_window"] is None
    # Contract 2, section 3.5: bbox(Segment_final) + 0.85 m.
    assert blueprint["parameters"]["position_range"] == {"x": [-2.36, 0.34], "y": [-1.26, 0.46], "z": [0.0, 0.0]}
    assert contains(blueprint["parameters"]["position_range"], TRUE_BUMP_CENTRE)
    assert wide_high["title"] == "Undetected bump on the last 1 m of the path before the fall (bump_100x10cm)"
    assert "4 bumps of the catalog" in wide_high["grounding_trace"]["split"], wide_high["grounding_trace"]
    small = bumps[0]
    assert small["simulation_blueprint"]["parameters"]["asset"] == "bump_25x2cm"
    assert small["simulation_blueprint"]["parameters"]["position_range"]["x"] == [-1.985, -0.035]   # 0.35 + 0.125
    # The four count together: with k = 3 none is simulated.
    assert [h["status"] for h in bump_entries(episode, budget=3)] == ["over_budget"] * 4
    # Clipped to the catalog bounds (x up to 3 m).
    far = copy.deepcopy(episode)
    far["segments"][0]["bbox"] = {"x": [2.5, 2.9], "y": [0.0, 0.1]}
    clipped = bump_entries(far)[2]
    assert clipped["simulation_blueprint"]["parameters"]["position_range"]["x"] == [1.65, 3.0], clipped
    # The cable lies along x: not on a segment parallel to it, yes on one that crosses it.
    cable = idea("obstacle_traversed", "Segment_final", shape="cable")
    parallel = single(episode, cable)
    assert parallel["status"] == "not_simulable" and "cannot orient the cable" in parallel["untestable_reason"]
    crossing = single(synthetic_episode(heading_deg=60.0), cable)
    assert crossing["status"] == "to_simulate", crossing
    assert crossing["simulation_blueprint"]["parameters"]["position_range"] == {
        "x": [-1.51, -0.51], "y": [-0.86, 0.06], "z": [0.0, 0.0]}                                # y + 0.35 + 0.1


def check_window_and_forces():
    episode = synthetic_episode(heading_deg=0.0)
    push = single(episode, idea("bottle_push", interval="Interval_fall", direction="any"))
    assert push["simulation_blueprint"]["activation_window"] == {"start_fraction": 0.7, "end_fraction": 0.82}
    # The whole range of a push on the bottle, from light to strong: 3-30 N.
    assert push["simulation_blueprint"]["parameters"] == {
        "target": "bottle", "force_range": {"x": [-30.0, 30.0], "y": [-30.0, 30.0], "z": [0.0, 0.0]}}
    assert push["title"] == "Push on the bottle during the last 2 s before the fall"

    def force(ep, mechanism, direction):
        hypothesis = single(ep, idea(mechanism, interval="Interval_fall", direction=direction))
        return hypothesis["simulation_blueprint"]["parameters"]["force_range"]

    # Robot heading +x: 3-30 N, widened by 0.2 x 30 N.
    assert force(episode, "bottle_push", "forward") == {"x": [-3.0, 36.0], "y": [-6.0, 6.0], "z": [0.0, 0.0]}
    assert force(episode, "bottle_push", "backward")["x"] == [-36.0, 3.0]
    assert force(episode, "bottle_push", "left") == {"x": [-6.0, 6.0], "y": [-3.0, 36.0], "z": [0.0, 0.0]}
    assert force(episode, "bottle_push", "right")["y"] == [-36.0, 3.0]
    # Robot heading +y: forward is +y in the room.
    assert force(synthetic_episode(heading_deg=90.0), "bottle_push", "forward")["y"] == [-3.0, 36.0]
    # On the robot, 150-500 N, clipped to the catalog (500 N).
    assert force(episode, "robot_push", "forward")["x"] == [50.0, 500.0]
    # The heading of an interval is the overlap-weighted mean of the segments it covers.
    halves = synthetic_episode(heading_deg=0.0)
    halves["segments"][1]["heading_deg"] = 90.0
    halves["intervals"].append({"id": "Interval_both", "start_s": 8.19, "end_s": 10.19})   # 1 s on each segment
    both = single(halves, idea("bottle_push", interval="Interval_both", direction="forward"))
    assert "h = 45.0 deg" in both["grounding_trace"]["force_range"], both["grounding_trace"]

    floor = single(episode, idea("slippery_floor"))
    assert floor["status"] == "discarded"                                    # 3.41 m/s2 sustained at the start
    assert floor["simulation_blueprint"]["parameters"] == {"target": "floor", "lateral_friction_range": [0.01, 0.3]}
    grip = with_evidence(episode, "max_sustained_horizontal_accel", value=0.5)
    assert single(grip, idea("slippery_floor"))["status"] == "to_simulate"


def check_unknown_wheel_and_budget():
    turning = with_evidence(synthetic_episode(), "yaw_change", value=6.0)    # not what was ordered: it survives
    wheel = idea("wheel_failure", interval="Interval_fall", side="unknown")
    both = publish(turning, wheel)["hypotheses"]
    assert [h["hypothesis_id"] for h in both] == ["H01_left", "H01_right"], both
    assert [h["simulation_blueprint"]["parameters"]["wheel_id"] for h in both] == ["left", "right"]
    assert [h["status"] for h in both] == ["to_simulate", "to_simulate"]
    assert both[0]["title"] == "Left drive wheel blocked during the last 2 s before the fall"
    assert {h["rank"] for h in both} == {1}
    # Both sides or none.
    assert [h["status"] for h in publish(turning, wheel, budget=1)["hypotheses"]] == ["over_budget"] * 2

    push = idea("bottle_push", interval="Interval_fall", direction="any")
    robot = idea("robot_push", interval="Interval_fall", direction="any")
    suspension = idea("suspension_failure")
    batch = publish(turning, push, suspension, wheel, robot, budget=2)
    assert [(h["hypothesis_id"], h["status"]) for h in batch["hypotheses"]] == [
        ("H01", "to_simulate"), ("H02", "not_simulable"), ("H03_left", "over_budget"), ("H03_right", "over_budget"),
        ("H04", "over_budget")]                       # H04 would fit, but it does not overtake H03
    assert batch["budget"] == {"k": 2, "used": 1, "unit": "compiled cause; the nominal run is free"}
    assert all(h["status"] == "over_budget" for h in publish(turning, push, robot, budget=0)["hypotheses"])
    over = publish(turning, push, robot, budget=1)["hypotheses"][1]
    assert over["simulation_blueprint"]["intervention"] == "apply_external_force" and not over["testable"]
    assert over["untestable_reason"].startswith("Over budget"), over
    # Without a budget (production since 07/10) everything that survives and can be simulated is.
    unlimited = publish(turning, push, suspension, wheel, robot, budget=None)
    assert [(h["hypothesis_id"], h["status"]) for h in unlimited["hypotheses"]] == [
        ("H01", "to_simulate"), ("H02", "not_simulable"), ("H03_left", "to_simulate"), ("H03_right", "to_simulate"),
        ("H04", "to_simulate")]
    assert unlimited["budget"]["k"] is None and unlimited["budget"]["used"] == 4
    assert publish_batch(turning, {"hypotheses": [push]}, arm="llm_full", tbox_path=TBOX)["budget"]["k"] is None


# --------------------------------------------------------------------------- #
# The batch: blueprint v2, the validation of hypothesis_service and the simulator's compiler
# --------------------------------------------------------------------------- #
def check_batch_for_the_simulator(batch):
    assert batch["schema_version"] == "2.0" and batch["status"] == "success"
    to_simulate = [h for h in batch["hypotheses"] if h["status"] == "to_simulate"]
    assert batch["budget"]["used"] == sum(h["cost_in_simulations"] for h in to_simulate)
    assert batch["budget"]["k"] is None or batch["budget"]["used"] <= batch["budget"]["k"]
    for hypothesis in batch["hypotheses"]:
        assert hypothesis["testable"] == (hypothesis["status"] == "to_simulate"), hypothesis
        assert hypothesis["testable"] or hypothesis["untestable_reason"], hypothesis
        blueprint = hypothesis["simulation_blueprint"]
        normalized, grounded, reason = _ground_blueprint(blueprint, CATALOG)
        assert grounded == (blueprint["intervention"] is not None), (hypothesis["hypothesis_id"], reason)
        assert not grounded or normalized == blueprint, (normalized, blueprint)
        if hypothesis["testable"]:
            assert grounded, hypothesis
    compiled = compile_batch(batch)
    assert compiled["entries"][0]["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID
    assert [e["hypothesis_id"] for e in compiled["entries"][1:]] == [h["hypothesis_id"] for h in to_simulate]
    assert len(compiled["skipped"]) == len(batch["hypotheses"]) - len(to_simulate), compiled["skipped"]
    return compiled


def validate_causes(compiled_batches):
    """The causes simulator accepts the compiled causes (needs pybullet; skipped without it)."""
    inner_root = REPO / "agents" / "inner_simulator"
    cwd = os.getcwd()
    try:
        os.chdir(inner_root)
        sys.path.insert(0, str(inner_root))
        from causes_simulator import CauseWrapper
    except ImportError as error:
        print(f"  (causes simulator not checked: {error})")
        return
    finally:
        os.chdir(cwd)
    for compiled in compiled_batches:
        for entry in compiled["entries"]:
            CauseWrapper.model_validate_json(json.dumps({"cause": entry["cause"]}))


# --------------------------------------------------------------------------- #
# The recordings of 2026-09-24
# --------------------------------------------------------------------------- #
#: Contract 3, section 4.4: the six ideas of gemma4:31b on 24/09 (batch
#: semantic_unexplained_20260924T102944Z), in layer 3a.
SIX_IDEAS_1229 = [
    ("Collision with Low-Profile Obstacle", idea("obstacle_traversed", "Segment_final", shape="cable")),
    ("External Physical Contact", idea("bottle_push", interval="Interval_fall", direction="any")),
    ("Slippery Floor Surface", idea("slippery_floor")),
    ("Mecanum Wheel Motor Failure", idea("wheel_failure", interval="Interval_fall", side="right")),
    ("Suspension System Instability", idea("suspension_failure")),
    ("Power Rail Voltage Spike", idea("uncommanded_motion", interval="Interval_fall", kind="jerk")),
]
#: Contract 3, section 4.4: (status, the check that decides it).
EXPECTED_1229 = {"H01": ("not_simulable", "cannot orient the cable"), "H02": ("to_simulate", ""),
                 "H03": ("discarded", "traction_vs_friction"), "H04": ("discarded", "turn_vs_command"),
                 "H05": ("not_simulable", "no suspension"), "H06": ("discarded", "speed_ratio_and_jolt")}


def six_ideas_1229():
    ideas = []
    for title, hypothesis in SIX_IDEAS_1229:
        ideas.append({**hypothesis, "title": title})
    return {"hypotheses": ideas}


def check_six_ideas_1229(episodes):
    batch = publish_batch(episodes["122924"], six_ideas_1229(), arm="production", model="gemma4:31b-cloud",
                          tbox_path=TBOX)
    by_id = {h["hypothesis_id"]: h for h in batch["hypotheses"]}
    assert set(by_id) == set(EXPECTED_1229), sorted(by_id)
    for hypothesis_id, (status, why) in EXPECTED_1229.items():
        hypothesis = by_id[hypothesis_id]
        assert hypothesis["status"] == status, (hypothesis_id, hypothesis["status"], hypothesis["untestable_reason"])
        assert why in hypothesis["untestable_reason"], (hypothesis_id, hypothesis["untestable_reason"])
    h02 = by_id["H02"]
    assert h02["simulation_blueprint"]["activation_window"] == {"start_fraction": 0.7, "end_fraction": 0.82}
    assert h02["simulation_blueprint"]["parameters"]["force_range"] == {
        "x": [-30.0, 30.0], "y": [-30.0, 30.0], "z": [0.0, 0.0]}
    assert h02["title"] == "Push on the bottle during the last 2 s before the fall"
    assert h02["llm_title"] == "External Physical Contact"
    assert batch["budget"]["used"] == 1                     # H02, plus the nominal run
    assert by_id["H06"]["simulation_blueprint"]["intervention"] is None     # no realization in the catalog
    return check_batch_for_the_simulator(batch)


def check_enumeration_2409(episodes):
    compiled = []
    for stamp in FALLS_2409:
        episode = episodes[stamp]
        # The production catalog: the bump is one dome of unknown size, which can reach the true bump.
        batch = publish_batch(episode, anchored_enumeration(TBOX), arm="enum_anchored_contrast", tbox_path=TBOX)
        assert len(batch["hypotheses"]) == 13, len(batch["hypotheses"])
        [dome] = [h for h in batch["hypotheses"] if h["qualitative_parameters"] == {"shape": "bump"}]
        assert dome["status"] == "to_simulate" and dome["cost_in_simulations"] == 4, dome
        area = dome["simulation_blueprint"]["parameters"]["area"]
        assert all(area[axis][0] - 0.5 <= value <= area[axis][1] + 0.5 for axis, value in zip("xy", TRUE_BUMP_CENTRE))
        compiled.append(check_batch_for_the_simulator(batch))
        # A catalog of fixed bumps only (before contracts 1.5).
        batch = publish_batch(episode, anchored_enumeration(TBOX), arm="enum_anchored_contrast", tbox_path=TBOX,
                              catalog=FIXED_CATALOG)
        assert len(batch["hypotheses"]) == 16, len(batch["hypotheses"])
        true_bump = [h for h in batch["hypotheses"] if h["mechanism"] == "obstacle_traversed"
                     and h["simulation_blueprint"]["parameters"].get("asset") == "bump_100x10cm"]
        assert len(true_bump) == 1 and true_bump[0]["anchors"]["segment"] == "Segment_final"
        box = true_bump[0]["simulation_blueprint"]["parameters"]["position_range"]
        # Criterion of subpaso 3.1, fixed before.
        assert contains(box, TRUE_BUMP_CENTRE), (stamp, box)
        assert true_bump[0]["status"] == "to_simulate", (stamp, true_bump[0]["status"])
        assert all(h["status"] != "discarded" for h in batch["hypotheses"] if h["mechanism"] == "obstacle_traversed")
        compiled.append(check_batch_for_the_simulator(batch))
    return compiled


def check_scaled_dome_when_the_catalog_has_it(episodes):
    from causes.implementations.cause_bump_scaled import CauseBumpScaled

    catalog = CATALOG

    def bump(episode, budget=6):
        return publish_batch(episode, {"hypotheses": [idea("obstacle_traversed", "Segment_final", shape="bump")]},
                             arm="llm_full", budget=budget, tbox_path=TBOX, catalog=catalog)

    batch = bump(synthetic_episode())
    [hypothesis] = batch["hypotheses"]
    assert hypothesis["hypothesis_id"] == "H01" and hypothesis["status"] == "to_simulate", hypothesis
    assert hypothesis["cost_in_simulations"] == 4 and batch["budget"]["used"] == 4
    assert hypothesis["title"] == "Undetected bump on the last 1 m of the path before the fall"
    blueprint = hypothesis["simulation_blueprint"]
    # The area is the segment plus the robot footprint; the size is not chosen.
    assert blueprint == {"intervention": "spawn_scaled_dome",
                         "parameters": {"area": {"x": [-1.86, -0.16], "y": [-0.76, -0.04], "z": [0.0, 0.0]},
                                        "diameter_range": [0.1, 1.0], "height_range": [0.01, 0.1]},
                         "activation_window": None}, blueprint
    normalized, grounded, reason = _ground_blueprint(blueprint, catalog)
    assert grounded and normalized == blueprint, reason
    assert [h["status"] for h in bump(synthetic_episode(), budget=3)["hypotheses"]] == ["over_budget"]

    # The simulator's cause draws 36 domes within the ranges, each reaching the area with its edge.
    area = blueprint["parameters"]["area"]
    payload = compile_batch(batch)["entries"][1]["cause"]
    assert payload["name"] == "bump_scaled" and Path(payload["mesh_file"]).exists(), payload
    cause = CauseBumpScaled(**payload)
    assert cause.num_of_repetitions == 36 and len(cause.samples) == 36
    for sample in cause.samples:
        assert 0.1 <= sample["diameter_m"] <= 1.0 and 0.01 <= sample["height_m"] <= 0.1, sample
        half = sample["diameter_m"] / 2
        assert area["x"][0] - half <= sample["x_m"] <= area["x"][1] + half, sample
        assert area["y"][0] - half <= sample["y_m"] <= area["y"][1] + half, sample

    # In the five falls, a 1 m dome can reach the true bump's centre.
    for stamp in FALLS_2409:
        reach = bump(episodes[stamp])["hypotheses"][0]["simulation_blueprint"]["parameters"]["area"]
        assert all(reach[axis][0] - 0.5 <= value <= reach[axis][1] + 0.5
                   for axis, value in zip("xy", TRUE_BUMP_CENTRE)), (stamp, reach)


def main():
    for check in (check_vocabulary_from_tbox, check_schema_is_refused, check_wrong_anchors_are_incoherent,
                  check_obstacle_box, check_window_and_forces, check_unknown_wheel_and_budget):
        check()
        print(f"OK {check.__name__}")
    episodes = episodes_2409()
    compiled = [check_six_ideas_1229(episodes)]
    print("OK check_six_ideas_1229")
    compiled += check_enumeration_2409(episodes)
    print("OK check_enumeration_2409")
    check_scaled_dome_when_the_catalog_has_it(episodes)
    print("OK check_scaled_dome_when_the_catalog_has_it")
    validate_causes(compiled)
    print("OK validate_causes")
    print("Hypothesis pipeline: all checks passed")


if __name__ == "__main__":
    main()
