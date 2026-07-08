"""Offline test: blueprint v2 validation + catalog grounding (no robot required).

Run from the repo root:  python3 tests/offline/test_blueprint_grounding.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))

from src.hypothesis_service import validate_hypothesis_batch
from src.intervention_catalog import InterventionCatalog


def hypothesis(hid, family, rank, blueprint, extra=None):
    payload = {
        "hypothesis_id": hid, "family": family, "rank": rank,
        "title": f"T {hid}", "rationale": f"R {hid}", "confidence": 0.5,
        "grounding": ["x"], "expected_observations": ["y"],
        "simulation_blueprint": blueprint,
    }
    if extra:
        payload.update(extra)
    return payload


def make_batch(hypotheses, case_id="test"):
    return {
        "schema_version": "1.0", "status": "success", "case_id": case_id, "trace_id": "t",
        "generated_at": "2026-01-01T00:00:00+00:00", "generator": "g", "model": "m",
        "trigger": {}, "context_summary": {}, "errors": [],
        "hypotheses": hypotheses,
    }


def main():
    catalog = InterventionCatalog.from_file(REPO / "etc" / "intervention_catalog.json")

    batch = make_batch([
            hypothesis("EXT_001", "external", 1, {
                "intervention": "spawn_static_object",
                "parameters": {"asset": "bump_100x5cm",
                               "position_range": {"x": [-1, 0.5], "y": [-0.2, 0.2], "z": [0, 0.01]}},
                "activation_window": {"start_fraction": 0, "end_fraction": 1}}),
            hypothesis("EXT_002", "external", 2, {
                "intervention": "apply_external_force",
                "parameters": {"target": "bottle",
                               "force_range": {"x": [-20, 20], "y": [-20, 20], "z": [0, 0]}},
                "activation_window": {"start_fraction": 0.3, "end_fraction": 0.5}}),
            hypothesis("EXT_003", "external", 3, {
                "intervention": "set_friction",
                "parameters": {"target": "floor", "lateral_friction_range": [0.01, 0.3]},
                "activation_window": None}),
            hypothesis("INT_001", "internal", 1, {
                "intervention": "disable_wheel", "parameters": {"wheel_id": "BR"},
                "activation_window": {"start_fraction": 0.4, "end_fraction": 0.7}}),
            hypothesis("INT_002", "internal", 2,
                       {"intervention": None, "parameters": {}, "activation_window": None},
                       {"untestable_reason": "Sensor fault not physically simulable"}),
            hypothesis("INT_003", "internal", 3,
                       {"intervention": "apply_vibration", "parameters": {}, "activation_window": None}),
    ])

    validated = validate_hypothesis_batch(batch=batch, internal_count=3, external_count=3, catalog=catalog)
    testables = [h for h in validated["hypotheses"] if h["testable"]]
    untestables = [h for h in validated["hypotheses"] if not h["testable"]]
    assert len(testables) == 4 and len(untestables) == 2, validated
    assert all(h["simulation_blueprint"]["intervention"] for h in testables)
    assert all(h["untestable_reason"] for h in untestables)

    # Out-of-bounds parameter -> untestable, not batch rejection
    oob = catalog.ground_blueprint({
        "intervention": "set_friction",
        "parameters": {"target": "floor", "lateral_friction_range": [0.001, 5.0]},
        "activation_window": None,
    })
    assert not oob.testable and "bounds" in oob.reason, oob

    print("test_blueprint_grounding OK")


if __name__ == "__main__":
    main()
