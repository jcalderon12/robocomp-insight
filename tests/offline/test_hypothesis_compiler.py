"""Offline test: semantic batch -> hypothesis compiler -> CauseWrapper validation.

Run from the repo root:  python3 tests/offline/test_hypothesis_compiler.py
(imports the causes simulator module, so pybullet must be installed)
"""
import json
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))

from src.hypothesis_service import validate_hypothesis_batch
from src.intervention_catalog import InterventionCatalog

from test_blueprint_grounding import hypothesis, make_batch


def main():
    catalog = InterventionCatalog.from_file(REPO / "etc" / "intervention_catalog.json")
    batch = make_batch([
            hypothesis("EXT_001", "external", 1, {
                "intervention": "spawn_static_object",
                "parameters": {"asset": "bump_100x5cm",
                               "position_range": {"x": [-3.4, -2.8], "y": [-0.5, -0.1], "z": [0.0, 0.01]}},
                "activation_window": {"start_fraction": 0, "end_fraction": 1}}),
            hypothesis("INT_001", "internal", 1, {
                "intervention": "disable_wheel", "parameters": {"wheel_id": "FL"},
                "activation_window": {"start_fraction": 0.2, "end_fraction": 0.8}}),
            hypothesis("INT_002", "internal", 2,
                       {"intervention": None, "parameters": {}, "activation_window": None},
                       {"untestable_reason": "untestable"}),
    ], case_id="compiler_test")
    validated = validate_hypothesis_batch(batch=batch, internal_count=2, external_count=1, catalog=catalog)

    # Simulator side: compile and validate against the dynamic cause union
    inner_root = REPO / "agents" / "inner_simulator"
    os.chdir(inner_root)
    sys.path.insert(0, str(inner_root))
    sys.path.insert(0, str(inner_root / "src"))
    from hypothesis_compiler import NOMINAL_HYPOTHESIS_ID, compile_batch
    from causes_simulator import CauseWrapper

    compiled = compile_batch(validated)
    assert compiled["entries"][0]["hypothesis_id"] == NOMINAL_HYPOTHESIS_ID
    assert len(compiled["entries"]) == 3, compiled  # nominal + 2 testables
    assert len(compiled["skipped"]) == 1, compiled["skipped"]

    for entry in compiled["entries"]:
        CauseWrapper.model_validate_json(json.dumps({"cause": entry["cause"]}))

    # The wheel cause must carry the activation window
    wheel = next(e["cause"] for e in compiled["entries"] if e["cause"]["name"] == "wheel")
    assert wheel["wheel_min"] == 0.2 and wheel["wheel_max"] == 0.8, wheel
    # The bump cause must resolve the asset to an absolute existing URDF
    bump = next(e["cause"] for e in compiled["entries"] if e["cause"]["name"] == "bump")
    assert Path(bump["bump_file"]).exists(), bump["bump_file"]

    print("test_hypothesis_compiler OK")


if __name__ == "__main__":
    main()
