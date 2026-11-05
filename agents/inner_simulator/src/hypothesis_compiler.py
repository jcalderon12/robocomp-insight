"""Compiles hypothesis batches (semantic agent, blueprint schema v2) into cause
payloads executable by the causes simulator.

Each testable hypothesis grounds in one intervention of the shared catalog
(etc/intervention_catalog.json); this module maps that intervention plus its
parameters onto the discriminated-union payload of the corresponding Cause
implementation under src/causes/implementations/. A nominal 'none' cause (the
null hypothesis) is always prepended so the verdict can compare against it.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Optional

# agents/inner_simulator/src/hypothesis_compiler.py -> repo root is parents[3]
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CATALOG_PATH = REPO_ROOT / "etc" / "intervention_catalog.json"

NOMINAL_HYPOTHESIS_ID = "__nominal__"

# The catalog and the causes simulator place bodies in meters; CauseBump takes the ranges of its
# grid in millimeters, as causes.json writes them.
M_TO_MM = 1000.0

# CauseBump sweeps a grid over the proposed range (one repetition per cell): 144 cells. Only a catalog of
# fixed bumps uses it; production's catalog has the dome of unknown size, with episode_scene.DEFAULT_REPETITIONS.
BUMP_GRID_DIMENSIONS = [12, 12]

# Spawned objects rest on the floor. The asset meshes start at z = 0, and the
# middle of the proposed z range (e.g. 25 mm for [0, 0.05]) left every bump
# floating; 1 mm matches how the Webots world places its bump.
FLOOR_OBJECT_Z_M = 0.001

# Drive wheel of the catalog -> drive wheel of the simulated base (CausesSimulator.wheel_names).
WHEEL_SIDES = {"left": "L", "right": "R"}


class HypothesisCompilationError(ValueError):
    """Raised when a testable hypothesis cannot be mapped to a cause payload."""


def _range_to_origin_span(axis_range: list[float]) -> tuple[float, float]:
    """[min, max] -> (origin, span) as expected by random_range_coordinates."""
    low, high = float(axis_range[0]), float(axis_range[1])
    return (low + high) / 2.0, high - low


def _window_fractions(blueprint: dict[str, Any]) -> tuple[float, float]:
    window = blueprint.get("activation_window")
    if not isinstance(window, dict):
        return 0.0, 1.0
    try:
        start = float(window.get("start_fraction", 0.0))
        end = float(window.get("end_fraction", 1.0))
    except (TypeError, ValueError):
        return 0.0, 1.0
    if not (0.0 <= start <= end <= 1.0):
        return 0.0, 1.0
    return start, end


def _resolve_asset_urdf(asset_name: str, catalog: dict[str, Any]) -> str:
    assets = catalog.get("assets", {})
    asset = assets.get(asset_name)
    if not isinstance(asset, dict) or not asset.get("urdf"):
        raise HypothesisCompilationError(
            f"Asset '{asset_name}' is not defined in the intervention catalog."
        )
    urdf_path = (REPO_ROOT / str(asset["urdf"])).resolve()
    if not urdf_path.exists():
        raise HypothesisCompilationError(
            f"Asset '{asset_name}' points to a missing URDF: {urdf_path}"
        )
    return str(urdf_path)


def _compile_spawn_static_object(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    position_range = params["position_range"]
    payload: dict[str, Any] = {
        "bump_file": _resolve_asset_urdf(str(params["asset"]), catalog),
        "grid_dimensions": list(BUMP_GRID_DIMENSIONS),
    }
    for axis in ("x", "y"):
        origin, span = _range_to_origin_span(position_range[axis])
        payload[f"bump_{axis}_origin"] = origin
        payload[f"bump_{axis}_range"] = span * M_TO_MM
    payload["bump_z_origin"] = FLOOR_OBJECT_Z_M
    payload["bump_z_range"] = 0.0
    return payload


def _compile_spawn_scaled_dome(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    """A bump of unknown size: the area it must reach and the size ranges; the cause draws the
    diameter, height and centre of each repetition (causes/implementations/cause_bump_scaled.py)."""
    spec = catalog["interventions"]["spawn_scaled_dome"]
    mesh = (REPO_ROOT / str(spec["mesh"])).resolve()
    if not mesh.exists():
        raise HypothesisCompilationError(f"Intervention 'spawn_scaled_dome' points to a missing mesh: {mesh}")
    bounds = spec["parameters"]["area"].get("bounds", {})
    return {
        "mesh_file": str(mesh),
        "mesh_dimensions_m": [float(v) for v in spec["mesh_dimensions_m"]],
        "area_x": [float(v) for v in params["area"]["x"]],
        "area_y": [float(v) for v in params["area"]["y"]],
        "diameter_range": [float(v) for v in params["diameter_range"]],
        "height_range": [float(v) for v in params["height_range"]],
        "bounds_x": [float(v) for v in bounds["x"]] if "x" in bounds else None,
        "bounds_y": [float(v) for v in bounds["y"]] if "y" in bounds else None,
        "num_of_repetitions": int(spec.get("repetitions", 36)),
        "seed": 0,
    }


def _compile_disable_wheel(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    start, end = _window_fractions(blueprint)
    return {
        "wheel_wheel": WHEEL_SIDES.get(str(params["wheel_id"]), str(params["wheel_id"])),
        "wheel_min": start,
        "wheel_max": end,
    }


def _compile_apply_external_force(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    start, end = _window_fractions(blueprint)
    force_range = params["force_range"]
    payload: dict[str, Any] = {
        "force_target": str(params["target"]),
        "force_window_start": start,
        "force_window_end": end,
    }
    for axis in ("x", "y", "z"):
        origin, span = _range_to_origin_span(force_range[axis])
        payload[f"force_{axis}_origin"] = origin
        payload[f"force_{axis}_range"] = span
    return payload


def _compile_set_friction(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    friction_range = params["lateral_friction_range"]
    return {
        "friction_target": str(params["target"]),
        "friction_min": float(friction_range[0]),
        "friction_max": float(friction_range[1]),
    }


INTERVENTION_COMPILERS = {
    "spawn_static_object": _compile_spawn_static_object,
    "spawn_scaled_dome": _compile_spawn_scaled_dome,
    "disable_wheel": _compile_disable_wheel,
    "apply_external_force": _compile_apply_external_force,
    "set_friction": _compile_set_friction,
}


def load_catalog(catalog_path: Optional[Path] = None) -> dict[str, Any]:
    path = Path(catalog_path or DEFAULT_CATALOG_PATH)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HypothesisCompilationError(f"Intervention catalog not readable at {path}: {exc}")


def compile_hypothesis(hypothesis: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    """Compile a single testable hypothesis into a cause payload."""
    blueprint = hypothesis.get("simulation_blueprint")
    if not isinstance(blueprint, dict):
        raise HypothesisCompilationError("Hypothesis has no simulation_blueprint object.")
    intervention = str(blueprint.get("intervention") or "").strip()
    compiler = INTERVENTION_COMPILERS.get(intervention)
    if compiler is None:
        raise HypothesisCompilationError(
            f"No compiler for intervention '{intervention}'. "
            f"Known interventions: {sorted(INTERVENTION_COMPILERS)}"
        )
    params = blueprint.get("parameters")
    if not isinstance(params, dict):
        raise HypothesisCompilationError("simulation_blueprint.parameters must be an object.")
    try:
        payload = compiler(params, blueprint, catalog)
        payload["name"] = str(catalog["interventions"][intervention]["cause_name"])
        return payload
    except HypothesisCompilationError:
        raise
    except (KeyError, TypeError, IndexError, ValueError) as exc:
        raise HypothesisCompilationError(
            f"Malformed parameters for intervention '{intervention}': {exc}"
        )


def compile_batch(batch: dict[str, Any], catalog_path: Optional[Path] = None) -> dict[str, Any]:
    """Compile a hypothesis batch into an ordered list of cause entries.

    Returns {case_id, entries: [{hypothesis_id, title, cause}, ...], skipped:
    [{hypothesis_id, reason}, ...]} with the nominal (null hypothesis) entry
    always first. Untestable hypotheses and compilation failures are reported
    in 'skipped', never silently dropped.
    """
    catalog = load_catalog(catalog_path)
    # The nominal replay is deterministic: one repetition is its exact likelihood.
    entries: list[dict[str, Any]] = [
        {
            "hypothesis_id": NOMINAL_HYPOTHESIS_ID,
            "title": "Nominal run (null hypothesis)",
            "mechanism": "nominal",
            "cause": {"name": "none", "num_of_repetitions": 1},
        }
    ]
    skipped: list[dict[str, Any]] = []

    for hypothesis in batch.get("hypotheses", []):
        hypothesis_id = str(hypothesis.get("hypothesis_id", "")).strip() or "<missing-id>"
        if not hypothesis.get("testable", False):
            skipped.append(
                {
                    "hypothesis_id": hypothesis_id,
                    "reason": str(hypothesis.get("untestable_reason", "")) or "Marked untestable.",
                }
            )
            continue
        try:
            cause_payload = compile_hypothesis(hypothesis, catalog)
        except HypothesisCompilationError as exc:
            skipped.append({"hypothesis_id": hypothesis_id, "reason": str(exc)})
            continue
        entries.append(
            {
                "hypothesis_id": hypothesis_id,
                "title": str(hypothesis.get("title", "")),
                # The verdict gives every mechanism the same prior (docs_output/diseno/veredicto.md).
                "mechanism": str(hypothesis.get("mechanism") or cause_payload["name"]),
                "cause": cause_payload,
            }
        )

    return {
        "case_id": str(batch.get("case_id", "")),
        "entries": entries,
        "skipped": skipped,
    }


def _intersect(span: list[float], bounds: list[float]) -> Optional[list[float]]:
    low, high = max(float(span[0]), float(bounds[0])), min(float(span[1]), float(bounds[1]))
    return [low, high] if low <= high else None


def anchor_to_footprint(compiled: dict[str, Any], footprint: Optional[dict]) -> dict[str, Any]:
    """Keep the obstacles of the batch where they can touch the robot during the anomaly interval:
    their area intersected with the footprint the robot covered then (episode_scene.anomaly_footprint).
    The semantic memory grounded the area in the path segment the hypothesis anchors to, which reaches
    seconds before the interval; an obstacle the robot only met before it cannot have acted in it. An
    obstacle whose area does not reach the footprint goes to 'skipped'. Without a footprint, nothing
    changes. docs_output/diseno/veredicto.md, 3.3."""
    if not footprint:
        return compiled
    entries, skipped = [], list(compiled.get("skipped", []))
    for entry in compiled["entries"]:
        cause = dict(entry["cause"])
        reason = None
        if cause.get("name") == "bump_scaled":
            x, y = _intersect(cause["area_x"], footprint["x"]), _intersect(cause["area_y"], footprint["y"])
            if x is None or y is None:
                reason = "Its area does not reach where the robot was during the anomaly interval."
            else:
                cause["area_x"], cause["area_y"] = x, y
        elif cause.get("name") == "bump":
            spans = {}
            for axis in ("x", "y"):
                origin, width = cause[f"bump_{axis}_origin"], cause[f"bump_{axis}_range"] / M_TO_MM
                spans[axis] = _intersect([origin - width / 2.0, origin + width / 2.0], footprint[axis])
            if spans["x"] is None or spans["y"] is None:
                reason = "Its position range does not reach where the robot was during the anomaly interval."
            else:
                for axis, (low, high) in spans.items():
                    cause[f"bump_{axis}_origin"], cause[f"bump_{axis}_range"] = (low + high) / 2.0, (high - low) * M_TO_MM
        if reason:
            skipped.append({"hypothesis_id": entry["hypothesis_id"], "reason": reason})
            continue
        entries.append({**entry, "cause": cause})
    return {**compiled, "entries": entries, "skipped": skipped}


def main():
    parser = argparse.ArgumentParser(
        prog="Hypothesis compiler",
        description="Compiles a semantic hypothesis batch (blueprint v2) into cause payloads for the causes simulator.",
        epilog="INSIGHT - 2026",
    )
    parser.add_argument("-f", "--file", required=True, help="Path to the hypothesis batch JSON.")
    parser.add_argument("-o", "--output", required=True, help="Output path for the compiled causes JSON.")
    parser.add_argument("-c", "--catalog", required=False, help="Path to the intervention catalog (defaults to the repo one).")
    args = parser.parse_args()

    batch = json.loads(Path(args.file).read_text(encoding="utf-8"))
    compiled = compile_batch(batch, Path(args.catalog) if args.catalog else None)
    Path(args.output).write_text(json.dumps(compiled, indent=4), encoding="utf-8")
    print(
        f"Compiled {len(compiled['entries']) - 1} testable hypotheses (+1 nominal) "
        f"to {args.output}; skipped {len(compiled['skipped'])}."
    )


if __name__ == "__main__":
    main()
