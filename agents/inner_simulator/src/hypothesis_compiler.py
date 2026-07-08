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
    }
    for axis in ("x", "y", "z"):
        origin, span = _range_to_origin_span(position_range[axis])
        payload[f"bump_{axis}_origin"] = origin
        payload[f"bump_{axis}_range"] = span
    return payload


def _compile_disable_wheel(params: dict[str, Any], blueprint: dict[str, Any], catalog: dict[str, Any]) -> dict[str, Any]:
    start, end = _window_fractions(blueprint)
    return {
        "wheel_wheel": str(params["wheel_id"]),
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
    entries: list[dict[str, Any]] = [
        {
            "hypothesis_id": NOMINAL_HYPOTHESIS_ID,
            "title": "Nominal run (null hypothesis)",
            "cause": {"name": "none"},
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
                "cause": cause_payload,
            }
        )

    return {
        "case_id": str(batch.get("case_id", "")),
        "entries": entries,
        "skipped": skipped,
    }


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
