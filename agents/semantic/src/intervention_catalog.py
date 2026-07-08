from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


class CatalogError(ValueError):
    """Raised when the intervention catalog file is missing or malformed."""


@dataclass(frozen=True)
class GroundingResult:
    testable: bool
    blueprint: Optional[dict[str, Any]]
    reason: str = ""
    uses_activation_window: bool = False


class InterventionCatalog:
    """Closed vocabulary of executable simulation interventions.

    The catalog is the contract between the semantic agent (hypothesis
    generation) and the inner simulator (hypothesis testing): a hypothesis is
    testable iff its simulation_blueprint grounds in one catalog intervention
    with parameters inside the declared bounds.
    """

    def __init__(self, data: dict[str, Any], source: str = ""):
        self.source = source
        self.schema_version = str(data.get("schema_version", ""))
        interventions = data.get("interventions")
        assets = data.get("assets")
        if not isinstance(interventions, dict) or not interventions:
            raise CatalogError("Catalog must define a non-empty 'interventions' object.")
        if not isinstance(assets, dict):
            raise CatalogError("Catalog must define an 'assets' object.")
        self.interventions: dict[str, dict[str, Any]] = interventions
        self.assets: dict[str, dict[str, Any]] = assets

    @classmethod
    def from_file(cls, path: Path) -> "InterventionCatalog":
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except OSError as exc:
            raise CatalogError(f"Cannot read intervention catalog at '{path}': {exc}")
        except json.JSONDecodeError as exc:
            raise CatalogError(f"Intervention catalog at '{path}' is not valid JSON: {exc}")
        return cls(data, source=str(path))

    def prompt_capsule(self) -> dict[str, Any]:
        """Compact, LLM-facing view of the catalog."""
        capsule_interventions = {}
        for name, spec in self.interventions.items():
            capsule_interventions[name] = {
                "description": spec.get("description", ""),
                "uses_activation_window": bool(spec.get("uses_activation_window", False)),
                "parameters": {
                    param_name: {
                        key: value
                        for key, value in param_spec.items()
                        if key in {"type", "values", "bounds", "units", "description", "required"}
                    }
                    for param_name, param_spec in spec.get("parameters", {}).items()
                },
            }
        capsule_assets = {
            name: {
                "description": asset.get("description", ""),
                "approx_dimensions_m": asset.get("approx_dimensions_m", []),
            }
            for name, asset in self.assets.items()
        }
        return {
            "schema_version": self.schema_version,
            "interventions": capsule_interventions,
            "assets": capsule_assets,
        }

    # ---- blueprint grounding -------------------------------------------------

    def ground_blueprint(self, blueprint: Any) -> GroundingResult:
        """Validate a simulation_blueprint against the catalog.

        Returns a normalized blueprint when the hypothesis is testable, or
        testable=False with a reason otherwise. Schema-level problems (not a
        dict, missing activation_window...) still raise ValueError so the
        batch validator can reject malformed output.
        """
        if not isinstance(blueprint, dict):
            raise ValueError("simulation_blueprint must be an object.")

        intervention_name = blueprint.get("intervention")
        if intervention_name is None:
            return GroundingResult(
                testable=False,
                blueprint=None,
                reason="Hypothesis declared untestable by the generator (no catalog intervention applies).",
            )

        intervention_name = str(intervention_name).strip()
        spec = self.interventions.get(intervention_name)
        if spec is None:
            return GroundingResult(
                testable=False,
                blueprint=None,
                reason=f"Intervention '{intervention_name}' is not in the catalog.",
            )

        parameters = blueprint.get("parameters")
        if not isinstance(parameters, dict):
            return GroundingResult(
                testable=False,
                blueprint=None,
                reason=f"Intervention '{intervention_name}' requires a 'parameters' object.",
            )

        normalized_params: dict[str, Any] = {}
        for param_name, param_spec in spec.get("parameters", {}).items():
            required = bool(param_spec.get("required", False))
            if param_name not in parameters:
                if required:
                    return GroundingResult(
                        testable=False,
                        blueprint=None,
                        reason=f"Intervention '{intervention_name}' is missing required parameter '{param_name}'.",
                    )
                continue
            try:
                normalized_params[param_name] = self._normalize_parameter(
                    param_spec, parameters[param_name], param_name
                )
            except ValueError as exc:
                return GroundingResult(testable=False, blueprint=None, reason=str(exc))

        unknown = set(parameters) - set(spec.get("parameters", {}))
        if unknown:
            return GroundingResult(
                testable=False,
                blueprint=None,
                reason=f"Intervention '{intervention_name}' received unknown parameters: {sorted(unknown)}.",
            )

        return GroundingResult(
            testable=True,
            blueprint={
                "intervention": intervention_name,
                "parameters": normalized_params,
            },
            reason="",
            uses_activation_window=bool(spec.get("uses_activation_window", False)),
        )

    def _normalize_parameter(self, param_spec: dict[str, Any], value: Any, name: str) -> Any:
        kind = str(param_spec.get("type", "")).strip()
        if kind == "enum":
            candidate = str(value).strip()
            values = param_spec.get("values", [])
            if candidate not in values:
                raise ValueError(f"Parameter '{name}' must be one of {values}, got '{candidate}'.")
            return candidate
        if kind == "asset_ref":
            candidate = str(value).strip()
            if candidate not in self.assets:
                raise ValueError(
                    f"Parameter '{name}' must reference a catalog asset "
                    f"({sorted(self.assets)}), got '{candidate}'."
                )
            return candidate
        if kind == "range1d":
            return self._normalize_range(value, param_spec.get("bounds"), name)
        if kind == "range3d":
            if not isinstance(value, dict):
                raise ValueError(f"Parameter '{name}' must be an object with x/y/z ranges.")
            bounds = param_spec.get("bounds", {})
            normalized = {}
            for axis in ("x", "y", "z"):
                if axis not in value:
                    raise ValueError(f"Parameter '{name}' is missing axis '{axis}'.")
                normalized[axis] = self._normalize_range(value[axis], bounds.get(axis), f"{name}.{axis}")
            return normalized
        raise ValueError(f"Parameter '{name}' has unsupported catalog type '{kind}'.")

    @staticmethod
    def _normalize_range(value: Any, bounds: Any, name: str) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"Parameter '{name}' must be a [min, max] pair.")
        try:
            low, high = float(value[0]), float(value[1])
        except (TypeError, ValueError):
            raise ValueError(f"Parameter '{name}' must contain numbers.")
        if low > high:
            raise ValueError(f"Parameter '{name}' must have min <= max.")
        if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
            bound_low, bound_high = float(bounds[0]), float(bounds[1])
            if low < bound_low or high > bound_high:
                raise ValueError(
                    f"Parameter '{name}' range [{low}, {high}] exceeds catalog bounds "
                    f"[{bound_low}, {bound_high}]."
                )
        return [low, high]
