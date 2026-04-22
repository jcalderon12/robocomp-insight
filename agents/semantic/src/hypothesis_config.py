from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _as_int(value: Any, default: int) -> int:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    text = str(value).strip()
    return text if text else default


def _get_section(config_data: dict[str, Any], section_name: str) -> dict[str, Any]:
    section = config_data.get(section_name)
    if isinstance(section, dict):
        return section

    prefix = f"{section_name}."
    flattened = {}
    for key, value in config_data.items():
        if isinstance(key, str) and key.startswith(prefix):
            flattened[key[len(prefix):]] = value
    return flattened


def _resolve_path(path_value: str, component_root: Path) -> Path:
    path = Path(path_value)
    if path.is_absolute():
        return path
    return (component_root / path).resolve()


@dataclass(frozen=True)
class HypothesisGeneratorConfig:
    enabled: bool
    output_dir: Path
    description_path: Path
    primary_model: str
    fallback_model: str
    ollama_base_url: str
    request_timeout_seconds: float
    internal_count: int
    external_count: int
    preferred_client: str
    description_char_limit: int

    @classmethod
    def from_config(cls, config_data: dict[str, Any], component_root: Path) -> "HypothesisGeneratorConfig":
        section = _get_section(config_data, "hypothesisGenerator")
        output_dir = _resolve_path(
            _as_str(section.get("OutputDir"), "generated_hypotheses"),
            component_root,
        )
        description_path = _resolve_path(
            _as_str(section.get("DescriptionPath"), "../../description.md"),
            component_root,
        )
        return cls(
            enabled=_as_bool(section.get("Enabled"), False),
            output_dir=output_dir,
            description_path=description_path,
            primary_model=_as_str(section.get("PrimaryModel"), "gpt-oss:120b-cloud"),
            fallback_model=_as_str(section.get("FallbackModel"), ""),
            ollama_base_url=_as_str(section.get("OllamaBaseUrl"), "http://localhost:11434"),
            request_timeout_seconds=_as_float(section.get("RequestTimeoutSeconds"), 180.0),
            internal_count=max(1, _as_int(section.get("InternalCount"), 3)),
            external_count=max(1, _as_int(section.get("ExternalCount"), 3)),
            preferred_client=_as_str(section.get("PreferredClient"), "ollama_http"),
            description_char_limit=max(500, _as_int(section.get("DescriptionCharLimit"), 3500)),
        )
