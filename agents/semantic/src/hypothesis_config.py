from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """Raised when a required configuration value is missing or invalid."""


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


def _require_str(section: dict[str, Any], key: str, section_name: str) -> str:
    """Return a non-empty string from the config or raise ConfigError."""
    raw = section.get(key)
    if raw is None:
        raise ConfigError(
            f"Missing required config key '{section_name}.{key}'."
        )
    text = str(raw).strip().strip('"').strip("'")
    if not text:
        raise ConfigError(
            f"Config key '{section_name}.{key}' must be a non-empty string."
        )
    return text


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
    catalog_path: Path
    primary_model: str
    fallback_model: str
    ollama_base_url: str
    request_timeout_seconds: float
    internal_count: int
    external_count: int
    preferred_client: str
    description_char_limit: int

    @classmethod
    def from_config(
        cls, config_data: dict[str, Any], component_root: Path
    ) -> "HypothesisGeneratorConfig":
        section_name = "hypothesisGenerator"
        section = _get_section(config_data, section_name)

        # ! Required values: must be present in the .cfg file
        primary_model = _require_str(section, "PrimaryModel", section_name)
        ollama_base_url = _require_str(section, "OllamaBaseUrl", section_name)
        preferred_client = _require_str(section, "PreferredClient", section_name)
        output_dir_raw = _require_str(section, "OutputDir", section_name)
        description_path_raw = _require_str(section, "DescriptionPath", section_name)
        catalog_path_raw = _require_str(section, "CatalogPath", section_name)
        request_timeout_raw = section.get("RequestTimeoutSeconds")
        internal_count_raw = section.get("InternalCount")
        external_count_raw = section.get("ExternalCount")
        description_char_limit_raw = section.get("DescriptionCharLimit")

        if request_timeout_raw is None:
            raise ConfigError(f"Missing required config key '{section_name}.RequestTimeoutSeconds'.")
        if internal_count_raw is None:
            raise ConfigError(f"Missing required config key '{section_name}.InternalCount'.")
        if external_count_raw is None:
            raise ConfigError(f"Missing required config key '{section_name}.ExternalCount'.")
        if description_char_limit_raw is None:
            raise ConfigError(f"Missing required config key '{section_name}.DescriptionCharLimit'.")

        enabled = _as_bool(section.get("Enabled"), False)
        fallback_model = _as_str(section.get("FallbackModel"), "")

        return cls(
            enabled=enabled,
            output_dir=_resolve_path(output_dir_raw, component_root),
            description_path=_resolve_path(description_path_raw, component_root),
            catalog_path=_resolve_path(catalog_path_raw, component_root),
            primary_model=primary_model,
            fallback_model=fallback_model,
            ollama_base_url=ollama_base_url,
            request_timeout_seconds=_as_float(request_timeout_raw, 0.0),
            internal_count=max(1, _as_int(internal_count_raw, 1)),
            external_count=max(1, _as_int(external_count_raw, 1)),
            preferred_client=preferred_client,
            description_char_limit=max(500, _as_int(description_char_limit_raw, 500)),
        )