from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import requests

from src.hypothesis_config import HypothesisGeneratorConfig


TOP_LEVEL_KEYS = {
    "schema_version",
    "status",
    "case_id",
    "trace_id",
    "generated_at",
    "generator",
    "model",
    "trigger",
    "context_summary",
    "hypotheses",
    "errors",
}

HYPOTHESIS_KEYS = {
    "hypothesis_id",
    "family",
    "rank",
    "title",
    "rationale",
    "confidence",
    "grounding",
    "simulation_blueprint",
    "expected_observations",
}

BLUEPRINT_KEYS = {
    "action",
    "entity",
    "reference_frame",
    "activation_window",
    "asset",
}


def _extract_json_object(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            obj, end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            return obj
    raise ValueError("Could not decode a JSON object from the model response.")


def _normalize_message_content(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, str):
                parts.append(item)
            elif isinstance(item, dict):
                text = item.get("text")
                if text:
                    parts.append(str(text))
        return "".join(parts)
    return str(content)


def _ensure_list_of_strings(value: Any, field_name: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"Field '{field_name}' must be a non-empty list.")
    items = [str(item).strip() for item in value if str(item).strip()]
    if not items:
        raise ValueError(f"Field '{field_name}' must contain non-empty values.")
    return items


def _validate_activation_window(window: Any) -> dict[str, float]:
    if not isinstance(window, dict):
        raise ValueError("simulation_blueprint.activation_window must be an object.")
    try:
        start_fraction = float(window.get("start_fraction"))
        end_fraction = float(window.get("end_fraction"))
    except (TypeError, ValueError):
        raise ValueError("activation_window requires numeric start_fraction and end_fraction.")
    if not (0.0 <= start_fraction <= 1.0 and 0.0 <= end_fraction <= 1.0):
        raise ValueError("activation_window fractions must be within [0, 1].")
    if start_fraction > end_fraction:
        raise ValueError("activation_window.start_fraction must be <= end_fraction.")
    return {
        "start_fraction": start_fraction,
        "end_fraction": end_fraction,
    }


def _validate_geometry(value: Any) -> Optional[dict[str, Any]]:
    if value in (None, {}):
        return None
    if not isinstance(value, dict):
        raise ValueError("simulation_blueprint.geometry must be an object or null.")
    kind = str(value.get("kind", "")).strip()
    if kind not in {"box", "cylinder", "mesh"}:
        raise ValueError("geometry.kind must be one of: box, cylinder, mesh.")
    dimensions = value.get("dimensions")
    if not isinstance(dimensions, list) or not dimensions:
        raise ValueError("geometry.dimensions must be a non-empty list.")
    dims = []
    for item in dimensions:
        try:
            numeric = float(item)
        except (TypeError, ValueError):
            raise ValueError("geometry.dimensions must contain numbers.")
        if numeric <= 0:
            raise ValueError("geometry.dimensions values must be > 0.")
        dims.append(numeric)

    position_range = value.get("position_range")
    if not isinstance(position_range, dict):
        raise ValueError("geometry.position_range must be an object.")
    normalized_range = {}
    for axis in ("x", "y", "z"):
        axis_range = position_range.get(axis)
        if not isinstance(axis_range, list) or len(axis_range) != 2:
            raise ValueError(f"geometry.position_range.{axis} must be a two-value list.")
        try:
            low = float(axis_range[0])
            high = float(axis_range[1])
        except (TypeError, ValueError):
            raise ValueError(f"geometry.position_range.{axis} must contain numbers.")
        if low > high:
            raise ValueError(f"geometry.position_range.{axis} must have low <= high.")
        normalized_range[axis] = [low, high]

    return {
        "kind": kind,
        "dimensions": dims,
        "position_range": normalized_range,
    }


def _validate_asset(value: Any) -> Optional[dict[str, Any]]:
    if value in (None, {}):
        return None
    if not isinstance(value, dict):
        raise ValueError("simulation_blueprint.asset must be an object or null.")
    asset_kind = str(value.get("kind", "")).strip()
    asset_hint = str(value.get("hint", "")).strip()
    if not asset_kind or not asset_hint:
        raise ValueError("simulation_blueprint.asset requires non-empty 'kind' and 'hint'.")
    return {
        "kind": asset_kind,
        "hint": asset_hint,
    }


def validate_hypothesis_batch(batch: dict[str, Any], internal_count: int, external_count: int) -> dict[str, Any]:
    if not isinstance(batch, dict):
        raise ValueError("Batch output must be a JSON object.")

    missing_keys = TOP_LEVEL_KEYS - set(batch)
    if missing_keys:
        raise ValueError(f"Batch output is missing top-level keys: {sorted(missing_keys)}")

    hypotheses = batch.get("hypotheses")
    if not isinstance(hypotheses, list):
        raise ValueError("Top-level 'hypotheses' must be a list.")

    internal_seen = 0
    external_seen = 0
    normalized_hypotheses = []
    family_ranks = {"internal": set(), "external": set()}

    for hypothesis in hypotheses:
        if not isinstance(hypothesis, dict):
            raise ValueError("Each hypothesis must be an object.")
        missing = HYPOTHESIS_KEYS - set(hypothesis)
        if missing:
            raise ValueError(f"Hypothesis missing keys: {sorted(missing)}")

        family = str(hypothesis.get("family", "")).strip().lower()
        if family not in {"internal", "external"}:
            raise ValueError("Each hypothesis family must be 'internal' or 'external'.")
        if family == "internal":
            internal_seen += 1
        else:
            external_seen += 1

        try:
            rank = int(hypothesis.get("rank"))
        except (TypeError, ValueError):
            raise ValueError("Each hypothesis rank must be an integer.")
        if rank < 1:
            raise ValueError("Each hypothesis rank must be >= 1.")
        if rank in family_ranks[family]:
            raise ValueError(f"Rank {rank} is duplicated within family '{family}'.")
        family_ranks[family].add(rank)

        try:
            confidence = float(hypothesis.get("confidence"))
        except (TypeError, ValueError):
            raise ValueError("Each hypothesis confidence must be numeric.")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("Each hypothesis confidence must be within [0, 1].")

        grounding = _ensure_list_of_strings(hypothesis.get("grounding"), "grounding")
        expected_observations = _ensure_list_of_strings(
            hypothesis.get("expected_observations"),
            "expected_observations",
        )

        blueprint = hypothesis.get("simulation_blueprint")
        if not isinstance(blueprint, dict):
            raise ValueError("Each hypothesis requires a simulation_blueprint object.")
        blueprint_missing = BLUEPRINT_KEYS - set(blueprint)
        if blueprint_missing:
            raise ValueError(f"simulation_blueprint missing keys: {sorted(blueprint_missing)}")

        normalized_blueprint = {
            "action": str(blueprint.get("action", "")).strip(),
            "entity": str(blueprint.get("entity", "")).strip(),
            "reference_frame": str(blueprint.get("reference_frame", "")).strip(),
            "activation_window": _validate_activation_window(blueprint.get("activation_window")),
            "asset": _validate_asset(blueprint.get("asset")),
        }
        if not normalized_blueprint["action"] or not normalized_blueprint["entity"] or not normalized_blueprint["reference_frame"]:
            raise ValueError("simulation_blueprint.action/entity/reference_frame must be non-empty.")
        if family == "external" and not normalized_blueprint["asset"]:
            raise ValueError("External hypotheses must define asset information.")

        normalized_hypotheses.append(
            {
                "hypothesis_id": str(hypothesis.get("hypothesis_id", "")).strip(),
                "family": family,
                "rank": rank,
                "title": str(hypothesis.get("title", "")).strip(),
                "rationale": str(hypothesis.get("rationale", "")).strip(),
                "confidence": confidence,
                "grounding": grounding,
                "simulation_blueprint": normalized_blueprint,
                "expected_observations": expected_observations,
            }
        )
        current = normalized_hypotheses[-1]
        if not current["hypothesis_id"] or not current["title"] or not current["rationale"]:
            raise ValueError("Each hypothesis requires non-empty hypothesis_id, title, and rationale.")

    if internal_seen != internal_count:
        raise ValueError(f"Expected {internal_count} internal hypotheses but received {internal_seen}.")
    if external_seen != external_count:
        raise ValueError(f"Expected {external_count} external hypotheses but received {external_seen}.")

    normalized = dict(batch)
    normalized["hypotheses"] = sorted(
        normalized_hypotheses,
        key=lambda item: (item["family"], item["rank"], item["hypothesis_id"]),
    )
    return normalized


def build_generation_prompt(context: dict[str, Any], config: HypothesisGeneratorConfig) -> str:
    schema_description = {
        "top_level": {
            "schema_version": "string",
            "status": "string",
            "case_id": "string",
            "trace_id": "string",
            "generated_at": "ISO-8601 string",
            "generator": "string",
            "model": "string",
            "trigger": "object",
            "context_summary": "object",
            "hypotheses": "list",
            "errors": "list[string]",
        },
        "hypothesis": {
            "hypothesis_id": "string",
            "family": "internal | external",
            "rank": "integer",
            "title": "string",
            "rationale": "string",
            "confidence": "float in [0,1]",
            "grounding": ["string", "..."],
            "simulation_blueprint": {
                "action": "string",
                "entity": "string",
                "reference_frame": "string",
                "activation_window": {
                    "start_fraction": "float in [0,1]",
                    "end_fraction": "float in [0,1]",
                },
                "asset": {
                    "kind": "string",
                    "hint": "string",
                },
            },
            "expected_observations": ["string", "..."],
        },
    }

    instructions = [
        "You are generating candidate explanatory hypotheses for a robotics anomaly.",
        "Output exactly one JSON object and nothing else.",
        f"Generate exactly {config.internal_count} internal hypotheses and {config.external_count} external hypotheses.",
        "Do not include markdown, comments, or introductory prose.",
        "Do not cite this prompt or describe the generation process.",
        "Do not propose abstract explanations with no simulation blueprint.",
        "Keep the output minimal: only use the fields shown in the required schema.",
        "Internal hypotheses must be grounded only in robot subsystems supported by the robot description capsule.",
        "External hypotheses must be grounded in local physical changes in the environment.",
        "For external hypotheses, include an asset object with a short kind and hint.",
        "Do not include parameter ranges or geometry.",
        "Do not copy the observed failure as the hypothesis itself; propose a latent cause.",
        "Keep ranks unique within each family, starting at 1.",
    ]

    payload = {
        "instructions": instructions,
        "required_schema": schema_description,
        "context": context,
    }
    return json.dumps(payload, indent=2, ensure_ascii=True)


@dataclass
class HypothesisGenerationResult:
    ok: bool
    batch: dict[str, Any]
    output_path: Path
    error: str = ""


class SemanticHypothesisService:
    def __init__(
        self,
        config: HypothesisGeneratorConfig,
        log_hook: Optional[Callable[[str, str], None]] = None,
    ):
        self.config = config
        self.session = requests.Session()
        self.log_hook = log_hook

    def _log(self, level: str, message: str) -> None:
        if self.log_hook is not None:
            self.log_hook(level, message)

    def _invoke_chatollama(self, prompt: str, model_name: str) -> str:
        self._log(
            "info",
            f"[HypothesisService] Invoking ChatOllama with model='{model_name}' "
            f"base_url='{self.config.ollama_base_url}' timeout={self.config.request_timeout_seconds}s.",
        )
        from langchain_ollama import ChatOllama

        chat = ChatOllama(
            model=model_name,
            base_url=self.config.ollama_base_url,
            temperature=0,
            client_kwargs={"timeout": self.config.request_timeout_seconds},
        )
        response = chat.invoke(prompt)
        normalized = _normalize_message_content(getattr(response, "content", response))
        self._log(
            "info",
            f"[HypothesisService] ChatOllama response received from model='{model_name}' "
            f"({len(normalized)} chars).",
        )
        return normalized

    def _invoke_ollama_http(self, prompt: str, model_name: str) -> str:
        endpoint = self.config.ollama_base_url.rstrip("/") + "/api/chat"
        self._log(
            "info",
            f"[HypothesisService] Invoking Ollama HTTP with model='{model_name}' "
            f"endpoint='{endpoint}' timeout={self.config.request_timeout_seconds}s.",
        )
        payload = {
            "model": model_name,
            "messages": [
                {
                    "role": "user",
                    "content": prompt,
                }
            ],
            "stream": False,
            "options": {
                "temperature": 0,
            },
        }
        response = self.session.post(
            endpoint,
            json=payload,
            timeout=self.config.request_timeout_seconds,
        )
        response.raise_for_status()
        data = response.json()
        message = data.get("message", {})
        normalized = _normalize_message_content(message.get("content", ""))
        self._log(
            "info",
            f"[HypothesisService] Ollama HTTP response received from model='{model_name}' "
            f"({len(normalized)} chars).",
        )
        return normalized

    def _generate_with_model(self, prompt: str, model_name: str) -> tuple[dict[str, Any], str]:
        errors = []
        preferred = (self.config.preferred_client or "").lower()
        clients = []
        if preferred == "chatollama":
            clients = [self._invoke_chatollama, self._invoke_ollama_http]
        else:
            clients = [self._invoke_ollama_http, self._invoke_chatollama]

        for client in clients:
            try:
                self._log(
                    "info",
                    f"[HypothesisService] Trying client='{client.__name__}' with model='{model_name}'.",
                )
                raw_text = client(prompt, model_name)
                decoded = _extract_json_object(raw_text)
                self._log(
                    "info",
                    f"[HypothesisService] JSON object decoded successfully with client='{client.__name__}' "
                    f"and model='{model_name}'.",
                )
                return decoded, client.__name__
            except Exception as exc:
                errors.append(f"{client.__name__}: {exc}")
                self._log(
                    "warning",
                    f"[HypothesisService] Client='{client.__name__}' failed with model='{model_name}': {exc}",
                )

        raise RuntimeError("; ".join(errors))

    def _build_error_payload(
        self,
        *,
        context: dict[str, Any],
        trace_id: str,
        error_messages: list[str],
        model_name: str = "",
    ) -> dict[str, Any]:
        return {
            "schema_version": "1.0",
            "status": "error",
            "case_id": context.get("case_id", "semantic_unexplained"),
            "trace_id": trace_id,
            "generated_at": context.get("generated_at", ""),
            "generator": "semantic_hypothesis_service",
            "model": model_name,
            "trigger": {
                "unexplained_reason": context.get("unexplained_reason", ""),
                "removed_triples": context.get("removed_triples", []),
                "added_triples": context.get("added_triples", []),
            },
            "context_summary": {
                "graph_source": context.get("graph_source", ""),
                "graph_endpoint": context.get("graph_endpoint", ""),
                "dsr_nodes": context.get("dsr_nodes", []),
                "robot_description_capsule": context.get("robot_description_capsule", {}),
            },
            "hypotheses": [],
            "errors": list(error_messages),
        }

    def generate(self, context: dict[str, Any]) -> HypothesisGenerationResult:
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        trace_id = f"trace_{uuid.uuid4().hex}"
        prompt = build_generation_prompt(context=context, config=self.config)
        self._log(
            "info",
            f"[HypothesisService] Starting generation for case_id='{context.get('case_id', '')}' "
            f"trace_id='{trace_id}' with prompt size {len(prompt)} chars.",
        )
        batch = None
        model_used = ""
        generator_used = ""
        errors = []

        candidate_models = [self.config.primary_model]
        if self.config.fallback_model and self.config.fallback_model != self.config.primary_model:
            candidate_models.append(self.config.fallback_model)

        for model_name in candidate_models:
            try:
                self._log(
                    "info",
                    f"[HypothesisService] Attempting model='{model_name}' for "
                    f"{self.config.internal_count} internal and {self.config.external_count} external hypotheses.",
                )
                batch, generator_used = self._generate_with_model(prompt=prompt, model_name=model_name)
                model_used = model_name
                break
            except Exception as exc:
                errors.append(f"{model_name}: {exc}")
                self._log(
                    "warning",
                    f"[HypothesisService] Model attempt failed for model='{model_name}': {exc}",
                )

        if batch is None:
            error_payload = self._build_error_payload(
                context=context,
                trace_id=trace_id,
                error_messages=errors,
            )
            output_path = self._write_batch(error_payload, context.get("case_id", "semantic_unexplained"))
            self._log(
                "warning",
                f"[HypothesisService] Generation ended with error payload written to '{output_path}'.",
            )
            return HypothesisGenerationResult(
                ok=False,
                batch=error_payload,
                output_path=output_path,
                error="; ".join(errors),
            )

        batch.setdefault("schema_version", "1.0")
        batch["status"] = "success"
        batch["case_id"] = context.get("case_id", batch.get("case_id", "semantic_unexplained"))
        batch["trace_id"] = trace_id
        batch["generated_at"] = context.get("generated_at", batch.get("generated_at", ""))
        batch["generator"] = generator_used
        batch["model"] = model_used
        batch["trigger"] = {
            "unexplained_reason": context.get("unexplained_reason", ""),
            "removed_triples": context.get("removed_triples", []),
            "added_triples": context.get("added_triples", []),
        }
        batch["context_summary"] = {
            "graph_source": context.get("graph_source", ""),
            "graph_endpoint": context.get("graph_endpoint", ""),
            "current_triples": context.get("current_triples", []),
            "dsr_nodes": context.get("dsr_nodes", []),
            "robot_description_capsule": context.get("robot_description_capsule", {}),
        }
        batch.setdefault("errors", [])
        try:
            self._log(
                "info",
                f"[HypothesisService] Validating hypothesis batch from model='{model_used}'.",
            )
            validated = validate_hypothesis_batch(
                batch=batch,
                internal_count=self.config.internal_count,
                external_count=self.config.external_count,
            )
            output_path = self._write_batch(validated, validated["case_id"])
            self._log(
                "info",
                f"[HypothesisService] Validation successful. Batch written to '{output_path}' "
                f"with {len(validated.get('hypotheses', []))} hypotheses.",
            )
            return HypothesisGenerationResult(
                ok=True,
                batch=validated,
                output_path=output_path,
            )
        except Exception as exc:
            error_payload = self._build_error_payload(
                context=context,
                trace_id=trace_id,
                error_messages=[str(exc)],
                model_name=model_used,
            )
            output_path = self._write_batch(error_payload, context.get("case_id", "semantic_unexplained"))
            self._log(
                "warning",
                f"[HypothesisService] Validation failed for model='{model_used}'. "
                f"Error payload written to '{output_path}'. Reason: {exc}",
            )
            return HypothesisGenerationResult(
                ok=False,
                batch=error_payload,
                output_path=output_path,
                error=str(exc),
            )

    def _write_batch(self, batch: dict[str, Any], case_id: str) -> Path:
        safe_case = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in case_id)
        output_path = self.config.output_dir / f"{safe_case}.json"
        output_path.write_text(json.dumps(batch, indent=2, ensure_ascii=True), encoding="utf-8")
        return output_path
