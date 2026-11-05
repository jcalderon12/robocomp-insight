from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def compact_timestamp_token() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _stringify_triple(triple: tuple[str, str, str]) -> dict[str, str]:
    return {
        "subject": triple[0],
        "predicate": triple[1],
        "object": triple[2],
    }


def _split_sentences(text: str) -> list[str]:
    normalized = re.sub(r"\s+", " ", text).strip()
    if not normalized:
        return []
    return [
        sentence.strip()
        for sentence in re.split(r"(?<=[.!?])\s+", normalized)
        if sentence.strip()
    ]


def extract_robot_description_capsule(description_path: Path, char_limit: int) -> dict[str, Any]:
    try:
        text = description_path.read_text(encoding="utf-8")
    except OSError as exc:
        return {
            "source": str(description_path),
            "available": False,
            "summary": "",
            "selected_sentences": [],
            "error": str(exc),
        }

    sentences = _split_sentences(text)
    selected = []
    seen = set()
    current_size = 0

    for sentence in sentences:
        if sentence in seen:
            continue
        extra = len(sentence) + (1 if selected else 0)
        if current_size + extra > char_limit:
            break
        selected.append(sentence)
        seen.add(sentence)
        current_size += extra

    summary = " ".join(selected)
    return {
        "source": str(description_path),
        "available": True,
        "summary": summary,
        "selected_sentences": selected,
        "error": "",
    }


def collect_dsr_nodes(dsr_graph: Any) -> list[dict[str, Any]]:
    try:
        nodes = dsr_graph.get_nodes()
    except Exception:
        return []

    collected = []
    for node in nodes:
        node_name = getattr(node, "name", "")
        node_type = getattr(node, "type", "")
        node_id = getattr(node, "id", None)
        collected.append(
            {
                "id": node_id,
                "name": node_name,
                "type": node_type,
            }
        )
    collected.sort(key=lambda item: (str(item.get("type", "")), str(item.get("name", ""))))
    return collected


def build_hypothesis_generation_context(
    *,
    dsr_graph: Any,
    current_triples: Iterable[tuple[str, str, str]],
    added_triples: Iterable[tuple[str, str, str]],
    removed_triples: Iterable[tuple[str, str, str]],
    unexplained_reason: str,
    description_path: Path,
    description_char_limit: int,
    graph_source: str,
    graph_endpoint: str = "",
) -> dict[str, Any]:
    current_triples = tuple(sorted(current_triples))
    added_triples = tuple(sorted(added_triples))
    removed_triples = tuple(sorted(removed_triples))
    nodes = collect_dsr_nodes(dsr_graph)
    description_capsule = extract_robot_description_capsule(
        description_path=description_path,
        char_limit=description_char_limit,
    )
    case_id = f"semantic_unexplained_{compact_timestamp_token()}"

    return {
        "schema_version": "1.0",
        "case_id": case_id,
        "generated_at": utc_timestamp(),
        "graph_source": graph_source,
        "graph_endpoint": graph_endpoint,
        "unexplained_reason": unexplained_reason,
        "removed_triples": [_stringify_triple(triple) for triple in removed_triples],
        "added_triples": [_stringify_triple(triple) for triple in added_triples],
        "current_triples": [_stringify_triple(triple) for triple in current_triples],
        "dsr_nodes": nodes,
        "robot_description_capsule": description_capsule,
    }
