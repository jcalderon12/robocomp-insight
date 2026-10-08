"""Observed changes and ontology-selected context for hypothesis generation.

The renderer does not infer a physical event from a working-memory delta. Explanation profiles
and their applicability live in the TBox; this module only interprets their matching fields.
The legacy episode adapter preserves recorded cases without changing their JSON or RDF.
"""

from __future__ import annotations

from string import Formatter
from typing import Any, Optional

from rdflib import Graph, Namespace, RDFS

INSIGHT = Namespace("http://insight.local/ontology#")
DEFAULT_ENTITY_NAMESPACE = "http://insight.local/instances#"


def _label(value: Any, labels: dict[str, str]) -> str:
    text = str(value)
    local = text.rsplit("#", 1)[-1].rsplit("/", 1)[-1]
    return labels.get(text, labels.get(local, local))


def mechanism_nodes(graph: Graph, profiles: Optional[tuple[str, ...]] = None) -> list:
    """Select before validating: unrelated profiles may use other checks or realizations.

    None is the complete vocabulary for existing offline callers. An empty tuple selects only
    mechanisms explicitly declared applicable to any change, never the current scenario by default.
    """
    nodes = sorted(set(graph.subjects(INSIGHT.mechanismId)), key=str)
    if profiles is None:
        return nodes
    selected = set(profiles)
    return [node for node in nodes
            if any(str(graph.value(profile, INSIGHT.profileId)) in selected
                   for profile in graph.objects(node, INSIGHT.appliesToProfile))
            or any(value.toPython() is True for value in graph.objects(node, INSIGHT.appliesToAnyChange))]


def build_explanation_context(episode: dict[str, Any], graph: Graph,
                              trigger: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    """Normalize explicit changes, DSR/RDF deltas, or the observation in a legacy episode.

    New readers can supply episode.change with a summary, changes (operation/subject/predicate/
    object), affected_entity, mission, unknowns, and optional declared profile_ids. No names of
    objects, missions or causal mechanisms are assumed here.
    """
    supplied = episode.get("change") or {}
    if not isinstance(supplied, dict):
        raise ValueError("episode.change must be an object")
    event = episode.get("observation") or episode.get("accident") or {}
    support = episode.get("support") or {}
    entities = episode.get("entities") or {}
    labels = episode.get("entity_labels") or {}
    roles = {role: _label(entity, labels) for role, entity in entities.items()}
    role_entities = dict(entities)
    for role in ("supported", "supporter"):
        if support.get(role):
            role_entities[role] = support[role]
            roles[role] = _label(support[role], labels)

    def same_entity(recorded: Any, changed: Any) -> bool:
        if recorded == changed:
            return True
        # Contract-1 identifiers are local instance names, whereas DSR deltas carry full IRIs.
        # Qualify with the recorded namespace, rather than comparing arbitrary URI fragments.
        if isinstance(recorded, str) and ":" not in recorded:
            return str(changed) == episode.get("entity_namespace", DEFAULT_ENTITY_NAMESPACE) + recorded
        return False

    changes = []
    if "changes" in supplied:
        if not isinstance(supplied["changes"], list):
            raise ValueError("episode.change.changes must be a list")
        if any(not isinstance(change, dict) for change in supplied["changes"]):
            raise ValueError("each entry of episode.change.changes must be an object")
        changes = [dict(change) for change in supplied["changes"]]
    elif supplied.get("operation"):
        changes = [{key: supplied[key] for key in ("operation", "subject", "predicate", "object")
                    if key in supplied}]
    else:
        for key, operation in (("removed_triples", "removed"), ("added_triples", "added")):
            for triple in (trigger or {}).get(key, []):
                changes.append({"operation": operation,
                                **{field: triple[field] for field in ("subject", "predicate", "object")
                                   if field in triple}})
    if any(not isinstance(change.get("operation"), str) for change in changes):
        raise ValueError("each observed change needs a string operation")

    # Old episodes describe their observation but do not contain the delta. Its physical meaning
    # remains unknown; applicability of that legacy shape is declared by the TBox, not guessed.
    legacy = not supplied and not changes and bool(event.get("observed_as"))
    affected = supplied.get("affected_entity")
    if affected is None and changes:
        subjects = {change.get("subject") for change in changes if change.get("subject")}
        affected = next(iter(subjects)) if len(subjects) == 1 else None
    if affected is None and legacy:
        affected = support.get("supported")
    roles["affected_entity"] = _label(affected, labels) if affected else "the affected entity (not identified)"

    registry = {}
    for node in graph.subjects(INSIGHT.profileId):
        profile_id = str(graph.value(node, INSIGHT.profileId))
        if profile_id in registry:
            raise ValueError(f"duplicate explanation profile '{profile_id}' in the TBox")
        registry[profile_id] = node
    explicit = supplied.get("profile_ids")
    selection_notes = []
    if explicit is not None:
        if not isinstance(explicit, list) or any(not isinstance(p, str) for p in explicit):
            raise ValueError("episode.change.profile_ids must be a list of profile ids")
        missing = set(explicit) - set(registry)
        if missing:
            raise ValueError(f"undeclared explanation profiles: {sorted(missing)}")
        profiles = sorted(set(explicit))
    else:
        profiles = []
        for profile_id, node in sorted(registry.items()):
            operations = {str(v) for v in graph.objects(node, INSIGHT.matchesOperation)}
            predicates = {str(v) for v in graph.objects(node, INSIGHT.matchesPredicate)}
            required_roles = {str(v) for v in graph.objects(node, INSIGHT.requiresAffectedRole)}
            sections = {str(v) for v in graph.objects(node, INSIGHT.legacyEpisodeSection)}
            if legacy:
                matches = bool(sections) and all(episode.get(section) for section in sections)
            else:
                matches = bool(operations) and any(
                    change["operation"] in operations
                    and (not predicates or str(change.get("predicate")) in predicates)
                    and all(role not in role_entities or same_entity(role_entities[role], change.get("subject"))
                            for role in required_roles)
                    for change in changes)
            if matches:
                profiles.append(profile_id)
                missing_roles = required_roles - set(role_entities)
                if missing_roles and not legacy:
                    selection_notes.append(f"Profile {profile_id}: affected roles {', '.join(sorted(missing_roles))} "
                                           "are not recorded; applicability is provisional, not contradicted.")

    unknowns = supplied.get("unknowns", [])
    if not isinstance(unknowns, list) or any(not isinstance(value, str) for value in unknowns):
        raise ValueError("episode.change.unknowns must be a list of statements")
    profile_notes = [str(note) for profile_id in profiles
                     for note in graph.objects(registry[profile_id], INSIGHT.observationNote)]
    return {
        "schema": "insight.explanation_context/1.0",
        "observation_id": supplied.get("id") or event.get("id") or episode.get("episode_id", "observation"),
        "observed_at_s": supplied.get("time_s", (episode.get("time") or {}).get("t_obs_s", event.get("time_s"))),
        "summary": supplied.get("summary") or event.get("observed_as") or "Observed changes listed below",
        "changes": changes,
        "affected_entity": affected,
        "roles": roles,
        "mission": supplied.get("mission", episode.get("mission")),
        "profile_ids": profiles,
        "profile_labels": [str(graph.value(registry[p], RDFS.label) or p) for p in profiles],
        "profile_notes": profile_notes,
        "selection_notes": selection_notes,
        "unknowns": unknowns,
        "selection_source": "explicit" if explicit is not None else "legacy_episode" if legacy else "observed_delta",
    }


def render_context_template(template: str, context: dict[str, Any]) -> str:
    """Bind ontology descriptions to recorded roles; unsupported placeholders fail visibly."""
    roles = context.get("roles") or {}
    values = {}
    for _, field, specification, conversion in Formatter().parse(template):
        if field is None:
            continue
        if not field.isidentifier() or specification or conversion:
            raise ValueError(f"invalid role placeholder '{field}' in ontology description")
        values[field] = roles.get(field, f"the entity in role '{field}' (not recorded)")
    return template.format_map(values)
