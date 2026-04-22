from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from owlready2 import Thing

Triple = Tuple[str, str, str]


@dataclass
class Explanation:
    retract: Triple
    event_iri: str
    event_name: str
    reason: str


class causal_validator:
    def __init__(
        self,
        runtime: Any,
        event_class_qnames: Optional[List[str]] = None,
        participant_prop_names: Optional[List[str]] = None,
        location_prop_names: Optional[List[str]] = None,
        change_event_class_qname: Optional[str] = None,
        strict_object_loss_mode: bool = False,
    ):
        self.rt = runtime

        self.event_class_qnames = event_class_qnames or ["SOMA.Event", "DUL.Event", "DUL.Action"]
        self.event_type_prop_names = ["classifies", "isOccurrenceOf", "executesTask"]
        self._event_root_classes = None

        self.change_event_class_qname = change_event_class_qname or "SOMA.ChangeDisappearanceEvent"

        self.participant_prop_names = participant_prop_names or ["hasParticipant"]
        self.location_prop_names = location_prop_names or ["hasLocation", "occursIn"]
        self.causal_prop_names = ["causes"]

        self.strict_object_loss_mode = strict_object_loss_mode
        self.allowed_object_loss_event_types = {"Task_TakeMedicine"}
        self.required_object_loss_participants = {"Agent_Nurse"}

        self.event_birth_step: Dict[Thing, int] = {}
        self.event_tags: Dict[Thing, List[str]] = {}

    def _get_event_roots(self):
        if self._event_root_classes is None:
            roots = []
            for qn in self.event_class_qnames:
                cls = self.rt._get_class(qn)
                if cls is not None:
                    roots.append(cls)
            self._event_root_classes = roots
        return self._event_root_classes

    def _is_event_class(self, cls):
        roots = self._get_event_roots()
        if not roots:
            return False

        for root in roots:
            if root in cls.ancestors():
                return True
        return False

    def _is_instance_of(self, inst: Thing, class_local: str) -> bool:
        cls = getattr(self.rt.ns, class_local, None)
        if cls is None:
            return False
        try:
            return cls in inst.is_a or cls in inst.INDIRECT_is_a
        except Exception:
            return False

    def _requires_object_anchor(self, subj: Thing) -> bool:
        return self._is_instance_of(subj, "PhysicalObject")

    def _is_action_instance(self, ev: Thing) -> bool:
        return self._is_instance_of(ev, "Action")

    def _strong_object_loss_ok(self, ev: Thing, subj: Thing, old_loc: Thing, step_index: int) -> bool:
        if not self._when_ok(ev, step_index):
            return False

        if not self._where_ok(ev, old_loc):
            return False

        if not self._is_action_instance(ev):
            return False

        event_type_names = set(self._get_event_types(ev))
        if not (event_type_names & self.allowed_object_loss_event_types):
            return False

        participants = self._get_participants(ev)
        if subj not in participants:
            return False

        participant_names = {p.name for p in participants}
        if not self.required_object_loss_participants.issubset(participant_names):
            return False

        return True

    def _assert_causal_link(self, cause_ev: Thing, observed_ev: Thing) -> bool:
        for pname in self.causal_prop_names:
            prop = getattr(self.rt.ns, pname, None)
            if prop is None:
                continue
            try:
                col = getattr(cause_ev, prop.name)
            except Exception:
                continue
            if observed_ev not in col:
                col.append(observed_ev)
                return True
        return False

    def _create_change_event_for_retract(self, retract: Triple, step_index: int) -> Thing:
        s, p, o = retract
        subj = self.rt._get_entity(s)
        old_loc = self.rt._get_entity(o)

        change_cls = self.rt._get_class(self.change_event_class_qname)
        if change_cls is None:
            change_cls = self.rt._get_class("SOMA.Event") or Thing

        with self.rt.onto:
            ev_name = f"Ep_{step_index}_{s}_{p.split('.')[-1]}_{o}"
            ev_name = ev_name.replace(".", "_")
            ep = change_cls(ev_name)

            if subj is not None:
                for pname in self.participant_prop_names:
                    prop = getattr(self.rt.ns, pname, None)
                    if prop is not None:
                        getattr(ep, prop.name).append(subj)
                        break

            if old_loc is not None:
                for pname in self.location_prop_names:
                    prop = getattr(self.rt.ns, pname, None)
                    if prop is not None:
                        getattr(ep, prop.name).append(old_loc)
                        break

        return ep

    def register_new_types(self, step: Any, step_index: int):
        for inst_name, class_qn in step.types:
            cls = self.rt._get_class(class_qn)
            if cls is None:
                continue
            if not self._is_event_class(cls):
                continue

            inst = self.rt._get_entity(inst_name)
            if inst is None:
                continue
            if inst not in self.event_birth_step:
                self.event_birth_step[inst] = step_index
                self.event_tags[inst] = list(getattr(step, "tags", []) or [])

    def has_hl_changes(self, step: Any) -> bool:
        return bool(step.retracts)

    def validate_step(self, step: Any, step_index: int):
        errors: List[str] = []
        explanations: List[Explanation] = []

        for retract in step.retracts:
            _, prop_qname, _ = retract

            prop_local = prop_qname.split(".")[-1]
            if prop_local != "hasLocation":
                continue

            explained, explanation = self._explain_retract_hasLocation(retract, step_index)
            if explained and explanation is not None:
                explanations.append(explanation)
            else:
                errors.append(
                    f"Retract {retract} en step '{step.name}' no tiene explicacion causal conocida."
                )

        return errors, explanations

    def unregister_deleted(self, names: List[str]):
        locals_to_remove = {n.split(".")[-1] for n in names}
        to_remove = [
            event
            for event in self.event_birth_step.keys()
            if getattr(event, "name", None) in locals_to_remove
        ]
        for event in to_remove:
            del self.event_birth_step[event]
            print(f"[Validator] Removed event from birth map: {event.name}")

    def _explain_retract_hasLocation(
        self, retract: Triple, step_index: int
    ) -> Tuple[bool, Optional[Explanation]]:
        ep = self._create_change_event_for_retract(retract, step_index)

        s, _, o = retract
        subj = self.rt._get_entity(s)
        old_loc = self.rt._get_entity(o)

        if subj is None or old_loc is None:
            return False, None

        candidate_events = self._get_candidate_events_upto(step_index)

        scored_candidates = []
        for ev in candidate_events:
            if "background" in (self.event_tags.get(ev, []) or []):
                continue

            if self.strict_object_loss_mode and self._requires_object_anchor(subj):
                if not self._strong_object_loss_ok(ev, subj, old_loc, step_index):
                    continue

                scored_candidates.append((2, ev, True))
                continue

            if not self._when_ok(ev, step_index):
                continue
            if not self._where_ok(ev, old_loc):
                continue
            if not self._has_event_type(ev):
                continue

            anchor_ok = self._who_anchor_ok(ev, subj, old_loc)
            if self._requires_object_anchor(subj) and not anchor_ok:
                continue

            shared_obj = self._who_shared(ev, subj)
            score = 1
            if shared_obj:
                score += 1

            scored_candidates.append((score, ev, shared_obj))

        if not scored_candidates:
            return False, None

        scored_candidates.sort(key=lambda x: x[0], reverse=True)
        _, best_event, shared = scored_candidates[0]
        if best_event is None:
            return False, None

        linked = self._assert_causal_link(best_event, ep)
        if linked:
            print(f"[CAUSAL-LINK] Added causal link: {best_event.name} -> {ep.name}")
        else:
            print(
                "[CAUSAL-LINK] No causal property found / "
                f"link not added for: {best_event.name} -> {ep.name}"
            )

        ep_name = ep.name
        event_name = best_event.name
        birth = self.event_birth_step.get(best_event, None)

        if birth is not None:
            if birth < step_index:
                when_txt = f"E precede a Ep (birth_step(E)={birth} < step(Ep)={step_index})"
            elif birth == step_index:
                when_txt = (
                    f"E coincide temporalmente con Ep (birth_step(E)={birth} = step(Ep)={step_index})"
                )
            else:
                when_txt = f"E ocurre despues de Ep (birth_step(E)={birth} > step(Ep)={step_index})"
        else:
            when_txt = "no se conoce el indice de creacion de E"

        event_location = self._get_location(best_event)
        where_ok = self._where_ok(best_event, old_loc)
        where_txt = (
            "E y Ep comparten localizacion relativa: "
            f"loc(E)={self._fmt_entity(event_location)}, "
            f"loc(Ep)={self._fmt_entity(old_loc)} -> "
            + ("compatibles" if where_ok else "NO compatibles")
        )

        participants = self._get_participants(best_event)
        if shared:
            who_txt = (
                f"E y Ep comparten participante principal: {self._fmt_entity(subj)} "
                f"(participantes de E: {', '.join(self._fmt_entity(p) for p in participants)})"
            )
        else:
            who_txt = (
                "E y Ep no comparten participante principal; "
                f"participantes de E: {', '.join(self._fmt_entity(p) for p in participants)}; "
                f"participante principal de Ep: {self._fmt_entity(subj)}"
            )

        event_types = self._get_event_types(best_event)
        if event_types:
            how_txt = f"E esta vinculado a tipo/tarea operacional: {', '.join(event_types)}"
        else:
            how_txt = (
                "E no tiene tipado o anclaje operacional suficiente "
                "(classifies / isOccurrenceOf / executesTask)"
            )

        reason = (
            f"Evento {event_name} ha sido seleccionado como causa de {ep_name} "
            f"para el retracto {retract} porque:\n"
            f"- When: {when_txt}.\n"
            f"- Where: {where_txt}.\n"
            f"- Who: {who_txt}.\n"
            f"- How: {how_txt}."
        )

        explanation = Explanation(
            retract=retract,
            event_iri=best_event.iri,
            event_name=best_event.name,
            reason=reason,
        )
        return True, explanation

    def _get_candidate_events_upto(self, step_index: int, window: int = 2) -> List[Thing]:
        lower_bound = max(1, step_index - window)
        return [
            event
            for event, birth_step in self.event_birth_step.items()
            if lower_bound <= birth_step <= step_index
        ]

    def _when_ok(self, ev: Thing, ep_step: int) -> bool:
        birth_step = self.event_birth_step.get(ev, None)
        return birth_step is not None and birth_step <= ep_step

    def _collect_locations(self, ent: Thing) -> List[Thing]:
        visited = set()
        frontier = [ent]
        locations = set()

        while frontier:
            current = frontier.pop()
            if current in visited:
                continue
            visited.add(current)
            locations.add(current)
            for pname in self.location_prop_names:
                prop = getattr(self.rt.ns, pname, None)
                if prop is None:
                    continue
                try:
                    values = list(getattr(current, prop.name))
                except AttributeError:
                    continue
                for value in values:
                    if value not in visited:
                        frontier.append(value)
        return list(locations)

    def _get_location(self, ent: Thing) -> Optional[Thing]:
        for pname in self.location_prop_names:
            prop = getattr(self.rt.ns, pname, None)
            if prop is None:
                continue
            try:
                values = list(getattr(ent, prop.name))
            except AttributeError:
                continue
            if values:
                return values[0]

        if ent in self.event_birth_step:
            participants = self._get_participants(ent)
            for participant in participants:
                for pname in self.location_prop_names:
                    prop = getattr(self.rt.ns, pname, None)
                    if prop is None:
                        continue
                    try:
                        values = list(getattr(participant, prop.name))
                    except AttributeError:
                        continue
                    if values:
                        return values[0]

        return None

    def _where_ok(self, ev: Thing, ep_loc: Thing) -> bool:
        event_location = self._get_location(ev)
        if event_location is None:
            return False

        ep_locations = self._collect_locations(ep_loc)
        return event_location in ep_locations

    def _get_participants(self, ev: Thing) -> List[Thing]:
        participants: List[Thing] = []
        for pname in self.participant_prop_names:
            prop = getattr(self.rt.ns, pname, None)
            if prop is None:
                continue
            try:
                values = list(getattr(ev, prop.name))
            except AttributeError:
                continue
            participants.extend(values)
        return participants

    def _fmt_entity(self, ent: Optional[Thing]) -> str:
        if ent is None:
            return "<none>"
        return ent.name

    def _get_event_types(self, ev: Thing) -> List[str]:
        type_names: List[str] = []
        for pname in self.event_type_prop_names:
            prop = getattr(self.rt.ns, pname, None)
            if prop is None:
                continue
            try:
                values = list(getattr(ev, prop.name))
            except AttributeError:
                continue
            type_names.extend(value.name for value in values)
        return type_names

    def _has_event_type(self, ev: Thing) -> bool:
        return len(self._get_event_types(ev)) > 0

    def _who_shared(self, ev: Thing, subj: Thing) -> bool:
        participants = self._get_participants(ev)
        return subj in participants

    def _who_anchor_ok(self, ev: Thing, subj: Thing, old_loc: Optional[Thing]) -> bool:
        participants = self._get_participants(ev)
        if subj in participants:
            return True
        if old_loc is not None and old_loc in participants:
            return True
        return False
