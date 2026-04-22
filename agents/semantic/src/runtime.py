# /src/validator/runtime.py

import time, types
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from owlready2 import *

from hypothesis.contracts import ExperimentResult, build_unexplained_context
from hypothesis.hooks import hypothesis_agent_enabled, on_unexplained
from validator.causal_validator import causal_validator

Triple = Tuple[str, str, str]

@dataclass
class Step:
    name: str
    asserts: List[Triple] = field(default_factory=list)
    retracts: List[Triple] = field(default_factory=list)
    updates: List[Tuple[str, str, str, str]] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)
    types: List[Tuple[str, str]] = field(default_factory=list)
    deletes: List[str] = field(default_factory=list)

@dataclass
class ExperimentConfig:
    ontology_path: str
    steps: List[Step]
    extra_ontology_paths: List[str] = field(default_factory=list)
    enable_reasoner: bool = True
    strict_object_loss_mode: bool = False

class OntologyRuntime:
    def __init__(self, ont_path: str, extra_paths: Optional[List[str]] = None):
        extra_paths = extra_paths or []

        def _as_file_uri(p: str) -> str:
            return p if p.startswith("file://") else "file://" + p

        base_uri = _as_file_uri(ont_path)
        self.onto = get_ontology(base_uri).load()

        # carga ontologías extra (MO)
        self.extra_ontos = []
        for p in extra_paths:
            o = get_ontology(_as_file_uri(p)).load()
            self.extra_ontos.append(o)

        # importante: unificar namespace "ns" con clases/props de TODO lo cargado
        self.ns = types.SimpleNamespace()
        all_ontos = [self.onto] + self.extra_ontos

        for onto in all_ontos:
            for cls in onto.classes():
                setattr(self.ns, cls.name, cls)
            for prop in onto.properties():
                setattr(self.ns, prop.name, prop)

        self.timing: List[Tuple[str, float]] = []

    # --- helpers internos ---

    def record_timing(self, label: str, dt: float):
        self.timing.append((label, dt))
    
    def _add_type(self, inst, cls):
        if cls not in inst.is_a:
            inst.is_a.append(cls)
            return 1
        return 0

    def _add_prop(self, s, prop, o):
        try:
            col = getattr(s, prop.name)
        except AttributeError:
            return 0
        if o not in col:
            col.append(o)
            return 1
        return 0

    def _mat_equivalent_properties(self):
        added = 0
        with self.onto:
            for p in self.onto.properties():
                eqs = [q for q in getattr(p, "equivalent_to", [])
                       if isinstance(q, type(p))]
                if not eqs:
                    continue
                pairs = list(p.get_relations())
                for s, o in pairs:
                    for q in eqs:
                        added += self._add_prop(s, q, o)
        return added

    def _mat_subproperty_closure(self):
        added = 0
        with self.onto:
            for p in self.onto.properties():
                supers = [sp for sp in p.ancestors()
                          if sp is not p and isinstance(sp, type(p))]
                
                skip = {getattr(owl, "topObjectProperty", None),
                        getattr(owl, "topDataProperty", None),
                        getattr(owl, "ObjectProperty", None),
                        getattr(owl, "DatatypeProperty", None)}
                supers = [sp for sp in supers if sp not in skip and sp is not None]
                if not supers:
                    continue
                pairs = list(p.get_relations())
                for s, o in pairs:
                    for sp in supers:
                        added += self._add_prop(s, sp, o)
        return added

    def _mat_property_chains(self):
        added = 0
        with self.onto:
            obj_props = [p for p in self.onto.properties()
                         if isinstance(p, ObjectPropertyClass)]
            adj = {}
            for p in obj_props:
                m = {}
                for s, o in list(p.get_relations()):
                    m.setdefault(s, set()).add(o)
                adj[p] = m
            for p in obj_props:
                chain = getattr(p, "property_chain", None)
                if not chain:
                    continue
                chain = [q for q in chain if isinstance(q, ObjectPropertyClass)]
                if len(chain) < 2:
                    continue
                current_map = {s: set(vs) for s, vs in adj.get(chain[0], {}).items()}
                for r in chain[1:]:
                    next_map = {}
                    rmap = adj.get(r, {})
                    for s, mids in current_map.items():
                        outs = set()
                        for m in mids:
                            outs |= rmap.get(m, set())
                        if outs:
                            next_map[s] = outs
                    current_map = next_map
                    if not current_map:
                        break
                for s, outs in current_map.items():
                    for o in outs:
                        added += self._add_prop(s, p, o)
        return added

    def _mat_inverse_and_symmetric(self):
        added = 0
        with self.onto:
            obj_props = [p for p in self.onto.properties()
                         if isinstance(p, ObjectPropertyClass)]
            for p in obj_props:
                invs = []
                ip = getattr(p, "inverse_property", None)
                if ip:
                    invs.extend(ip if isinstance(ip, list) else [ip])
                is_sym = SymmetricProperty in getattr(p, "is_a", [])
                pairs = list(p.get_relations())
                for s, o in pairs:
                    if is_sym:
                        added += self._add_prop(o, p, s)
                    for inv in invs:
                        added += self._add_prop(o, inv, s)
        return added

    def _mat_transitive_closure(self):
        added = 0
        with self.onto:
            for p in [x for x in self.onto.properties()
                      if isinstance(x, ObjectPropertyClass)
                      and TransitiveProperty in getattr(x, "is_a", [])]:
                succ = {}
                for s, o in list(p.get_relations()):
                    succ.setdefault(s, set()).add(o)
                for s in list(succ.keys()):
                    frontier = list(succ.get(s, []))
                    seen = set(frontier)
                    while frontier:
                        mid = frontier.pop()
                        for o in succ.get(mid, []):
                            if o not in seen:
                                seen.add(o)
                                frontier.append(o)
                            added += self._add_prop(s, p, o)
        return added

    def _mat_domain_range_types(self):
        added = 0
        with self.onto:
            for p in self.onto.properties():
                doms = [d for d in getattr(p, "domain", []) if isinstance(d, ThingClass)]
                rngs = [r for r in getattr(p, "range", []) if isinstance(r, ThingClass)]
                if not doms and not rngs:
                    continue
                pairs = list(p.get_relations())
                for s, o in pairs:
                    for d in doms:
                        added += self._add_type(s, d)
                    if isinstance(p, ObjectPropertyClass) and isinstance(o, Thing):
                        for r in rngs:
                            added += self._add_type(o, r)
        return added

    def _mat_equivalent_classes(self):
        added = 0
        with self.onto:
            for C in self.onto.classes():
                eqs = [E for E in getattr(C, "equivalent_to", [])
                       if isinstance(E, ThingClass)]
                if not eqs:
                    continue
                insts = list(C.instances())
                for x in insts:
                    for E in eqs:
                        added += self._add_type(x, E)
        return added

    def _mat_subclass_closure(self):
        added = 0
        with self.onto:
            for C in self.onto.classes():
                supers = [S for S in C.ancestors()
                          if isinstance(S, ThingClass)
                          and S is not C
                          and S is not Thing]
                if not supers:
                    continue
                insts = list(C.instances())
                for x in insts:
                    for S in supers:
                        added += self._add_type(x, S)
        return added

    def materialize_all(self, max_rounds=3,
                        include_transitive=True,
                        include_chains=True):

        total = 0
        for r in range(max_rounds):
            added = 0
            added += self._mat_equivalent_properties()
            added += self._mat_subproperty_closure()
            if include_chains:
                added += self._mat_property_chains()
            added += self._mat_inverse_and_symmetric()
            if include_transitive:
                added += self._mat_transitive_closure()
            added += self._mat_domain_range_types()
            added += self._mat_equivalent_classes()
            added += self._mat_subclass_closure()
            print(f"[Materialize] round {r+1}: +{added} nuevas aserciones")
            total += added
            if added == 0:
                break
        return total


    #########################

    def _get_by_local_name(self, local: str):
        base = self.onto.base_iri or ""
        cands = []
        if base:
            if base.endswith(("#", "/")):
                cands.append(base + local)
            else:
                cands.append(base + "#" + local)
                cands.append(base + "/" + local)
        cands.append("*" + local)
        for iri in cands:
            ent = self.onto.search_one(iri=iri)
            if ent is not None:
                return ent
        return None

    def _get_entity(self, qname_or_iri: str):
        s = qname_or_iri.strip()
        if s.startswith("http://") or s.startswith("https://"):
            ent = default_world[s]
            if ent is not None:
                return ent
            with self.onto:
                local = s.rsplit("#", 1)[-1].rsplit("/", 1)[-1]
                inst = Thing(local)
                inst.iri = s
                return inst
        local = s.split(".")[-1]
        ent = self._get_by_local_name(local)
        if ent is not None:
            return ent
        with self.onto:
            return Thing(local)

    def _get_class(self, class_qname: str):
        cname = class_qname.split('.')[-1]
        return getattr(self.ns, cname, None)

    def delete_instances(self, names: List[str]):
        with self.onto:
            for n in names:
                local = n.split(".")[-1]
                inst = self._get_by_local_name(local)
                if inst is not None:
                    destroy_entity(inst)
                    print(f"[Delete] Destroyed individual: {local}")

    def apply_types(self, typings: List[Tuple[str, str]]):
        for inst_name, class_qn in typings:
            cls = self._get_class(class_qn)
            if cls is None:
                print(f"[WARN] Class not found for typing: {class_qn}")
                continue
            local = inst_name.split('.')[-1]
            existing = self._get_by_local_name(local)
            if existing:
                if cls not in existing.is_a:
                    existing.is_a.append(cls)
            else:
                with self.onto:
                    cls(local)

    def apply_triples(self,
                      asserts: List[Triple],
                      retracts: List[Triple],
                      updates: List[Tuple[str, str, str, str]]):

        for s, p, o in retracts:
            subj = self._get_entity(s)
            prop_name = p.split('.')[-1]
            prop = getattr(self.ns, prop_name, None)
            obj = self._get_entity(o)
            if prop is None:
                print(f"[WARN] Property not found (retract): {p}")
                continue
            col = getattr(subj, prop.name, None)
            if col is not None and obj in col:
                col.remove(obj)

        for s, p, o in asserts:
            subj = self._get_entity(s)
            prop_name = p.split('.')[-1]
            prop = getattr(self.ns, prop_name, None)
            obj = self._get_entity(o)
            if prop is None:
                print(f"[WARN] Property not found (assert): {p}")
                continue
            getattr(subj, prop.name).append(obj)

        for s, p, old_o, new_o in updates:
            subj = self._get_entity(s)
            prop_name = p.split('.')[-1]
            prop = getattr(self.ns, prop_name, None)
            old_obj = self._get_entity(old_o)
            new_obj = self._get_entity(new_o)
            if prop is None:
                print(f"[WARN] Property not found (update): {p}")
                continue
            col = getattr(subj, prop.name, None)
            if col is not None and old_obj in col:
                col.remove(old_obj)
            getattr(subj, prop.name).append(new_obj)

    def reason(self, label: str = "") -> float:
        t0 = time.time()
        sync_reasoner(infer_property_values=False)
        inconsistent = list(self.onto.inconsistent_classes())
        if inconsistent:
            print(f"[Reason] Ontología inconsistente después de '{label}':")
            for c in inconsistent:
                print("   -", c)
        dt = time.time() - t0
        # self.timing.append((label, dt))
        print(f"[Reason] {label}: {dt:.3f}s")
        return dt


    def _safe_filename(self, s: str) -> str:
        return "".join(c if (c.isalnum() or c in "-_.") else "_" for c in s)

def run_experiment(cfg: ExperimentConfig):
    rt = OntologyRuntime(cfg.ontology_path, extra_paths=getattr(cfg, "extra_ontology_paths", []))
    validator = causal_validator(
        rt,
        strict_object_loss_mode=getattr(cfg, "strict_object_loss_mode", False),
    )

    result = ExperimentResult(scenario_id=getattr(cfg, "scenario_id", None))

    print("\n=== START EXPERIMENT ===")
    for i, step in enumerate(cfg.steps, 1):
        print(f"\n--- Step {i}: {step.name} ---")

        t_step0 = time.time()

        if step.types:
            rt.apply_types(step.types)
            validator.register_new_types(step, i)

        rt.apply_triples(step.asserts, step.retracts, step.updates)

        if validator.has_hl_changes(step):
            if getattr(cfg, "enable_reasoner", True):
                rt.reason(step.name)
            else:
                print(f"[Reason] skipped for '{step.name}' (enable_reasoner=False)")

            errors, explanations = validator.validate_step(step, i)

            g = default_world.as_rdflib_graph()
            print("[Debug] triples en world:", len(g))

            if errors:
                print("\n[CAUSAL-VALIDATION] Cambios no explicados:")
                for msg in errors:
                    print("  -", msg)
                rt.record_timing(f"{i}:{step.name}:step_total", time.time() - t_step0)

                result.status = "unexplained"
                result.failed_step = step.name
                result.errors = list(errors)
                result.explanations = list(explanations)
                result.timing = list(rt.timing)

                if hypothesis_agent_enabled():
                    context = build_unexplained_context(
                        cfg=cfg,
                        step=step,
                        step_index=i,
                        errors=errors,
                        rt=rt,
                        validator=validator,
                    )
                    on_unexplained(context)
                break
            else:
                print("\n[CAUSAL-VALIDATION] Cambios explicados causalmente:")
                for exp in explanations:
                    print(f"  - {exp.reason}")

                result.status = "explained"
                result.failed_step = step.name
                result.errors = []
                result.explanations = list(explanations)
                result.timing = list(rt.timing)

        if step.deletes:
            validator.unregister_deleted(step.deletes)
            rt.delete_instances(step.deletes)

        rt.record_timing(f"{i}:{step.name}:step_total", time.time() - t_step0)

    print("\n=== END EXPERIMENT ===")
    print("Timings:")
    for label, dt in rt.timing:
        print(f"  {label}: {dt:.3f}s")

    result.timing = list(rt.timing)
    return result.to_legacy_dict()

