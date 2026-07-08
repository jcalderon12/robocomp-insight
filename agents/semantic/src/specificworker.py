#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from pathlib import Path
import threading
import time
import sys

from PySide6 import QtCore
from PySide6.QtCore import QTimer, Signal, Qt
from PySide6.QtWidgets import QApplication
from pydsr import signals, Attribute, Node, Edge
from rich.console import Console
from genericworker import *
import interfaces as ifaces



sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from src.ontology_mapping import DSRSemanticWrapper, UNEXPLAINED_INTENTION_NAME
from src.graphdb_client import GraphDBClient, GraphDBConfig
from src.hypothesis_config import HypothesisGeneratorConfig
from src.hypothesis_context import build_hypothesis_generation_context
from src.hypothesis_service import SemanticHypothesisService
from src.live_causal_validator import LiveCausalValidator
from src.verdict_ingestor import ingest_verdict

# DSR contract with the inner simulator (attributes of the intention node)
HYPOTHESES_FILEPATH_ATTR = "hypotheses_filepath"
VERDICT_FILEPATH_ATTR = "verdict_filepath"
GRAPHDB_RETRY_SECONDS = 5.0



class SpecificWorker(GenericWorker):
    # Class-level Qt signal: emitted from any thread (DSR slots may not run on the
    # main Qt thread). The QueuedConnection delivers it to _arm_sync_timer on the
    # main thread, where touching QTimer is safe.
    _sync_request = Signal()

    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # Coalescing infrastructure. Initialized first because everything below may
        # mutate semantic state, and from this point on slots could (in principle)
        # observe a partially-built worker.
        self._sync_lock = threading.Lock()
        self._sync_needed = False
        self._sync_pending = False
        self._sync_timer = QTimer()
        self._sync_timer.setSingleShot(True)
        self._sync_timer.setInterval(50)
        self._sync_timer.timeout.connect(self._drain_sync)
        self._sync_request.connect(self._arm_sync_timer, Qt.QueuedConnection)

        ##
        self.mapper = DSRSemanticWrapper()
        self.graphdb_config = GraphDBConfig.from_config(configData)
        self.graphdb_client = GraphDBClient(self.graphdb_config) if self.graphdb_config.enabled else None
        self.last_signature = None
        self.last_triples = set()
        self._last_validated_signature = None
        self.causal_validator = LiveCausalValidator()
        self.unexplained = False
        self.unexplained_reason = ""
        self.stop_inserted = False
        self.bootstrap_sync = True
        self.trigger_added: frozenset[tuple[str, str, str]] = frozenset()
        self.trigger_removed: frozenset[tuple[str, str, str]] = frozenset()
        self.hypothesis_generation_done = False
        self.last_hypotheses_path = None
        self.hypotheses_path_published = False
        self.ingested_verdict_paths: set[str] = set()
        self._parsed_ingestions = {}
        self._graphdb_retry_at = 0.0
        self.component_root = Path(__file__).resolve().parent.parent
        self.hypothesis_config = HypothesisGeneratorConfig.from_config(configData, self.component_root)
        self.hypothesis_service = (
            SemanticHypothesisService(self.hypothesis_config, log_hook=self._log_hypothesis_event)
            if self.hypothesis_config.enabled
            else None
        )

        self.mapper.initialize_from_dsr(self.g)
        self._bootstrap_remote_state()
        ##

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except Exception as e:
            console.print(f"Failed to connect DSR signals: {e}. Aborting startup.", style="red")
            raise
    
        if startup_check:
            self.startup_check()
        else:


            self.sync_semantic_to_graphdb()

            # for n in self.g.nodes():
            #     print(n)
            #     for e in self.g.edges(n):
            #         print("  ", e)
            self.print_graph()

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        


            # self._sync_semantic_graph(force=True)


    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        if self.unexplained and not self.stop_inserted:
            inserted = self.insert_intention_hanging_for_robot()
            if inserted:
                self.stop_inserted = True
                console.print(
                    f"Unexplained change detected. Intention node inserted. Reason: {self.unexplained_reason}",
                    style="red",
                )
            else:
                console.print(
                    f"Unexplained change detected, but intention node could not be inserted. Reason: {self.unexplained_reason}",
                    style="red",
                )

        if self.unexplained and self.hypothesis_service is not None and not self.hypothesis_generation_done:
            self.generate_hypotheses_json()

        # Publish (and retry until the node accepts it) the hypotheses batch path so
        # the inner simulator can pick it up from the working memory.
        if (
            self.unexplained
            and self.hypothesis_generation_done
            and self.last_hypotheses_path is not None
            and not self.hypotheses_path_published
        ):
            self.hypotheses_path_published = self._publish_path_attribute(
                HYPOTHESES_FILEPATH_ATTR, self.last_hypotheses_path
            )

        # Once the hypotheses are out, watch for the inner simulator's verdict and
        # close the loop: consolidate the accepted causal relation and resolve the
        # unexplained intention.
        if self.unexplained and self.hypotheses_path_published:
            self._check_and_ingest_verdict()

        return True


    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def _log_hypothesis_event(self, level: str, message: str) -> None:
        styles = {
            "info": "cyan",
            "warning": "yellow",
            "error": "red",
        }
        console.print(message, style=styles.get(level, "white"))

    def _publish_path_attribute(self, attribute_name: str, path) -> bool:
        """Expose a file path as an attribute of the 'unexplained' intention node so
        other agents (inner_simulator) can pick it up from the working memory."""
        node = self.g.get_node(UNEXPLAINED_INTENTION_NAME)
        if node is None:
            console.print(
                f"Cannot publish {attribute_name}: '{UNEXPLAINED_INTENTION_NAME}' node not found yet.",
                style="yellow",
            )
            return False
        try:
            node.attrs[attribute_name] = Attribute(str(path), self.agent_id)
            self.g.update_node(node)
        except Exception as exc:
            console.print(f"Cannot publish {attribute_name} on intention node: {exc}", style="red")
            return False
        console.print(f"Published {attribute_name}='{path}' on intention node.", style="cyan")
        return True

    def _check_and_ingest_verdict(self) -> None:
        """Ingest the verdict published by the inner simulator on the intention node:
        write the episodic explanation to GraphDB and resolve the unexplained cycle."""
        node = self.g.get_node(UNEXPLAINED_INTENTION_NAME)
        if node is None or VERDICT_FILEPATH_ATTR not in node.attrs:
            return
        verdict_path = str(node.attrs[VERDICT_FILEPATH_ATTR].value)
        if not verdict_path or verdict_path in self.ingested_verdict_paths:
            return

        ingestion = self._parsed_ingestions.get(verdict_path)
        if ingestion is None:
            ingestion = ingest_verdict(verdict_path)
            self._parsed_ingestions[verdict_path] = ingestion
        if not ingestion.ok:
            self.ingested_verdict_paths.add(verdict_path)
            console.print(f"Verdict at '{verdict_path}' could not be ingested: {ingestion.reason}", style="red")
            return

        if ingestion.triples:
            if self.graphdb_client is not None:
                if time.monotonic() < self._graphdb_retry_at:
                    return
                try:
                    self.graphdb_client.apply_delta(added=set(ingestion.triples), removed=set())
                except Exception as exc:
                    self._graphdb_retry_at = time.monotonic() + GRAPHDB_RETRY_SECONDS
                    console.print(f"Could not consolidate causal triples in GraphDB: {exc}", style="red")
                    return
            console.print(
                f"Case '{ingestion.case_id}' explained by intervention "
                f"'{ingestion.accepted_intervention}' ({len(ingestion.triples)} triples).",
                style="green",
            )
        else:
            console.print(
                f"Verdict for case '{ingestion.case_id}' accepted no hypothesis: {ingestion.reason}",
                style="yellow",
            )

        self.ingested_verdict_paths.add(verdict_path)
        self._resolve_unexplained_cycle(node)

    def _resolve_unexplained_cycle(self, intention_node) -> None:
        """Remove the unexplained intention node and re-arm the detection cycle."""
        try:
            self.g.delete_node(intention_node.id)
        except Exception as exc:
            console.print(f"Could not delete intention node: {exc}", style="red")
        self.unexplained = False
        self.unexplained_reason = ""
        self.stop_inserted = False
        self.hypothesis_generation_done = False
        self.last_hypotheses_path = None
        self.hypotheses_path_published = False
        self.trigger_added = frozenset()
        self.trigger_removed = frozenset()
        console.print("Unexplained cycle resolved and re-armed.", style="green")

    def insert_intention_hanging_for_robot(self) -> bool:
        """Insert an unexplained intention node hanging from robot with a has_intention edge."""
        robot_node = self.g.get_node("robot")
        if robot_node is None:
            console.print("Cannot insert intention: robot node not found.", style="red")
            return False

        intention_node = Node(
            agent_id=self.agent_id,
            type="intention",
            name=UNEXPLAINED_INTENTION_NAME,
        )

        intention_id = self.g.insert_node(intention_node)
        if intention_id is None:
            existing = self.g.get_node("unexplained")
            if existing is None:
                console.print("Cannot insert intention: node creation failed.", style="red")
                return False
            intention_node = existing
            intention_id = intention_node.id

        has_intention_edge = Edge(robot_node.id, intention_id, "has_intention", self.agent_id)
        edge_ok = self.g.insert_or_assign_edge(has_intention_edge)
        if not edge_ok:
            console.print("Cannot link robot to intention using has_intention edge.", style="red")
            return False

        return True

    def print_graph(self):
        """Print node names and outgoing edges for quick graph visualization."""
        try:
            nodes = self.g.get_nodes()
        except Exception as e:
            console.print(f"Could not read graph nodes: {e}", style="red")
            return

        console.print("=== DSR Graph ===", style="cyan")
        if not nodes:
            console.print("(empty graph)", style="yellow")
            return

        for node in nodes:
            node_id = getattr(node, "id", "?")
            node_name = getattr(node, "name", f"node_{node_id}")
            node_type = getattr(node, "type", "unknown")
            console.print(f"- {node_name} [{node_type}] id={node_id}", style="white")

            try:
                node_edges = node.get_edges()
            except Exception:
                node_edges = []

            outgoing = [e for e in node_edges if getattr(e, "origin", None) == node_id]
            if not outgoing:
                console.print("    -> (no outgoing edges)", style="dim")
                continue

            for edge in outgoing:
                dst_id = getattr(edge, "destination", None)
                dst_node = self.g.get_node(dst_id) if dst_id is not None else None
                dst_name = getattr(dst_node, "name", str(dst_id))
                edge_type = getattr(edge, "type", "unknown")
                console.print(f"    -> [{edge_type}] {dst_name}", style="green")


    # Semantic

    def _bootstrap_remote_state(self) -> None:
        if self.graphdb_client is None:
            return

        try:
            remote_triples = self.graphdb_client.get_graph_triples()
            managed_remote_triples = self.mapper.filter_managed_triples(remote_triples)
            self.last_triples = managed_remote_triples
            self.last_signature = tuple(sorted(managed_remote_triples))
            console.print(
                f"Bootstrap semantic snapshot loaded from GraphDB: "
                f"{len(managed_remote_triples)} managed triples.",
                style="cyan",
            )
        except Exception as e:
            console.print(
                f"Could not bootstrap semantic snapshot from GraphDB: {e}",
                style="yellow",
            )

    def _arm_sync_timer(self) -> None:
        """Arm the coalescing timer. Runs on the main thread (QueuedConnection)."""
        if not self._sync_timer.isActive():
            self._sync_timer.start()

    def _request_sync(self) -> None:
        """Mark semantic state needed and request a coalesced sync. Thread-safe."""
        emit = False
        with self._sync_lock:
            self._sync_needed = True
            if not self._sync_pending:
                self._sync_pending = True
                emit = True
        if emit:
            self._sync_request.emit()

    def _drain_sync(self) -> None:
        """Coalesced sync entrypoint. Runs on the main thread when the QTimer fires."""
        with self._sync_lock:
            if not self._sync_needed:
                self._sync_pending = False
                return
            self._sync_needed = False

        self.sync_semantic_to_graphdb()

        rearm = False
        with self._sync_lock:
            if self._sync_needed:
                # A slot mutated state during the sync; arm another coalescing window.
                rearm = True
            else:
                self._sync_pending = False
        if rearm:
            self._sync_timer.start()

    def sync_semantic_to_graphdb(self):
        # Snapshot all state under the lock; do HTTP outside the lock so a slow
        # apply_delta does not block DSR slot handlers.
        with self._sync_lock:
            state = self.mapper.get_state()
            current_triples = set(state.triples)
            signature = state.signature
            removed_triples = self.last_triples - current_triples
            added_triples = current_triples - self.last_triples

            # Only validate when the semantic state has actually changed since the last
            # validation. last_signature tracks "last signature pushed to remote" and gets
            # held back on apply_delta failure; _last_validated_signature is independent so
            # transient remote failures don't cause us to re-fire the validator on the same
            # delta every Compute tick.
            if signature != self._last_validated_signature:
                if self.bootstrap_sync:
                    console.print("Skipping causal validation during bootstrap synchronization.", style="yellow")
                else:
                    validation = self.causal_validator.validate_delta(
                        removed=removed_triples,
                        added=added_triples,
                        current=current_triples,
                    )
                    if validation.unexplained:
                        # Snapshot the triggering delta only on the False->True transition,
                        # so retries / unrelated later syncs don't overwrite the original cause.
                        if not self.unexplained:
                            self.trigger_added = frozenset(added_triples)
                            self.trigger_removed = frozenset(removed_triples)
                        self.unexplained = True
                        self.unexplained_reason = validation.reason
                        console.print(
                            f"[CausalValidator] unexplained=True for retract {validation.retract}. {validation.reason}",
                            style="red",
                        )
                        self.hypothesis_generation_done = False
                self._last_validated_signature = signature

            if signature == self.last_signature:
                self.bootstrap_sync = False
                return

            if self.graphdb_client is None:
                self.last_signature = signature
                self.last_triples = current_triples
                self.bootstrap_sync = False
                console.print("GraphDB disabled. Semantic state updated locally.", style="yellow")
                return

        # HTTP without the lock: apply_delta may take hundreds of ms.
        try:
            self.graphdb_client.apply_delta(
                added=added_triples,
                removed=removed_triples,
            )
        except Exception as e:
            with self._sync_lock:
                self.bootstrap_sync = False
            console.print(f"Failed to update GraphDB: {e}", style="red")
            return

        with self._sync_lock:
            self.last_signature = signature
            self.last_triples = current_triples
            self.bootstrap_sync = False

        console.print(
            f"Semantic Graph synchronized to {self.graphdb_config.named_graph} "
            f"with +{len(added_triples)} / -{len(removed_triples)} changes "
            f"({len(current_triples)} current triples).",
            style="green",
        )

    def generate_hypotheses_json(self) -> None:
        if self.hypothesis_service is None:
            return

        current_triples = set(self.mapper.get_state().triples)
        graph_source = "graphdb_mirror" if self.graphdb_client is not None else "local_semantic_state"
        graph_endpoint = self.graphdb_config.statements_url if self.graphdb_client is not None else ""

        context = build_hypothesis_generation_context(
            dsr_graph=self.g,
            current_triples=current_triples,
            added_triples=self.trigger_added,
            removed_triples=self.trigger_removed,
            unexplained_reason=self.unexplained_reason,
            description_path=self.hypothesis_config.description_path,
            description_char_limit=self.hypothesis_config.description_char_limit,
            graph_source=graph_source,
            graph_endpoint=graph_endpoint,
        )
        console.print(
            f"[HypothesisService] Context built for case_id='{context.get('case_id', '')}' "
            f"with {len(context.get('current_triples', []))} current triples, "
            f"+{len(context.get('added_triples', []))} added, "
            f"-{len(context.get('removed_triples', []))} removed and "
            f"{len(context.get('dsr_nodes', []))} DSR nodes.",
            style="cyan",
        )

        try:
            generation_started_at = time.perf_counter()
            result = self.hypothesis_service.generate(context)
            generation_elapsed = time.perf_counter() - generation_started_at
            self.hypothesis_generation_done = True
            if result.ok:
                self.last_hypotheses_path = result.output_path
                self.hypotheses_path_published = False
                console.print(
                    f"Hypothesis batch generated at {result.output_path}. "
                    f"{len(result.batch.get('hypotheses', []))} hypotheses exported. "
                    f"Elapsed: {generation_elapsed:.3f}s.",
                    style="cyan",
                )
            else:
                console.print(
                    f"Hypothesis batch generation failed. Error payload stored at {result.output_path}. "
                    f"Reason: {result.error}. Elapsed: {generation_elapsed:.3f}s.",
                    style="yellow",
                )
        except Exception as exc:
            generation_elapsed = time.perf_counter() - generation_started_at
            self.hypothesis_generation_done = True
            console.print(
                f"Unexpected error during hypothesis generation: {exc}. "
                f"Elapsed: {generation_elapsed:.3f}s.",
                style="red",
            )

    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names):
        node = self.g.get_node(id)
        node_type = getattr(node, "type", None) if node is not None else None
        with self._sync_lock:
            semantic_changed = self.mapper.updated_node(self.g, node_type)
        if semantic_changed:
            self._request_sync()
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        with self._sync_lock:
            semantic_changed = self.mapper.updated_node(self.g, type)
        if semantic_changed:
            self._request_sync()

        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        with self._sync_lock:
            self.mapper.initialize_from_dsr(self.g)
        self._request_sync()


        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        with self._sync_lock:
            semantic_changed = self.mapper.updated_edge(self.g, type)
        if semantic_changed:
            self._request_sync()

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names):
        with self._sync_lock:
            semantic_changed = self.mapper.updated_edge(self.g, type)
        if semantic_changed:
            self._request_sync()
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        with self._sync_lock:
            semantic_changed = self.mapper.deleted_edge(self.g, type)
        if semantic_changed:
            self._request_sync()

        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
