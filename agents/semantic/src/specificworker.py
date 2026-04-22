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
import time
from unittest import signals
import sys

from PySide6 import QtCore
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

# Javir's imports



sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from src.ontology_mapping import DSRSemanticWrapper
from src.graphdb_client import GraphDBClient, GraphDBConfig
from src.hypothesis_config import HypothesisGeneratorConfig
from src.hypothesis_context import build_hypothesis_generation_context
from src.hypothesis_service import SemanticHypothesisService
from src.live_causal_validator import LiveCausalValidator
from pydsr import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        ##
        self.mapper = DSRSemanticWrapper()
        self.graphdb_config = GraphDBConfig.from_config(configData)
        self.graphdb_client = GraphDBClient(self.graphdb_config) if self.graphdb_config.enabled else None
        self.last_signature = None
        self.last_triples = set()
        self.initialized = False
        self.causal_validator = LiveCausalValidator()
        self.unexplained = False
        self.unexplained_reason = ""
        self.stop_inserted = False
        self.bootstrap_sync = True
        self.last_added_triples = set()
        self.last_removed_triples = set()
        self.hypothesis_generation_done = False
        self.component_root = Path(__file__).resolve().parent.parent
        self.hypothesis_config = HypothesisGeneratorConfig.from_config(configData, self.component_root)
        self.hypothesis_service = (
            SemanticHypothesisService(self.hypothesis_config, log_hook=self._log_hypothesis_event)
            if self.hypothesis_config.enabled
            else None
        )

        self.mapper.initialize_from_dsr(self.g)
        self.initialized=True
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
        except RuntimeError as e:
            print(e)
    
        if startup_check:
            self.startup_check()
        else:


            self.initialized_semantic_graph()

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

    def insert_intention_hanging_for_robot(self) -> bool:
        """Insert an unexplained intention node hanging from robot with a has_intention edge."""
        robot_node = self.g.get_node("robot")
        if robot_node is None:
            console.print("Cannot insert intention: robot node not found.", style="red")
            return False

        intention_node = Node(
            agent_id=self.agent_id,
            type="intention",
            name="unexplained",
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

    def initialized_semantic_graph(self):
        state = self.mapper.get_state()
        current_triples = set(state.triples)
        removed_triples = self.last_triples - current_triples
        added_triples = current_triples - self.last_triples
        self.last_removed_triples = set(removed_triples)
        self.last_added_triples = set(added_triples)

        if self.bootstrap_sync:
            console.print("Skipping causal validation during bootstrap synchronization.", style="yellow")
        else:
            validation = self.causal_validator.validate_delta(
                removed=removed_triples,
                added=added_triples,
                current=current_triples,
            )
            if validation.unexplained:
                self.unexplained = True
                self.unexplained_reason = validation.reason
                console.print(
                    f"[CausalValidator] unexplained=True for retract {validation.retract}. {validation.reason}",
                    style="red",
                )
                self.hypothesis_generation_done = False

        if state.signature == self.last_signature:
            self.initialized = False
            self.bootstrap_sync = False
            return
        
        if self.graphdb_client is None:
            self.last_signature = state.signature
            self.last_triples = current_triples
            self.initialized = False
            self.bootstrap_sync = False
            console.print("GraphDB disabled. Semantic state updated locally.", style="yellow")
            return
        
        try:
            self.graphdb_client.apply_delta(
                added=added_triples,
                removed=removed_triples,
            )
            self.last_signature = state.signature
            self.last_triples = current_triples
            self.initialized = False
            self.bootstrap_sync = False
            console.print(
                f"Semantic Graph synchronized to {self.graphdb_config.named_graph} "
                f"with +{len(added_triples)} / -{len(removed_triples)} changes "
                f"({len(state.triples)} current triples).",
                style="green",
            )
        except Exception as e:
            self.initialized = True
            self.bootstrap_sync = False
            console.print(f"Failed to update GraphDB: {e}", style="red")

    def generate_hypotheses_json(self) -> None:
        if self.hypothesis_service is None:
            return

        current_triples = set(self.mapper.get_state().triples)
        graph_source = "graphdb_mirror" if self.graphdb_client is not None else "local_semantic_state"
        graph_endpoint = self.graphdb_config.statements_url if self.graphdb_client is not None else ""

        context = build_hypothesis_generation_context(
            dsr_graph=self.g,
            current_triples=current_triples,
            added_triples=self.last_added_triples,
            removed_triples=self.last_removed_triples,
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
        semantic_changed = self.mapper.updated_node(self.g, node_type)
        if semantic_changed:
            self.initialized_semantic_graph()
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        semantic_changed = self.mapper.updated_node(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()

        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        self.mapper.initialize_from_dsr(self.g)
        self.initialized_semantic_graph()
        

        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        semantic_changed = self.mapper.updated_edge(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names):
        semantic_changed = self.mapper.updated_edge(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        semantic_changed = self.mapper.deleted_edge(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()

        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
