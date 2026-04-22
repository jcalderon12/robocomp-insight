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

        self.mapper.initialize_from_dsr(self.g)
        self.initialized=True
        ##

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
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


        return True


    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def insert_intention_hanging_for_robot(self) -> bool:
        """Insert an intention node hanging from robot with a HAS edge."""
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

        has_edge = Edge(intention_id, robot_node.id, "has", self.agent_id)
        edge_ok = self.g.insert_or_assign_edge(has_edge)
        if not edge_ok:
            console.print("Cannot link intention to robot using HAS edge.", style="red")
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

    def initialized_semantic_graph(self):
        state = self.mapper.get_state()
        current_triples = set(state.triples)

        validation = self.causal_validator.validate_delta(
            removed=self.last_triples - current_triples,
            added=current_triples - self.last_triples,
            current=current_triples,
        )
        if validation.unexplained:
            self.unexplained = True
            self.unexplained_reason = validation.reason
            console.print(
                f"[CausalValidator] unexplained=True for retract {validation.retract}. {validation.reason}",
                style="red",
            )

        if state.signature == self.last_signature:
            self.initialized = False
            return
        
        if self.graphdb_client is None:
            self.last_signature = state.signature
            self.last_triples = current_triples
            self.initialized = False
            console.print("GraphDB disabled. Semantic state updated locally.", style="yellow")
            return
        
        try:
            self.graphdb_client.replace_graph(state.to_turtle())
            self.last_signature = state.signature
            self.last_triples = current_triples
            self.initialized = False
            console.print(
                f"Semantic Graph synchronized to {self.graphdb_config.named_graph} "
                f"with {len(state.triples)} triples.", style="green"
            )
        except Exception as e:
            self.initialized = True
            console.print(f"Failed to update GraphDB: {e}", style="red")

    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: list[str]):
        self.mapper.initialize_from_dsr(self.g)
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

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: list[str]):
        semantic_changed = self.mapper.updated_edge(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        semantic_changed = self.mapper.deleted_edge(self.g, type)
        if semantic_changed:
            self.initialized_semantic_graph()

        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
