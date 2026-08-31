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

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)
    
        if startup_check:
            self.startup_check()
        else:


#  Read graph and update ontology


            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        # 1. Check global variables and update ontology
        

        # 2. Call validator to check if explainability conditions are met


        # 3. If not explainable: create stop node in self.g and call explainer to generate explanations


        # 4. While not candidate call agent


        # 5. JSON to robot node


        # 6. Wait to resume


        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)







    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        if "cause_confirmed" in attribute_names:
            node = self.g.get_node(id)
            if node is not None and node.name == "problem" and node.attrs["cause_confirmed"].value:
                self.handle_cause_confirmed()

    def handle_cause_confirmed(self):
        # Validates inner_simulator's causal search result: replaces "problem" with "bump" +
        # "photograph_me" (has_intention), the take_photos counterpart of follow_me/person.
        problem_node = self.g.get_node("problem")
        if problem_node is None:
            return

        if self.g.get_node("bump") is not None:
            return  # Already handled (avoid double creation on repeated signals)

        if "problem_position" not in problem_node.attrs:
            console.print("cause_confirmed but no problem_position (non-spatial cause); dropping 'problem'.", style='yellow')
            self.g.delete_node(problem_node.id)
            return
        position_value = problem_node.attrs["problem_position"].value

        bump_node = Node(self.agent_id, "object", name="bump")
        bump_node.attrs["pos_x"] = Attribute(problem_node.attrs["pos_x"].value, self.agent_id)
        bump_node.attrs["pos_y"] = Attribute(problem_node.attrs["pos_y"].value, self.agent_id)
        bump_node.attrs["problem_position"] = Attribute(position_value, self.agent_id)  # global mm; will move to RT robot->bump later
        self.g.insert_node(bump_node)
        bump_node = self.g.get_node("bump")

        photograph_me_node = Node(self.agent_id, "affordance", name="photograph_me")
        photograph_me_node.attrs["pos_x"] = Attribute(bump_node.attrs["pos_x"].value, self.agent_id)
        photograph_me_node.attrs["pos_y"] = Attribute(bump_node.attrs["pos_y"].value - 100.0, self.agent_id)
        photograph_me_node.attrs["parent"] = Attribute(bump_node.id, self.agent_id)
        photograph_me_node.attrs["aff_interacting"] = Attribute(False, self.agent_id)
        self.g.insert_node(photograph_me_node)
        photograph_me_node = self.g.get_node("photograph_me")

        intention_edge = Edge(photograph_me_node.id, bump_node.id, "has_intention", self.agent_id)
        self.g.insert_or_assign_edge(intention_edge)

        # TODO: generate + launch the concept_bump agent here (agent_generation), once its
        # template creates create_affordance()-style logic mirroring concept_person.

        self.g.delete_node(problem_node.id)
        console.print("Created 'bump' + 'photograph_me' from resolved 'problem'.", style='bold green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
