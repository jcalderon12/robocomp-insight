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
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            self.print_dsr_signals = False

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        bottle_over_robot = self.check_bottle_related_robot()

        if not bottle_over_robot:
            self.deactivate_affordance()
        

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)



    def check_bottle_related_robot(self):
        bottle_node = self.g.get_node("bottle")
        if not bottle_node:
            return False 
        
        robot_node = self.g.get_node("robot")
        if not robot_node:
            return False
        
        edge = self.g.get_edge(robot_node.id, bottle_node.id, "RT")
        if not edge:
            return False
        
        return True
    
    def deactivate_affordance(self):
        follow_me_node = self.g.get_node("follow_me")
        if follow_me_node:
            print("Deactivating the affordance FOLLOW ME")
            follow_me_node.attrs["aff_interacting"].value = False
            self.g.update_node(follow_me_node)


    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        if self.print_dsr_signals:
            console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        if self.print_dsr_signals:
            console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        if self.print_dsr_signals:
            console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        if self.print_dsr_signals:
            console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        if self.print_dsr_signals:
            console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
