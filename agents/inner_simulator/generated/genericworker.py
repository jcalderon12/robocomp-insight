#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 20262026 by YOUR NAME HERE
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

import sys, Ice, os
from PySide6 import QtWidgets, QtCore
from pydsr import DSRGraph
from ConfigLoader import ConfigLoader



class GenericWorker(QtCore.QObject):

    kill = QtCore.Signal()

    def __init__(self, mprx, configData):
        super(GenericWorker, self).__init__()


        self.configData = configData

        self.Period = 30
        self.timer = QtCore.QTimer(self)
        self.agent_name = configData.get("Agent", {}).get("name")
        self.agent_id = configData.get("Agent", {}).get("id")

        # Initialize DSR
        sur_names = ConfigLoader.get_sur_names(configData, "Agent")

        self.graphs = {}
        self.g = None
        if not sur_names:
            domain = configData.get("Agent", {}).get("domain", 0)
            config_file = configData.get("Agent", {}).get("configFile")
    
            new_graph = DSRGraph(0, self.agent_name, self.agent_id, config_file, True, domain)
            self.g = new_graph
    
            print("Graph loaded")
            self.graphs[""] = self.g
        else:
            print(f"Multiple graphs found: {len(sur_names)}")
    
            for name in sur_names:
                prefix_data = configData["Agent"][name]
        
                config_file = prefix_data.get("configFile")
                domain = prefix_data.get("domain", 0)

                self.graphs[name] = DSRGraph(0, self.agent_name, self.agent_id, config_file, True, domain)
                print(f"Graph {name} loaded")

            self.g = self.graphs[sur_names[0]]

    @QtCore.Slot()
    def killYourSelf(self):
        rDebug("Killing myself")
        self.kill.emit()

    # \brief Change compute period
    # @param per Period in ms
    @QtCore.Slot(int)
    def setPeriod(self, p):
        print("Period changed", p)
        self.Period = p
        self.timer.start(self.Period)
