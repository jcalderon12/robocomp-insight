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

import subprocess

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

dir_name = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(dir_name)
sys.path.append(parent_dir + "/src/")
console = Console(highlight=False)

# Ruta a 'agents' para alcanzar el paquete compartido agent_generation (plantillas y
# generación de agentes), el mismo bootstrap que usa inner_simulator.
agents_root = os.path.dirname(parent_dir)
if agents_root not in sys.path:
    sys.path.insert(0, agents_root)

from agent_generation.agent_generator import generate_agent

from pydsr import *
from pydsr import Node, Edge, rt_api, Attribute


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.print_dsr_signals = False

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

            self.rt_api = rt_api(self.g)

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        bottle_over_robot = self.check_bottle_related_robot()

        if not bottle_over_robot:
            if self.g.get_node("bump") is None:
                self.generate_problem_node()
            self.deactivate_affordance()
        
        print(flush=True, end="")
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
    
    def generate_problem_node(self):
        problem_node = self.g.get_node("problem")
        if problem_node is None:
            robot_node = self.g.get_node("robot")
            problem_node = Node(self.agent_id, "intention", name="problem")
            problem_node.attrs["parent"] = Attribute(robot_node.id, self.agent_id)
            problem_node.attrs["level"] = Attribute(robot_node.attrs["level"].value + 1, self.agent_id)
            problem_node.attrs["pos_x"] = Attribute(robot_node.attrs["pos_x"].value - 200, self.agent_id)
            problem_node.attrs["pos_y"] = Attribute(robot_node.attrs["pos_y"].value, self.agent_id)
            self.g.insert_node(problem_node)
            
            problem_node = self.g.get_node("problem")

            problem_edge = Edge(robot_node.id, problem_node.id, "has", self.agent_id);
            self.g.insert_or_assign_edge(problem_edge)

    
    def deactivate_affordance(self):
        follow_me_node = self.g.get_node("follow_me")
        if follow_me_node:
            if follow_me_node.attrs["aff_interacting"].value == True:
                print("Deactivating the affordance FOLLOW ME")
                follow_me_node.attrs["aff_interacting"].value = False
                self.g.update_node(follow_me_node)


    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        if self.print_dsr_signals:
            console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

        if "cause_confirmed" in attribute_names:
            node = self.g.get_node(id)
            if node is not None and node.name == "problem" and node.attrs["cause_confirmed"].value:
                self.handle_cause_confirmed()

    def handle_cause_confirmed(self):
        # Valida el resultado de la búsqueda causal de inner_simulator: sustituye "problem" por
        # "bump" + "photograph_me" (has_intention), el equivalente de follow_me/person para la
        # misión de fotos.
        problem_node = self.g.get_node("problem")
        if problem_node is None:
            return

        if self.g.get_node("bump") is not None:
            return  # Ya tratado; evita crearlo dos veces si la señal se repite.

        if "problem_position" not in problem_node.attrs:
            # Sólo es una comprobación de que la causa es espacial (¿aplica aquí la búsqueda en
            # rejilla de inner_simulator?); el valor en sí ya no se usa.
            console.print("cause_confirmed sin problem_position (causa no espacial); se descarta 'problem'.", style='yellow')
            self.g.delete_node(problem_node.id)
            return

        bump_node = Node(self.agent_id, "object", name="bump")
        bump_node.attrs["pos_x"] = Attribute(problem_node.attrs["pos_x"].value, self.agent_id)
        bump_node.attrs["pos_y"] = Attribute(problem_node.attrs["pos_y"].value, self.agent_id)
        # Sin problem_position: la posición real del bache la publica concept_bump como una RT
        # robot->bump viva, igual que concept_person con person, no como atributo global.
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

        if not self.launch_concept_agent("bump"):
            console.print("No se pudo generar/lanzar concept_bump.", style='bold red')

        self.g.delete_node(problem_node.id)
        console.print("Creados 'bump' + 'photograph_me' a partir del 'problem' resuelto.", style='bold green')

    def launch_concept_agent(self, cause_name: str) -> bool:
        # Genera concept_<cause_name> en caliente desde las plantillas de agent_generation
        # (robocompdsl + cmake + make) y lanza el binario: Program Manager sólo arranca lo que
        # tiene en su config estática, así que no hay a quién avisar.
        agent_name = f"concept_{cause_name}"
        console.print(f"Generando {agent_name} desde las plantillas...", style='bold yellow')
        if not generate_agent(cause_name, agents_root):
            return False

        agent_dir = os.path.join(agents_root, agent_name)
        # Popen hereda la salida de este proceso, fácil de perder de vista; se redirige a un log
        # para poder diagnosticar después si el agente casca al arrancar.
        log_path = os.path.join(agent_dir, f"{agent_name}.log")
        log_file = open(log_path, "w")
        subprocess.Popen(["bin/" + agent_name, "etc/config"], cwd=agent_dir, stdout=log_file, stderr=subprocess.STDOUT)
        console.print(f"{agent_name} generado y lanzado desde {agent_dir} (log: {log_path}).", style='bold green')
        return True

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
