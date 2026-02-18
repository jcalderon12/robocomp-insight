from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionStopRobotWheel(BaseModel, ApplyAction):
    
    type:Literal["stop_robot_wheel"]
    label:str # Avoid name conflicts if used several times in the same cause
    wheel:str
    time_input:str

    def render_action_variables(self):
        if self.time_input == "instance_none":
            return [f"{self.label}_stop_time:float"]
        return []

    def render_action_call(self):
        code = []
        code.append(f"# [{self.label}] Stop {self.wheel} wheel action")
        if self.time_input != "instance_none":
            code.extend([f"if self.{self.label}_stop_time == None:",
                         f"    self.{self.label}_stop_time = self.{self.time_input}()",
                         f"time = engine.get_simulation_time()",
                         f"if time >= self.{self.label}_stop_time * time:",
                         f"    engine.disable_robot_wheel(\"{self.wheel}\")"])
        else:
            code.extend([f"time = engine.get_simulation_time()",
                         f"if time >= self.{self.label}_stop_time:",
                         f"    engine.disable_robot_wheel(\"{self.wheel}\")"])
        return code