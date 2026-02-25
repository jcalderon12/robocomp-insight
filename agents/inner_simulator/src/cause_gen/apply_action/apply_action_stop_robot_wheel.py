from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionStopRobotWheel(BaseModel, ApplyAction):
    
    type:Literal["stop_robot_wheel"]
    label:str # Avoid name conflicts if used several times in the same cause
    time_input:str

    def render_action_variables(self):
        code = [f"{self.label}_wheel:str"]
        if self.time_input == "instance_none":
            code.extend([f"{self.label}_stop_time:float"])
        else:

            code.extend([f"from pydantic import PrivateAttr",
                        f"_{self.label}_stop_time:float | None = PrivateAttr(default=None)"])
        return code
        
    def render_action_call(self):
        code = []
        code.append(f"# [{self.label}] Stop wheel action")
        if self.time_input != "instance_none":
            code.extend([f"if self._{self.label}_stop_time == None:",
                         f"    self._{self.label}_stop_time = self.{self.time_input}()",
                         f"time = engine.get_simulation_time()",
                         f"if time >= self._{self.label}_stop_time * engine.get_simulation_length():",
                         f"    engine.disable_robot_wheel(self.{self.label}_wheel)"])
        else:
            code.extend([f"time = engine.get_simulation_time()",
                         f"if time >= self.{self.label}_stop_time:",
                         f"    engine.disable_robot_wheel(self.{self.label}_wheel)"])
            
        code.extend([f"if time == 0:",
                     f"  self._{self.label}_stop_time = None"])
        
        return code