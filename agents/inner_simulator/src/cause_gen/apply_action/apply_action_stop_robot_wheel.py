from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionStopRobotWheel(BaseModel, ApplyAction):
    
    type:Literal["stop_robot_wheel"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    time_input_type:str #  The instance generator that provides the time input, or "instance none" if it is a direct input (a float)
    time_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        code = [f"{self.id}_wheel:str"]
        if self.time_input_type == "instance_none":
            code.extend([f"{self.id}_stop_time:float"])
        else:

            code.extend([f"from pydantic import PrivateAttr",
                        f"_{self.id}_stop_time:float | None = PrivateAttr(default=None)"])
        return code
        
    def render_action_call(self):
        code = []
        code.append(f"# [{self.id}] Stop wheel action")
        if self.time_input_type != "instance_none":
            code.extend([f"if self._{self.id}_stop_time == None:",
                         f"    self._{self.id}_stop_time = self.{self.time_input_id}_{self.time_input_type}()",
                         f"time = engine.get_simulation_time()",
                         f"if time >= self._{self.id}_stop_time * engine.get_simulation_length():",
                         f"    engine.disable_robot_wheel(self.{self.id}_wheel)"])
        else:
            code.extend([f"time = engine.get_simulation_time()",
                         f"if time >= self.{self.id}_stop_time:",
                         f"    engine.disable_robot_wheel(self.{self.id}_wheel)"])
            
        code.extend([f"if time == 0:",
                     f"  self._{self.id}_stop_time = None"])
        
        return code