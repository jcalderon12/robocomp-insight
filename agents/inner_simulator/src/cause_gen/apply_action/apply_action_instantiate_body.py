from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionInstantiateBody(BaseModel, ApplyAction):
    
    type:Literal["instantiate_body_at_input"]
    body_name:str
    body_position_input:str

    def render_action_variables(self):
        return [f"{self.body_name}_file:str"]
        
    def render_action_call(self):
        code = [f"x, y, z = self.{self.body_position_input}()",
                f"coordinates = [x, y, z]",
                f"engine.instantiate_body(self.{self.body_name}_file, coordinates)"]
        return code