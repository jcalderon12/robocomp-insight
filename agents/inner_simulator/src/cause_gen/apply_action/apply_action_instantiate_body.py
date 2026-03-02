from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionInstantiateBody(BaseModel, ApplyAction):
    
    type:Literal["instantiate_body_at_input"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    body_position_input:str # The instance generator that provides the position input
    body_position_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        return [f"{self.id}_file:str"]
        
    def render_action_call(self):
        code = [f"x, y, z = self.{self.body_position_input_id}_{self.body_position_input}()",
                f"coordinates = [x, y, z]",
                f"engine.instantiate_body(self.{self.id}_file, coordinates)"]
        return code