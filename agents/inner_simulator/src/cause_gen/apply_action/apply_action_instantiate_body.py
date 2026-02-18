from pydantic import BaseModel
from typing import Literal
from .apply_action import ApplyAction

class ApplyActionInstantiateBody(BaseModel, ApplyAction):
    
    type:Literal["instantiate_body"]
    body_name:str
    body_position_input:str

    def render_action_variables(self):
        return [f"{self.body_name}_file:str"]

    def render_action_call(self):
        return [f"engine.instantiate_body(self.{self.body_name}_file, self.{self.body_position_input}())"]