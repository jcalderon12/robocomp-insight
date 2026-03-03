from pydantic import BaseModel
from typing import Literal
from jinja2 import Template
from .apply_action import ApplyAction

class ApplyActionInstantiateBody(BaseModel, ApplyAction):
    
    type:Literal["instantiate_body_at_input"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    body_position_input:str # The instance generator that provides the position input
    body_position_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        template = Template("{{ id }}_file:str")
        return [template.render(id=self.id)]
        
    def render_action_call(self):
        template = Template("""
x, y, z = self.{{ body_position_input_id }}_{{ body_position_input }}()
coordinates = [x, y, z]
engine.instantiate_body(self.{{ id }}_file, coordinates)""")
        code = template.render(
            id=self.id,
            body_position_input_id=self.body_position_input_id,
            body_position_input=self.body_position_input
        )
        return code