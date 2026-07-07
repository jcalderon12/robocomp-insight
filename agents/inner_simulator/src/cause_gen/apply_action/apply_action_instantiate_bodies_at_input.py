from pydantic import BaseModel
from typing import Literal
from jinja2 import Template
from .apply_action import ApplyAction

class ApplyActionInstantiateBodiesAtInput(BaseModel, ApplyAction):
    type: Literal["instantiate_bodies_at_input"]
    id: str
    body_position_input: str
    body_position_input_id: str

    def render_action_variables(self):
        template = Template("{{ id }}_file:str")
        return [template.render(id=self.id)]

    def render_action_call(self):
        template = Template("""
for x, y, z in self.{{ body_position_input_id }}_{{ body_position_input }}():
    engine.instantiate_body(self.{{ id }}_file, [x, y, z])""")
        return template.render(
            id=self.id,
            body_position_input_id=self.body_position_input_id,
            body_position_input=self.body_position_input,
        )
