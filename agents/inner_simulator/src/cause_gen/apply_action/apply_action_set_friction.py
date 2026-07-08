from pydantic import BaseModel
from typing import Literal
from jinja2 import Template
from .apply_action import ApplyAction

class ApplyActionSetFriction(BaseModel, ApplyAction):
    """Sets the lateral friction coefficient of a named body (e.g. the floor).
    Intended as an apply action (executed once when the scenario is set up)."""

    type:Literal["set_friction"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    value_input_type:str # The instance generator that provides the friction value (e.g. random_uniform_range)
    value_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        return [f"{self.id}_target:str"]

    def render_action_call(self):
        template = Template("""
# [{{id}}] Set lateral friction
engine.set_lateral_friction(self.{{id}}_target, self.{{value_input_id}}_{{value_input_type}}())""")

        code = template.render(
            id=self.id,
            value_input_type=self.value_input_type,
            value_input_id=self.value_input_id,
        )
        return code
