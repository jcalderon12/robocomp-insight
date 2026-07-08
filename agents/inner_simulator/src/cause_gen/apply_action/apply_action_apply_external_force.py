from pydantic import BaseModel
from typing import Literal
from jinja2 import Template
from .apply_action import ApplyAction

class ApplyActionApplyExternalForce(BaseModel, ApplyAction):
    """Applies an external force vector to a named body while the simulation
    time is inside the [window_start, window_end] fraction of the episode.
    Intended as an apply_compute action (the engine re-applies the force each
    simulation step)."""

    type:Literal["apply_external_force"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    force_input_type:str # The instance generator that provides the force vector (e.g. random_range_coordinates)
    force_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        return [
            f"{self.id}_target:str",
            f"{self.id}_window_start:float",
            f"{self.id}_window_end:float",
        ]

    def render_action_call(self):
        template = Template("""
# [{{id}}] External force during activation window
sim_time = engine.get_simulation_time()
sim_length = engine.get_simulation_length()
if self.{{id}}_window_start * sim_length <= sim_time <= self.{{id}}_window_end * sim_length:
    fx, fy, fz = self.{{force_input_id}}_{{force_input_type}}()
    engine.apply_external_force(self.{{id}}_target, [fx, fy, fz])""")

        code = template.render(
            id=self.id,
            force_input_type=self.force_input_type,
            force_input_id=self.force_input_id,
        )
        return code
