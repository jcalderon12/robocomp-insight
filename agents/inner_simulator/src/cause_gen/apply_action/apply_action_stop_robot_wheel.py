from pydantic import BaseModel
from typing import Literal
from jinja2 import Template
from .apply_action import ApplyAction

class ApplyActionStopRobotWheel(BaseModel, ApplyAction):
    
    type:Literal["stop_robot_wheel"]
    id:str # Unique ID for differentiating multiple instances of this type of apply action
    time_input_type:str #  The instance generator that provides the time input, or "instance none" if it is a direct input (a float)
    time_input_id:str # The id defined in the instance generator

    def render_action_variables(self):
        template = Template("""
{{id}}_wheel:str
{% if time_input_type == "instance_none" %}
{{id}}_stop_time:float
{% else %}
_{{id}}_stop_time:float | None = PrivateAttr(default=None)  
{% endif %}""", 

id=self.id, time_input_type=self.time_input_type)
        return template.render()
        
    def render_action_call(self):
        template = Template("""
# [{{id}}] Stop wheel action
{% if time_input_type != "instance_none" %}
self._{{id}}_stop_time = self._{{id}}_stop_time or self.{{time_input_id}}_{{time_input_type}}()
threshold = self._{{id}}_stop_time * engine.get_simulation_length()
{% else %}
threshold = self.{{id}}_stop_time
{% endif %}
if engine.get_simulation_time() >= threshold:
    engine.disable_robot_wheel(self.{{id}}_wheel)""", 

    id=self.id, time_input_type=self.time_input_type, time_input_id=self.time_input_id)
        
        code = template.render()
        return code