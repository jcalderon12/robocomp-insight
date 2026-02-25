from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceNone(BaseModel, InstanceGenerator):
    
    type:Literal["instance_none"]

    def render_generate_instance(self):
        template = Template('''
    def instance_none(self):
        """None."""
        pass''')
        
        return template.render()
        