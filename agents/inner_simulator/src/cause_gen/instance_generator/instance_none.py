from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceNone(BaseModel, InstanceGenerator):
    
    type:Literal["instance_none"]
    id:str

    def render_generate_instance(self):
        template = Template('''
    def {{id}}_instance_none(self):
        """Do nothing."""
        pass''')
        
        return template.render(id=self.id)
        