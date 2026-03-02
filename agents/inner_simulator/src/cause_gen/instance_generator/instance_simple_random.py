from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceSimpleRandom(BaseModel, InstanceGenerator):
    
    type:Literal["simple_random"]
    id:str

    def render_generate_instance(self):
        template = Template('''
    _{{id}}_{{type}}:float | None = PrivateAttr(default=None)
    def {{id}}_{{type}}(self):
        """Generates n random float numbers betweem 0 and 1.
        Args:
            n (int): The number of random numbers to generate.
        Returns:
            A generator yielding random numbers."""
        import random                 
        if self._{{id}}_{{type}} is None:
             self._{{id}}_{{type}} = random.random()
        return self._{{id}}_{{type}}''')
        
        return template.render(id=self.id, type=self.type)