from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceSimpleRandom(BaseModel, InstanceGenerator):
    
    type:Literal["simple_random"]

    def render_generate_instance(self):
        template = Template('''
    def simple_random(self):
        """Generates n random float numbers betweem 0 and 1.
        Args:
            n (int): The number of random numbers to generate.
        Returns:
            A generator yielding random numbers."""
        import random
        return random.random()''')
        
        return template.render()
        