from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceRandomRangeCoordinates(BaseModel, InstanceGenerator):
    
    type:Literal["random_range_coordinates"]

    def render_generate_instance(self):
        template = Template('''
    x_range:float
    y_range:float
    x_origin:float
    y_origin:float
    z_range:float
    z_origin:float
    def random_range_coordinates(self):
        """Generates n random coordinates within the specified x and y ranges.
        Args:
            n (int): The number of random coordinates to generate.
        Returns:
            A generator yielding random coordinates."""
        import random
        return self.x_origin + random.uniform(-self.x_range/2, self.x_range/2), self.y_origin + random.uniform(-self.y_range/2, self.y_range/2), self.z_origin + random.uniform(-self.z_range/2, self.z_range/2)''')
        
        return template.render()
        