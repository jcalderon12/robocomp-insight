from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceRandomRangeCoordinates(BaseModel, InstanceGenerator):
    
    type:Literal["random_range_coordinates"]
    id:str
        
    def render_generate_instance(self):
        template = Template('''
{{id}}_x_range:float
{{id}}_y_range:float
{{id}}_z_range:float
{{id}}_x_origin:float
{{id}}_y_origin:float
{{id}}_z_origin:float
_{{id}}_{{type}}:float | None = PrivateAttr(default=None)
def {{id}}_{{type}}(self):
    """Generates n random coordinates within the specified x and y ranges.
    Args:
        n (int): The number of random coordinates to generate.
    Returns:
        A generator yielding random coordinates."""
    import random
    if self._{{id}}_{{type}} is None:
        self._{{id}}_{{type}} = (self.{{id}}_x_origin + random.uniform(-self.{{id}}_x_range/2, self.{{id}}_x_range/2), self.{{id}}_y_origin + random.uniform(-self.{{id}}_y_range/2, self.{{id}}_y_range/2), self.{{id}}_z_origin + random.uniform(-self.{{id}}_z_range/2, self.{{id}}_z_range/2))

    return self._{{id}}_{{type}}''')
        
        return template.render(id=self.id, type=self.type)
        