
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel
from typing import Literal

class CauseBump(BaseModel, Cause):
    """A bump in the way."""

    name:Literal["bump"]

    
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
        return self.x_origin + random.uniform(-self.x_range/2, self.x_range/2), self.y_origin + random.uniform(-self.y_range/2, self.y_range/2), self.z_origin + random.uniform(-self.z_range/2, self.z_range/2)

    bump_file:str 
    def apply(self, engine:Engine):
        
        x, y, z = self.random_range_coordinates()
        
        coordinates = [x, y, z]
        
        engine.instantiate_body(self.bump_file, coordinates)
        
        pass
    
    
    def apply_compute(self, engine:Engine):
        
        pass