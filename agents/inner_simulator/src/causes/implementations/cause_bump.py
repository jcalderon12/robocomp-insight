
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseBump(BaseModel, Cause):
    """A bump in the way."""

    name:Literal["bump"]

    
    bump_x_range:float
    bump_y_range:float
    bump_z_range:float
    bump_x_origin:float
    bump_y_origin:float
    bump_z_origin:float
    _bump_random_range_coordinates:float | None = PrivateAttr(default=None)
    def bump_random_range_coordinates(self):
        """Generates n random coordinates within the specified x and y ranges.
        Args:
            n (int): The number of random coordinates to generate.
        Returns:
            A generator yielding random coordinates."""
        import random
        if self._bump_random_range_coordinates is None:
            self._bump_random_range_coordinates = (self.bump_x_origin + random.uniform(-self.bump_x_range/2, self.bump_x_range/2), self.bump_y_origin + random.uniform(-self.bump_y_range/2, self.bump_y_range/2), self.bump_z_origin + random.uniform(-self.bump_z_range/2, self.bump_z_range/2))

        return self._bump_random_range_coordinates

    bump_file:str 
    def apply(self, engine:Engine):
        
        x, y, z = self.bump_random_range_coordinates()
        
        coordinates = [x, y, z]
        
        engine.instantiate_body(self.bump_file, coordinates)
        
        pass
    
    
    def apply_compute(self, engine:Engine):
        
        pass
        
    def get_generated_instances(self):
        return {
        
        "random_range_coordinates": [self.bump_random_range_coordinates()],
        
        }