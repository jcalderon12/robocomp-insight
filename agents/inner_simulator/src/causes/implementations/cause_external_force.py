
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseExternal_force(BaseModel, Cause):
    """An external force pushes a body (robot or bottle) during an activation window."""

    name:Literal["external_force"]

    force_x_range:float
    force_y_range:float
    force_z_range:float
    force_x_origin:float
    force_y_origin:float
    force_z_origin:float
    _force_random_range_coordinates:float | None = PrivateAttr(default=None)
    def force_random_range_coordinates(self):
        """Generates n random coordinates within the specified x and y ranges.
        Args:
            n (int): The number of random coordinates to generate.
        Returns:
            A generator yielding random coordinates."""
        import random
        if self._force_random_range_coordinates is None:
            self._force_random_range_coordinates = (self.force_x_origin + random.uniform(-self.force_x_range/2, self.force_x_range/2), self.force_y_origin + random.uniform(-self.force_y_range/2, self.force_y_range/2), self.force_z_origin + random.uniform(-self.force_z_range/2, self.force_z_range/2))

        return self._force_random_range_coordinates

     
    def apply(self, engine:Engine):
        
        pass
    
    force_target:str
    force_window_start:float
    force_window_end:float
    def apply_compute(self, engine:Engine):
        
        # [force] External force during activation window
        
        sim_time = engine.get_simulation_time()
        
        sim_length = engine.get_simulation_length()
        
        if self.force_window_start * sim_length <= sim_time <= self.force_window_end * sim_length:
        
            fx, fy, fz = self.force_random_range_coordinates()
        
            engine.apply_external_force(self.force_target, [fx, fy, fz])
        
        pass
        
    def get_generated_instances(self):
        return {
        
        "random_range_coordinates": [self.force_random_range_coordinates()],
        
        }