from cause import Cause
from engines.engine import Engine
from pydantic import BaseModel
from typing import Literal

class CauseBump(BaseModel, Cause):
    """A bump in the way."""

    name:Literal["bump"]

    
    x_range:float
    y_range:float
    def random_range_coordinates(self):
        """Generates n random coordinates within the specified x and y ranges.
        Args:
            n (int): The number of random coordinates to generate.
        Returns:
            A generator yielding random coordinates."""
        import random
        return random.uniform(self.x_range), random.uniform(self.y_range)

    
    def simple_random(self):
        """Generates n random float numbers betweem 0 and 1.
        Args:
            n (int): The number of random numbers to generate.
        Returns:
            A generator yielding random numbers."""
        import random
        return random.random()

    
    def instance_none(self):
        """None."""
        pass

    bump_file:str 
    def apply(self, engine:Engine):
        
        engine.instantiate_body(self.bump_file, self.random_range_coordinates())
        
    
    
    def apply_compute(self, engine:Engine):
        
        # [fr] Stop FR wheel action
        
        if self.fr_stop_time == None:
        
            self.fr_stop_time = self.simple_random()
        
        time = engine.get_simulation_time()
        
        if time >= self.fr_stop_time * time:
        
            engine.disable_robot_wheel("FR")
        