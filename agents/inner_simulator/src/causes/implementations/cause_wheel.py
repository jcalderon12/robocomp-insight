
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel
from typing import Literal

class CauseWheel(BaseModel, Cause):
    """A wheel that stops working."""

    name:Literal["wheel"]

    
    def simple_random(self):
        """Generates n random float numbers betweem 0 and 1.
        Args:
            n (int): The number of random numbers to generate.
        Returns:
            A generator yielding random numbers."""
        import random
        return random.random()

    id_wheel:str
    from pydantic import PrivateAttr
    _id_stop_time:float | None = PrivateAttr(default=None) 
    def apply(self, engine:Engine):
        pass
    
    
    def apply_compute(self, engine:Engine):
        # [id] Stop wheel action

        if self._id_stop_time == None:
            self._id_stop_time = self.simple_random()
            print(f"Generated random {self._id_stop_time}. Will stop wheel at {self._id_stop_time * engine.get_simulation_length()} seconds")

        time = engine.get_simulation_time()
        
        if time >= self._id_stop_time * engine.get_simulation_length():
        
            engine.disable_robot_wheel(self.id_wheel)
        
        if time == 0:
            print(f"{self._id_stop_time} reset!")
            self._id_stop_time = None