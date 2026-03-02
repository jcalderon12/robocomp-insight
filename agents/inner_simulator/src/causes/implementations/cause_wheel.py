
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseWheel(BaseModel, Cause):
    """A wheel that stops working."""

    name:Literal["wheel"]

    
    _wheel_simple_random:float | None = PrivateAttr(default=None)
    def wheel_simple_random(self):
        """Generates n random float numbers betweem 0 and 1.
        Args:
            n (int): The number of random numbers to generate.
        Returns:
            A generator yielding random numbers."""
        import random                 
        if self._wheel_simple_random is None:
             self._wheel_simple_random = random.random()
        return self._wheel_simple_random

     
    def apply(self, engine:Engine):
        
        pass
    
    wheel_wheel:str
    from pydantic import PrivateAttr
    _wheel_stop_time:float | None = PrivateAttr(default=None)
    def apply_compute(self, engine:Engine):
        
        # [wheel] Stop wheel action
        
        if self._wheel_stop_time == None:
        
            self._wheel_stop_time = self.wheel_simple_random()
        
        time = engine.get_simulation_time()
        
        if time >= self._wheel_stop_time * engine.get_simulation_length():
        
            engine.disable_robot_wheel(self.wheel_wheel)
        
        if time == 0:
        
          self._wheel_stop_time = None
        
        pass
        
    def get_generated_instances(self):
        return {
        
        "simple_random": [self.wheel_simple_random()],
        
        }