
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseWheel(BaseModel, Cause):
    """A wheel that stops working at an instant sampled inside the activation window."""

    name:Literal["wheel"]

    wheel_min:float
    wheel_max:float
    _wheel_random_uniform_range:float | None = PrivateAttr(default=None)
    def wheel_random_uniform_range(self):
        """Random float uniformly sampled in [min, max], cached per cause instance."""
        import random
        if self._wheel_random_uniform_range is None:
            self._wheel_random_uniform_range = random.uniform(self.wheel_min, self.wheel_max)

        return self._wheel_random_uniform_range

     
    def apply(self, engine:Engine):
        
        pass
    
    wheel_wheel:str
    _wheel_stop_time:float | None = PrivateAttr(default=None)
    def apply_compute(self, engine:Engine):
        
        # [wheel] Stop wheel action
        
        
        
        if self._wheel_stop_time is None:
        
            self._wheel_stop_time = self.wheel_random_uniform_range()
        
        threshold = self._wheel_stop_time * engine.get_simulation_length()
        
        
        
        if engine.get_simulation_time() >= threshold:
        
            engine.disable_robot_wheel(self.wheel_wheel)
        
        pass
        
    def get_generated_instances(self):
        return {
        
        "random_uniform_range": [self.wheel_random_uniform_range()],
        
        }