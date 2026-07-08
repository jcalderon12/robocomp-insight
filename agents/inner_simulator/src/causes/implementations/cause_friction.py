
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseFriction(BaseModel, Cause):
    """The floor's lateral friction coefficient changes for the whole episode (slippery surface)."""

    name:Literal["friction"]

    friction_min:float
    friction_max:float
    _friction_random_uniform_range:float | None = PrivateAttr(default=None)
    def friction_random_uniform_range(self):
        """Random float uniformly sampled in [min, max], cached per cause instance."""
        import random
        if self._friction_random_uniform_range is None:
            self._friction_random_uniform_range = random.uniform(self.friction_min, self.friction_max)

        return self._friction_random_uniform_range

    friction_target:str 
    def apply(self, engine:Engine):
        
        # [friction] Set lateral friction
        
        engine.set_lateral_friction(self.friction_target, self.friction_random_uniform_range())
        
        pass
    
    
    def apply_compute(self, engine:Engine):
        
        pass
        
    def get_generated_instances(self):
        return {
        
        "random_uniform_range": [self.friction_random_uniform_range()],
        
        }