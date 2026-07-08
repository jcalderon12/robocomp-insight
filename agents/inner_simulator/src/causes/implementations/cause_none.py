
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class CauseNone(BaseModel, Cause):
    """Nominal run with no intervention. Serves as the null-hypothesis baseline for the verdict."""

    name:Literal["none"]

    

     
    def apply(self, engine:Engine):
        
        pass
    
    
    def apply_compute(self, engine:Engine):
        
        pass
        
    def get_generated_instances(self):
        return {
        
        }