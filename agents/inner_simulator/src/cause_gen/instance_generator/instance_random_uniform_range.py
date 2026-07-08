from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceRandomUniformRange(BaseModel, InstanceGenerator):
    """Single random float sampled uniformly in [id_min, id_max]."""

    type:Literal["random_uniform_range"]
    id:str

    def render_generate_instance(self):
        template = Template('''
{{id}}_min:float
{{id}}_max:float
_{{id}}_{{type}}:float | None = PrivateAttr(default=None)
def {{id}}_{{type}}(self):
    """Random float uniformly sampled in [min, max], cached per cause instance."""
    import random
    if self._{{id}}_{{type}} is None:
        self._{{id}}_{{type}} = random.uniform(self.{{id}}_min, self.{{id}}_max)

    return self._{{id}}_{{type}}''')

        return template.render(id=self.id, type=self.type)
