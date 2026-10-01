from typing import Literal
from .instance_generator import InstanceGenerator
from pydantic import BaseModel
from jinja2 import Template

class InstanceFixedGridPositions(BaseModel, InstanceGenerator):
    type: Literal["fixed_grid_positions"]
    id: str
    center_x_origin: float
    center_y_origin: float
    center_z_origin: float
    x_count: int
    y_count: int
    x_spacing: float
    y_spacing: float

    def render_generate_instance(self):
        template = Template('''
_{{id}}_{{type}}: list[tuple[float, float, float]] | None = PrivateAttr(default=None)
def {{id}}_{{type}}(self):
    """Returns a logically distributed grid of positions around the problem origin."""
    if self._{{id}}_{{type}} is None:
        positions = []
        x_start = self.center_x_origin - ((self.x_count - 1) / 2) * self.x_spacing
        y_start = self.center_y_origin - ((self.y_count - 1) / 2) * self.y_spacing
        for i in range(self.x_count):
            for j in range(self.y_count):
                positions.append(
                    (
                        x_start + i * self.x_spacing,
                        y_start + j * self.y_spacing,
                        self.center_z_origin,
                    )
                )
        self._{{id}}_{{type}} = positions

    return self._{{id}}_{{type}}''')

        return template.render(id=self.id, type=self.type)
