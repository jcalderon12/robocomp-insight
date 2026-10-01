
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal, Optional

class CauseBump(BaseModel, Cause):
    """A bump in the way."""

    name:Literal["bump"]
    bump_file:str
    bump_x_range:float
    bump_y_range:float
    bump_z_range:float
    grid_dimensions:list[int] = [10, 10]
    bump_x_origin:Optional[float] = None
    bump_y_origin:Optional[float] = None
    bump_z_origin:Optional[float] = None
    _bump_positions:list[tuple[float, float, float]] | None = PrivateAttr(default=None)
    
    def bump_origin(self, engine:Engine | None = None) -> tuple[float, float, float]:
        if self.bump_x_origin is None or self.bump_y_origin is None or self.bump_z_origin is None:
            if engine is None:
                raise ValueError("Bump origin is not defined and no engine is available to resolve it.")
            self.bump_x_origin, self.bump_y_origin, self.bump_z_origin = tuple(engine.sim_instance.problem_position)
        return (self.bump_x_origin, self.bump_y_origin, self.bump_z_origin)

    def bump_grid_dimensions(self) -> tuple[int, int]:
        """Returns configured grid dimensions."""
        return tuple(self.grid_dimensions)

    def bump_distributed_positions(self, engine:Engine | None = None):
        """Returns a deterministic grid of bump positions around the problem origin."""
        if self._bump_positions is None:
            if engine is None:
                raise ValueError("Cannot compute bump positions without engine context.")

            origin_x, origin_y, origin_z = self.bump_origin(engine)
            x_count, y_count = self.bump_grid_dimensions()
            num_positions = x_count * y_count
            x_spacing = self.bump_x_range / (x_count - 1) if x_count > 1 else 0.0
            y_spacing = self.bump_y_range / (y_count - 1) if y_count > 1 else 0.0
            x_start = origin_x - x_spacing * (x_count - 1) / 2
            y_start = origin_y - y_spacing * (y_count - 1) / 2
            positions = []
            for i in range(x_count):
                for j in range(y_count):
                    positions.append(
                        (
                            x_start + i * x_spacing,
                            y_start + j * y_spacing,
                            origin_z,
                        )
                    )
            self._bump_positions = positions

        return self._bump_positions

    def bump_position_for_repetition(self, engine:Engine):
        """Selects a bump position based on current repetition index."""
        if engine is None:
            raise ValueError("Engine context is required to select bump position for repetition.")
        positions = self.bump_distributed_positions(engine)
        repetition = getattr(engine.sim_instance, 'current_repetition', 0)
        # Wrap repetition index using modulo
        position_index = repetition % len(positions) if positions else 0
        return positions[position_index]

    def apply(self, engine:Engine):
        x, y, z = self.bump_position_for_repetition(engine)
        engine.instantiate_body(self.bump_file, [x, y, z])
    
    def apply_compute(self, engine:Engine):
        pass
        
    def get_generated_instances(self):
        return {
            "distributed_positions": self._bump_positions or [],
        }