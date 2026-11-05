
"""
A slippery patch of floor: the floor's friction changes while the robot crosses it.

The robot drove normally before the anomaly (the recording says so), so a slippery floor can only be a
patch where the robot was during the anomaly interval. Each repetition draws the friction coefficient
within its range; the floor has it inside the activation window (the blueprint's, or the scene's
anomaly interval when the blueprint gives none) and Webots' default friction, 1, outside it.
docs_output/diseno/veredicto.md, 3.3.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

#: The floor's friction outside the patch: Webots' default contact (causes_simulator.WEBOTS_FRICTION).
FLOOR_FRICTION = 1.0


class CauseFriction(BaseModel, Cause):
    """A slippery patch of floor where the robot was during the anomaly interval."""

    name:Literal["friction"]
    num_of_repetitions:int | None = None

    friction_min:float
    friction_max:float
    friction_target:str
    friction_window_start:float | None = None
    friction_window_end:float | None = None
    _friction_random_uniform_range:float | None = PrivateAttr(default=None)
    _window:tuple | None = PrivateAttr(default=None)
    _slippery:bool | None = PrivateAttr(default=None)

    def friction_random_uniform_range(self):
        """Random float uniformly sampled in [min, max], cached per cause instance."""
        import random
        if self._friction_random_uniform_range is None:
            self._friction_random_uniform_range = random.uniform(self.friction_min, self.friction_max)
        return self._friction_random_uniform_range

    def _active_window(self, engine:Engine):
        if self._window is None:
            length = engine.get_simulation_length()
            if self.friction_window_start is not None and self.friction_window_end is not None:
                self._window = (self.friction_window_start * length, self.friction_window_end * length)
            else:
                interval = getattr(engine.sim_instance.simulation_scene, "anomaly_interval", None)
                self._window = tuple(interval) if interval else (0.0, length)
        return self._window

    def apply(self, engine:Engine):
        self._slippery = None
        engine.set_lateral_friction(self.friction_target, FLOOR_FRICTION)

    def apply_compute(self, engine:Engine):
        # [friction] The patch: slippery inside the window, the floor's own friction outside it
        start, end = self._active_window(engine)
        slippery = start <= engine.get_simulation_time() <= end
        if slippery != self._slippery:
            engine.set_lateral_friction(self.friction_target,
                                        self.friction_random_uniform_range() if slippery else FLOOR_FRICTION)
            self._slippery = slippery

    def get_generated_instances(self):
        return {"random_uniform_range": [self.friction_random_uniform_range()]}
