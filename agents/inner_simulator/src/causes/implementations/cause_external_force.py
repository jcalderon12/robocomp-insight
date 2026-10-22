
"""
A push: a short force on a body (robot or bottle) at an instant of the activation window.

A push by a person lasts tenths of a second, and what the pushed body does depends on its impulse
(force times duration) more than on the force alone. Each repetition draws the force vector within
the ranges, a duration within force_duration_range_s, and the onset within the activation window so
that the whole push fits in it. On the robot, the force acts where a standing person's hand would
push it (PUSH_HEIGHT_M above the floor, on its axis); on the bottle, at its centre of mass.
docs_output/diseno/veredicto.md, 3.3.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

#: Height (m above the floor) at which a standing person's hands push the robot: its column.
PUSH_HEIGHT_M = 1.0


class CauseExternal_force(BaseModel, Cause):
    """A short push on a body (robot or bottle) at an instant of the activation window."""

    name:Literal["external_force"]
    num_of_repetitions:int | None = None

    force_x_range:float
    force_y_range:float
    force_z_range:float
    force_x_origin:float
    force_y_origin:float
    force_z_origin:float
    force_target:str
    force_window_start:float
    force_window_end:float
    force_duration_range_s:list[float] = [0.1, 0.3]
    _force:tuple | None = PrivateAttr(default=None)
    _duration:float | None = PrivateAttr(default=None)
    _onset_fraction:float | None = PrivateAttr(default=None)
    _onset:float | None = PrivateAttr(default=None)

    def force_random_range_coordinates(self):
        """The force vector (N, world frame) of this repetition, drawn once within the ranges."""
        import random
        if self._force is None:
            self._force = (self.force_x_origin + random.uniform(-self.force_x_range/2, self.force_x_range/2),
                           self.force_y_origin + random.uniform(-self.force_y_range/2, self.force_y_range/2),
                           self.force_z_origin + random.uniform(-self.force_z_range/2, self.force_z_range/2))
        return self._force

    def _draw_timing(self):
        import random
        if self._duration is None:
            low, high = self.force_duration_range_s
            self._duration = random.uniform(low, high)
            self._onset_fraction = random.random()

    def apply(self, engine:Engine):
        pass

    def apply_compute(self, engine:Engine):
        # [force] A short push inside the activation window
        self._draw_timing()
        sim_time = engine.get_simulation_time()
        if self._onset is None:
            sim_length = engine.get_simulation_length()
            start, end = self.force_window_start * sim_length, self.force_window_end * sim_length
            latest = max(start, end - self._duration)
            self._onset = start + self._onset_fraction * (latest - start)
        if self._onset <= sim_time < self._onset + self._duration:
            height = PUSH_HEIGHT_M if self.force_target == "robot" else None
            engine.apply_external_force(self.force_target, list(self.force_random_range_coordinates()), height)

    def get_generated_instances(self):
        self._draw_timing()
        return {
            "random_range_coordinates": [self.force_random_range_coordinates()],
            "duration_s": self._duration,
            "onset_s": self._onset,
        }
