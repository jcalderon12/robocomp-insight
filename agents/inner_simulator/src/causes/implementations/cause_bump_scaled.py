"""
A dome-shaped bump of unknown size (catalog intervention `spawn_scaled_dome`).

Each repetition draws the diameter and height of the dome within the given ranges, and its centre
within the area dilated by half the drawn diameter, so that the dome reaches the area with its edge.
A Latin hypercube spreads the draws over the four; the seed fixes them. The dome is the catalog mesh
scaled to the drawn size. An explicit list of samples can replace the draw (offline studies split
the repetitions across processes).
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal, Optional

import numpy as np

# Spawned objects rest on the floor, as the compiler places the fixed bumps (1 mm).
FLOOR_Z_M = 0.001


def latin_hypercube(n: int, dims: int, rng: np.random.Generator) -> np.ndarray:
    """n points in [0, 1)^dims, one per stratum in every dimension."""
    points = np.empty((n, dims))
    for dim in range(dims):
        points[:, dim] = (rng.permutation(n) + rng.random(n)) / n
    return points


def draw_samples(area_x, area_y, diameter_range, height_range, n, seed, bounds_x=None, bounds_y=None):
    """Diameter, height and centre (meters) of each repetition. The centre box is the area dilated by
    half the drawn diameter, clipped to the bounds."""
    bounds = {"x": bounds_x or [-np.inf, np.inf], "y": bounds_y or [-np.inf, np.inf]}
    area = {"x": area_x, "y": area_y}
    samples = []
    for u_d, u_h, u_x, u_y in latin_hypercube(n, 4, np.random.default_rng(seed)):
        diameter = diameter_range[0] + u_d * (diameter_range[1] - diameter_range[0])
        height = height_range[0] + u_h * (height_range[1] - height_range[0])
        centre = {}
        for axis, u in (("x", u_x), ("y", u_y)):
            low = max(area[axis][0] - diameter / 2.0, bounds[axis][0])
            high = min(area[axis][1] + diameter / 2.0, bounds[axis][1])
            centre[axis] = low + u * (high - low)
        samples.append({"diameter_m": round(float(diameter), 4), "height_m": round(float(height), 4),
                        "x_m": round(float(centre["x"]), 4), "y_m": round(float(centre["y"]), 4)})
    return samples


class CauseBumpScaled(BaseModel, Cause):
    """A bump of unknown size in the way."""

    name: Literal["bump_scaled"]
    mesh_file: str
    mesh_dimensions_m: list[float]
    area_x: list[float]
    area_y: list[float]
    diameter_range: list[float]
    height_range: list[float]
    bounds_x: Optional[list[float]] = None
    bounds_y: Optional[list[float]] = None
    num_of_repetitions: int = 36
    seed: int = 0
    samples: Optional[list[dict[str, float]]] = None
    _chosen: Optional[dict[str, float]] = PrivateAttr(default=None)

    def model_post_init(self, __context) -> None:
        if self.samples is None:
            self.samples = draw_samples(self.area_x, self.area_y, self.diameter_range, self.height_range,
                                        self.num_of_repetitions, self.seed, self.bounds_x, self.bounds_y)
        self.num_of_repetitions = len(self.samples)

    def apply(self, engine: Engine):
        simulator = engine.sim_instance
        sample = self.samples[getattr(simulator, "current_repetition", 0) % len(self.samples)]
        self._chosen = sample
        p = simulator.get_pybullet_instance()
        width, depth, height = self.mesh_dimensions_m
        scale = [sample["diameter_m"] / width, sample["diameter_m"] / depth, sample["height_m"] / height]
        collision = p.createCollisionShape(p.GEOM_MESH, fileName=self.mesh_file, meshScale=scale)
        visual = p.createVisualShape(p.GEOM_MESH, fileName=self.mesh_file, meshScale=scale)
        body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
                                 basePosition=[sample["x_m"], sample["y_m"], FLOOR_Z_M])
        simulator.loaded_bodies.append(body)

    def apply_compute(self, engine: Engine):
        pass

    def get_generated_instances(self):
        return {"sample": self._chosen}
