"""The draws of a cause, given or recorded (docs_output/resultados/45_busqueda_con_refinado and 46).

The causes generated from templates (a push, a blocked wheel, a slippery floor) draw their numbers with the random
module, random.uniform(a, b) and random.random(), in an order that is the same in every repetition. The search in
rounds (search_rounds.py) needs those draws as points of the unit cube, one coordinate per call in call order, both
to know where a repetition was and to say where the next ones go. DrawControl takes the place of both functions in
the simulator's process: every call takes the next unit value of the current repetition, from the given rows if
there are any (0.5 past the end of a row) or from its own seeded generator, and keeps it with the width of the range
it was drawn in (0 for a value fixed by the cause, which the search leaves alone).
"""
from __future__ import annotations

import random
from typing import Optional

#: Keys of a repetition that carries its draws.
DRAWS = "draws"
DRAW_WIDTHS = "draw_widths"


class DrawControl:
    def __init__(self, given: Optional[list[list[float]]] = None, seed: Optional[int] = None):
        self.given = given
        self.rng = random.Random(seed)
        self.row: Optional[int] = None
        self.units: list[float] = []
        self.widths: list[float] = []

    def start(self, row: int) -> None:
        """A new repetition: row `row` of the given draws."""
        self.row = row
        self.units, self.widths = [], []

    def _unit(self, width: float) -> float:
        k = len(self.units)
        if self.given is not None:
            values = self.given[self.row] if self.row is not None and 0 <= self.row < len(self.given) else []
            unit = float(values[k]) if k < len(values) else 0.5
        else:
            unit = self.rng.random()
        self.units.append(unit)
        self.widths.append(float(width))
        return unit

    def uniform(self, a: float, b: float) -> float:
        return a + self._unit(b - a) * (b - a)

    def random(self) -> float:
        return self._unit(1.0)

    def install(self) -> None:
        """Every random.uniform and random.random of this process from now on."""
        random.uniform = self.uniform
        random.random = self.random
