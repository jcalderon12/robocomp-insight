"""The search of a batch's causes in rounds, with the importance weights the verdict needs
(docs_output/resultados/45_busqueda_con_refinado and 46_refinado_con_lineas).

Every cause but the nominal gets a budget B of repetitions (its own num_of_repetitions, or the scene's):
  * round 0, B - R * n draws from its prior: the scaled bump's own draw (its Latin hypercube, seeded per case), or
    the generated cause's draws with the random module (recorded by draws.DrawControl);
  * R rounds of n draws around the TOP best repetitions so far. Every draw is gaussian around a centre (WIDTHS[r] of
    its range) and truncated to its range, and a share LINE_SHARE of them sweep one draw over its whole range, so
    that a repetition right in some numbers and wrong in another can be put right (a bump in its place but of the
    wrong size). The bump is refined in metres (diameter, height, centre), so that a change of size does not move
    its centre; the others in the unit cube of their draws, along the draws that have a range.
Every repetition gets LOG_WEIGHT = log(prior density / the deterministic mixture of every round's proposal), which the
verdict adds to its kernel: drawing more where the best are does not count as evidence. With B = 4,000 the true
cause's best repetition comes much closer to the record than the prior alone gets, which is what lets the verdict's
adequacy rule tell a cause the batch lacks (resultados/46).

`search` drives the rounds through `launch(specs)`, which runs chunks of repetitions (the worker: causes_simulator
--channels processes) and returns their repetitions, so that it can be tested without a simulator.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Optional

import numpy as np

from src.draws import DRAW_WIDTHS, DRAWS
from src.verdict import DISTANCE, GENERATED_INSTANCES, LOG_WEIGHT

WIDTHS = (0.05, 0.025, 0.0125, 0.006)
PER_ROUND = 250
TOP = 10
LINE_SHARE = 0.5
#: Repetitions per process in round 0: small enough that no core idles while the last chunks finish.
FIRST_CHUNK = 150
BUMP = "bump_scaled"
NOMINAL = "none"


def plan(budget: int) -> tuple[int, int]:
    """(draws from the prior, draws per round) for a budget: R rounds of at most PER_ROUND, half the budget at most."""
    per_round = max(0, min(PER_ROUND, budget // (2 * len(WIDTHS))))
    return budget - len(WIDTHS) * per_round, per_round


# --------------------------------------------------------------------------- #
# the bump in metres: theta = (diameter, height, x, y), the order of its unit draws
# --------------------------------------------------------------------------- #
def bump_geometry(cause: dict) -> dict:
    bx = cause.get("bounds_x") or [-math.inf, math.inf]
    by = cause.get("bounds_y") or [-math.inf, math.inf]
    d0, d1 = cause["diameter_range"]
    h0, h1 = cause["height_range"]
    ax, ay = cause["area_x"], cause["area_y"]
    return {"ax": ax, "ay": ay, "bx": bx, "by": by, "d": (d0, d1), "h": (h0, h1),
            "lows": np.array([d0, h0, max(bx[0], ax[0] - d1 / 2), max(by[0], ay[0] - d1 / 2)]),
            "highs": np.array([d1, h1, min(bx[1], ax[1] + d1 / 2), min(by[1], ay[1] + d1 / 2)])}


def bump_log_prior(geo: dict, theta: np.ndarray) -> np.ndarray:
    """log density of cause_bump_scaled's draw at theta (n x 4): diameter and height uniform, the centre uniform
    over the area dilated by half the diameter (clipped to the bounds)."""
    d, h, x, y = theta.T
    lx = np.maximum(geo["bx"][0], geo["ax"][0] - d / 2)
    hx = np.minimum(geo["bx"][1], geo["ax"][1] + d / 2)
    ly = np.maximum(geo["by"][0], geo["ay"][0] - d / 2)
    hy = np.minimum(geo["by"][1], geo["ay"][1] + d / 2)
    tol = 1e-4   # the draws are rounded to 0.1 mm
    inside = ((d >= geo["d"][0] - tol) & (d <= geo["d"][1] + tol) & (h >= geo["h"][0] - tol) & (h <= geo["h"][1] + tol)
              & (x >= lx - tol) & (x <= hx + tol) & (y >= ly - tol) & (y <= hy + tol))
    with np.errstate(divide="ignore", invalid="ignore"):
        volume = (geo["d"][1] - geo["d"][0]) * (geo["h"][1] - geo["h"][0]) * (hx - lx) * (hy - ly)
        return np.where(inside, -np.log(volume), -np.inf)


def theta_of(rep: dict) -> list[float]:
    sample = (rep.get(GENERATED_INSTANCES) or {}).get("sample") or {}
    return [sample.get("diameter_m", math.nan), sample.get("height_m", math.nan), sample.get("x_m", math.nan),
            sample.get("y_m", math.nan)]


def samples_of(theta: np.ndarray) -> list[dict]:
    return [{"diameter_m": round(float(t[0]), 4), "height_m": round(float(t[1]), 4),
             "x_m": round(float(t[2]), 4), "y_m": round(float(t[3]), 4)} for t in theta]


# --------------------------------------------------------------------------- #
# the proposals of the rounds and their densities
# --------------------------------------------------------------------------- #
_erf = np.vectorize(math.erf)


def _Phi(z):
    return 0.5 * (1.0 + _erf(np.asarray(z) / math.sqrt(2.0)))


def _truncated_normal(rng, centre: float, sigma: float, low: float, high: float) -> float:
    while True:
        value = rng.normal(centre, sigma)
        if low <= value <= high:
            return value


def propose(rng, centres: np.ndarray, lows: np.ndarray, highs: np.ndarray, width: float, n: int) -> np.ndarray:
    """n draws around the centres, taken in turn: each coordinate gaussian (width x its range) truncated to its range;
    with probability LINE_SHARE one coordinate, chosen at random, uniform over its whole range instead."""
    scales = width * (highs - lows)
    out = np.empty((n, len(lows)))
    for i in range(n):
        centre = centres[i % len(centres)]
        for j in range(len(lows)):
            out[i, j] = _truncated_normal(rng, centre[j], scales[j], lows[j], highs[j])
        if rng.random() < LINE_SHARE:
            j = int(rng.integers(len(lows)))
            out[i, j] = rng.uniform(lows[j], highs[j])
    return out


def proposal_log_density(proposal: dict, values: np.ndarray) -> np.ndarray:
    """log density of `values` (n x a) under a round of `propose`."""
    lows, highs, centres = proposal["lows"], proposal["highs"], proposal["centres"]
    if values.shape[1] == 0:
        return np.zeros(len(values))
    scales = proposal["width"] * (highs - lows)
    v, c = values[:, None, :], centres[None, :, :]
    z = (v - c) / scales
    norm = _Phi((highs - c) / scales) - _Phi((lows - c) / scales)
    log_tn = -0.5 * z * z - np.log(scales * math.sqrt(2 * math.pi)) - np.log(np.maximum(norm, 1e-300))   # n x m x a
    local = log_tn.sum(axis=2)
    swept = local[:, :, None] - log_tn - np.log(highs - lows)[None, None, :]
    line = np.logaddexp.reduce(swept, axis=2) - math.log(len(lows))
    per_centre = np.logaddexp(math.log(1 - LINE_SHARE) + local, math.log(LINE_SHARE) + line)
    out = np.logaddexp.reduce(per_centre, axis=1) - math.log(len(centres))
    inside = ((values >= lows - 1e-9) & (values <= highs + 1e-9)).all(axis=1)
    return np.where(inside, out, -np.inf)


# --------------------------------------------------------------------------- #
# one cause
# --------------------------------------------------------------------------- #
class CauseSearch:
    def __init__(self, index: int, cause: dict, budget: int):
        self.index, self.cause = index, cause
        self.kind = "nominal" if cause["name"] == NOMINAL else ("bump" if cause["name"] == BUMP else "draws")
        self.n0, self.per_round = (1, 0) if self.kind == "nominal" else plan(budget)
        self.reps: list[dict] = []
        self.values: list[list[float]] = []    # theta (bump) or every unit draw (others), per repetition
        self.proposals: list[dict] = []
        self.active: Optional[list[int]] = list(range(4)) if self.kind == "bump" else None
        self.geo = bump_geometry(cause) if self.kind == "bump" else None

    def absorb(self, reps: list[dict]) -> None:
        for rep in reps:
            self.reps.append(rep)
            if self.kind == "bump":
                self.values.append(theta_of(rep))
            elif self.kind == "draws":
                self.values.append(list(rep.get(DRAWS) or []))
                if self.active is None and rep.get(DRAW_WIDTHS) is not None:
                    self.active = [j for j, w in enumerate(rep[DRAW_WIDTHS]) if w > 0]

    def _matrix(self) -> np.ndarray:
        """Every repetition's values, padded with 0.5 (a draw a repetition did not make)."""
        width = max((len(v) for v in self.values), default=0)
        return np.array([v + [0.5] * (width - len(v)) for v in self.values], dtype=float).reshape(len(self.values), width)

    def _bounds(self) -> tuple[np.ndarray, np.ndarray]:
        if self.kind == "bump":
            return self.geo["lows"], self.geo["highs"]
        return np.zeros(len(self.active or [])), np.ones(len(self.active or []))

    def distances(self) -> np.ndarray:
        return np.array([math.inf if r.get(DISTANCE) is None else float(r[DISTANCE]) for r in self.reps])

    def next_round(self, rng, width: float) -> Optional[dict]:
        """The draws of the next round: {"theta"} for a bump, {"rows"} of unit draws otherwise."""
        if self.kind == "nominal" or self.per_round == 0:
            return None
        d = self.distances()
        order = np.argsort(d)[:TOP]
        lows, highs = self._bounds()
        matrix = self._matrix()
        finite = np.isfinite(d[order]).any() and bool(self.active)
        if self.kind == "bump":
            if not finite:
                self.proposals.append({"kind": "prior", "n": self.per_round})
                return {"prior": True}
            centres = matrix[order]
            theta = propose(rng, centres, lows, highs, width, self.per_round)
            self.proposals.append({"kind": "line", "centres": centres, "lows": lows, "highs": highs, "width": width,
                                   "n": self.per_round})
            return {"theta": theta}
        if not finite:
            active_values = rng.random((self.per_round, len(self.active or [])))
            self.proposals.append({"kind": "prior", "n": self.per_round})
            base = np.full(matrix.shape[1] if matrix.size else 0, 0.5)
        else:
            centres = matrix[order][:, self.active]
            active_values = propose(rng, centres, lows, highs, width, self.per_round)
            self.proposals.append({"kind": "line", "centres": centres, "lows": lows, "highs": highs, "width": width,
                                   "n": self.per_round})
            base = matrix[order[0]]
        rows = np.tile(base, (self.per_round, 1))
        if self.active:
            rows[:, self.active] = active_values
        return {"rows": rows}

    def log_weights(self) -> np.ndarray:
        n = len(self.reps)
        if self.kind == "nominal" or not self.proposals:
            return np.zeros(n)
        matrix = self._matrix()
        if self.kind == "bump":
            values = matrix
            log_prior = bump_log_prior(self.geo, values)
        else:
            values = matrix[:, self.active or []]
            log_prior = np.zeros(n)
        terms = [math.log(self.n0 / n) + log_prior]
        for proposal in self.proposals:
            density = log_prior if proposal["kind"] == "prior" else proposal_log_density(proposal, values)
            terms.append(math.log(proposal["n"] / n) + density)
        return log_prior - np.logaddexp.reduce(np.vstack(terms), axis=0)


def _cost_rank(cause: dict) -> int:
    """Scheduling only: the costliest chunks first (a bump and a push on the bottle are seldom stopped early)."""
    if cause["name"] == BUMP:
        return 0
    if cause["name"] == "external_force":
        return 1 if cause.get("force_target") == "bottle" else 3
    return 2


# --------------------------------------------------------------------------- #
# the rounds
# --------------------------------------------------------------------------- #
def search(causes: list[dict], budget_of: Callable[[dict], int], launch: Callable[[list[dict]], list[list[dict]]],
           processes: int, seed: int, log: Callable[[str], None] = print) -> list[list[dict]]:
    """Every cause's repetitions, in the order of `causes`, each with LOG_WEIGHT. `launch(specs)` runs chunks of
    repetitions and returns, per spec and in order, its repetitions with their DISTANCE (and DRAWS for a generated
    cause). A spec is {"index", "cause", "first", "count", "draws" (unit rows or None), "seed" (or None)}."""
    rng = np.random.default_rng(seed)
    states = [CauseSearch(i, cause, 1 if cause["name"] == NOMINAL else int(budget_of(cause)))
              for i, cause in enumerate(causes)]

    def run(specs: list[dict], label: str) -> None:
        started = time.time()
        results = launch(specs)
        for spec, reps in sorted(zip(specs, results), key=lambda sr: (sr[0]["index"], sr[0].get("order", sr[0]["first"]))):
            states[spec["index"]].absorb(reps)
        log(f"Search {label}: {len(specs)} processes, {sum(len(r) for r in results)} repetitions in "
            f"{time.time() - started:.1f} s.")

    specs = []
    for st in states:
        if st.kind == "nominal":
            specs.append({"index": st.index, "cause": st.cause, "first": 0, "count": 1, "draws": None, "seed": None})
            continue
        cause = {k: v for k, v in st.cause.items() if k != "samples"}
        cause["num_of_repetitions"] = st.n0
        if st.kind == "bump":
            cause["seed"] = seed
        for chunk, first in enumerate(range(0, st.n0, FIRST_CHUNK)):
            specs.append({"index": st.index, "cause": cause, "first": first, "count": min(FIRST_CHUNK, st.n0 - first),
                          "draws": None, "seed": None if st.kind == "bump" else seed + 1000 * st.index + chunk})
    specs.sort(key=lambda s: _cost_rank(s["cause"]))
    run(specs, "round 0 (prior)")

    searchable = [st for st in states if st.kind != "nominal" and st.per_round]
    for r, width in enumerate(WIDTHS, start=1):
        specs = []
        for st in searchable:
            chunk = math.ceil(st.per_round / max(1, processes // max(1, len(searchable))))
            proposal = st.next_round(rng, width)
            if st.kind == "bump" and proposal.get("prior"):
                cause = {k: v for k, v in st.cause.items() if k != "samples"}
                cause.update(num_of_repetitions=st.per_round, seed=seed + 7919 * r)
                specs += [{"index": st.index, "cause": cause, "first": f, "count": min(chunk, st.per_round - f),
                           "draws": None, "seed": None, "round": r} for f in range(0, st.per_round, chunk)]
            elif st.kind == "bump":
                samples = samples_of(proposal["theta"])
                for f in range(0, st.per_round, chunk):
                    part = samples[f:f + chunk]
                    specs.append({"index": st.index, "cause": {**st.cause, "samples": part, "num_of_repetitions": len(part)},
                                  "first": 0, "count": len(part), "draws": None, "seed": None, "round": r})
            else:
                rows = proposal["rows"].tolist()
                for f in range(0, st.per_round, chunk):
                    part = rows[f:f + chunk]
                    specs.append({"index": st.index, "cause": {**st.cause, "num_of_repetitions": len(part)},
                                  "first": 0, "count": len(part), "draws": part, "seed": None, "round": r})
        # within a round, a cause's chunks are absorbed in the order they were drawn
        offset = {}
        for spec in specs:
            spec["order"] = offset.setdefault(spec["index"], 0)
            offset[spec["index"]] += spec["count"]
        run(specs, f"round {r} (width {width})")

    out = []
    for st in states:
        weights = st.log_weights()
        for rep, weight in zip(st.reps, weights):
            rep[LOG_WEIGHT] = float(weight) if np.isfinite(weight) else None
        out.append(st.reps)
    return out
