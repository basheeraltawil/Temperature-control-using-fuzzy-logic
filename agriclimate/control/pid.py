"""Industrial PID baseline: derivative on measurement with first-order filter,
set-point weighting and back-calculation anti-windup."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .base import ControlContext, Controller


@dataclass
class PIDController(Controller):
    """u = Kp(beta r - y) + (Kp/Ti) int e dt - Kp Td dy_f/dt, with back-calculation anti-windup."""
    kp: float = 0.35          # 1/K
    ti: float = 770.0         # s
    td: float = 0.0           # s (tuning found no benefit from D)
    n_filter: float = 8.0     # derivative filter: Td/N
    beta: float = 1.0         # set-point weight on the P term
    tt: float = 300.0         # anti-windup tracking time constant, s
    name: str = "pid"

    def __post_init__(self):
        self.reset()

    def reset(self, demand: float = 0.0) -> None:
        """Restart with the integrator set to the given demand (bumpless transfer)."""
        self._i = demand
        self._d = 0.0
        self._y_prev = None

    def update(self, ctx: ControlContext) -> float:
        """One discrete PID step; returns the demand clipped to [-1, 1]."""
        y, r, dt = ctx.temperature, ctx.setpoint, ctx.dt
        if self._y_prev is None:
            self._y_prev = y
        p = self.kp * (self.beta * r - y)
        tf = self.td / self.n_filter
        self._d = (tf * self._d - self.kp * self.td * (y - self._y_prev)) / (tf + dt) if self.td > 0 else 0.0
        self._y_prev = y
        v = p + self._i + self._d
        u = float(np.clip(v, -1.0, 1.0))
        if self.ti > 0:
            self._i += dt * (self.kp / self.ti * (r - y) + (u - v) / self.tt)
        return u
