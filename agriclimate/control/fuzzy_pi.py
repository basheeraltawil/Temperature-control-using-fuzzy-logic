"""Incremental fuzzy-PI controller (setpoint independent, anti-windup, bumpless).

    e   = clip(Ke * (r - y))                      normalised error
    de  = clip(Kr * d(r - y)/dt)                  normalised (filtered) error rate
    du  = FIS(e, de)                              7x7 Mac Vicar-Whelan rule base
    u  += Ku * du * dt,  u in [-1, 1]             integration with clamping

Around the origin it behaves like a PI controller with Kp ~ Ku*Kr and
Ki ~ Ku*Ke, while the fuzzy surface adds gain scheduling for large errors.
Default gains were found with ``agriclimate.ai.tuner`` (differential evolution
on the digital twin); re-tune per facility.
``use_lut=True`` evaluates the pre-computed surface exactly as the generated
PLC / MCU code does (see ``agriclimate.fuzzy.codegen``).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..fuzzy.codegen import build_lut, interp_lut
from ..fuzzy.rulebases import fuzzy_pi_rulebase
from .base import ControlContext, Controller


@dataclass
class FuzzyPIController(Controller):
    """Incremental fuzzy-PI; gains ke (1/K), kr (s/K), ku (1/s)."""
    ke: float = 0.3           # 1/K    : |error| >= 3.3 K saturates the error input
    kr: float = 200.0         # s/K    : error-rate scaling
    ku: float = 0.0015        # 1/s    : output increment scaling
    use_lut: bool = True
    lut_size: int = 41
    rate_filter_s: float = 30.0
    name: str = "fuzzy_pi"

    def __post_init__(self):
        self.fis = fuzzy_pi_rulebase()
        if self.use_lut:
            self.lut = build_lut(lambda x, y: self.fis(x, y)[0], (-1, 1), (-1, 1),
                                 self.lut_size, self.lut_size)
        self.reset()

    def reset(self, demand: float = 0.0) -> None:
        """Restart from a given demand (bumpless transfer)."""
        self.u = float(demand)
        self._e_prev = None
        self._de_f = 0.0

    def surface(self, e: float, de: float) -> float:
        """Normalised output increment du for normalised error e and error rate de."""
        if self.use_lut:
            return interp_lut(*self.lut, e, de)
        return float(self.fis(e, de)[0])

    def update(self, ctx: ControlContext) -> float:
        """One control step: filter the error rate, evaluate the rule base, integrate."""
        err = ctx.setpoint - ctx.temperature
        if self._e_prev is None:
            self._e_prev = err
        raw_rate = (err - self._e_prev) / ctx.dt
        self._e_prev = err
        a = ctx.dt / (self.rate_filter_s + ctx.dt)       # filter sensor noise on the rate
        self._de_f += a * (raw_rate - self._de_f)
        e_n = float(np.clip(self.ke * err, -1, 1))
        de_n = float(np.clip(self.kr * self._de_f, -1, 1))
        du = self.surface(e_n, de_n)
        self.u = float(np.clip(self.u + self.ku * du * ctx.dt, -1.0, 1.0))
        return self.u
