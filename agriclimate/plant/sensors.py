"""Realistic sensor model: lag, noise, bias, quantisation and injectable faults."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class SensorFault:
    """Fault active between start_h and end_h (hours from the start of the run)."""
    kind: str            # "stuck" | "drift" | "offset" | "spike" | "dropout" | "noise"
    start_h: float
    end_h: float = float("inf")
    value: float = 0.0   # drift: degC/h, offset: degC, spike: amplitude, noise: extra std

    def active(self, t_h: float) -> bool:
        """True if the fault is active at t_h."""
        return self.start_h <= t_h < self.end_h


@dataclass
class Sensor:
    """First-order lag + bias + noise + quantisation, with optional faults."""
    name: str = "T1"
    noise_std: float = 0.05
    bias: float = 0.0
    resolution: float = 0.01
    tau_s: float = 30.0                  # radiation shield / probe time constant
    seed: int = 0
    faults: List[SensorFault] = field(default_factory=list)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)
        self._filtered: Optional[float] = None
        self._stuck_value: Optional[float] = None

    def read(self, true_value: float, t_s: float, dt: float) -> float:
        """Measured value for the true value at time t_s (NaN during a dropout)."""
        if self._filtered is None:
            self._filtered = true_value
        self._filtered += (true_value - self._filtered) * min(1.0, dt / max(self.tau_s, 1e-6))
        y = self._filtered + self.bias + self._rng.normal(0.0, self.noise_std)
        t_h = t_s / 3600.0
        stuck = False
        for f in self.faults:
            if not f.active(t_h):
                continue
            if f.kind == "stuck":
                stuck = True
            elif f.kind == "drift":
                y += f.value * (t_h - f.start_h)
            elif f.kind == "offset":
                y += f.value
            elif f.kind == "noise":
                y += self._rng.normal(0.0, f.value)
            elif f.kind == "spike" and self._rng.random() < 0.05:
                y += f.value * self._rng.choice([-1.0, 1.0])
            elif f.kind == "dropout":
                return float("nan")
        if stuck:
            if self._stuck_value is None:
                self._stuck_value = y
            return self._stuck_value
        self._stuck_value = None
        if self.resolution > 0:
            y = round(y / self.resolution) * self.resolution
        return float(y)
