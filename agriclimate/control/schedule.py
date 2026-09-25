"""Setpoint schedules used by agricultural climate recipes."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


class Schedule:
    """Base class: a callable that returns the setpoint (degC) at time t_s (s)."""
    def __call__(self, t_s: float) -> float:
        raise NotImplementedError


@dataclass
class ConstantSchedule(Schedule):
    """Fixed setpoint."""
    value: float

    def __call__(self, t_s: float) -> float:
        return self.value


@dataclass
class DayNightSchedule(Schedule):
    """Day/night temperature (DIF) with linear ramps - e.g. tomato 21/17 degC.
    Optional pre-dawn drop ("DROP") is a common height-control technique."""
    day: float
    night: float
    day_start_h: float = 6.0
    day_end_h: float = 18.0
    ramp_h: float = 1.5
    predawn_drop: float = 0.0
    predawn_h: float = 2.0

    def __call__(self, t_s: float) -> float:
        h = (t_s / 3600.0) % 24.0
        if self.day_start_h <= h < self.day_end_h:
            up = min(1.0, (h - self.day_start_h) / self.ramp_h) if self.ramp_h > 0 else 1.0
            value = self.night + (self.day - self.night) * up
            if self.predawn_drop and h < self.day_start_h + self.predawn_h:
                value -= self.predawn_drop
            return value
        since = (h - self.day_end_h) % 24.0
        down = min(1.0, since / self.ramp_h) if self.ramp_h > 0 else 1.0
        return self.day + (self.night - self.day) * down


@dataclass
class TableSchedule(Schedule):
    """Piecewise-linear (or step) profile from (hour, value) points."""
    points: Sequence[Tuple[float, float]]
    interpolate: bool = True
    repeat_h: float = 0.0          # >0: periodic profile

    def __call__(self, t_s: float) -> float:
        h = t_s / 3600.0
        if self.repeat_h > 0:
            h %= self.repeat_h
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        if self.interpolate:
            return float(np.interp(h, xs, ys))
        idx = max(0, np.searchsorted(xs, h, side="right") - 1)
        return float(ys[idx])


@dataclass
class BroodingSchedule(Schedule):
    """Broiler brooding curve: starts at ``start`` degC on day 0 and decreases
    by ``drop_per_day`` until ``final`` (typ. 32 -> 21 degC over ~4 weeks).
    ``start_day`` lets a short simulation start at a given flock age."""
    start: float = 32.0
    final: float = 21.0
    drop_per_day: float = 0.4
    start_day: float = 0.0

    def __call__(self, t_s: float) -> float:
        day = self.start_day + t_s / 86400.0
        return max(self.final, self.start - self.drop_per_day * day)


def schedule_from_config(cfg) -> Schedule:
    """Build a schedule from a number or from a ``{type: ...}`` YAML mapping."""
    if isinstance(cfg, (int, float)):
        return ConstantSchedule(float(cfg))
    cfg = dict(cfg)
    kind = cfg.pop("type")
    if kind == "constant":
        return ConstantSchedule(**cfg)
    if kind == "day_night":
        return DayNightSchedule(**cfg)
    if kind == "table":
        cfg["points"] = [tuple(p) for p in cfg["points"]]
        return TableSchedule(**cfg)
    if kind == "brooding":
        return BroodingSchedule(**cfg)
    raise ValueError(f"Unknown schedule type '{kind}'")
