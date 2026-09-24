"""Safety supervisor: the deterministic layer that sits between any controller
(fuzzy, PID, MPC, AI) and the actuators.

Nothing produced by a learning or language model reaches the hardware without
passing through here.  In a real installation the same checks run in the PLC,
and an *independent hard-wired* high-limit thermostat is still mandatory.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set

import numpy as np

from ..plant.facility import ActuatorCommand


@dataclass
class Alarm:
    t_s: float
    level: str        # "INFO" | "WARN" | "ALARM"
    code: str
    message: str

    def as_dict(self) -> Dict:
        return {"t_h": round(self.t_s / 3600.0, 3), "level": self.level, "code": self.code,
                "message": self.message}


@dataclass
class SupervisorConfig:
    sensor_min: float = -40.0
    sensor_max: float = 70.0
    max_rate_c_per_s: float = 0.05       # physically implausible faster changes
    disagreement_tol: float = 1.5        # K between redundant sensors
    stale_timeout_s: float = 120.0
    t_low_limit: float = 2.0             # frost protection
    t_high_limit: float = 38.0           # crop/animal heat stress
    limit_hysteresis: float = 1.0        # K, avoids alarm chatter and actuator hunting
    heat_cool_interlock: bool = True
    max_slew_per_s: float = 0.02         # 0 -> 100 % in 50 s
    compressor_min_on_s: float = 0.0     # anti short-cycle for refrigeration
    compressor_min_off_s: float = 0.0
    failsafe_setpoint: float = 12.0
    failsafe_min_vent: float = 0.05


@dataclass
class SupervisorOutput:
    command: ActuatorCommand
    temperature: float                   # validated / fused temperature (nan if none)
    quality: str                         # "GOOD" | "DEGRADED" | "BAD"
    mode: str                            # "AUTO" | "FAILSAFE" | "HIGH_LIMIT" | "LOW_LIMIT"
    alarms: List[Alarm]


class SafetySupervisor:
    def __init__(self, config: Optional[SupervisorConfig] = None):
        self.cfg = config or SupervisorConfig()
        self._last_valid: Optional[float] = None
        self._last_valid_t = -math.inf
        self._prev_readings: List[Optional[float]] = []
        self._prev_cmd = ActuatorCommand()
        self._compressor_on = False
        self._compressor_changed_t = -math.inf
        self._active: Set[str] = set()

    # --------------------------------------------------------------- sensors
    def validate(self, readings: Sequence[float], t_s: float, dt: float,
                 excluded: Sequence[int] = ()) -> (float, str, List[Alarm]):
        c, alarms = self.cfg, []
        valid = []
        if len(self._prev_readings) != len(readings):
            self._prev_readings = [None] * len(readings)
        for i, r in enumerate(readings):
            ok = r is not None and not math.isnan(r) and c.sensor_min <= r <= c.sensor_max
            prev = self._prev_readings[i]
            if ok and prev is not None and abs(r - prev) / dt > c.max_rate_c_per_s:
                ok = False
                alarms += self._raise(t_s, "WARN", f"SENSOR{i + 1}_RATE", f"sensor {i + 1} implausible rate")
            self._prev_readings[i] = r if r is not None and not math.isnan(r) else None
            if ok and i in excluded:
                alarms += self._raise(t_s, "WARN", f"SENSOR{i + 1}_ANOMALY",
                                      f"sensor {i + 1} excluded by anomaly detector")
                continue
            self._clear(f"SENSOR{i + 1}_ANOMALY")
            if ok:
                valid.append(r)
                for suffix in ("INVALID", "RATE"):
                    self._clear(f"SENSOR{i + 1}_{suffix}")
            else:
                alarms += self._raise(t_s, "WARN", f"SENSOR{i + 1}_INVALID", f"sensor {i + 1} invalid ({r})")
        if len(valid) >= 2:
            spread = max(valid) - min(valid)
            if spread > c.disagreement_tol or ("SENSOR_DISAGREE" in self._active
                                                and spread > 0.5 * c.disagreement_tol):
                alarms += self._raise(t_s, "WARN", "SENSOR_DISAGREE",
                                      f"redundant sensors disagree by {spread:.1f} K")
                # keep the reading closest to the last trusted value
                ref = self._last_valid if self._last_valid is not None else float(np.median(valid))
                value, quality = min(valid, key=lambda v: abs(v - ref)), "DEGRADED"
            else:
                self._clear("SENSOR_DISAGREE")
                value, quality = float(np.mean(valid)), "GOOD"
        elif len(valid) == 1:
            value, quality = valid[0], ("DEGRADED" if len(readings) > 1 else "GOOD")
        else:
            if t_s - self._last_valid_t <= c.stale_timeout_s and self._last_valid is not None:
                return self._last_valid, "DEGRADED", alarms
            alarms += self._raise(t_s, "ALARM", "NO_VALID_SENSOR", "no valid temperature measurement")
            return float("nan"), "BAD", alarms
        self._clear("NO_VALID_SENSOR")
        self._last_valid, self._last_valid_t = value, t_s
        return value, quality, alarms

    # -------------------------------------------------------------- actuators
    def apply(self, requested: ActuatorCommand, temperature: float, quality: str, t_s: float,
              dt: float, t_out: float = 10.0) -> SupervisorOutput:
        c, alarms = self.cfg, []
        cmd = requested.clipped()
        mode = "AUTO"
        if any(math.isnan(v) for v in (cmd.heater, cmd.cooler, cmd.vent)):
            cmd = ActuatorCommand()
            quality = "BAD"
        if quality == "BAD":
            mode = "FAILSAFE"
            alarms += self._raise(t_s, "ALARM", "FAILSAFE", "fail-safe mode: open-loop frost protection")
            heat = float(np.clip((c.failsafe_setpoint - t_out) / 20.0, 0.0, 1.0))
            cmd = ActuatorCommand(heater=heat, cooler=0.0, vent=c.failsafe_min_vent)
        else:
            self._clear("FAILSAFE")
            # latch the limit modes until the temperature has recovered by the hysteresis
            low = temperature <= c.t_low_limit or (
                "LOW_TEMP" in self._active and temperature < c.t_low_limit + c.limit_hysteresis)
            high = temperature >= c.t_high_limit or (
                "HIGH_TEMP" in self._active and temperature > c.t_high_limit - c.limit_hysteresis)
            if low:
                mode = "LOW_LIMIT"
                alarms += self._raise(t_s, "ALARM", "LOW_TEMP", f"low temperature {temperature:.1f} degC")
                cmd = ActuatorCommand(heater=1.0, cooler=0.0, vent=0.0)
            elif high:
                mode = "HIGH_LIMIT"
                alarms += self._raise(t_s, "ALARM", "HIGH_TEMP", f"high temperature {temperature:.1f} degC")
                cmd = ActuatorCommand(heater=0.0, cooler=1.0, vent=1.0)
            else:
                self._clear("LOW_TEMP")
                self._clear("HIGH_TEMP")
        if c.heat_cool_interlock and cmd.heater > 0 and cmd.cooler > 0:
            if cmd.heater >= cmd.cooler:
                cmd.cooler = 0.0
            else:
                cmd.heater = 0.0
        # slew-rate limit (protects burners, valves, compressors and the grid);
        # limits and fail-safe transitions bypass it for fast reaction
        if mode == "AUTO" and c.max_slew_per_s > 0:
            lim = c.max_slew_per_s * dt
            p = self._prev_cmd
            cmd = ActuatorCommand(p.heater + np.clip(cmd.heater - p.heater, -lim, lim),
                                  p.cooler + np.clip(cmd.cooler - p.cooler, -lim, lim),
                                  p.vent + np.clip(cmd.vent - p.vent, -lim, lim)).clipped()
        cmd = self._anti_short_cycle(cmd, t_s)
        self._prev_cmd = cmd
        return SupervisorOutput(cmd, temperature, quality, mode, alarms)

    def _anti_short_cycle(self, cmd: ActuatorCommand, t_s: float) -> ActuatorCommand:
        c = self.cfg
        if c.compressor_min_on_s <= 0 and c.compressor_min_off_s <= 0:
            return cmd
        want_on = cmd.cooler > 0.05
        elapsed = t_s - self._compressor_changed_t
        if want_on != self._compressor_on:
            limit = c.compressor_min_on_s if self._compressor_on else c.compressor_min_off_s
            if elapsed >= limit:
                self._compressor_on = want_on
                self._compressor_changed_t = t_s
        if not self._compressor_on:
            cmd.cooler = 0.0
        elif cmd.cooler < 0.05:
            cmd.cooler = 0.2        # minimum compressor load while forced on
        return cmd

    # --------------------------------------------------------------- alarms
    def raise_external(self, t_s: float, level: str, code: str, message: str) -> List[Alarm]:
        """Alarms reported by other components (e.g. the AI anomaly monitor)."""
        return self._raise(t_s, level, code, message)

    def _raise(self, t_s, level, code, message) -> List[Alarm]:
        if code in self._active:
            return []                       # alarm already active: no flooding
        self._active.add(code)
        return [Alarm(t_s, level, code, message)]

    def _clear(self, code: str) -> None:
        self._active.discard(code)

    @property
    def active_alarms(self) -> Set[str]:
        return set(self._active)
