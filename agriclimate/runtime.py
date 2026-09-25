"""One control cycle, shared by the simulator, the ROS 2 nodes and the
field (Modbus/MQTT) edge loop - so what is validated in simulation is
exactly what runs on the plant.

    readings -> [AI anomaly monitor] -> supervisor.validate -> controller
             -> split-range allocator -> supervisor.apply -> actuator command
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional

import numpy as np

from .control.allocator import SplitRangeAllocator
from .control.base import ControlContext, Controller
from .control.supervisor import Alarm, SafetySupervisor, SupervisorConfig
from .plant.facility import ActuatorCommand


@dataclass
class RuntimeOutput:
    """Result of one control cycle."""
    command: ActuatorCommand
    temperature: float
    quality: str
    mode: str
    demand: float
    excluded: List[int]
    alarms: List[Alarm]


class ClimateRuntime:
    """One control cycle shared by simulation, ROS 2 nodes and the edge loop."""
    def __init__(self, controller: Controller, allocator: Optional[SplitRangeAllocator] = None,
                 supervisor: Optional[SupervisorConfig] = None, supervised: bool = True, detector=None):
        self.controller = controller
        self.allocator = allocator or SplitRangeAllocator()
        self.supervisor = SafetySupervisor(supervisor)
        self.supervised = supervised
        self.detector = detector
        self.last_cmd = ActuatorCommand(vent=self.allocator.min_vent)
        self.demand = 0.0
        self.excluded: List[int] = []
        self._was_bad = False
        self._last_raw: Optional[float] = None

    def step(self, t_s: float, dt: float, readings: List[float], setpoint: float, rh: float,
             t_out: float, rh_out: float = 60.0, solar: float = 0.0, forecast=None,
             setpoint_preview: Optional[Callable[[float], float]] = None) -> RuntimeOutput:
        """Run one cycle: monitor, validate, control, allocate, supervise."""
        alarms: List[Alarm] = []
        sup = self.supervisor
        if self.detector is not None:
            c = self.last_cmd
            rep = self.detector.update(t_s, readings, c.heater, c.cooler, c.vent, t_out, rh_out, solar)
            if rep is not None:
                self.excluded = rep.suspect_sensors
                if rep.actuator_fault == "HEATER_UNDERPERFORMING":
                    alarms += sup.raise_external(t_s, "WARN", rep.actuator_fault,
                                                 "AI: heater delivers less heat than the model expects")
                elif rep.actuator_fault == "COMMON_MODE_DEVIATION":
                    alarms += sup.raise_external(t_s, "INFO", rep.actuator_fault,
                                                 "AI: all sensors deviate from the model - check actuators and doors")
        t_valid, quality, new = sup.validate(readings, t_s, dt, self.excluded)
        alarms += new

        if not self.supervised:
            # legacy mode: first sensor, direct outputs, no interlocks (as originally designed)
            raw = readings[0] if readings and np.isfinite(readings[0]) else self._last_raw
            raw = setpoint if raw is None else raw
            self._last_raw = raw
            out = self.controller.update(ControlContext(t_s, dt, raw, setpoint, rh, t_out, solar))
            cmd = out if isinstance(out, ActuatorCommand) else self.allocator.allocate(out, raw, t_out, rh)
            self.demand = cmd.heater - cmd.cooler
            self.last_cmd = cmd
            return RuntimeOutput(cmd, t_valid, quality, "UNSUPERVISED", self.demand, self.excluded, alarms)

        if quality == "BAD":
            self._was_bad = True
            req = self.last_cmd
        else:
            if self._was_bad:
                self.controller.reset(self.demand)       # bumpless return to automatic
                self._was_bad = False
            ctx = ControlContext(t_s, dt, t_valid, setpoint, rh, t_out, solar, forecast, 300.0, setpoint_preview)
            out = self.controller.update(ctx)
            if isinstance(out, ActuatorCommand):
                req = out
            else:
                self.demand = float(out) if math.isfinite(out) else 0.0
                req = self.allocator.allocate(self.demand, t_valid, t_out, rh)
        res = sup.apply(req, t_valid, quality, t_s, dt, t_out)
        alarms += res.alarms
        self.last_cmd = res.command
        return RuntimeOutput(res.command, t_valid, quality, res.mode, self.demand, self.excluded, alarms)
