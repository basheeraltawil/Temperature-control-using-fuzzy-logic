"""The original MATLAB controller (``matlab/temperature_controlling1.fis``).

Inputs are the absolute sensed and target temperatures, outputs are two 8-bit
PWM duties driving a heating (P) and a cooling (N) element directly, exactly
like the original hardware.  Kept as a baseline to quantify the improvements.
"""
from __future__ import annotations

from ..fuzzy.fis_io import load_fis
from ..paths import find_file
from ..plant.facility import ActuatorCommand
from .base import ControlContext, Controller

DEFAULT_FIS = find_file("matlab", "temperature_controlling1.fis")


class LegacyFISController(Controller):
    name = "legacy_fis"

    def __init__(self, fis_path=DEFAULT_FIS, min_vent: float = 0.0):
        self.fis = load_fis(fis_path)
        self.min_vent = min_vent
        self.no_rule_count = 0

    def update(self, ctx: ControlContext) -> ActuatorCommand:
        res = self.fis.evaluate((ctx.temperature, ctx.setpoint), default=(0.0, 0.0))
        if not res.fired:
            self.no_rule_count += 1
        p_pwm, n_pwm = res.outputs
        return ActuatorCommand(heater=p_pwm / 255.0, cooler=n_pwm / 255.0, vent=self.min_vent).clipped()
