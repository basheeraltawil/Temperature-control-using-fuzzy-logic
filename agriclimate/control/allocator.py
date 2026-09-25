"""Split-range sequencing of a signed climate demand onto real actuators.

Industrial climate computers never let heating and cooling fight each other.
The demand u in [-1, 1] is mapped as

    u > +db          -> heater  (0..1)
    -vent_stage < u  -> vents open first ("free cooling") when outdoor air is colder
    u < -vent_stage  -> mechanical / evaporative cooling stage

plus a minimum ventilation for air quality and a heat-and-vent
dehumidification override, a standard greenhouse practice against
condensation and fungal disease (Botrytis) at high RH.
"""
from __future__ import annotations

from dataclasses import dataclass

from ..plant.facility import ActuatorCommand


@dataclass
class SplitRangeAllocator:
    """Maps the signed demand u in [-1, 1] to heater, cooler and vent commands."""
    deadband: float = 0.02
    vent_stage: float = 0.5          # share of the cooling range served by vents (0 = none)
    min_vent: float = 0.0            # minimum ventilation (CO2 / ammonia / air quality)
    free_cooling_margin: float = 1.0 # outdoor must be this much colder to use vents, K
    rh_max: float = 100.0            # dehumidification threshold, %
    dehum_vent: float = 0.15
    dehum_heat: float = 0.10

    def allocate(self, demand: float, t_air: float, t_out: float, rh: float = 50.0) -> ActuatorCommand:
        """Return the actuator command for demand u at the given indoor/outdoor state."""
        u = min(1.0, max(-1.0, float(demand)))
        heater = cooler = 0.0
        vent = self.min_vent
        if u > self.deadband:
            heater = (u - self.deadband) / (1.0 - self.deadband)
        elif u < -self.deadband:
            c = (-u - self.deadband) / (1.0 - self.deadband)
            free_cooling = t_out < t_air - self.free_cooling_margin
            if self.vent_stage > 0 and free_cooling:
                vent = max(vent, min(1.0, c / self.vent_stage))
                cooler = max(0.0, (c - self.vent_stage) / (1.0 - self.vent_stage))
            else:
                cooler = c
        if rh > self.rh_max and cooler == 0.0:
            # heat-and-vent dehumidification; only when not actively cooling
            excess = min(1.0, (rh - self.rh_max) / 10.0)
            vent = max(vent, self.dehum_vent * excess + self.min_vent)
            heater = max(heater, self.dehum_heat * excess)
        return ActuatorCommand(min(heater, 1.0), min(cooler, 1.0), min(vent, 1.0))
