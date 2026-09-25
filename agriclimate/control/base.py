"""Common controller interface."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, List, Optional, Union

from ..plant.facility import ActuatorCommand
from ..plant.weather import WeatherSample


@dataclass
class ControlContext:
    """Everything a controller may use in one control cycle."""
    t_s: float                      # time since start, s
    dt: float                       # control period, s
    temperature: float              # validated air temperature, degC
    setpoint: float                 # degC
    rh: float = 60.0                # %
    t_out: float = 10.0             # degC
    solar: float = 0.0              # W/m2
    forecast: Optional[List[WeatherSample]] = None          # weather preview (MPC)
    forecast_step_s: float = 300.0
    setpoint_preview: Optional[Callable[[float], float]] = None  # t_s -> setpoint


class Controller(ABC):
    """A controller returns either a signed demand in [-1, 1] (+ heat, - cool),
    which the ``SplitRangeAllocator`` sequences onto heater / vents / cooler,
    or a complete ``ActuatorCommand`` if it drives the actuators itself."""

    name = "controller"

    @abstractmethod
    def update(self, ctx: ControlContext) -> Union[float, ActuatorCommand]:
        """Compute one control action for the given context."""
        ...

    def reset(self, demand: float = 0.0) -> None:
        """Bumpless (re)initialisation, e.g. after a manual/auto transfer."""
