"""Grey-box digital twin of an agricultural climate zone.

Two thermal nodes (air, thermal mass = soil/floor/structure/crop) and a
water-vapour balance.  The formulation follows the lumped models commonly
used in greenhouse climate literature (e.g. Vanthoor et al. 2011,
Stanghellini transpiration) but is reduced to parameters that can be
estimated on a real site from a few days of logged data.

Air:   C_a dTa/dt = Q_heat + Q_solar,a + Q_int - UA (Ta - To) - h_m A (Ta - Tm)
                    + rho cp F_vent (To - Ta) + Q_cool
Mass:  C_m dTm/dt = Q_solar,m + h_m A (Ta - Tm) - h_g A (Tm - Tg)
Vapour: V drho_v/dt = E_crop + E_int + F_vent (rho_v,o - rho_v) + pad/coil terms
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Optional

import numpy as np

from .psychrometrics import (CP_AIR, LATENT, RHO_AIR, relative_humidity, vapor_density, vpd, wet_bulb)
from .weather import WeatherSample


@dataclass
class FacilityParams:
    """Physical parameters of a climate zone (SI units; see presets.py for examples)."""
    name: str = "greenhouse"
    floor_area: float = 1000.0            # m2
    volume: float = 4500.0                # m3
    envelope_area: float = 1400.0         # m2 (cover or insulated walls + roof)
    u_value: float = 6.0                  # W/(m2 K) envelope heat-loss coefficient
    solar_transmittance: float = 0.7      # 0 for opaque buildings
    shade: float = 0.0                    # fixed shade screen fraction
    air_capacity_factor: float = 2.0      # >1 accounts for benches, pipes, fixtures
    mass_capacity: float = 60e3           # J/(m2 K) floor/soil/crop thermal mass
    h_mass: float = 8.0                   # W/(m2 K) air <-> mass exchange
    h_ground: float = 2.0                 # W/(m2 K) mass <-> deep ground
    t_ground: float = 12.0                # degC
    solar_to_air: float = 0.6             # share of sensible solar heating the air directly
    crop_factor: float = 1.0              # 0 = no transpiring crop
    leakage_ach: float = 0.5              # air changes per hour through leaks
    vent_max_ach: float = 40.0            # at 100 % vent opening
    natural_ventilation: bool = True      # wind-assisted vents vs. fans
    vent_fan_power_w: float = 0.0         # electrical, at 100 % (mechanical ventilation)
    heater_max_w: float = 250e3
    heater_efficiency: float = 0.9
    heater_tau_s: float = 180.0           # boiler / hot-water pipe lag
    cooler_type: str = "evaporative"      # "evaporative" | "refrigeration" | "none"
    cooler_max_w: float = 0.0             # refrigeration capacity (thermal)
    cooler_cop: float = 3.0
    pad_airflow_m3s: float = 30.0         # evaporative pad-and-fan airflow at 100 %
    pad_efficiency: float = 0.8
    pad_fan_power_w: float = 12e3
    cooler_tau_s: float = 60.0
    vent_rate_per_s: float = 1.0 / 120.0  # full stroke in 2 min
    internal_gain_w: float = 0.0          # animals, lamps, respiring produce, compost
    internal_moisture_kg_s: float = 0.0

    @classmethod
    def from_dict(cls, d: Dict) -> "FacilityParams":
        """Create parameters from a mapping; unknown keys raise an error."""
        known = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        unknown = set(d) - set(known)
        if unknown:
            raise ValueError(f"Unknown facility parameters: {sorted(unknown)}")
        return cls(**known)

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class ActuatorCommand:
    """Actuator commands, each 0..1."""
    heater: float = 0.0   # 0..1  (8-bit PWM 0..255 on the legacy hardware)
    cooler: float = 0.0   # 0..1
    vent: float = 0.0     # 0..1  (vent opening or fan stage)

    def clipped(self) -> "ActuatorCommand":
        """Copy with every command limited to 0..1."""
        return ActuatorCommand(*(float(np.clip(v, 0.0, 1.0)) for v in (self.heater, self.cooler, self.vent)))

    def as_pwm(self) -> Dict[str, int]:
        """Commands as 8-bit PWM duties (0..255), as in the original project."""
        c = self.clipped()
        return {"heater": round(c.heater * 255), "cooler": round(c.cooler * 255), "vent": round(c.vent * 255)}


@dataclass
class ActuatorHealth:
    """Fault injection on the actuator side (used by scenarios)."""
    heater_capacity: float = 1.0
    cooler_capacity: float = 1.0
    vent_stuck_at: Optional[float] = None


@dataclass
class FacilityState:
    """Simulated state: temperatures, vapour density, actuator positions, energy counters."""
    t_air: float
    t_mass: float
    rho_v: float                       # kg/m3
    heater: float = 0.0                # actual actuator positions
    cooler: float = 0.0
    vent: float = 0.0
    energy_heat_kwh: float = 0.0       # delivered fuel/electric energy for heating
    energy_cool_kwh: float = 0.0       # electrical energy for cooling
    energy_fan_kwh: float = 0.0


class Facility:
    """Two-node thermal model with vapour balance; see the module docstring for the equations."""
    def __init__(self, params: FacilityParams, t_init: float = 18.0, rh_init: float = 70.0):
        self.p = params
        p = params
        self.c_air = RHO_AIR * CP_AIR * p.volume * p.air_capacity_factor
        self.c_mass = p.mass_capacity * p.floor_area
        self.ua = p.u_value * p.envelope_area
        self.state = FacilityState(t_init, t_init, float(vapor_density(t_init, rh_init)))
        self.health = ActuatorHealth()
        self.last_flows: Dict[str, float] = {}

    @property
    def rh(self) -> float:
        """Indoor relative humidity, %."""
        return float(relative_humidity(self.state.t_air, self.state.rho_v))

    def _actuators(self, cmd: ActuatorCommand, dt: float) -> None:
        s, p, h = self.state, self.p, self.health
        c = cmd.clipped()
        s.heater += (c.heater - s.heater) * min(1.0, dt / p.heater_tau_s)
        s.cooler += (c.cooler - s.cooler) * min(1.0, dt / p.cooler_tau_s)
        if h.vent_stuck_at is not None:
            s.vent = h.vent_stuck_at
        else:
            s.vent += float(np.clip(c.vent - s.vent, -p.vent_rate_per_s * dt, p.vent_rate_per_s * dt))

    def step(self, dt: float, cmd: ActuatorCommand, w: WeatherSample,
             extra_ach: float = 0.0, extra_gain_w: float = 0.0, substep: float = 2.0) -> FacilityState:
        """Advance the model by dt seconds (explicit Euler with sub-steps of at most `substep` s)."""
        n = max(1, int(np.ceil(dt / substep)))
        h = dt / n
        for _ in range(n):
            self._actuators(cmd, h)
            self._integrate(h, w, extra_ach, extra_gain_w)
        return self.state

    def _integrate(self, dt: float, w: WeatherSample, extra_ach: float, extra_gain_w: float) -> None:
        s, p = self.state, self.p
        ta, tm = s.t_air, s.t_mass
        rh_air = float(relative_humidity(ta, s.rho_v))

        # --- solar and crop transpiration
        solar_in = w.solar * p.floor_area * p.solar_transmittance * (1.0 - p.shade)
        latent = p.crop_factor * (0.45 * solar_in + 25.0 * p.floor_area * float(vpd(ta, rh_air)) * 0.2)
        latent = min(latent, 0.9 * solar_in + 5.0 * p.floor_area)
        sensible = max(solar_in - latent, 0.0)
        q_solar_air = p.solar_to_air * sensible
        q_solar_mass = (1.0 - p.solar_to_air) * sensible
        e_crop = latent / LATENT

        # --- heating
        q_heat = s.heater * p.heater_max_w * self.health.heater_capacity * p.heater_efficiency
        s.energy_heat_kwh += s.heater * p.heater_max_w * self.health.heater_capacity * dt / 3.6e6

        # --- ventilation (+ wind effect for natural vents)
        wind_factor = float(np.clip(0.5 + 0.12 * w.wind, 0.4, 1.6)) if p.natural_ventilation else 1.0
        ach = p.leakage_ach + extra_ach + s.vent * p.vent_max_ach * wind_factor
        f_vent = ach * p.volume / 3600.0
        rho_v_out = float(vapor_density(w.t_out, w.rh_out))
        s.energy_fan_kwh += s.vent * p.vent_fan_power_w * dt / 3.6e6

        # --- cooling
        q_cool, e_cool, f_pad = 0.0, 0.0, 0.0
        cool = s.cooler * self.health.cooler_capacity
        if p.cooler_type == "evaporative" and cool > 0:
            f_pad = cool * p.pad_airflow_m3s
            t_sup = w.t_out - p.pad_efficiency * (w.t_out - float(wet_bulb(w.t_out, w.rh_out)))
            rho_v_sup = rho_v_out + RHO_AIR * CP_AIR * (w.t_out - t_sup) / LATENT
            q_cool = RHO_AIR * CP_AIR * f_pad * (t_sup - ta)
            e_cool = f_pad * (rho_v_sup - s.rho_v)
            s.energy_cool_kwh += s.cooler * p.pad_fan_power_w * dt / 3.6e6
        elif p.cooler_type == "refrigeration" and cool > 0:
            q_cool = -cool * p.cooler_max_w
            coil_flow = cool * p.cooler_max_w / 5000.0
            rho_sat_coil = float(vapor_density(ta - 8.0, 100.0))
            e_cool = -coil_flow * max(0.0, s.rho_v - rho_sat_coil)
            s.energy_cool_kwh += cool * p.cooler_max_w / p.cooler_cop * dt / 3.6e6

        q_env = self.ua * (w.t_out - ta)
        q_vent = RHO_AIR * CP_AIR * f_vent * (w.t_out - ta)
        q_mass = p.h_mass * p.floor_area * (tm - ta)
        q_int = p.internal_gain_w + extra_gain_w

        dta = (q_heat + q_solar_air + q_int + q_env + q_vent + q_mass + q_cool) / self.c_air
        dtm = (q_solar_mass - q_mass - p.h_ground * p.floor_area * (tm - p.t_ground)) / self.c_mass
        drho = (e_crop + p.internal_moisture_kg_s + f_vent * (rho_v_out - s.rho_v) + e_cool) / p.volume

        s.t_air = ta + dta * dt
        s.t_mass = tm + dtm * dt
        # vapour cannot exceed saturation (condensation on the cover)
        s.rho_v = float(min(max(s.rho_v + drho * dt, 1e-4), vapor_density(s.t_air, 100.0)))
        self.last_flows = {"q_heat": q_heat, "q_solar": q_solar_air + q_solar_mass, "q_env": q_env,
                           "q_vent": q_vent, "q_cool": q_cool, "q_int": q_int, "ach": ach}
