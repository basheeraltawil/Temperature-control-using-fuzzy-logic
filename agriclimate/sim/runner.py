"""Closed-loop simulation: weather -> facility -> sensors -> (AI monitor) ->
supervisor -> controller -> allocator -> supervisor -> actuators."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union

import numpy as np
import pandas as pd

from ..control.base import Controller
from ..control.fuzzy_pi import FuzzyPIController
from ..control.legacy_fis import LegacyFISController
from ..control.pid import PIDController
from ..control.supervisor import Alarm
from ..plant.facility import Facility
from ..plant.sensors import Sensor
from ..runtime import ClimateRuntime
from .metrics import compute_metrics
from .scenario import Scenario

_MODEL_CACHE: Dict[str, object] = {}


@dataclass
class RunResult:
    scenario: str
    controller: str
    log: pd.DataFrame
    alarms: List[Alarm]
    metrics: Dict[str, float] = field(default_factory=dict)


def identify_model(scenario: Scenario, hours: float = 48.0, seed: int = 123):
    """Identify (and cache) a learned thermal model of the scenario's facility."""
    from ..ai.sysid import LearnedThermalModel, excitation_experiment

    key = f"{scenario.name}:{hours}:{seed}"
    if key not in _MODEL_CACHE:
        log = excitation_experiment(scenario, hours=hours, seed=seed)
        _MODEL_CACHE[key] = LearnedThermalModel().fit(log)
    return _MODEL_CACHE[key]


def build_detector(scenario: Scenario, seed: int = 0):
    """Train the AI anomaly monitor on fault-free operation of the facility.
    On a real site: identify the model and fit the detector on a week of
    commissioning data that was reviewed as healthy."""
    from ..ai.anomaly import AnomalyDetector

    healthy = run_scenario(scenario.copy(faults={}, duration_h=min(scenario.duration_h, 48.0)),
                           "fuzzy_pi", seed=seed + 7).log
    return AnomalyDetector(model=identify_model(scenario)).fit(healthy)


def make_controller(name: str, scenario: Scenario) -> (Controller, bool):
    """Returns (controller, supervised). A '+ai' suffix enables anomaly detection in the runner."""
    base = name.replace("+ai", "")
    params = scenario.controller_params.get(base, {})
    if base == "fuzzy_pi":
        return FuzzyPIController(**params), True
    if base == "pid":
        return PIDController(**params), True
    if base == "legacy_fis":
        return LegacyFISController(**params), False     # as originally designed: no supervisor
    if base == "mpc":
        from ..control.mpc import MPCController

        return MPCController(model=identify_model(scenario), allocator=scenario.allocator(), **params), True
    raise ValueError(f"Unknown controller '{name}' (fuzzy_pi, pid, legacy_fis, mpc, optional '+ai')")


def _disturbance(scenario: Scenario, t_h: float):
    extra_ach, extra_gain = 0.0, 0.0
    for d in scenario.disturbances:
        kind = d["kind"]
        if kind == "door":
            start = d.get("start_h", 0.0)
            if t_h >= start and ((t_h - start) % d["period_h"]) * 60.0 < d["duration_min"]:
                extra_ach += d["ach"]
        elif d.get("start_h", 0.0) <= t_h < d.get("end_h", math.inf):
            if kind == "infiltration":
                extra_ach += d["ach"]
            elif kind == "internal_gain":
                extra_gain += d["watts"]
    return extra_ach, extra_gain


def _apply_actuator_faults(scenario: Scenario, plant: Facility, t_h: float) -> None:
    h = plant.health
    h.heater_capacity, h.cooler_capacity, h.vent_stuck_at = 1.0, 1.0, None
    for f in scenario.faults.get("actuators", []):
        if f["start_h"] <= t_h < f.get("end_h", math.inf):
            if f["kind"] == "heater_capacity":
                h.heater_capacity = f["value"]
            elif f["kind"] == "cooler_capacity":
                h.cooler_capacity = f["value"]
            elif f["kind"] == "vent_stuck":
                h.vent_stuck_at = f["value"]


def run_scenario(scenario: Scenario, controller: Union[str, Controller] = "fuzzy_pi", seed: int = 0,
                 anomaly_detection: Optional[bool] = None, supervised: Optional[bool] = None) -> RunResult:
    if isinstance(controller, str):
        ctrl_name = controller
        ctrl, default_sup = make_controller(controller, scenario)
        use_ai = controller.endswith("+ai") if anomaly_detection is None else anomaly_detection
    else:
        ctrl_name, ctrl, default_sup = controller.name, controller, True
        use_ai = bool(anomaly_detection)
    supervised = default_sup if supervised is None else supervised

    dt = scenario.control_dt_s
    plant = Facility(scenario.facility, scenario.initial.get("t_air", 18.0), scenario.initial.get("rh", 70.0))
    weather = scenario.weather()
    fc_err = float(scenario.weather_cfg.get("forecast_error_std", 0.05))
    scfg = dict(scenario.sensors_cfg)
    n_sensors = int(scfg.pop("count", 2))
    sensors = [Sensor(name=f"T{i + 1}", seed=seed * 100 + i, faults=scenario.sensor_faults(i), **scfg)
               for i in range(n_sensors)]
    detector = build_detector(scenario, seed) if use_ai else None
    runtime = ClimateRuntime(ctrl, scenario.allocator(), scenario.supervisor_config(), supervised, detector)
    needs_forecast = hasattr(ctrl, "model")
    rng = np.random.default_rng(seed)

    rows, alarms = [], []
    for k in range(int(scenario.duration_h * 3600 / dt)):
        t = k * dt
        t_h = t / 3600.0
        w = weather.sample(t)
        _apply_actuator_faults(scenario, plant, t_h)
        extra_ach, extra_gain = _disturbance(scenario, t_h)
        sp = scenario.setpoint(t)
        readings = [s.read(plant.state.t_air, t, dt) for s in sensors]
        forecast = weather.forecast(t, 3 * 3600, 300, fc_err, rng) if needs_forecast else None
        out = runtime.step(t, dt, readings, sp, plant.rh, w.t_out, w.rh_out, w.solar, forecast, scenario.setpoint)
        alarms += out.alarms
        cmd = out.command

        row = {"t_s": t, "t_h": t_h, "setpoint": sp, "t_true": plant.state.t_air, "t_meas": out.temperature,
               "rh": plant.rh, "t_out": w.t_out, "rh_out": w.rh_out, "solar": w.solar, "demand": out.demand,
               "heater": cmd.heater, "cooler": cmd.cooler, "vent": cmd.vent, "vent_pos": plant.state.vent,
               "mode": out.mode, "quality": out.quality, "excluded": len(out.excluded),
               "e_heat_kwh": plant.state.energy_heat_kwh, "e_cool_kwh": plant.state.energy_cool_kwh,
               "e_fan_kwh": plant.state.energy_fan_kwh}
        for i, r in enumerate(readings):
            row[f"sensor_{i + 1}"] = r
        rows.append(row)
        plant.step(dt, cmd, w, extra_ach, extra_gain)

    log = pd.DataFrame(rows)
    res = RunResult(scenario.name, ctrl_name, log, alarms)
    res.metrics = compute_metrics(log, scenario, alarms)
    return res


def compare_controllers(scenario: Scenario, controllers: Optional[List[str]] = None, seed: int = 0):
    return [run_scenario(scenario, c, seed=seed) for c in (controllers or scenario.controllers)]
