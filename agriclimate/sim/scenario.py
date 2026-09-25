"""Scenario definition (YAML) for agricultural climate-control studies."""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

import yaml

from ..control.allocator import SplitRangeAllocator
from ..control.schedule import Schedule, schedule_from_config
from ..control.supervisor import SupervisorConfig
from ..plant.facility import FacilityParams
from ..plant.presets import get_preset
from ..plant.sensors import SensorFault
from ..plant.weather import CsvWeather, SyntheticWeather, WeatherEvent

from ..paths import find_file, first_dir


@dataclass
class Scenario:
    """Everything needed to simulate one agricultural task (loaded from YAML)."""
    name: str
    description: str
    facility: FacilityParams
    setpoint: Schedule
    weather_cfg: Dict[str, Any]
    duration_h: float = 48.0
    control_dt_s: float = 30.0
    initial: Dict[str, float] = field(default_factory=lambda: {"t_air": 18.0, "rh": 70.0})
    comfort_band: float = 1.0
    crop_limits: Dict[str, float] = field(default_factory=lambda: {"min": 5.0, "max": 35.0})
    allocator_cfg: Dict[str, Any] = field(default_factory=dict)
    supervisor_cfg: Dict[str, Any] = field(default_factory=dict)
    sensors_cfg: Dict[str, Any] = field(default_factory=lambda: {"count": 2})
    faults: Dict[str, List[Dict[str, Any]]] = field(default_factory=dict)
    disturbances: List[Dict[str, Any]] = field(default_factory=list)
    controllers: List[str] = field(default_factory=lambda: ["fuzzy_pi", "pid", "legacy_fis"])
    controller_params: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    agronomy: str = ""
    raw: Dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------- builders
    def weather(self):
        """Build the weather source."""
        cfg = dict(self.weather_cfg)
        kind = cfg.pop("type", "synthetic")
        cfg.pop("forecast_error_std", None)
        if kind == "csv":
            return CsvWeather(cfg["path"])
        cfg["events"] = [WeatherEvent(**e) for e in cfg.get("events", [])]
        return SyntheticWeather(**cfg)

    def allocator(self) -> SplitRangeAllocator:
        """Build the split-range allocator."""
        return SplitRangeAllocator(**self.allocator_cfg)

    def supervisor_config(self) -> SupervisorConfig:
        """Build the supervisor configuration."""
        return SupervisorConfig(**self.supervisor_cfg)

    def sensor_faults(self, index: int) -> List[SensorFault]:
        """Faults configured for sensor ``index`` (0-based)."""
        return [SensorFault(**{k: v for k, v in f.items() if k != "sensor"})
                for f in self.faults.get("sensors", []) if int(f.get("sensor", 1)) == index + 1]

    def copy(self, **changes) -> "Scenario":
        """Deep copy with some attributes replaced, e.g. copy(duration_h=24)."""
        new = copy.deepcopy(self)
        for k, v in changes.items():
            setattr(new, k, v)
        return new

    # ------------------------------------------------------------- loading
    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Scenario":
        """Build a scenario from a parsed YAML mapping."""
        d = copy.deepcopy(d)
        fac = d.get("facility", "greenhouse_glass")
        params = get_preset(fac) if isinstance(fac, str) else FacilityParams.from_dict(fac)
        overrides = d.get("facility_overrides", {})
        if overrides:
            params = FacilityParams.from_dict({**params.to_dict(), **overrides})
        return cls(
            name=d["name"], description=d.get("description", ""), facility=params,
            setpoint=schedule_from_config(d["setpoint"]), weather_cfg=d.get("weather", {}),
            duration_h=float(d.get("duration_h", 48)), control_dt_s=float(d.get("control_dt_s", 30)),
            initial=d.get("initial", {"t_air": 18.0, "rh": 70.0}), comfort_band=float(d.get("comfort_band", 1.0)),
            crop_limits=d.get("crop_limits", {"min": 5.0, "max": 35.0}), allocator_cfg=d.get("allocator", {}),
            supervisor_cfg=d.get("supervisor", {}), sensors_cfg=d.get("sensors", {"count": 2}),
            faults=d.get("faults", {}) or {}, disturbances=d.get("disturbances", []) or [],
            controllers=d.get("controllers", ["fuzzy_pi", "pid", "legacy_fis"]),
            controller_params=d.get("controller_params", {}) or {}, agronomy=d.get("agronomy", ""), raw=d,
        )

    @classmethod
    def load(cls, path_or_name) -> "Scenario":
        """Load a scenario by file path or by name from the scenarios folder."""
        p = Path(path_or_name)
        if not p.exists():
            p = find_file("scenarios", f"{path_or_name}.yaml")
        with open(p) as fh:
            return cls.from_dict(yaml.safe_load(fh))


def list_scenarios() -> List[str]:
    """Names of the available scenario files."""
    return sorted(p.stem for p in first_dir("scenarios").glob("*.yaml"))
