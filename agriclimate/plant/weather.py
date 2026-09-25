"""Outdoor weather: a seeded synthetic generator and a CSV replay source.

The synthetic generator produces diurnal temperature / humidity / solar
cycles with stochastic cloud cover and optional events (cold front, heat wave).
Replace it with ``CsvWeather`` to replay logged station data or a
downloaded forecast (e.g. Open-Meteo, DWD, NOAA) for site-specific studies.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class WeatherSample:
    """Outdoor conditions at one instant."""
    t_out: float        # degC
    rh_out: float       # %
    solar: float        # W/m2 global horizontal irradiance
    wind: float         # m/s


@dataclass
class WeatherEvent:
    """Temporary change (cold front, heat wave, cloud, wind) with linear ramps."""
    kind: str           # "temperature_offset" | "cloud" | "wind"
    start_h: float
    end_h: float
    value: float
    ramp_h: float = 1.0

    def weight(self, t_h: float) -> float:
        """0..1 activation of the event at t_h, including the ramps."""
        if t_h < self.start_h or t_h > self.end_h:
            return 0.0
        up = min(1.0, (t_h - self.start_h) / max(self.ramp_h, 1e-6))
        down = min(1.0, (self.end_h - t_h) / max(self.ramp_h, 1e-6))
        return max(0.0, min(up, down))


@dataclass
class SyntheticWeather:
    """Seeded diurnal weather with random cloud cover and optional events."""
    t_mean: float = 15.0
    t_amplitude: float = 6.0          # half of the daily swing
    t_min_hour: float = 5.0
    rh_mean: float = 65.0
    rh_amplitude: float = 20.0
    solar_peak: float = 750.0         # W/m2 at solar noon, clear sky
    sunrise_h: float = 6.0
    sunset_h: float = 20.0
    cloudiness: float = 0.3           # 0 clear .. 1 overcast (mean)
    wind_mean: float = 3.0
    seed: int = 0
    events: List[WeatherEvent] = field(default_factory=list)

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        # Pre-compute a smooth random cloud / wind process with 5 min resolution for 60 days.
        n = 60 * 24 * 12
        w = rng.normal(size=n)
        kernel = np.exp(-np.linspace(0, 4, 36))
        smooth = np.convolve(w, kernel / kernel.sum(), mode="same") * 4.0
        self._cloud = np.clip(self.cloudiness + 0.35 * smooth, 0.0, 1.0)
        self._wind = np.clip(self.wind_mean * (1 + 0.4 * np.roll(smooth, 1000)), 0.0, None)
        self._tnoise = 0.6 * np.roll(smooth, 3000)

    def _idx(self, t_s: float) -> int:
        return int(t_s // 300) % self._cloud.size

    def sample(self, t_s: float) -> WeatherSample:
        """Weather at time t_s (s from the start)."""
        t_h = t_s / 3600.0
        hod = t_h % 24.0
        i = self._idx(t_s)
        # temperature: minimum near sunrise, maximum ~3 pm
        phase = 2 * np.pi * (hod - self.t_min_hour) / 24.0
        t_out = self.t_mean - self.t_amplitude * np.cos(phase) + self._tnoise[i]
        cloud = self._cloud[i]
        wind = self._wind[i]
        for ev in self.events:
            wgt = ev.weight(t_h)
            if ev.kind == "temperature_offset":
                t_out += wgt * ev.value
            elif ev.kind == "cloud":
                cloud = cloud + wgt * (ev.value - cloud)
            elif ev.kind == "wind":
                wind += wgt * ev.value
        if self.sunrise_h < hod < self.sunset_h:
            day_len = self.sunset_h - self.sunrise_h
            solar = self.solar_peak * np.sin(np.pi * (hod - self.sunrise_h) / day_len) ** 1.3
            solar *= 1.0 - 0.75 * cloud
        else:
            solar = 0.0
        rh = np.clip(self.rh_mean + self.rh_amplitude * np.cos(phase) + 10 * (cloud - 0.5), 15, 100)
        return WeatherSample(float(t_out), float(rh), float(max(solar, 0.0)), float(wind))

    def forecast(self, t_s: float, horizon_s: float, step_s: float,
                 error_std: float = 0.0, rng: Optional[np.random.Generator] = None):
        """Return a list of WeatherSamples with optional forecast error (random walk)."""
        rng = rng or np.random.default_rng()
        out, bias = [], 0.0
        for k in range(int(horizon_s // step_s)):
            s = self.sample(t_s + (k + 1) * step_s)
            if error_std > 0:
                bias += rng.normal(0, error_std)
                s = WeatherSample(s.t_out + bias, s.rh_out, max(0.0, s.solar * (1 + 0.1 * bias)), s.wind)
            out.append(s)
        return out


class CsvWeather:
    """Replay weather from a CSV with columns: time_s, t_out, rh_out, solar[, wind]."""

    def __init__(self, path: str):
        import pandas as pd

        df = pd.read_csv(path)
        self._t = df["time_s"].to_numpy(float)
        self._cols = {c: df[c].to_numpy(float) for c in ("t_out", "rh_out", "solar")}
        self._wind = df["wind"].to_numpy(float) if "wind" in df else np.full(len(df), 2.0)

    def sample(self, t_s: float) -> WeatherSample:
        """Weather at time t_s (s from the start)."""
        g = lambda arr: float(np.interp(t_s, self._t, arr))  # noqa: E731
        return WeatherSample(g(self._cols["t_out"]), g(self._cols["rh_out"]),
                             g(self._cols["solar"]), g(self._wind))

    def forecast(self, t_s, horizon_s, step_s, error_std=0.0, rng=None):
        """Future samples over horizon_s every step_s."""
        return [self.sample(t_s + (k + 1) * step_s) for k in range(int(horizon_s // step_s))]
