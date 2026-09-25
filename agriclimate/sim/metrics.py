"""Key performance indicators for agricultural climate control."""
from __future__ import annotations

from typing import Dict, List

import numpy as np
import pandas as pd


def compute_metrics(log: pd.DataFrame, scenario, alarms: List = ()) -> Dict[str, float]:
    """KPIs of one run (true air temperature, energy, wear, alarms); formulas in docs/THEORY.md."""
    dt_h = scenario.control_dt_s / 3600.0
    # KPIs use the TRUE air temperature: that is what the crop / animals experience,
    # regardless of what a faulty sensor reports.
    err = log["t_true"] - log["setpoint"]
    lim = scenario.crop_limits
    act = log[["heater", "cooler", "vent"]].to_numpy()
    overlap = (log["heater"] > 0.05) & (log["cooler"] > 0.05)
    e_heat = float(log["e_heat_kwh"].iat[-1])
    e_cool = float(log["e_cool_kwh"].iat[-1] + log["e_fan_kwh"].iat[-1])
    return {
        "rmse_K": float(np.sqrt(np.mean(err ** 2))),
        "mae_K": float(np.mean(np.abs(err))),
        "max_abs_err_K": float(np.max(np.abs(err))),
        "in_band_pct": float(100.0 * np.mean(np.abs(err) <= scenario.comfort_band)),
        "stress_h": float(((log["t_true"] < lim["min"]) | (log["t_true"] > lim["max"])).sum() * dt_h),
        "rh_over_90_h": float((log["rh"] > 90).sum() * dt_h),
        "heating_kwh": e_heat,
        "cooling_fan_kwh": e_cool,
        "total_kwh": e_heat + e_cool,
        "actuator_travel": float(np.abs(np.diff(act, axis=0)).sum()),
        "heat_cool_overlap_h": float(overlap.sum() * dt_h),
        "alarms": float(len(alarms)),
    }


def metrics_table(results) -> pd.DataFrame:
    """KPIs of several runs as a DataFrame."""
    rows = [{"scenario": r.scenario, "controller": r.controller, **r.metrics} for r in results]
    return pd.DataFrame(rows)
