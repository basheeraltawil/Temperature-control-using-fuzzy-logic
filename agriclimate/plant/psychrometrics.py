"""Moist-air helper functions (Tetens / FAO-56 formulations, sea-level pressure)."""
from __future__ import annotations

import numpy as np

P_ATM = 101_325.0      # Pa
R_V = 461.5            # J/(kg K), water vapour
RHO_AIR = 1.2          # kg/m3
CP_AIR = 1006.0        # J/(kg K)
LATENT = 2.45e6        # J/kg


def saturation_vapor_pressure(t_c):
    """kPa (FAO-56 eq. 11)."""
    return 0.6108 * np.exp(17.27 * np.asarray(t_c) / (np.asarray(t_c) + 237.3))


def vapor_density(t_c, rh_pct):
    """Absolute humidity in kg/m3."""
    e = saturation_vapor_pressure(t_c) * 1000.0 * np.asarray(rh_pct) / 100.0
    return e / (R_V * (np.asarray(t_c) + 273.15))


def relative_humidity(t_c, rho_v):
    e = np.asarray(rho_v) * R_V * (np.asarray(t_c) + 273.15)
    return np.clip(100.0 * e / (saturation_vapor_pressure(t_c) * 1000.0), 0.0, 100.0)


def vpd(t_c, rh_pct):
    """Vapour-pressure deficit in kPa - the key crop-stress indicator."""
    return saturation_vapor_pressure(t_c) * (1.0 - np.asarray(rh_pct) / 100.0)


def wet_bulb(t_c, rh_pct):
    """Stull (2011) empirical wet-bulb temperature, degC."""
    t, rh = np.asarray(t_c, dtype=float), np.asarray(rh_pct, dtype=float)
    return (t * np.arctan(0.151977 * np.sqrt(rh + 8.313659)) + np.arctan(t + rh)
            - np.arctan(rh - 1.676331) + 0.00391838 * rh ** 1.5 * np.arctan(0.023101 * rh) - 4.686035)
