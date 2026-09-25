"""Membership functions (MATLAB Fuzzy Logic Toolbox compatible names and parameters)."""
from __future__ import annotations

import numpy as np


def trimf(x, p):
    """Triangle with feet a, c and peak b."""
    a, b, c = p
    x = np.asarray(x, dtype=float)
    left = np.where(b > a, (x - a) / (b - a + 1e-300), (x >= b).astype(float))
    right = np.where(c > b, (c - x) / (c - b + 1e-300), (x <= b).astype(float))
    return np.clip(np.minimum(left, right), 0.0, 1.0)


def trapmf(x, p):
    """Trapezoid with feet a, d and shoulders b, c."""
    a, b, c, d = p
    x = np.asarray(x, dtype=float)
    left = (x - a) / (b - a) if b > a else (x >= b).astype(float)
    right = (d - x) / (d - c) if d > c else (x <= c).astype(float)
    return np.clip(np.minimum(np.minimum(left, 1.0), right), 0.0, 1.0)


def gaussmf(x, p):
    """Gaussian exp(-(x-c)^2 / (2 sigma^2))."""
    sigma, c = p
    x = np.asarray(x, dtype=float)
    return np.exp(-((x - c) ** 2) / (2.0 * sigma**2))


def gbellmf(x, p):
    """Generalised bell 1 / (1 + |(x-c)/a|^(2b))."""
    a, b, c = p
    x = np.asarray(x, dtype=float)
    return 1.0 / (1.0 + np.abs((x - c) / a) ** (2 * b))


def sigmf(x, p):
    """Sigmoid 1 / (1 + exp(-a (x-c)))."""
    a, c = p
    x = np.asarray(x, dtype=float)
    return 1.0 / (1.0 + np.exp(-a * (x - c)))


MF_TYPES = {
    "trimf": trimf,
    "trapmf": trapmf,
    "gaussmf": gaussmf,
    "gbellmf": gbellmf,
    "sigmf": sigmf,
}


def evaluate_mf(mf_type: str, x, params):
    """Evaluate a membership function given by its MATLAB type name."""
    try:
        fn = MF_TYPES[mf_type]
    except KeyError as exc:
        raise ValueError(f"Unsupported membership function '{mf_type}'") from exc
    return fn(x, params)
