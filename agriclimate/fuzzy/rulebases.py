"""Rule-base factories.

The original project fuzzified *absolute* sensed and target temperatures on a
fixed [-50, 50] degC universe.  That design has three weaknesses for real plants:

* the rule base has to be re-written for every setpoint region,
* 8 of 25 input combinations have no rule (dead zones, see ``coverage_gaps``),
* there is no integral action, so a steady-state offset remains under load.

Industrial fuzzy controllers are instead built on the *normalised error* and
*error rate* (a fuzzy PI/PD structure), which is setpoint independent and can
be tuned with three scaling gains.  ``fuzzy_pi_rulebase`` builds that classic
7x7 Mac Vicar-Whelan table.
"""
from __future__ import annotations

import numpy as np

from .engine import MamdaniFIS, MembershipFunction, Rule, Variable

LABELS7 = ["NB", "NM", "NS", "ZE", "PS", "PM", "PB"]


def _tri_partition(labels, lo=-1.0, hi=1.0, saturate=True):
    centers = np.linspace(lo, hi, len(labels))
    step = centers[1] - centers[0]
    mfs = []
    for i, (lab, c) in enumerate(zip(labels, centers)):
        if saturate and i == 0:
            mfs.append(MembershipFunction(lab, "trapmf", [lo - 1.0, lo - 0.5, c, c + step]))
        elif saturate and i == len(labels) - 1:
            mfs.append(MembershipFunction(lab, "trapmf", [c - step, c, hi + 0.5, hi + 1.0]))
        else:
            mfs.append(MembershipFunction(lab, "trimf", [c - step, c, c + step]))
    return mfs


def fuzzy_pi_rulebase(resolution: int = 401) -> MamdaniFIS:
    """Inputs: e = (setpoint - T)*Ke and de = d(e)/dt*Kd, both on [-1, 1].
    Output: du (normalised increment of the signed heating(+)/cooling(-) demand)."""
    e = Variable("error", [-1, 1], _tri_partition(LABELS7))
    de = Variable("error_rate", [-1, 1], _tri_partition(LABELS7))
    du = Variable("delta_u", [-1, 1], _tri_partition(LABELS7, saturate=False))
    rules = []
    for i in range(7):
        for j in range(7):
            k = int(np.clip(i + j - 3, 0, 6))
            rules.append(Rule([i + 1, j + 1], [k + 1]))
    return MamdaniFIS("fuzzy_pi_climate", [e, de], [du], rules, resolution=resolution)
