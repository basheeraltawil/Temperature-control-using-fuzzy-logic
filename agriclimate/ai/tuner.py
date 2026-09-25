"""Automatic controller tuning on the digital twin (evolutionary optimisation).

Hand-tuning fuzzy scaling gains is the main practical obstacle to fuzzy
control.  Here the gains are optimised with differential evolution against a
multi-objective cost (tracking error, energy, actuator wear) over one or more
agricultural scenarios - a standard "evolutionary fuzzy" approach.  On a real
site, first calibrate the twin with ``agriclimate.ai.sysid`` so the tuned
gains transfer; then roll out with conservative gains and refine.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np
from scipy.optimize import differential_evolution

SPACES = {
    # parameter: (low, high) searched in log10 space
    "fuzzy_pi": {"ke": (0.2, 3.0), "kr": (30.0, 3000.0), "ku": (5e-4, 2e-2)},
    "pid": {"kp": (0.02, 2.0), "ti": (60.0, 5000.0), "td": (1.0, 600.0)},
}


@dataclass
class TuningObjective:
    """Cost of one parameter vector (log10 gains), averaged over the scenarios."""
    scenarios: Sequence            # list of Scenario
    controller: str = "fuzzy_pi"
    w_travel: float = 0.02         # per unit actuator travel per hour
    w_energy: float = 0.5          # per relative energy increase over the reference
    energy_ref: Sequence[float] = ()

    def params(self, x) -> Dict[str, float]:
        """Convert the log10 search vector into controller keyword arguments."""
        return {k: float(10 ** v) for k, v in zip(SPACES[self.controller], x)}

    def __call__(self, x) -> float:
        from ..sim.runner import run_scenario
        from ..control.fuzzy_pi import FuzzyPIController
        from ..control.pid import PIDController

        cls = FuzzyPIController if self.controller == "fuzzy_pi" else PIDController
        cost = 0.0
        for i, sc in enumerate(self.scenarios):
            m = run_scenario(sc, cls(**self.params(x))).metrics
            ref = self.energy_ref[i] if self.energy_ref else m["total_kwh"]
            cost += (m["mae_K"] + 0.5 * m["stress_h"] / sc.duration_h
                     + self.w_travel * m["actuator_travel"] / sc.duration_h
                     + self.w_energy * max(0.0, m["total_kwh"] / max(ref, 1e-9) - 1.0))
        return cost / len(self.scenarios)


def tune(scenarios: List, controller: str = "fuzzy_pi", maxiter: int = 12, popsize: int = 8,
         workers: int = -1, seed: int = 0) -> Dict:
    """Differential evolution over SPACES[controller]; returns the best gains and their cost."""
    from ..sim.runner import run_scenario

    ref = [run_scenario(sc, "pid").metrics["total_kwh"] for sc in scenarios]
    obj = TuningObjective(scenarios, controller, energy_ref=ref)
    bounds = [(np.log10(lo), np.log10(hi)) for lo, hi in SPACES[controller].values()]
    res = differential_evolution(obj, bounds, maxiter=maxiter, popsize=popsize, seed=seed, tol=1e-3,
                                 polish=False, workers=workers, updating="deferred")
    return {"controller": controller, "params": obj.params(res.x), "cost": float(res.fun),
            "evaluations": int(res.nfev)}
