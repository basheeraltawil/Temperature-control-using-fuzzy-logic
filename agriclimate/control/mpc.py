"""Economic model-predictive control on top of the learned thermal model.

Every ``solve_period_s`` the controller optimises the signed demand over a
receding horizon using the weather forecast and the setpoint preview (e.g.
pre-heating before the day setpoint ramps up, exploiting free solar gain
instead of the boiler).  Offset-free tracking uses an output-disturbance
estimate.  It is a supervisory layer: its output still goes through the
split-range allocator and the safety supervisor, and a fuzzy-PI fallback
(kept warm for bumpless transfer) takes over when the optimiser fails.

Optimisation problem, solved with L-BFGS-B over the blocked demands z:

    min_z  sum_k [ w_track * max(|T_k - r_k| - band, 0)^2
                 + w_heat * heater_k + w_cool * (cooler_k + 0.3 vent_k) ]
         + w_move * sum_j (z_j - z_{j-1})^2,        -1 <= z_j <= 1

T_k comes from the learned model (see agriclimate.ai.sysid), r_k from the
recipe preview, and heater/cooler/vent from the split-range allocator.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

from ..ai.sysid import ModelState, wet_bulb
from .allocator import SplitRangeAllocator
from .base import ControlContext, Controller
from .fuzzy_pi import FuzzyPIController


@dataclass
class MPCController(Controller):
    """Receding-horizon controller; see the module docstring for the cost function."""
    model: object = None                      # agriclimate.ai.sysid.LearnedThermalModel
    allocator: SplitRangeAllocator = field(default_factory=SplitRangeAllocator)
    horizon_s: float = 3600.0
    blocks: int = 6                           # move blocking: free decision variables
    solve_period_s: float = 300.0
    w_track: float = 1.0
    w_energy_heat: float = 0.02
    w_energy_cool: float = 0.03
    w_move: float = 0.5
    band: float = 0.2                         # no tracking penalty inside +-band
    disturbance_gain: float = 0.2
    name: str = "mpc"

    def __post_init__(self):
        if self.model is None:
            raise ValueError("MPCController needs a LearnedThermalModel (see agriclimate.ai.sysid)")
        self.steps = max(int(self.horizon_s / self.model.dt), self.blocks)
        # smooth copy for prediction: a deadband would give zero finite-difference gradients
        self._pred_alloc = SplitRangeAllocator(**{**self.allocator.__dict__, "deadband": 0.0})
        if getattr(self.model, "_mlp", None) is not None:
            raise NotImplementedError("MPC fast path supports the linear grey-box model only")
        # unscaled model coefficients, order = sysid.FEATURES:
        # [dT_env, heater_f, cooler_f, evap, vent_x, solar, dT_prev, bias]
        self._w = [float(c / s) for c, s in zip(self.model.coef, self.model.scale)]
        dt = self.model.dt
        self._ah = dt / (self.model.tau_heater + dt) if self.model.tau_heater > 0 else 1.0
        self._ac = dt / (self.model.tau_cooler + dt) if self.model.tau_cooler > 0 else 1.0
        self.fallback = FuzzyPIController()
        self.reset()

    def reset(self, demand: float = 0.0) -> None:
        """Restart the optimiser and the fallback from a given demand."""
        self._state_cls = ModelState
        self._z = np.full(self.blocks, float(demand))
        self._next_solve = 0.0
        self._u = float(demand)
        self._state = None
        self._dist = 0.0
        self._last_T = None
        self.fallback.reset(demand)
        self.solve_failures = 0

    def _rollout(self, z, st0, weather, sps, rh):
        """Predicted cost of a blocked demand sequence (inlined linear model for speed)."""
        alloc = self._pred_alloc
        w = self._w
        ah, ac = self._ah, self._ac
        per = -(-self.steps // self.blocks)
        T, Tp, hf, cf = st0.T, st0.T_prev, st0.hf, st0.cf
        cost = 0.0
        for k in range(self.steps):
            t_out, t_sup, solar = weather[k]
            cmd = alloc.allocate(z[min(k // per, self.blocks - 1)], T, t_out, rh)
            hf += ah * (cmd.heater - hf)
            cf += ac * (cmd.cooler - cf)
            dT = (w[0] * (t_out - T) + w[1] * hf + w[2] * cf + w[3] * cf * (t_sup - T)
                  + w[4] * cmd.vent * (t_out - T) + w[5] * solar + w[6] * (T - Tp) + w[7])
            Tp, T = T, T + dT + self._dist
            dev = abs(T - sps[k]) - self.band
            if dev > 0:
                cost += self.w_track * dev * dev
            cost += self.w_energy_heat * cmd.heater + self.w_energy_cool * (cmd.cooler + 0.3 * cmd.vent)
        dz = np.diff(np.concatenate(([self._u], z)))
        return cost + self.w_move * float(dz @ dz)

    def _track_state(self, ctx: ControlContext, cmd) -> None:
        """Advance the internal model state at model.dt resolution (observer)."""
        T = ctx.temperature
        if self._state is None:
            self._state = self._state_cls(T, T)
            return
        pred = self.model.step(self._state, cmd.heater, cmd.cooler, cmd.vent, ctx.t_out,
                               ctx.forecast[0].rh_out if ctx.forecast else 60.0, ctx.solar)
        # output-disturbance estimate for offset-free tracking (clamped)
        self._dist = float(np.clip(self._dist + self.disturbance_gain * (T - pred.T - self._dist) / 10.0,
                                   -0.05, 0.05))
        self._state = self._state_cls(T, self._state.T, pred.hf, pred.cf)

    def update(self, ctx: ControlContext) -> float:
        """Track the model state every model.dt, re-optimise every solve_period_s,
        hold the first planned demand in between."""
        fb = self.fallback.update(ctx)         # keep the fallback warm (bumpless switch)
        dt = self.model.dt
        if self._last_T is None or ctx.t_s - self._last_T >= dt - 1e-9:
            self._track_state(ctx, self.allocator.allocate(self._u, ctx.temperature, ctx.t_out, ctx.rh))
            self._last_T = ctx.t_s
        if ctx.t_s < self._next_solve:
            return self._u
        self._next_solve = ctx.t_s + self.solve_period_s

        weather = []
        for k in range(self.steps):
            if ctx.forecast:
                idx = min(int((k + 1) * dt / ctx.forecast_step_s), len(ctx.forecast) - 1)
                s = ctx.forecast[idx]
                t_out, rh_out, solar = s.t_out, s.rh_out, s.solar
            else:
                t_out, rh_out, solar = ctx.t_out, 60.0, ctx.solar
            t_sup = t_out - 0.8 * (t_out - wet_bulb(t_out, rh_out))
            weather.append((t_out, t_sup, solar / 1000.0))
        preview = ctx.setpoint_preview or (lambda t: ctx.setpoint)
        sps = [preview(ctx.t_s + (k + 1) * dt) for k in range(self.steps)]

        k_shift = max(1, int(self.solve_period_s / (self.steps / self.blocks * dt)))
        args = (self._state, weather, sps, ctx.rh)
        shifted = np.concatenate((self._z[k_shift:], np.repeat(self._z[-1], min(k_shift, self.blocks))))
        candidates = [np.clip(shifted[: self.blocks], -1, 1), np.full(self.blocks, fb)]
        z0 = min(candidates, key=lambda z: self._rollout(z, *args))   # warm start
        try:
            res = minimize(self._rollout, z0, args=args, method="L-BFGS-B",
                           bounds=[(-1.0, 1.0)] * self.blocks, options={"maxiter": 30, "eps": 1e-2})
            ok = bool(np.all(np.isfinite(res.x)))
        except (ValueError, FloatingPointError, ArithmeticError):
            ok = False
        if not ok:
            self.solve_failures += 1
            self._u = fb
            return fb
        self._z = res.x
        self._u = float(res.x[0])
        self.fallback.reset(self._u)
        return self._u
