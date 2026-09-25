"""Data-driven (grey-box) thermal model identification.

A *physics-informed* regression: the regressors are the heat-flow terms of the
energy balance (envelope loss, heater, cooler, ventilation exchange, solar) so
the learned coefficients stay interpretable and extrapolate safely.  Actuator
lags (boiler, pipe rail, compressor) are identified automatically as
first-order input filters.  An optional small neural network
(``residual="mlp"``) learns what the physics terms miss - the "hybrid
modelling" approach now common in building and greenhouse control.

The same code identifies a real facility from historian / ROS bag data: log
temperature, actuator outputs and weather (see ``REQUIRED_COLUMNS``), then
call ``fit``.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

FEATURES = ["dT_env", "heater_f", "cooler_f", "evap", "vent_x", "solar", "dT_prev", "bias"]
REQUIRED_COLUMNS = ["t_s", "t_meas", "heater", "cooler", "vent", "t_out", "rh_out", "solar"]
LAG_GRID = (0.0, 60.0, 180.0, 300.0, 600.0)


def wet_bulb(t: float, rh: float) -> float:
    """Stull (2011) wet-bulb temperature, scalar version (fast in MPC loops)."""
    return (t * math.atan(0.151977 * math.sqrt(rh + 8.313659)) + math.atan(t + rh)
            - math.atan(rh - 1.676331) + 0.00391838 * rh ** 1.5 * math.atan(0.023101 * rh) - 4.686035)


def _features(T, T_prev, hf, cf, vent, t_out, rh_out, solar) -> List[float]:
    """Regressors in the order of FEATURES (one row of the design matrix)."""
    t_sup = t_out - 0.8 * (t_out - wet_bulb(t_out, rh_out))
    return [t_out - T, hf, cf, cf * (t_sup - T), vent * (t_out - T), solar / 1000.0, T - T_prev, 1.0]


@dataclass
class ModelState:
    """State of the learned model: temperature now and one step ago, filtered actuator inputs."""
    T: float
    T_prev: float
    hf: float = 0.0      # filtered heater
    cf: float = 0.0      # filtered cooler


@dataclass
class LearnedThermalModel:
    """Grey-box model  dT_{k+1} = theta . phi(T_k, T_{k-1}, u_k, weather_k)  fitted by ridge regression."""
    dt: float = 60.0                           # model sample time, s
    alpha: float = 1e-3                        # ridge regularisation
    residual: Optional[str] = None             # None | "mlp"
    tau_heater: float = 0.0                    # identified actuator lags, s
    tau_cooler: float = 0.0
    coef: Optional[np.ndarray] = None
    scale: Optional[np.ndarray] = None
    metrics: Dict[str, float] = field(default_factory=dict)
    _mlp: object = None

    # ------------------------------------------------------------ training
    @staticmethod
    def resample(df, dt: float):
        """Average inputs and take the first temperature in each dt bin."""
        missing = set(REQUIRED_COLUMNS) - set(df.columns)
        if missing:
            raise ValueError(f"training data is missing columns {sorted(missing)}")
        d = df[REQUIRED_COLUMNS].copy()
        d["bin"] = (d["t_s"] // dt).astype(int)
        agg = d.groupby("bin").agg({"t_s": "first", "t_meas": "first", "heater": "mean", "cooler": "mean",
                                    "vent": "mean", "t_out": "mean", "rh_out": "mean", "solar": "mean"})
        return agg.dropna().reset_index(drop=True)

    @staticmethod
    def _filter(u: np.ndarray, tau: float, dt: float) -> np.ndarray:
        if tau <= 0:
            return u.copy()
        a = dt / (tau + dt)
        out = np.empty_like(u)
        acc = u[0]
        for i, v in enumerate(u):
            acc += a * (v - acc)
            out[i] = acc
        return out

    def _design(self, d, tau_h, tau_c):
        T = d["t_meas"].to_numpy()
        hf = self._filter(d["heater"].to_numpy(), tau_h, self.dt)
        cf = self._filter(d["cooler"].to_numpy(), tau_c, self.dt)
        v, to, rh, s = (d[c].to_numpy() for c in ("vent", "t_out", "rh_out", "solar"))
        X = np.array([_features(T[k], T[k - 1], hf[k], cf[k], v[k], to[k], rh[k], s[k])
                      for k in range(1, len(T) - 1)])
        y = T[2:] - T[1:-1]
        return X, y, (hf, cf)

    def _solve(self, X, y):
        scale = np.maximum(np.abs(X).max(axis=0), 1e-6)
        A = X / scale
        coef = np.linalg.solve(A.T @ A + self.alpha * np.eye(A.shape[1]), A.T @ y)
        return coef, scale

    def fit(self, df, validation_fraction: float = 0.25) -> "LearnedThermalModel":
        """Identify actuator lags (grid search) and coefficients; report validation metrics."""
        d = self.resample(df, self.dt)
        n_val = max(int(len(d) * validation_fraction), 10)
        train, val = d.iloc[:-n_val].reset_index(drop=True), d.iloc[-n_val:].reset_index(drop=True)
        best = None
        for tau_h in LAG_GRID:                  # identify actuator lags by validation error
            for tau_c in LAG_GRID:
                X, y, _ = self._design(train, tau_h, tau_c)
                coef, scale = self._solve(X, y)
                Xv, yv, _ = self._design(val, tau_h, tau_c)
                err = float(np.sqrt(np.mean(((Xv / scale) @ coef - yv) ** 2)))
                if best is None or err < best[0]:
                    best = (err, tau_h, tau_c)
        _, self.tau_heater, self.tau_cooler = best
        X, y, _ = self._design(train, self.tau_heater, self.tau_cooler)
        self.coef, self.scale = self._solve(X, y)
        if self.residual == "mlp":
            from sklearn.neural_network import MLPRegressor

            A = X / self.scale
            self._mlp = MLPRegressor(hidden_layer_sizes=(32, 32), alpha=1e-3, max_iter=800,
                                     early_stopping=True, random_state=0).fit(A, y - A @ self.coef)
        Xv, yv, _ = self._design(val, self.tau_heater, self.tau_cooler)
        pred = np.array([self._delta_from_features(list(x)) for x in Xv])
        err = pred - yv
        self.metrics = {
            "one_step_rmse_K": float(np.sqrt(np.mean(err ** 2))),
            "one_step_r2": float(1 - np.sum(err ** 2) / max(np.sum((yv - yv.mean()) ** 2), 1e-12)),
            "rollout_1h_rmse_K": self._rollout_error(val, horizon_s=3600.0),
            "tau_heater_s": self.tau_heater,
            "tau_cooler_s": self.tau_cooler,
            "n_samples": int(len(d)),
        }
        return self

    def _rollout_error(self, d, horizon_s: float) -> float:
        """Open-loop multi-step error - the quantity that matters for MPC."""
        n = int(horizon_s / self.dt)
        T = d["t_meas"].to_numpy()
        cols = [d[c].to_numpy() for c in ("heater", "cooler", "vent", "t_out", "rh_out", "solar")]
        errs = []
        for start in range(1, len(T) - n, max(n // 2, 1)):
            st = ModelState(T[start], T[start - 1], cols[0][start - 1], cols[1][start - 1])
            for k in range(start, start + n):
                st = self.step(st, *(c[k] for c in cols))
            errs.append(st.T - T[start + n])
        return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")

    # ---------------------------------------------------------- prediction
    def _delta_from_features(self, f: List[float]) -> float:
        out = sum(fi / si * ci for fi, si, ci in zip(f, self.scale, self.coef))
        if self._mlp is not None:
            out += float(self._mlp.predict(np.array([f]) / self.scale)[0])
        return out

    def step(self, st: ModelState, heater, cooler, vent, t_out, rh_out, solar) -> ModelState:
        """Advance one model step (dt). Fast pure-python path for MPC rollouts."""
        ah = self.dt / (self.tau_heater + self.dt) if self.tau_heater > 0 else 1.0
        ac = self.dt / (self.tau_cooler + self.dt) if self.tau_cooler > 0 else 1.0
        hf = st.hf + ah * (heater - st.hf)
        cf = st.cf + ac * (cooler - st.cf)
        dT = self._delta_from_features(_features(st.T, st.T_prev, hf, cf, vent, t_out, rh_out, solar))
        return ModelState(st.T + dT, st.T, hf, cf)

    def simulate(self, st: ModelState, u: Sequence[Tuple], weather: Sequence[Tuple]) -> np.ndarray:
        """Open-loop rollout. u: [(heater, cooler, vent)], weather: [(t_out, rh_out, solar)]."""
        out = []
        for uk, wk in zip(u, weather):
            st = self.step(st, *uk, *wk)
            out.append(st.T)
        return np.array(out)

    def explain(self) -> Dict[str, float]:
        """Physical interpretation of the linear part: K per model step per unit regressor."""
        return {name: float(c / s) for name, c, s in zip(FEATURES, self.coef, self.scale)}

    # --------------------------------------------------------- persistence
    def save(self, path) -> None:
        """Write the linear model to JSON (coefficients, scaling, lags, metrics)."""
        if self._mlp is not None:
            raise NotImplementedError("persisting the MLP residual: use joblib on model._mlp")
        Path(path).write_text(json.dumps({
            "dt": self.dt, "alpha": self.alpha, "tau_heater": self.tau_heater, "tau_cooler": self.tau_cooler,
            "coef": self.coef.tolist(), "scale": self.scale.tolist(), "features": FEATURES,
            "metrics": self.metrics}, indent=2))

    @classmethod
    def load(cls, path) -> "LearnedThermalModel":
        """Read a model written by save()."""
        d = json.loads(Path(path).read_text())
        return cls(dt=d["dt"], alpha=d["alpha"], tau_heater=d["tau_heater"], tau_cooler=d["tau_cooler"],
                   coef=np.array(d["coef"]), scale=np.array(d["scale"]), metrics=d.get("metrics", {}))


def excitation_experiment(scenario, hours: float = 48.0, seed: int = 0):
    """Identification experiment on the digital twin: a PI loop tracking a
    randomly stepped setpoint plus a pseudo-random dither on the demand.
    On a real site run the same experiment during commissioning (with the
    safety supervisor active) to collect informative data."""
    from ..control.base import Controller
    from ..control.pid import PIDController
    from ..sim.runner import run_scenario

    rng = np.random.default_rng(seed)

    class Dithered(Controller):
        """PI loop with random setpoint offsets and a random dither on the demand."""
        name = "excitation"

        def __init__(self):
            self.pid = PIDController()
            self.hold, self.dither, self.offset = 0.0, 0.0, 0.0

        def update(self, ctx):
            """PI demand for a randomly offset setpoint plus a random dither."""
            if ctx.t_s >= self.hold:
                self.hold = ctx.t_s + rng.uniform(300, 1800)
                self.dither = rng.choice([-0.4, -0.2, 0.0, 0.2, 0.4])
                self.offset = rng.uniform(-3, 3)
            ctx.setpoint += self.offset
            return float(np.clip(self.pid.update(ctx) + self.dither, -1, 1))

    # keep the normal disturbances (lights, doors, animals): real commissioning data contains them
    sc = scenario.copy(duration_h=hours, faults={})
    return run_scenario(sc, controller=Dithered(), seed=seed).log
