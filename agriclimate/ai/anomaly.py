"""Sensor and actuator fault detection (predictive maintenance).

For every redundant temperature sensor the detector builds a feature vector
over a sliding window from the *model residual* (measured change minus the
change the learned thermal model expects) and the raw signal statistics:

    mean residual   -> drift / slow bias
    cumulative res. -> drift over the longer window
    residual std    -> noise, intermittent contacts
    max |residual|  -> spikes (EMI, loose connector)
    signal std      -> 0 for a stuck / frozen transmitter

An IsolationForest learns the healthy distribution from fault-free data (it
needs no labelled faults - important because real fault data is rare).
Anomalous sensors are handed to the safety supervisor, which excludes them
from the sensor fusion.  A common-mode negative residual while the heater is
driven hard is reported as an actuator fault (burner/boiler/valve failure).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import numpy as np


@dataclass
class AnomalyReport:
    suspect_sensors: List[int]
    scores: List[float]
    actuator_fault: Optional[str] = None


@dataclass
class AnomalyDetector:
    model: object                          # LearnedThermalModel
    window: int = 30                       # samples of model.dt (30 min at 60 s)
    contamination: float = 0.001           # quantile of healthy scores used for the threshold
    margin: float = 0.05                   # extra distance below that quantile
    threshold: float = float("-inf")
    flat_threshold: float = 0.0
    heater_threshold: float = float("-inf")
    confirm: int = 5                       # consecutive anomalous evaluations before flagging
    release: int = 30                      # consecutive healthy evaluations before clearing
    heater_fault_threshold_K: float = 1.0  # cumulative shortfall over the window
    seed: int = 0
    _forest: object = None
    _hist: List[Deque] = field(default_factory=list)
    _count: List[int] = field(default_factory=list)
    _healthy: List[int] = field(default_factory=list)
    _flagged: set = field(default_factory=set)
    _prev: Optional[tuple] = None
    _filt: tuple = (0.0, 0.0)
    _next_t: float = 0.0
    _common: Deque = field(default_factory=lambda: deque(maxlen=30))

    # ---------------------------------------------------------- features
    def _features(self, hist) -> np.ndarray:
        r = np.array([h[0] for h in hist])
        x = np.array([h[1] for h in hist])
        # [recent mean residual, window sum, residual std, max |residual|, signal step std]
        return np.array([r[-4:].mean(), r.sum(), r.std(), np.abs(r).max(), np.diff(x).std() if len(x) > 2 else 0.0])

    def _residuals(self, log):
        """Per-sensor one-step residuals from a runner log (resampled to model.dt)."""
        dt = self.model.dt
        d = log.copy()
        d["bin"] = (d["t_s"] // dt).astype(int)
        cols = [c for c in d.columns if c.startswith("sensor_")]
        g = d.groupby("bin").agg({**{c: "first" for c in cols}, "heater": "mean", "cooler": "mean",
                                  "vent": "mean", "t_out": "mean", "rh_out": "mean", "solar": "mean"})
        return g, cols

    def _predict(self, T, T_prev, filt, u):
        from .sysid import ModelState

        st = self.model.step(ModelState(T, T_prev, *filt), *u)
        return st.T - T, (st.hf, st.cf)

    def fit(self, healthy_log) -> "AnomalyDetector":
        from sklearn.ensemble import IsolationForest

        g, cols = self._residuals(healthy_log)
        U = g[["heater", "cooler", "vent", "t_out", "rh_out", "solar"]].to_numpy()
        feats, heat_sums = [], []
        for c in cols:
            x = g[c].to_numpy()
            hist: Deque = deque(maxlen=self.window)
            filt = (U[0, 0], U[0, 1])
            for k in range(2, len(x)):
                pred, filt = self._predict(x[k - 1], x[k - 2], filt, U[k - 1])
                hist.append((x[k] - x[k - 1] - pred, x[k], U[k - 1, 0]))
                if len(hist) == self.window:
                    feats.append(self._features(hist))
                    if min(h[2] for h in hist) > 0.6:
                        heat_sums.append(sum(h[0] for h in hist))
        feats = np.array(feats)
        self._forest = IsolationForest(n_estimators=200, random_state=self.seed).fit(feats)
        # calibrate the decision threshold on healthy data: a false alarm costs the
        # operator's trust, so sit below the lowest healthy scores with a margin
        healthy_scores = self._forest.score_samples(feats)
        self.threshold = float(np.quantile(healthy_scores, self.contamination)) - self.margin
        # deterministic companions to the forest (hybrid AI + rules):
        #  * a frozen transmitter has (almost) no sample-to-sample variation
        #  * a weak heater leaves a persistent negative residual while driven hard
        self.flat_threshold = 0.3 * float(np.quantile(feats[:, 4], self.contamination))
        base = float(np.min(heat_sums)) if heat_sums else 0.0
        self.heater_threshold = min(base, 0.0) - self.heater_fault_threshold_K
        return self

    # ----------------------------------------------------------- online
    def update(self, t_s: float, readings: List[float], heater: float, cooler: float, vent: float,
               t_out: float, rh_out: float, solar: float) -> Optional[AnomalyReport]:
        """Call every control step; evaluates once per model.dt. Returns a report when evaluated."""
        if self._forest is None or t_s < self._next_t:
            return None
        self._next_t = t_s + self.model.dt
        n = len(readings)
        if not self._hist:
            self._hist = [deque(maxlen=self.window) for _ in range(n)]
            self._count, self._healthy = [0] * n, [0] * n
        scores = [0.0] * n
        common = []
        if self._prev is not None:
            prev, prev2, u = self._prev
            pred_filt = self._filt
            for i, x in enumerate(readings):
                if not np.isfinite(x) or not np.isfinite(prev[i]) or not np.isfinite(prev2[i]):
                    continue
                pred, pred_filt = self._predict(prev[i], prev2[i], self._filt, u)
                res = x - prev[i] - pred
                if i not in self._flagged and abs(res) < 1.0:   # healthy channels only
                    common.append(res)
                self._hist[i].append((res, x, heater))
                if len(self._hist[i]) == self.window:
                    f = self._features(self._hist[i])
                    scores[i] = float(self._forest.score_samples(f[None, :])[0])
                    if scores[i] < self.threshold or f[4] < self.flat_threshold:
                        self._count[i] += 1
                        self._healthy[i] = 0
                    else:
                        self._count[i] = 0
                        self._healthy[i] += 1
                    if self._count[i] >= self.confirm:
                        self._flagged.add(i)
                    elif self._healthy[i] >= self.release:
                        self._flagged.discard(i)
        if self._prev is not None:
            self._filt = pred_filt
        prev2 = self._prev[0] if self._prev is not None else list(readings)
        self._prev = (list(readings), prev2, (heater, cooler, vent, t_out, rh_out, solar))

        actuator = None
        if common:
            self._common.append((float(np.median(common)), heater))
            if (len(self._common) == self._common.maxlen and min(h for _, h in self._common) > 0.6
                    and sum(r for r, _ in self._common) < self.heater_threshold):
                actuator = "HEATER_UNDERPERFORMING"
        # A fault in one transmitter affects only that channel. If *every* channel is
        # anomalous the cause is common-mode (process change, model mismatch,
        # actuator fault) - excluding sensors would then be wrong.
        suspects = sorted(self._flagged)
        if n > 1 and len(suspects) >= n:
            suspects = []
            if actuator is None:
                actuator = "COMMON_MODE_DEVIATION"
        return AnomalyReport(suspects, scores, actuator)
