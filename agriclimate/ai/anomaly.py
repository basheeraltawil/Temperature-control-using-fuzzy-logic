"""Sensor and actuator fault detection.

Idea: a healthy sensor changes the way the learned thermal model predicts.
For each redundant temperature sensor we compute the one-step *residual*

    r_k = (x_k - x_{k-1}) - dT_model(x_{k-1}, x_{k-2}, u_{k-1}, weather_{k-1})

and summarise the last ``window`` residuals in five features:

    ====================  =====================================
    feature               typical fault it reveals
    ====================  =====================================
    mean of last 4 r      sudden offset, fast drift
    sum of r              slow drift over the window
    std of r              noise, intermittent contact
    max |r|               spikes (EMI, loose connector)
    std of x_k - x_{k-1}  ~0 for a frozen transmitter
    ====================  =====================================

An IsolationForest is trained on these features from *healthy* data only,
so no labelled fault data is needed. Its decision threshold is set just
below the lowest healthy scores to keep false alarms rare. Three rules,
calibrated on the same healthy data, complement it: a flat-line check, a
drift check on the residual sum (tree models extrapolate poorly, so a slow
drift can stay inside the forest's healthy range) and a
heater-underperformance check.

A sensor is flagged after ``confirm`` anomalous evaluations in a row and
released after ``release`` healthy ones. If *all* sensors look anomalous the
cause is common-mode (process or actuator), so no sensor is excluded.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

INPUT_COLUMNS = ["heater", "cooler", "vent", "t_out", "rh_out", "solar"]


class Sample(NamedTuple):
    """One step of one sensor inside the sliding window."""
    residual: float      # measured change minus predicted change, K
    value: float         # sensor reading, degC
    heater: float        # heater command during the step, 0..1


@dataclass
class AnomalyReport:
    """Result of one evaluation of the anomaly detector."""
    suspect_sensors: List[int]           # 0-based indices to exclude from fusion
    scores: List[float]                  # IsolationForest score per sensor (lower = more anomalous)
    actuator_fault: Optional[str] = None


def window_features(window: Sequence[Sample]) -> np.ndarray:
    """The five features described in the module docstring."""
    r = np.array([s.residual for s in window])
    x = np.array([s.value for s in window])
    step_std = np.diff(x).std() if len(x) > 2 else 0.0
    return np.array([r[-4:].mean(), r.sum(), r.std(), np.abs(r).max(), step_std])


@dataclass
class AnomalyDetector:
    """IsolationForest + learned-model residuals; see the module docstring."""
    model: object                          # agriclimate.ai.sysid.LearnedThermalModel
    window: int = 30                       # samples of model.dt (30 min at 60 s)
    threshold_quantile: float = 0.001      # healthy-score quantile used for the threshold
    margin: float = 0.05                   # extra distance below that quantile
    confirm: int = 5                       # anomalous evaluations in a row before flagging
    release: int = 30                      # healthy evaluations in a row before releasing
    heater_fault_margin_K: float = 1.0     # extra shortfall beyond the healthy minimum
    seed: int = 0

    # calibrated by fit()
    threshold: float = field(default=float("-inf"), init=False)
    flat_threshold: float = field(default=0.0, init=False)
    drift_threshold: float = field(default=float("inf"), init=False)
    heater_threshold: float = field(default=float("-inf"), init=False)

    # online state
    _forest: object = field(default=None, init=False, repr=False)
    _windows: List[Deque[Sample]] = field(default_factory=list, init=False, repr=False)
    _bad_count: List[int] = field(default_factory=list, init=False, repr=False)
    _good_count: List[int] = field(default_factory=list, init=False, repr=False)
    _flagged: set = field(default_factory=set, init=False, repr=False)
    _prev: Optional[Tuple[list, list, tuple]] = field(default=None, init=False, repr=False)
    _filt: Tuple[float, float] = field(default=(0.0, 0.0), init=False, repr=False)
    _next_t: float = field(default=0.0, init=False, repr=False)
    _common: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=30),
                                                init=False, repr=False)

    # ------------------------------------------------------------- helpers
    def _predicted_change(self, T, T_prev, filt, u):
        """Model prediction of the next temperature change and the updated actuator filters."""
        from .sysid import ModelState

        st = self.model.step(ModelState(T, T_prev, *filt), *u)
        return st.T - T, (st.hf, st.cf)

    def _resample(self, log):
        """Resample a runner log to the model time step (sensor columns: first value per bin)."""
        d = log.copy()
        d["bin"] = (d["t_s"] // self.model.dt).astype(int)
        sensors = [c for c in d.columns if c.startswith("sensor_")]
        agg = {**{c: "first" for c in sensors}, **{c: "mean" for c in INPUT_COLUMNS}}
        return d.groupby("bin").agg(agg), sensors

    # ---------------------------------------------------------------- fit
    def fit(self, healthy_log) -> "AnomalyDetector":
        """Train on a log of fault-free operation (columns sensor_1.., heater, ..., solar)."""
        from sklearn.ensemble import IsolationForest

        g, sensors = self._resample(healthy_log)
        U = g[INPUT_COLUMNS].to_numpy()
        feats, heater_sums = [], []
        for col in sensors:
            x = g[col].to_numpy()
            win: Deque[Sample] = deque(maxlen=self.window)
            filt = (U[0, 0], U[0, 1])
            for k in range(2, len(x)):
                pred, filt = self._predicted_change(x[k - 1], x[k - 2], filt, U[k - 1])
                win.append(Sample(x[k] - x[k - 1] - pred, x[k], U[k - 1, 0]))
                if len(win) == self.window:
                    feats.append(window_features(win))
                    if min(s.heater for s in win) > 0.6:
                        heater_sums.append(sum(s.residual for s in win))
        feats = np.array(feats)
        self._forest = IsolationForest(n_estimators=200, random_state=self.seed).fit(feats)

        # Thresholds sit below everything seen in healthy operation.
        healthy_scores = self._forest.score_samples(feats)
        self.threshold = float(np.quantile(healthy_scores, self.threshold_quantile)) - self.margin
        self.flat_threshold = 0.3 * float(np.quantile(feats[:, 4], self.threshold_quantile))
        self.drift_threshold = 1.5 * float(np.max(np.abs(feats[:, 1])))
        lowest_healthy = min(float(np.min(heater_sums)), 0.0) if heater_sums else 0.0
        self.heater_threshold = lowest_healthy - self.heater_fault_margin_K
        return self

    # -------------------------------------------------------------- online
    def _evaluate_sensor(self, i: int, win: Deque[Sample]) -> float:
        """Score one full window and update the flag counters of sensor i."""
        f = window_features(win)
        score = float(self._forest.score_samples(f[None, :])[0])
        anomalous = (score < self.threshold or f[4] < self.flat_threshold
                     or abs(f[1]) > self.drift_threshold)
        if anomalous:
            self._bad_count[i] += 1
            self._good_count[i] = 0
        else:
            self._bad_count[i] = 0
            self._good_count[i] += 1
        if self._bad_count[i] >= self.confirm:
            self._flagged.add(i)
        elif self._good_count[i] >= self.release:
            self._flagged.discard(i)
        return score

    def update(self, t_s: float, readings: List[float], heater: float, cooler: float, vent: float,
               t_out: float, rh_out: float, solar: float) -> Optional[AnomalyReport]:
        """Call every control cycle. Evaluates once per model.dt and then returns a report."""
        if self._forest is None or t_s < self._next_t:
            return None
        self._next_t = t_s + self.model.dt
        n = len(readings)
        if not self._windows:
            self._windows = [deque(maxlen=self.window) for _ in range(n)]
            self._bad_count, self._good_count = [0] * n, [0] * n

        scores = [0.0] * n
        healthy_residuals, all_residuals = [], []
        if self._prev is not None:
            prev, prev2, u = self._prev
            new_filt = self._filt
            for i, x in enumerate(readings):
                if not (np.isfinite(x) and np.isfinite(prev[i]) and np.isfinite(prev2[i])):
                    continue
                pred, new_filt = self._predicted_change(prev[i], prev2[i], self._filt, u)
                res = x - prev[i] - pred
                if abs(res) < 1.0:
                    all_residuals.append(res)
                    if i not in self._flagged:
                        healthy_residuals.append(res)
                self._windows[i].append(Sample(res, x, heater))
                if len(self._windows[i]) == self.window:
                    scores[i] = self._evaluate_sensor(i, self._windows[i])
            self._filt = new_filt
        prev2 = self._prev[0] if self._prev is not None else list(readings)
        self._prev = (list(readings), prev2, (heater, cooler, vent, t_out, rh_out, solar))

        # Heater check: heater driven hard for a whole window, yet the sensors warm up
        # much less than the model expects. Use the unflagged sensors; if all are
        # flagged (common-mode deviation) use all of them.
        actuator = None
        residuals = healthy_residuals or all_residuals
        if residuals:
            self._common.append((float(np.median(residuals)), heater))
            window_full = len(self._common) == self._common.maxlen
            if (window_full and min(h for _, h in self._common) > 0.6
                    and sum(r for r, _ in self._common) < self.heater_threshold):
                actuator = "HEATER_UNDERPERFORMING"

        # One transmitter fault affects one channel. All channels anomalous means
        # a common cause (process, model mismatch, actuator): exclude nothing.
        suspects = sorted(self._flagged)
        if n > 1 and len(suspects) >= n:
            suspects = []
            actuator = actuator or "COMMON_MODE_DEVIATION"
        return AnomalyReport(suspects, scores, actuator)
