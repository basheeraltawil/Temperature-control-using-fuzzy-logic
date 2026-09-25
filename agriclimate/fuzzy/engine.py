"""A small, dependency-free (numpy only) Mamdani fuzzy inference engine.

It reproduces the MATLAB Fuzzy Logic Toolbox semantics used by the original
``temperature_controlling1.fis`` (min AND, max OR, min implication, max
aggregation, centroid defuzzification) so the same rule base can run on a PC,
inside ROS 2, or be compiled to a lookup table for a PLC / microcontroller.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .membership import evaluate_mf


@dataclass
class MembershipFunction:
    """Named membership function, e.g. ('ZE', 'trimf', [-1, 0, 1])."""
    name: str
    type: str
    params: List[float]

    def __call__(self, x):
        return evaluate_mf(self.type, x, self.params)


@dataclass
class Variable:
    """Linguistic variable: name, universe of discourse and its membership functions."""
    name: str
    range: Sequence[float]
    mfs: List[MembershipFunction] = field(default_factory=list)

    def mf_index(self, name: str) -> int:
        """1-based index (FIS convention) of the MF called ``name``."""
        for i, mf in enumerate(self.mfs):
            if mf.name == name:
                return i + 1
        raise KeyError(f"{self.name} has no membership function '{name}'")


@dataclass
class Rule:
    """IF-THEN rule in MATLAB index form."""
    antecedent: List[int]   # 1-based MF index per input, 0 = don't care, negative = NOT
    consequent: List[int]   # 1-based MF index per output, 0 = rule does not drive that output
    weight: float = 1.0
    connective: int = 1     # 1 = AND, 2 = OR


@dataclass
class InferenceResult:
    """Crisp outputs, rule firing strengths and whether any rule fired."""
    outputs: np.ndarray
    firing: np.ndarray       # firing strength of every rule
    fired: bool              # False when no rule fired for at least one output


class MamdaniFIS:
    """Mamdani fuzzy inference system (fuzzify -> rule strengths -> implication -> aggregation -> defuzzify)."""
    def __init__(
        self,
        name: str,
        inputs: List[Variable],
        outputs: List[Variable],
        rules: List[Rule],
        and_method: str = "min",
        or_method: str = "max",
        imp_method: str = "min",
        agg_method: str = "max",
        defuzz_method: str = "centroid",
        resolution: int = 501,
    ):
        for m, ok in ((and_method, ("min", "prod")), (or_method, ("max", "probor")),
                      (imp_method, ("min", "prod")), (agg_method, ("max", "sum")),
                      (defuzz_method, ("centroid", "bisector", "mom"))):
            if m not in ok:
                raise ValueError(f"Unsupported method '{m}', expected one of {ok}")
        self.name = name
        self.inputs = inputs
        self.outputs = outputs
        self.rules = rules
        self.and_method = and_method
        self.or_method = or_method
        self.imp_method = imp_method
        self.agg_method = agg_method
        self.defuzz_method = defuzz_method
        self.resolution = resolution
        self._compile()

    # ------------------------------------------------------------------ setup
    def _compile(self) -> None:
        n_in, n_out = len(self.inputs), len(self.outputs)
        for r in self.rules:
            if len(r.antecedent) != n_in or len(r.consequent) != n_out:
                raise ValueError(f"Rule {r} does not match {n_in} inputs / {n_out} outputs")
        self._ante = np.array([r.antecedent for r in self.rules], dtype=int).reshape(-1, n_in)
        self._cons = np.array([r.consequent for r in self.rules], dtype=int).reshape(-1, n_out)
        self._weights = np.array([r.weight for r in self.rules], dtype=float)
        self._is_or = np.array([r.connective == 2 for r in self.rules], dtype=bool)
        self._universe = []
        self._out_mf = []   # per output: (n_mfs, resolution)
        for var in self.outputs:
            u = np.linspace(var.range[0], var.range[1], self.resolution)
            self._universe.append(u)
            self._out_mf.append(np.vstack([mf(u) for mf in var.mfs]) if var.mfs else np.zeros((0, u.size)))

    # -------------------------------------------------------------- inference
    def _fuzzify(self, x: Sequence[float]) -> List[np.ndarray]:
        degrees = []
        for var, xi in zip(self.inputs, x):
            xi = float(np.clip(xi, var.range[0], var.range[1]))
            degrees.append(np.array([float(mf(xi)) for mf in var.mfs]))
        return degrees

    def firing_strengths(self, x: Sequence[float]) -> np.ndarray:
        """Degree of fulfilment of every rule for the crisp inputs x."""
        degrees = self._fuzzify(x)
        n_rules, n_in = self._ante.shape
        mu = np.empty((n_rules, n_in))
        for j in range(n_in):
            idx = self._ante[:, j]
            d = np.concatenate(([np.nan], degrees[j]))[np.abs(idx)]
            d = np.where(idx < 0, 1.0 - d, d)
            # "don't care" is neutral for the connective in use
            mu[:, j] = np.where(idx == 0, np.where(self._is_or, 0.0, 1.0), d)
        if self.and_method == "min":
            w_and = mu.min(axis=1)
        else:
            w_and = mu.prod(axis=1)
        if self.or_method == "max":
            w_or = mu.max(axis=1)
        else:
            w_or = 1.0 - np.prod(1.0 - mu, axis=1)
        return np.where(self._is_or, w_or, w_and) * self._weights

    def evaluate(self, x: Sequence[float], default: Optional[Sequence[float]] = None) -> InferenceResult:
        """Evaluate the FIS.  ``default`` is returned for an output when no rule
        fires (MATLAB returns the mid-range; control code should pass a safe value)."""
        if len(x) != len(self.inputs):
            raise ValueError(f"Expected {len(self.inputs)} inputs, got {len(x)}")
        w = self.firing_strengths(x)
        out = np.empty(len(self.outputs))
        all_fired = True
        for k, var in enumerate(self.outputs):
            cons = self._cons[:, k]
            active = (cons != 0) & (w > 0)
            u = self._universe[k]
            if not np.any(active):
                all_fired = False
                out[k] = (0.5 * (var.range[0] + var.range[1]) if default is None else default[k])
                continue
            mfs = self._out_mf[k][np.abs(cons[active]) - 1]
            mfs = np.where(cons[active, None] < 0, 1.0 - mfs, mfs)
            wa = w[active, None]
            implied = np.minimum(wa, mfs) if self.imp_method == "min" else wa * mfs
            agg = implied.max(axis=0) if self.agg_method == "max" else np.minimum(implied.sum(axis=0), 1.0)
            out[k] = _defuzzify(u, agg, self.defuzz_method)
        return InferenceResult(out, w, all_fired)

    def __call__(self, *x: float) -> np.ndarray:
        return self.evaluate(x).outputs

    # ---------------------------------------------------------------- analysis
    def coverage_gaps(self, n: int = 101) -> Dict[str, float]:
        """Fraction of a 2-input grid where no rule fires for each output.
        Uncovered regions are a classic source of erratic behaviour in the field."""
        if len(self.inputs) != 2:
            raise ValueError("coverage_gaps supports 2-input systems")
        a = np.linspace(*self.inputs[0].range, n)
        b = np.linspace(*self.inputs[1].range, n)
        gaps = np.zeros(len(self.outputs))
        for xa in a:
            for xb in b:
                w = self.firing_strengths((xa, xb))
                for k in range(len(self.outputs)):
                    if not np.any((self._cons[:, k] != 0) & (w > 0)):
                        gaps[k] += 1
        return {v.name: g / (n * n) for v, g in zip(self.outputs, gaps)}

    def surface(self, output: int = 0, n: int = 41):
        """Control surface over a 2-input grid -> (X, Y, Z)."""
        a = np.linspace(*self.inputs[0].range, n)
        b = np.linspace(*self.inputs[1].range, n)
        Z = np.empty((n, n))
        for i, xb in enumerate(b):
            for j, xa in enumerate(a):
                Z[i, j] = self.evaluate((xa, xb)).outputs[output]
        X, Y = np.meshgrid(a, b)
        return X, Y, Z


def _defuzzify(u: np.ndarray, mu: np.ndarray, method: str) -> float:
    area = mu.sum()
    if area <= 0:
        return float(0.5 * (u[0] + u[-1]))
    if method == "centroid":
        return float((u * mu).sum() / area)
    if method == "bisector":
        c = np.cumsum(mu)
        return float(u[np.searchsorted(c, c[-1] / 2.0)])
    # mean of maximum
    return float(u[mu >= mu.max() - 1e-12].mean())
