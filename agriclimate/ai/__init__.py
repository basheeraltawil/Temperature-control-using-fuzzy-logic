"""AI components. Heavy / optional dependencies are imported lazily:
scikit-learn (anomaly detection, MLP residual) and anthropic (LLM assistant)."""
from .sysid import LearnedThermalModel, excitation_experiment

__all__ = ["LearnedThermalModel", "excitation_experiment"]
