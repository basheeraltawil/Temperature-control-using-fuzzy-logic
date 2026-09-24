"""Matplotlib reports for scenario comparisons."""
from __future__ import annotations

from pathlib import Path
from typing import List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"fuzzy_pi": "#1b7837", "fuzzy_pi+ai": "#5aae61", "pid": "#2166ac", "mpc": "#762a83",
          "mpc+ai": "#9970ab", "legacy_fis": "#d6604d"}


def plot_comparison(results: List, scenario, path) -> Path:
    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1.4, 1.4, 1.2]})
    first = results[0].log
    ax[0].plot(first["t_h"], first["setpoint"], "k--", lw=1.4, label="setpoint")
    ax[0].fill_between(first["t_h"], first["setpoint"] - scenario.comfort_band,
                       first["setpoint"] + scenario.comfort_band, color="k", alpha=0.07, label="comfort band")
    ax[0].plot(first["t_h"], first["t_out"], color="#999999", lw=1, label="outdoor")
    for r in results:
        ax[0].plot(r.log["t_h"], r.log["t_true"], lw=1.3, color=COLORS.get(r.controller),
                   label=f"{r.controller}  (RMSE {r.metrics['rmse_K']:.2f} K)")
    ax[0].set_ylabel("air temperature [degC]")
    ax[0].legend(loc="upper left", fontsize=8, ncol=2)
    ax[0].set_title(f"{scenario.name}: {scenario.description}", fontsize=10)

    best = min((r for r in results if r.controller != "legacy_fis"), key=lambda r: r.metrics["rmse_K"],
               default=results[0])
    lg = best.log
    ax[1].plot(lg["t_h"], lg["heater"], color="#d73027", label="heater")
    ax[1].plot(lg["t_h"], lg["cooler"], color="#4575b4", label="cooler")
    ax[1].plot(lg["t_h"], lg["vent"], color="#fdae61", label="vent")
    ax[1].set_ylabel(f"actuators\n({best.controller})")
    ax[1].set_ylim(-0.05, 1.05)
    ax[1].legend(loc="upper left", fontsize=8, ncol=3)

    ax[2].plot(lg["t_h"], lg["rh"], color="#35978f", label="indoor RH")
    ax[2].plot(lg["t_h"], lg["rh_out"], color="#999999", lw=0.8, label="outdoor RH")
    ax[2].set_ylabel("RH [%]")
    ax2b = ax[2].twinx()
    ax2b.plot(lg["t_h"], lg["solar"], color="#fee08b", lw=0.8)
    ax2b.set_ylabel("solar [W/m2]", color="#b8860b")
    ax[2].legend(loc="upper left", fontsize=8)

    for r in results:
        ax[3].plot(r.log["t_h"], r.log["e_heat_kwh"] + r.log["e_cool_kwh"] + r.log["e_fan_kwh"],
                   color=COLORS.get(r.controller), label=r.controller)
    ax[3].set_ylabel("energy [kWh]")
    ax[3].set_xlabel("time [h]")
    ax[3].legend(loc="upper left", fontsize=8, ncol=3)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_fis_surfaces(legacy_fis, new_fis, path) -> Path:
    fig = plt.figure(figsize=(14, 4.5))
    for i, (fis, out, title) in enumerate([(legacy_fis, 0, "legacy P-PWM (heater)"),
                                           (legacy_fis, 1, "legacy N-PWM (cooler)"),
                                           (new_fis, 0, "fuzzy-PI  delta_u")]):
        X, Y, Z = fis.surface(out, n=31)
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        ax.plot_surface(X, Y, Z, cmap="coolwarm", linewidth=0)
        ax.set_xlabel(fis.inputs[0].name)
        ax.set_ylabel(fis.inputs[1].name)
        ax.set_title(title, fontsize=10)
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path
