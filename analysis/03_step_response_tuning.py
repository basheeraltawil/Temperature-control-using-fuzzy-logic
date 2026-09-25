"""03 - Open-loop step test, FOPDT model fit and PI tuning rules.

A classic commissioning procedure, run on the digital twin:

  1. Hold constant weather, apply a heater step 0 -> 40 % and record T(t).
  2. Fit a first-order-plus-dead-time model   G(s) = K e^{-theta s} / (tau s + 1)
     (least squares on the step response   T(t) = T0 + K * du * (1 - exp(-(t - theta)/tau))).
  3. Derive PI gains with the SIMC rule (Skogestad 2003) for a chosen closed-loop time tau_c:
         Kc = tau / (K (tau_c + theta)),   Ti = min(tau, 4 (tau_c + theta))
  4. Compare with the gains found by the automatic tuner (agriclimate.ai.tuner).

Outputs: output/03_step_response.png, output/03_step_response.md
"""
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

from _common import OUT, md_table, save_text
from agriclimate.control.pid import PIDController
from agriclimate.plant import ActuatorCommand, Facility, WeatherSample, get_preset

DT = 30.0
rows, fig = [], plt.figure(figsize=(12, 7))
cases = [("greenhouse_glass", 5.0, 0.4), ("germination_chamber", 10.0, 0.4), ("cold_storage", 20.0, 0.4)]
for n, (preset, t_out, du) in enumerate(cases, 1):
    plant = Facility(get_preset(preset), t_init=t_out, rh_init=70)
    w = WeatherSample(t_out, 70.0, 0.0, 2.0)                  # constant weather, night
    for _ in range(int(12 * 3600 / DT)):                       # settle with actuators off
        plant.step(DT, ActuatorCommand(), w)
    T0 = plant.state.t_air
    t = np.arange(0, 10 * 3600, DT)
    T = []
    for _ in t:
        plant.step(DT, ActuatorCommand(heater=du), w)
        T.append(plant.state.t_air)
    T = np.array(T)

    def fopdt(tt, K, tau, theta):
        return T0 + K * du * (1 - np.exp(-np.clip(tt - theta, 0, None) / tau))

    (K, tau, theta), _ = curve_fit(fopdt, t, T, p0=[(T[-1] - T0) / du, 3600, 60],
                                   bounds=([0, 10, 0], [500, 1e6, 3600]))
    tau_c = max(theta, tau / 4)                                # moderately fast, robust choice
    kc = tau / (K * (tau_c + theta))
    ti = min(tau, 4 * (tau_c + theta))
    rows.append((preset, f"{K:.1f}", f"{tau / 60:.0f}", f"{theta:.0f}", f"{kc:.3f}", f"{ti:.0f}"))

    ax = fig.add_subplot(1, 3, n)
    ax.plot(t / 3600, T, label="twin")
    ax.plot(t / 3600, fopdt(t, K, tau, theta), "--", label="FOPDT fit")
    ax.set_title(f"{preset}\nK={K:.1f} K, tau={tau / 60:.0f} min, theta={theta:.0f} s", fontsize=9)
    ax.set_xlabel("time [h]")
    ax.set_ylabel("air temperature [degC]")
    ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(OUT / "03_step_response.png", dpi=110)

d = PIDController()
text = "# 03 Step response and PI tuning\n\n"
text += "Heater step 0 -> 40 %, constant weather. SIMC with tau_c = max(theta, tau/4).\n\n"
text += md_table(rows, ["facility", "K [K per unit heater]", "tau [min]", "theta [s]", "SIMC Kc [1/K]", "SIMC Ti [s]"])
text += (f"\nAuto-tuned default PID (tomato greenhouse scenario, differential evolution): "
         f"Kp = {d.kp}, Ti = {d.ti:.0f} s.\n")
save_text("03_step_response.md", text)
