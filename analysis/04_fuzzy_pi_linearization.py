"""04 - What the fuzzy-PI does: local linear gains and nonlinearity.

The fuzzy-PI output increment is  du = F(e_n, de_n)  with  e_n = ke*e,  de_n = kr*de/dt.
Near the origin F is approximately linear:  F ~ a*e_n + b*de_n, so the controller
behaves like a PI controller

    du/dt = ku * F   =>   u ~ ku*kr*b * e  +  ku*ke*a * integral(e) dt
    Kp_eq = ku * kr * b,        Ki_eq = ku * ke * a,        Ti_eq = Kp_eq / Ki_eq

a and b are the slopes of the surface at the origin (finite differences). Away from the
origin the local slope varies (ripple from the triangular terms) and the output saturates
for large errors; this nonlinearity is what distinguishes it from a linear PI.

Outputs: output/04_fuzzy_surface.png, output/04_fuzzy_linearization.md
"""
import matplotlib.pyplot as plt
import numpy as np

from _common import OUT, md_table, save_text
from agriclimate.control.fuzzy_pi import FuzzyPIController

c = FuzzyPIController(use_lut=False)
F = lambda e, de: float(c.fis(e, de)[0])  # noqa: E731
h = 0.02
a = (F(h, 0) - F(-h, 0)) / (2 * h)
b = (F(0, h) - F(0, -h)) / (2 * h)
kp, ki = c.ku * c.kr * b, c.ku * c.ke * a

e = np.linspace(-1, 1, 81)
fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
for de in (-0.6, -0.3, 0.0, 0.3, 0.6):
    ax[0].plot(e, [F(x, de) for x in e], label=f"de_n = {de}")
ax[0].plot(e, np.clip(a * e, -1, 1), "k:", label="linear approximation (de_n = 0)")
ax[0].set_xlabel("normalised error e_n")
ax[0].set_ylabel("output increment du")
ax[0].legend(fontsize=8)
ax[0].set_title("fuzzy-PI surface slices")
slope = np.gradient([F(x, 0.0) for x in e], e)
ax[1].plot(e, slope)
ax[1].set_xlabel("normalised error e_n")
ax[1].set_ylabel("local gain dF/de_n")
ax[1].set_title("gain scheduling: local slope vs error")
for x in ax:
    x.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "04_fuzzy_surface.png", dpi=110)

text = "# 04 Fuzzy-PI linearisation\n\n"
text += md_table([("slope a = dF/de_n", f"{a:.3f}"), ("slope b = dF/d(de_n)", f"{b:.3f}"),
                  ("Kp_eq = ku*kr*b [1/K]", f"{kp:.3f}"), ("Ki_eq = ku*ke*a [1/(K s)]", f"{ki:.2e}"),
                  ("Ti_eq = Kp_eq/Ki_eq [s]", f"{kp / ki:.0f}")], ["quantity", "value"])
text += f"\nDefault gains: ke = {c.ke}, kr = {c.kr}, ku = {c.ku}. Compare with the default PID: Kp = 0.35, Ti = 770 s.\n"
save_text("04_fuzzy_linearization.md", text)
