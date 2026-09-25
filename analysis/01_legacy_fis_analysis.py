"""01 - Analysis of the original MATLAB controller (temperature_controlling1.fis).

Questions answered:
  1. Coverage: for which (sensed, target) pairs does no rule fire?
  2. Overlap: where do the heater (P-PWM) and cooler (N-PWM) outputs run together?
  3. Behaviour at the setpoint: what does the controller output when sensed == target?

Method: evaluate the FIS on a 101 x 101 grid over its input universe [-50, 50] degC.
Outputs: output/01_legacy_maps.png, output/01_legacy_summary.md
"""
import matplotlib.pyplot as plt
import numpy as np

from _common import OUT, md_table, save_text
from agriclimate.control.legacy_fis import DEFAULT_FIS
from agriclimate.fuzzy import load_fis

fis = load_fis(DEFAULT_FIS)
grid = np.linspace(-50, 50, 101)
no_rule = np.zeros((grid.size, grid.size), dtype=bool)
heat = np.zeros_like(no_rule, dtype=float)
cool = np.zeros_like(heat)
for i, target in enumerate(grid):
    for j, sensed in enumerate(grid):
        res = fis.evaluate((sensed, target), default=(0.0, 0.0))
        no_rule[i, j] = not res.fired
        heat[i, j], cool[i, j] = res.outputs
overlap = (heat > 0.1 * 255) & (cool > 0.1 * 255)          # both above 10 % duty

fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
ext = [grid[0], grid[-1], grid[0], grid[-1]]
for a, data, title in [(ax[0], heat, "heater P-PWM (0-255)"), (ax[1], cool, "cooler N-PWM (0-255)")]:
    im = a.imshow(data, origin="lower", extent=ext, cmap="viridis")
    fig.colorbar(im, ax=a)
    a.set_title(title)
ax[2].imshow(np.where(no_rule, 2, np.where(overlap, 1, 0)), origin="lower", extent=ext,
             cmap=plt.matplotlib.colors.ListedColormap(["#f7f7f7", "#f4a582", "#b2182b"]), vmin=0, vmax=2)
ax[2].set_title("red: no rule fires\norange: heater and cooler both > 10 %", fontsize=10)
for a in ax:
    a.plot(grid, grid, "k--", lw=0.8)                       # sensed == target line
    a.set_xlabel("sensed temperature [degC]")
    a.set_ylabel("target temperature [degC]")
fig.tight_layout()
fig.savefig(OUT / "01_legacy_maps.png", dpi=110)

diag = [(t, *fis.evaluate((t, t), default=(0.0, 0.0)).outputs) for t in (-30.0, -10.0, 0.0, 10.0, 20.0, 30.0)]
text = "# 01 Legacy FIS analysis\n\n"
text += f"* rules defined: {len(fis.rules)} of {len(fis.inputs[0].mfs) * len(fis.inputs[1].mfs)} input combinations\n"
text += f"* input plane with no firing rule: {100 * no_rule.mean():.1f} %\n"
text += f"* input plane with heater and cooler both above 10 % duty: {100 * overlap.mean():.1f} %\n\n"
text += "Output at the setpoint (sensed = target):\n\n"
text += md_table([(f"{t:.0f}", f"{p:.1f}", f"{n:.1f}") for t, p, n in diag],
                 ["temperature [degC]", "heater PWM", "cooler PWM"])
save_text("01_legacy_summary.md", text)
