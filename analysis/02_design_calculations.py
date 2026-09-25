"""02 - Steady-state design calculations for every facility preset.

Hand calculations an engineer does before choosing equipment, applied to the
presets in agriclimate/plant/presets.py (formulas explained in docs/DESIGN_CALCULATIONS.md):

  envelope conductance      UA    = U * A_env                          [W/K]
  infiltration conductance  H_inf = rho * cp * V * ACH / 3600          [W/K]
  heat demand (winter)      Q_h   = (UA + H_inf) * (T_set - T_out) - Q_int
  heater margin             M     = eta * P_heater / Q_h
  air time constant         tau_a = C_air / (UA + H_inf + h_m * A_floor)
  solar gain (peak)         Q_sol = I * A_floor * tau_cover * (1 - shade) * (1 - f_latent)
  ventilation for dT = 4 K  V_dot = Q_sol / (rho * cp * 4)  ->  ACH = 3600 V_dot / V
  pad cooling capacity      Q_pad = rho * cp * V_pad * eta_pad * (T_out - T_wb)
  cooling load (summer)     Q_c   = (UA + H_inf) * (T_out - T_set) + Q_int + Q_sol

Operating envelope ("workspace"): required power to hold the design setpoint versus
outdoor temperature, compared with installed capacity:
  required   Q_req(T_out) = (UA + H_inf) (T_set - T_out) - Q_int     (night, no sun)
             (> 0 heating, < 0 cooling)
  heating    + eta * P_heater
  cooling    refrigeration  - P_cool
             pad cooling    - rho cp V_pad (T_set - T_supply),  T_supply = T_out - eta_pad (T_out - T_wb)
             vents          - rho cp V_vent,max (T_set - T_out)   (only when T_out < T_set)
The setpoint can be held where the required curve lies inside the capacity band.

Design conditions are typical Central-European values and are stated in the output.
Outputs: output/02_design_heating.md, output/02_design_cooling.md, output/02_operating_envelope.png
"""
import matplotlib.pyplot as plt
import numpy as np

from _common import OUT, md_table, save_text
from agriclimate.plant.presets import PRESETS
from agriclimate.plant.psychrometrics import CP_AIR, RHO_AIR, wet_bulb

RHO_CP = RHO_AIR * CP_AIR       # J/(m3 K)
LATENT_FRACTION = 0.45          # share of absorbed solar used for crop transpiration (greenhouses)

# preset: (winter T_out, winter setpoint, summer T_out, summer RH, summer setpoint, peak solar, extra summer gain W)
DESIGN = {
    "greenhouse_glass":    (-10, 17, 32, 40, 26, 850, 0),
    "polytunnel":          (-5, 5, 30, 45, 25, 800, 0),
    "germination_chamber": (0, 25, 35, 40, 25, 0, 0),
    "poultry_house":       (-10, 30, 33, 40, 21, 0, 60e3),   # birds near slaughter weight
    "cold_storage":        (-10, 2, 32, 45, 2, 0, 0),
    "mushroom_room":       (-10, 18, 30, 50, 18, 0, 0),
    "vertical_farm":       (-10, 17, 32, 45, 21, 0, 30e3),   # LEDs on
}

heat_rows, cool_rows = [], []
for name, (t_w, sp_w, t_s, rh_s, sp_s, solar, extra) in DESIGN.items():
    p = PRESETS[name]
    ua = p.u_value * p.envelope_area
    h_inf = RHO_CP * p.volume * p.leakage_ach / 3600
    c_air = RHO_CP * p.volume * p.air_capacity_factor
    tau_air_min = c_air / (ua + h_inf + p.h_mass * p.floor_area) / 60
    q_heat = (ua + h_inf) * (sp_w - t_w) - p.internal_gain_w
    margin = p.heater_efficiency * p.heater_max_w / q_heat if q_heat > 0 else float("inf")
    dt_max = (p.heater_efficiency * p.heater_max_w + p.internal_gain_w) / (ua + h_inf)
    heat_rows.append((name, f"{ua / 1e3:.2f}", f"{h_inf / 1e3:.2f}", f"{sp_w} / {t_w}",
                      f"{max(q_heat, 0) / 1e3:.1f}", f"{p.heater_max_w / 1e3:.0f}",
                      "no heating needed" if q_heat <= 0 else f"{margin:.2f}", f"{dt_max:.0f}",
                      f"{tau_air_min:.0f}"))

    q_sol = solar * p.floor_area * p.solar_transmittance * (1 - p.shade) * (1 - LATENT_FRACTION * (p.crop_factor > 0))
    q_cool = (ua + h_inf) * (t_s - sp_s) + p.internal_gain_w + extra + q_sol
    if p.cooler_type == "evaporative":
        t_wb = float(wet_bulb(t_s, rh_s))
        cap = RHO_CP * p.pad_airflow_m3s * p.pad_efficiency * (t_s - t_wb)
        cooler = f"pad {cap / 1e3:.0f} kW (supply {t_s - p.pad_efficiency * (t_s - t_wb):.1f} degC)"
    elif p.cooler_type == "refrigeration":
        cap = p.cooler_max_w
        cooler = f"refrigeration {cap / 1e3:.0f} kW"
    else:
        cap, cooler = 0.0, "none"
    ach_needed = q_sol / (RHO_CP * 4.0) * 3600 / p.volume if q_sol > 0 else 0.0
    cool_rows.append((name, f"{sp_s} / {t_s} @ {rh_s} %", f"{q_sol / 1e3:.0f}", f"{q_cool / 1e3:.0f}",
                      cooler, f"{ach_needed:.0f} / {p.vent_max_ach:.0f}" if q_sol > 0 else "-"))

save_text("02_design_heating.md", "# 02 Heating design (winter)\n\n" + md_table(heat_rows, [
    "facility", "UA [kW/K]", "H_inf [kW/K]", "setpoint / outdoor [degC]", "heat demand [kW]",
    "heater [kW]", "heater margin [-]", "max dT [K]", "air time constant [min]"]))
save_text("02_design_cooling.md", "# 02 Cooling design (summer)\n\n" + md_table(cool_rows, [
    "facility", "setpoint / outdoor", "solar gain [kW]", "cooling load [kW]", "installed cooling",
    "ACH for dT = 4 K / installed"]))

# ---------------------------------------------------------------- operating envelope
t_out = np.linspace(-15, 40, 111)
fig, axes = plt.subplots(2, 4, figsize=(16, 7.5), sharex=True)
for ax, name in zip(axes.flat, DESIGN):
    p = PRESETS[name]
    t_w, sp_w, t_s, rh_s, sp_s, solar, extra = DESIGN[name]
    sp = sp_w if name != "vertical_farm" else sp_s
    h = p.u_value * p.envelope_area + RHO_CP * p.volume * p.leakage_ach / 3600
    q_int = p.internal_gain_w + (extra if name == "vertical_farm" else 0.0)
    req = (h * (sp - t_out) - q_int) / 1e3
    heat = np.full_like(t_out, p.heater_efficiency * p.heater_max_w / 1e3)
    vent = -RHO_CP * p.volume * p.vent_max_ach / 3600 * np.clip(sp - t_out, 0, None) / 1e3
    if p.cooler_type == "refrigeration":
        cool = vent - p.cooler_max_w / 1e3
    elif p.cooler_type == "evaporative":
        t_sup = t_out - p.pad_efficiency * (t_out - wet_bulb(t_out, 40.0))
        cool = np.minimum(vent, -RHO_CP * p.pad_airflow_m3s * np.clip(sp - t_sup, 0, None) / 1e3)
    else:
        cool = vent
    ax.fill_between(t_out, cool, heat, color="#4393c3", alpha=0.25, label="installed capacity")
    ax.plot(t_out, req, "k", lw=1.5, label=f"required at {sp} degC")
    ax.axhline(0, color="grey", lw=0.6)
    ax.set_title(name, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="upper right")
axes.flat[-1].axis("off")
for ax in axes[1]:
    ax.set_xlabel("outdoor temperature [degC]")
for ax in axes[:, 0]:
    ax.set_ylabel("power [kW]  (+ heat, - cool)")
fig.suptitle("Operating envelope: required power vs installed capacity", fontsize=11)
fig.tight_layout()
fig.savefig(OUT / "02_operating_envelope.png", dpi=110)
print("envelope plot written")
