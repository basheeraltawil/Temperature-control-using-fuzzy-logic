"""Parameter presets for typical agricultural climate zones.

Values are order-of-magnitude figures from extension-service design guides
(greenhouse heat-loss coefficients, broiler-house ventilation rates, cold
store refrigeration loads).  Calibrate them on the real site with
``agriclimate.ai.sysid`` before trusting absolute numbers.
"""
from __future__ import annotations

from .facility import FacilityParams

PRESETS = {
    # 1000 m2 glass Venlo greenhouse, hot-water pipe heating, roof vents, pad-and-fan
    "greenhouse_glass": FacilityParams(
        name="greenhouse_glass", floor_area=1000, volume=4500, envelope_area=1400, u_value=6.0,
        solar_transmittance=0.7, heater_max_w=250e3, heater_tau_s=300, cooler_type="evaporative",
        pad_airflow_m3s=30, vent_max_ach=40),
    # 300 m2 single-layer PE polytunnel with a small forced-air heater (frost protection)
    "polytunnel": FacilityParams(
        name="polytunnel", floor_area=300, volume=900, envelope_area=520, u_value=7.5,
        solar_transmittance=0.8, air_capacity_factor=1.5, mass_capacity=50e3, heater_max_w=45e3,
        heater_tau_s=60, cooler_type="none", vent_max_ach=30),
    # insulated germination / growth chamber with heater and refrigeration unit
    "germination_chamber": FacilityParams(
        name="germination_chamber", floor_area=20, volume=60, envelope_area=100, u_value=0.35,
        solar_transmittance=0.0, air_capacity_factor=3.0, mass_capacity=30e3, h_ground=0.5,
        crop_factor=0.0, leakage_ach=0.3, vent_max_ach=4, natural_ventilation=False,
        vent_fan_power_w=150, heater_max_w=3e3, heater_efficiency=1.0, heater_tau_s=30,
        cooler_type="refrigeration", cooler_max_w=3e3, cooler_cop=2.8, cooler_tau_s=45,
        vent_rate_per_s=1 / 10, internal_gain_w=150, internal_moisture_kg_s=2e-6),
    # 1200 m2 tunnel-ventilated broiler house; birds are a large internal heat source
    "poultry_house": FacilityParams(
        name="poultry_house", floor_area=1200, volume=3600, envelope_area=2600, u_value=0.8,
        solar_transmittance=0.0, crop_factor=0.0, mass_capacity=40e3, leakage_ach=1.0,
        vent_max_ach=60, natural_ventilation=False, vent_fan_power_w=20e3, heater_max_w=150e3,
        heater_tau_s=90, cooler_type="evaporative", pad_airflow_m3s=45, pad_fan_power_w=18e3,
        vent_rate_per_s=1 / 30, internal_gain_w=20e3, internal_moisture_kg_s=1.5e-3),
    # 100 m2 post-harvest cold room for vegetables / fruit
    "cold_storage": FacilityParams(
        name="cold_storage", floor_area=100, volume=500, envelope_area=450, u_value=0.25,
        solar_transmittance=0.0, crop_factor=0.0, air_capacity_factor=4.0, mass_capacity=80e3,
        h_ground=0.3, t_ground=10.0, leakage_ach=0.2, vent_max_ach=2, natural_ventilation=False,
        vent_fan_power_w=200, heater_max_w=6e3, heater_efficiency=1.0, heater_tau_s=30,
        cooler_type="refrigeration", cooler_max_w=25e3, cooler_cop=2.2, cooler_tau_s=90,
        vent_rate_per_s=1 / 10, internal_gain_w=1.5e3, internal_moisture_kg_s=4e-5),
    # 200 m2 mushroom growing room (compost generates heat during spawn run)
    "mushroom_room": FacilityParams(
        name="mushroom_room", floor_area=200, volume=800, envelope_area=640, u_value=0.35,
        solar_transmittance=0.0, crop_factor=0.0, air_capacity_factor=3.0, mass_capacity=90e3,
        leakage_ach=0.3, vent_max_ach=6, natural_ventilation=False, vent_fan_power_w=1500,
        heater_max_w=20e3, heater_efficiency=1.0, heater_tau_s=60, cooler_type="refrigeration",
        cooler_max_w=25e3, cooler_cop=3.0, vent_rate_per_s=1 / 20, internal_gain_w=4e3,
        internal_moisture_kg_s=3e-4),
}


def get_preset(name: str) -> FacilityParams:
    try:
        return FacilityParams.from_dict(PRESETS[name].to_dict())   # defensive copy
    except KeyError as exc:
        raise KeyError(f"Unknown facility preset '{name}'. Available: {sorted(PRESETS)}") from exc
