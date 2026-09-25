# Agricultural scenarios

Each scenario is a YAML file in [`scenarios/`](../scenarios) describing one real operating
problem: facility, weather, crop recipe, damage limits, faults and disturbances. The agronomic
background is written inside each file (`agronomy:`). Results below come from
`agriclimate benchmark` ([full table](results/benchmark.md)); all numbers are from the digital
twin, not from a real site.

Run one yourself:

```bash
agriclimate run tomato_greenhouse_spring --plot tomato.png
```

## Overview

| Scenario | Real-world problem | Best result (true air temperature) |
|---|---|---|
| [tomato_greenhouse_spring](#tomato-greenhouse-spring) | day/night temperature recipe with strong sun | all redesigned controllers ≥ 98 % in band; MPC lowest peak error and energy |
| [seed_germination_chamber](#seed-germination-chamber) | ±0.5 K hold for germination, then hardening | ≥ 99 % in band; fuzzy-PI and MPC move the compressor far less than PID |
| [frost_protection_polytunnel](#frost-protection-polytunnel) | −7 °C frost night with an undersized heater | no hours below 1 °C with any redesigned controller |
| [heatwave_cucumber_cooling](#heat-wave-cucumber-greenhouse) | 38 °C dry heat wave | capacity-limited (~78 % in band); MPC saves 8.5 % energy |
| [broiler_brooding](#broiler-brooding) | chick brooding curve in winter | ≥ 99.7 % in band |
| [cold_storage_door_openings](#cold-store-door-openings) | 2 °C cold room with door openings | ~95 % in band; brief excursions at each opening |
| [mushroom_spawn_to_pinning](#mushroom-spawn-run-to-pinning) | 25 → 18 °C step to trigger pinning | ≥ 99.6 % in band |
| [vertical_farm_lettuce](#vertical-farm) | 30 kW LED heat switching on and off | fuzzy-PI / PID ≈ 99 % in band; MPC not suitable without a light model |
| [greenhouse_sensor_actuator_faults](#sensor-and-actuator-faults) | drifting, frozen and spiking sensors, weak boiler | with AI monitor 97.6 % in band vs 65 % with rules only |
| [sensor_loss_failsafe](#total-sensor-loss) | all sensors lost for 2 h on a cold night | fail-safe keeps the crop above its limit; bumpless return |

---

### Tomato greenhouse, spring
* **Problem**: hold 21 °C by day and 17 °C by night with a 2 K pre-dawn drop (limits stem
  stretching), while strong sun heats the house by ~300 kW.
* **Tests**: recipe ramps, heating to vent-cooling transitions, RH limit of 88 % (Botrytis).
* **Result**: fuzzy-PI 0.29 K RMSE, PID 0.21 K, MPC 0.34 K with the lowest peak error (1.5 K),
  2 % less energy and the least actuator movement. Legacy FIS: 5.3 K RMSE, 17 h of crop stress.

### Seed germination chamber
* **Problem**: seed trays need 25 ± 0.5 °C, then 20 °C to harden seedlings; the chamber stands in
  an unheated shed with a 20 K daily swing.
* **Tests**: tight tolerance, heater/compressor changeover, compressor anti-short-cycle.
* **Result**: all ≥ 99 % in band. PID moves the actuators 481 units, fuzzy-PI 102, MPC 23:
  fewer compressor starts and longer equipment life.

### Frost protection, polytunnel
* **Problem**: strawberry flowers die below about −1 °C. A clear night drops to −7 °C, and the
  heater is sized for about −4 °C (see [design calculations](DESIGN_CALCULATIONS.md)).
* **Tests**: behaviour at the capacity limit, frost limit in the supervisor.
* **Result**: no hours below 1 °C with the redesigned controllers (legacy: 8 h). Tracking of the
  5 °C night setpoint is capacity-limited, the same for every controller.

### Heat wave, cucumber greenhouse
* **Problem**: 38 °C and dry air. Vents first, then evaporative pads, with a 30 % shade screen.
* **Tests**: cooling staging, evaporative cooling physics (wet-bulb limit).
* **Result**: about 78 % in band for all controllers because the pads run at their limit. MPC uses
  8.5 % less energy than PID.

### Broiler brooding
* **Problem**: day-7 to day-12 chicks, setpoint falling 0.4 K per day from 29 °C, outdoor 1 °C,
  minimum ventilation for ammonia and moisture; the birds' own heat grows.
* **Tests**: slow recipe, large internal gains, minimum ventilation.
* **Result**: ≥ 99.7 % in band (legacy: 21 K too cold on average).

### Cold store door openings
* **Problem**: vegetables at 2 °C (freezing injury below −0.5 °C); every 3 h a forklift door
  opens for 6 min in summer.
* **Tests**: refrigeration with minimum on/off times, repeated disturbances.
* **Result**: about 95 % in band; each opening causes a short excursion of up to 4 K. Faster
  recovery would need more compressor capacity or a strip curtain / air lock, a design
  decision rather than a control one.

### Mushroom spawn run to pinning
* **Problem**: 48 h at 25 °C (mycelium growth, compost releases heat), then a drop to 18 °C to
  trigger pinning.
* **Tests**: large setpoint step, internal heat, refrigeration.
* **Result**: ≥ 99.6 % in band for all redesigned controllers.

### Vertical farm
* **Problem**: 16 h LED photoperiod; switching the lights adds or removes about 30 kW of heat in
  an insulated room.
* **Tests**: large, fast, periodic disturbance; cooling-dominated operation.
* **Result**: fuzzy-PI 0.31 K and PID 0.22 K RMSE (gains auto-tuned for this room). MPC performs
  poorly (1.8 K RMSE) because its model has no input for the lights. A known, scheduled
  disturbance like this should be added to the model as a feed-forward input (listed in the
  roadmap). This case is kept to show where the method needs extension.

### Sensor and actuator faults
* **Problem**: sensor 1 drifts +0.35 K/h, sensor 2 freezes before the evening temperature drop,
  sensor 1 then spikes, and the boiler loses 65 % of its capacity at night.
* **Tests**: sensor validation, fusion, AI fault isolation, heater diagnosis.
* **Result**: rules only 1.71 K RMSE (65 % in band); rules + AI monitor 0.35 K (97.6 % in band).
  The AI flags the frozen sensor 33 min after it freezes and reports the weak boiler.
  [analysis/output/06_fault_detection.md](../analysis/output/06_fault_detection.md) compares five
  fault types systematically, including a 3-sensor installation.

### Total sensor loss
* **Problem**: a tripped 24 V supply removes both sensors from 02:00 to 04:00 on a 0 °C night.
* **Tests**: fail-safe mode (open-loop heating estimated from the outdoor temperature), alarm,
  bumpless return to automatic.
* **Result**: no crop stress with the supervised controllers; the legacy controller, which holds its
  last reading, leaves the crop 11 K off target on average.

## Writing your own scenario

```yaml
name: my_zone
description: one line
facility: greenhouse_glass            # preset from agriclimate/plant/presets.py
facility_overrides: {u_value: 4.0}    # any FacilityParams field
duration_h: 48
control_dt_s: 30
weather: {type: synthetic, t_mean: 5, t_amplitude: 6, solar_peak: 600, seed: 1}   # or {type: csv, path: ...}
setpoint: {type: day_night, day: 21, night: 17, day_start_h: 7, day_end_h: 19}
comfort_band: 1.0
crop_limits: {min: 10, max: 32}
allocator: {vent_stage: 0.6, rh_max: 88}
supervisor: {t_low_limit: 6, t_high_limit: 36}
sensors: {count: 3}
faults: {sensors: [{sensor: 1, kind: drift, start_h: 10, value: 0.5}]}
disturbances: [{kind: door, start_h: 1, period_h: 3, duration_min: 5, ach: 6}]
controllers: [fuzzy_pi, pid, mpc]
```

Setpoint types: `constant`, `day_night`, `table` (piecewise-linear points), `brooding`.
Sensor fault kinds: `drift`, `offset`, `stuck`, `spike`, `dropout`, `noise`. Actuator faults:
`heater_capacity`, `cooler_capacity`, `vent_stuck`. Disturbances: `door`, `infiltration`,
`internal_gain` (optionally periodic with `period_h`, `on_h`).
