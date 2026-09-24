# From simulation to a real facility – implementation guide

This guide takes the controller from the digital twin to a greenhouse, growth chamber,
livestock house or cold store. It follows the order used on industrial automation
projects: **specify → design → build → virtually commission → commission → operate**.
Every step names the `agriclimate` tool that supports it.

> **Safety first.** Software, including this project and any AI component, is never the only
> protection for crops, animals or people. Every installation needs independent, hard-wired
> protection (high-limit and frost thermostats, burner safety controls, alarm dialer).
> Qualified electricians and gas/refrigeration engineers must do the electrical, gas and
> refrigerant work under local regulations.

---

## 1. Specify the process

| Item | Example (tomato glasshouse) | Where it goes |
|---|---|---|
| Crop / animal targets per growth stage | day 21 °C, night 17 °C, pre-dawn drop 2 K | `setpoint:` in a scenario YAML, or `agriclimate advise` (LLM draft) |
| Tolerance / KPI | ±1 K for 95 % of the time | `comfort_band:` |
| Damage limits | < 10 °C chilling, > 32 °C pollination failure | `crop_limits:` + `supervisor:` limits |
| Humidity policy | RH < 88 % (Botrytis) | `allocator: rh_max` (heat-and-vent) |
| Air-quality minimum ventilation | broilers ~1 ACH in winter | `allocator: min_vent` |
| Design weather | local 1 % winter / summer design temperatures | `weather:` (synthetic) or CSV replay |

Write one scenario file per zone and season, then **simulate before buying hardware**:

```bash
agriclimate run my_zone.yaml --controllers fuzzy_pi pid mpc --plot my_zone.png
```

Check that the heater, cooler and ventilation capacities in `facility_overrides` can hold the
design conditions. The frost and heat-wave scenarios show what an undersized actuator looks
like.

## 2. Control architecture

```
 Level 4  Enterprise / cloud   MQTT → Grafana / InfluxDB / SCADA, LLM shift reports, recipe library
 Level 3  Supervisory (edge)   ROS 2 or edge_loop: MPC, AI anomaly monitor, recipes, logging (rosbag2)
 Level 2  Control (PLC)        FB_FuzzyPIClimate (generated ST), interlocks, heartbeat watchdog
 Level 1  Field                sensors, 4-20 mA / 0-10 V I/O, contactors, VFDs, valves
 Level 0  Hard-wired safety    high-limit & frost thermostats, burner manager, E-stop, alarm dialer
```

* **The PLC can run the site by itself.** It holds the generated fuzzy-PI function block,
  every interlock and a watchdog on the edge computer's heartbeat (holding register 3).
  If the edge computer, network or ROS stack fails, the PLC switches to local fuzzy-PI
  control within 5 s. The virtual PLC (`agriclimate.io.plc_simulator`) shows this behaviour.
* **The edge layer adds optimisation and intelligence**: MPC with weather forecasts, the AI
  anomaly monitor, recipe scheduling and data logging. Losing it degrades performance but
  keeps the site safe.
* **AI and LLM outputs are advisory or subtractive.** The anomaly monitor can only exclude a
  sensor from fusion. LLM recipes pass a deterministic validator and then human approval.
  Neither ever writes an actuator.

## 3. Hardware selection (reference bill of materials)

| Function | Industrial choice | Budget / research choice |
|---|---|---|
| Air temperature (×2 per zone, redundant) | Pt100/Pt1000 4-wire + 4-20 mA head transmitter in an **aspirated** radiation shield | SHT45 / DS18B20 in a ventilated shield |
| Humidity | capacitive RH transmitter, 4-20 mA, ±2 % | SHT45 (I²C) |
| Outdoor weather | weather station with pyranometer, wind and rain (Modbus RTU) | online forecast API + local temperature probe |
| PLC / controller | Siemens S7-1200/1500, CODESYS (WAGO, Beckhoff, Schneider M241) | Arduino Opta / Controllino / ESP32 + relay board |
| Edge computer | fanless industrial PC, Ubuntu 22.04 + ROS 2 Humble | Raspberry Pi 5 / CM4 industrial carrier |
| Heating | boiler + mixing valve (0-10 V), or staged gas unit heaters (relays) | electric heater via zero-cross SSR (PWM) |
| Cooling | evaporative pad pump + fans on a VFD; refrigeration compressor via contactor | Peltier / small split unit |
| Ventilation | vent motors with position feedback (4-20 mA), exhaust fans on a VFD | servo / linear actuator |
| Networks | Modbus TCP or OPC UA to the PLC, MQTT (TLS) to SCADA/cloud | Modbus RTU (RS-485), Wi-Fi MQTT |
| Wireless sensor nodes | – | ESP32 + micro-ROS or MQTT |

**Actuator mapping.** Each controller output is a 0–1 command. The original project drove
8-bit PWM (0–255); `ActuatorCommand.as_pwm()` gives the same mapping. For relays and staged
heaters, use slow time-proportioning (e.g. a 60 s period) or stage thresholds with hysteresis,
never fast PWM on contactors. Compressors are covered by the supervisor's minimum on/off
times.

## 4. Safety and compliance checklist

- [ ] Hard-wired **high-limit thermostat** that cuts heating independently of the PLC
      (manual reset), and a **frost thermostat** that forces heating on or raises an alarm.
- [ ] Burners use a certified burner/flame safeguard control (e.g. EN 298). The PLC only
      *requests* heat.
- [ ] Refrigeration follows EN 378 / F-gas rules. Anti-short-cycle is also set in the PLC.
- [ ] Livestock houses: alarm system with a backup power supply and a remote alarm dialer (EU
      Directive 2007/43/EC for broilers requires alarms and a backup system for mechanically
      ventilated houses). Include a **fail-safe curtain/vent drop** on power loss.
- [ ] Electrical design to IEC 60204-1 / local code; PLC programs to IEC 61131-3.
- [ ] Assess functional safety (ISO 13849 / IEC 61508 risk assessment) for automatic vents,
      doors and hazardous gas.
- [ ] Network: separate OT VLAN, no inbound internet to the PLC, MQTT over TLS with
      per-device credentials. Remote setpoints are range-checked (`MqttBridge`).

## 5. I/O list template

| Tag | Signal | Type | Range / scaling | Modbus register |
|---|---|---|---|---|
| TT-101 | air temperature A | AI 4-20 mA | −40…70 °C → ×100 | IR 0 |
| TT-102 | air temperature B | AI 4-20 mA | −40…70 °C → ×100 | IR 1 |
| MT-101 | indoor RH | AI 4-20 mA | 0…100 % → ×100 | IR 2 |
| TT-001 | outdoor temperature | AI | ×100 | IR 3 |
| RT-001 | global radiation | AI | W/m² | IR 4 |
| HV-201 | heating valve / burner | AO 0-10 V or DO | 0…10000 = 0…100 % | HR 0 |
| CU-301 | pad pump + fans / compressor | AO / DO | 0…10000 | HR 1 |
| VM-401 | roof vent / exhaust fans | AO | 0…10000 | HR 2 |
| – | edge heartbeat | – | counter | HR 3 |
| – | remote/local request | – | 0/1 | HR 4 |

## 6. Software deployment

```bash
# edge computer (Ubuntu 22.04)
git clone https://github.com/basheeraltawil/Temperature-control-using-fuzzy-logic.git
cd Temperature-control-using-fuzzy-logic
pip install -e ".[ai,field]"
agriclimate export --out generated          # PLC ST, C header, MATLAB FIS
```

1. **PLC**: import `generated/plc/FB_FuzzyPIClimate.st` (TIA Portal: *External source files*;
   CODESYS/TwinCAT: add as a POU). Call it cyclically (e.g. 1 s), connect scaled I/O, add the
   interlocks and watchdog logic from `plc_simulator.VirtualPLC._step`. The generated block
   and `FuzzyPIController(use_lut=True)` give identical outputs, so gains tuned in simulation
   carry over.
2. **Microcontroller option**: include `generated/firmware/fuzzy_pi_lut.h`. `fuzzy_pi_lut_eval(e, de)`
   is allocation-free C99 (ESP32, STM32, AVR).
3. **Edge without ROS**: `python -m agriclimate.io.edge_loop --plc <ip> --mqtt <broker>`
   (install as a systemd service with `Restart=always`).
4. **Edge with ROS 2**: build `ros2_ws`, then
   `ros2 launch agriclimate_ros hardware.launch.py plc_host:=<ip> namespace:=greenhouse3`.
   Use one namespace per climate zone. Record data with
   `ros2 bag record -a -o /data/zone3` and aggregate `/diagnostics` in your SCADA/HMI.

## 7. Virtual commissioning (before site work)

```bash
python -m agriclimate.io.plc_simulator --port 5020 --time-scale 30 &
python -m agriclimate.io.edge_loop --plc 127.0.0.1 --port 5020 --period 1 --time-scale 30
```

Check register scaling and signs, heartbeat fallback (stop the edge loop and the PLC log must
show `LOCAL`), alarm routing to MQTT, and the fault scenarios
(`greenhouse_sensor_actuator_faults`) through the whole chain. Once the real PLC program
exists, point the edge loop at a PLC simulator (PLCSIM Advanced / CODESYS SoftPLC) for
hardware-in-the-loop testing.

## 8. On-site commissioning

1. **Loop checks**: force each output from the HMI and confirm the right device moves the
   right way at 0, 50 and 100 %.
2. **Sensor calibration**: compare every probe against a calibrated reference thermometer at
   two points (e.g. an ice bath and ambient). Enter offsets in the transmitter or PLC scaling.
   Mount probes in aspirated shields at crop/animal height, out of direct sun and away from
   heating pipes.
3. **Open-loop step tests**: heater 0→50 %, vents 0→50 %, cooler 0→50 %, one at a time, in
   stable weather. Log at ≤ 60 s.
4. **System identification**: run the excitation experiment with the supervisor active
   (`excitation_experiment` shows the pattern), export the log as CSV with columns
   `t_s,t_meas,heater,cooler,vent,t_out,rh_out,solar`, then
   `agriclimate identify --csv site_log.csv --out site_model.json`.
   Accept the model when the 1 h rollout RMSE is below about 1 K.
5. **Calibrate the twin**: adjust `facility_overrides` (U-value, capacities, time constants)
   until the simulated step responses match the logged ones.
6. **Tune**: `agriclimate tune my_zone.yaml --controller fuzzy_pi`, then apply the gains with
   10–20 % margin (lower `ku`) and refine on site.
7. **Fallback and alarm tests**: unplug the network (the PLC must go local), disconnect a probe
   (`SENSOR*_INVALID` raised, fusion continues), heat a probe with a hand (implausible-rate
   alarm), trip the high-limit thermostat (the hard-wired cut must work with the PLC running).
8. **Acceptance run**: 7 days of automatic operation. Compare KPIs (in-band %, stress hours,
   kWh/m², actuator travel) with the simulation and the previous control system.

## 9. Deploying the AI components safely

| Phase | Anomaly monitor | MPC | LLM assistant |
|---|---|---|---|
| Weeks 0–2 | collect healthy data | identify the model | draft recipes offline only |
| Weeks 2–6 | **shadow mode**: alarms only, no sensor exclusion | shadow: log its demand beside fuzzy-PI | agronomist reviews every recipe |
| After review | enable exclusion (`anomaly/excluded_sensors`) | enable with fuzzy-PI fallback | recipes via validator + approval |
| Ongoing | retrain each season / after maintenance | re-identify each season | shift reports for operators |

* Version models (JSON from `LearnedThermalModel.save`) together with the scenario YAML.
* Monitor the model: if its one-step residual grows over weeks, the building has changed
  (new screen, fouled glass) and it is time to re-identify.
* Keep false alarms rare. The detector threshold is calibrated below the healthy score
  distribution because operators ignore noisy alarms.
* LLM: the API key belongs on the edge/cloud side, never in the PLC. Log every prompt and
  approved recipe. Recipes are simulated (`agriclimate advise ... --simulate`) before use.

## 10. Operation and maintenance

* Dashboards: temperature vs setpoint, actuator positions, `/diagnostics` levels, energy per
  day, anomaly scores.
* Monthly: check probe calibration against a reference and clean the aspirated shields.
  Before each season: test the thermostats and the alarm dialer.
* Use `agriclimate report <scenario>` style shift reports on logged runs to turn alarm logs
  into maintenance actions.
