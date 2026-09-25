# Documentation

| Document | Contents |
|---|---|
| [../README.md](../README.md) | overview, results, quick start |
| [SCENARIOS.md](SCENARIOS.md) | the ten agricultural scenarios: problem, what they test, results; how to write your own |
| [THEORY.md](THEORY.md) | every equation used in the code (fuzzy inference, controllers, plant model, AI methods, KPIs) |
| [DESIGN_CALCULATIONS.md](DESIGN_CALCULATIONS.md) | heat-balance sizing, operating envelope, time constants, control design choices |
| [ARCHITECTURE.md](ARCHITECTURE.md) | code structure, conventions, how to extend the project |
| [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) | from simulation to a real facility: hardware, safety, commissioning, AI roll-out |
| [../analysis/README.md](../analysis/README.md) | the six engineering analyses and their results |
| [results/benchmark.md](results/benchmark.md) | full controller benchmark table and plots |

## Suggested reading paths

```mermaid
flowchart LR
  S["Student / educator"] --> S1["README"] --> S2["THEORY §1-3<br/>fuzzy logic, PI, PID"] --> S3["analysis 01, 04<br/>legacy FIS, fuzzy-PI surface"] --> S4["run a scenario<br/>and change the gains"]
  R["Researcher"] --> R1["README results"] --> R2["THEORY §5, 8-10<br/>plant model and AI methods"] --> R3["analysis 05, 06<br/>robustness, fault detection"] --> R4["ARCHITECTURE §5<br/>add your method"]
  C["Engineer / company"] --> C1["SCENARIOS"] --> C2["DESIGN_CALCULATIONS"] --> C3["IMPLEMENTATION_GUIDE"] --> C4["ROS 2 / Modbus / PLC export"]
```

## Scope and limits

* All results come from a digital twin with typical parameters, not from measurements on a real
  site. Calibrate the twin with site data (`agriclimate identify --csv`) before drawing
  site-specific conclusions.
* The controllers handle temperature. Humidity is modelled and limited (heat-and-vent), but not
  controlled to a setpoint; CO₂ is not modelled.
* The MPC has no model input for scheduled internal loads (e.g. LEDs), so it performs poorly in the
  vertical-farm scenario.
* Safety still depends on independent hard-wired protection; see the implementation guide.
