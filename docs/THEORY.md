# Theory and equations

This page lists the equations the code actually implements, each with the file it lives in.
Symbols are defined where they first appear. Units are SI unless stated otherwise
(temperatures in °C, power in W, time in s).

**Contents**
1. [Fuzzy inference (Mamdani)](#1-fuzzy-inference-mamdani)
2. [Fuzzy-PI controller](#2-fuzzy-pi-controller)
3. [PID controller](#3-pid-controller)
4. [Split-range allocator](#4-split-range-allocator)
5. [Facility model (digital twin)](#5-facility-model-digital-twin)
6. [Sensor model](#6-sensor-model)
7. [Safety supervisor](#7-safety-supervisor)
8. [System identification](#8-system-identification)
9. [Model predictive control](#9-model-predictive-control)
10. [Anomaly detection](#10-anomaly-detection)
11. [Automatic tuning](#11-automatic-tuning)
12. [Lookup-table code generation](#12-lookup-table-code-generation)
13. [Performance indicators](#13-performance-indicators)

---

## 1. Fuzzy inference (Mamdani)

`agriclimate/fuzzy/engine.py`, `membership.py`

A rule $r$ reads *IF $x_1$ is $A_{r1}$ AND $x_2$ is $A_{r2}$ THEN $y$ is $B_r$*.

| Step | Equation |
|---|---|
| fuzzification | $\mu_{A}(x)$, e.g. triangle $\mu(x)=\max\big(0,\min(\tfrac{x-a}{b-a},\tfrac{c-x}{c-b})\big)$ |
| rule strength (AND = min) | $w_r = \min_i \mu_{A_{ri}}(x_i)$ |
| implication (min) | $\mu'_r(y) = \min\big(w_r,\ \mu_{B_r}(y)\big)$ |
| aggregation (max) | $\mu(y) = \max_r \mu'_r(y)$ |
| defuzzification (centroid) | $y^* = \dfrac{\sum_y y\,\mu(y)}{\sum_y \mu(y)}$ on a discretised output universe |

If no rule fires, $\mu(y)\equiv 0$ and the centroid is undefined. MATLAB then returns the
mid-range; the controllers here return a safe value instead. The original controller has such gaps
(see `analysis/01_legacy_fis_analysis.py`).

## 2. Fuzzy-PI controller

`agriclimate/control/fuzzy_pi.py`, `fuzzy/rulebases.py`

Error $e = r - T$ (setpoint minus validated temperature). With control period $\Delta t$:

$$
e_n = \mathrm{sat}(k_e\,e), \qquad
\dot e_f \leftarrow \dot e_f + \frac{\Delta t}{\tau_f+\Delta t}\Big(\frac{e_k-e_{k-1}}{\Delta t}-\dot e_f\Big), \qquad
\dot e_n = \mathrm{sat}(k_r\,\dot e_f)
$$

$$
\Delta u = F(e_n,\dot e_n), \qquad u_k = \mathrm{sat}\big(u_{k-1} + k_u\,\Delta u\,\Delta t\big), \qquad \mathrm{sat}(x)=\min(1,\max(-1,x))
$$

$F$ is a Mamdani system with 7 terms per input (NB, NM, NS, ZE, PS, PM, PB). The rule for
input terms $i, j \in \{0..6\}$ has output term $\mathrm{clip}(i+j-3,\,0,\,6)$ (Mac Vicar-Whelan
table). Integrating $\Delta u$ gives integral action; clamping $u$ is the anti-windup.

**Linear equivalent.** Near the origin, $F \approx a\,e_n + b\,\dot e_n$, so

$$
u \approx \underbrace{k_u k_r b}_{K_p}\, e + \underbrace{k_u k_e a}_{K_i} \int e\,dt .
$$

With the default gains this gives $K_p \approx 0.42\ \mathrm{K^{-1}}$ and $T_i = K_p/K_i \approx 670$ s
(`analysis/04_fuzzy_pi_linearization.py`). Away from the origin the local slope of $F$ varies
between about 0.8 and 1.4 and the increment saturates for large errors, so the controller is
mildly nonlinear rather than a linear PI.

## 3. PID controller

`agriclimate/control/pid.py`. Derivative on the measurement, filtered, with back-calculation
anti-windup:

$$
P = K_p(\beta r - y), \qquad
D_k = \frac{T_f D_{k-1} - K_p T_d (y_k-y_{k-1})}{T_f+\Delta t},\ T_f=\frac{T_d}{N}
$$

$$
v = P + I + D, \qquad u=\mathrm{sat}(v), \qquad
I_{k+1} = I_k + \Delta t\Big(\frac{K_p}{T_i}e_k + \frac{u-v}{T_t}\Big)
$$

## 4. Split-range allocator

`agriclimate/control/allocator.py`. Signed demand $u\in[-1,1]$, deadband $d$, vent share $s$:

$$
\text{heater}=\frac{u-d}{1-d}\ \ (u>d), \qquad c=\frac{-u-d}{1-d}\ \ (u<-d)
$$

If the outdoor air is colder than the indoor air by more than a margin, vents take the first part
of the cooling range: $\text{vent}=\min(1,c/s)$ and $\text{cooler}=\max\big(0,\frac{c-s}{1-s}\big)$.
Otherwise $\text{cooler}=c$. A minimum vent opening always applies. When $RH > RH_{max}$ and
nothing is cooling, a small heat-and-vent share is added to dehumidify.

## 5. Facility model (digital twin)

`agriclimate/plant/facility.py`, `psychrometrics.py`. Two thermal nodes (air $T_a$, thermal mass
$T_m$) and a vapour balance (vapour density $\rho_v$):

$$
C_a \frac{dT_a}{dt} = Q_{heat} + f_a Q_{sol,s} + Q_{int} + UA(T_o-T_a) + \rho c_p F (T_o-T_a) + h_m A_f (T_m-T_a) + Q_{cool}
$$

$$
C_m \frac{dT_m}{dt} = (1-f_a) Q_{sol,s} - h_m A_f (T_m-T_a) - h_g A_f (T_m-T_g)
$$

$$
V \frac{d\rho_v}{dt} = E_{crop} + E_{int} + F(\rho_{v,o}-\rho_v) + E_{cool}
$$

| Term | Model |
|---|---|
| air capacity | $C_a = \rho c_p V f_{cap}$; $f_{cap}>1$ accounts for fixtures |
| ventilation flow | $F = \frac{V}{3600}\big(ACH_{leak} + ACH_{extra} + v\,ACH_{max}\,f_{wind}\big)$, $f_{wind}=\mathrm{clip}(0.5+0.12\,w,\ 0.4,\ 1.6)$ for natural vents |
| solar input | $Q_{in} = I\,A_f\,\tau\,(1-s_{shade})$ |
| transpiration (latent) | $L = c_f\,(0.45\,Q_{in} + 5\,A_f\,VPD)$, capped; $E_{crop}=L/\lambda$ |
| sensible solar | $Q_{sol,s} = Q_{in} - L$, split $f_a$ to air, $1-f_a$ to mass |
| heater | $Q_{heat} = \eta\,P_{heat}\,x_h$ |
| pad cooling | $T_{sup} = T_o - \eta_{pad}(T_o - T_{wb})$, $Q_{cool} = \rho c_p F_{pad}(T_{sup}-T_a)$, adds vapour |
| refrigeration | $Q_{cool} = -x_c P_{cool}$, electrical power $= Q/COP$, coil condenses vapour |
| actuator dynamics | $x \leftarrow x + (u-x)\,\Delta t/\tau_{act}$; vents slew-rate limited |

Psychrometrics (FAO-56 and Stull 2011):

$$
e_s(T)=0.6108\,\exp\!\Big(\frac{17.27\,T}{T+237.3}\Big)\ \mathrm{kPa},\qquad
\rho_v=\frac{e}{R_v (T+273.15)},\qquad
VPD = e_s(T)\,(1-RH/100)
$$

Integration: explicit Euler with sub-steps of at most 2 s. The fastest time constant among the
presets is about 2 min, so the step is well inside the stability limit.

## 6. Sensor model

`agriclimate/plant/sensors.py`

$$
y_f \leftarrow y_f + (T - y_f)\frac{\Delta t}{\tau_s}, \qquad
y = \mathrm{round}\big(y_f + b + \mathcal N(0,\sigma^2),\ q\big)
$$

Injectable faults: drift (+rate·time), offset, frozen value, spikes, dropout (NaN) and extra noise.

## 7. Safety supervisor

`agriclimate/control/supervisor.py`

* **Validation**: reading within range, not NaN, $|y_k-y_{k-1}|/\Delta t \le \dot T_{max}$, not
  excluded by the AI monitor.
* **Fusion**: 1 valid sensor, use it; 2 sensors, use the mean (marked DEGRADED if they disagree by
  more than the tolerance); 3 or more, use the median (2-out-of-3 voting).
* **Limits with hysteresis**: LOW_LIMIT when $T\le T_{low}$, released at $T \ge T_{low}+h$;
  HIGH_LIMIT likewise.
* **Fail-safe** (no valid sensor for longer than the timeout): heater $=\mathrm{clip}\big((T_{fs}-T_o)/20,\,0,\,1\big)$.
* **Slew limit**: $|u_k-u_{k-1}| \le r\,\Delta t$. **Anti-short-cycle**: minimum compressor on and off times.

## 8. System identification

`agriclimate/ai/sysid.py`. One-step model at $\Delta t_m = 60$ s:

$$
T_{k+1}-T_k = \theta^\top \varphi_k,\qquad
\varphi_k = \big[T_o-T,\ h_f,\ c_f,\ c_f(T_{sup}-T),\ v(T_o-T),\ I/1000,\ T_k-T_{k-1},\ 1\big]
$$

Here $h_f, c_f$ are the heater and cooler commands passed through first-order filters,
$h_f \leftarrow h_f + \frac{\Delta t_m}{\tau_h+\Delta t_m}(h-h_f)$. This represents boiler, pipe and
compressor lags. $\tau_h,\tau_c$ are chosen by grid search on validation data. The coefficients come
from ridge regression on scaled features:

$$
\hat\theta = (\Phi^\top\Phi + \alpha I)^{-1}\Phi^\top y .
$$

Reported quality: one-step RMSE and $R^2$ on the last 25 % of the data, plus the 1-hour open-loop
rollout RMSE, which is what the MPC relies on.

## 9. Model predictive control

`agriclimate/control/mpc.py`. Every 5 min, over a 1 h horizon ($N=60$ model steps) with 6
move blocks $z_j$:

$$
\min_{z}\ \sum_{k=1}^{N}\Big[w_T\,\max(|T_k-r_k|-\delta,\,0)^2 + w_h\,h_k + w_c\,(c_k+0.3\,v_k)\Big] + w_{\Delta}\sum_j (z_j-z_{j-1})^2,
\qquad -1\le z_j\le 1
$$

$T_k$ comes from the identified model driven by the weather forecast; $r_k$ is the recipe
preview; $h_k,c_k,v_k$ come from the allocator. The solver is L-BFGS-B, warm-started with the
better of the previous plan and the fuzzy-PI demand. Offset-free correction:
$d \leftarrow \mathrm{clip}\big(d + g\,(T - \hat T - d)/10,\ \pm 0.05\big)$, added to each predicted step.

## 10. Anomaly detection

`agriclimate/ai/anomaly.py`. Per sensor, one-step residual
$r_k = (x_k-x_{k-1}) - \Delta\hat T_k$ from the identified model. Over a 30-sample window:

$$
\mathbf f = \big[\overline{r}_{last\,4},\ \textstyle\sum r,\ \sigma_r,\ \max|r|,\ \sigma_{\Delta x}\big]
$$

The IsolationForest score is based on the average path length $E[h(\mathbf f)]$ needed to isolate
a point in random trees, $s = 2^{-E[h]/c(n)}$; short paths mean anomalies. The threshold is the
0.1 % quantile of healthy scores minus a margin. Rule companions calibrated on the same healthy
data: flat line ($\sigma_{\Delta x}$ very small), drift ($|\sum r| > 1.5\max_{healthy}|\sum r|$) and
heater underperformance (heater above 60 % for the whole window while the healthy sensors' residual
sum is below the healthy minimum). A sensor is excluded after 5 anomalous evaluations in a row and
released after 30 healthy ones. If all sensors are anomalous, none is excluded: the cause is
common-mode.

## 11. Automatic tuning

`agriclimate/ai/tuner.py`. Differential evolution over $\log_{10}$ of the gains, minimising the
mean over the scenarios of

$$
J = \mathrm{MAE} + 0.5\frac{t_{stress}}{t_{run}} + w_{tr}\frac{\sum|\Delta u|}{t_{run}} + w_E \max\Big(0,\frac{E}{E_{PID}}-1\Big).
$$

## 12. Lookup-table code generation

`agriclimate/fuzzy/codegen.py`. The fuzzy surface is sampled on a grid $F_{ij}$ and interpolated
bilinearly at run time (identical in Python, C and Structured Text):

$$
F(x,y) \approx (1-f_x)(1-f_y)F_{i,j} + f_x(1-f_y)F_{i,j+1} + (1-f_x)f_yF_{i+1,j} + f_xf_yF_{i+1,j+1}.
$$

## 13. Performance indicators

`agriclimate/sim/metrics.py`. All use the **true** air temperature $T$ (not the measured one) and
the setpoint $r$, with $N$ samples at period $\Delta t$:

| KPI | Definition |
|---|---|
| RMSE | $\sqrt{\frac1N\sum (T-r)^2}$ |
| in band | share of samples with $\lvert T-r\rvert \le$ comfort band |
| crop-stress hours | $\Delta t \cdot \#\{T<T_{min}\ \text{or}\ T>T_{max}\}$ |
| actuator travel | $\sum_k \big(\lvert\Delta h_k\rvert + \lvert\Delta c_k\rvert + \lvert\Delta v_k\rvert\big)$, a proxy for wear |
| heat/cool overlap | hours with heater and cooler both above 5 % |
| energy | heating (fuel or electric input) + cooling electricity + fan electricity, kWh |
