# 03 Step response and PI tuning

Heater step 0 -> 40 %, constant weather. SIMC with tau_c = max(theta, tau/4).

| facility | K [K per unit heater] | tau [min] | theta [s] | SIMC Kc [1/K] | SIMC Ti [s] |
|---|---|---|---|---|---|
| greenhouse_glass | 19.6 | 46 | 0 | 0.204 | 2774 |
| germination_chamber | 53.8 | 183 | 0 | 0.074 | 10966 |
| cold_storage | 22.5 | 307 | 0 | 0.178 | 18420 |

Auto-tuned default PID (tomato greenhouse scenario, differential evolution): Kp = 0.35, Ti = 770 s.
