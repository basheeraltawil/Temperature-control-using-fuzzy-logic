# 05 Robustness to parameter mismatch (tomato greenhouse, 24 h)

| parameter | controller | RMSE -30 % [K] | RMSE nominal [K] | RMSE +30 % [K] | travel -30 % / +30 % |
|---|---|---|---|---|---|
| u_value | fuzzy_pi | 0.32 | 0.35 | 0.36 | 43 / 40 |
| u_value | pid | 0.22 | 0.21 | 0.23 | 97 / 86 |
| mass_capacity | fuzzy_pi | 0.35 | 0.35 | 0.35 | 41 / 43 |
| mass_capacity | pid | 0.21 | 0.21 | 0.21 | 90 / 93 |
| heater_max_w | fuzzy_pi | 0.33 | 0.35 | 0.32 | 40 / 43 |
| heater_max_w | pid | 0.26 | 0.21 | 0.20 | 91 / 94 |
| solar_transmittance | fuzzy_pi | 0.33 | 0.35 | 0.31 | 40 / 43 |
| solar_transmittance | pid | 0.21 | 0.21 | 0.21 | 86 / 100 |
