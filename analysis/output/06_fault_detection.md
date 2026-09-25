# 06 Fault detection and isolation (sensor 1, fault at 20 h, 3 seeds)

| fault | set-up | first alarm | faulty sensor isolated | false alarms | RMSE after fault [K] |
|---|---|---|---|---|---|
| drift +0.5 K/h | 2 sensors, rules | 160 min (3/3) | no (0/3) | 0 | 2.29 |
| drift +0.5 K/h | 2 sensors, rules + AI | 160 min (3/3) | 462 min (3/3) | 0 | 0.85 |
| drift +0.5 K/h | 3 sensors, rules (2oo3) | 153 min (3/3) | no (0/3) | 0 | 0.27 |
| offset +2 K | 2 sensors, rules | 0 min (3/3) | 0 min (3/3) | 0 | 1.02 |
| offset +2 K | 2 sensors, rules + AI | 0 min (3/3) | 0 min (3/3) | 0 | 0.99 |
| offset +2 K | 3 sensors, rules (2oo3) | 0 min (3/3) | 0 min (3/3) | 0 | 0.27 |
| frozen value | 2 sensors, rules | 21 min (3/3) | no (0/3) | 0 | 1.79 |
| frozen value | 2 sensors, rules + AI | 21 min (3/3) | 32 min (3/3) | 0 | 0.30 |
| frozen value | 3 sensors, rules (2oo3) | 31 min (3/3) | no (0/3) | 0 | 0.27 |
| spikes 5 K | 2 sensors, rules | 8 min (3/3) | 8 min (3/3) | 0 | 0.27 |
| spikes 5 K | 2 sensors, rules + AI | 8 min (3/3) | 8 min (3/3) | 0 | 0.27 |
| spikes 5 K | 3 sensors, rules (2oo3) | 8 min (3/3) | 8 min (3/3) | 0 | 0.27 |
| extra noise 0.5 K | 2 sensors, rules | 23 min (3/3) | 23 min (3/3) | 0 | 0.36 |
| extra noise 0.5 K | 2 sensors, rules + AI | 6 min (3/3) | 6 min (3/3) | 0 | 0.27 |
| extra noise 0.5 K | 3 sensors, rules (2oo3) | 23 min (3/3) | 23 min (3/3) | 0 | 0.27 |
