| ID | Test | Metric | Threshold | Measured | Result |
|---|---|---|---|---|---|
| S1 | Sim self-check (60 deg turn) | max |d(VSI)/dt - earthVertG| (g) | < 0.001 | 2.31e-16 | PASS |
| S2 | Sim self-check | max |Rnb*f_b vs wind-axis f| (g) | < 1e-09 | 4.44e-16 | PASS |
| A1 | Axis table ISM330DHCX: entries found | orientation pairs in table (of 24) | > 23.5 | 24 | PASS |
| A2 | Axis table ISM330DHCX: entries agreeing on box geometry + handedness | consistent entries (of 24) | > 23.5 | 23 (bad: LEFT/DOWN; ports=+Y top=+Z, right-handed chip) | **FAIL** |
| A3 | Axis map ISM330DHCX, physically generated chip data | entries giving Ax=fx,Ay=fy,Az=-fz,Gx=-p,Gy=-q,Gz=r | > 23.5 | 23 | **FAIL** |
| A4 | Axis table LSM9DS1: entries found | orientation pairs in table (of 24) | > 23.5 | 24 | PASS |
| A5 | Axis table LSM9DS1: entries agreeing on box geometry + handedness | consistent entries (of 24) | > 23.5 | 22 (bad: FORWARD/LEFT, LEFT/DOWN; ports=+X top=+Z, left-handed chip) | **FAIL** |
| A6 | Axis map LSM9DS1, physically generated chip data | entries giving Ax=fx,Ay=fy,Az=-fz (accel only) | > 23.5 | 22 | **FAIL** |
| G1 | Gyro bias 0.3 dps on yaw axis only, level 120 kt | max attitude err 20-300 s (deg) | < 1 | 2.04 | **FAIL** |
| G1p | Gyro bias 0.3 dps on yaw axis only, level 120 kt | predicted roll err atan(TAS*b_yaw/g) (deg) | info | 1.89 | info |
| G2 | Gyro bias 0.3 dps on roll+pitch axes only, level 120 kt | max attitude err 20-300 s (deg) | < 1 | 0.50 | PASS |
| G3 | Gyro bias 0.1 dps on yaw axis only, level 120 kt | max attitude err 20-300 s (deg) | < 1 | 0.78 | PASS |
| G3p | Gyro bias 0.1 dps on yaw axis only, level 120 kt | predicted roll err atan(TAS*b_yaw/g) (deg) | info | 0.63 | info |
| G4 | Gyro bias 1.0 dps all axes, level 120 kt | max attitude err 20-300 s (deg) | info | 38.68 | info |
| G5 | Gyro bias 0.9+0.9 dps roll+pitch (|b|=1.27 < slew limit 2*beta=1.33 dps), level | max attitude err 20-300 s (deg) | info | 1.12 | info |
| G6 | Gyro bias 1.0+1.0 dps roll+pitch (|b|=1.41 > slew limit), level | max attitude err 20-300 s (deg) | info | 58.47 | info |
| G7 | Gyro calibrated at power-up (bias drift 0.002 dps/rt-s remains), level 10 min | max attitude err (deg) | < 1 | 1.04 | **FAIL** |
| I1 | Install 5 deg nose-up, 3 deg roll; calibrated | pitchBias, rollBias (deg) | info | -4.99, -2.99 | info |
| I2 | Install bias, level after a 45 deg turn | max attitude err 110-130 s (deg) | < 1 | 0.30 | PASS |
| I2b | Install bias, 45 deg turn | roll max err in turn (deg) | < 2 | 0.58 | PASS |
| I3 | Install bias, power-up roll init (original setup()) | roll err at t=0.1 s (deg) | < 1 | 5.92 | **FAIL** |
| I4 | Install bias, power-up roll init with fixKnownBugs | roll err at t=0.1 s (deg) | < 1 | 0.04 | PASS |
| V1 | VSI, Kalman as in original | climb-entry lag (s) | info | 0.01 | info |
| V2 | VSI, Kalman as in original | VSI RMS err steady climb (fpm) | info | 6.4 | info |
| V3 | VSI, Kalman as in original | VSI RMS err in turbulence (fpm) | info | 14.6 | info |
| V4 | VSI, Kalman with constrain() fix | climb-entry lag (s) | info | 0.02 | info |
| V5 | VSI, Kalman with constrain() fix | VSI RMS err steady climb (fpm) | info | 4.5 | info |
| V6 | VSI, Kalman with constrain() fix | VSI RMS err in turbulence (fpm) | info | 8.6 | info |
| V7 | VSI, baro 0.3 m noise without 0.122 mbar quantisation | VSI RMS err steady climb (fpm) | info | 6.5 | info |
| N1 | Zero accel input 2 s (all axes 0 g) | non-finite outputs (count) | < 0.5 | 0 | PASS |
| N2 | Zero accel input 2 s | max attitude err after (deg) | info | 4.67 | info |
| N3 | Madgwick: exact 0 accel and exact-gravity (zero gradient) inputs | non-finite / bad |q| | < 0.5 | 0 | PASS |
| N4 | Gyro 2000 dps all axes for 2 s | non-finite outputs (count) | < 0.5 | 0 | PASS |
| N5 | Gyro 2000 dps for 2 s | time to |att err|<2 deg after (s) | info | 36.5 | info |
| N6 | Gyro saturated at 250 dps for 2 s (rate clipping) | time to |att err|<2 deg after (s) | info | 58.0 | info |
| N7 | Static sensor I2C timeout (-1 mbar) once, original | outputs still NaN 20 s later (1=yes) | < 0.5 | 1 | **FAIL** |
| N7b | Static sensor I2C timeout, original | pitch/roll NaN 20 s later (1=yes) | info | 1 | info |
| N8 | Static sensor I2C timeout (-1 mbar), fixKnownBugs | outputs still NaN 20 s later (1=yes) | < 0.5 | 0 | PASS |
| N9 | IAS glitch 26 kt in 15 deg dive, original | samples with NaN flightPath | < 0.5 | 208 | **FAIL** |
| N10 | IAS glitch 26 kt in 15 deg dive, fixKnownBugs | samples with NaN flightPath | < 0.5 | 0 | PASS |
| N11 | 1 h level flight | | |q|-1 | (fast invSqrt) | < 0.002 | 1.69e-03 | PASS |
| N12 | 1 h level flight | max attitude err last 10 min (deg) | < 1.5 | 1.16 | PASS |

### Flight scenarios x configuration

| ID | Scenario | Metric | Threshold | C0 Gen2 as-is | C1 +gyro cal | C2 +fixes | C3 +AOA comp |
|---|---|---|---|---|---|---|---|
| P1 | Power-up on ground, 11 deg nose up (taildragger), 60 s | attitude err at first sample (deg) | < 1 | 0.56 PASS | 0.56 PASS | 0.57 PASS | 0.57 PASS |
| P2 | Power-up on ground, 11 deg nose up (taildragger), 60 s | max attitude err 0-60 s (deg) | < 1.5 | 1.67 **FAIL** | 1.46 PASS | 1.46 PASS | 1.46 PASS |
| P3 | Power-up on ground, 11 deg nose up (taildragger), 60 s | max |VSI| (IAS<25 -> 0) (fpm) | < 1 | 0.00 PASS | 0.00 PASS | 0.00 PASS | 0.00 PASS |
| L1 | Level cruise 120 kt, 120 s | pitch RMS err (deg) | < 0.75 | 0.71 PASS | 0.57 PASS | 0.57 PASS | 0.58 PASS |
| L2 | Level cruise 120 kt, 120 s | roll RMS err (deg) | < 0.75 | 1.42 **FAIL** | 0.56 PASS | 0.56 PASS | 0.56 PASS |
| L3 | Level cruise 120 kt, 120 s | max attitude err (deg) | < 1.5 | 1.54 **FAIL** | 0.88 PASS | 0.88 PASS | 0.88 PASS |
| L4 | Level cruise 120 kt, 120 s | VSI RMS err (fpm) | < 50 | 2.4 PASS | 2.5 PASS | 4.0 PASS | 4.0 PASS |
| C1 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | pitch max err, steady climb (deg) | < 1 | 1.11 **FAIL** | 0.76 PASS | 0.76 PASS | 0.77 PASS |
| C2 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | VSI mean err, steady climb (fpm) | < 50 | 3.2 PASS | 3.1 PASS | 0.7 PASS | 0.7 PASS |
| C3 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | VSI RMS err, steady climb (fpm) | < 75 | 6.4 PASS | 6.4 PASS | 4.5 PASS | 4.5 PASS |
| C4 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | VSI max err, climb entry 15-45 s (fpm) | < 300 | 20 PASS | 20 PASS | 18 PASS | 18 PASS |
| C5 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | VSI lag vs truth, best-fit delay (s) | < 1.5 | 0.01 PASS | 0.02 PASS | 0.02 PASS | 0.02 PASS |
| C6 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | flight path angle RMS err (deg) | < 0.5 | 0.04 PASS | 0.04 PASS | 0.03 PASS | 0.03 PASS |
| C7 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | derived AOA (pitch - FPA) RMS err (deg) | < 1 | 0.78 PASS | 0.44 PASS | 0.46 PASS | 0.47 PASS |
| C8 | Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s | earthVertG RMS err (g) | < 0.02 | 0.0154 PASS | 0.0151 PASS | 0.0151 PASS | 0.0150 PASS |
| T1 | 45 deg coordinated level turn, 90 s (2.7 orbits), 100 kt | roll max err, steady turn 20-100 s (deg) | < 2 | 1.36 PASS | 1.29 PASS | 1.29 PASS | 0.92 PASS |
| T2 | 45 deg coordinated level turn, 90 s (2.7 orbits), 100 kt | pitch max err, steady turn (deg) | < 2 | 4.12 **FAIL** | 3.95 **FAIL** | 3.95 **FAIL** | 1.31 PASS |
| T3 | 45 deg coordinated level turn, 90 s (2.7 orbits), 100 kt | max attitude err incl. roll in/out (deg) | < 3 | 4.16 **FAIL** | 3.95 **FAIL** | 3.95 **FAIL** | 1.86 PASS |
| T4 | 45 deg coordinated level turn, 90 s (2.7 orbits), 100 kt | VSI RMS err, turn + 30 s after (fpm) | < 100 | 168 **FAIL** | 62 PASS | 13 PASS | 10 PASS |
| T5 | 45 deg coordinated level turn, 90 s (2.7 orbits), 100 kt | earthVertG RMS err in turn (g) | < 0.03 | 0.0159 PASS | 0.0250 PASS | 0.0250 PASS | 0.0219 PASS |
| T11 | 45 deg turn, accel compensation DISABLED (classic Madgwick) | roll max err, steady turn (deg) | info | 8.76 | 6.05 | 6.05 | 6.05 |
| T12 | 45 deg turn, accel compensation DISABLED (classic Madgwick) | pitch max err, steady turn (deg) | info | 15.08 | 10.98 | 10.98 | 10.98 |
| T6 | 60 deg coordinated level turn, 90 s (4.7 orbits), 100 kt | roll max err, steady turn 20-100 s (deg) | < 3 | 5.64 **FAIL** | 4.76 **FAIL** | 4.76 **FAIL** | 0.70 PASS |
| T7 | 60 deg coordinated level turn, 90 s (4.7 orbits), 100 kt | pitch max err, steady turn (deg) | < 3 | 5.46 **FAIL** | 2.14 PASS | 2.14 PASS | 1.72 PASS |
| T8 | 60 deg coordinated level turn, 90 s (4.7 orbits), 100 kt | max attitude err incl. roll in/out (deg) | < 4 | 5.64 **FAIL** | 4.76 **FAIL** | 4.76 **FAIL** | 1.75 PASS |
| T9 | 60 deg coordinated level turn, 90 s (4.7 orbits), 100 kt | VSI RMS err, turn + 30 s after (fpm) | < 100 | 488 **FAIL** | 426 **FAIL** | 127 **FAIL** | 18 PASS |
| T10 | 60 deg coordinated level turn, 90 s (4.7 orbits), 100 kt | earthVertG RMS err in turn (g) | < 0.03 | 0.1136 **FAIL** | 0.1175 **FAIL** | 0.1175 **FAIL** | 0.0289 PASS |
| T13 | 60 deg turn, accel compensation DISABLED (classic Madgwick) | roll max err, steady turn (deg) | info | 6.05 | 3.96 | 3.96 | 3.96 |
| T14 | 60 deg turn, accel compensation DISABLED (classic Madgwick) | pitch max err, steady turn (deg) | info | 9.27 | 6.44 | 6.44 | 6.44 |
| U1 | Pull-up to 2 g, 20 deg climb, push-over (120 kt) | pitch max err (deg) | < 3 | 5.95 **FAIL** | 7.82 **FAIL** | 7.82 **FAIL** | 4.00 **FAIL** |
| U2 | Pull-up to 2 g, 20 deg climb, push-over (120 kt) | roll max err (deg) | < 2 | 2.88 **FAIL** | 0.76 PASS | 0.76 PASS | 0.80 PASS |
| U3 | Pull-up to 2 g, 20 deg climb, push-over (120 kt) | flight path angle max err (deg) | < 3 | 1.34 PASS | 1.65 PASS | 1.35 PASS | 0.51 PASS |
| W1 | Power-off stall approach, level, 80->52 kt at 1 kt/s (pitch 16 deg) | pitch max err (deg) | < 2 | 1.80 PASS | 1.38 PASS | 1.38 PASS | 1.31 PASS |
| W2 | Power-off stall approach, level, 80->52 kt at 1 kt/s (pitch 16 deg) | flight path angle RMS err (deg) | < 1 | 0.14 PASS | 0.13 PASS | 0.05 PASS | 0.05 PASS |
| W3 | Power-off stall approach, level, 80->52 kt at 1 kt/s (pitch 16 deg) | derived AOA RMS err, approach (deg) | < 1.5 | 0.62 PASS | 0.40 PASS | 0.42 PASS | 0.57 PASS |
| W4 | Power-on stall approach, 8 deg climb, 80->52 kt (pitch 24 deg) | pitch max err (deg) | < 2 | 2.62 **FAIL** | 1.89 PASS | 1.89 PASS | 1.32 PASS |
| W5 | Power-on stall approach, 8 deg climb, 80->52 kt (pitch 24 deg) | flight path angle RMS err (deg) | < 1 | 0.10 PASS | 0.10 PASS | 0.05 PASS | 0.05 PASS |
| W6 | Power-on stall approach, 8 deg climb, 80->52 kt (pitch 24 deg) | derived AOA RMS err, approach (deg) | < 1.5 | 0.58 PASS | 0.30 PASS | 0.30 PASS | 0.41 PASS |
| X1 | Forward slip 12 deg roll + 12 deg yaw, -5 deg path, 70 kt (0.19 g lateral) | roll max err incl. entry/exit (deg) | < 3 | 4.98 **FAIL** | 4.12 **FAIL** | 4.12 **FAIL** | 4.08 **FAIL** |
| X2 | Forward slip 12 deg roll + 12 deg yaw, -5 deg path, 70 kt (0.19 g lateral) | roll RMS err (deg) | < 1 | 1.33 **FAIL** | 1.13 **FAIL** | 1.13 **FAIL** | 1.12 **FAIL** |
| X3 | Forward slip 12 deg roll + 12 deg yaw, -5 deg path, 70 kt (0.19 g lateral) | pitch max err (deg) | < 2 | 1.82 PASS | 1.09 PASS | 1.09 PASS | 1.14 PASS |
| X4 | Skidding turn: 30 deg path bank, body 22 deg (0.16 g lateral) | roll max err (deg) | < 3 | 4.97 **FAIL** | 3.69 **FAIL** | 3.69 **FAIL** | 2.80 PASS |
| B1 | Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg | pitch RMS err (deg) | < 1 | 0.85 PASS | 0.62 PASS | 0.62 PASS | 0.67 PASS |
| B2 | Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg | roll RMS err (deg) | < 1 | 1.77 **FAIL** | 0.80 PASS | 0.80 PASS | 0.61 PASS |
| B3 | Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg | max attitude err (deg) | < 3 | 3.35 **FAIL** | 2.41 PASS | 2.41 PASS | 1.51 PASS |
| B4 | Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg | VSI RMS err (fpm) | < 150 | 16 PASS | 5 PASS | 6 PASS | 6 PASS |
| B5 | Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg | flight path RMS err (deg) | < 1 | 0.07 PASS | 0.02 PASS | 0.03 PASS | 0.03 PASS |
| M1 | 10 min pattern: alternating 30 deg turns, one climb, 110 kt | pitch RMS err (deg) | < 1 | 0.66 PASS | 0.59 PASS | 0.59 PASS | 0.71 PASS |
| M2 | 10 min pattern: alternating 30 deg turns, one climb, 110 kt | roll RMS err (deg) | < 1 | 1.45 **FAIL** | 1.03 **FAIL** | 1.03 **FAIL** | 0.76 PASS |
| M3 | 10 min pattern: alternating 30 deg turns, one climb, 110 kt | max attitude err (deg) | < 3 | 5.03 **FAIL** | 3.98 **FAIL** | 3.98 **FAIL** | 1.63 PASS |
| M4 | 10 min pattern: alternating 30 deg turns, one climb, 110 kt | VSI RMS err (fpm) | < 100 | 137 **FAIL** | 47 PASS | 13 PASS | 7 PASS |
| M5 | 10 min pattern: alternating 30 deg turns, one climb, 110 kt | altitude RMS err (ft) | < 10 | 6.04 PASS | 2.51 PASS | 0.24 PASS | 0.20 PASS |
| R1 | In-flight reboot at t=50 s in a 45 deg turn (accel-only init) | roll err right after reboot (deg) | info | 44.75 | 44.75 | 44.73 | 44.73 |
| R2 | In-flight reboot at t=50 s in a 45 deg turn (accel-only init) | time to |roll err| < 2 deg (s) | < 60 | 37.4 PASS | 36.4 PASS | 35.9 PASS | 37.0 PASS |
| R3 | In-flight reboot at t=30 s, level 120 kt (prevTAS = 0 -> TAS step) | max pitch err after reboot (deg) | < 2 | 3.16 **FAIL** | 2.79 **FAIL** | 0.84 PASS | 0.85 PASS |
| R4 | In-flight reboot at t=30 s, level 120 kt (prevTAS = 0 -> TAS step) | time to |pitch err| < 1 deg (s) | < 10 | 4.1 PASS | 3.2 PASS | 0.0 PASS | 0.0 PASS |
| | | **pass / fail** | | 27 / 24 | 38 / 13 | 39 / 12 | 48 / 3 |

single-config tests: 16 pass, 9 fail, 17 info
