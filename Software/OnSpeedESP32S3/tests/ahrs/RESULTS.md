# OnSpeed Gen2 AHRS port: host test results

Port: `components/onspeed_core/include/ahrs.h` and `components/onspeed_core/ahrs.cpp`, ported from
OnSpeed-Gen2 v3.3.7g: `AHRS.ino`, `IMUread.ino`, `MadgwickFusion.cpp`, `KalmanFilter.cpp` and the AHRS part of `setup()`.
The test is `/home/claude/sim/hosttest/ahrs`. Run it with `./run.sh` or `make run` (host g++ only; the
ESP32-S3 cross-compile check uses the toolchain under `/home/claude/sim/tools`).

CMake: add `"ahrs.cpp"` to SRCS in `components/onspeed_core/CMakeLists.txt`:
`idf_component_register(SRCS "onspeed_core.cpp" "tone_synth.cpp" "g5.cpp" "ahrs.cpp" INCLUDE_DIRS "include")`

## What the Gen2 AHRS does (per IMU sample, 208 Hz)

1. **IMU read and axis map** (`IMUread.ino`). The ISM330DHCX runs at 208 Hz ODR, ±8 g and 250 dps, with the gyro HPF and LPF1 off. A 24-entry
   (ports, box-top) table maps the chip axes to vertical, lateral and forward axes. `getGyroForAxis()` negates **every** gyro axis.
   From the code, the resulting conventions (the original does not document them) are:
   `Ax=f_x, Ay=f_y, Az=-f_z` (specific force in g; Az = +1 when level) and `Gx=-p, Gy=-q, Gz=+r` (dps, NED body rates).
   Madgwick therefore runs in a right-handed (aft, left, down) frame, and the code negates the outputs to give pitch up and right roll as positive.
   The closed-loop sim confirms this frame is consistent.
2. **TAS**: `IAS*(1+Palt/1000*0.02)` (or density altitude from OAT). TAS is differentiated at 208 Hz, although IAS only updates at 50 Hz, then smoothed with an EMA (alpha 0.0179).
3. **Installation correction**: a DCM built from pitchBias and rollBias (yaw 0) rotates both the accels and the gyros.
4. **Acceleration compensation**: forward `dTAS/dt`, lateral `TAS*r`, vertical `TAS*q`. These assume the velocity lies along the body x axis.
   The compensation is applied to accels that have been smoothed by an EMA (alpha 0.0609).
5. **Madgwick IMU filter**, beta = 0.011617, with a fast inverse square root (one Newton step). Because the gradient step is normalised,
   the correction slews at a constant **2·beta = 1.33 deg/s**.
6. **earthVertG** = (body specific force rotated to earth by the quaternion) − 1 g, using the corrected but *uncompensated, unsmoothed* accels.
7. **Kalman altitude/VSI** with states z, v and accel bias (har-in-air vario). Settings: R = 0.79078 m², bias PSD 1e-11. The process noise is
   `|a|/50`: the code meant to clamp it to [1, 50] but never does (bug 1 below).
8. VSI is forced to 0 below 25 kt IAS. `flightPath = asin(VSI/TAS)` and `derivedAOA = pitch − flightPath`.
   It also produces 30-sample averages of the rates (gRoll, gPitch, gYaw) for the display and the g-limit logic.

The Matlab `Extended_Kalman_Filter_with-alpha.m` is a separate, unflown 6-state EKF (roll, pitch, gamma, 3 gyro biases, tuned at 238 Hz).
The firmware does not use it, so it was read for reference only and not ported.

## Port notes

* Every value is `float`, and only the f-variants of libm are used. The port compiles cleanly with `-Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Wconversion
  -fsingle-precision-constant -Werror`. `nm` on the ESP32-S3 object shows **no double-precision helpers**: the only FP helper is `__divsf3`,
  and the libm calls are `atan2f`, `asinf`, `sinf`, `cosf`, `powf` and `sqrtf`.
* **Double use in the original** (it costs time on Teensy as well): the `DEG2RAD`, `RAD2DEG`, `MPS2G`, `G2MPS`, `FT2M` and `KTS2MPS` macros are double literals,
  so every expression in `processAHRS()` that uses them is promoted to double. The original also calls `cos/sin/asin/atan2/pow` (double) in
  `processAHRS()` and `Madgwick::begin()`, and assigns the double literal `Pav_ = 0.0` in Kalman `Configure()`. The port removes all of this.
* The original computes the install-rotation sin/cos (about 20 trig calls) on every sample. The port caches them in `setInstallBias()`, and the results are identical.
* `fixKnownBugs` (default **false**, which gives a faithful port) enables the fixes below. `aoaCompensation` (default false) is an **experimental, non-original**
  option that uses `omega x v` with `v = TAS·(cos AOA, 0, sin AOA)`. `compensation=false` is a test-only switch.

## Simulator and sensor model (`sim.h`)

* **Truth model**: point-mass wind-axis kinematics (V, gamma, chi), driven by commanded Vdot, bank mu, and load factor n (or a flight-path-hold law).
  The model is integrated with RK4 at 8×208 Hz. The body frame is the wind frame rotated by sideslip, by AOA (an RV-4-like lift curve, α = 43277·n/V_kt², which gives 16° at 52 kt) and by a slip
  roll offset. Specific force is `f_w = (Vdot + g sinγ, 0, −n g)`. Gyro values are the mean body rate from a central difference of the attitude DCM.
  Self-checks S1 and S2 pass to 1e-16.
* **IMU (ISM330DHCX-like)**: white noise 80 µg/√Hz and 5 mdps/√Hz over fs/2, giving 0.82 mg and 0.051 dps rms. Constant bias is ±10 mg and ±0.3 dps
  per chip axis with random signs. Bias random walk is 0.04 mg/√s and 0.002 dps/√s, which is about 1 mg and 0.05 dps over 10 min, similar to a temperature drift of about 10 °C.
  Readings are quantised to int16 at ±8 g and 250 dps, written in chip axes for the FORWARD/UP mounting, and then passed through the ported `scaleImu()`
  and `mapAxes()`. The orientation is therefore exactly what IMUread produces.
* **Static pressure**: ISA, with 0.3 m white noise and then quantisation to the SSCSRNN1.6BA7A3's 14-bit counts (0.122 mbar ≈ 1 m at sea level).
  Palt uses the firmware formula.
* **IAS**: 50 Hz samples, held between updates. Pfwd carries 2.1 Pa rms of noise and 1.05 Pa quantisation (±1 psi, 14-bit), then passes through
  median(15) and SMA(10) as in `SensorRead()`. That chain gives about 0.23 s of delay.
* The default mounting is FORWARD/UP (ISM330), at 3000 ft.
* **Configurations in the matrix**:
  * **C0** is Gen2 exactly as ported, with the task's error budget (gyro bias 0.3 dps uncalibrated).
  * **C1** is C0 plus Gen2's own ground gyro calibration (`gxBias = −mean`), so only drift remains.
  * **C2** is C1 with `fixKnownBugs`.
  * **C3** is C2 with the experimental `aoaCompensation`, using truth AOA plus a 0.5° bias.

Thresholds are requirement-style limits that I chose before looking at the results (roughly what a pilot-facing AHRS would need: 1° static, 2–3° dynamic, VSI within 100 fpm).
A FAIL is a finding about the algorithm, not a defect in the harness.

## Results
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

## What the numbers say

**Steep turns (classic Madgwick turn drift).** With compensation disabled (T11–T14), Madgwick drifts 6–9° in roll and 9–15° in pitch.
Gen2's TAS-based compensation fixes **roll in 45° turns** (1.3° max) and roughly halves the pitch error. It does **not** fix the steep-turn case:
* In the 45° turn, pitch is off by 4° (T2).
* In the 60° turn, roll is off by 4.8–5.6° (T6).
* The roll error feeds earthVertG (−0.12 g), which gives a VSI error of 430–490 fpm in a level 60° turn (T9).

Root cause: the compensation assumes velocity along the body x axis, so it leaves out the `q·w = q·TAS·sin(AOA)` forward term
(0.075 g at 45° and 6° AOA; 0.23 g at 60° and 8.7° AOA), plus smaller `p·w` and `(1−cos α)` terms.
With the low beta, the error target rotates at the turn rate, which spreads the error into both roll and pitch.
Using the AOA OnSpeed already measures (C3) cuts the 60° turn to 0.7° roll, 1.7° pitch and 18 fpm VSI error, and C3 passes 48 of 51 matrix checks.

**Gyro bias.** Two effects:
* Madgwick rejects pitch- and roll-axis bias up to the 1.33 °/s slew limit. G2 shows 0.5° at 0.3 dps and G5 1.1° at 1.27 dps.
  Above that limit the attitude diverges (G6: 58°; G4 at 1.0 dps on all axes: 39°).
* The **yaw-gyro bias is multiplied by TAS** in `aLatCp = TAS·r`. The result is a steady roll error of atan(TAS·b/g): 1.9° at 120 kt for 0.3 dps (G1 measured 2.0°, predicted 1.9°).
  This is the main reason C0 fails in level flight (L2: 1.42° roll RMS), and Gen2's gyro calibration fixes it (C1: 0.56°).
  Because the ISM330 has no temperature compensation in Gen2, a temperature change after calibration (about 0.005 dps/°C typical)
  brings this error back, at roughly 0.6° per 0.1 dps at 120 kt.

**VSI and flight-path angle.** The Kalman filter is excellent in straight flight:
* Steady-climb error is 3 fpm mean and 4.5–6.4 fpm RMS (C2–C3).
* The best-fit lag against truth is 0.01–0.02 s (the accel term makes it nearly lag-free).
* The error peaks at 20 fpm during a 3-second climb entry.
* Flight-path angle error is 0.03–0.04° RMS, and the derived AOA is within 0.3–0.8° RMS.

The weak spot is that the original Kalman trusts the accelerometer almost completely: the dead `constrain()` leaves the process noise at about 0.
Any earthVertG error from attitude error (turns) therefore becomes a large VSI error that recovers slowly (T4 C0 168 fpm, T9 488 fpm).
With the clamp fixed (C2), T4 drops to 13 fpm, M4 from 47 to 13 fpm, and turbulence VSI from 14.6 to 8.6 fpm (V3/V6). Climb lag is unchanged.

**Convergence.** On the ground, attitude is seeded from the accels, so the first sample is within 0.56° (P1, limited by the 10 mg accel bias).
After an in-flight reboot in a 45° turn, the accel seed is 45° wrong (R1), and the 1.33 °/s slew limit means about 37 s to get within 2° (R2).
An in-flight reboot in level flight kicks pitch by 3.2° (R3) because of bug 4.

**Other remaining FAILs.**
* **Slip entry and exit (X1/X2):** a yaw rate with no path curvature is treated as centripetal, so roll is off by up to 4–5° during the transient.
  This is inherent to `TAS·r`, and AOA compensation does not help.
* **2 g pull-up (U1):** pitch is off by 4–8°, from the AOA term plus the 0.23 s IAS/EMA lag on dTAS/dt during deceleration.
* **Ground (P2):** pitch wanders by up to 1.5° from IAS noise at 0 kt, which feeds aFwdCp because nothing gates the compensation at low IAS.
* **Fast inverse sqrt:** it keeps |q| = 0.9983 (N11), so earthVertG carries a −3.4 mg offset that the Kalman bias state absorbs.

**Numerical robustness.**
* Exactly zero accel, a zero gradient, 2000 dps, and saturation at 250 dps all stay finite (N1, N3, N4). Recovery from a 2 s rate glitch takes 37–58 s at the 1.33 °/s slew.
* NaN cases are listed under bugs 2 and 3 (N7, N9).
* The 1 h run shows no drift (N12).

## Bugs and defects found in the original

1. **`KalmanFilter.cpp`: `constrain(zAccelVariance_, 1.0f, 50.0f);` discards its result.** The process noise is `|a|/50` (about 0 in smooth flight),
   and the `zAccelVariance` value of 26.0638 tuned in `setup()` is overwritten and never used. The fix (C2) improves VSI after turns by 5–10×.
2. **A single static-pressure I2C timeout permanently NaNs the AHRS.** `GetStaticPressure()` returns −1, and Palt = `145366*(1−pow(negative, 0.19))` is NaN.
   The NaN TAS sticks in `TASdiffSmoothed` and the Kalman state, so pitch, roll, VSI, flightPath and derivedAOA stay NaN until reboot (N7, N7b). The fix holds the last finite Palt (N8).
3. **`flightPath = asin(VSI/TAS)` is NaN when |VSI| > TAS**, for example after a pitot glitch to 26 kt in a dive: 208 NaN samples (N9). The fix clamps the argument (N10).
4. **`prevTAS` starts at 0**, so the first sample after a reboot sees a TAS step of about 60 m/s. aFwdCp spikes to about 20 g and pitch is kicked 3.2° (R3). The fix seeds prevTAS.
5. **Roll initialisation sign:** `setup()` uses `+calcRoll()+rollBias`, while the sensor calibration (`WifiSerial.ino`) defines rollBias against `−calcRoll()`.
   With a nonzero rollBias the filter starts 2·rollBias off (5.9° for a 3° install, I3). With rollBias = 0 the two sign errors cancel. The fix gives 0.04° (I4).
6. **`AHRS.ino` rollRateCorr typo:** `sin(installPitchRad) * - sin(installYawRad)` has `*` where `-` was meant, which drops the `Gy·sinR·sinP` term.
   This is second order: 0.4% of the pitch rate for a 5°/3° install.
7. **Axis tables:**
   * The ISM330 entry **LEFT/DOWN** (`"-Z","-Y","X"`) has the wrong forward sign (should be `-X`). It is the only one of the 24 entries with the wrong handedness, so on that install pitch and the roll gyro are inverted (A2/A3).
   * The LSM9DS1 table has the opposite handedness to the ISM330 table (it was derived by swapping X and Y, which is a reflection), so at most one of the two can be right for a right-handed chip.
   * In the LSM9DS1 table, **FORWARD/LEFT** still has the error that was fixed for the ISM330 ("Paul's, fixed (used to be +Z)"), and **LEFT/DOWN** has the same error as the ISM330 entry.
8. Minor:
   * Madgwick computes yaw (one extra `atan2f`) that nothing uses.
   * `checkGlimit()` tests `gRoll>=` and `gYaw>=` one-sided (no `fabs`), and since gRoll = −p it only triggers for left roll.
   * There is no low-IAS gate on the acceleration compensation.

## Cost per update

| | value |
|---|---|
| Host (x86-64, g++ -O2) | processAHRS **≈ 90–125 ns** (run to run). Of that: Madgwick update + angles 60–95 ns, Kalman 16–21 ns, scale + map 10–14 ns |
| Dynamic float ops / update (counted by compiling the unmodified ahrs.cpp with a counting float type) | **141 add/sub, 242 mul, 9 div, 3 cmp, 6 neg, 1 fabs, 2 atan2f, 2 asinf** (no sqrt, no trig except with OAT or aoaComp). Madgwick is 47 add and 100 mul; Kalman is 40 add, 43 mul and 3 div |
| ESP32-S3 static code (xtensa-esp32s3 gcc 14.2 -O2) | per-sample path ≈ 1000 instructions, 280 FPU arithmetic (123 fused madd/msub), 304 FPU loads and stores |

The ESP32-S3 estimate rests on these assumptions:
* FPU add, mul and madd issue at 1 cycle, with about 30% stall overhead.
* `__divsf3` is the div0.s/maddn/divn.s sequence (27 instructions in libgcc), about **40 cycles** including the call.
* newlib `atan2f` is about 250 cycles and `asinf` about 200 (115 + 181 and 213 static instructions, polynomial plus a division).
* `sqrtf` is about 40 (not used per sample).

Total: about 1250 cycles of own code, plus 9×40 for divisions, 2×250 for atan2f and 2×200 for asinf, which comes to **≈ 2500 cycles ≈ 10 µs at 240 MHz**, or about 0.2% of one core at 208 Hz.
Cheap savings that do not change behaviour:
* Drop the unused yaw `atan2f` (−250 cycles).
* Precompute `1/imuSampleRate` and use multiplications for `/1000`, `/50` and `/cnt` (−5 divisions).
