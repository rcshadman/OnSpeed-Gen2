# OnSpeed port: real-world numbers for simulation and tests

Generated 2026-10-25. The machine-readable version is in `real-data.json`, which uses the same section names.

Each number is labelled with its confidence:
- **[DS]**: quoted from a manufacturer datasheet or document.
- **[REPO]**: taken from the OnSpeed Gen2 repo.
- **[DER]**: computed from the two kinds above.
- **[EST]**: an estimate, because no retrievable source was found. Replace these with bench measurements.

Some of the web was not reachable from this sandbox: github.com, esp32.com (bot check), te.com (robots) and tc.faa.gov (robots). Firecrawl also had no credits left. Datasheets were read through mirrors instead, and these are listed in the Sources section.

---

## 1. Pressure sensors

### Correction: the differential sensors are TE, not Honeywell
The Assembly Procedure, STEP 27, gives the two dual-port sensors as **TE Connectivity (MEAS) 4525DO-DS5AI001DP** (the schematic labels them "MS4525"). The part number decodes as:
- DS: dual side port
- 5: 5 V grade
- A: Type A output, 10–90 %
- I: I²C address 0x28
- 001: 1 psi
- D: differential

The two sensors are:
- **PS1 = P45**, on Wire2 (Teensy pins 3/4).
- **PS2 = Pfwd**, on Wire (pins 18/19).

The static sensor is **Honeywell SSCSRNN1.6BA7A3** (STEP 29). It is a TruStability SSC: SIP package, radial port, 0–1.6 bar absolute, I²C address 0x78, 10–90 % output, 3.3 V. It sits on Wire1 (pins 37/38), shared with the IMU.

### 4525DO-DS5AI001DP (Pfwd, P45)
| Item | Value |
|---|---|
| Range | ±1 psi (±6894.757 Pa) differential [DS] |
| Transfer function | counts = 0.8·16383·(P−Pmin)/(Pmax−Pmin) + 0.1·16383. Gives 1638 at −1 psi, **8192 at zero**, and 14745 at +1 psi [DS] |
| Scale | **1.052 Pa/count** (1.526e-4 psi) [DER] |
| Accuracy | ±0.25 %Span BFSL [DS] |
| Total error band | **±1.0 %Span = ±131 counts = ±138 Pa**, over −10…85 °C, at the calibrated supply [DS/DER] |
| Update time | **0.5 ms**. First data comes 8.4 ms after power-on reset [DS] |
| Noise | Not specified in the datasheet. **Use 1.5 counts RMS** until measured [EST] |
| Long-term drift | Not specified (life 1 M FS cycles) [DS] |
| Temperature output | 11-bit. T(°C) = counts·200/2047 − 50 [DS] |
| Supply | 3.3/5 V, ~3 mA. The output is **not** ratiometric [DS] |
| Status bits (byte 1, bits 7:6) | 00 normal; 01 reserved/command mode; **10 stale** (already fetched since the last conversion); 11 fault/diagnostic [DS, Honeywell TN, same protocol] |
| Read formats | 2 bytes = status + 14-bit P. 3 bytes adds T[10:3]. 4 bytes adds T[2:0] in bits 7:5 [DS] |
| I²C clock in the docs | 100–400 kHz (fast mode) |
| **Gen2 I²C clock** | **3 MHz** on both buses (i2c.ino:117,121). This is far outside the documented spec. The ESP32-S3 master is limited to **400 kHz** |
| Gen2 read | write(0) then requestFrom(0x28,2), at 50 Hz. Status bits are masked off (0x3F) and never checked [REPO] |
| Reading faster than the update rate | The sensor returns the previous buffer with status=10. This never happens at 50 Hz [DS] |
| **Real zero offsets (RV-4)** | PFWD_BIAS 8108, P45_BIAS 8058, against an ideal 8192. That is **−84 and −134 counts (−0.64 / −1.02 %Span)**, at the edge of the TEB [REPO default_config.h] |

Dynamic pressure in counts, at sea level [DER]:

| kt | 10 | 20 | 40 | 50 | 60 | 100 | 150 |
|---|---|---|---|---|---|---|---|
| q (Pa) | 16 | 65 | 259 | 405 | 584 | 1621 | 3647 |
| counts above zero | 15 | 62 | 247 | 385 | 555 | 1541 | 3467 |
| kt per count | 0.32 | 0.16 | 0.081 | 0.065 | 0.054 | 0.033 | 0.022 |

The full-scale reading (+1 psi) is about 206 kt.

### SSCSRNN1.6BA7A3 (static)
| Item | Value |
|---|---|
| Transfer function | P_mbar = (counts−1638)·1600/(14746−1638). Gen2 uses (raw−1638.4)/8192·1000, which is the same [DS/REPO] |
| Scale | **0.122 mbar/count** (12.2 Pa, about 3.4 ft at sea level) [DER] |
| Total error band | **±2 %FSS = ±32 mbar** [DS] |
| Accuracy | ±0.25 %FSS BFSL [DS] |
| Long-term stability | ±0.25 %FSS per 1000 h [DS] |
| Response time | 0.46 ms typical. Startup 3 ms max. The output updates at about 1 kHz [DS] |
| Resolution | 14-bit output, at least 12-bit effective [DS] |
| Compensated range | −20…85 °C [DS] |
| Supply current | 3.1 mA typical / 3.9 mA max at 3.3 V [DS] |
| I²C | 100–400 kHz. Gen2 runs it at 400 kHz and reads 4 bytes at 208 Hz (v3.3.0, Kalman VSI) [DS/REPO] |
| Noise | Not specified. Use 1 count RMS [EST] |
| Real bias (RV-4) | PSTATIC_BIAS = −2.20 mbar [REPO] |

---

## 2. IMU: ISM330DHCX (the define in OnSpeedTeensy_AHRS.ino:119; older v3 boards used the LSM9DS1)

The Gen2 configuration (IMUread.ino):
- **CTRL1_XL = 0b01011100**: 208 Hz, ±8 g, LPF2 off, so the bandwidth is ODR/2 = 104 Hz.
- **CTRL2_G = 0b01010000**: 208 Hz, ±250 dps.
- High-pass filter off, LPF1 off, FIFO bypass.
- Reads 12 bytes every 4808 µs on Wire1 at 400 kHz.

| Parameter | Accel (±8 g) | Gyro (±250 dps) |
|---|---|---|
| Sensitivity | 0.244 mg/LSB ±2 % | 8.75 mdps/LSB ±2 % |
| Sensitivity tempco | ±0.005 %/°C typ (0.01 max) | ±0.007 %/°C typ (0.015 max) |
| Zero offset | ±10 mg typ (±65 max) | ±1 dps typ (±3 max) |
| Offset tempco | ±0.1 mg/°C typ (±0.5 max) | ±0.005 dps/°C typ (±0.015 max) |
| Noise density (high-performance mode) | 60 µg/√Hz typ (100 max) | 5 mdps/√Hz typ (8 max) |
| RMS at Gen2 bandwidth | 0.61–0.77 mg, about **2.5 LSB** (104 Hz) [DER] | 41–51 mdps, about **5 LSB** (about 67 Hz assumed) [DER] |

Other ISM330DHCX figures [DS]:
- Temperature sensor: 256 LSB/°C, 0 LSB at 25 °C, ±15 °C offset.
- Turn-on time: 35 ms.

Real RV-4 biases [REPO]:
- GX/GY/GZ = −0.167 / +0.290 / +0.322 dps.
- Pitch bias −4.29°, roll bias −0.34°.

**Real measured temperature compensation for the LSM9DS1** (box cycled from −25 °C to 50 °C, in IMUread.ino:111-116). This is useful for a realistic drift model:
```
ax -= -2.996e-07 T² - 2.899e-05 T - 0.01741   ay -= -8.759e-07 T² + 7.991e-04 T - 0.03251
az -= -3.799e-06 T² - 1.109e-04 T + 0.001731  gx += 2.9e-04 T² + 0.08007 T - 2.261
gy += 4.215e-04 T² + 0.004307 T - 0.3264       gz += 4.122e-04 T² - 0.01383 T + 0.04016
```
On that unit, gx drifts by about 0.08 dps/°C. The ISM330DHCX was chosen because its drift is about 10× smaller.

---

## 3. Flap position pot and the ESP32-S3 ADC

**How Gen2 reads the flap pot** [REPO]:
- The pot is on A2 (DB15 pin 7) with `INPUT_PULLUP`, so the pot is a rheostat to ground. This is why the manual says "ohms".
- Readings are 10-bit (13-bit only if `HIGHRES_ANALOGREAD` is defined), at 50 Hz.
- The RV-4 positions are **129 / 158 / 206 counts** for flaps 0/20/40. The smallest half-gap between zones is 14.5 counts at 10-bit, which is **about 58 counts at 12-bit**.
- The volume pot range is 170–339 counts.

| ESP32-S3 ADC1 | Value |
|---|---|
| Resolution | 12-bit SAR, 100 kSPS [DS via Espressif blog] |
| DNL / INL | **±4 / ±8 LSB** [DS] |
| Range | 0–2900 mV at 12 dB (IDF oneshot docs list 0–950 / 1250 / 1750 / 3100 mV for 0 / 2.5 / 6 / 12 dB). Nonlinear above about 2750 mV |
| Vref | 1100 mV design, 1000–1200 mV chip to chip. eFuse curve-fitting calibration applies |
| Calibrated error | −30…0 mV up to about 2750 mV (Espressif measurement) |
| Noise | **No retrievable measured S3 figure.** Model 3 LSB RMS without Wi-Fi, 6 LSB RMS with the AP active, and occasional 30 LSB spikes during Wi-Fi TX [EST]. Sources for the original ESP32 cite ±10–30 LSB raw. ENOB is about 9–10 bits [EST] |
| Wi-Fi | ADC1 works with Wi-Fi. **ADC2 is shared with Wi-Fi**, so never put the pots on ADC2 |

Recommendations for the port:
- Oversample 16–64× and apply a median filter.
- Use an **external** pull-up of about 10 kΩ, or wire the pot as a divider, rather than relying on the internal pull-up. Internal pull-ups vary widely and are not specified tightly.
- Keep the input under about 2.7 V at 12 dB.

---

## 4. Audio

**Gen2 today** [REPO + PJRC source]:
- Output comes from Teensy 3.6 DAC0/DAC1 through AudioOutputAnalogStereo (12-bit, 44.1 kHz).
- The library's default reference is the **1.2 V internal** one (`output_dacs.cpp`: "DAC0_C0 = DAC_C0_DACEN; // 1.2V"), and Gen2 never calls `analogReference(EXTERNAL)`.
- Full scale is therefore **1.2 Vpp = 0.42 Vrms sine**, on a 0.6 V DC offset.
- Two 150 Ω resistors (R11/R14) sit beside AUDIO R/L on the v3 schematic, probably as series output resistors (verify on the board).
- Tone amplitudes:
  - Low tone 0.25.
  - High tone ramps from 0.25 to 1.0; stall tone 1.0.
  - Everything is multiplied by volume % and by the 3D-audio gain (0–2, which can clip).
- At 100 % volume this gives about **0.11 Vrms for the 400 Hz tone and 0.42 Vrms for the stall tone**.

**What the OnSpeed docs say about wiring** [REPO]:
- DB15 pin 6 = AUDIO RIGHT, pin 8 = AUDIO LEFT, pin 15 = AUDIO GND.
- Connect these to a stereo intercom or music input to get 3D audio.
- An "optional audio mixer" exists for mono intercoms or when there is "insufficient" volume, because it "provides amplification". In other words, Gen2 output level is already on the low side.
- The docs give no recommended level.

**PCM5102A** [DS, TI]:
- 2.1 Vrms full scale (5.94 Vpp), ground-centred, so no DC-blocking capacitors are needed.
- 112 dB SNR/DR (the 5101A is 106 dB, the 5100A 100 dB).
- Line driver rated down to a **1 kΩ** load.
- Mute SNR 120 dB.
- THD+N about −93 dB, from memory; the TI table did not render, so verify.

**Audio panel inputs**:

| Panel | Input | Impedance | Level |
|---|---|---|---|
| Garmin GMA 240 | Music (differential) | 600 Ω | <200 mVrms for full power at max volume; 3.5 Vrms max |
| Garmin GMA 240 | Others | 500 Ω | – |
| Garmin GMA 245 | Music | 600 Ω | <200 mVrms for full power; 3.0 Vrms max |
| Garmin GMA 245 | 4 alert inputs | 600 Ω | 5 Vrms max |
| Garmin GMA 340 | Inputs (2 unswitched) | 500 Ω | – |
| PSE PMA450 | Unswitched 1–4 | 510 Ω | 1–2 fixed 1:1, adjust at the source; 3–4 adjustable |
| PSE PMA450 | 2 stereo music inputs | not stated | – |
| PSE PMA8000 | – | manual not retrievable (expected 500–600 Ω; verify) | – |

**Divider recommendation.** To match Gen2's level, PCM5102A full scale needs to come down by ×0.20 (−13.9 dB). There are two ways to do this:
- **A (no parts):** scale I²S samples by 0.2 in firmware. This still leaves about 98 dB of dynamic range.
- **B (pad, per channel):** **R_series 1.0 kΩ, R_shunt 470 Ω to AUDIO GND.**
  - The PCM5102A sees 1.47 kΩ, which is above its 1 kΩ minimum.
  - Source impedance is 320 Ω.
  - Full-scale output: 0.67 Vrms open circuit, **0.44 Vrms into 600 Ω**, and 0.41 Vrms into 510/500 Ω. That matches Gen2 at 0.42 Vrms.
  - Using 330 Ω instead of 470 Ω gives 0.37 Vrms into 600 Ω.

Do not go below Gen2's level, because users already find it quiet. A pad plus about ±6 dB of firmware gain is the most flexible option.

---

## 5. Timing

| Item | Value |
|---|---|
| ESP32-S3 CoreMark | **1329.92 for two cores at 240 MHz (5.54 CM/MHz)**, about 2.77 CM/MHz per core [DS] |
| Teensy 3.6 CoreMark | **440.72 at 180 MHz (2.45 CM/MHz)** [PJRC CoreMark README]. One S3 core is roughly 1.5× a Teensy 3.6 |
| S3 float loop cost (Espressif blog, average of 10k iterations including loop) | add 25, **div 69**, **cosf 121**, cosf(sqrtf) chain 312 cycles. Double: 70 / 75 / 1619 / 3886. **Avoid double**, which is emulated in software |
| FPU per instruction | add.s / mul.s / madd.s about 1 cycle throughput. div and sqrt have no single-cycle instruction (DIV0.S/SQRT0.S plus Newton steps): about 20–70 and 20–40 cycles. expf about 150–300, sinf about 120 [EST, apart from the measured cosf/div] |
| qf_math vs libm on S3 | sinf 4.3×, cosf 4.4×, expf 2.6×, logf 2.2×, atan2f 1.4×, sqrtf 1.07× |
| esp-dsp on S3 | dotprod 256 = 432 cycles; FFT 1024 complex = 97,847 cycles; biquad = 17.1 cycles/sample |
| I²C bus time at 400 kHz [DER, 9 bits/byte] | **MS4525 2-byte read 72.5 µs**; SSC 4-byte read 117.5 µs; 2-byte register read 120 µs; ISM330 12-byte register read 345 µs. Add about 20–60 µs of IDF driver overhead per transaction [EST]. On Gen2 at 3 MHz the 4525 read takes 9.7 µs |
| Bus budget | 50 Hz × (2×72.5) plus 208 Hz × (345+117.5) ≈ 7.3 + 96 ms/s, which is about **10 % of one 400 kHz bus**. Fine, but keep the IMU and static sensor off the 4525 buses, as Gen2 does |
| Wi-Fi | Driver task priority 23, pinned to core 0 [IDF docs]. CPU load in AP mode serving a page: **no measured source found**. Model 5–20 % of core 0, bursty [EST]. Measure with `vTaskGetRunTimeStats` |
| Interrupt latency | No retrievable figure (esp32.com blocked). Model about 2 µs for a GPIO ISR in IRAM through the IDF dispatcher [EST] |
| I²S DMA | Each DMA buffer must be ≤ 4092 bytes. buffer = dma_frame_num·slots·bits/8, and the interrupt interval is dma_frame_num/fs. Example: 44.1 kHz stereo 16-bit with 240 frames gives 960 B every 5.44 ms |
| S3 I²C SCL max | 400 kHz. Recommended pull-ups 2–5 kΩ. Clock stretch can last up to 12 ms |

Gen2 runtime facts [REPO]:
- Sensors are read and logged at 50 Hz; the IMU runs at 208 Hz.
- Pressure filtering is a median of 15 followed by an average of 10.
- AOA smoothing defaults to 20 (10 in the RV-4 config). AOA is clamped to −20…40°.
- CP = P45/Pfwd.
- The I²C timeout is 2000 µs.
- Logs grow at about 50 MB per hour.

---

## 6. Real OnSpeed flight data

**No public SD log CSVs were found.** Places checked:
- `TestFlightData/`, which holds only a README.
- The local clone's git history, which is shallow (2 commits).
- flyonspeed.org: technical resources and the calibration and blog posts.
- raw.githubusercontent paths for flyonspeed and flyonspeed-org Gen1/Gen2/Gen3. Only Gen1 and Gen2 exist, and neither has data.
- Van's Air Force threads and the EAA white paper.

flyonspeed.org says data is shared on request (team@flyonspeed.org). Forks on github.com could not be listed from this sandbox. `real-logs/` was not created because nothing was downloaded.

**Log format** (LogData.ino) [REPO]:
- 50 Hz, one file `log_N.csv` per boot.
- Header: `timeStamp,Pfwd,PfwdSmoothed,P45,P45Smoothed,PStatic,Palt,IAS,AngleofAttack,flapsPos,DataMark`
- Optional `,OAT,TAS`.
- Then `,imuTemp,VerticalG,LateralG,ForwardG,RollRate,PitchRate,YawRate,Pitch,Roll`.
- Optional boom columns: `boomStatic,boomDynamic,boomAlpha,boomBeta,boomIAS,boomAge`.
- Optional VN-300 (26 columns) or EFIS (18 columns).
- Then `,EarthVerticalG, FlightPath, VSI, Altitude`.

**Real calibration data (Vac's RV-4, from default_config.h and "RV-4 Quick Look Up.docx")**:
- Flap pot positions: 129 / 158 / 206 for flaps 0 / 20 / 40.
- AOA(CP), 2nd-order fit with the VN-300 auto-cal:
  - F0 = 8.4845 CP² + 24.0804 CP + 4.6157
  - F20 = 4.5244 CP² + 27.9928 CP + 2.7661
  - F40 = 1.2984 CP² + 30.4239 CP + 1.4550
- Linear trim-shot fits: F0 22.556 CP + 4.1106; F20 23.318 CP + 2.5496; F40 24.959 CP + 0.7909.
- Setpoints in degrees, VN-300 auto-cal (F0 / F20 / F40):

  | Setpoint | F0 | F20 | F40 |
  |---|---|---|---|
  | L/Dmax | 8.03 | 5.73 | 4.78 |
  | OnSpeed fast | 11.25 | 9.74 | 8.62 |
  | OnSpeed slow | 13.84 | 12.64 | 12.44 |
  | Stall warning | 16.48 | 16.29 | 14.51 |

- Trim-shot setpoints, AOA° (mph):

  | Flaps | L/D | OS fast | OS slow | Stall |
  |---|---|---|---|---|
  | F0 | 5.45 (97.5) | 9.33 (77) | 11.6 (69) | 14.3 (56.5) |
  | F20 | 3.70 (94) | 7.03 (75.5) | 10.0 (63) | 14.01 (62) |
  | F40 | 2.17 (94) | 5.77 (74.5) | 8.69 (66) | 12.47 (60.5) |

- CAS = 0.977·IAS + 4.162.
- G limits +4 / −2. VNO 157.
- From the F0 curve, CP is about 0.135 at L/Dmax, 0.254 at OnSpeed fast, and 0.42 at stall warning. Use these CP values when building synthetic logs.

---

## 7. Aerodynamics: RV-4

**Baseline setpoints in mph IAS at 1 g** ("RV-4 Quick Look Up", 5000 ft PA, BOW 1148 lb, 1.25–1.35 Vs):

| | Flaps 0 | Flaps 20 | Flaps 40 |
|---|---|---|---|
| L/Dmax | 90 | 90 | 90 |
| OnSpeed fast | 77 | 75 | 74 |
| OnSpeed slow | 69 | 67 | 66 |
| Stall warning | 63.5 | 60 | 59 |
| Stall | 58.5 | 55 | 54 |

The test weight runs from 1208 to 1328 lb as fuel goes from 10 to 30 gal (6 lb/gal).

Van's figures and pilot reports:
- Van's: stall 54 mph, gross 1500 lb, wing 110 ft², span 23 ft, chord 54 in, white arc 54–100 mph, green arc 58–180 mph, Vne 210 mph.
- Pilot-reported (VAF): best glide / L/Dmax about 105 mph (Vac); Carson speed about 140 mph; Vx 75–100 mph; Vy 100–140 mph.
- Stall with flaps down: 45–55 kt.
- Final approach with full flaps: 60–75 kt.

**OnSpeed relationships** (user's manual, EAA white paper, blog):
- OnSpeed AOA ≈ **1.73 × L/Dmax AOA** (minimum power required, 1 g Vref ≈ 1.3 Vs). Carson AOA ≈ 0.58 × L/Dmax AOA.
- The OnSpeed band is 1.25–1.35 Vs: about 1° wide and 5 kt.
- Stall warning comes at stall + 5 kt (about 1.1 Vs), 1.5–2° of AOA before stall.
- Percent lift: L/Dmax 50 %, OnSpeed 60 %, stall warning > 90 %.
- Tones: 400 Hz at 1.5–8.2 pps (fast), solid 400 Hz (OnSpeed), 1600 Hz at 1.5–6.2 pps (slow), 22 pps (stall).
- Accuracy is 0.1–0.25° at ≤ 2 g and 0.5–1° at 6 g. Measured average absolute error with the Dynon probe was 0.084 / 0.089 / 0.125° for flaps 0 / 20 / 40.
- Calibration: decelerate at about 1 kt/s, capture 10 points per second, R² ≥ 0.98, stall detected at pitch rate > 5 °/s.

---

## Sources
- OnSpeed repo: Documents/OnSpeed Gen 2 v3 Assembly Procedure.pdf, ONSPEED User's Manual 28 Sep Version.pdf, Hardware/onspeed_Gen2_v3_schematic.pdf, RV-4 Quick Look Up.docx, Software/OnSpeedTeensy_AHRS/{SensorRead,IMUread,i2c,LogData,Flaps,Volume,Tones,AOAcalc}.ino, default_config.h, OnSpeedTeensy_AHRS.ino
- MS4525DO datasheet: https://www.mouser.com/datasheet/2/418/MS4525DO-710170.pdf
- Honeywell TN "I2C Communications with Honeywell Digital Output Pressure Sensors" (status bits, read formats, I²C timing): https://forums.parallax.com/uploads/editor/79/9ekv6t69d0fx.pdf
- Honeywell TruStability SSC datasheet (TEB 1.6BA, digital transfer function, address code 7=0x78): https://docs.rs-online.com/e3e7/A700000015336742.pdf
- ISM330DHCX datasheet: https://cdn.sparkfun.com/assets/d/4/6/d/f/ism330dhcx_Datasheet.pdf
- Espressif ADC performance blog: https://developer.espressif.com/blog/2025/08/adc-performance/
- ESP-IDF ADC (S3): https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32s3/api-reference/peripherals/adc.html and https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/index.html
- ESP32-S3 datasheet (CoreMark): https://documentation.espressif.com/esp32-s3_datasheet_en.pdf
- Espressif "cores with FPU" blog (float cycle counts): https://developer.espressif.com/blog/2025/10/cores_with_fpu/
- qf_math S3 ratios: https://components.espressif.com/components/deftio/qf_math/versions/1.0.1/readme
- esp-dsp benchmarks: https://docs.espressif.com/projects/esp-dsp/en/latest/esp-dsp-benchmarks.html
- ESP-IDF speed guide (task priorities): https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/performance/speed.html
- ESP-IDF I2S / I2C (S3): https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2s.html , .../i2c.html
- Teensy CoreMark: https://raw.githubusercontent.com/PaulStoffregen/CoreMark/master/README.md
- Teensy audio DAC reference: https://raw.githubusercontent.com/PaulStoffregen/Audio/master/output_dacs.cpp
- PCM5102A: https://www.ti.com/product/PCM5102A/part-details/PCM5102APW
- GMA 240: https://www.aircraftspruce.com/catalog/pdf/GMA240.pdf ; GMA 245 (190-01879-11 Rev 14): https://static.garmin.com/pumac/190-01879-11_14.pdf ; GMA 340: https://www.sarasotaavionics.com/Manuals/GARMIN/GMA340.pdf
- PMA450 manual pp.12-13: https://www.manualsdir.com/manuals/708955/ps-engineering-pma450-installation-manual.html?page=12
- flyonspeed.org technical resources: https://www.flyonspeed.org/technical-resources ; calibration post: https://www.flyonspeed.org/post/calibration-step-1-determining-and-programming-the-aircraft-curves ; June 2023 update: https://www.flyonspeed.org/post/june-2023-update ; dirt-cheap sensor: https://www.flyonspeed.org/post/testing-the-dirt-cheap-aoa-sensor
- EAA 2021 white paper: https://www.flyonspeed.org/_files/ugd/3c039a_bb0fd55c9437442b9a78a4997a70070b.pdf
- RV-4: https://en.wikipedia.org/wiki/Van%27s_Aircraft_RV-4 ; https://vansairforce.net/threads/recommended-v-speeds.125848
- ESP32 ADC noise (original ESP32, SEO-grade): https://zbotic.in/esp32-adc-accuracy-fix-noise-and-non-linearity-issues/
