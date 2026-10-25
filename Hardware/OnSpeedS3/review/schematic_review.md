# OnSpeed ESP32-S3 board: schematic review before JLCPCB assembly

Reviewed on 2026-10-25. Inputs:

- `/home/claude/pcb/design.py`
- `/home/claude/pcb/out/onspeed.xml`, the netlist
- `/home/claude/pcb/out/onspeed_final.kicad_pcb`, read for pad-to-net checks
- `parts.json` and `parts.md`
- `firmware/main/board_pins.h`
- the Gen2 Eagle schematic (`Hardware/Eagle Files/onspeed.sch`), used for the sensor and DB-15 pinouts

The review is ranked **MUST-FIX** (fix before ordering), **SHOULD-FIX** (real risk or degraded behaviour) and **NOTE**.

## Summary

The pin-to-net mapping is correct for every IC whose datasheet I could check. That includes the ESP32-S3 module, LM5164, PCM5102A, PCA9306, TJA1051T/3, SP3232E, LDL1117, USBLC6, USB-C and BAT54S. The firmware pin map matches `design.py` on all 36 assignments.

The real problems are in the input protection, the energy and voltage chain that has to meet "survives 28 V systems and load dump":

- The bulk capacitor is rated below the TVS clamp voltage.
- The TVS itself starts conducting on a sustained surge, which will destroy it.

There are also a few should-fix items:

- The LM5164 ripple injection is too low at the bottom of the input range.
- The microSD shell is left floating because the wrong symbol variant is used.
- An ADC pin is overdriven during surges.

---

## MUST-FIX

### M1. The 47 µF bulk capacitor is rated 50 V. The input clamp reaches 53 V, and the schematic says "63 V".

- **Where:** `design.py:183`. The part is `PART["bulk"]` = RVT1H470M0607, C970679, and its Rating field reads "63V".
- **Evidence:**
  - The LCSC page for C970679 lists "47 µF 50 V, 105 °C".
  - LCSC C10219 (SMBJ33A, RUILON): VBR 36.7–42.2 V, VC 53.3 V at 11.3 A (10/1000 µs).
  - The capacitor sits on VIN, after the ferrite and directly across the TVS clamp. Any surge large enough to make the TVS conduct takes a 50 V aluminium capacitor beyond its rated voltage, and probably beyond its surge rating as well.
  - The BOM also contradicts itself: the Rating field says 63 V but the LCSC number is a 50 V part.
- **Fix:** use a 100 V electrolytic in the same CP_Elec_6.3x7.7 or 8x10 footprint, for example 22–33 µF 100 V (the damping only needs an ESR of about 0.3–1 Ω). It must be at least 63 V if M2 is not done, and 100 V if M2 is done. Correct the Rating field. Everything else on VIN (2 × 2.2 µF and 100 nF) is already rated 100 V, and so is the LM5164.

### M2. The SMBJ33A conducts on a sustained overvoltage and will fail on a load dump or abnormal surge. Choose the TVS for the 100 V buck.

- **Where:** `design.py:181` (`tvs_in`) and the protection note on line 210.
- **Evidence:**
  - VBR(min) is 36.7 V, and the part is rated 600 W only for a 10/1000 µs pulse.
  - The LM5164 is rated 6–100 V (TI SNVSB16, Abs Max table; LCSC C477928). EN is rated to 100 V.
  - An alternator load dump on a 14 V system lasts hundreds of milliseconds and reaches 35–40 V or more.
  - The abnormal surge for 28 V systems in DO-160 Section 16 is about 46–50 V for up to 100 ms. I did not re-verify the exact category figures; check them against the category you intend to claim.
  - In both cases the SMBJ33A goes into breakdown and has to carry amps from a low-impedance bus for tens to hundreds of milliseconds. That is far beyond 600 W at 1 ms.
  - The failure mode is a short. The PTC then trips, so the box is safe but dead.
  - The buck itself would have ridden through the event.
- **Fix:**
  - Replace the part with an **SMBJ58A**: VRWM 58 V, VBR(min) 64.4 V, VC about 93 V, which is below the LM5164's 100 V limit.
  - The TVS then only catches fast spikes, and the converter carries surges up to 58 V.
  - With this change:
    - The bulk capacitor must be 100 V (see M1).
    - The PTC (60 V Vmax) still sees no more than the bus voltage when it trips.
    - The LM5164 on-time at 58 V is 218 ns. Off-time is limited to 250 ns, and the 50 ns minimum on-time is met.
    - VIN_SENSE has to be rescaled (S3).
  - If a 600 W part is too small for the expected spike energy, use an SMCJ58A or an SM6T68A in an SMC package.

---

## SHOULD-FIX

### S1. LM5164 type-3 ripple injection is about 9 mV at minimum VIN, below the datasheet minimum of 12 mV.

- **Where:** RA = 150 k (`design.py:198`), CA = 2.2 nF, CB = 220 pF.
- **Evidence:** In the LM5164 datasheet's type-3 ripple network section (Eq. 24–26):
  - About 20 mV of FB ripple is recommended, with **at least 12 mV at minimum VIN**.
  - The FB ripple is about (VIN − VOUT)·tON / (RA·CA), with tON(µs) = RRON / (2.5·VIN).
  - At the 6.45 V UVLO turn-on (cranking): 1.45 V × 1.96 µs / 330 µs ≈ **8.6 mV**.
  - At 9 V: 17 mV. At 14 V: 25 mV. At 36 V: 33 mV.
  - The checks that pass:
    - CA ≥ 10 / (fSW·(RFB1‖RFB2)) = 1.05 nF, so 2.2 nF is OK.
    - RA·CA must be ≤ 406 µs at 14 V, and 330 µs is OK.
- **Fix:** change RA to **100 k** (C25803). RA·CA = 220 µs still meets Eq. 25. The ripple becomes about 13 mV at 6.45 V, 37 mV at 14 V and 49 mV at 36 V. The output shifts up by about 1–2 % at high VIN, which is acceptable. If you would rather keep 150 k, raise the UVLO turn-on to about 8 V instead.

### S2. The microSD socket uses a symbol with one detect pin, so the socket shell (pad 11) is left floating.

- **Where:** `design.py:394` uses `Micro_SD_Card_Det1`; the footprint is `Connector_Card:microSD_HC_Molex_104031-0811`.
- **Evidence:** In the KiCad 7 library:
  - The footprint's pads 9 and 10 are the two detect-switch contacts, and the four pads numbered 11 are the shell.
  - `Micro_SD_Card_Det1` maps SHIELD to pin 10, so the second detect contact is grounded.
  - `Micro_SD_Card_Det2` (DET_B 9, DET_A 10, SHIELD 11) is the matching symbol.
  - In `onspeed_final.kicad_pcb`, all four J5 pads 11 have no net.
  - The Molex 104031 series page says it has a "card detect switch". The LCSC page says "Card Detection: No". Whether the switch is normally open or normally closed is **not verified**.
- **Fix:**
  - Switch the symbol to `Micro_SD_Card_Det2`.
  - Connect DET_A (pad 10) to GND, SHIELD (pad 11) to GND, and DET_B (pad 9) to SD_DET with the existing 10 k pull-up.
  - Confirm the switch sense from the Molex drawing 1040310811 and set it in the firmware.

### S3. VIN_SENSE overdrives GPIO3 above VDD during surges.

- **Where:** 100 k / 8.2 k divider (`design.py:205`).
- **Evidence:** At 53 V (the present clamp) or 58 V (after M2), the pin sees 4.0–4.4 V. ESP32-S3 inputs are rated to VDD + 0.3 V. The 100 k resistor limits the injected current to about 10 µA, which is probably survivable but out of specification. The divider also skews the ADC readings on the other channels while it is injecting.
- **Fix:** change the top resistor to **150 k** (C22807). 60 V then gives 3.11 V, the full scale is 60 V, and the 100 nF filter stays. Update `VIN_DIVIDER` in `board_pins.h` to `(158.2f / 8.2f)`.

### S4. The input PTC has a thin margin at high cabin temperature and low bus voltage.

- **Where:** BSMD1812-050-60V. Its hold current is 0.5 A at 25 °C (LCSC C883142), and PTC hold current typically falls to about 0.35 A at 70 °C.
- **Evidence:**
  - Worst-case 5 V load: 3.3 V rail about 0.5 A (ESP32 with Wi-Fi on the ground at 355 mA, SD writes, DAC), plus the CAN transceiver in dominant state, giving about 0.6 A.
  - Input current at 6.5–9 V is about 0.35–0.55 A, which can trip a warm PTC while the engine cranks.
- **Fix:** use a 0.75 A or 1.1 A hold 1812 PTC with at least 60 V Vmax (for example BSMD1812-075-60V or -110-60V; check the LCSC numbers). The SS210 (2 A) and the ferrite (2 A) already have the headroom.

### S5. The LED parts do not match their MPN fields, and the LEDs will be very dim.

- **Where:** `led_g` C72043 and `led_b` C72041.
- **Evidence:**
  - The LCSC pages show Everlight **19-217/GHC-YR1S2/3T** (emerald-green InGaN) and **19-217/BHC-ZL1M2RY/3T** (blue), not the KT-0603G and KT-0603B in the MPN fields.
  - Both are InGaN with a forward voltage of about 2.7–3.2 V.
  - The power LED is driven from 3.3 V through 2 k, giving about 0.1–0.25 mA, not the 0.6 mA in the note.
  - The status LED is driven from the GPIO through 1 k, giving about 0.2–0.4 mA.
- **Fix:** either correct the MPN fields and drop the resistors to 330–470 Ω, or use a red or yellow-green AlGaInP LED (for example C2286 KT-0603R, Basic) with the present resistors.

### S6. The LDO's thermal budget is not documented.

- **Evidence:** the LDL1117 drops 1.7 V. At 0.4–0.5 A average (Wi-Fi on the ground, SD writes, 100 mA allowed on DB-15 pin 11), it dissipates 0.7–0.85 W in a SOT-223. At 50–60 °C/W on good copper and 70 °C cabin ambient, the junction reaches about 115–120 °C.
- **Fix:** give the tab at least 2 cm² of copper with vias to the inner planes, and record the power-budget assumption. If in-flight Wi-Fi stays off, this is acceptable.

### S7. DB-15 pin 11 (3.3 V out) now delivers about 2.95 V, and its ESD diode dies if the pin is miswired to the bus.

- **Where:** `design.py:297-298`. The SS14 on pin 11 is added relative to Gen2, which used only the MFR010 PTC.
- **Evidence:**
  - Any pot powered from pin 11 (for example the volume pot) now sits about 10 % below the 3.3 V pull-up in the VOL_ADC network. Readings shift, and their zero depends on the diode's temperature.
  - On a 28 V aircraft, a pin 11 shorted to the bus puts 28 V across the 15 V PESD15VL1BA (D13), which then shorts. The SS14 is rated 40 V reverse, so it survives.
- **Fix:**
  - Rescale the volume curve in firmware, or take the volume reference from 3V3_OUT_F.
  - Put the 24 V PESD24VL1BA on X_3V3, as on X_FLAP. That covers 14 V aircraft. For 28 V aircraft, accept the risk and document it.

### S8. The JST-GH I2C expansion connector has no ESD protection and puts 3.3 V where Pixhawk has 5 V.

- **Where:** J3 (`design.py:323`).
- **Evidence:**
  - Pixhawk's connector standard puts 5 V on pin 1, and `parts.md` says the same. The board supplies +3V3_S on pin 1.
  - The bus it exposes (bus B) also carries the IMU and the P45 sensor's translator, and it leaves the board with no protection at all.
- **Fix:**
  - Add a 2-line 5 V ESD array on SCL and SDA (for example the USBLC6-2SC6 already in the BOM) if the cable can leave the enclosure.
  - Change the silkscreen and Note to say "3.3 V" clearly.

---

## NOTE

1. **USB-only power back-feeds VIN through the LM5164 high-side body diode.** The path is +5V → L1 → SW → body diode → VIN, which holds VIN at about 4.2 V.
   - EN is then at 0.98 V, below the 1.1 V shutdown threshold (datasheet EN table), so nothing breaks.
   - VIN_SENSE will read about 4 V on the bench; the firmware should treat that as "USB only".
   - The 47 µF and 4.4 µF on VIN also add to the USB hot-plug inrush. The total on VBUS is about 70 µF plus, through the body diode, about 50 µF, against USB 2.0's 10 µF limit. Laptops tolerate this, but a strict hub may not.
2. **PCA9306 VREF2 filter.** TI's PCA9306 datasheet (application section) specifies a 200 kΩ resistor to the high-side supply with EN tied to VREF2. The design does that correctly. TI recommends a **100 pF** filter capacitor; the design has 100 nF. With 200 k, that gives about a 20 ms enable time constant, which is harmless because the ESP32 boots later, but you can change it to 100 pF.
3. **The MS4525DO runs from the USB-only rail.** +5V_S is then about 4.6 V, which is outside the DS5 calibration. That is acceptable for the bench, but don't calibrate on USB. The TJA1051 VCC (minimum 4.5 V) is marginal in the same case.
4. **BAT54S clamps back-power +3V3.** When the board is off and BOOM (5 V via 1 k), FLAP (14 V via 22 k) or VOL lines are live, current flows into +3V3 through the clamps: about 1.4 mA from BOOM and 0.5 mA from FLAP. This is not damaging, but the ESP32 may half-power. Accept it or add a 3.3 V Zener load.
5. **FLAP uses the 24 V ESD diode.** That is fine on 14 V aircraft. On 28 V aircraft with the flap sensor on the bus, D9 conducts. The 22 k / 5.6 k divider also gives 5.7 V into the clamp, which is OK at 1.1 mA. Document "14 V flap sensor only", or use a 33 V part and a different divider on 28 V aircraft.
6. **CAN DE-9 grounding.** CiA 303-1 assigns pin 3 CAN_GND, pin 5 CAN_SHLD and pin 6 GND (optional), and pin 8 is reserved. The design grounds pins 3, 5 and 8 and leaves 6 open, and its note calls pin 8 "shield". Recommendation: ground pins 3, 5 and 6 and leave pin 8 unconnected.
7. **UART0 RX** (module pin 36) goes only to a test point. The internal pull-up handles it; add an external 10 k if the ROM log shows noise.
8. **The firmware header comment is out of date.** `board_pins.h:2` says strapping pin 3 is avoided, but `ADC_CH_VIN` uses GPIO3. That is legitimate, because GPIO3 only acts as a strap once the JTAG-select eFuse is burned. Fix the comment.
9. **`parts.md` is out of date.**
   - It still lists the SMBJ24A as the input TVS, which would be wrong for 28 V systems because its stand-off is below 28.8 V.
   - It lists the TF-01A, XRRH105R and TS-1187A parts that were later swapped.
   - Design.py's SMBJ33A and FNR6045S330MT (33 µH, Isat 1.8 A, LCSC C168083) are the current choices. The inductor's Isat is just above the LM5164's 1.75 A maximum peak current limit, which is acceptable.
10. **PCM5102A.** The pins and control straps are correct: FMT, FLT, DEMP and SCK are low, and XSMT is driven from GPIO21 with a 10 k pull-down.
    - The firmware runs 44.1 kHz, 16-bit stereo, which is 32·fs BCK and supported by the PLL table.
    - The 0.1 µF on LDOO is confirmed by TI.
    - The 2.2 µF values on CAPP–CAPM and VNEG come from the datasheet figure. The extracted text was garbled, so I could not confirm the value in text.
    - The output pad delivers about 0.39 Vrms into 600 Ω from about 420 Ω, and the DAC sees about 1.3 kΩ, above its 1 kΩ minimum.

---

## What was verified

### Pinouts against the manufacturer or Gen2 source: all correct

| IC | Source | Result |
|---|---|---|
| ESP32-S3-WROOM-1 pins 1–41 | Espressif WROOM-1 datasheet pin table | Matches the netlist exactly |
| LM5164 DDA | TI datasheet pin table | 1 GND, 2 VIN, 3 EN/UVLO, 4 RON, 5 FB, 6 PGOOD, 7 BST, 8 SW, EP GND |
| PCM5102A PW | TI datasheet | All 20 pins match |
| PCA9306 DCU | TI datasheet | 1 GND, 2 VREF1, 3 SCL1, 4 SDA1, 5 SDA2, 6 SCL2, 7 VREF2, 8 EN |
| TJA1051T/3, SP3232E, LDL1117 SOT-223, USBLC6-2SC6, USB-C 16P, BAT54S | Standard pinouts | Match |
| MS4525DO | Gen2 Eagle library | 1 GND, 2 VCC, 3 SDA, 4 SCL, 5 INT/SS |
| Honeywell SSC SIP | Gen2 Eagle library | 1 GND, 2 VCC, 3 SDA, 4 SCL |
| DB-15 pins 1–14 | Gen2 schematic | Same functions as Gen2 |

Notes on the table:

- **ESP32-S3 module:** IO35–37 are free on the N8, which has no PSRAM. The straps are correct:
  - IO0 has the 10 k pull-up and the BOOT button.
  - IO45 and IO46 are open, using their internal pull-downs (3.3 V VDD_SPI, normal boot).
  - IO3 is a strap only if the eFuse is set.
  - EN has 10 k / 1 µF, as Espressif recommends.
- **LM5164 values:**
  - VFB = 1.2 V, so the output is 5.00 V.
  - fSW = 5 × 2500 / 31.6 = 396 kHz.
  - CBST is 2.2 nF, 50 V X7R. The datasheet allows only 1.5–2.5 nF, and C1604 is confirmed as 2.2 nF ±10 % X7R 50 V.
  - UVLO turns on at 1.5 × 4.3 = 6.45 V and off at 1.4 × 4.3 = 6.02 V.
  - EN is rated to 100 V, and PGOOD to 14 V with its 3.3 V pull-up.
- **Sensor addresses:**
  - The SSC "7" output code is I2C address 0x78 (Honeywell TruStability ordering guide).
  - The MS4525DO "I" interface code is 0x28.
  - The ISM330DHCX has SA0 to GND (0x6A) and CS high. SDx and SCx are grounded, as `parts.md` recommends from the ST datasheet. I could not fetch the ST PDF; the site returned an error.
- **DB-15:** pin 15 is now AUDIO_GND through a 0 Ω link to ground (Gen2 tied it to GND); pin 11 is discussed in S7.

### Nets

There are no unintended single-node nets. The only ones are deliberate no-connects:

- IO45 and IO46
- the SBU pins
- MS4525 pins 5–8
- IMU INT2, OCS_Aux and SDO_Aux
- SP3232 T2OUT (T2IN is tied to GND)
- DE-9 pins 1, 4, 6 and 9
- U0RXD and U0TXD, which go only to test points

No two rails are shorted. In the PCB, every IC pad carries the net the netlist gives it; the exception is the J5 shell (S2).

### Firmware

`board_pins.h` matches `design.py` on:

- the I2C buses: A on 8/9, B on 17/18
- the ADCs: GPIO1 on channel 0, GPIO2 on channel 1, GPIO3 on channel 2
- I2S on 12/13/14, and XSMT on 21
- the UARTs: 10, 11, 15, 16 and 7
- CAN on 47/4
- the panel switch and LED on 5/6
- the SD card on 39–42
- IMU INT1 on 35, SD detect on 36, CAN S on 37, the status LED on 38 and PGOOD on 48

### LCSC spot checks: live LCSC pages, 2026-10-25

These match their stated value and package:

| LCSC number | Part |
|---|---|
| C2913198 | WROOM-1-N8 |
| C477928 | LM5164DDAR |
| C435835 | LDL1117S33R |
| C33196 | PCA9306DCUR |
| C9378 | SP3232EEN-L/TR |
| C38695 | TJA1051T/3/1J |
| C2655101 | ISM330DHCXTR |
| C107671 | PCM5102APWR |
| C10219 | SMBJ33A |
| C14996 | SS210, 100 V 2 A |
| C2480 | SS14, 40 V |
| C168083 | FNR6045S330MT |
| C883142 and C883088 | The two PTCs |
| C2985482 | PESD15VL1BA |
| C314263 | PESD24VL1BA-N |
| C2687131 | PESD2CAN, 24 V |
| C19726 | BAT54SLT1G |
| C85840 | BLM21PG221SN1D |
| C221895 | PTS810 |
| C75750 and C77835 | CONNFLY DE-9 male and DA-15 female |
| C585350 | Molex 104031-0811 |
| C1604 | 2.2 nF X7R |
| C1805 | 220 pF C0G 0805 |
| C15849 | 1 µF, which is actually rated 50 V X5R, not the "16 V X7R" in the table |
| C23630 | 2.2 µF 16 V X5R |
| C28233 | 100 nF 100 V |
| C53987 | 4.7 nF |
| Resistors | C25967 (31.6 k), C22808 (150 Ω), C22787 (120 Ω), C4184 (20 k), C25981 (8.2 k), C23137 (330 k), C22807 (150 k), C25811 (200 k), C23179 (470 Ω), C23204 (560 Ω), C23140 (33 Ω) |

These do not match:

- C970679 is rated 50 V, not 63 V (M1).
- C72043 and C72041 are Everlight parts, not the MPNs given (S5).

### Not verified

- The ISM330DHCX datasheet (the ST site refused the fetch) and the footprint's pin-1 position.
- The TE MS4525DO datasheet (TE's site blocked the fetch). I relied on the Gen2 footprint and library.
- The Honeywell SIP pinout in the Honeywell document. The Gen2 library was used.
- The sense of the Molex detect switch.
- The exact DO-160 Section 16 surge figures.
- The LCSC pages for C165948, C7519, C189895, C85833, C45783, C15850 and C1779, which I did not reopen; they were taken from `parts.md`.
