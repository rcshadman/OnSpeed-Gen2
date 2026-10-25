# OnSpeed ESP32-S3 port: hardware verification

Checked 2026-10-25 against the Gen2 repo (`flyonspeed/onspeed-gen2`: Eagle schematic netlist, Assembly Procedure, `OnSpeedTeensy_AHRS/*.ino`), the current port firmware (`sim/onspeed-s3/main/main.cpp`, `sensors.cpp`, `sdkconfig`), Espressif and Seeed documentation, the TI PCM5102A datasheet, and the TE/Honeywell sensor documents. Machine-readable copies are in `pinmap.json` and `bom.json` next to this file.

## Summary

- **The XIAO ESP32-S3 does not have enough pins.** It has 11 edge GPIOs, plus 4 JTAG solder pads. Gen2 feature parity needs 21 signals, and even the reduced set listed in the brief needs 15. Two of its pins are UART0 and carry the ROM boot log; one is a strapping pin. **Switch the prototype to the Espressif ESP32-S3-DevKitC-1** (N8R8, N8R2 or N8). It uses the same ESP32-S3-WROOM-1 module as the planned product PCB, so the pin map below carries straight over to the PCB.
- **The current firmware wiring has three bugs that would stop it working on hardware:**
  1. I2S is on GPIO43/44 while the app console is also on UART0 (`CONFIG_ESP_CONSOLE_UART`, 921600 baud).
  2. The I2C buses use only the internal pull-ups.
  3. The I2S clocking (22.05 kHz, 16-bit, BCK = 705.6 kHz) is outside the PCM5102A's internal-PLL table. Expect no audio, or unstable audio.
- **The Gen2 AOA sensors are not Honeywell parts.** They are **TE Connectivity 4525DO-DS5AI001DP** (MS4525DO, ±1 psid, I2C 0x28, calibrated for 5 V). Gen2 runs them at 3.3 V. The static sensor is a Honeywell SSCSRNN1.6BA7A3 (0x78), as expected.
- **BOM: $485.75 for a complete new box. With the Gen2 sensors, IMU, enclosure, fittings, display and harness parts reused, $72.31 of new parts is needed** (details in the BOM table below).

## 1. Final pin map: ESP32-S3-DevKitC-1 (WROOM-1)

| Signal | GPIO | DevKitC pin | Peripheral | Notes |
|---|---|---|---|---|
| Flap position (DB-15 pin 7) | 1 | J3 GPIO1 | ADC1_CH0 | Same input circuit as Gen2: 22k in series, 5.6k to GND, 100 nF. Scales up to about 14 V. 12 dB attenuation, curve-fitting calibration, average 32–64 samples. No internal pull-up. |
| Volume (DB-15 pin 10) | 2 | J3 GPIO2 | ADC1_CH1 | Same as Gen2: 22k in series, 22k to 3.3 V, 5.6k to GND, 100 nF. |
| OAT / spare analog | 4 | J1 GPIO4 | ADC1_CH3 | Spare. |
| Panel switch (DB-15 pin 3) | 5 | J1 GPIO5 | GPIO in | 150R in series, 20k pull-up (Gen2 R14/R13). |
| AOA LED (DB-15 pin 5) | 6 | J1 GPIO6 | LEDC PWM | 150R in series (Gen2 R5). |
| Boom TTL RX (DB-15 pin 13) | 7 | J1 GPIO7 | UART0 RX (via GPIO matrix) | 150R in series. Accepts 3.3 V logic only. |
| I2C bus A SDA | 8 | J1 GPIO8 | I2C0 | Pfwd MS4525DO 0x28 and static SSC 0x78. External 4.7k pull-up. 400 kHz. |
| I2C bus A SCL | 9 | J1 GPIO9 | I2C0 | External 4.7k pull-up. |
| Display TX (DB-15 pin 9) | 10 | J1 GPIO10 | UART2 TX | 115200 baud, 3.3 V TTL to the M5Stack, 150R in series. |
| EFIS TX (DB-15 pin 12) | 11 | J1 GPIO11 | UART1 TX | Through the MAX3232 T1. |
| I2S BCLK | 12 | J1 GPIO12 | I2S0 | To PCM5102A BCK. |
| I2S LRCK | 13 | J1 GPIO13 | I2S0 | To PCM5102A LRCK. |
| I2S DOUT | 14 | J1 GPIO14 | I2S0 | To PCM5102A DIN. |
| EFIS RX (DB-15 pin 2) | 15 | J1 GPIO15 | UART1 RX | Through the MAX3232 R1 (for example from a Garmin G5). |
| GPS/VN-300 RX (DB-15 pin 4) | 16 | J1 GPIO16 | UART2 RX | Through the MAX3232 R2. |
| I2C bus B SDA | 17 | J1 GPIO17 | I2C1 | P45 MS4525DO 0x28 and ISM330DHCX 0x6A (optional ADS1115 0x48). External 4.7k pull-up. |
| I2C bus B SCL | 18 | J1 GPIO18 | I2C1 | External 4.7k pull-up. |
| DAC XSMT (mute) | 21 | J3 GPIO21 | GPIO out | 10k pull-down, so the DAC is muted by default. No RC filter, because the edge must be under 20 ns. |
| SD CLK | 39 | J3 GPIO39 | SDMMC 1-bit | About 20 MHz. JTAG uses USB, so the MTCK pin is free. |
| SD CMD | 40 | J3 GPIO40 | SDMMC | 10k pull-up. |
| SD D0 | 41 | J3 GPIO41 | SDMMC | 10k pull-up. |
| SD D3 | 42 | J3 GPIO42 | SDMMC / GPIO | 10k pull-up (selects SD mode). Pull D1 and D2 up through 10k as well. |
| Console / flashing / JTAG | 19/20 | USB connector | USB-Serial-JTAG | Never repurpose these pins. |
| *Not used:* 0, 3, 45, 46 (strapping pins), 35–37 (octal PSRAM), 38/48 (on-board RGB LED), 43/44 (on-board USB-UART bridge; the ROM boot log appears here harmlessly), 47 (spare) | | | | |

Peripheral budget: 2 of 2 I2C controllers, 1 of 2 I2S, 3 of 3 UARTs (UART0 boom RX; UART1 EFIS RX/TX; UART2 display TX and GPS RX), SDMMC 1-bit, and 2 ADC1 channels plus 1 spare. That is 22 GPIOs used out of roughly 33 that the DevKitC breaks out. Pins left spare: 4, 47 and 48 (48 only on a v1.1 board).

Firmware changes this pin map requires:
- Set `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` and `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`.
- Pass `i2sBegin(12, 13, 14)` and `hw.begin(8, 9, 17, 18, ADC_CHANNEL_0)`.
- Add the IMU at 0x6A on bus B.
- Set `enable_internal_pullup = false` and fit the external 4.7k resistors.

On the product PCB, use an **ESP32-S3-WROOM-1-N8** (or -N8R2). Do not use a "V" variant such as N8R8V or N16R16V: on those, GPIO47/48 run at 1.8 V. Add a 10k pull-up and 1 µF RC on EN, and break out GPIO0 to a button.

### Why not the XIAO, and what to do if you must use it

| Requirement | Signals | XIAO ESP32-S3 |
|---|---|---|
| 2 I2C buses (both AOA sensors are 0x28) | 4 | D4/D5, D9/D10 |
| I2S plus mute | 3 + 1 | D8 plus D6/D7 (UART0, see below) |
| EFIS RX/TX, GPS RX, boom RX, display TX | 5 (3 minimum) | no pins left |
| SD, 1-bit SDMMC | 3 (SPI needs 4) | none |
| Flap and volume ADC | 2 | A0, A1 |
| Switch and LED | 2 | D2 (GPIO3 is a strapping pin), D3 |
| **Total** | **21 for parity, 15 for the reduced brief** | **11 edge GPIOs, plus 4 JTAG pads that must be soldered** |

The XIAO ESP32-S3 Plus ($7.99, in stock) has 9 more GPIOs, but they sit on SMD pads meant for board-to-board soldering, not headers. A Plus on a custom carrier could work. For a solderless prototype it is the wrong tool, and it is still tight (20 GPIOs).

## 2. Pin audit findings

| # | Finding | Applies to | Action |
|---|---|---|---|
| P1 | **Strapping pins.** GPIO0 selects boot mode (pull-up). GPIO3 selects the JTAG source. GPIO45 sets VDD_SPI voltage: pulled high at reset, it selects 1.8 V flash and a 3.3 V-flash module **will not boot**. GPIO46 controls boot mode and ROM printing. XIAO D2 is GPIO3. | both | None of the four is used in the DevKitC map. |
| P2 | **USB D-/D+ are GPIO19/20.** If the app reconfigures them, the board "disappears from the system" and needs GPIO0 held low to recover. | both | Reserved. Console runs on USB-Serial-JTAG. |
| P3 | **Octal PSRAM (N8R8) uses GPIO35–37**, and GPIO33/34 are not broken out. The XIAO's PSRAM is also octal, but those pins are not on its edge anyway. "V" modules run GPIO47/48 at 1.8 V. | both | Avoided. A product PCB with an -N8 module frees GPIO35–37. |
| P4 | **ADC2 (GPIO11–20) is shared with Wi-Fi.** The driver arbitrates access, so reads can time out while Wi-Fi is active. ADC1 is GPIO1–10. Errata ADC-183: the ADC2 DMA controller does not work at all. The XIAO exposes ADC1 on A0–A5 and A8–A10 (GPIO1–9); GPIO41/42 have no ADC. | both | Analog inputs use only GPIO1, 2 and 4 (ADC1). GPIO11–18 carry digital signals only. |
| P5 | **GPIO43/44 carry the ROM boot log** at 115200 baud (U0TXD/U0RXD). In the XIAO firmware they are I2S LRCK and DIN, and the sdkconfig also sends the app console to UART0 at 921600 baud. Three problems follow: (a) any `printf` before or around `i2sBegin` drives DIN; (b) UART0 RX on GPIO44 would read LRCK as garbage console input; (c) the ROM log toggles DIN at every reset. Effect on the DAC: in (c) the PCM5102A has no BCK, so it stays in standby and the toggling is mostly harmless. (a) and (b) are real bugs. The ROM log can be silenced **permanently** with `CONFIG_BOOT_ROM_LOG_ALWAYS_OFF`, which burns the eFuse once and cannot be undone. A GPIO46-controlled option also exists, but GPIO46 is not on the XIAO. | XIAO | The DevKitC map leaves 43/44 on the USB-UART bridge only. If you stay on the XIAO: move the console to USB-Serial-JTAG, and optionally burn ROM_LOG_ALWAYS_OFF on that unit. |
| P6 | **XIAO Sense conflicts.** The expansion-board SD card uses CS GPIO21, SCK GPIO7, MISO GPIO8 and MOSI GPIO9. These clash with the firmware's BCLK (D8 = GPIO7) and I2C bus B (D9/D10 = GPIO8/9). The PDM mic uses GPIO41/42. The camera uses GPIO10–18, 38–40, 47 and 48. The user LED is GPIO21. | XIAO Sense | Do not use a Sense board, or cut J3. |
| P7 | **Power-up glitches.** GPIO1–18 pulse low for about 60 µs at power-up, and GPIO18–20 glitch high. A UART TX pulled low that briefly reads as a break, so the EFIS and display receivers may log one bad byte. The LED is unaffected. | both | Accept. The mute line sits on GPIO21, which does not glitch. Do not drive MOSFET gates from GPIO1–18 without pull resistors. |
| P8 | **I2C speed.** ESP-IDF says the master SCL "should not be larger than 400 kHz". Gen2 ran its two pressure buses at a nominal **3 MHz** (`i2c.ino`). The Honeywell I2C note specifies 100–400 kHz. Gen2 timing therefore cannot be copied. At 400 kHz a 2-byte pressure read takes about 70 µs, and an IMU sample (2 × 6 bytes at 208 Hz) uses under 10% of bus B. That is enough. | both | 400 kHz on both buses. |
| P9 | **Pull-ups.** `sensors.cpp` uses `enable_internal_pullup = true`, about 45 kΩ. ESP-IDF says this is "not strong enough to pullup buses under high-speed frequency" and recommends 2–5 kΩ. Gen2 fits 4.7k on every line (R1–R4, R6, R7). The Adafruit ISM330 board adds its own 10k. | both | External 4.7k on each SDA/SCL line, connected to the same 3.3 V rail as the sensors. |
| P10 | **5 V tolerance.** No ESP32-S3 pin is 5 V tolerant. Gen2's Teensy 3.6 was not either, so existing boom and display wiring should already be 3.3 V. Confirm this on each aircraft before connecting. | both | Keep the Gen2 150R series resistors. Add a BAT54S clamp to 3.3 V on the boom RX line. |
| P11 | **I2S clock and PCM5102A PLL.** With SCK grounded, the PCM5102A builds its master clock from BCK. Its table lists only 32fs and 64fs at 16 kHz (64fs only), 32, 44.1 and 48 kHz and up, with BCK of at least 1.024 MHz. The firmware runs **22.05 kHz with 16-bit stereo, so BCK = 32 × 22050 = 705.6 kHz**. That is not a listed mode. | firmware | Run I2S at **44.1 kHz** (32fs = 1.411 MHz, or 64fs with 32-bit slots). This also matches the Gen2 voice clips, which are u-law at 44.1 kHz, so they need no resampling. |
| P12 | **Flap/volume ADC scale.** Gen2 calls `analogReadResolution(13)`, so the stored `flapPotPositions` and volume limits are on a 0–8191 scale. The port returns `raw >> 2`, a 10-bit value. | firmware | Return `raw << 1` (12-bit to 13-bit) so existing configs keep working, then repeat the flap calibration in the aircraft. Gen2 also used `INPUT_PULLUP` on the flap pin; do not copy that. |
| P13 | **Sensor status bits are ignored.** Gen2 and the port both mask off the top two bits (`& 63`). For Honeywell parts: 00 = OK, 01 = command mode, 10 = **stale** (data already read), 11 = diagnostic fault. MS4525DO parts use the same 2-bit status. | firmware | Drop samples with status 10 or 11, keep the last good value, and count errors for the log. A pressure value of −1 on timeout should also be rejected rather than filtered. |
| P14 | **Address plan.** Bus A: 0x28 (Pfwd) and 0x78 (static). Bus B: 0x28 (P45), 0x6A (ISM330; the Gen2 firmware `_i2CAddressISM330`), and optionally 0x48 (ADS1115). There are no conflicts. MS4525DO parts also come in 0x36 (J) and 0x46 (K) versions, so new builds can put both AOA sensors on a single bus. | both | As mapped. |

## 3. Noise and reliability findings

| # | Finding | Source | Mitigation |
|---|---|---|---|
| N1 | The ESP32-S3 ADC "is sensitive to noise, leading to large discrepancies in ADC readings". Its reference voltage varies from 1000 to 1200 mV between chips. Only the curve-fitting calibration scheme exists on the S3. | https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/adc_calibration.html | Use `adc_cali_create_scheme_curve_fitting` and work in mV. Fit 100 nF at the pin (Gen2 already has C12/C13). Average 32–64 samples per read, and add a median filter. Flap position only needs a few positions, and the ±1–2% residual non-linearity is far below the gap between them. If bench tests show trouble, add an ADS1115 (BOM option). |
| N2 | ADC2 is used by Wi-Fi. Reads are arbitrated and can fail with ESP_ERR_TIMEOUT. `adc_oneshot_read` must not be called from an ISR. Spikes appear on analog inputs while Wi-Fi transmits; this is widely reported for ESP32 analog microphones. | https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/adc_oneshot.html ; https://github.com/atuline/WLED/wiki/Noise-and-Spikes | Use ADC1 only. Read the ADC from the 50 Hz sensor task, not from an ISR. The median filter removes Wi-Fi spikes. Lower Wi-Fi TX power (`esp_wifi_set_max_tx_power`, for example to 11–13 dBm). |
| N3 | The I2C master can hang or watchdog in `i2c_new_master_bus` or `add_device` when the bus is bad (missing pull-ups, or an unresponsive device). The bug was reported on IDF v5.3-dev. ESP-IDF provides `i2c_master_bus_reset()` but documents no recovery procedure. `xfer_timeout_ms = -1` blocks forever. | https://github.com/espressif/esp-idf/issues/12929 ; https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html | Pin a stable IDF release (v5.4 or v5.5). Fit external pull-ups. Always use a finite timeout (2 ms, as the current code does). After N consecutive failures, call `i2c_master_bus_reset`; if that fails, delete the bus, toggle SCL 9 times as a GPIO, send a STOP, and recreate the bus. Feed the task watchdog from the sensor task. Gen2's `I2C_COMMUNICATION_TIMEOUT` of 2000 µs is the same idea. |
| N4 | None of the sensors stretch the clock (MS4525DO, SSC and ISM330 are simple slaves). ESP-IDF notes that stretching can last up to 12 ms and sets it with `scl_wait_us`. | ESP-IDF I2C docs (above) | Leave the default. Use `glitch_ignore_cnt = 7`, as the current code does. |
| N5 | Honeywell status 10 means **stale data**, returned when the master polls faster than the sensor updates. SCL must be 100–400 kHz. Pull-ups must be 1–50 kΩ, and bus capacitance under 400 pF. | Honeywell TN 008201-3-EN: https://forums.parallax.com/uploads/editor/79/9ekv6t69d0fx.pdf | See P13. Keep the internal sensor tubes and I2C wires short (under 15 cm) and inside the box. |
| N6 | **MS4525DO supply voltage.** The ordering code digit 3 means 3.3 V and 5 means 5.0 V. Accuracy (total error band) is "valid only at the calibrated supply voltage". Gen2 uses the 5 V-calibrated DS**5** part on its 3.3 V rail (netlist: PS1/PS2 VCC on the Teensy 3.3 V net). The supply range is 2.7–5.5 V, so the part works, and the boot-time bias zeroing plus in-flight AOA calibration absorb most of the error. | https://www.mouser.com/datasheet/2/418/MS4525DO-710170.pdf | Reused Gen2 sensors: acceptable; repeat the AOA calibration in the new box. New sensors: order 4525DO-DS**3**AI001DP (0x28), and optionally DS3AJ001DP (0x36). |
| N7 | **PCM5102A muting.** XSMT low gives a soft ramp of 104 samples. On clock error the device hard-mutes. If clocks halt for over 1 s it powers down. Pops occur when power is lost faster than the mute sequence (150 samples + 0.2 ms). Stopping clocks 3 ms before power-down is the recommended alternative. The PLL starts after 16 LRCK periods with SCK low. | TI PCM5102A datasheet: https://kyohritsu.com/eclib/OTHER/DATASHEET/TI/pcm5102a.pdf | Pull XSMT down with 10k so it is muted at power-up. Start I2S sending zeros, wait 50 ms, then raise XSMT. Before any `i2s_channel_disable`, reboot or OTA, lower XSMT and wait at least 10 ms. On sudden power loss, the 47 µF input bulk capacitor plus the regulator hold-up give the clock-error mute time to act. Bench-test pull-the-plug pops into the real audio panel. |
| N8 | **PCM5102A board settings.** SCK goes to GND (internal PLL). FLT low selects normal FIR latency (about 20 samples); high selects low-latency IIR. DEMP low means off. FMT low means I2S. Recommended supply filtering is 100 nF plus 100 µF, and filtered or separate supplies for DAC and amplifier. | https://www.makerguides.com/playing-audio-with-esp32-and-pcm5102/ ; https://www.adafruit.com/product/6250 | The Adafruit #6250 already defaults FLT, DEMP and FMT low and runs the PLL without MCLK. Power its VIN from the regulated 5 V; its own LDO makes the 3.3 V, which isolates it from ESP32 Wi-Fi current spikes. Add 100 µF at the DAC VIN. |
| N9 | **Audio level.** The PCM5102A gives 2.1 Vrms full scale, ground-centred, into at least 1 kΩ. That is far hotter than the Teensy 3.6 DAC (at most 3.3 Vpp, about 1.2 Vrms). Aircraft audio panel music and aux inputs expect a much lower level. | TI datasheet (above) | Fit a resistive pad on each channel: 10k in series and 3.3k to audio GND, about −12 dB, giving 0.52 Vrms maximum. Also limit full scale in firmware. Keep the Gen2 10 µF coupling capacitors (C6/C7) for DC blocking and ESD robustness. Return audio GND on DB-15 pin 15 to the panel's audio low only. |
| N10 | **Ground loops.** In Gen2, DB-15 pins 14 (GND) and 15 (audio GND) share one board ground net. Alternator whine through the audio panel is a common experimental-aircraft complaint. *(Engineering practice; I could not open a forum source to cite.)* | Gen2 netlist (this repo) | Connect audio low at the audio panel only, never to airframe ground at the OnSpeed end. Use shielded twisted pair with the shield grounded at one end. If whine appears, fit a 600:600 Ω audio isolation transformer on each channel. |
| N11 | **Wi-Fi and real-time timing.** Flash writes (NVS, OTA, config save) disable the cache. While the cache is off, I2S and I2C interrupts are deferred unless `CONFIG_I2S_ISR_IRAM_SAFE` and `CONFIG_I2C_ISR_IRAM_SAFE` are set. Each DMA buffer is limited to 4092 bytes. | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2s.html | Run the audio task on core 1 at high priority and leave Wi-Fi on core 0 (the default). Set `dma_desc_num = 6` and `dma_frame_num = 240` (about 33 ms of buffer at 44.1 kHz). Enable both IRAM-safe options. Do not save config or run OTA in flight. Write the SD card from its own low-priority task. |
| N12 | **USB-Serial-JTAG console.** When its small buffer fills with no host attached, the chip does "a one-time wait of 50 ms". Light or deep sleep drops the USB connection. Arduino HWCDC has a history of `Serial.begin()` hangs and "port closed" problems (fixed in 2.0.17 / 3.x). | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html ; https://github.com/espressif/arduino-esp32/issues/9580 | Never `printf` from the sensor or audio tasks; queue log lines to a low-priority logger task. Do not use sleep modes. If you use Arduino, use core 3.x or later. In flight the console is unconnected, which is exactly the case where the 50 ms wait happens. |
| N13 | **Silencing the ROM boot log** (only needed if I2S stays on GPIO43): `CONFIG_BOOT_ROM_LOG_ALWAYS_OFF` burns eFuse bits once at app boot. The GPIO-controlled option uses a chip-dependent strap, GPIO46 on the S3. | https://codex.ac6.fr/Zephyr/xref/hal_espressif-latest/components/esp_rom/Kconfig.projbuild | Not needed with the DevKitC map. |
| N14 | **Power input.** The Pololu D36V28F5 accepts 5.3–50 V and has reverse protection to 40 V. The D36V6F5 has none, and Pololu warns that LC spikes at hot-plug can exceed 50 V, recommending at least 33 µF at VIN. The Gen2 OKI-78SR-5/1.5 is rated 7–36 V in and drops out during engine cranking, when the bus can fall below 7 V. | https://www.pololu.com/product/3782 ; https://www.pololu.com/product/3792 | Input chain: aircraft breaker (1–2 A), then MF-R050 polyfuse, then 1N5819 series diode (optional with the D36V28F5), then P6KE22A TVS to ground, then ferrite plus 47 µF/63 V, then D36V28F5, giving 5 V to the DevKitC 5V pin, DAC VIN and MAX3232. The DevKitC's on-board 3.3 V LDO feeds the sensors through MF-R010. If reverse polarity is applied, the TVS conducts and the polyfuse trips. The 18.8 V TVS standoff passes normal 14 V bus variation and clamps load-dump spikes below the 50 V regulator rating. This is DO-160 Section 16/17 *in spirit only*; it is not tested. |
| N15 | **Brownout and Wi-Fi current.** ESP32-S3 Wi-Fi transmit peaks are a few hundred mA. A weak 5 V supply or long thin wiring triggers the brownout detector. | https://www.universal-solder.ca/troubleshooting-esp32-brownout/ (search result, not opened) | Size the regulator at 1 A or more (the D36V28F5 gives 3.2 A). Keep the brownout detector enabled (`CONFIG_ESP_BROWNOUT_DET`). Fit 47–100 µF near the module's 3V3. Log `esp_reset_reason()` to SD at every boot. |
| N16 | **ESP32-S3 errata.** The summary has no I2C, I2S or UART items. Relevant ones: ADC-183 (the ADC2 DMA controller does not work) and USBOTG-4289 (USB-OTG download is unavailable; USB-Serial-JTAG download works). | https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32s3/02-errata-summary/index.html | Already avoided. |
| N17 | **FlyOnSpeed team plans** (Feb 2025): the team says the Teensy 3.6 is no longer available. Its V4B test board uses the ESP32, drops into the RV-4 enclosure, and keeps the DSUB-15, back plate and quick releases. The code is a complete rewrite in the Arduino IDE, including "off-board sound processing". The latest post (Jul 2025, "Applied AOA") has no hardware news. V4 hardware and code are still not public. | https://www.flyonspeed.org/post/february-2025-program-update ; https://www.flyonspeed.org/blog | Keep the DB-15 pinout identical, as this map does. Email team@flyonspeed.org before laying out a PCB. |
| N18 | **Longevity.** Espressif commits to the ESP32-S3 for 12 years from 2021. The DevKitC-1 and WROOM-1 are Espressif's own parts, sold through Mouser and Digi-Key. | https://www.espressif.com/en/products/longevity-commitment | n/a |

Not verifiable: esp32.com forum threads (bot wall, Firecrawl out of credits), Reddit, and VAF/EAA forums. Search results pointed to esp32.com threads on I2C recovery (viewtopic.php?p=145188), suppressing S3 boot messages (p=144837) and I2S BCK problems (p=61424). I could not read them, so no finding above depends on them.

## 4. Bill of materials

"R" means the part can be reused from an existing Gen2 box. Prices are USD as listed on 2026-10-25. Prices marked approx could not be fetched. Distributor sites block automated fetches, so Mouser prices come from findchips listings and Adafruit, SparkFun, Pololu and RobotShop prices come from their own product pages.

| Part | Role | Qty | Unit USD | Ext USD | Vendor / link | Reuse | Notes |
|---|---|---|---|---|---|---|---|
| Espressif ESP32-S3-DevKitC-1-N8R8 | Controller (ESP32-S3-WROOM-1, 8 MB flash, 8 MB octal PSRAM) | 1 | 15.00 | 15.00 | [Mouser](https://www.mouser.com/ProductDetail/356-EP32S3DVKTC1N8R8) |  | Mouser $15.00 per findchips (stock fluctuates: 0 to 1677 shown on different listings). Alternates: Digi-Key $15.00, Adafruit #5336 $19.95 (out of stock 2026-10-25). -N8 or -N8R2 also fine; the pin map avoids GPIO33-37. |
| Adafruit ISM330DHCX 6-DoF IMU (#4502) | IMU / AHRS, I2C 0x6A | 1 | 19.95 | 19.95 | [Adafruit](https://www.adafruit.com/product/4502) | R | Reusable if the Gen2 box already has the ISM330DHCX (firmware 3.2.2+); older boxes have an LSM9DS1, which the firmware no longer recommends. In stock. |
| TE Connectivity 4525DO-DS5AI001DP (MS4525DO, +/-1 psid, 10-90%, I2C 0x28, 5 V cal) | AOA differential pressure P45 and Pfwd (PS1/PS2) | 2 | 83.57 | 167.14 | [Mouser](https://www.mouser.com/ProductDetail/824-4525DODS5AI001DP) | R | Gen2 part number from the Assembly Procedure, step 27. It is TE, not Honeywell. Mouser $83.57 (5 in stock); Heilind $61.11. Gen2 runs this 5 V-calibrated part at 3.3 V. For new purchases ask for 4525DO-DS3AI001DP (3.3 V) and optionally 4525DO-DS3AJ001DP (address 0x36) so both can share one bus. Price and stock for those are not confirmed. |
| Honeywell SSCSRNN1.6BA7A3 | Static pressure (PS3), I2C 0x78, 0-1.6 bar abs | 1 | 67.00 | 67.00 | [Mouser](https://www.mouser.com/ProductDetail/785-SSCSRNN1.6BA7A3) | R | Mouser $67.00 (98 in stock); Onlinecomponents $66.83; Verical $93.56. |
| Adafruit PCM5102 I2S DAC, line out (#6250) | Stereo audio DAC to audio panel | 1 | 4.95 | 4.95 | [Adafruit](https://www.adafruit.com/product/6250) |  | In stock. 2.1 Vrms full scale, ground-centred, load at least 1 kOhm. FLT, DEMP and FMT default low; MCLK comes from the internal PLL. Bring XSMT to GPIO21. |
| SparkFun Transceiver Breakout - MAX3232 (BOB-11189) | RS-232 for EFIS RX/TX and GPS/VN-300 RX | 1 | 7.26 | 7.26 | [SparkFun](https://www.sparkfun.com/sparkfun-transceiver-breakout-max3232.html) |  | In stock, runs at 3.0-5.5 V. The MAX3232 has 2 drivers and 2 receivers, the same count as the Gen2 MAX3222E. On a PCB the Gen2 MAX3222EEPN+ can be reused. |
| Adafruit Micro SD SPI or SDIO breakout, 3 V (#4682) | microSD logging (SDMMC 1-bit) | 1 | 3.50 | 3.50 | [Adafruit](https://www.adafruit.com/product/4682) |  | In stock. No level shifter and no regulator (3 V only, which suits the ESP32-S3). Add 10 kOhm pull-ups on CMD/D0-D3 if the board lacks them. |
| SanDisk Extreme PRO microSDXC 64 GB A2 | Log card (Gen2-tested model) | 1 | 12.00 | 12.00 | [Amazon](https://www.amazon.com/gp/product/B07G3GMRYF) | R | approx price. This is the card Gen2 tested for low write latency. |
| Pololu D36V28F5 5 V 3.2 A step-down regulator (#3782) | 12-14 V bus to 5 V, 50 V max in, reverse protection to 40 V | 1 | 23.95 | 23.95 | [Pololu](https://www.pololu.com/product/3782) |  | $23.95. Cheaper option: Pololu D36V6F5 (#3792, 600 mA, $5.95, no reverse protection), or reuse the Gen2 Murata OKI-78SR-5/1.5-W36H-C (Mouser $6.00, 7-36 V in). |
| Littelfuse P6KE22A TVS (600 W, 18.8 V standoff) | Input transient clamp (crowbar together with the fuse) | 1 | 0.60 | 0.60 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=P6KE22A) |  | approx price. Use SMBJ18A on a PCB. |
| Bourns MF-R050 PTC resettable fuse | Input fuse (same as Gen2 F1) | 1 | 0.55 | 0.55 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=MF-R050) | R | approx price. Keep an aircraft breaker or fuse (1-2 A) upstream as well. |
| Bourns MF-R010 PTC resettable fuse | 3.3 V sensor-power output to DB-15 pin 11 (Gen2 F2) | 1 | 0.50 | 0.50 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=MF-R010) | R | approx price |
| 1N5819 Schottky diode | Series reverse-polarity diode (Gen2 used 1N4001) | 1 | 0.40 | 0.40 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=1N5819) | R | approx price. Optional with the D36V28F5, which has reverse protection built in. |
| Aluminium electrolytic 47 uF 63 V + 100 nF ceramic | Input bulk/decoupling at the regulator (Pololu recommends 33 uF or more) | 1 | 0.60 | 0.60 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=47uF%2063V%20electrolytic) |  | approx price |
| Ferrite bead / small common-mode choke | Input EMI filter ahead of the regulator | 1 | 0.50 | 0.50 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=ferrite%20bead%20through%20hole) |  | approx price |
| Resistor assortment: 4.7k x4, 22k x3, 5.6k x2, 20k x1, 150R x4, 10k x7, 3.3k x2 | I2C pull-ups, Gen2 analog front ends, series protection, SD pull-ups, XSMT pull-down, audio attenuator | 1 | 3.00 | 3.00 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=resistor%20kit) | R | approx price. Most values are already on the Gen2 BOM. |
| Capacitors: 100 nF x10, 10 uF x4 | ADC input caps (Gen2 C12/C13), decoupling, audio coupling | 1 | 2.00 | 2.00 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=100nF%20ceramic%20capacitor) | R | approx price |
| DA-15 (DB-15) female connector, PCB right angle | Aircraft harness connector, Gen2 pinout | 1 | 3.00 | 3.00 | [Digi-Key](https://www.digikey.com/en/products/result?keywords=DA-15%20female%20right%20angle) | R | approx price. Gen2 X1 is a Harting F15HP. Reuse the Gen2 back plate, connector and harness. |
| Perma-proto / protoboard, full size | Prototype carrier | 1 | 10.95 | 10.95 | [Adafruit](https://www.adafruit.com/product/1606) |  | $10.95, out of stock 2026-10-25; any 0.1 in. protoboard works. |
| Headers, hookup wire, standoffs | Assembly | 1 | 5.00 | 5.00 | [Adafruit](https://www.adafruit.com/category/122) |  | approx price |
| BEX B2-080BK enclosure | Enclosure (Gen2) | 1 | 20.00 | 20.00 | [Amazon](https://www.amazon.com/dp/B07M77BVPC/) | R | approx price. The DevKitC (about 63 x 25 mm) fits; check the layout against the Gen2 cutout DXFs. |
| McMaster 5779K246 push-to-connect, 1/4 in. tube x 10-32 | Pitot/AOA/static panel fittings | 3 | 6.00 | 18.00 | [McMaster-Carr](https://www.mcmaster.com/5779K246) | R | approx price |
| McMaster 2844K12 barbed fitting, 5/64 in. ID x 10-32 | Internal tubing fittings | 2 | 5.00 | 10.00 | [McMaster-Carr](https://www.mcmaster.com/2844K12) | R | approx price |
| Beswick MCACS-1012-1132 barb manifold | Static tee manifold | 1 | 25.00 | 25.00 | [Beswick (phone order 603-433-1188)](http://catalog.beswick.com/) | R | approx price. Special order. |
| McMaster 5255K12 tubing 3/32 in. ID and 5255K31 1/16 in. ID | Internal sensor tubing | 2 | 5.00 | 10.00 | [McMaster-Carr](https://www.mcmaster.com/5255K12) | R | approx price, per length |
| Volume pot / switch / LED combo with knob (Gen2 Mouser project) | Panel volume, data-mark switch, status LED | 1 | 15.00 | 15.00 | [Mouser](https://www.mouser.com/ProjectManager/ProjectDetail.aspx?AccessID=17388885d5) | R | approx price. Panel-mounted and wired through the DB-15, so it is reused from the aircraft. |
| M5Stack Basic Core IoT Dev Kit v2.7 (ESP32) | Optional energy display (Gen2 OnSpeed_M5_display, board M5Stack-Core-ESP32) | 1 | 39.90 | 39.90 | [RobotShop](https://www.robotshop.com/products/m5stack-esp32-basic-core-iot-development-kit-v27) | R | RobotShop $39.90, stock not shown. Adafruit #4178 is no longer stocked. Takes 115200 baud 3.3 V TTL on GPIO22 (straight M5) or GPIO16 (power board, TTL or inverted RS-232). |
| Adafruit ADS1115 16-bit ADC (#1085) | Optional: flap/volume ADC if the ESP32-S3 ADC proves inadequate | 1 | 14.95 | 14.95 (opt) | [Adafruit](https://www.adafruit.com/product/1085) |  | OPTIONAL, not in the total. In stock. Goes on I2C bus B at 0x48. |

**Total for a complete new box: $485.75.** That breaks down as **$72.31 of new parts** when reusing a Gen2 box, plus $413.44 for the reusable items. The three pressure sensors alone are $234.14. The optional ADS1115 ($14.95) is not included.

Not in the BOM: aircraft-side wiring and flap position sensor (already installed), the Gen2 PCB, and, for the product, the custom PCB (about $5–30 for 5 boards) plus a WROOM-1-N8 module (about $4–5).

## 5. Open risks

1. **PCM5102A at the chosen clocking and level.** Bench-verify 44.1 kHz PLL lock, pops at power on and off into the actual audio panel, and the attenuator value. This is the most likely place for an unexpected result.
2. **ADC adequacy for flap detection.** Measure noise in mV and the flap-position separation with Wi-Fi transmitting. The fallback is an ADS1115 on bus B.
3. **5 V-calibrated MS4525DO at 3.3 V** (inherited from Gen2). Accuracy is unspecified. Recalibration should absorb it; buy DS3 parts for new builds, although DS3 stock and pricing are unconfirmed.
4. **Boom and display signal levels on each aircraft.** These must be at most 3.3 V TTL. Measure before connecting.
5. **Power transients.** The input chain is designed but not tested to DO-160. Scope it at engine start and alternator load switching.
6. **ESP-IDF I2C driver maturity.** Pin the IDF version and soak-test for 24 h or more with forced bus faults (short SDA momentarily) to prove recovery.
7. **Firmware fixes that must land before hardware bring-up** (details in section 2):
   - Move the console to USB-Serial-JTAG (P5).
   - Use external pull-ups and the new pins (P9).
   - Run I2S at 44.1 kHz (P11).
   - Scale the ADC to 13 bits (P12).
   - Check the sensor status bits (P13).
   - Add the IMU on bus B.
   - Set flash size to 8 MB.
8. **Parts availability.** DevKitC-1 N8R8 stock moves around: Adafruit is out, and Mouser and Digi-Key listings vary. The N8 and N8R2 boards are pin-identical for this map. The M5Stack Basic v2.7 is no longer stocked at Adafruit; Core2 or CoreS3 boards would need display-code changes.
9. **Advisory-only device.** This is experimental and uncertified. Nothing here substitutes for flight testing against a Gen2 unit.
