# OnSpeed on the ESP32-S3

This folder ports the OnSpeed Gen2 firmware from the end-of-life Teensy 3.6 to the ESP32-S3. It runs on Espressif's QEMU, so the whole
flight can be tested without hardware.

## Folders

- `firmware/` is an ESP-IDF 5.4 project.
  - `components/onspeed_core` holds the Gen2 logic as plain C++: AOA, tone selection, the tone synth, the M5 display sentence,
    the Garmin G5 parser, MakerPlane CAN-FiX, the flap detector and the AHRS (Madgwick + Kalman).
  - `main/` runs it as FreeRTOS tasks. The default build reads simulated sensors from a scripted flight, including an ISM330DHCX
    model with datasheet noise, offsets and quantisation.
  - `ONSPEED_HW=1 idf.py -B build-hw build` builds the hardware variant for the ESP32-S3-DevKitC-1 (pins in `main/board_pins.h`).
    It compiles, but it has not been run on hardware yet.
- `sim/check_run.py` checks a QEMU run against an independent Python version of the Gen2 maths (`gen2_reference.py`),
  and decodes the CAN-FiX frames with MakerPlane's python-canfix.
- `tests/` holds the host unit tests:
  - `core`: 325 checks, built with ASan/UBSan
  - `canfix`
  - `ahrs`: an IMU simulator with real noise, plus checks for each part of the algorithm
  - `ahrs_flight`: the QEMU flight profile through the AHRS
- `wasm/` builds the same core to WebAssembly for the browser demo in `demo/`.

## Design decisions

- **No Wi-Fi in flight.** Every aircraft connection is wired. Wi-Fi is only for ground maintenance mode (panel switch held at
  power-up) or USB, and is not compiled in yet.
- **The EFIS link is swappable.** MakerPlane CAN-FiX is the current EFIS. Display and EFIS code sit behind
  `DisplaySink`/`EfisSource`, so a custom display can replace it.
- **Flap position comes from a sensor** (Hall, pot or detent ladder) on one ADC input. Learn mode records each detent.
  Stage changes use hysteresis plus a 0.2 s debounce. On a sensor fault the last good stage is held.
- **Sensor faults never give a wrong tone.** Stale reads and diagnostic status are rejected. 25 bad reads in a row mute the tones
  until the sensor recovers.
- **The AHRS runs with the Gen2 bug fixes on** (`fixKnownBugs`). See `tests/ahrs/RESULTS.md` for the seven bugs found in the original.

## Run

```
cd firmware && idf.py build
cd build && python -m esptool --chip esp32s3 merge_bin --fill-flash-size 4MB -o flash.bin @flash_args && cd ..
qemu-system-xtensa -nographic -machine esp32s3 -icount shift=0 -drive file=build/flash.bin,if=mtd,format=raw -serial file:run.log
python3 ../sim/check_run.py run.log out/
```
