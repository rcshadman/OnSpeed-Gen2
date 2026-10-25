# OnSpeed ESP32-S3 port: virtual-board test

- `firmware/`: ESP-IDF 5.4 project. `components/onspeed_core` is the Gen2 logic ported to plain C++
  (AOA, tone selection, a tone synth replacing the Teensy Audio library, the M5 display sentence, the Garmin G5 parser).
  `main/` runs it as FreeRTOS tasks. The default build drives simulated sensors from a scripted flight.
  `ONSPEED_HW=1` builds the real-hardware variant (I2C sensors, ADC flap pot, I2S DAC). It compiles but is untested.
- `qemu_run.log`: console output of the firmware running on Espressif QEMU (`-machine esp32s3`).
- `check_run.py` / `gen2_reference.py`: check that output against an independent Python
  re-implementation of the Gen2 math and analyse the rendered audio. Last run: 16 of 16 passed.

Run: build with `idf.py build`, merge with `esptool merge_bin`, then
`qemu-system-xtensa -nographic -machine esp32s3 -drive file=flash.bin,if=mtd,format=raw -serial file:run.log`,
then `python3 check_run.py run.log out/`.
