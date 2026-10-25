// Pin map for the ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-1 N8R8) prototype and the OnSpeed S3 custom board
// (Hardware/OnSpeedS3, WROOM-1 N8, no PSRAM). Every pin below is the same on both. Avoided: strapping pins 0/45/46 and USB 19/20
// (console runs on USB-Serial-JTAG), 43/44 (UART0, boot ROM output). GPIO3 is a strap only once the JTAG-select eFuse is burned, so the
// custom board uses it for the VIN monitor. On the DevKitC, 35-37 are octal PSRAM and 38/48 the RGB LED; the custom board (N8, no PSRAM)
// uses them in the ONSPEED_BOARD_S3 block below.
// Source: /mnt/project-files/onspeed-port/hardware-verification.md (pin audit).
#pragma once

// I2C bus A: Pfwd MS4525DO (0x28) + static SSCSRNN1.6BA7A3 (0x78). External 4.7k pull-ups required.
#define PIN_I2C_A_SDA 8
#define PIN_I2C_A_SCL 9
// I2C bus B: P45 MS4525DO (0x28) + ISM330DHCX IMU (0x6A). External 4.7k pull-ups required.
#define PIN_I2C_B_SDA 17
#define PIN_I2C_B_SCL 18
#define I2C_HZ 400000          // ESP32-S3 maximum (Gen2 Teensy ran these buses faster)

// Analog inputs on ADC1 (ADC2 is unusable while Wi-Fi runs)
#define ADC_CH_FLAP 0          // GPIO1, DB-15 pin 7, Gen2 divider 22k/5.6k + 100 nF
#define ADC_CH_VOLUME 1        // GPIO2, DB-15 pin 10

// PCM5102A I2S DAC
#define PIN_I2S_BCLK 12
#define PIN_I2S_WS 13
#define PIN_I2S_DOUT 14
#define PIN_DAC_XSMT 21        // 10k pull-down on the board keeps the DAC muted until firmware is up

// Serial
#define PIN_DISPLAY_TX 10      // M5Stack "#1" sentences, 115200 8N1, 3.3 V TTL
#define PIN_EFIS_TX 11         // MAX3232 T1IN
#define PIN_EFIS_RX 15         // MAX3232 R1OUT (Garmin G5 / G3X / Dynon / MGL)
#define PIN_GPS_RX 16          // MAX3232 R2OUT (VN-300)
#define PIN_BOOM_RX 7          // 3.3 V TTL only

// CAN-FiX (MakerPlane EFIS), 125 kbit/s. Custom board: TJA1051T/3 (VCC 5 V, VIO 3.3 V)
#define PIN_CAN_TX 47
#define PIN_CAN_RX 4

// Panel
#define PIN_PANEL_SWITCH 5
#define PIN_AOA_LED 6

// microSD, 1-bit SDMMC
#define PIN_SD_CLK 39
#define PIN_SD_CMD 40
#define PIN_SD_D0 41
#define PIN_SD_D3 42

// Custom board only (OnSpeed S3 rev A). Define ONSPEED_BOARD_S3 when building for it. On the DevKitC these
// GPIOs carry the octal PSRAM (35-37) and the RGB LED (38, 48), so they must stay unused there.
#ifdef ONSPEED_BOARD_S3
#define PIN_IMU_INT1 35        // ISM330DHCX INT1, push-pull, no pull-up
#define PIN_SD_DET 36          // card detect, 10k pull-up
#define PIN_CAN_SILENT 37      // TJA1051 S: high = listen-only, 10k pull-down = normal
#define PIN_STATUS_LED 38      // blue LED, active high
#define PIN_PGOOD 48           // LM5164 PGOOD, 100k pull-up; low = 5 V rail out of regulation
#define ADC_CH_VIN 2           // GPIO3 (ADC1 CH2), aircraft bus: VIN = Vadc x (100k + 8.2k) / 8.2k
#define VIN_DIVIDER (158.2f / 8.2f)
// The two MS4525DO run from 5 V behind PCA9306 translators; firmware sees them at 0x28 on buses A and B
// exactly as on the prototype.
#endif
