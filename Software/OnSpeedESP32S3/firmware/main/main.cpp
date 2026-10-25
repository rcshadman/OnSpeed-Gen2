// OnSpeed on ESP32-S3: task layout for the port.
//   core 1, sensorTask  50 Hz : read pressures -> AOA/IAS -> tone selection -> display/EFIS serial
//   core 1, audioTask         : tone synth -> I2S (real hardware) or test sink (simulator)
//   core 0                    : Wi-Fi/web UI and SD logging go here in the full port (not in this test)
// Build with ONSPEED_SIM=1 (default) to drive the virtual sensors from a scripted flight and print
// telemetry on the console UART; that is what runs on the virtual board in QEMU.
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "onspeed_core.h"
#include "tone_synth.h"
#include "g5.h"
#include "canfix.h"
#include "flight_model.h"
#include "sensors.h"
#include "imu_sim.h"
#include "ahrs.h"
#include "board_pins.h"
#include "driver/gpio.h"

#ifndef ONSPEED_SIM
#define ONSPEED_SIM 1
#endif

// 44.1 kHz: at 22.05 kHz/16-bit the bit clock (705.6 kHz) is below what the PCM5102A's internal PLL
// accepts when SCK is tied low; 44.1 kHz also matches the Gen2 voice clips.
static constexpr int kSampleRate = 44100;
static constexpr int kTickMs = 20;
static constexpr int kFramesPerTick = kSampleRate * kTickMs / 1000;  // 882

static onspeed::Config g_cfg;
static onspeed::Core *g_core;
static onspeed::ToneSynth *g_synth;
static SemaphoreHandle_t g_audioGo, g_audioDone;
static volatile bool g_done = false;
static volatile uint32_t g_synthCycles = 0, g_synthBlocks = 0;
static i2s_chan_handle_t g_i2s = nullptr;

// ---- real-hardware audio path (PCM5102A on I2S0); compiled but not started in the simulator ----
[[maybe_unused]] static bool i2sBegin(int bclk, int ws, int dout) {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&cc, &g_i2s, nullptr) != ESP_OK) return false;
  i2s_std_config_t sc = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)bclk, .ws = (gpio_num_t)ws,
                   .dout = (gpio_num_t)dout, .din = I2S_GPIO_UNUSED,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  if (i2s_channel_init_std_mode(g_i2s, &sc) != ESP_OK) return false;
  return i2s_channel_enable(g_i2s) == ESP_OK;
}

#if ONSPEED_SIM
static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static int b64(const uint8_t *in, int n, char *out) {
  int o = 0;
  for (int i = 0; i < n; i += 3) {
    uint32_t v = in[i] << 16 | (i + 1 < n ? in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
    out[o++] = kB64[(v >> 18) & 63];
    out[o++] = kB64[(v >> 12) & 63];
    out[o++] = i + 1 < n ? kB64[(v >> 6) & 63] : '=';
    out[o++] = i + 2 < n ? kB64[v & 63] : '=';
  }
  out[o] = 0;
  return o;
}

#endif

static void audioTask(void *) {
  static int16_t pcm[kFramesPerTick * 2];
#if ONSPEED_SIM
  static uint8_t u8[kFramesPerTick / 2];   // 11025 Hz stereo u8 copy for the demo page
  static char line[1100];
#endif
  for (;;) {
    xSemaphoreTake(g_audioGo, portMAX_DELAY);
    if (g_done) { xSemaphoreGive(g_audioDone); vTaskDelete(nullptr); }
    uint32_t c0 = esp_cpu_get_cycle_count();
    g_synth->render(pcm, kFramesPerTick);
    g_synthCycles += esp_cpu_get_cycle_count() - c0;
    g_synthBlocks = g_synthBlocks + 1;
    if (g_i2s) {
      size_t w; i2s_channel_write(g_i2s, pcm, sizeof pcm, &w, portMAX_DELAY);
    }
#if ONSPEED_SIM
    onspeed::ToneEvent e;
    while (g_synth->popEvent(e)) printf("P,%lu,%u,%u,%.2f\n", (unsigned long)e.sampleIndex, e.freq, e.mode, e.volume);
    int n = 0;
    for (int i = 0; i + 3 < kFramesPerTick; i += 4) {  // decimate 4:1
      u8[n++] = (uint8_t)((pcm[2 * i] >> 8) + 128);
      u8[n++] = (uint8_t)((pcm[2 * i + 1] >> 8) + 128);
    }
    line[0] = 'W'; line[1] = ',';
    b64(u8, n, line + 2);
    puts(line);
#endif
    xSemaphoreGive(g_audioDone);
  }
}

static void sensorTask(void *) {
  SimSensorBus sim(g_cfg);
  SensorBus *bus = &sim;
#if !ONSPEED_SIM
  static I2cSensorBus hw;  // ESP32-S3-DevKitC-1 pin map, board_pins.h
  if (hw.begin(PIN_I2C_A_SDA, PIN_I2C_A_SCL, PIN_I2C_B_SDA, PIN_I2C_B_SCL, ADC_CH_FLAP)) bus = &hw;
#endif
  FlightModel fm;
  onspeed::G5Parser g5;
  // CAN-FiX bus to the MakerPlane EFIS. On hardware the send hook queues to the TWAI driver;
  // here it prints each frame the board puts on the bus.
  static char canLine[512];
  static int canLen = 0;
  auto canTap = [](const onspeed::CanFrame &f, void *) {
    if (canLen > (int)sizeof canLine - 32) return true;
    canLen += snprintf(canLine + canLen, sizeof canLine - canLen, "%03X#", f.id);
    for (int i = 0; i < f.dlc; i++) canLen += snprintf(canLine + canLen, sizeof canLine - canLen, "%02X", f.data[i]);
    canLine[canLen++] = ' ';
    canLine[canLen] = 0;
    return true;
  };
  onspeed::CanfixNode can(onspeed::CanfixNode::Config{}, canTap, nullptr);
  const float dt = kTickMs / 1000.0f;
  const int ticks = (int)(fm.duration() / dt);
  const char *lastPhase = nullptr;
  uint32_t sensorCycles = 0;
  ImuSim imu;
  onspeed::AhrsConfig acfg;
  acfg.imuSampleRate = 200;
  acfg.fixKnownBugs = true;  // the port runs with the Gen2 bug fixes on (see ahrs.cpp BUG notes)
  onspeed::Ahrs ahrs(acfg);
  uint32_t ahrsCycles = 0;
  TickType_t wake = xTaskGetTickCount();
  for (int k = 0; k <= ticks; k++) {
    float t = k * dt;
    fm.step(t, dt);
    const FlightState &fs = fm.state();
    sim.setFlight(fs);

    // On-board AHRS (Gen2 Madgwick + Kalman port) fed by the simulated ISM330DHCX.
    // The IMU runs at 200 Hz here (4 samples per 20 ms tick); the real chip runs at 208 Hz.
    imu.step(fs, dt);
    uint32_t a0 = esp_cpu_get_cycle_count();
    for (int j = 0; j < 4; j++) {
      onspeed::AircraftImu m = imu.sample();
      if (k == 0 && j == 0) ahrs.begin(m, fs.palt);
      ahrs.processAHRS(m, g_core->state().IAS, g_core->state().Palt, 15, g_core->state().AOA);
    }
    ahrsCycles += esp_cpu_get_cycle_count() - a0;

    uint32_t c0 = esp_cpu_get_cycle_count();
    uint8_t pf[2] = {0}, p45[2] = {0}, ps[4] = {0};
    bool okF = bus->readPfwd(pf);
    bool okA = bus->readP45(p45);
    bool okS = bus->readStatic(ps);
    g_core->sensorReadBytes(pf, okF, p45, okA, ps, okS, bus->readFlapPot());
    g_core->updateTones();
    const onspeed::State &st = g_core->state();
    g_synth->setToneState(st.toneMode, st.highTone, st.pps);
    g_synth->update3DAudio(fs.latG);
    sensorCycles += esp_cpu_get_cycle_count() - c0;

    // EFIS serial line: the simulator plays the Garmin G5; bytes go through the Gen2 parser.
    if (k % 5 == 0) {
      char g5line[64];
      int s = (int)t;
      onspeed::buildG5Sentence(g5line, sizeof g5line, 14, (s / 60) % 60, s % 60, (k % 50) * 2,
                               fs.pitch, fs.bank, (int)fs.heading, fs.ias, (int)fs.palt, fs.latG,
                               fs.vertG, (int)fs.vsi);
      for (char *p = g5line; *p; p++) g5.feed(*p);
      g5line[strcspn(g5line, "\r\n")] = 0;
      if (ONSPEED_SIM) printf("G,%s\n", g5line);

      // Display serial out (to the M5Stack display), attitude taken from the G5.
      const onspeed::EfisData &ed = g5.data();
      char disp[96];
      g_core->buildDisplaySentence(disp, sizeof disp, ed.pitch, ed.roll, st.Palt, 0, ed.lateralG,
                                   ed.verticalG, ed.vsi, fs.gamma, 0);
      disp[strcspn(disp, "\r\n")] = 0;
      if (ONSPEED_SIM) printf("S,%s\n", disp);

      // MakerPlane EFIS: the board sends AOA/IAS, the EFIS (node 0x91) sends attitude and air data
      canLen = 0; canLine[0] = 0;
      onspeed::OnSpeedOutputs o;
      o.AOA = st.sensorFault ? onspeed::kNaN : st.AOA;
      o.IAS = st.IAS;
      o.LDmaxAOA = st.LDmaxAOA; o.onSpeedFastAOA = st.onSpeedAOAfast; o.onSpeedSlowAOA = st.onSpeedAOAslow;
      o.stallWarnAOA = st.stallWarningAOA;
      can.publish(o, k * kTickMs);
      struct { uint16_t id; float v; } efis[] = {{onspeed::canfix::PITCH, fs.pitch}, {onspeed::canfix::ROLL, fs.bank},
          {onspeed::canfix::IAS, fs.ias}, {onspeed::canfix::PALT, fs.palt}, {onspeed::canfix::HEAD, fs.heading},
          {onspeed::canfix::VS, fs.vsi}, {onspeed::canfix::ALAT, fs.latG}, {onspeed::canfix::ANORM, fs.vertG}};
      onspeed::CanFrame cf;
      for (auto &e : efis) if (onspeed::canfix::encodeParam(0x91, e.id, e.v, cf)) can.onFrame(cf, k * kTickMs);
      if (ONSPEED_SIM) printf("C,%s|%.2f,%.2f,%d\n", canLine, can.efis().pitch, can.efis().roll, can.fresh(k * kTickMs));
    }
    if (ONSPEED_SIM) {
      const onspeed::AhrsState &as = ahrs.state();
      printf("A,%d,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f\n", k * kTickMs, as.smoothedPitch, as.smoothedRoll, fs.pitch, fs.bank,
             as.kalmanVSI * onspeed::AHRS_MPS2FPM, fs.vsi);
    }
    if (ONSPEED_SIM && fs.phase != lastPhase) { printf("PH,%d,%s\n", k * kTickMs, fs.phase); lastPhase = fs.phase; }
    if (ONSPEED_SIM) printf("D,%d,%.2f,%.4f,%.2f,%.1f,%.0f,%d,%d,%.2f,%d,%.2f,%.2f,%.3f,%.2f,%d,%.4f,%d\n", k * kTickMs, fs.aoa,
           st.AOA, st.IAS, fs.ias, st.Palt, st.toneMode, st.highTone, st.pps, st.flapsPos, fs.pitch,
           fs.bank, fs.latG, fs.vertG, g_core->percentLift(), st.coeffP, st.sensorFault);

    xSemaphoreGive(g_audioGo);
    xSemaphoreTake(g_audioDone, portMAX_DELAY);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kTickMs));
  }
  const onspeed::State &fin = g_core->state();
  printf("M,sensor_cycles_per_tick=%lu,synth_cycles_per_block=%lu,g5_good=%lu,g5_bad=%lu,can_sent=%lu,"
         "pfwd_err=%lu,pfwd_stale=%lu,p45_diag=%lu,ahrs_cycles_per_update=%lu\n",
         (unsigned long)(sensorCycles / (ticks + 1)), (unsigned long)(g_synthCycles / g_synthBlocks),
         (unsigned long)g5.data().goodFrames, (unsigned long)g5.data().badFrames, (unsigned long)can.framesSent(),
         (unsigned long)fin.pfwdHealth.readErrors, (unsigned long)fin.pfwdHealth.stale, (unsigned long)fin.p45Health.diagnostic,
         (unsigned long)(ahrsCycles / (4 * (ticks + 1))));
  g_done = true;
  xSemaphoreGive(g_audioGo);
  xSemaphoreTake(g_audioDone, portMAX_DELAY);
  printf("DONE\n");
  vTaskDelete(nullptr);
}

extern "C" void app_main(void) {
  esp_log_level_set("*", ESP_LOG_WARN);
#if ONSPEED_SIM
  // per-unit sensor zero offsets: the simulated sensors are ideal, so their zero is mid-scale
  g_cfg.pFwdBias = 8192;
  g_cfg.p45Bias = 8192;
#endif
  static onspeed::Core core(g_cfg);
  static onspeed::ToneSynth synth(kSampleRate);
  g_core = &core;
  g_synth = &synth;
  g_audioGo = xSemaphoreCreateBinary();
  g_audioDone = xSemaphoreCreateBinary();
#if !ONSPEED_SIM
  // DAC stays muted (XSMT low, held by the board's pull-down) until I2S is clocking silence,
  // which avoids the PCM5102A power-up pop.
  gpio_set_direction((gpio_num_t)PIN_DAC_XSMT, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)PIN_DAC_XSMT, 0);
  if (i2sBegin(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT)) {
    static int16_t zeros[256] = {0};
    size_t w;
    for (int i = 0; i < 8; i++) i2s_channel_write(g_i2s, zeros, sizeof zeros, &w, portMAX_DELAY);
    gpio_set_level((gpio_num_t)PIN_DAC_XSMT, 1);
  }
#endif
  printf("BOOT,OnSpeed ESP32-S3 port test,cores=%d,sample_rate=%d\n", portNUM_PROCESSORS, kSampleRate);
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensors", 8192, nullptr, 5, nullptr, 1);
}
