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
#include "flight_model.h"
#include "sensors.h"

#ifndef ONSPEED_SIM
#define ONSPEED_SIM 1
#endif

static constexpr int kSampleRate = 22050;
static constexpr int kTickMs = 20;
static constexpr int kFramesPerTick = kSampleRate * kTickMs / 1000;  // 441

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

static void audioTask(void *) {
  static int16_t pcm[kFramesPerTick * 2];
  static uint8_t u8[kFramesPerTick];   // 11025 Hz stereo u8 copy for the demo page
  static char line[1024];
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
    for (int i = 0; i + 1 < kFramesPerTick; i += 2) {  // decimate 2:1
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
  static I2cSensorBus hw;  // XIAO ESP32-S3 pin map: bus A D4/D5, bus B D9/D10, flap pot A0
  if (hw.begin(5, 6, 8, 9, 0)) bus = &hw;
#endif
  FlightModel fm;
  onspeed::G5Parser g5;
  const float dt = kTickMs / 1000.0f;
  const int ticks = (int)(fm.duration() / dt);
  const char *lastPhase = nullptr;
  uint32_t sensorCycles = 0;
  TickType_t wake = xTaskGetTickCount();
  for (int k = 0; k <= ticks; k++) {
    float t = k * dt;
    fm.step(t, dt);
    const FlightState &fs = fm.state();
    sim.setFlight(fs);

    uint32_t c0 = esp_cpu_get_cycle_count();
    uint8_t pf[2], p45[2], ps[4];
    bus->readPfwd(pf);
    bus->readP45(p45);
    bus->readStatic(ps);
    g_core->sensorRead(onspeed::parseDiffPressure(pf), onspeed::parseDiffPressure(p45),
                       onspeed::parseStaticPressure(ps), bus->readFlapPot());
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
      printf("G,%s\n", g5line);

      // Display serial out (to the M5Stack display), attitude taken from the G5.
      const onspeed::EfisData &ed = g5.data();
      char disp[96];
      g_core->buildDisplaySentence(disp, sizeof disp, ed.pitch, ed.roll, st.Palt, 0, ed.lateralG,
                                   ed.verticalG, ed.vsi, fs.gamma, 0);
      disp[strcspn(disp, "\r\n")] = 0;
      printf("S,%s\n", disp);
    }
    if (fs.phase != lastPhase) { printf("PH,%d,%s\n", k * kTickMs, fs.phase); lastPhase = fs.phase; }
    printf("D,%d,%.2f,%.4f,%.2f,%.1f,%.0f,%d,%d,%.2f,%d,%.2f,%.2f,%.3f,%.2f,%d,%.4f\n", k * kTickMs, fs.aoa,
           st.AOA, st.IAS, fs.ias, st.Palt, st.toneMode, st.highTone, st.pps, st.flapsPos, fs.pitch,
           fs.bank, fs.latG, fs.vertG, g_core->percentLift(), st.coeffP);

    xSemaphoreGive(g_audioGo);
    xSemaphoreTake(g_audioDone, portMAX_DELAY);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kTickMs));
  }
  printf("M,sensor_cycles_per_tick=%lu,synth_cycles_per_block=%lu,g5_good=%lu,g5_bad=%lu\n",
         (unsigned long)(sensorCycles / (ticks + 1)), (unsigned long)(g_synthCycles / g_synthBlocks),
         (unsigned long)g5.data().goodFrames, (unsigned long)g5.data().badFrames);
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
  i2sBegin(7, 44, 43);  // PCM5102A BCK/LRCK/DIN
#endif
  printf("BOOT,OnSpeed ESP32-S3 port test,cores=%d,sample_rate=%d\n", portNUM_PROCESSORS, kSampleRate);
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensors", 8192, nullptr, 5, nullptr, 1);
}
