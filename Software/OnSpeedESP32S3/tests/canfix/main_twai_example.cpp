// ESP-IDF (v5.4, legacy driver/twai.h) glue: OnSpeed box as a CAN-FiX node on the
// MakerPlane bus. Example only; not part of main/. Wire an external 3.3 V CAN
// transceiver (e.g. TI SN65HVD230 / TCAN334 for bench, TCAN1044AV-Q1 with VIO=3.3V
// for the aircraft) to TWAI_TX_GPIO / TWAI_RX_GPIO.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "canfix.h"

static const char *TAG = "canfix";

// Proposed pins (prototype already uses 1,5,6,7,8,9,43,44; avoid strapping 0/3/45/46,
// USB 19/20, flash/PSRAM 26-37). Any GPIO works via the GPIO matrix.
#define TWAI_TX_GPIO GPIO_NUM_17
#define TWAI_RX_GPIO GPIO_NUM_18

static uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

static bool twaiSend(const onspeed::CanFrame &f, void *) {
  twai_message_t m = {};
  m.identifier = f.id;     // 11-bit standard id (extd = 0)
  m.data_length_code = f.dlc;
  for (int i = 0; i < f.dlc; i++) m.data[i] = f.data[i];
  return twai_transmit(&m, 0) == ESP_OK;   // never block the AOA loop
}

static bool twaiInstall() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16;
  g.rx_queue_len = 32;
  // CAN-FiX default bit rate is 125 kbit/s (125k and 250k are the mandatory rates).
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
  // Hardware filter: standard frames only. The ids we need are spread over
  // 0x180-0x1BF (flight data), 0x406/0x407 (air temp), 0x6E0-0x7DF (node specific),
  // which no single code/mask covers, so accept all 11-bit ids and let
  // CanfixNode::onFrame() discard the rest (bus load at 125k is small).
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

// Shared with the AOA task in the real firmware.
static onspeed::OnSpeedOutputs g_out;

static void canfixTask(void *) {
  onspeed::CanfixNode::Config cfg;   // node 0x50 by default; must be unique on the bus
  onspeed::CanfixNode node(cfg, twaiSend, nullptr);
  onspeed::DisplaySink *sink = &node;      // swap for a custom display link later
  onspeed::EfisSource *efis = &node;
  uint32_t lastTx = 0;
  for (;;) {
    twai_message_t m;
    while (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (m.extd || m.rtr) continue;
      onspeed::CanFrame f;
      f.id = (uint16_t)m.identifier;
      f.dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
      for (int i = 0; i < f.dlc; i++) f.data[i] = m.data[i];
      node.onFrame(f, nowMs());
    }
    uint32_t t = nowMs();
    if (t - lastTx >= 50) {   // 20 Hz AOA/IAS; meta + node status at 1 Hz internally
      lastTx = t;
      sink->publish(g_out, t);
    }
    // Recover from bus-off (e.g. unterminated bus during bench work).
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
      if (st.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "bus off, recovering");
        twai_initiate_recovery();
      } else if (st.state == TWAI_STATE_STOPPED) {
        twai_start();   // recovery finished
      }
    }
    if (efis->fresh(t)) {
      // efis->efis().pitch / roll / IAS / palt / vsi / lateralG / verticalG ready
    }
  }
}

extern "C" void app_main(void) {
  if (!twaiInstall()) { ESP_LOGE(TAG, "TWAI install failed"); return; }
  ESP_LOGI(TAG, "TWAI up at 125 kbit/s, TX=%d RX=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
  g_out.AOA = 0; g_out.IAS = 0;
  g_out.stallWarnAOA = 16.48f; g_out.gaugeMinAOA = -10; g_out.gaugeMaxAOA = 25;
  xTaskCreatePinnedToCore(canfixTask, "canfix", 4096, nullptr, 5, nullptr, 0);
}
