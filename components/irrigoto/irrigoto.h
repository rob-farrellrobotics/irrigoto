#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// irrigoto — ESPHome component wrapper
//
// Thin C++ layer bridging ESPHome's component lifecycle with the irrigoto
// firmware written in C (main/irrigoto.c).
//
// Architecture:
//   • setup()  → calls irrigoto_init() (C extern), which does hardware init,
//                mounts LittleFS, starts web server, spins the watering-loop
//                task. WiFi/NVS are skipped — ESPHome already owns them.
//   • loop()   → polls irrigoto state getters and publishes to ESPHome entities
//                every ~2 s (PUBLISH_INTERVAL_MS).
//
// Entities exposed to Home Assistant:
//   Sensors     : pressure_psi, battery_mv, throw_mm
//   BinarySensor: watering (running / idle)
//   TextSensor  : status, zone_name
//   Select      : zone (1-4), mode (Pulse / Gentle / Smooth)
// ─────────────────────────────────────────────────────────────────────────────

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/deep_sleep/deep_sleep_component.h"
#include "esphome/components/api/custom_api_device.h"
#include <string>
#include <vector>

// Extern C interface into irrigoto.c
#include "irrigoto_api.h"

namespace esphome {
namespace irrigoto {

// ─── Zone / mode selects ─────────────────────────────────────────────────────

class IrrigotoZoneSelect : public select::Select, public Component {
 public:
  void control(const std::string &value) override;
};

class IrrigotoModeSelect : public select::Select, public Component {
 public:
  void control(const std::string &value) override;
};

// ─── Main component ──────────────────────────────────────────────────────────

class IrrigotoComponent : public Component, public api::CustomAPIDevice {
 public:
  // Component lifecycle
  void setup() override;
  void loop() override;
  // AFTER_WIFI: must run after the wifi component initialises esp_netif /
  // lwIP, because irrigoto_init() starts an esp_http_server (ota_http_start
  // + zone_web_start) which requires the tcpip mutex to exist. Running at
  // HARDWARE caused a NULL-mutex assert in lwip_socket → sys_mutex_lock.
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Called from HA service lambdas
  void start_watering(int zone, int mode, int duration_s);
  float read_pressure_now();
  void set_led(const std::string &color);
  void set_auto_sleep(bool enabled);
  bool get_auto_sleep();
  void sleep_now(int duration_s);
  void valve_goto(float target_deg);
  void nozzle_goto(float target_deg);
  void aim_valve(float bearing_deg, float valve_deg);
  void aim_throw(float bearing_deg, float throw_mm);
  void water_arc(float throw_mm, float arc_start_deg, float arc_end_deg,
                 float speed_dps, const std::string &direction);
  float get_valve_deg();
  float get_nozzle_deg();
  void stop_and_close();
  void valve_close();
  void valve_open();
  void cal_pressure_start();
  void cal_pressure_throw(float throw_mm);
  void cal_pressure_cancel();
  void cal_nozzle_start();

  // Persistent power-management settings (NVS-backed via the C layer).
  // b347: read the most-recent boot diagnostics record (NVS-persisted at
  // the end of check_battery_on_boot / check_valve_closed_on_boot). Empty
  // string if the device has never written one. Full per-entry detail
  // available via GET /api/boot_diag on the device's HTTP server.
  std::string get_boot_diag_summary();

  uint32_t get_inactivity_minutes();
  void     set_inactivity_minutes(uint32_t m);
  uint32_t get_sleep_duration_s();
  void     set_sleep_duration_s(uint32_t s);
  uint32_t get_dwell_timeout_s();
  void     set_dwell_timeout_s(uint32_t s);

  // Schedule (HA scheduler-agnostic — pushes raw text format).
  // set_schedule(): legacy REPLACE-ALL (no IDs). Deprecated; kept for
  //                 back-compat with older HA installs.
  // sync_schedule(): new merge / LWW push, see docs/schedule_sync_design.md
  void set_schedule(const std::string &text);
  void sync_schedule(const std::string &text);
  void clear_schedule();
  // Rain / wind delay. Combined "hours" service: pass 0 to clear,
  // anything else to delay that many hours from now. Single service
  // (not two) keeps the API service count low — the b256 workaround
  // for the ESPHome 2026.4 buffer crash is sensitive to that count.
  void delay_schedule_hours(int hours);

  // Cal state text sensor (published from loop()).
  void set_cal_state_sensor(text_sensor::TextSensor *s) { cal_state_sensor_ = s; }
  void set_last_sleep_reason_sensor(text_sensor::TextSensor *s) { last_sleep_reason_sensor_ = s; }
  void set_next_run_sensor(text_sensor::TextSensor *s) { next_run_sensor_ = s; }
  void set_schedule_status_sensor(text_sensor::TextSensor *s) { schedule_status_sensor_ = s; }
  void set_schedule_text_sensor(text_sensor::TextSensor *s) { schedule_text_sensor_ = s; }
  void set_schedule_delay_sensor(text_sensor::TextSensor *s) { schedule_delay_sensor_ = s; }
  // b356: schedule_version text_sensor REMOVED. It was an extra ListEntities
  // row contributing to the b256-class API send-buffer overflow crashes.
  // The version is still surfaced via the schedule_changed event's
  // schedule_version field, and the C-level irrigoto_schedule_version()
  // getter remains for direct HTTP queries.

  // Entity setters (called from code-gen)
  void set_pressure_sensor(sensor::Sensor *s)             { pressure_sensor_  = s; }
  void set_battery_sensor(sensor::Sensor *s)              { battery_sensor_   = s; }
  void set_throw_sensor(sensor::Sensor *s)                { throw_sensor_     = s; }
  void set_water_minutes_remaining_sensor(sensor::Sensor *s) { water_remaining_sensor_ = s; }
  void set_watering_sensor(binary_sensor::BinarySensor *s){ watering_sensor_  = s; }
  void set_valve_open_sensor(binary_sensor::BinarySensor *s){ valve_open_sensor_ = s; }
  void set_status_sensor(text_sensor::TextSensor *s)      { status_sensor_    = s; }
  void set_zone_name_sensor(text_sensor::TextSensor *s)   { zone_name_sensor_ = s; }
  void set_zone_select(IrrigotoZoneSelect *s)             { zone_select_      = s; }
  void set_mode_select(IrrigotoModeSelect *s)             { mode_select_      = s; }
  // Optional link to ESPHome's deep_sleep component. If unset, sleep
  // requests fall back to a direct esp_deep_sleep_start() and HA will see
  // an unannounced disconnect.
  void set_deep_sleep(deep_sleep::DeepSleepComponent *ds) { deep_sleep_ = ds; }

 protected:
  static constexpr uint32_t PUBLISH_INTERVAL_MS = 2000;
  uint32_t last_publish_ms_{0};

  // Transition trackers for the watering / cal completion HA events.
  // Each loop tick compares against the live state and fires an event
  // when the device just exited an active phase.
  bool prev_watering_{false};
  int  prev_cal_state_{0};
  void fire_watering_complete_event_();
  void fire_cal_complete_event_(int prev_state, int cur_state);
  // Fired whenever the schedule text changes (HA push, web UI edit, clear).
  // Lets HA mirror the live schedule without having to read a long sensor
  // value piece by piece.
  void fire_schedule_changed_event_();
  // Last schedule text we observed; loop() compares against this to decide
  // whether to republish the text sensor and fire the changed event.
  std::string prev_schedule_text_;

  sensor::Sensor              *pressure_sensor_{nullptr};
  sensor::Sensor              *battery_sensor_{nullptr};
  sensor::Sensor              *throw_sensor_{nullptr};
  sensor::Sensor              *water_remaining_sensor_{nullptr};
  binary_sensor::BinarySensor *watering_sensor_{nullptr};
  binary_sensor::BinarySensor *valve_open_sensor_{nullptr};
  int                          prev_valve_open_{-2};  // -2 = never published
  text_sensor::TextSensor     *status_sensor_{nullptr};
  text_sensor::TextSensor     *zone_name_sensor_{nullptr};
  text_sensor::TextSensor     *cal_state_sensor_{nullptr};
  text_sensor::TextSensor     *last_sleep_reason_sensor_{nullptr};
  text_sensor::TextSensor     *next_run_sensor_{nullptr};
  text_sensor::TextSensor     *schedule_status_sensor_{nullptr};
  text_sensor::TextSensor     *schedule_text_sensor_{nullptr};
  text_sensor::TextSensor     *schedule_delay_sensor_{nullptr};
  time_t                       prev_delay_until_{-1};  // -1 = first tick
  // Backing storage for the zone-select option strings. ESPHome's
  // SelectTraits stores raw const char* pointers, so the underlying
  // string buffers have to outlive the call to set_options(). Keeping
  // them on the component (which lives for the lifetime of the app)
  // satisfies that.
  std::vector<std::string>     zone_option_storage_;
  IrrigotoZoneSelect          *zone_select_{nullptr};
  IrrigotoModeSelect          *mode_select_{nullptr};
  deep_sleep::DeepSleepComponent *deep_sleep_{nullptr};
};

}  // namespace irrigoto
}  // namespace esphome
