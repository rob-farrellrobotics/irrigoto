#include "irrigoto.h"
#include "fw_version.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/wifi/wifi_component.h"   // b293
#include <cstring>
#include <cstdio>
#include <ctime>
#include <map>
#include <esp_sleep.h>

namespace esphome {
namespace irrigoto {

static const char *TAG = "irrigoto";

// ─────────────────────────────────────────────────────────────────────────────
// IrrigotoComponent — lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void IrrigotoComponent::setup() {
    ESP_LOGI(TAG, "Irrigoto init (ESPHome mode)");
    // irrigoto_init() skips wifi_init() and nvs_flash_init() because ESPHome
    // already called those before components are set up.
    irrigoto_init();
    ESP_LOGI(TAG, "Irrigoto ready");
}

void IrrigotoComponent::loop() {
    // ── Watering / cal completion → HA event ─────────────────────────────────
    // Fire a single event when the device transitions from "active" to
    // "idle" so HA automations can react (notify, log, fetch CSV, etc.).
    // Polled every loop tick — cheap, doesn't depend on the publish gate.
    bool cur_watering = irrigoto_is_watering();
    if (prev_watering_ && !cur_watering) {
        fire_watering_complete_event_();
    }
    prev_watering_ = cur_watering;

    int cur_cal = irrigoto_cal_state_code();
    // "Active" = scanning, awaiting throw, or nozzle scan. Done/Error/Idle
    // all count as terminal — fire on transition into any of them from
    // an active state, so cancellation also notifies HA.
    auto cal_is_active = [](int s){ return s == 1 || s == 2 || s == 3; };
    if (cal_is_active(prev_cal_state_) && !cal_is_active(cur_cal)) {
        fire_cal_complete_event_(prev_cal_state_, cur_cal);
    }
    prev_cal_state_ = cur_cal;

    // Sleep request handoff. Polled every loop tick (not gated by the
    // publish interval) so the device sleeps promptly after a request.
    uint32_t sleep_dur_s = 0;
    if (irrigoto_sleep_request_pending(&sleep_dur_s)) {
        irrigoto_sleep_request_clear();
        if (deep_sleep_ != nullptr) {
            ESP_LOGI(TAG, "Routing %u s sleep through ESPHome deep_sleep "
                     "(HA notified gracefully)", sleep_dur_s);
            deep_sleep_->set_sleep_duration(sleep_dur_s * 1000);  // ms
            deep_sleep_->begin_sleep(true);  // manual=true
            return;  // begin_sleep schedules sleep on next loop; bail now
        }
        // Fallback: no deep_sleep wired. Sleep directly (HA will see an
        // unannounced disconnect).
        ESP_LOGW(TAG, "Sleep requested but no deep_sleep component linked; "
                 "sleeping directly (HA disconnect will be unannounced)");
        esp_sleep_enable_timer_wakeup((uint64_t)sleep_dur_s * 1000000ULL);
        esp_deep_sleep_start();
    }

    uint32_t now = millis();
    if (now - last_publish_ms_ < PUBLISH_INTERVAL_MS) return;
    last_publish_ms_ = now;

    // b287: "Watering Quiet" mode -- while a smooth/gentle run is in flight,
    // throttle high-rate sensor publishes to keep the WiFi/NVS pipe calm.
    // The race that crashes the device during long runs is between SPI
    // flash operations (LittleFS / ESPHome NVS preferences) and WiFi PHY
    // activity, and every sensor publish that flows through ESPHome's API
    // can trigger NVS state-persistence writes. Skipping pressure/battery/
    // throw/cal/sleep_reason/next_run during the run dramatically reduces
    // the rate of these writes. Progress sensors (watering binary, status
    // text, zone name, watering mode) keep publishing so HA still sees
    // the run live.
    //
    // Resumes within a few seconds of the run ending (after a 3s LFS-settle
    // delay applied inside phase_water_zone()). On resume, the first loop()
    // iteration after the quiet mode clears will catch up by publishing
    // every sensor with its latest value -- so HA's history shows the
    // pre-watering value, a gap during watering, then the post-watering
    // value, which is what we want for diagnostics.
    bool quiet = irrigoto_is_watering_quiet();

    // ── Sensors ──────────────────────────────────────────────────────────────
    // Sanity-clamp each numeric sensor before publishing. A bogus I2C
    // read or transient ADC glitch can occasionally produce a wildly
    // out-of-range value (e.g. 8000 psi from an MPRLS hiccup, or a
    // momentary 4 V → 4000 mV interpretation error). Recorder treats
    // these as real and stores them forever, which makes long-term
    // history graphs unreadable. Drop the publish (keep last good
    // state in HA) instead of letting garbage poison the recorder.
    if (!quiet && pressure_sensor_ != nullptr) {
        float psi = irrigoto_get_pressure_psi();
        // Residential supply tops out around 80 psi; allow plenty of
        // headroom but reject obvious nonsense (negative, > 150).
        if (psi >= 0.0f && psi <= 150.0f) {
            pressure_sensor_->publish_state(psi);
        } else {
            ESP_LOGW(TAG, "Pressure publish skipped (out of range): %.2f psi", psi);
        }
    }

    if (!quiet && battery_sensor_ != nullptr) {
        float v = irrigoto_get_battery_mv() / 1000.0f;
        // LiFePO4 / Li-ion realistic range: 2.5 V (dead) to 4.3 V
        // (full while charging). Anything outside is a glitch.
        if (v >= 2.0f && v <= 4.5f) {
            battery_sensor_->publish_state(v);
        } else {
            ESP_LOGW(TAG, "Battery publish skipped (out of range): %.3f V", v);
        }
    }

    if (!quiet && throw_sensor_ != nullptr)
        throw_sensor_->publish_state(irrigoto_get_throw_mm());

    // Water minutes remaining: deliberately NOT gated by the quiet
    // throttle. This sensor is most useful DURING watering, which
    // is exactly when quiet is true. ESPHome dedupes identical
    // values so a static int won't spam HA -- only changes (every
    // ~minute as the estimate ticks down) actually publish.
    if (water_remaining_sensor_ != nullptr)
        water_remaining_sensor_->publish_state(
            irrigoto_get_water_minutes_remaining());

    // ── Binary sensor ─────────────────────────────────────────────────────────
    if (watering_sensor_ != nullptr)
        watering_sensor_->publish_state(irrigoto_is_watering());

    // valve_open: confirmed open/closed from the cached angle (no I2C poll).
    // Skip while state is unknown (-1, before any move/boot read); only
    // republish on change to avoid log spam and needless API traffic.
    if (valve_open_sensor_ != nullptr) {
        int vo = irrigoto_valve_is_open();
        if (vo >= 0 && vo != prev_valve_open_) {
            valve_open_sensor_->publish_state(vo == 1);
            prev_valve_open_ = vo;
        }
    }

    // ── Text sensors ──────────────────────────────────────────────────────────
    if (status_sensor_ != nullptr) {
        char buf[64];
        irrigoto_get_status(buf, sizeof(buf));
        status_sensor_->publish_state(buf);
    }
    if (zone_name_sensor_ != nullptr) {
        char buf[32];
        irrigoto_get_zone_name(buf, sizeof(buf));
        zone_name_sensor_->publish_state(buf);
    }
    // b287: cal_state is Idle during normal watering -- no need to keep
    // pushing it. Sleep reason and next_run also don't change mid-run.
    // Gated under !quiet to reduce the NVS-write rate triggered by
    // ESPHome's state-persistence on each publish.
    if (!quiet && cal_state_sensor_ != nullptr) {
        char buf[96];
        irrigoto_cal_state_str(buf, sizeof(buf));
        cal_state_sensor_->publish_state(buf);
    }
    if (!quiet && last_sleep_reason_sensor_ != nullptr) {
        char buf[32];
        irrigoto_get_last_sleep_reason(buf, sizeof(buf));
        last_sleep_reason_sensor_->publish_state(buf);
    }
    if (!quiet && next_run_sensor_ != nullptr) {
        time_t now = ::time(nullptr);
        time_t next_t = 0;
        int    next_zone = 0;
        char   buf[96];
        if (irrigoto_schedule_next_run(now, &next_t, &next_zone)) {
            struct tm lt;
            localtime_r(&next_t, &lt);
            // b422: name-first ("Combined (#1) at ..."). Schedule entries
            // carry 1-based zone numbers on the wire; the "(#id)" suffix is
            // the 0-based display id matching HA's schedule tab and cards.
            // An unnamed zone already resolves to "Zone #<id>" -- no suffix.
            char zn[32], zlabel[44];
            irrigoto_zone_name_by_id(next_zone - 1, zn, sizeof(zn));
            if (strncmp(zn, "Zone #", 6) == 0)
                snprintf(zlabel, sizeof(zlabel), "%s", zn);
            else
                snprintf(zlabel, sizeof(zlabel), "%s (#%d)", zn, next_zone - 1);
            snprintf(buf, sizeof(buf),
                     "%s at %04d-%02d-%02d %02d:%02d",
                     zlabel, lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                     lt.tm_hour, lt.tm_min);
        } else if (irrigoto_schedule_count() == 0) {
            snprintf(buf, sizeof(buf), "no schedule");
        } else if (now < 1700000000) {
            // Have entries but the system clock hasn't been set by HA's
            // time component yet — usually within seconds of API connect.
            snprintf(buf, sizeof(buf), "waiting for time sync");
        } else {
            snprintf(buf, sizeof(buf), "no upcoming runs");
        }
        next_run_sensor_->publish_state(buf);
    }
    // b287: schedule_status doesn't change during a watering -- gate it.
    if (!quiet && schedule_status_sensor_ != nullptr) {
        char buf[160];
        irrigoto_schedule_get_last_status(buf, sizeof(buf));
        schedule_status_sensor_->publish_state(buf);
    }

    // ── Schedule text mirror + change-event ──────────────────────────────────
    // Detects edits from any source — HA set_schedule, /api/schedule web UI,
    // future rain-delay services. On change we both republish the text
    // sensor (so dashboards stay in sync) and fire an HA event with the
    // text + entry count (so automations can react and stash the schedule
    // into a helper / file).
    //
    // b356: reverted to publishing the LEGACY 7-field format (no IDs /
    // last_modified) for the schedule_text text_sensor. The b355 structured
    // format with IDs was too large per-publish and contributed (with the
    // new schedule_version sensor) to the b256-family API send-buffer
    // overflow that took out the API connection. HA listeners that need
    // ID-level reconciliation now fetch GET /api/schedule via HTTP for the
    // structured payload — out-of-band of the ESPHome API.
    {
        static char raw[512];
        irrigoto_schedule_get_text(raw, sizeof(raw));
        std::string cur(raw);
        if (cur != prev_schedule_text_) {
            prev_schedule_text_ = cur;
            if (schedule_text_sensor_ != nullptr) {
                if (cur.empty()) {
                    schedule_text_sensor_->publish_state("(empty)");
                } else if (cur.size() <= 240) {
                    schedule_text_sensor_->publish_state(cur);
                } else {
                    size_t cut = cur.rfind(';', 240);
                    if (cut == std::string::npos) cut = 240;
                    int kept = 1;
                    for (size_t i = 0; i < cut; i++) if (cur[i] == ';') kept++;
                    int total = irrigoto_schedule_count();
                    char tail[40];
                    snprintf(tail, sizeof(tail), " ...(+%d more)", total - kept);
                    schedule_text_sensor_->publish_state(cur.substr(0, cut) + tail);
                }
            }
            fire_schedule_changed_event_();
        }
    }

    // ── Schedule rain/wind delay status ──────────────────────────────────────
    // The C-layer getter self-clears expired delays, so polling it here
    // also acts as the expiry trigger. On delay state change we republish
    // the text sensor and re-fire schedule_changed so HA sees the new
    // next_run window in one shot.
    {
        time_t cur_delay = irrigoto_schedule_get_delay_until();
        if (cur_delay != prev_delay_until_) {
            prev_delay_until_ = cur_delay;
            if (schedule_delay_sensor_ != nullptr) {
                if (cur_delay == 0) {
                    schedule_delay_sensor_->publish_state("off");
                } else {
                    struct tm lt;
                    localtime_r(&cur_delay, &lt);
                    char buf[48];
                    snprintf(buf, sizeof(buf),
                             "until %04d-%02d-%02d %02d:%02d",
                             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                             lt.tm_hour, lt.tm_min);
                    schedule_delay_sensor_->publish_state(buf);
                }
            }
            // Notify HA — delay changes mutate the effective next_run.
            fire_schedule_changed_event_();
        }
    }

    // ── Select state sync ─────────────────────────────────────────────────────
    if (zone_select_ != nullptr) {
        // Build the option list dynamically from storage: real zone names,
        // not generic "Zone 1..4". Update HA only when the list actually
        // changes so we don't spam state updates every 2 s.
        std::vector<std::string> new_opts;
        for (int i = 0; i < 32; i++) {
            char nm[32];
            int z = irrigoto_zone_at(i, nm, sizeof(nm));
            if (z < 0) break;
            new_opts.emplace_back(nm);
        }
        // Detect change by comparing against zone_option_storage_, which
        // is the source of truth for what's currently in the select's
        // traits (we own the string memory).
        bool changed = (new_opts.size() != zone_option_storage_.size());
        if (!changed) {
            for (size_t i = 0; i < new_opts.size(); i++) {
                if (new_opts[i] != zone_option_storage_[i]) { changed = true; break; }
            }
        }
        if (changed) {
            ESP_LOGI(TAG, "Zone select: pushing %u option(s) to HA",
                     (unsigned)new_opts.size());
            // Move ownership of the strings to the component; SelectTraits
            // will hold raw const char* pointers into these buffers.
            zone_option_storage_ = std::move(new_opts);
            FixedVector<const char *> fv;
            fv.init(zone_option_storage_.size());
            for (const auto &s : zone_option_storage_) fv.push_back(s.c_str());
            zone_select_->traits.set_options(fv);
        }
        // Publish the active zone's name as the current selection -- but
        // only if it's actually one of the options. A dangling selection
        // (zone deleted out from under it, or storage not yet ready) would
        // otherwise trip ESPHome's "Invalid option" error every loop (b420;
        // the firmware also self-heals the id in irrigoto_get_zone_name).
        char active_nm[32] = {0};
        irrigoto_get_zone_name(active_nm, sizeof(active_nm));
        if (active_nm[0]) {
            for (const auto &o : zone_option_storage_) {
                if (o == active_nm) { zone_select_->publish_state(active_nm); break; }
            }
        }
    }
    if (mode_select_ != nullptr) {
        int m = irrigoto_get_mode();  // 0=Pulse, 1=Gentle, 2=Smooth, 3=Serpentine (b431)
        const char *labels[] = {"Pulse", "Gentle", "Smooth", "Serpentine"};
        if (m >= 0 && m <= 3)
            mode_select_->publish_state(labels[m]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HA service handlers
// ─────────────────────────────────────────────────────────────────────────────

void IrrigotoComponent::start_watering(int zone, int mode, int duration_s) {
    ESP_LOGI(TAG, "HA start_watering: zone=%d mode=%d duration=%ds", zone, mode, duration_s);
    irrigoto_start_watering(zone, mode, duration_s);
}

float IrrigotoComponent::read_pressure_now() {
    float psi = irrigoto_read_pressure_now();
    ESP_LOGI(TAG, "HA read_pressure_now -> %.3f PSI", psi);
    if (psi >= 0.0f && pressure_sensor_ != nullptr)
        pressure_sensor_->publish_state(psi);
    return psi;
}

void IrrigotoComponent::set_led(const std::string &color) {
    ESP_LOGI(TAG, "HA set_led: %s", color.c_str());
    irrigoto_set_led(color.c_str());
}

void IrrigotoComponent::set_auto_sleep(bool enabled) {
    ESP_LOGI(TAG, "HA set_auto_sleep: %s", enabled ? "on" : "off");
    irrigoto_set_auto_sleep_enabled(enabled);
}

bool IrrigotoComponent::get_auto_sleep() {
    return irrigoto_get_auto_sleep_enabled();
}

void IrrigotoComponent::sleep_now(int duration_s) {
    ESP_LOGI(TAG, "HA sleep_now: %ds", duration_s);
    irrigoto_sleep_now(duration_s > 0 ? (uint32_t)duration_s : 0);
}

void IrrigotoComponent::valve_goto(float target_deg) {
    ESP_LOGI(TAG, "HA valve_goto: %.1f", target_deg);
    irrigoto_valve_goto(target_deg);
}

void IrrigotoComponent::nozzle_goto(float target_deg) {
    ESP_LOGI(TAG, "HA nozzle_goto: %.1f", target_deg);
    irrigoto_nozzle_goto(target_deg);
}

void IrrigotoComponent::aim_valve(float bearing_deg, float valve_deg) {
    ESP_LOGI(TAG, "HA aim_valve: bearing=%.1f valve=%.1f", bearing_deg, valve_deg);
    irrigoto_aim_valve(bearing_deg, valve_deg);
}

void IrrigotoComponent::aim_throw(float bearing_deg, float throw_mm) {
    ESP_LOGI(TAG, "HA aim_throw: bearing=%.1f throw=%.0f mm", bearing_deg, throw_mm);
    irrigoto_aim_throw(bearing_deg, throw_mm);
}

void IrrigotoComponent::water_arc(float throw_mm, float arc_start_deg,
                                  float arc_end_deg, float speed_dps,
                                  const std::string &direction) {
    ESP_LOGI(TAG, "HA water_arc: throw=%.0fmm arc=%.1f->%.1f speed=%.1f dps dir=%s",
             throw_mm, arc_start_deg, arc_end_deg, speed_dps,
             direction.empty() ? "shortest" : direction.c_str());
    irrigoto_water_arc(throw_mm, arc_start_deg, arc_end_deg, speed_dps,
                       direction.c_str());
}

float IrrigotoComponent::get_valve_deg() {
    return irrigoto_get_valve_deg();
}

float IrrigotoComponent::get_nozzle_deg() {
    return irrigoto_get_nozzle_deg();
}

void IrrigotoComponent::stop_and_close() {
    ESP_LOGI(TAG, "HA stop_and_close");
    irrigoto_stop_and_close();
}

void IrrigotoComponent::valve_close() {
    ESP_LOGI(TAG, "HA valve_close");
    irrigoto_valve_close();
}

void IrrigotoComponent::valve_open() {
    ESP_LOGI(TAG, "HA valve_open");
    irrigoto_valve_open();
}

void IrrigotoComponent::cal_pressure_start() {
    ESP_LOGI(TAG, "HA cal_pressure_start");
    irrigoto_cal_pressure_start();
}

void IrrigotoComponent::cal_pressure_throw(float throw_mm) {
    ESP_LOGI(TAG, "HA cal_pressure_throw: %.0f mm", throw_mm);
    irrigoto_cal_pressure_throw(throw_mm);
}

void IrrigotoComponent::cal_pressure_cancel() {
    ESP_LOGI(TAG, "HA cal_pressure_cancel");
    irrigoto_cal_pressure_cancel();
}

void IrrigotoComponent::cal_nozzle_start() {
    ESP_LOGI(TAG, "HA cal_nozzle_start");
    irrigoto_cal_nozzle_start();
}

std::string IrrigotoComponent::get_boot_diag_summary() {
    char buf[200];
    irrigoto_boot_diag_summary(buf, sizeof(buf));
    return std::string(buf);
}

uint32_t IrrigotoComponent::get_inactivity_minutes() {
    return irrigoto_get_inactivity_minutes();
}

void IrrigotoComponent::set_inactivity_minutes(uint32_t m) {
    ESP_LOGI(TAG, "HA set_inactivity_minutes: %u", (unsigned)m);
    irrigoto_set_inactivity_minutes(m);
}

uint32_t IrrigotoComponent::get_sleep_duration_s() {
    return irrigoto_get_sleep_duration_s();
}

void IrrigotoComponent::set_sleep_duration_s(uint32_t s) {
    ESP_LOGI(TAG, "HA set_sleep_duration_s: %u", (unsigned)s);
    irrigoto_set_sleep_duration_s(s);
}

uint32_t IrrigotoComponent::get_dwell_timeout_s() {
    return irrigoto_get_dwell_timeout_s();
}

void IrrigotoComponent::set_dwell_timeout_s(uint32_t s) {
    ESP_LOGI(TAG, "HA set_dwell_timeout_s: %u", (unsigned)s);
    irrigoto_set_dwell_timeout_s(s);
}

void IrrigotoComponent::set_schedule(const std::string &text) {
    ESP_LOGI(TAG, "HA set_schedule: %u chars", (unsigned)text.size());
    bool ok = irrigoto_schedule_set_text(text.c_str());
    if (!ok) {
        // Could be a parse error or a validation rejection (overlapping
        // entries on shared days). The C layer logs the specific cause
        // — see "schedule: parse failed" or "Schedule rejected" in the
        // device log. Either way, prior schedule is preserved.
        ESP_LOGW(TAG, "HA set_schedule: rejected, prior schedule kept");
    }
}

void IrrigotoComponent::sync_schedule(const std::string &text) {
    ESP_LOGI(TAG, "HA sync_schedule: %u chars", (unsigned)text.size());
    bool ok = irrigoto_schedule_sync_text(text.c_str());
    if (!ok) {
        ESP_LOGW(TAG, "HA sync_schedule: rejected, prior schedule kept");
    }
}

void IrrigotoComponent::clear_schedule() {
    ESP_LOGI(TAG, "HA clear_schedule");
    irrigoto_schedule_clear();
}

void IrrigotoComponent::delay_schedule_hours(int hours) {
    ESP_LOGI(TAG, "HA delay_schedule_hours: %d", hours);
    if (hours <= 0) {
        irrigoto_schedule_clear_delay();
    } else {
        irrigoto_schedule_set_delay_hours((uint32_t)hours);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IrrigotoZoneSelect / IrrigotoModeSelect — handle user changes from HA
// ─────────────────────────────────────────────────────────────────────────────

void IrrigotoZoneSelect::control(const std::string &value) {
    // Map the picked label back to a zone index by looking up each
    // configured zone's name and matching. Names can be anything the
    // user typed, so we can't parse a number out of them.
    int matched_zone = -1;
    for (int i = 0; i < 32; i++) {
        char nm[32];
        int z = irrigoto_zone_at(i, nm, sizeof(nm));
        if (z < 0) break;
        if (value == nm) { matched_zone = z; break; }
    }
    if (matched_zone > 0) {
        ESP_LOGI(TAG, "HA zone select -> %d (%s)", matched_zone, value.c_str());
        irrigoto_set_zone(matched_zone);
    } else {
        // Stale-option fallbacks in case HA still has an old option set.
        // "Zone #N" (b421+, N = 0-based storage id) takes priority; the
        // legacy 1-based "Zone N" form is kept for pre-b421 option sets.
        // sscanf("Zone %d") can't match "Zone #N" ('#' isn't a digit), so
        // the two parses are unambiguous.
        int z = 0;
        if (sscanf(value.c_str(), "Zone #%d", &z) == 1 && z >= 0)
            irrigoto_set_zone(z + 1);
        else if (sscanf(value.c_str(), "Zone %d", &z) == 1 && z >= 1)
            irrigoto_set_zone(z);
    }
    publish_state(value);
}

void IrrigotoModeSelect::control(const std::string &value) {
    int m = -1;
    if (value == "Pulse")        m = 0;
    else if (value == "Gentle")  m = 1;
    else if (value == "Smooth")  m = 2;
    else if (value == "Serpentine")   m = 3;   // b431
    if (m >= 0) irrigoto_set_mode(m);
    publish_state(value);
}

// ─────────────────────────────────────────────────────────────────────────────
// HA completion events
// ─────────────────────────────────────────────────────────────────────────────

static std::string f2s(float v, int decimals) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return std::string(buf);
}
static std::string i2s(int v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", v);
    return std::string(buf);
}

void IrrigotoComponent::fire_watering_complete_event_() {
    char zone_name[32]={0}, mode_label[24]={0};
    irrigoto_last_water_zone_name(zone_name, sizeof(zone_name));
    irrigoto_last_water_mode_label(mode_label, sizeof(mode_label));

    std::map<std::string, std::string> data;
    data["zone"]            = i2s(irrigoto_last_water_zone());
    data["zone_name"]       = zone_name;
    data["mode"]            = i2s(irrigoto_last_water_mode_code());
    data["mode_label"]      = mode_label;
    data["duration_s"]      = f2s(irrigoto_last_water_duration_s(), 1);
    data["num_rings"]       = i2s(irrigoto_last_water_num_rings());
    data["avg_psi"]         = f2s(irrigoto_last_water_avg_psi(), 2);
    data["max_throw_mm"]    = f2s(irrigoto_last_water_max_throw_mm(), 0);
    data["total_depth_mm"]  = f2s(irrigoto_last_water_total_depth_mm(), 2);
    data["area_m2"]         = f2s(irrigoto_last_water_area_m2(), 2);
    data["zone_area_m2"]    = f2s(irrigoto_last_water_zone_area_m2(), 2);
    data["volume_l"]        = f2s(irrigoto_last_water_volume_l(), 2);
    // Headline target-vs-actual fields (b276+). HA flags runs where
    // actual_avg_depth_mm falls below e.g. 80% of target_depth_mm.
    data["target_depth_mm"]      = f2s(irrigoto_last_water_target_depth_mm(), 3);
    data["actual_avg_depth_mm"]  = f2s(irrigoto_last_water_actual_avg_depth_mm(), 3);
    data["score"]           = f2s(irrigoto_last_water_score(), 2);
    // True coverage = (sprayed ∩ polygon) / polygon, 0..100. Honest answer
    // to "did my zone get watered" — see irrigoto_last_water_polygon_coverage_pct().
    data["polygon_coverage_pct"] = f2s(irrigoto_last_water_polygon_coverage_pct(), 1);
    // b281: Run-level supply pressure summary (pre-valve, back-computed from
    // per-ring avg_psi using zone cal's f(valve_deg)). On well-fed systems
    // these reveal pump cycling: supply_psi_max ≈ tank cut-out pressure,
    // supply_psi_min ≈ tank cut-in or deepest drawdown during the run.
    // A wide max-min spread means the supply was cycling actively; a
    // supply_psi_avg far below the cal-time supply means rings under-delivered.
    // All three are 0.0 if the zone has insufficient cal data to back-compute.
    data["supply_psi_min"] = f2s(irrigoto_last_water_supply_psi_min(), 2);
    data["supply_psi_max"] = f2s(irrigoto_last_water_supply_psi_max(), 2);
    data["supply_psi_avg"] = f2s(irrigoto_last_water_supply_psi_avg(), 2);
    // b282: Count of rings flagged as supply-limited (under-throw correlated
    // with depleted supply). Each is a candidate the b283 cleanup pass would
    // re-water once pump cycle recovers. 0 means the run was clean.
    data["rings_supply_limited"] = i2s(irrigoto_last_water_rings_supply_limited());
    char status_buf[24]={0};
    irrigoto_last_water_status_str(status_buf, sizeof(status_buf));
    data["status"]          = status_buf;
    data["fw_build"]        = i2s(FW_BUILD);

    std::string event_name = "esphome." + App.get_name() + "_watering_complete";
    ESP_LOGI(TAG, "Firing HA event: %s (zone=%s status=%s)",
             event_name.c_str(), zone_name, data["status"].c_str());
    fire_homeassistant_event(event_name, data);
}

void IrrigotoComponent::fire_cal_complete_event_(int prev_state, int cur_state) {
    // prev_state was active (1=PressureScan, 2=AwaitingThrow, 3=NozzleScan).
    // cur_state is now terminal (0=Idle, 5=Done, 6=Error).
    const char *type = (prev_state == 3) ? "nozzle" : "pressure";
    const char *status;
    switch (cur_state) {
        case 5:  status = "completed"; break;
        case 6:  status = "failed";    break;
        default: status = "canceled";  break;
    }
    std::map<std::string, std::string> data;
    data["type"]     = type;
    data["status"]   = status;
    data["fw_build"] = i2s(FW_BUILD);

    std::string event_name = "esphome." + App.get_name() + "_cal_complete";
    ESP_LOGI(TAG, "Firing HA event: %s (type=%s status=%s)",
             event_name.c_str(), type, status);
    fire_homeassistant_event(event_name, data);
}

void IrrigotoComponent::fire_schedule_changed_event_() {
    // b356: emit only the legacy 7-field text + schedule_version. b355
    // additionally carried text_full (~2KB of structured per-entry data
    // with IDs); together with the new schedule_version sensor that
    // bloated the API send buffer enough to cross the b256 buffer-
    // overflow threshold under load. HA listeners that need by-id
    // reconciliation now fetch GET /api/schedule (HTTP) for the
    // structured payload — out-of-band of the ESPHome API.
    static char raw_legacy[1024];
    irrigoto_schedule_get_text(raw_legacy, sizeof(raw_legacy));
    char status[160] = {0};
    irrigoto_schedule_get_last_status(status, sizeof(status));

    std::map<std::string, std::string> data;
    data["text"]             = raw_legacy;
    data["count"]            = i2s(irrigoto_schedule_count());
    data["status"]           = status;
    data["schedule_version"] = i2s((int)irrigoto_schedule_version());
    // Rain/wind delay: 0 = no delay, otherwise unix epoch when the delay
    // expires. Keep it in epoch form so HA listeners can format in their
    // own TZ without parsing a string.
    data["delay_until"] = i2s((int)irrigoto_schedule_get_delay_until());
    data["fw_build"]    = i2s(FW_BUILD);

    std::string event_name = "esphome." + App.get_name() + "_schedule_changed";
    ESP_LOGI(TAG, "Firing HA event: %s (count=%s v=%s)",
             event_name.c_str(), data["count"].c_str(),
             data["schedule_version"].c_str());
    fire_homeassistant_event(event_name, data);
}

}  // namespace irrigoto
}  // namespace esphome

// ─────────────────────────────────────────────────────────────────────────────
// b293: C-callable bridges to ESPHome's WiFi component
// ─────────────────────────────────────────────────────────────────────────────
// b292's direct esp_wifi_stop() didn't stick because ESPHome's WiFi component
// runs its own state machine and re-initialized WiFi within milliseconds.
// The proper fix is to tell the WiFi *component* to enter DISABLED state, which
// suppresses its reconnect logic AND calls the IDF teardown internally.
// These bridges expose the C++ disable()/enable() to our C watering code.
extern "C" void irrigoto_wifi_disable_component(void) {
    if (esphome::wifi::global_wifi_component != nullptr) {
        esphome::wifi::global_wifi_component->disable();
    }
}

extern "C" void irrigoto_wifi_enable_component(void) {
    if (esphome::wifi::global_wifi_component != nullptr) {
        esphome::wifi::global_wifi_component->enable();
    }
}

// b396/b397: AP setup mode. clear_sta() removes the station config, then
// start() re-runs WiFiComponent's boot decision: with no STA it falls into the
// `else if (has_ap()) setup_ap_config_()` branch, which calls wifi_mode_({},true)
// and brings the AP up immediately (clear_sta() alone only removes the config --
// it doesn't drop the live link or trigger the AP, which is why b396 never
// showed an AP). clear_sta() is RAM-only and start() reloads the compiled creds
// on the next BOOT, so a reboot/power-cycle reverts to the home network.
extern "C" void irrigoto_wifi_ap_mode(void) {
    auto *w = esphome::wifi::global_wifi_component;
    if (w != nullptr) {
        w->clear_sta();
        w->start();
    }
}
