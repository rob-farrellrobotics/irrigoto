#include <stdio.h>
#include "irrigoto_types.h"
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <strings.h>     // strcasecmp (used in irrigoto_set_led)
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>        // for scheduler: time(), localtime()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_random.h"   // chase mode state-machine PRNG
#include "esp_sleep.h"
#include "esp_system.h"   // esp_reset_reason() for boot_diag
#include "driver/uart.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "esp_http_server.h"
#include "fw_version.h"
#include "storage.h"
#ifdef ESPHOME_COMPONENT
#include "irrigoto_api.h"  // visible at top so callsites above the impl block compile
#endif

static const char *TAG = "irrigoto";

// ── GPIO assignments ──────────────────────────────────────────────────────────
#define GPIO_3V3SEN     GPIO_NUM_4
#define GPIO_9V_EN      GPIO_NUM_18
#define GPIO_K          GPIO_NUM_13

#define GPIO_VFWD       GPIO_NUM_22
#define GPIO_VREV       GPIO_NUM_25
#define GPIO_NFWD       GPIO_NUM_26
#define GPIO_NREV       GPIO_NUM_27

#define GPIO_WAKE_HALL  GPIO_NUM_23

// ── ADC channels ──────────────────────────────────────────────────────────────
// GPIO36 = ADC1_CH0, GPIO32 = ADC1_CH4, GPIO34 = ADC1_CH6, GPIO35 = ADC1_CH7
#define ADC_CH_VBATT    ADC_CHANNEL_0   // GPIO36 (SENSOR_VP) - battery voltage divider
#define ADC_CH_VCUR     ADC_CHANNEL_4   // GPIO32 - valve current
#define ADC_CH_PCUR     ADC_CHANNEL_6   // GPIO34 - pump current (unused)
#define ADC_CH_CHARGE   ADC_CHANNEL_3   // GPIO39 (SENSOR_VN) - solar/charge current
#define ADC_CH_NCUR     ADC_CHANNEL_7   // GPIO35 - nozzle current

// ── I2C addresses ─────────────────────────────────────────────────────────────
#define ADDR_AS5600L    0x40
#define ADDR_AS5600     0x36
#define ADDR_MPRLS      0x08    // write: trigger measurement
#define ADDR_TCA6408A   0x20

// ── AS5600 registers ──────────────────────────────────────────────────────────
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_AGC          0x1A
#define AS5600_STATUS_MD        (1 << 5)
#define AS5600_STATUS_ML        (1 << 4)
#define AS5600_STATUS_MH        (1 << 3)

// ── MPRLS ─────────────────────────────────────────────────────────────────────
#define MPRLS_CMD_START         0xAA
#define MPRLS_STATUS_BUSY       0x20
#define MPRLS_PSI_MAX           25.0f
#define MPRLS_COUNT_MIN         0x19999Au
#define MPRLS_COUNT_MAX         0xE66666u

// ── TCA6408A registers ────────────────────────────────────────────────────────
#define TCA6408A_REG_OUTPUT     0x01
#define TCA6408A_REG_CONFIG     0x03

// TCA6408A LED bit mapping (confirmed from LED test menu):
//
//   Bit 0 = Red   (active LOW  -- 0=on, 1=off)
//   Bit 1 = Blue  (active HIGH -- 1=on, 0=off)
//   Bit 2 = Green (active HIGH -- 1=on, 0=off)
//   Bits 3-7 = auxiliary / pump valve controls (unpopulated in this unit)
//
// DUAL-DRIVE ARCHITECTURE FOR RED LED (confirmed by hardware testing):
//   The red LED has two independent drive paths that OR together:
//   1. BQ25504 charger IC status pin -- lights red autonomously whenever
//      solar charging is active, even during ESP32 deep sleep.
//      This is hardware-only, no firmware involvement required.
//      CONFIRMED: red LED lights during deep sleep when solar connected.
//   2. TCA6408A bit 0 (active LOW) -- firmware-controllable.
//      Can be used for error states, low battery warnings, etc.
//   Either path alone lights the LED; both active = same result.
//   Before sleep, set TCA to LED_OFF (0xF9) so bit0=HIGH, releasing
//   red LED control to the BQ25504 charger exclusively.
//
//   Green and Blue are firmware-only (TCA6408A bits 1 and 2).
//
// Base "off" state: bit0=HIGH, bit1=LOW, bit2=LOW = 0xF9
// To add a colour: set its bit (HIGH for B/G, LOW for R)
#define LED_OFF         0xF9   // R=off(HIGH) G=off(LOW)  B=off(LOW)
#define LED_RED         0xF8   // R=on(LOW)   G=off(LOW)  B=off(LOW)
#define LED_GREEN       0xFD   // R=off(HIGH) G=on(HIGH)  B=off(LOW)
#define LED_BLUE        0xFB   // R=off(HIGH) G=off(LOW)  B=on(HIGH)
#define LED_YELLOW      0xFC   // R=on(LOW)   G=on(HIGH)  B=off(LOW)
#define LED_CYAN        0xFF   // R=off(HIGH) G=on(HIGH)  B=on(HIGH)
#define LED_PURPLE      0xFA   // R=on(LOW)   G=off(LOW)  B=on(HIGH)
#define LED_WHITE       0xFE   // R=on(LOW)   G=on(HIGH)  B=on(HIGH)

// ── ADC / battery ─────────────────────────────────────────────────────────────
#define VBATT_DIVIDER_RATIO     2.0f    // adjust after measuring resistors near J5
#define VBATT_MIN_MV            3300
#define ADC_SAMPLES             16

// ── UART ─────────────────────────────────────────────────────────────────────
#define UART_BUF_SIZE           256

// ── INA4180A3: gain=100, Rshunt=50mΩ → I(mA) = Vout_mV * 0.2 ────────────────
#define CURRENT_MA(mv)          ((mv) * 0.2f)

// ── State ─────────────────────────────────────────────────────────────────────
// Forward declarations
static void tca_led_set(uint8_t val);

static int s_pass = 0;
static int s_fail = 0;
static bool s_sensor_rail = false;
static bool s_motor_rail  = false;
static bool s_tca_outputs  = false;  // true once TCA6408A config set to outputs
static TickType_t          s_last_activity     = 0;  // tick count of last user input
static volatile TickType_t s_last_web_req_tick = 0;  // tick of last zone web HTTP request
static volatile bool s_ota_in_progress = false;  // true during OTA -- suppress sleep
static bool          s_sleep_disabled  = false;  // user-toggled; persisted in NVS as "pm_disable"
// Configurable inactivity threshold (ms) and post-sleep wake timer (s).
// Loaded from NVS at boot; defaults match the legacy compile-time constants.
static uint32_t      s_inactivity_ms   = (5u * 60u * 1000u);  // NVS "pm_inact_s"
static uint32_t      s_sleep_dur_s     = 300u;                 // NVS "pm_dur_s"
// Nozzle dwell watchdog timeout (ms). The hard ceiling on how long the
// nozzle can sit near one bearing with the valve open before a
// nozzle_fault is declared. Loaded from NVS at boot.
static uint32_t      s_dwell_timeout_ms = 30000u;               // NVS "pm_dwell_s"
// Last sleep reason published to HA on boot (text_sensor "Last sleep reason").
// Set just before each sleep entry, persisted in NVS as "pm_reason".
static char          s_last_sleep_reason[32] = "";
// Web UI theme. true=dark (legacy default), false=light. NVS key "ui_theme".
static bool          s_theme_dark      = true;
static int s_nozzle_last_dir = 0;  // +1=CW, -1=CCW, 0=unknown
static int s_valve_last_dir  = 0;  // +1=opening, -1=closing, 0=unknown
// Last known valve angle (deg), updated on every encoder read / move so the
// HA valve_open binary_sensor can report confirmed state without an I2C poll
// (which would keep the sensor rail powered and block deep-sleep). -1 = unknown.
static float s_valve_deg_cached = -1.0f;
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t         s_cal = NULL;

// ── WiFi state ────────────────────────────────────────────────────────────────
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
static int  s_wifi_retry_count = 0;
#define WIFI_MAX_RETRY  10
static bool s_wifi_connected = false;
static char s_wifi_ip[16]    = "";    // dotted-decimal IP, empty when disconnected
static int  wifi_get_rssi(void);
static char s_device_name[32] = {0};
static esp_netif_t *s_wifi_netif = NULL;

// ── TCP terminal state ───────────────────────────────────────────────────────
#define TCP_PORT        3333
#define OTA_HTTP_PORT   8080
static int s_tcp_client_fd = -1;
static volatile bool s_water_abort   = false; // set by terminal q or web cancel
// Cached MPRLS reading written by every mprls_read() / mprls_read_quiet().
// Used by irrigoto_get_pressure_psi() to serve HA / web without competing
// for the I2C bus mutex during active watering / cal motion control.
static volatile float s_cached_psi   = 0.0f;
static int           s_web_water_mode  = 0;    // 0=idle, 1-4=metered, 5-6=gentle, 7=smooth, 50=chase, 99=demo
static volatile int  s_water_cleanup_pass = 0; // 0=not in cleanup, N=cleanup pass N
// b311: chase mode -- entertainment watering for dogs/kids. Lissajous-wander
// (bearing, throw) inside the zone polygon for N minutes (1..10). No coverage
// guarantees; the goal is a slowly-moving stream the chaser can dodge.
#define WATER_MODE_CHASE       50
#define CHASE_MIN_MINUTES      1
#define CHASE_MAX_MINUTES      10
static int           s_chase_duration_min = 0; // 1..10; only valid when s_web_water_mode == WATER_MODE_CHASE
// EXP (2026-06-06): lash experiment (reversal dead-time / hysteresis). Runs in
// its own PRO_CPU task like watering; mutually exclusive with watering + cal.
static volatile bool s_exp_running = false;
static volatile int  s_exp_section = 0;        // 1 = A (dry dead-time), 2 = B (wet hysteresis)
// Depth target for the current web/schedule run, in eighths of an inch
// (1..8 = 1/8".. 1"). 0 = use the legacy digit-encoded depth. Set on each
// start path; consumed (reset to 0) by the water task.
static int           s_web_water_depth_eighths = 0;
// b423: serpentine-mode web params, consumed once by phase_water_zone like
// depth_eighths. dps 0 = default; dry runs the motion with the valve held.
static float         s_web_serpentine_dps = 0.0f;
static bool          s_web_serpentine_dry = false;
// b317-b323: spiral watering mode (entertainment / fun spiral around zone
// centroid). Removed in b324. Archived under milestone/esphome-phase2-b323-spiral.
// b287: "Watering Quiet" mode flag. Set true at the start of a smooth/gentle
// watering run; cleared a few seconds after the run's final LFS write completes.
// While true, irrigoto.cpp::loop() suppresses high-rate sensor publishes
// (pressure, battery, throw) so the WiFi/ESPHome-NVS pipe stays calm during
// the run. This reduces the WiFi-PHY + SPI-flash race that has been crashing
// the device in b283-b286 on long smooth-mode runs.
//
// Progress sensors (watering status, current zone, watering binary sensor)
// continue to publish at the normal cadence so HA still sees the run live.
static volatile bool s_watering_quiet  = false;
static uint16_t      s_web_zone_id     = 0;    // zone id being edited in setup page
static char          s_web_zone_name[32] = {0}; // name of zone being edited
static bool          s_jog_web_mode    = false; // true when jog cal triggered from web
// ── Binary watering log format ─────────────────────────────────────────────
// Stored as /lfs/water/water_NNN_name.wbin  (magic header + packed rows).
// The /csv HTTP endpoint converts to text CSV on the fly; the web UI is unchanged.
#define WATER_BIN_MAGIC  0x574F5442U   // 'W','O','T','B'
typedef struct __attribute__((packed)) {
    uint16_t time_ds;              // elapsed time × 10  (0.1 s resolution)
    uint8_t  ring;                 // ring index (0-based)
    uint8_t  arc;                  // arc index
    uint16_t sector;               // sector number
    uint16_t nozzle_deg_target_d;  // degrees × 10
    uint16_t nozzle_deg_actual_d;  // degrees × 10
    uint16_t valve_deg_target_d;   // degrees × 10
    uint16_t valve_deg_actual_d;   // degrees × 10
    uint16_t throw_mm_target;      // mm
    uint16_t throw_mm_actual;      // mm
    uint16_t psi_target_c;         // PSI × 100
    uint16_t psi_actual_c;         // PSI × 100
    int16_t  target_x;             // mm  (throw × sin(bearing))
    int16_t  target_y;             // mm  (throw × cos(bearing))
    int16_t  actual_x;             // mm
    int16_t  actual_y;             // mm
    uint8_t  pass_type;
    uint8_t  active;
} water_row_t;   // 32 bytes

// ── Smooth-mode aggregate accumulator ─────────────────────────────────────
// Instead of one wbin record per 2°-sample per pass (which can exceed 1MB for
// a large zone with many passes), accumulate measurements into per-(ring,sector)
// buckets during the run and write one summary record per sector at run end.
// Max file size: WATER_RUN_MAX_RINGS × WATER_SECTORS × 32 = ~41 KB regardless
// of pass count.  pass_type = 0xFF marks aggregate records in the .wbin.
typedef struct {
    uint32_t sum_actual_deg_d;  // sum of (actual_nozzle_deg × 10) for averaging
    uint32_t sum_actual_throw;  // sum of actual_throw_mm for averaging
    uint32_t sum_psi_c;         // sum of (actual_psi × 100) for averaging
    uint16_t count;             // number of 2°-samples accumulated this sector
} water_sector_accum_t;
// 14 bytes × 36 rings × 36 sectors ≈ 18 KB static
// (36 = WATER_SECTORS, defined later in the file -- use literal here to avoid forward-ref)
static water_sector_accum_t s_smooth_accum[WATER_RUN_MAX_RINGS][36];
static bool                 s_smooth_accum_mode = false;

// Write one binary row to the open water log file.
static void water_csv_write_row(FILE *f, float t_s, int ring_, int arc_,
    int sector_, float nozzle_target, float nozzle_actual,
    float valve_target, float valve_actual,
    float throw_target, float throw_actual,
    float psi_target, float psi_actual,
    float bearing_rad, uint8_t pass_type)
{
    // Smooth aggregate mode: accumulate into per-(ring,sector) bucket, don't write.
    if (s_smooth_accum_mode) {
        if ((unsigned)ring_ < WATER_RUN_MAX_RINGS && (unsigned)sector_ < 36) {
            water_sector_accum_t *a = &s_smooth_accum[ring_][sector_];
            a->sum_actual_deg_d += (uint32_t)(nozzle_actual * 10.0f + 0.5f);
            a->sum_actual_throw += (uint32_t)(throw_actual           + 0.5f);
            a->sum_psi_c        += (uint32_t)(psi_actual   * 100.0f + 0.5f);
            if (a->count < 65535) a->count++;
        }
        return;
    }
    if (!f) return;
    float sx = sinf(bearing_rad), cx = cosf(bearing_rad);
    water_row_t r = {
        .time_ds              = (uint16_t)fminf(t_s * 10.0f + 0.5f,   65535.0f),
        .ring                 = (uint8_t)ring_,
        .arc                  = (uint8_t)arc_,
        .sector               = (uint16_t)sector_,
        .nozzle_deg_target_d  = (uint16_t)fminf(nozzle_target * 10.0f + 0.5f, 65535.0f),
        .nozzle_deg_actual_d  = (uint16_t)fminf(nozzle_actual * 10.0f + 0.5f, 65535.0f),
        .valve_deg_target_d   = (uint16_t)fminf(valve_target  * 10.0f + 0.5f, 65535.0f),
        .valve_deg_actual_d   = (uint16_t)fminf(valve_actual  * 10.0f + 0.5f, 65535.0f),
        .throw_mm_target      = (uint16_t)fminf(throw_target  + 0.5f,  65535.0f),
        .throw_mm_actual      = (uint16_t)fminf(throw_actual  + 0.5f,  65535.0f),
        .psi_target_c         = (uint16_t)fminf(psi_target  * 100.0f + 0.5f, 65535.0f),
        .psi_actual_c         = (uint16_t)fminf(psi_actual  * 100.0f + 0.5f, 65535.0f),
        .target_x  = (int16_t)fmaxf(-32767.0f, fminf(throw_target * sx, 32767.0f)),
        .target_y  = (int16_t)fmaxf(-32767.0f, fminf(throw_target * cx, 32767.0f)),
        .actual_x  = (int16_t)fmaxf(-32767.0f, fminf(throw_actual * sx, 32767.0f)),
        .actual_y  = (int16_t)fmaxf(-32767.0f, fminf(throw_actual * cx, 32767.0f)),
        .pass_type = pass_type,
        .active    = 1,
    };
    fwrite(&r, sizeof(r), 1, f);
}

static bool          s_web_zone_is_new = false; // true when creating a new zone
static int           s_water_est_min  = 0;    // estimated minutes, shown in web UI
// b362: live tick-down anchor so HA "minutes remaining" sensor decreases
// continuously between explicit pass-boundary updates. s_eta_anchor_secs is
// the seconds-remaining value at s_eta_anchor_tick; the getter back-computes
// "now" from wall clock. tick=0 means "no anchor set" -> return s_water_est_min
// verbatim (used between runs and during the seed window before phase_water_zone
// has computed a real estimate).
static float         s_eta_anchor_secs = 0.0f;
static TickType_t    s_eta_anchor_tick = 0;
static FILE         *s_water_csv_f    = NULL; // watering CSV log (/lfs/water/water_000.csv)
static uint8_t       s_csv_pass_type  = 1;    // 1=initial pass, 2+=cleanup pass
static bool          s_water_detail_log = false; // when true: write per-pass rows (disables smooth aggregate)
// b288: in smooth-aggregate mode the wbin file is created at run-END, not at start,
// so the file is never held open during the loop (avoids LittleFS metadata-relocation
// races against WiFi PHY work that caused the b285-b287 mid-run crashes).
// Path is computed once at run-start and stashed here so the end-of-run flush can
// open it without re-doing storage_zone_load (a flash read we want to keep out of
// the watering loop's tail).
static char          s_smooth_wbin_path[64] = {0};

#define CLEANUP_THRESHOLD   0.75f  // rings below 75% of target get a cleanup pass
#define CLEANUP_MAX_PASSES     2   // maximum cleanup passes per watering
// Gentle mode: spread the target depth across many shallow passes (seed-safe).
// Per-pass depth is the orifice-flow-model target at min_dps; pass count is an
// adaptive upper bound (loop terminates per-ring once cumulative >= target).
#define GENTLE_PER_PASS_DEPTH_MM  0.635f   // 1/8" / 5 -- gradual application
#define GENTLE_MAX_PASSES            20    // cap; adaptive termination usually hits 5-12
static float         s_last_depth_mm  = 3.175f; // target depth from last watering
static uint16_t      s_water_zone_id  = 0;    // which zone is currently watering
static SemaphoreHandle_t s_tcp_mutex = NULL;

// ── Logging helpers ───────────────────────────────────────────────────────────

// Send a string to the TCP client (if connected)
static void tcp_send(const char *data, int len)
{
    if (s_tcp_client_fd < 0) return;
    if (s_tcp_mutex && xSemaphoreTake(s_tcp_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        send(s_tcp_client_fd, data, len, MSG_DONTWAIT);
        xSemaphoreGive(s_tcp_mutex);
    }
}

// Custom vprintf: writes to UART stdout AND to TCP client
static int dual_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len > 0) {
        // Always write to UART
        fwrite(buf, 1, (len < (int)sizeof(buf)) ? len : (int)sizeof(buf) - 1, stdout);
        fflush(stdout);
        // Also send to TCP client
        tcp_send(buf, (len < (int)sizeof(buf)) ? len : (int)sizeof(buf) - 1);
    }
    return len;
}
#define PASS(fmt, ...) do { ESP_LOGI(TAG, "  [PASS] " fmt, ##__VA_ARGS__); s_pass++; } while(0)
#define FAIL(fmt, ...) do { ESP_LOGE(TAG, "  [FAIL] " fmt, ##__VA_ARGS__); s_fail++; } while(0)
#define STEP(fmt, ...) ESP_LOGI(TAG, "\n-- " fmt " --", ##__VA_ARGS__)
#define INFO(fmt, ...) ESP_LOGI(TAG, "  " fmt, ##__VA_ARGS__)
#define WARN(fmt, ...) ESP_LOGW(TAG, "  " fmt, ##__VA_ARGS__)
#define TOUCH_ACTIVITY()   do { s_last_activity = xTaskGetTickCount(); } while(0)
// WEB_TOUCH: called from every zone web handler.
// Updates both inactivity timer AND web-client-presence timestamp.
// Sleep is suppressed for 30 seconds after the last WEB_TOUCH call,
// matching the 900ms poll interval with a large safety margin.
#define WEB_TOUCH() do { \
    s_last_activity    = xTaskGetTickCount(); \
    s_last_web_req_tick = xTaskGetTickCount(); \
} while(0)
#define WEB_CLIENT_TIMEOUT_MS 30000   // 30s: much longer than 900ms poll

// Power management
#define INACTIVITY_SLEEP_MS   (5 * 60 * 1000)  // 5 minutes
#define BATT_MIN_VOLTAGE_V        3.6f         // sleep forever below this (no motor activity)
#define BATT_VALVE_SAFE_VOLTAGE_V 3.8f         // require this much margin to attempt the wake-time valve-close check. Lowered from 3.9V (b347) -- water safety > controller safety, and a single ADC sample at boot can read lower than steady-state if the battery hasn't recovered from prior watering load.

// Zone perimeter
#define ZONE_ROTATE_STEP_DEG   5.0f   // nozzle step per rotate command
#define ZONE_PRESSURE_STEP_DEG 5.0f   // valve angle fallback step (no cal data)
#define ZONE_PRESSURE_STEP_PSI 0.5f   // pressure step per +/- command (PSI)
// b413: 15 -> 5. The /exp/lash hysteresis run (2026-06-06) measured the actual
// direction-dependent stopping bias at ~2.8 deg (from-above stops ~2.8 deg more
// open than from-below); raw lash takeup is ~67ms (Section A), only a few deg
// motor-side. 15 deg was ~5x oversized. 5 deg covers the 2.8 deg bias with
// headroom while shaving the reversing-approach excursion.
#define VALVE_BACKLASH_DEG  5.0f
// b389: per-unit valve frame. There is NO mechanical hard stop on the valve;
// the AS5600L position magnet is mounted at a slightly different rotation on
// each unit, so these absolute angles differ per unit by a CONSTANT offset.
// The literals below are the reference unit's frame; g_valve_offset_deg (loaded
// from NVS at boot, measured by POST /cal/valve against the pressure peak)
// shifts the whole frame for other units. Offset 0 => reference unit unchanged.
// Every call site reads these macros, so the offset applies everywhere with no
// per-site edits. (Verified: no use requires a compile-time constant.)
static float g_valve_offset_deg = 0.0f;
// b418: per-unit valve motor polarity. +1 = reference wiring (driving the
// physical VFWD pin DECREASES encoder angle / closes), -1 = motor leads
// swapped. Probed open-loop at the start of POST /cal/valve (before any
// closed-loop move) and persisted in NVS ("vframe"/"dir"), loaded before the
// boot valve-close. VALVE_PIN_FWD always means "the pin that drives encoder
// angle DOWN" regardless of wiring, so every direction convention downstream
// (forward-decreases in valve_goto_direct, VALVE_OPEN_DIR = -1 in the chase
// loops, exp_drv_start's closing map) holds on both wirings. Bring-up console
// diagnostics keep the raw GPIOs on purpose: they exercise physical pins.
static int8_t g_valve_motor_dir = +1;
#define VALVE_PIN_FWD ((g_valve_motor_dir >= 0) ? GPIO_VFWD : GPIO_VREV)
#define VALVE_PIN_REV ((g_valve_motor_dir >= 0) ? GPIO_VREV : GPIO_VFWD)
// b419: same for the nozzle motor. Reference wiring: driving the physical
// NFWD pin INCREASES encoder angle (CW). Probed at the start of the nozzle
// speed cal (wcal_nozzle_task), persisted as "vframe"/"ndir". NOZZLE_PIN_FWD
// always means "the pin that drives encoder angle UP / CW", so nozzle_goto's
// forward-increases convention, the CW/CCW speed maps, the sweep cw pin
// selection, and the chase loops' n_dir=+1=CW all hold on both wirings.
static int8_t g_nozzle_motor_dir = +1;
#define NOZZLE_PIN_FWD ((g_nozzle_motor_dir >= 0) ? GPIO_NFWD : GPIO_NREV)
#define NOZZLE_PIN_REV ((g_nozzle_motor_dir >= 0) ? GPIO_NREV : GPIO_NFWD)
#define VALVE_CLOSED_DEG   (231.0f + g_valve_offset_deg)  // valve closed (new side)
#define VALVE_OPEN_DEG     (308.0f + g_valve_offset_deg)  // valve fully open (peak at 306.7)
#define VALVE_PEAK_DEG     (306.7f + g_valve_offset_deg)  // angle of maximum nozzle pressure
#define VALVE_CAL_START_DEG (263.0f + g_valve_offset_deg) // pressure begins rising here
#define VALVE_CAL_STEP_DEG    1.0f   // finer step in active pressure range
#define WATER_MIN_FLOW_PSI    0.30f  // below this PSI after valve open = no detectable flow, skip ring
#define CAL_DISC_STEPS       13     // discovery scan steps from closed position
#define NOZZLE_BACKLASH_DEG  15.0f

// Nozzle speed calibration
#define SPD_ROTATION_DEG  360.0f // measure time for full rotation
#define SPD_SETTLE_MS     500    // settle time before measuring

// Pressure calibration constants
#define CAL_NVS_NAMESPACE   "OtO"
// Forward declarations for LittleFS-primary cal/speed wrappers
static esp_err_t cal_save_primary(const pressure_map_t *map);
static esp_err_t cal_load_primary(pressure_map_t *map);
static esp_err_t spd_save_primary(const speed_map_t *m);
static esp_err_t spd_load_primary(speed_map_t *m);
static esp_err_t cal_save_nvs_internal(const pressure_map_t *map);
static esp_err_t cal_load_nvs_internal(pressure_map_t *map);
static esp_err_t spd_save_nvs_internal(const speed_map_t *m);
static esp_err_t spd_load_nvs_internal(speed_map_t *m);
#define HTTP_CONN_CLOSE(req) httpd_resp_set_hdr((req), "Connection", "close")
#define CAL_STEP_DEG        10.0f
#define CAL_SETTLE_MS       2000
#define CAL_SAMPLES         8

static void result(bool ok, const char *desc)
{
    if (ok) PASS("%s", desc);
    else    FAIL("%s", desc);
}

// ─────────────────────────────────────────────────────────────────────────────
// UART / TCP input helpers
// ─────────────────────────────────────────────────────────────────────────────
static void tprintf(const char *fmt, ...);  // forward declaration
static void uart_setup(void)
{
    // In ESPHome mode the logger has already installed the UART0 driver.
    // Re-installing returns ESP_ERR_INVALID_STATE and (worse) we observed
    // it leaving driver state half-broken on subsequent semaphore takes.
    if (uart_is_driver_installed(UART_NUM_0)) {
        return;
    }
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &cfg);
    uart_driver_install(UART_NUM_0, UART_BUF_SIZE, 0, 0, NULL, 0);
}

// Read one char from UART or TCP (whichever arrives first)
static char uart_getchar(uint32_t timeout_ms)
{
    // UINT32_MAX = wait indefinitely; timeout_ms=0 = non-blocking single attempt
    bool infinite = (timeout_ms == UINT32_MAX);
    TickType_t deadline = infinite ? 0 : xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms) + 1;
    // do-while ensures at least one attempt even with timeout_ms=0
    do {
        // Check UART
        uint8_t c = 0;
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(20)) > 0) {
            return (char)c;
        }
        // Check TCP socket
        if (s_tcp_client_fd >= 0) {
            struct timeval tv = { .tv_sec = 0, .tv_usec = 0 };
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(s_tcp_client_fd, &fds);
            if (select(s_tcp_client_fd + 1, &fds, NULL, NULL, &tv) > 0) {
                uint8_t tc = 0;
                int n = recv(s_tcp_client_fd, &tc, 1, 0);
                if (n > 0) {
                    return (char)tc;
                } else if (n == 0) {
                    // Client disconnected
                    close(s_tcp_client_fd);
                    s_tcp_client_fd = -1;
                    ESP_LOGI(TAG, "TCP client disconnected");
                }
            }
        }
    } while (infinite || xTaskGetTickCount() < deadline);
    return 0;
}

static int uart_readline(char *buf, int maxlen, uint32_t timeout_ms)
{
    int n = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (n < maxlen - 1) {
        if (xTaskGetTickCount() > deadline) break;
        char c = uart_getchar(100);
        if (c == 0)             continue;
        if (c == '\r' || c == '\n') {
            tprintf("\r\n");
            // Drain trailing \r or \n (TCP clients often send \r\n)
            char peek = uart_getchar(50);
            (void)peek;  // discard if it's another newline char
            break;
        }
        if (c == '\b' || c == 127)  {
            if (n > 0) { n--; tprintf("\b \b"); }
            continue;
        }
        buf[n++] = c;
        tprintf("%c\r\n", c);
        TOUCH_ACTIVITY();
    }
    buf[n] = '\0';
    return n;
}

// Custom printf wrapper that goes to both UART and TCP
static void tprintf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len > 0) {
        int out = (len < (int)sizeof(buf)) ? len : (int)sizeof(buf) - 1;
        fwrite(buf, 1, out, stdout);
        fflush(stdout);
        tcp_send(buf, out);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GPIO and ADC initialisation
// ─────────────────────────────────────────────────────────────────────────────
static void gpio_setup(void)
{
    uint64_t mask = (1ULL << GPIO_3V3SEN) | (1ULL << GPIO_9V_EN)  |
                    (1ULL << GPIO_K)      | (1ULL << GPIO_VFWD)   |
                    (1ULL << GPIO_VREV)   | (1ULL << GPIO_NFWD)   |
                    (1ULL << GPIO_NREV);
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(GPIO_3V3SEN, 0);
    gpio_set_level(GPIO_9V_EN,  0);
    gpio_set_level(GPIO_K,      1);   // brake on
    gpio_set_level(GPIO_VFWD,   0);
    gpio_set_level(GPIO_VREV,   0);
    gpio_set_level(GPIO_NFWD,   0);
    gpio_set_level(GPIO_NREV,   0);
}

static void adc_setup(void)
{
    if (s_adc) return;

    adc_oneshot_unit_init_cfg_t unit = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));

    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_VBATT, &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_VCUR,  &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_PCUR,  &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_CHARGE, &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_NCUR,  &ch));

    // ESP32 supports line fitting only (curve fitting is S2/S3 only)
    adc_cali_line_fitting_config_t cal = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t r = adc_cali_create_scheme_line_fitting(&cal, &s_cal);
    if (r == ESP_OK) {
        INFO("ADC calibration: line fitting OK");
    } else {
        WARN("ADC calibration unavailable (r=0x%x), using raw fallback", r);
        s_cal = NULL;
    }
}

static uint32_t adc_mv(adc_channel_t ch)
{
    int sum = 0, raw = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int avg = sum / ADC_SAMPLES;
    int mv  = 0;
    if (s_cal) {
        adc_cali_raw_to_voltage(s_cal, avg, &mv);
    } else {
        mv = avg * 3900 / 4095;
    }
    return (uint32_t)mv;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rail control
// ─────────────────────────────────────────────────────────────────────────────
static void sensor_rail_on(void)
{
    if (s_sensor_rail) return;
    gpio_set_level(GPIO_3V3SEN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));    // extra settling time
    i2c_bus_init();
    vTaskDelay(pdMS_TO_TICKS(100));   // sensors need time after power-on
    s_sensor_rail = true;
    INFO("Sensor rail ON (3V3Sen=GPIO4 HIGH)");
}
static void sensor_rail_off(void)
{
    if (!s_sensor_rail) return;
    i2c_bus_deinit();
    gpio_set_level(GPIO_3V3SEN, 0);
    s_sensor_rail = false;
    INFO("Sensor rail OFF");
}

static void motor_rail_on(void)
{
    if (s_motor_rail) return;
    gpio_set_level(GPIO_9V_EN, 1);
    gpio_set_level(GPIO_K,     0);
    vTaskDelay(pdMS_TO_TICKS(100));
    s_motor_rail = true;
    INFO("Motor rail ON (9V_EN=GPIO18, K=GPIO13 released)");
}

static void motor_rail_off(void)
{
    if (!s_motor_rail) return;
    gpio_set_level(GPIO_VFWD,  0);
    gpio_set_level(GPIO_VREV,  0);
    gpio_set_level(GPIO_NFWD,  0);
    gpio_set_level(GPIO_NREV,  0);
    gpio_set_level(GPIO_K,     1);
    gpio_set_level(GPIO_9V_EN, 0);
    s_motor_rail = false;
    INFO("Motor rail OFF");
}

// b376: active brake hold for calibration sampling, via the dedicated K line.
// b375 tried shorting both H-bridge inputs HIGH; that injected enough electrical
// noise onto the shared sensor rail / I2C bus that the encoder + pressure reads
// taken during the hold came back garbage (a 1-point scan with an impossible
// 217.9deg / 5.4PSI sample). K is the H-bridge's dedicated brake control, which
// engages a cleaner internal brake path than driving both inputs. The motor
// rail stays powered throughout cal_do_pressure_scan, so K=1 brakes; we release
// (K=0) before the next move so valve_goto drives normally.
//
// b377: gated on s_cal_brake so coast vs brake can be A/B'd at runtime.
// b380: a controlled A/B (the /valve/probe instrument, fixed angle list, coast
// and braked reads interleaved per-angle over 3 repeats) showed NO measurable
// cal benefit from braking -- achieved-angle scatter was a wash (the move/stall
// mechanics dominate, and the K hold only engages after valve_goto returns), and
// pressure-sample variance was dominated by supply-pressure drift (~0.5 PSI
// run-to-run), which a brake can't fix. So braking now defaults OFF (coast); the
// web UI "Run scan" and the HA path both coast. The K brake is retained behind
// POST /cal/pressure/start?brake=1 for future experiments (e.g. once supply
// pressure is stabilised). Brake is clean -- no read corruption -- that was only
// the b375 both-inputs-HIGH bridge short.
static bool s_cal_brake = false;
static inline void valve_brake_hold(void)    { if (s_cal_brake) gpio_set_level(GPIO_K, 1); }
static inline void valve_brake_release(void) { if (s_cal_brake) gpio_set_level(GPIO_K, 0); }

// ─────────────────────────────────────────────────────────────────────────────
// AS5600 helper
// ─────────────────────────────────────────────────────────────────────────────
static bool as5600_read(uint8_t addr, uint16_t *angle,
                        uint8_t *agc, uint8_t *status)
{
    uint8_t st = 0, ag = 0, buf[2] = {0};
    if (i2c_bus_read_reg(addr, AS5600_REG_STATUS, &st, 1) != ESP_OK) return false;
    i2c_bus_read_reg(addr, AS5600_REG_AGC, &ag, 1);
    if (i2c_bus_read_reg(addr, AS5600_REG_RAW_ANGLE_H, buf, 2) != ESP_OK) return false;
    if (angle)  *angle  = ((buf[0] & 0x0F) << 8) | buf[1];
    if (agc)    *agc    = ag;
    if (status) *status = st;
    return true;
}

static void print_as5600(uint8_t addr, const char *name)
{
    uint16_t angle = 0; uint8_t agc = 0, status = 0;
    bool ok = as5600_read(addr, &angle, &agc, &status);
    result(ok, name);
    if (!ok) return;
    INFO("  Status:0x%02x AGC:%d Angle:%u (%.1f deg)",
         status, agc, angle, angle * 360.0f / 4096.0f);
    result((status & AS5600_STATUS_MD) != 0, "  Magnet detected");
    result((status & AS5600_STATUS_ML) == 0, "  Magnet not too weak");
    result((status & AS5600_STATUS_MH) == 0, "  Magnet not too strong");
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1: I2C scan — uses i2c_bus probe via write attempt
// ─────────────────────────────────────────────────────────────────────────────
static int i2c_probe_bus(const uint8_t *expected, bool *found, int n_exp)
{
    int count = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        if (r == ESP_OK) {
            count++;
            bool is_exp = false;
            for (int i = 0; i < n_exp; i++) {
                if (addr == expected[i]) { found[i] = true; is_exp = true; }
            }
            INFO("  0x%02x %s", addr, is_exp ? "[expected]" : "[unexpected]");
        }
    }
    return count;
}

static void phase_i2c_scan(void)
{
    STEP("I2C Bus Scan");

    const uint8_t expected[]  = { ADDR_AS5600L, ADDR_AS5600, ADDR_MPRLS, ADDR_TCA6408A };
    const char   *exp_names[] = {
        "AS5600L valve    (0x40)",
        "AS5600  nozzle   (0x36)",
        "MPRLS   pressure (0x18)",
        "TCA6408A expander(0x20)",
    };
    bool found[4] = {false};

    sensor_rail_on();

    memset(found, 0, sizeof(found));
    int n = i2c_probe_bus(expected, found, 4);
    INFO("Found %d device(s)", n);


    if (n == 0) {
        WARN("No devices found. Run 't' to toggle 3V3Sen and verify:");
        INFO("  SDA (GPIO17 module pin28) should read ~3.3V when rail ON");
        INFO("  SCL (GPIO23 module pin37) should read ~3.3V when rail ON");
        INFO("  AS5600L U8 VCC (pin4) should read 3.3V when rail ON");
    }

    INFO("");
    for (int i = 0; i < 4; i++) result(found[i], exp_names[i]);
}
static void phase_valve_sensor(void)
{
    STEP("AS5600L Valve Position Sensor (0x40)");
    sensor_rail_on();
    print_as5600(ADDR_AS5600L, "AS5600L responds on I2C");
    INFO("Rotate valve shaft manually and re-run to verify angle changes");

    INFO("Reading 3x at 200ms intervals:");
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        uint16_t a = 0;
        as5600_read(ADDR_AS5600L, &a, NULL, NULL);
        INFO("  [%d] %u (%.1f deg)", i+1, a, a * 360.0f / 4096.0f);
    }
}

static void phase_nozzle_sensor(void)
{
    STEP("AS5600 Nozzle Position Sensor (0x36)");
    sensor_rail_on();
    print_as5600(ADDR_AS5600, "AS5600 responds on I2C");
    INFO("Rotate nozzle manually and re-run to verify angle changes");

    INFO("Reading 3x at 200ms intervals:");
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        uint16_t a = 0;
        as5600_read(ADDR_AS5600, &a, NULL, NULL);
        INFO("  [%d] %u (%.1f deg)", i+1, a, a * 360.0f / 4096.0f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 4: Pressure sensor
// ─────────────────────────────────────────────────────────────────────────────

static bool mprls_read(float *psi_out)
{
    uint8_t buf[4] = {0};
    esp_err_t r;

    // Protocol confirmed from original firmware disassembly:
    //   Write 0xAA 0x00 0x00  ->  address 0x08  (trigger measurement)
    //   Read  4 bytes          <-  address 0x18  (status + 24-bit pressure)
    // Two different I2C addresses for the same physical sensor.

    // Step 1: send measurement trigger to 0x08
    uint8_t trigger[3] = { 0xAA, 0x00, 0x00 };
    r = i2c_bus_write(ADDR_MPRLS, trigger, 3);
    INFO("MPRLS write trigger to 0x%02x: r=0x%x (%s)",
         ADDR_MPRLS, r, esp_err_to_name(r));

    // Wait for measurement (~5ms typical, 10ms safe)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2: read 4 bytes from 0x18
    r = i2c_bus_read(ADDR_MPRLS, buf, 4);
    INFO("MPRLS read from 0x%02x: r=0x%x  0x%02x 0x%02x 0x%02x 0x%02x",
         ADDR_MPRLS, r, buf[0], buf[1], buf[2], buf[3]);
    if (r != ESP_OK) {
        INFO("MPRLS read failed: %s", esp_err_to_name(r));
        return false;
    }

    if (buf[0] & MPRLS_STATUS_BUSY) {
        INFO("MPRLS busy -- retrying read after 10ms");
        vTaskDelay(pdMS_TO_TICKS(10));
        r = i2c_bus_read(ADDR_MPRLS, buf, 4);
        if (r != ESP_OK || (buf[0] & MPRLS_STATUS_BUSY)) {
            INFO("MPRLS still busy (status=0x%02x)", buf[0]);
            return false;
        }
    }
    if (buf[0] & 0x04) {
        INFO("MPRLS integrity error (status=0x%02x)", buf[0]);
        return false;
    }

    uint32_t raw = ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] << 8)  |
                    (uint32_t)buf[3];
    INFO("MPRLS raw count: 0x%06lx  status: 0x%02x", raw, buf[0]);

    // b298: reject readings whose raw count falls outside the sensor's
    // calibrated 10-90% range. Brief sensor glitches (air bubble in the
    // line, EMI, ADC noise) can produce raw values near 0x000000 which,
    // when fed through the uint32 subtraction below, wrap around and
    // yield ~8000 PSI. A single such sample averaged with normal ~5 PSI
    // reads produced a 670 PSI ring-average during the Zone b297 run.
    // The brief disturbance "passes quickly" -- the right behavior is
    // to drop that sample and let neighboring good samples drive the
    // average. Callers already handle the false return by skipping.
    if (raw < MPRLS_COUNT_MIN || raw > MPRLS_COUNT_MAX) {
        INFO("MPRLS reading out of range (raw=0x%06lx) -- rejected", raw);
        return false;
    }

    *psi_out = MPRLS_PSI_MAX *
               ((float)(raw - MPRLS_COUNT_MIN) /
                (float)(MPRLS_COUNT_MAX - MPRLS_COUNT_MIN));
    s_cached_psi = *psi_out;  // share with HA / web (no extra I2C needed)
    return true;
}

// Quiet MPRLS read for use in HTTP/state-JSON context.
// Identical to mprls_read() but suppresses all INFO-level logging
// so zone watering logs are not flooded with sensor debug output.
static bool mprls_read_quiet(float *psi_out)
{
    uint8_t buf[4] = {0};
    uint8_t trigger[3] = { 0xAA, 0x00, 0x00 };
    if (i2c_bus_write(ADDR_MPRLS, trigger, 3) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (i2c_bus_read(ADDR_MPRLS, buf, 4) != ESP_OK) return false;
    if (buf[0] & MPRLS_STATUS_BUSY) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (i2c_bus_read(ADDR_MPRLS, buf, 4) != ESP_OK) return false;
        if (buf[0] & MPRLS_STATUS_BUSY) return false;
    }
    if (buf[0] & 0x04) return false;
    uint32_t raw = ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] << 8)  |
                    (uint32_t)buf[3];
    // b298: see mprls_read() above for rationale -- reject out-of-range
    // raw counts so brief sensor glitches (air bubble, EMI) don't
    // contaminate downstream averages.
    if (raw < MPRLS_COUNT_MIN || raw > MPRLS_COUNT_MAX) return false;
    *psi_out = MPRLS_PSI_MAX *
               ((float)(raw - MPRLS_COUNT_MIN) /
                (float)(MPRLS_COUNT_MAX - MPRLS_COUNT_MIN));
    s_cached_psi = *psi_out;
    return true;
}

static void phase_pressure(void)
{
    STEP("MPRLS Pressure Sensor (0x%02x)", ADDR_MPRLS);
    sensor_rail_on();
    float psi = 0.0f;
    bool ok = mprls_read(&psi);
    result(ok, "MPRLS read");
    if (!ok) return;
    INFO("Pressure: %.2f PSI (%.1f hPa)", psi, psi * 68.9476f);
    // Near-zero is correct with no water connected.
    // With water: expect 30-120 PSI (residential supply).
    result(psi >= 0.0f && psi < 25.0f,
           "In valid gauge range (0-25 PSI)");

    INFO("Reading 3x at 300ms intervals:");
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(300));
        mprls_read(&psi);
        INFO("  [%d] %.2f PSI (%.1f hPa)", i+1, psi, psi * 68.9476f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 5: TCA6408A / LEDs
// ─────────────────────────────────────────────────────────────────────────────
static void phase_tca6408a(void)
{
    STEP("TCA6408A GPIO Expander (0x20) / RGB LEDs");
    sensor_rail_on();

    // Force bus reset to clear any stuck state from a prior crash mid-transaction
    i2c_bus_deinit();
    vTaskDelay(pdMS_TO_TICKS(20));
    i2c_bus_init();
    s_sensor_rail = true;
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg = 0xFF;
    esp_err_t r = i2c_bus_read_reg(ADDR_TCA6408A, TCA6408A_REG_CONFIG, &cfg, 1);
    result(r == ESP_OK, "TCA6408A config register read");
    INFO("Config: 0x%02x (0=output, 1=input)", cfg);

    // Cycle each bit individually: switch ONE pin to output HIGH, then pulse LOW.
    // This avoids switching all 8 LED sinks simultaneously which causes brownout.
    // Known TCA6408A bit mapping (confirmed):
    //   Bit 0 = Red LED   (0xFE)
    //   Bit 1 = Blue LED  (0xFD)
    //   Bit 2 = Green LED (0xFB)
    //   Bit 3 = Unknown   (brief flash)
    //   Bits 4-7 = Auxiliary valve controls (not present in this unit)
    const char *bit_names[] = {
        "Red  ", "Blue ", "Green", "Unk3 ",
        "AuxV4", "AuxV5", "AuxV6", "AuxV7"
    };

    // Set all 8 pins as outputs with all HIGH (LEDs off) first
    uint8_t val = 0xFF;
    i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &val, 1);
    uint8_t all_out = 0x00;
    i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_CONFIG, &all_out, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    INFO("Cycling bits LOW one at a time (5s each, all pins as outputs):");
    for (int bit = 0; bit < 8; bit++) {
        // All HIGH except this bit
        val = (uint8_t)(~(1 << bit));
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &val, 1);
        INFO("  Bit %d [%s] LOW (0x%02x)", bit, bit_names[bit], val);
        vTaskDelay(pdMS_TO_TICKS(5000));
        // All HIGH (LED off) before next bit
        val = 0xFF;
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &val, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Leave pins as outputs (all HIGH = LEDs off)
    // Returning to inputs causes glitch on next tca_led_set call
    result(true, "TCA6408A LED scan complete -- note bit->color mapping");
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 6: ADC
// ─────────────────────────────────────────────────────────────────────────────
static void phase_adc(void)
{
    STEP("ADC -- Battery Voltage and Current Sense");
    adc_setup();

    uint32_t vb = adc_mv(ADC_CH_VBATT);
    uint32_t vb_actual = (uint32_t)(vb * VBATT_DIVIDER_RATIO);
    INFO("VBattRaw GPIO36 ADC1_CH0: %lu mV (after divider)", vb);
    INFO("Battery estimate:          %lu mV (%.2fV) [ratio=%.1f]",
         vb_actual, vb_actual / 1000.0f, VBATT_DIVIDER_RATIO);
    INFO("NOTE: Measure divider resistors near J5, adjust VBATT_DIVIDER_RATIO");
    result(vb > 500,          "VBattRaw non-zero");
    result(vb_actual > VBATT_MIN_MV, "Battery above 3.3V threshold");

    uint32_t vc = adc_mv(ADC_CH_VCUR);
    uint32_t nc = adc_mv(ADC_CH_NCUR);
    uint32_t pc = adc_mv(ADC_CH_PCUR);
    INFO("VCur GPIO32 ADC1_CH4: %lu mV -> %.0f mA (motors off)", vc, CURRENT_MA(vc));
    INFO("NCur GPIO35 ADC1_CH7: %lu mV -> %.0f mA (motors off)", nc, CURRENT_MA(nc));
    INFO("PCur   GPIO34 ADC1_CH6: %lu mV (unused pump channel)",  pc);
    uint32_t chg = adc_mv(ADC_CH_CHARGE);
    INFO("Charge GPIO39 ADC1_CH3: %lu mV -> %.0f mA (solar/charge input)", chg, CURRENT_MA(chg));
    result(vc < 200, "VCur near zero (motors off)");
    result(nc < 200, "NCur near zero (motors off)");
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 7 & 8: Motor tests
// ─────────────────────────────────────────────────────────────────────────────
#include "driver/mcpwm_prelude.h"

// Run one motor drive pin using MCPWM at ~50% duty, 1kHz
// Drive a motor: pwm_pin gets 50% PWM, brake_pin held LOW (H-bridge direction)
// motor_run: PWM on drive_pin, direction_pin held LOW
// For forward: drive_pin=VFWD, direction_pin=VREV
// For reverse: drive_pin=VREV, direction_pin=VFWD
static void motor_run(gpio_num_t drive_pin, gpio_num_t direction_pin,
                      const char *label,
                      adc_channel_t cur_ch, const char *cur_label,
                      uint32_t ms)
{
    INFO("%s DRIVE=GPIO%d DIR=GPIO%d -- %lu ms",
         label, (int)drive_pin, (int)direction_pin, ms);

    // Hold direction pin LOW
    gpio_set_direction(direction_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(direction_pin, 0);

    // PWM on drive pin
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .period_ticks  = 500,       // 20kHz
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_cfg, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_new_operator(&oper_cfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, 250);  // 50% duty @ 20kHz

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = drive_pin };
    mcpwm_new_generator(oper, &gen_cfg, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    vTaskDelay(pdMS_TO_TICKS(ms / 2));
    uint32_t mv = adc_mv(cur_ch);
    INFO("  %s: %lu mV -> %.0f mA", cur_label, mv, CURRENT_MA(mv));
    result(mv > 200, "Motor drawing current");
    vTaskDelay(pdMS_TO_TICKS(ms / 2));

    // Stop: both pins LOW
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(drive_pin, 0);
    gpio_set_level(direction_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(400));
}

static void phase_valve_motor(void)
{
    STEP("Valve Motor Test");
    WARN("Valve will move. 3 second warning...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    uint16_t before = 0, after = 0;
    as5600_read(ADDR_AS5600L, &before, NULL, NULL);
    INFO("Valve angle before: %u (%.1f deg)", before, before * 360.0f / 4096.0f);

    motor_run(GPIO_VFWD, GPIO_VREV, "Valve FORWARD", ADC_CH_VCUR, "VCur", 800);
    as5600_read(ADDR_AS5600L, &after, NULL, NULL);
    INFO("Valve angle after fwd: %u (%.1f deg)  delta: %d counts",
         after, after * 360.0f / 4096.0f, (int)after - (int)before);
    result(abs((int)after - (int)before) > 10,
           "Valve position changed during forward run");

    before = after;
    motor_run(GPIO_VREV, GPIO_VFWD, "Valve REVERSE", ADC_CH_VCUR, "VCur", 800);
    as5600_read(ADDR_AS5600L, &after, NULL, NULL);
    INFO("Valve angle after rev: %u (%.1f deg)  delta: %d counts",
         after, after * 360.0f / 4096.0f, (int)after - (int)before);
    result(abs((int)after - (int)before) > 10,
           "Valve position changed during reverse run");

    motor_rail_off();
}

static void phase_nozzle_motor(void)
{
    STEP("Nozzle Motor Test");
    WARN("Nozzle will rotate. 3 second warning...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    uint16_t before = 0, after = 0;
    as5600_read(ADDR_AS5600, &before, NULL, NULL);
    INFO("Nozzle angle before: %u (%.1f deg)", before, before * 360.0f / 4096.0f);

    motor_run(GPIO_NFWD, GPIO_NREV, "Nozzle FORWARD", ADC_CH_NCUR, "NCur", 800);
    as5600_read(ADDR_AS5600, &after, NULL, NULL);
    INFO("Nozzle angle after fwd: %u (%.1f deg)", after, after * 360.0f / 4096.0f);

    before = after;
    motor_run(GPIO_NREV, GPIO_NFWD, "Nozzle REVERSE", ADC_CH_NCUR, "NCur", 800);
    as5600_read(ADDR_AS5600, &after, NULL, NULL);
    INFO("Nozzle angle after rev: %u (%.1f deg)", after, after * 360.0f / 4096.0f);

    motor_rail_off();
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 9: Kill/brake
// ─────────────────────────────────────────────────────────────────────────────
// Move nozzle to target angle using same control logic as valve.
// Nozzle direction: forward increases angle, reverse decreases.
// Uses backlash compensation -- always approaches from below.

static bool nozzle_goto_direct(float target_deg, float tolerance_deg,
                                uint32_t timeout_ms, bool verbose)
{
    const float DEG_PER_COUNT  = 360.0f / 4096.0f;
    const float MIN_MA         = 32.0f;
    const float STALL_MA       = 500.0f;
    const uint32_t POLL_MS     = 10;    // 100 Hz -- faster vel estimate
    const float COAST_S        = 0.04f; // at ~47 dps run speed, 0.12 gave 18-22 deg undershoot

    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) {
        ESP_LOGE(TAG, "nozzle_goto: sensor read failed");
        return false;
    }
    float current_deg = raw * DEG_PER_COUNT;

    float error = target_deg - current_deg;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= tolerance_deg) {
        if (verbose) ESP_LOGI(TAG, "  Already within tolerance");
        return true;
    }

    // Forward increases angle, reverse decreases (NOZZLE_PIN_* resolves the
    // physical GPIO per stored motor polarity, b419)
    bool forward = (error > 0);
    gpio_num_t drive_pin = forward ? NOZZLE_PIN_FWD : NOZZLE_PIN_REV;
    gpio_num_t dir_pin   = forward ? NOZZLE_PIN_REV : NOZZLE_PIN_FWD;

    gpio_set_direction(dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dir_pin, 0);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .period_ticks = 500,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_cfg, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_new_operator(&oper_cfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, 200);  // initial duty
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = drive_pin };
    mcpwm_new_generator(oper, &gen_cfg, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // Kickstart: brief full-duty pulse to overcome static friction.
    // Critical for small jog moves where proportional duty alone
    // may not break the motor free before the position check exits.
    // Duration scales with move size: tiny moves get a short kick.
    // Kick duration scales tightly with move size.
    // 50ms at full duty moves ~1.5 deg -- longer than most inter-ring steps.
    float abs_err_kick = fabsf(error);
    uint32_t kick_ms = (abs_err_kick <  2.0f) ?  8 :
                       (abs_err_kick <  5.0f) ? 15 :
                       (abs_err_kick < 15.0f) ? 25 :
                       (abs_err_kick < 30.0f) ? 20 : 15;
    mcpwm_comparator_set_compare_value(cmpr, 480); // ~96% duty kickstart
    vTaskDelay(pdMS_TO_TICKS(kick_ms));
    mcpwm_comparator_set_compare_value(cmpr, 200); // back to run duty

    uint32_t elapsed = kick_ms + POLL_MS;
    bool reached = false, stalled = false;
    float peak_ma = 0;
    int low_current_count = 0;
    float prev_deg = current_deg;  // for velocity estimation

    while (elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        elapsed += POLL_MS;

        uint32_t mv = adc_mv(ADC_CH_NCUR);
        float ma = CURRENT_MA(mv);
        if (ma > peak_ma) peak_ma = ma;

        if (ma < MIN_MA) {
            if (++low_current_count >= 10) {
                ESP_LOGW(TAG, "  Nozzle low current %.0f mA", ma);
                break;
            }
        } else {
            low_current_count = 0;
        }
        if (ma > STALL_MA) {
            stalled = true;
            ESP_LOGW(TAG, "  Nozzle stall %.0f mA", ma);
            break;
        }

        if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) break;
        float new_deg = raw * DEG_PER_COUNT;

        // Velocity estimate (deg/s), wrap-safe
        float delta = new_deg - prev_deg;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        float vel_dps = delta / (POLL_MS / 1000.0f);
        prev_deg    = new_deg;
        current_deg = new_deg;

        error = target_deg - current_deg;
        if (error >  180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        float abs_err = fabsf(error);

        // Predictive braking: stop when coasting distance >= remaining error.
        // stopping_dist = vel * coast_time. Clamp velocity to avoid
        // encoder 0/360 wrap artifacts from triggering early stop.
        // At run_duty=200 (~47 dps) anything above 80 dps is a wrap spike.
        // When spike detected: stopping_dist=0 so that tick is skipped.
        float stopping_dist = 0.0f;
        if (elapsed > kick_ms + 3*POLL_MS) {  // ignore first few ticks
            float vel_clamped = fabsf(vel_dps);
            if (vel_clamped > 80.0f) vel_clamped = 0.0f;  // discard wrap spike
            stopping_dist = vel_clamped * COAST_S;
        }

        // Proportional duty -- still ramp down when close
        uint32_t duty = abs_err > 10.0f ? 350 : abs_err > 5.0f ? 175 : 125;
        mcpwm_comparator_set_compare_value(cmpr, duty);

        if (verbose)
            ESP_LOGI(TAG, "  pos=%.1f err=%.1f vel=%.1f stop=%.1f ma=%.0f",
                     current_deg, error, vel_dps, stopping_dist, ma);

        // Stop conditions: within tolerance, overshot, or coast will overshoot
        if (abs_err <= tolerance_deg)                        { reached = true; break; }
        if (forward  && error < 0)                           { reached = true; break; }
        if (!forward && error > 0)                           { reached = true; break; }
        if (stopping_dist >= abs_err + tolerance_deg * 0.5f) { reached = true; break; }
    }

    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(drive_pin, 0);
    gpio_set_level(dir_pin, 0);

    if (!reached && !stalled)
        ESP_LOGW(TAG, "  Nozzle timeout after %lu ms", elapsed);

    ESP_LOGI(TAG, "  Nozzle final: %.1f deg (target %.1f, error %.1f, peak %.0f mA)",
             current_deg, target_deg, error, peak_ma);

    return reached && !stalled && peak_ma >= MIN_MA;
}

// nozzle_goto: backlash compensation only when direction reverses.
// prev_dir: +1 = last move was CW (increasing angle), -1 = CCW, 0 = unknown.
// Pass s_nozzle_last_dir for the zone setup use case.
static bool nozzle_goto_ex(float target_deg, float tolerance_deg,
                            uint32_t timeout_ms, bool verbose, int prev_dir)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) return false;
    float current_deg = raw * DEG_PER_COUNT;

    float error = target_deg - current_deg;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= tolerance_deg) {
        if (verbose) ESP_LOGI(TAG, "  Already within tolerance");
        return true;
    }

    int this_dir = (error > 0) ? 1 : -1;

    // Only overshoot if direction has reversed (or unknown)
    bool need_backlash = (this_dir < 0) && (prev_dir >= 0);

    if (need_backlash) {
        float overshoot = target_deg - NOZZLE_BACKLASH_DEG;
        if (overshoot < 0) overshoot += 360.0f;
        if (verbose)
            ESP_LOGI(TAG, "  Nozzle backlash: overshooting to %.1f", overshoot);
        if (!nozzle_goto_direct(overshoot, tolerance_deg, timeout_ms / 2, verbose))
            return false;
    }

    return nozzle_goto_direct(target_deg, tolerance_deg, timeout_ms / 2, verbose);
}

// Standard nozzle_goto always applies backlash compensation (safe default)
static bool nozzle_goto(float target_deg, float tolerance_deg,
                        uint32_t timeout_ms, bool verbose)
{
    return nozzle_goto_ex(target_deg, tolerance_deg, timeout_ms, verbose, 1);
}


static bool valve_goto(float target_deg, float tolerance_deg, uint32_t timeout_ms, bool verbose);
// speed_map_t and NVS helpers defined before stall finder which uses them
// speed_map_t defined in irrigoto_types.h

static esp_err_t spd_save_nvs_internal(const speed_map_t *m)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_blob(h, "speed_map", m, sizeof(*m));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}

static esp_err_t spd_load_nvs_internal(speed_map_t *m)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    size_t sz = sizeof(*m);
    r = nvs_get_blob(h, "speed_map", m, &sz);
    nvs_close(h);
    return r;
}

// Find minimum duty to START moving from rest.
// Steps up from zero in step_duty increments.
// At each level ensures motor is stopped, then applies duty and checks for movement.
// Returns lowest duty that produced movement from rest, or 0 if none found.
static uint16_t spd_find_start_point(uint16_t step_duty, uint32_t measure_ms)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    const float MIN_MOVEMENT  = 2.0f;
    const uint16_t MAX_DUTY   = 150;  // stop searching above 30% -- clearly moving

    for (uint16_t duty = step_duty; duty <= MAX_DUTY; duty += step_duty) {
        float pct = duty * 100.0f / 500.0f;
        INFO("  Testing duty=%d (%.0f%%)...", duty, pct);

        // Ensure motor is fully stopped before each test
        gpio_set_level(GPIO_NFWD, 0);
        gpio_set_level(GPIO_NREV, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Read start position
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600, &raw, NULL, NULL);
        float prev_deg = raw * DEG_PER_COUNT;
        float travelled = 0.0f;

        // Apply duty from rest
        gpio_set_direction(GPIO_NREV, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NREV, 0);

        mcpwm_timer_handle_t timer = NULL;
        mcpwm_timer_config_t tcfg = {
            .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000, .period_ticks = 500,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        mcpwm_new_timer(&tcfg, &timer);
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t ocfg = { .group_id = 0 };
        mcpwm_new_operator(&ocfg, &oper);
        mcpwm_operator_connect_timer(oper, timer);
        mcpwm_cmpr_handle_t cmpr = NULL;
        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
        mcpwm_new_comparator(oper, &ccfg, &cmpr);
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gcfg = { .gen_gpio_num = GPIO_NFWD };
        mcpwm_new_generator(oper, &gcfg, &gen);
        mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           cmpr, MCPWM_GEN_ACTION_LOW));
        mcpwm_timer_enable(timer);
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

        // Monitor for movement
        TickType_t t_start = xTaskGetTickCount();
        bool started = false;
        while (pdTICKS_TO_MS(xTaskGetTickCount() - t_start) < measure_ms) {
            vTaskDelay(pdMS_TO_TICKS(50));
            as5600_read(ADDR_AS5600, &raw, NULL, NULL);
            float cur = raw * DEG_PER_COUNT;
            float delta = cur - prev_deg;
            if (delta < -180.0f) delta += 360.0f;
            if (delta >  180.0f) delta -= 360.0f;
            travelled += delta;
            prev_deg = cur;
            if (fabsf(travelled) >= MIN_MOVEMENT) { started = true; break; }
        }

        // Stop motor
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(timer);
        mcpwm_del_generator(gen);
        mcpwm_del_comparator(cmpr);
        mcpwm_del_operator(oper);
        mcpwm_del_timer(timer);
        gpio_set_level(GPIO_NFWD, 0);
        gpio_set_level(GPIO_NREV, 0);

        if (started) {
            INFO("    -> STARTED: %.1f deg in %.0fms", fabsf(travelled), (float)measure_ms);
            return duty;
        } else {
            INFO("    -> NO START: %.2f deg (below %.1f deg threshold)",
                 fabsf(travelled), MIN_MOVEMENT);
        }
    }
    return 0;
}

// Find minimum duty before nozzle motor stalls.
// Steps down from start_duty in step_duty increments.
// At each level runs for measure_ms and checks AS5600 for movement.
// Returns lowest duty that produced movement, or 0 if all stalled.
static uint16_t spd_find_stall_point(uint16_t start_duty, uint16_t step_duty,
                                      uint32_t measure_ms)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    const float MIN_MOVEMENT  = 2.0f;  // deg -- below this = stalled
    uint16_t last_good = 0;

    for (uint16_t duty = start_duty; duty >= step_duty; duty -= step_duty) {
        float pct = duty * 100.0f / 500.0f;
        INFO("  Testing duty=%d (%.0f%%)...", duty, pct);

        // Read start position
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600, &raw, NULL, NULL);
        float start_deg = raw * DEG_PER_COUNT;
        float prev_deg  = start_deg;
        float travelled = 0.0f;

        // Run motor at this duty
        gpio_set_direction(GPIO_NREV, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NREV, 0);

        mcpwm_timer_handle_t timer = NULL;
        mcpwm_timer_config_t tcfg = {
            .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000, .period_ticks = 500,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        mcpwm_new_timer(&tcfg, &timer);
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t ocfg = { .group_id = 0 };
        mcpwm_new_operator(&ocfg, &oper);
        mcpwm_operator_connect_timer(oper, timer);
        mcpwm_cmpr_handle_t cmpr = NULL;
        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
        mcpwm_new_comparator(oper, &ccfg, &cmpr);
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gcfg = { .gen_gpio_num = GPIO_NFWD };
        mcpwm_new_generator(oper, &gcfg, &gen);
        mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           cmpr, MCPWM_GEN_ACTION_LOW));
        mcpwm_timer_enable(timer);
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

        // Poll AS5600 for movement during measure window
        TickType_t t_start = xTaskGetTickCount();
        while (pdTICKS_TO_MS(xTaskGetTickCount() - t_start) < measure_ms) {
            vTaskDelay(pdMS_TO_TICKS(50));
            as5600_read(ADDR_AS5600, &raw, NULL, NULL);
            float cur = raw * DEG_PER_COUNT;
            float delta = cur - prev_deg;
            if (delta < -180.0f) delta += 360.0f;
            if (delta >  180.0f) delta -= 360.0f;
            travelled += delta;
            prev_deg = cur;
        }

        // Stop motor
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(timer);
        mcpwm_del_generator(gen);
        mcpwm_del_comparator(cmpr);
        mcpwm_del_operator(oper);
        mcpwm_del_timer(timer);
        gpio_set_level(GPIO_NFWD, 0);
        gpio_set_level(GPIO_NREV, 0);
        vTaskDelay(pdMS_TO_TICKS(300));  // brief coast before next step

        float abs_travel = fabsf(travelled);
        float dps = abs_travel / (measure_ms / 1000.0f);

        if (abs_travel >= MIN_MOVEMENT) {
            INFO("    -> MOVING: %.1f deg total, %.1f deg/sec", abs_travel, dps);
            last_good = duty;
        } else {
            INFO("    -> STALLED: %.2f deg total (below %.1f deg threshold)",
                 abs_travel, MIN_MOVEMENT);
            break;  // stalled -- stop searching lower
        }
    }

    return last_good;
}

static void phase_nozzle_stall_find(void)
{
    STEP("Nozzle Stall Point Finder");
    INFO("Steps duty down from 20%% until motor stalls.");
    INFO("Each step runs for 2 seconds while monitoring AS5600 for movement.");
    INFO("Press Enter to start, q to abort.");

    int c = uart_getchar(30000);
    if (c == 'q' || c == 'Q' || c == 0) { INFO("Aborted."); return; }

    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- Start scan: step UP from rest ---
    INFO("Phase 1: Start scan (stepping up from 0%% until movement from rest)...");
    INFO("  Duty   %%PWM   Result");
    INFO("  -----  -----  ------");
    uint16_t start_duty = spd_find_start_point(10, 2000);

    if (start_duty > 0) {
        INFO("Start threshold: duty=%d (%.0f%%) -- motor starts from rest above this.",
             start_duty, start_duty * 100.0f / 500.0f);
        PASS("Start threshold found");
    } else {
        INFO("Could not find start threshold below 30%%.");
        FAIL("Start threshold finder");
    }

    // --- Stall scan: step DOWN from 20%% while running ---
    INFO("Phase 2: Stall scan (stepping down from 20%% until stall while running)...");
    INFO("  Duty   %%PWM   Result");
    INFO("  -----  -----  ------");
    uint16_t min_duty = spd_find_stall_point(100, 10, 2000);

    if (min_duty > 0) {
        float min_pct   = min_duty   * 100.0f / 500.0f;
        float start_pct = start_duty * 100.0f / 500.0f;
        INFO("Stall threshold: duty=%d (%.0f%%)", min_duty, min_pct);
        INFO("Hysteresis: start=%.0f%% stall=%.0f%% (diff=%.0f%%)",
             start_pct, min_pct, start_pct - min_pct);
        PASS("Stall threshold found");

        // Update NVS -- use start_duty for jog (must reliably start from rest)
        speed_map_t m = {0};
        if (spd_load_primary(&m) == ESP_OK && m.num_points > 0) {
            float max_dps  = m.deg_per_sec[m.num_points - 1];
            float max_duty = (float)m.duty[m.num_points - 1];
            m.min_continuous_dps = (min_duty / max_duty) * max_dps;
            m.jog_pulse_duty     = (start_duty > 0) ? start_duty : min_duty;
            spd_save_primary(&m);
            INFO("Updated NVS: min_continuous_dps=%.1f deg/sec",
                 m.min_continuous_dps);
            INFO("             jog_pulse_duty=%d (%.0f%% -- start threshold)",
                 m.jog_pulse_duty, m.jog_pulse_duty * 100.0f / 500.0f);
        }
    } else {
        INFO("Motor stalled at all tested duty levels.");
        FAIL("Stall threshold finder");
    }

    motor_rail_off();
    INFO("Done.");
}

static void phase_nozzle_speed_cal(void);
static void phase_nozzle_speed_view(void);
static bool valve_goto_ex(float target_deg, float tolerance_deg, uint32_t timeout_ms, bool verbose, int prev_dir);
static bool valve_goto_jog(float target_deg, float tolerance_deg, uint32_t timeout_ms);
static inline bool valve_in_friction_zone(float deg);

// ─────────────────────────────────────────────────────────────────────────────
// Pressure calibration scan
// Sweeps valve from open to closed in steps, records pressure at each point,
// saves to NVS under namespace "OtO" as blob "pressure_map"
// ─────────────────────────────────────────────────────────────────────────────

// pressure_map_t defined in irrigoto_types.h

static esp_err_t cal_save_primary(const pressure_map_t *map)
{
    if (storage_ready()) storage_cal_save(map);
    return cal_save_nvs_internal(map);
}
static esp_err_t cal_load_primary(pressure_map_t *map)
{
    if (storage_ready() && storage_cal_load(map) == ESP_OK && map->num_points > 0)
        return ESP_OK;
    return cal_load_nvs_internal(map);
}
static esp_err_t spd_save_primary(const speed_map_t *m)
{
    if (storage_ready()) storage_spd_save(m);
    return spd_save_nvs_internal(m);
}
static esp_err_t spd_load_primary(speed_map_t *m)
{
    if (storage_ready() && storage_spd_load(m) == ESP_OK && m->num_points > 0)
        return ESP_OK;
    return spd_load_nvs_internal(m);
}
static esp_err_t cal_save_nvs_internal(const pressure_map_t *map)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_blob(h, "pressure_map", map, sizeof(*map));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}

static esp_err_t cal_load_nvs_internal(pressure_map_t *map)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    size_t sz = sizeof(*map);
    r = nvs_get_blob(h, "pressure_map", map, &sz);
    nvs_close(h);
    return r;
}

// b389: per-unit valve frame offset. Stored in its own NVS namespace ("vframe")
// so valve_offset_nvs_load() can run at the very top of irrigoto_init() --
// BEFORE the boot valve-close -- whereas the LittleFS cal data isn't mounted
// that early. Offset 0 (no key stored) leaves the reference frame untouched.
static void valve_offset_nvs_save(float off)
{
    nvs_handle_t h;
    if (nvs_open("vframe", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "off", &off, sizeof(off));
    nvs_commit(h);
    nvs_close(h);
}

// b418/b419: per-unit motor polarity, same namespace (see g_valve_motor_dir /
// g_nozzle_motor_dir). key = "dir" (valve) or "ndir" (nozzle).
static void motor_dir_nvs_save(const char *key, int8_t dir)
{
    nvs_handle_t h;
    if (nvs_open("vframe", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, key, dir);
    nvs_commit(h);
    nvs_close(h);
}

static void valve_offset_nvs_load(void)
{
    nvs_handle_t h;
    if (nvs_open("vframe", NVS_READONLY, &h) != ESP_OK) return;
    float off = 0.0f;
    size_t sz = sizeof(off);
    if (nvs_get_blob(h, "off", &off, &sz) == ESP_OK && sz == sizeof(off)) {
        if (off > -60.0f && off < 60.0f) {
            g_valve_offset_deg = off;
            INFO("Valve frame offset loaded: %.2f deg", off);
        } else {
            ESP_LOGW(TAG, "Valve frame offset %.2f out of range -- ignoring", off);
        }
    }
    int8_t dir = 0;
    if (nvs_get_i8(h, "dir", &dir) == ESP_OK && (dir == 1 || dir == -1)) {
        g_valve_motor_dir = dir;
        if (dir < 0)
            ESP_LOGW(TAG, "Valve motor polarity: INVERTED (leads swapped; from /cal/valve probe)");
    }
    int8_t ndir = 0;
    if (nvs_get_i8(h, "ndir", &ndir) == ESP_OK && (ndir == 1 || ndir == -1)) {
        g_nozzle_motor_dir = ndir;
        if (ndir < 0)
            ESP_LOGW(TAG, "Nozzle motor polarity: INVERTED (leads swapped; from nozzle cal probe)");
    }
    nvs_close(h);
}

static float cal_read_pressure_avg(void)
{
    float sum = 0;
    int ok = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        float psi = 0;
        if (mprls_read(&psi)) { sum += psi; ok++; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ok ? sum / ok : -1.0f;
}

// b391: median of n pressure samples taken while STATIONARY. The median
// rejects the brief water-hammer / valve-motion transients that fooled the
// b390 valve-frame sweep (a single spike latched the argmax ~6 deg past the
// true peak). Returns -1 if no sample read.
static float cal_pressure_settled_median(int n)
{
    if (n < 1) n = 1;
    if (n > 9) n = 9;
    float v[9];
    int got = 0;
    for (int i = 0; i < n; i++) {
        float p = 0;
        if (mprls_read_quiet(&p)) v[got++] = p;
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (got == 0) return -1.0f;
    for (int i = 1; i < got; i++) {            // insertion sort
        float key = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j+1] = v[j]; j--; }
        v[j+1] = key;
    }
    return v[got / 2];
}

static void phase_valve_pct(void);
static void phase_pressure_monitor(void);
static void phase_perim_water(void);
static void phase_water_zone(void);
static void water_cleanup_pass(const zone_perimeter_t *, float, float, float, float, float, float, float, int);
static void phase_water_zone_mode(int mode);  // 1-4 = metered, 99 = demo
// b283: pressure trace recorder helpers (defined just above phase_water_zone).
// Forward-declared so sweep functions earlier in the file can invoke
// water_trace_sample() at their MPRLS sample points.
static void water_trace_reset(uint16_t zone_id);
static void water_trace_mark_ring(int ring_idx, float valve_deg);
static void water_trace_sample(float psi);
static bool water_trace_save(uint16_t zone_id);
static void water_trace_refit_f_and_log(void);  // b284
// b292: watering-mode helpers (WiFi pause + RAM log buffer).
static void watering_log_capture_start(void);
static void watering_log_capture_stop(void);
static void watering_wifi_pause(void);
static void watering_wifi_resume(void);
static float water_hold_pressure(float target_psi, float psi_min, float psi_max);
static float water_seat_valve_from_closed(float target_psi, float *out_psi);
static void zone_web_start(void);
static void phase_valve_jog_explore(void);
static void phase_valve_hysteresis(void);   // EXP: reversal dead-time + hysteresis
static void phase_encoder_health(void);
static float zone_get_psi_max(void);
static float cal_throw_to_valve_deg(float throw_mm);
static float cal_get_max_throw_mm(void);
static float cal_get_min_throw_mm(void);
// --- Web calibration state machine (declared here so cal_do_pressure_scan can access it) ---
typedef enum {
    WCAL_IDLE = 0,
    WCAL_PRESSURE_SCANNING,
    WCAL_PRESSURE_AWAIT_THROW,
    WCAL_NOZZLE_RUNNING,
    WCAL_JOG_RUNNING,
    WCAL_DONE,
    WCAL_ERROR,
    WCAL_PRESSURE_AWAIT_THROW_LOW,   // b385: appended -- keeps prior enum values
                                     // stable for the cal page's JS constants
} wcal_state_t;

static struct {
    wcal_state_t state;
    char         msg[128];
    int          progress;
    pressure_map_t pmap;
    float          pmap_open_psi;
    float          pmap_max_throw;
    float          pmap_low_psi;     // b385: low throw anchor (pressure at the
    float          pmap_low_throw;   //       short opening) for two-anchor fit
} s_wcal;

static void wcal_reset(void) { memset(&s_wcal, 0, sizeof(s_wcal)); }

// -----------------------------------------------------------------------
// cal_post_process_scan: shared post-processing for both terminal (x) and
// web pressure cal paths.  Operates on map.* in-place, returns new n.
//   1. Sort by valve_deg ascending
//   2. Outlier rejection (>0.8 PSI from linear neighbours)
//   3. Dedup (valve angles within 0.5 deg)
//   4. Monotone enforcement (PSI non-decreasing)
// -----------------------------------------------------------------------
static int cal_post_process_scan(pressure_map_t *m)
{
    int n = m->num_points;

    // 1. Sort ascending by valve_deg
    for (int si = 0; si < n-1; si++)
        for (int sj = si+1; sj < n; sj++)
            if (m->valve_deg[sj] < m->valve_deg[si]) {
                float tv=m->valve_deg[si]; m->valve_deg[si]=m->valve_deg[sj]; m->valve_deg[sj]=tv;
                float tp=m->pressure_psi[si]; m->pressure_psi[si]=m->pressure_psi[sj]; m->pressure_psi[sj]=tp;
                float tt=m->throw_mm[si]; m->throw_mm[si]=m->throw_mm[sj]; m->throw_mm[sj]=tt;
            }

    // 2. Outlier rejection: remove points > 0.8 PSI from linear neighbours
    {
        bool keep[CAL_MAX_POINTS]; for (int i=0;i<n;i++) keep[i]=true;
        for (int i=1; i<n-1; i++) {
            float dv = m->valve_deg[i+1]-m->valve_deg[i-1];
            if (dv < 0.01f) continue;
            float t = (m->valve_deg[i]-m->valve_deg[i-1])/dv;
            float pi = m->pressure_psi[i-1]+t*(m->pressure_psi[i+1]-m->pressure_psi[i-1]);
            if (fabsf(m->pressure_psi[i]-pi) > 0.8f) {
                INFO("  Outlier: pt %d %.1fdeg %.3fPSI (interp=%.3f) removed",
                     i+1, m->valve_deg[i], m->pressure_psi[i], pi);
                keep[i] = false;
            }
        }
        int j=0;
        for (int i=0;i<n;i++) if(keep[i]) {
            m->valve_deg[j]=m->valve_deg[i];
            m->pressure_psi[j]=m->pressure_psi[i];
            m->throw_mm[j]=m->throw_mm[i]; j++;
        }
        if (j < n) INFO("  Outlier removal: %d removed, %d remain.", n-j, j);
        n=j; m->num_points=n;
    }

    // 3. Dedup: remove valve angles within 0.5 deg of a prior point
    {
        int j=0;
        for (int i=0;i<n;i++) {
            bool dup=false;
            for (int k=0;k<j;k++)
                if (fabsf(m->valve_deg[i]-m->valve_deg[k])<0.5f) { dup=true; break; }
            if (!dup) {
                if (j!=i) { m->valve_deg[j]=m->valve_deg[i];
                             m->pressure_psi[j]=m->pressure_psi[i];
                             m->throw_mm[j]=m->throw_mm[i]; }
                j++;
            } else INFO("  Dedup: removing pt %d (%.1f deg)", i+1, m->valve_deg[i]);
        }
        if (j < n) INFO("  %d duplicate(s) removed, %d remain.", n-j, j);
        n=j; m->num_points=n;
    }

    // 3b. Saturation dedup: remove plateau points where PSI delta from previous < 0.05
    //     (catches saturated gap-fill points that the fine-scan skip missed)
    if (n > 1) {
        int j = 1; // always keep first point
        for (int i = 1; i < n; i++) {
            if (m->pressure_psi[i] - m->pressure_psi[j-1] < 0.05f) {
                INFO("  Sat-dedup: pt %d (%.1f deg %.4f PSI, delta %.4f) removed",
                     i+1, m->valve_deg[i], m->pressure_psi[i],
                     m->pressure_psi[i] - m->pressure_psi[j-1]);
                continue;
            }
            if (j != i) { m->valve_deg[j]    = m->valve_deg[i];
                          m->pressure_psi[j] = m->pressure_psi[i];
                          m->throw_mm[j]     = m->throw_mm[i]; }
            j++;
        }
        if (j < n) INFO("  Sat-dedup: %d plateau point(s) removed, %d remain.", n-j, j);
        n = j; m->num_points = n;
    }

    // 4. Monotone enforcement: PSI must be non-decreasing with valve_deg
    {
        float pmax=m->pressure_psi[0]; int nfix=0;
        for (int i=1;i<n;i++) {
            if (m->pressure_psi[i]<pmax) { m->pressure_psi[i]=pmax; nfix++; }
            else pmax=m->pressure_psi[i];
        }
        if (nfix) INFO("  Monotone fix: %d inversion(s) corrected.", nfix);
    }

    return n;
}

// b385: two-anchor throw model. The legacy model derived every ring's throw
// from ONE max-pressure measurement as throw[i] = psi[i]/psi_max * max_throw --
// a proportional ray through the origin, assuming throw is proportional to
// pressure. A real sprinkler's throw-vs-pressure curve has a positive pressure
// intercept that depends on mount height: a head mounted high off the ground
// throws farther for the same pressure than a ground-mounted head (more
// airtime). The single-ray model overstated reach at low pressure -- the root
// of the ~982mm "floor" on this ground-mounted unit. Anchoring the curve with
// TWO measured points (a low opening and full open) gives the fit a real slope
// AND intercept instead of forcing it through the origin.
static void cal_apply_two_anchor_throw(pressure_map_t *pm,
                                       float psi_lo, float throw_lo,
                                       float psi_hi, float throw_hi)
{
    if (psi_hi - psi_lo < 0.05f) {
        // Anchors too close in pressure to define a slope: fall back to the
        // legacy proportional ray off the high anchor rather than divide by ~0.
        for (int i = 0; i < (int)pm->num_points; i++)
            pm->throw_mm[i] = (psi_hi > 0) ? pm->pressure_psi[i] / psi_hi * throw_hi : 0.0f;
        return;
    }
    float slope = (throw_hi - throw_lo) / (psi_hi - psi_lo);
    for (int i = 0; i < (int)pm->num_points; i++) {
        float t = throw_lo + (pm->pressure_psi[i] - psi_lo) * slope;
        pm->throw_mm[i] = (t > 0.0f) ? t : 0.0f;
    }
}

// Adam 6114-E / v420 local fix: after /cal/valve the calibrated valve frame can
// put the active pressure range close to the encoder's 360->0 wrap. Keep scan
// analysis in an *unwrapped* angle frame relative to the commanded sweep angle;
// otherwise discovery can see det_start near 303deg and det_end near 28deg and
// skip the fine scan, collapsing pressure calibration to one point.
static float cal_unwrap_deg_near(float deg, float reference_deg)
{
    while (deg < reference_deg - 180.0f) deg += 360.0f;
    while (deg > reference_deg + 180.0f) deg -= 360.0f;
    return deg;
}

// Shared pressure calibration scan used by both terminal ('x') and web paths.
// Runs discovery, coarse, fine, gap-fill, return sweep, and post-process.
// When is_web=true, updates s_wcal.progress and s_wcal.msg for the web UI.
// Returns number of calibration points stored in map, or -1 if no water found.
static int cal_do_pressure_scan(pressure_map_t *map, bool is_web)
{
#define WP(pct, str)       do { if (is_web) { s_wcal.progress=(pct); \
                                snprintf(s_wcal.msg,sizeof(s_wcal.msg),"%s",(str)); } } while(0)
#define WPF(pct, fmt, ...) do { if (is_web) { s_wcal.progress=(pct); \
                                snprintf(s_wcal.msg,sizeof(s_wcal.msg),(fmt),##__VA_ARGS__); } } while(0)

    memset(map, 0, sizeof(*map));
    int n = 0;

    // ---- Phase 0: Discovery scan ----
    WP(5, "Discovery scan...");
    INFO("\n-- Phase 0: Discovery scan (%d steps at 10 deg from closed) --", CAL_DISC_STEPS);
    float disc_psi[CAL_DISC_STEPS], disc_act[CAL_DISC_STEPS];
    for (int di = 0; di < CAL_DISC_STEPS; di++) {
        float angle = VALVE_CLOSED_DEG + di * 10.0f;
        valve_goto(angle, 3.0f, 15000, false);
        valve_brake_hold();   // b376: brake-hold so the sample isn't taken while coasting/relaxing
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS / 2));
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        disc_act[di] = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), angle);
        disc_psi[di] = cal_read_pressure_avg();
        valve_brake_release();
        INFO("  %5.1f deg: %.4f PSI", disc_act[di], disc_psi[di]);
    }
    float baseline = disc_psi[0];
    for (int di = 1; di < CAL_DISC_STEPS; di++)
        if (disc_psi[di] >= 0.0f && disc_psi[di] < baseline) baseline = disc_psi[di];
    float det_thresh = baseline + 0.15f;
    float det_start = -1.0f, det_end = -1.0f, det_peak = baseline;
    float det_peak_angle = disc_act[0];
    for (int di = 0; di < CAL_DISC_STEPS; di++) {
        if (disc_psi[di] > det_thresh) {
            if (det_start < 0.0f) det_start = disc_act[di];
            det_end = disc_act[di];
            if (disc_psi[di] > det_peak) { det_peak = disc_psi[di]; det_peak_angle = disc_act[di]; }
        }
    }
    if (det_start < 0.0f) {
        INFO("*** No active pressure range found. Check water supply. ***");
        return -1;
    }
    float fine_start = fmaxf(0.0f,   det_start - 10.0f);
    // b430: no 359 cap -- det_end lives in the unwrapped frame now and can
    // legitimately exceed 360 (offset > ~+49 puts the active band across the
    // encoder wrap). VALVE_OPEN_DEG + 2 below is the frame-correct bound.
    float fine_end   = det_end + 10.0f;
    float fine_end_capped = fminf(fine_end, VALVE_OPEN_DEG + 2.0f);
    INFO("\nDetected: baseline=%.4f PSI  active=%.1f to %.1f deg  peak=%.4f PSI",
         baseline, det_start, det_end, det_peak);
    INFO("Suggested constants: VALVE_CAL_START_DEG %.1f  VALVE_OPEN_DEG %.1f (peak at %.1f)",
         fine_start, det_peak_angle + 10.0f, det_peak_angle);

    // ---- Phase 1: Coarse scan (zero-pressure region) ----
    WP(25, "Coarse scan...");
    INFO("\n-- Phase 1: Coarse scan (%.1f to %.1f deg at 10 deg steps) --", fine_start, det_start);
    for (float angle = fine_start; angle < det_start && n < CAL_MAX_POINTS; angle += 10.0f) {
        valve_goto(angle, 2.0f, 15000, false);
        valve_brake_hold();   // b376
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        // b430: unwrap so the stored map stays monotonic across the 360 wrap
        float actual_deg = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), angle);
        float psi = cal_read_pressure_avg();
        valve_brake_release();
        map->valve_deg[n]    = actual_deg;
        map->pressure_psi[n] = psi;
        map->throw_mm[n]     = 0.0f;
        INFO("  Point %2d [coarse]: %.1f deg  %.4f PSI", n+1, actual_deg, psi);
        n++;
    }

    // ---- Phase 1b: Low-flow-band fine scan (VALVE_CAL_START_DEG -> det_start) ----
    // b385: the 263->265 deg band carries real but sub-threshold flow -- the
    // pressures that water the innermost rings. Phase 1 coarse only sampled it
    // at 10 deg spacing, so cal_throw_to_valve_deg had no fine low-end anchors
    // to interpolate against and saturated (the ~982mm floor). Sample it finely
    // here, approaching every opening from CLOSED in one monotonic upward sweep
    // (b367 anti-backlash): gear backlash makes a valve driven DOWN to a small
    // opening land in a dead spot, so a low reading is only repeatable when the
    // opening is reached from below. throw is left 0 here -- the two-anchor fit
    // recomputes it from psi after the user's measurements.
    WP(32, "Low-flow-band scan...");
    INFO("\n-- Phase 1b: Low-flow-band fine scan (%.1f to %.1f deg at 0.5 deg, from closed) --",
         VALVE_CAL_START_DEG, det_start);
    valve_goto_ex(VALVE_CLOSED_DEG, 2.0f, 8000, false, -1);
    s_valve_last_dir = -1;
    vTaskDelay(pdMS_TO_TICKS(400));
    for (float angle = VALVE_CAL_START_DEG; angle < det_start - 0.25f && n < CAL_MAX_POINTS;
         angle += 0.5f) {
        valve_goto_ex(angle, 0.2f, 6000, false, 1);   // open upward -- never reverse
        s_valve_last_dir = 1;
        valve_brake_hold();
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        float actual_deg = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), angle);  // b430
        float psi = cal_read_pressure_avg();
        valve_brake_release();
        map->valve_deg[n]    = actual_deg;
        map->pressure_psi[n] = psi;
        map->throw_mm[n]     = 0.0f;
        INFO("  Point %2d [lowband]: %.1f deg  %.4f PSI", n+1, actual_deg, psi);
        n++;
    }

    // ---- Phase 2: Fine scan (active pressure range, opening direction) ----
    WP(40, "Fine scan...");
    INFO("\n-- Phase 2: Fine scan (%.1f to %.1f deg at %.0f deg steps) --",
         det_start, fine_end_capped, VALVE_CAL_STEP_DEG);
    int total_fine = (int)((fine_end_capped - det_start) / VALVE_CAL_STEP_DEG) + 1;
    int fi = 0;
    for (float angle = det_start; angle <= fine_end_capped && n < CAL_MAX_POINTS;
         angle += VALVE_CAL_STEP_DEG) {
        WPF(40 + fi * 30 / (total_fine > 0 ? total_fine : 1),
            "Fine scan %.0f deg...", angle);
        valve_goto(angle, 0.5f, 15000, false);
        valve_brake_hold();   // b376
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        float actual_deg = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), angle);  // b430
        float psi = cal_read_pressure_avg();
        valve_brake_release();
        // Skip saturated points: if PSI hasn't risen ≥0.05 from previous point
        // this slot would be a duplicate on the flat plateau -- save it for the low end.
        if (n > 0 && psi - map->pressure_psi[n-1] < 0.05f) {
            INFO("  [skip saturated] %.1f deg  %.4f PSI (delta <0.05)", actual_deg, psi);
            fi++; continue;
        }
        map->valve_deg[n]    = actual_deg;
        map->pressure_psi[n] = psi;
        map->throw_mm[n]     = 0.0f;
        INFO("  Point %2d [fine]:   %.1f deg  %.4f PSI", n+1, actual_deg, psi);
        n++; fi++;
    }
    map->num_points = n;

    // ---- Phase 3: Gap-fill (closing direction through hydrodynamic zone) ----
    // Gaps left by the opening-direction snap-through are filled from the closing
    // side, which is also the direction valve_goto_jog uses during watering.
    // The stall point at gap_lo is purged; closing-direction points replace it.
    WP(72, "Gap-fill scan...");
    {
        INFO("\n-- Phase 3: Gap-fill (closing direction through hydrodynamic zone) --");
        int orig_n = n;
        bool any_gap = false;
        int gap_lo_idx[8]; int num_gaps = 0;
        for (int gi = 0; gi < orig_n - 1 && n < CAL_MAX_POINTS; gi++) {
            float gap     = map->valve_deg[gi+1]    - map->valve_deg[gi];
            float psi_gap = map->pressure_psi[gi+1] - map->pressure_psi[gi];
            if (gap <= VALVE_CAL_STEP_DEG * 2.5f && psi_gap < 0.3f) continue;
            float gap_lo = map->valve_deg[gi];
            float gap_hi = map->valve_deg[gi+1];
            INFO("  Gap %.1f->%.1f deg (%.1f deg, +%.3f PSI) -- filling from high side",
                 gap_lo, gap_hi, gap, psi_gap);
            any_gap = true;
            if (num_gaps < 8) { gap_lo_idx[num_gaps++] = gi; }
            WPF(73, "Gap-fill %.0f-%.0f deg...", gap_lo, gap_hi);
            valve_goto_ex(gap_hi + 1.0f, 2.0f, 10000, false, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            for (float a = gap_hi - 0.5f; a > gap_lo + 0.5f && n < CAL_MAX_POINTS; a -= 1.0f) {
                valve_goto_ex(a, 2.0f, 8000, false, -1);
                valve_brake_hold();   // b376
                vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
                uint16_t raw = 0;
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float actual_deg = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), a);  // b430
                float psi = cal_read_pressure_avg();
                valve_brake_release();
                map->valve_deg[n]    = actual_deg;
                map->pressure_psi[n] = psi;
                map->throw_mm[n]     = 0.0f;
                INFO("  Point %2d [gap-fill]: %.1f deg (cmd %.1f)  %.4f PSI",
                     n+1, actual_deg, a, psi);
                n++;
            }
        }
        if (!any_gap)
            INFO("  No gaps > %.0fdeg or >0.3PSI -- scan looks complete.", VALVE_CAL_STEP_DEG * 2.5f);
        if (any_gap) {
            // Purge opening-direction stall points at the bottom edge of each gap
            bool keep[CAL_MAX_POINTS];
            for (int i = 0; i < n; i++) keep[i] = true;
            for (int g = 0; g < num_gaps; g++) {
                keep[gap_lo_idx[g]] = false;
                INFO("  Purge opening-dir stall pt %d (%.1f deg) -- closing-dir takes priority",
                     gap_lo_idx[g]+1, map->valve_deg[gap_lo_idx[g]]);
            }
            int j = 0;
            for (int i = 0; i < n; i++) if (keep[i]) {
                if (j != i) { map->valve_deg[j]=map->valve_deg[i];
                              map->pressure_psi[j]=map->pressure_psi[i];
                              map->throw_mm[j]=map->throw_mm[i]; }
                j++;
            }
            n = j;
            // Insertion sort
            for (int si = 1; si < n; si++) {
                float tv=map->valve_deg[si], tp=map->pressure_psi[si], tt=map->throw_mm[si];
                int sj = si - 1;
                while (sj >= 0 && map->valve_deg[sj] > tv) {
                    map->valve_deg[sj+1]=map->valve_deg[sj];
                    map->pressure_psi[sj+1]=map->pressure_psi[sj];
                    map->throw_mm[sj+1]=map->throw_mm[sj];
                    sj--;
                }
                map->valve_deg[sj+1]=tv; map->pressure_psi[sj+1]=tp; map->throw_mm[sj+1]=tt;
            }
            INFO("  Gap-fill: +%d points, -%d stall pts, total %d.",
                 n - orig_n + num_gaps, num_gaps, n);
        }
    }

    // ---- Return sweep: close -> open, average with opening readings ----
    WP(80, "Return sweep...");
    INFO("\nReturn sweep (open -> closed) for hysteresis averaging...");
    float psi_ret[CAL_MAX_POINTS];
    for (int i = 0; i < n; i++) psi_ret[i] = -1.0f;
    s_valve_last_dir = -1;
    for (int i = n - 1; i >= 0; i--) {
        valve_goto_ex(map->valve_deg[i], 2.0f, 15000, false, -1);
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        float actual_deg = cal_unwrap_deg_near(raw * (360.0f / 4096.0f), map->valve_deg[i]);  // b430
        psi_ret[i] = cal_read_pressure_avg();
        INFO("  Return pt %2d: target=%.1f actual=%.1f deg  %.3f PSI",
             i+1, map->valve_deg[i], actual_deg, psi_ret[i]);
    }
    INFO("Averaging opening and closing readings...");
    for (int i = 0; i < n; i++)
        if (psi_ret[i] >= 0.0f)
            map->pressure_psi[i] = (map->pressure_psi[i] + psi_ret[i]) * 0.5f;

    // ---- Post-process: sort, outlier rejection, dedup, monotone enforcement ----
    WP(95, "Cleaning cal data...");
    map->num_points = n;
    n = cal_post_process_scan(map);
    INFO("Scan complete: %d calibration points.", n);

#undef WP
#undef WPF
    return n;
}

static void phase_pressure_cal(void)
{
    STEP("Pressure Calibration Scan");
    INFO("CONNECT WATER SUPPLY before proceeding.");
    INFO("Press Enter to start, 'q' to abort.");
    while (true) {
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q') { INFO("Aborted."); return; }
        if (c == '\r' || c == '\n') break;
    }

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    pressure_map_t map = {0};
    int n = cal_do_pressure_scan(&map, false);
    if (n < 0) { motor_rail_off(); return; }

    // Pretty table
    INFO("\n=== Pressure Calibration (%d points) ===", n);
    INFO("  Pt  Valve(deg)  Avg(PSI)");
    INFO("  --  ----------  --------");
    for (int i = 0; i < n; i++)
        INFO("  %2d   %6.2f     %6.4f", i+1, map.valve_deg[i], map.pressure_psi[i]);
    tprintf("\r\npt\tvalve_deg\tavg_psi\r\n");
    for (int i = 0; i < n; i++)
        tprintf("%d\t%.2f\t%.4f\r\n", i+1, map.valve_deg[i], map.pressure_psi[i]);
    tprintf("\r\n");

    // b385: two-anchor throw. Low anchor first (short opening, from closed),
    // then the max anchor at full open. Captured into low_psi/low_throw.
    float low_psi = 0.0f, low_throw = 0.0f;
    INFO("\n-- Low throw measurement (short opening) --");
    INFO("Press Enter to open valve a little, or 'q' to skip (falls back to single-point).");
    tprintf("Ready> ");
    {
        int c = uart_getchar(UINT32_MAX);
        if (c == 'q' || c == 'Q') {
            INFO("Low throw skipped -- max-only proportional fit.");
        } else {
            INFO("Opening valve to short opening (from closed)...");
            water_seat_valve_from_closed(WATER_MIN_FLOW_PSI + 0.20f, &low_psi);
            INFO("Short opening pressure: %.4f PSI", low_psi);
            INFO("Measure the OUTER EDGE of the (short) spray ellipse.");
            INFO("Enter distance in mm, or feet with 'f' suffix. Enter to skip.");
            tprintf("> ");
            char buf[16]; int bi = 0;
            memset(buf, 0, sizeof(buf));
            while (bi < (int)sizeof(buf)-1) {
                int ch = uart_getchar(UINT32_MAX);
                if (ch == 'q' || ch == 'Q') { printf("\r\n"); break; }
                if (ch == '\r' || ch == '\n') { printf("\r\n"); break; }
                if (ch == 8 || ch == 127) { if (bi > 0) { bi--; buf[bi]=0; printf("\b \b"); } continue; }
                buf[bi++] = (char)ch; printf("%c", ch);
            }
            if (bi > 0) {
                bool in_feet = (buf[bi-1] == 'f' || buf[bi-1] == 'F');
                if (in_feet) buf[bi-1] = '\0';
                float dist = atof(buf);
                if (in_feet) dist *= 304.8f;
                if (dist > 100.0f && dist < 15000.0f) {
                    low_throw = dist;
                    INFO("Low throw: %.0f mm (%.1f ft) at %.4f PSI", dist, dist/304.8f, low_psi);
                } else {
                    INFO("Value out of range -- low anchor skipped.");
                    low_psi = 0.0f;
                }
            } else {
                low_psi = 0.0f;
            }
        }
    }

    // Max throw measurement
    INFO("\n-- Max throw measurement --");
    INFO("Press Enter to open valve fully, or 'q' to skip.");
    tprintf("Ready> ");
    {
        int c = uart_getchar(UINT32_MAX);
        if (c == 'q' || c == 'Q') {
            INFO("Skipped -- use 'c' menu to enter throw distances manually.");
            goto skip_throw;
        }
    }
    INFO("Opening valve to full open (%.1f deg)...", VALVE_OPEN_DEG);
    valve_goto(VALVE_OPEN_DEG, 1.0f, 15000, false);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
    INFO("Full open pressure: %.4f PSI", cal_read_pressure_avg());
    INFO("Measure the OUTER EDGE of the spray ellipse.");
    INFO("Enter distance in mm, or feet with 'f' suffix (e.g. 25.5f). Enter to skip.");
    tprintf("> ");
    {
        char buf[16]; int bi = 0;
        memset(buf, 0, sizeof(buf));
        while (bi < (int)sizeof(buf)-1) {
            int c = uart_getchar(UINT32_MAX);
            if (c == 'q' || c == 'Q') { printf("\r\n"); break; }
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) { if (bi > 0) { bi--; buf[bi]=0; printf("\b \b"); } continue; }
            buf[bi++] = (char)c; printf("%c", c);
        }
        if (bi > 0) {
            bool in_feet = (buf[bi-1] == 'f' || buf[bi-1] == 'F');
            if (in_feet) buf[bi-1] = '\0';
            float dist = atof(buf);
            if (in_feet) dist *= 304.8f;
            if (dist > 100.0f && dist < 15000.0f) {
                float psi_max_cal = map.pressure_psi[n-1];
                // b385: two-anchor fit (degenerates to proportional if the low
                // anchor was skipped, i.e. low_throw==0 && low_psi==0).
                cal_apply_two_anchor_throw(&map, low_psi, low_throw, psi_max_cal, dist);
                INFO("Max throw: %.0f mm (%.1f ft)", dist, dist/304.8f);
                tprintf("\r\npt\tvalve_deg\tavg_psi\tthrow_mm\tthrow_ft\r\n");
                for (int i = 0; i < n; i++)
                    tprintf("%d\t%.2f\t%.4f\t%.0f\t%.2f\r\n",
                            i+1, map.valve_deg[i], map.pressure_psi[i],
                            map.throw_mm[i], map.throw_mm[i]/304.8f);
                tprintf("\r\n");
            } else {
                INFO("Value out of range (100-15000 mm) -- skipped.");
            }
        }
    }
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);
    {
        esp_err_t r = cal_save_primary(&map);
        if (r == ESP_OK) { INFO("\nCalibration saved (%d points).", n); PASS("Pressure calibration complete"); }
        else { ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(r)); FAIL("Pressure calibration NVS save"); }
    }
skip_throw:;
    motor_rail_off();
}

static void phase_cal_view(void)
{
    STEP("View / Edit Calibration");
    sensor_rail_on();

    pressure_map_t map = {0};
    if (cal_load_primary(&map) != ESP_OK || map.num_points == 0) {
        INFO("No calibration data in NVS. Run pressure scan first ('x').");
        return;
    }

    INFO("=== Stored Calibration (%d points) ===", map.num_points);
    INFO("  Pt  Valve(deg)  Pressure(PSI)  Throw(mm)");
    INFO("  --  ----------  -------------  ---------");
    for (int i = 0; i < map.num_points; i++) {
        if (map.throw_mm[i] > 0)
            INFO("  %2d   %7.1f      %7.3f       %6.0f",
                 i+1, map.valve_deg[i], map.pressure_psi[i], map.throw_mm[i]);
        else
            INFO("  %2d   %7.1f      %7.3f          ---",
                 i+1, map.valve_deg[i], map.pressure_psi[i]);
    }

    INFO("\nEnter point number to set throw distance, or 'q' to quit:");
    char buf[16];
    while (true) {
        INFO("Point> ");
        int i = 0;
        memset(buf, 0, sizeof(buf));
        while (i < (int)sizeof(buf)-1) {
            int c = uart_getchar(30000);
            if (c == 0 || c == 'q' || c == 'Q') goto done;
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) { if (i > 0) { i--; buf[i]=0; printf("\b \b"); } continue; }
            buf[i++] = (char)c; printf("%c", c);
        }
        if (i == 0) continue;
        int pt = atoi(buf) - 1;
        if (pt < 0 || pt >= map.num_points) { INFO("Invalid point."); continue; }

        INFO("Throw distance for point %d (mm)> ", pt+1);
        i = 0; memset(buf, 0, sizeof(buf));
        while (i < (int)sizeof(buf)-1) {
            int c = uart_getchar(30000);
            if (c == 0 || c == 'q' || c == 'Q') goto done;
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) { if (i > 0) { i--; buf[i]=0; printf("\b \b"); } continue; }
            buf[i++] = (char)c; printf("%c", c);
        }
        if (i == 0) continue;
        float mm = atof(buf);
        if (mm <= 0) { INFO("Invalid distance."); continue; }
        map.throw_mm[pt] = mm;
        INFO("  Set point %d throw = %.0f mm", pt+1, mm);

        // Auto-save
        cal_save_primary(&map);
        INFO("  Saved.");
    }
done:
    INFO("Done.");
}



// Look up approximate valve angle for a target pressure using calibration table.
// Linear interpolation between nearest two calibration points.
// Returns -1 if no calibration data available.
static float cal_pressure_to_valve_deg(float target_psi)
{
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) return -1.0f;

    // Find bracketing points (table goes from open/low-pressure to closed/high-pressure
    // or vice versa — sort by pressure to be safe)
    int n = cal.num_points;

    // Find the two points that bracket target_psi
    for (int i = 0; i < n - 1; i++) {
        float p0 = cal.pressure_psi[i];
        float p1 = cal.pressure_psi[i + 1];
        float v0 = cal.valve_deg[i];
        float v1 = cal.valve_deg[i + 1];

        // Check if target is between these two points
        if ((target_psi >= p0 && target_psi <= p1) ||
            (target_psi <= p0 && target_psi >= p1)) {
            float t = (target_psi - p0) / (p1 - p0);
            return v0 + t * (v1 - v0);
        }
    }

    // Clamp to nearest endpoint
    float pmin = cal.pressure_psi[0], pmax = cal.pressure_psi[n-1];
    if (pmin > pmax) { float tmp = pmin; pmin = pmax; pmax = tmp; }
    if (target_psi <= pmin) {
        // Return valve angle of lowest pressure point
        return (cal.pressure_psi[0] < cal.pressure_psi[n-1]) ?
               cal.valve_deg[0] : cal.valve_deg[n-1];
    }
    return (cal.pressure_psi[0] > cal.pressure_psi[n-1]) ?
           cal.valve_deg[0] : cal.valve_deg[n-1];
}


// Interpolate throw distance (mm) from pressure using calibration table.
// Returns 0 if no throw data available.
static float cal_pressure_to_throw_mm(float psi)
{
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) return 0.0f;
    int n = cal.num_points;
    // Find first and last calibrated points for clamping
    float psi_lo = 1e9f, psi_hi = -1e9f, t_lo = 0, t_hi = 0;
    for (int i = 0; i < n; i++) {
        if (cal.throw_mm[i] <= 0) continue;
        if (cal.pressure_psi[i] < psi_lo) { psi_lo=cal.pressure_psi[i]; t_lo=cal.throw_mm[i]; }
        if (cal.pressure_psi[i] > psi_hi) { psi_hi=cal.pressure_psi[i]; t_hi=cal.throw_mm[i]; }
    }
    // Clamp out-of-range PSI to boundary throw values instead of returning 0
    if (psi <= psi_lo) return t_lo;
    if (psi >= psi_hi) return t_hi;
    for (int i = 0; i < n - 1; i++) {
        float p0 = cal.pressure_psi[i],   p1 = cal.pressure_psi[i+1];
        float t0 = cal.throw_mm[i],        t1 = cal.throw_mm[i+1];
        if (t0 <= 0 || t1 <= 0) continue;
        if ((psi >= p0 && psi <= p1) || (psi <= p0 && psi >= p1)) {
            float t = (psi - p0) / (p1 - p0);
            return t0 + t * (t1 - t0);
        }
    }
    return t_hi;  // fallback: should not reach here
}

// Interpolate throw distance (mm) from valve angle using calibration table.
// Used by zone setup open-loop valve control to estimate throw when water is off.
static float cal_valve_deg_to_throw_mm(float vdeg)
{
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) return 0.0f;
    int n = cal.num_points;
    if (vdeg <= cal.valve_deg[0])   return cal.throw_mm[0];
    if (vdeg >= cal.valve_deg[n-1]) return cal.throw_mm[n-1];
    for (int i = 0; i < n - 1; i++) {
        if (vdeg >= cal.valve_deg[i] && vdeg <= cal.valve_deg[i+1]) {
            float t = (vdeg - cal.valve_deg[i]) / (cal.valve_deg[i+1] - cal.valve_deg[i]);
            return cal.throw_mm[i] + t * (cal.throw_mm[i+1] - cal.throw_mm[i]);
        }
    }
    return cal.throw_mm[n-1];
}

// ─────────────────────────────────────────────────────────────────────────────
// Reverse of cal_pressure_to_throw_mm: PSI for a given throw_mm.
// Used by pressure PID in nozzle_sweep_pulse.
static float cal_throw_to_psi(float throw_mm)
{
    pressure_map_t c = {0};
    if (cal_load_primary(&c) != ESP_OK || c.num_points < 2)
        return throw_mm / 1359.0f;
    for (int i = 0; i < c.num_points - 1; i++) {
        float t0=c.throw_mm[i], t1=c.throw_mm[i+1];
        float p0=c.pressure_psi[i], p1=c.pressure_psi[i+1];
        if (t0<=0||t1<=0) continue;
        float lo=fminf(t0,t1), hi=fmaxf(t0,t1);
        if (throw_mm>=lo && throw_mm<=hi) {
            float frac=(hi>lo)?(throw_mm-t0)/(t1-t0):0.0f;
            return p0+frac*(p1-p0);
        }
    }
    return throw_mm / 1359.0f;
}

// Interpolate flow rate (L/min) from pressure percentage.
// Confirmed linear fit: flow_lpm = 0.12 * pressure_pct + 1.0
// Measured: 100% = 13 L/min, 50% = 7 L/min
static float cal_pressure_to_flow_lpm(float pressure_pct)
{
    if (pressure_pct <= 0.0f)   return 1.0f;
    if (pressure_pct >= 100.0f) return 13.0f;
    return 0.12f * pressure_pct + 1.0f;
}

// Watering time (minutes) for a ring given area, target depth and pressure %.
static float cal_watering_time_min(float ring_area_m2, float target_depth_mm,
                                   float pressure_pct)
{
    float flow = cal_pressure_to_flow_lpm(pressure_pct);
    if (flow <= 0.0f) return 0.0f;
    return (target_depth_mm * ring_area_m2) / flow;
}

// b281: Estimate cal-time supply pressure (pre-valve) from a per-zone cal
// walk plus a query (valve_deg, nozzle_psi). Mathematical basis:
//   P_nozzle = P_supply * f(valve_deg)
//   f(valve_deg) is a hardware property of the valve geometry
//
// The cal walk records (valve_deg, nozzle_psi) at varying valve positions and
// varying supply pressure (since the well pump cycles during cal too). To
// extract supply we use the most-open valve_deg samples as supply anchors
// (where f ~= 1, so nozzle_psi ~= supply); interpolate supply by walk_idx
// for samples at other valve_deg; then back-compute f for each valve_deg.
//
// Returns supply estimate in PSI, or 0.0 if cal is insufficient (need >= 2
// distinct valve_deg values with at least 1 sample at the most-open).
//
// For runtime queries: pass the watering ring's valve_deg and avg_psi;
// returned supply estimate tells you what the pump/tank was actually
// providing during that ring's sweep. Compare across rings (or across runs)
// to see how supply pressure swings with pump cycling.
static float cal_estimate_f_at_valve_deg(const zone_perimeter_t *z,
                                          float query_valve_deg)
{
    if (!z || z->num_points < 2) return 1.0f;

    // Pass 1: find the most-open valve_deg (largest valve_deg value), and
    // collect supply anchors -- the nozzle_psi at those samples is taken as
    // a proxy for supply pressure at that walk_idx (f ~= 1 at full open).
    float max_vd = -1e9f;
    for (int i = 0; i < z->num_points; i++) {
        if (z->points[i].valve_deg > max_vd) max_vd = z->points[i].valve_deg;
    }
    if (max_vd < 0.0f) return 1.0f;

    // Tag the anchor samples (within 0.5 deg of max_vd treated as fully open).
    // For each anchor: walk_idx -> supply_psi proxy.
    // For non-anchor samples: walk_idx -> nozzle_psi at their valve_deg.
    struct { float walk_idx; float val; } anchors[CAL_MAX_POINTS];
    int n_anchors = 0;
    for (int i = 0; i < z->num_points && n_anchors < CAL_MAX_POINTS; i++) {
        if (z->points[i].valve_deg >= max_vd - 0.5f) {
            anchors[n_anchors].walk_idx = (float)z->points[i].walk_idx;
            anchors[n_anchors].val      = z->points[i].pressure_psi;
            n_anchors++;
        }
    }
    if (n_anchors < 1) return 1.0f;

    // For the query: find the nearest two cal valve_deg "bins" surrounding
    // query_valve_deg, compute f at each bin (averaging samples within that
    // bin against interpolated supply), then linearly interpolate f().
    //
    // Collect distinct valve_deg bins (sorted ascending).
    float bins[CAL_MAX_POINTS]; int n_bins = 0;
    for (int i = 0; i < z->num_points; i++) {
        float v = z->points[i].valve_deg;
        bool seen = false;
        for (int j = 0; j < n_bins; j++) {
            if (fabsf(bins[j] - v) < 0.5f) { seen = true; break; }
        }
        if (!seen && n_bins < CAL_MAX_POINTS) bins[n_bins++] = v;
    }
    // simple insertion sort, n is small (<= cal point count)
    for (int i = 1; i < n_bins; i++) {
        float k = bins[i]; int j = i - 1;
        while (j >= 0 && bins[j] > k) { bins[j+1] = bins[j]; j--; }
        bins[j+1] = k;
    }

    // f-at-bin computation: for each sample at this bin, interpolate supply
    // from the anchor list using walk_idx, then f_sample = nozzle/supply.
    // Average across samples in the bin.
    float f_bins[CAL_MAX_POINTS] = {0};
    for (int b = 0; b < n_bins; b++) {
        float sum_f = 0.0f; int n_f = 0;
        for (int i = 0; i < z->num_points; i++) {
            if (fabsf(z->points[i].valve_deg - bins[b]) > 0.5f) continue;
            // Interpolate supply at this walk_idx from anchors
            float wi = (float)z->points[i].walk_idx;
            float supply = 0.0f;
            if (n_anchors == 1) {
                supply = anchors[0].val;
            } else {
                // Find anchor pair surrounding wi
                float lo_wi = -1e9f, lo_v = 0; float hi_wi = 1e9f, hi_v = 0;
                bool have_lo = false, have_hi = false;
                for (int a = 0; a < n_anchors; a++) {
                    if (anchors[a].walk_idx <= wi && anchors[a].walk_idx > lo_wi) {
                        lo_wi = anchors[a].walk_idx; lo_v = anchors[a].val;
                        have_lo = true;
                    }
                    if (anchors[a].walk_idx >= wi && anchors[a].walk_idx < hi_wi) {
                        hi_wi = anchors[a].walk_idx; hi_v = anchors[a].val;
                        have_hi = true;
                    }
                }
                if (have_lo && have_hi && hi_wi > lo_wi) {
                    float t = (wi - lo_wi) / (hi_wi - lo_wi);
                    supply = lo_v + t * (hi_v - lo_v);
                } else if (have_lo) {
                    supply = lo_v;
                } else if (have_hi) {
                    supply = hi_v;
                }
            }
            if (supply > 0.5f && z->points[i].pressure_psi > 0.05f) {
                float f = z->points[i].pressure_psi / supply;
                if (f > 0.05f && f <= 1.05f) { sum_f += f; n_f++; }
            }
        }
        f_bins[b] = n_f > 0 ? sum_f / n_f : 1.0f;
    }

    // Interpolate f at query_valve_deg using f_bins[].
    if (n_bins == 0) return 1.0f;
    if (n_bins == 1) return f_bins[0];
    if (query_valve_deg <= bins[0])         return f_bins[0];
    if (query_valve_deg >= bins[n_bins-1])  return f_bins[n_bins-1];
    for (int b = 0; b < n_bins - 1; b++) {
        if (query_valve_deg >= bins[b] && query_valve_deg <= bins[b+1]) {
            float t = (query_valve_deg - bins[b]) / (bins[b+1] - bins[b]);
            return f_bins[b] + t * (f_bins[b+1] - f_bins[b]);
        }
    }
    return f_bins[n_bins-1];
}

// Wrapper: supply_psi = nozzle_psi / f(valve_deg). Returns 0 if cal can't
// give a meaningful f (avoids divide-by-zero garbage in HA dashboards).
static float cal_estimate_supply_psi(const zone_perimeter_t *z,
                                     float valve_deg, float nozzle_psi)
{
    if (!z || z->num_points < 2 || nozzle_psi < 0.05f) return 0.0f;
    float f = cal_estimate_f_at_valve_deg(z, valve_deg);
    if (f < 0.05f) return 0.0f;
    return nozzle_psi / f;
}

// b284: Build f(valve_deg) from the device's global pressure_map_t cal table
// (loaded via cal_load_primary). This is preferred over the zone perimeter
// cal because:
//   - pressure_map_t has more cal points (typically 29 vs 11 in zone perim)
//   - they're monotonically sorted by valve_deg (clean interpolation)
//   - they cover a wider valve_deg range
//   - one source-of-truth across all zones rather than per-zone variation
//
// Treats the max-pressure cal sample as the supply anchor (f=1.0 there).
// Linear-interpolates between cal points; clamps at the extremes.
// Returns the supply pressure estimate, or 0.0 if cal is unavailable or
// the query is outside the meaningful range.
//
// This is the production replacement for cal_estimate_supply_psi() above
// (which used zone perim cal). Both remain in the codebase for now -- the
// zone-perim path is kept as a fallback in case pressure_map_t cal is
// missing, but new code paths should call this function.
static float supply_psi_from_pressure_map(float valve_deg, float nozzle_psi)
{
    if (nozzle_psi < 0.05f) return 0.0f;

    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) return 0.0f;
    int n = cal.num_points;

    // Find max pressure across the cal table (the supply anchor: f=1.0 here).
    float max_psi = 0.0f;
    for (int i = 0; i < n; i++) {
        if (cal.pressure_psi[i] > max_psi) max_psi = cal.pressure_psi[i];
    }
    if (max_psi < 0.5f) return 0.0f;  // pathological cal -- give up

    // Linear-interpolate pressure at the query valve_deg. Cal table is
    // assumed sorted ascending by valve_deg (verified by cal_export paths).
    // If unsorted, the loop below still produces a reasonable answer because
    // we test the bracket condition both ways.
    float interp_psi;
    if (valve_deg <= cal.valve_deg[0]) {
        interp_psi = cal.pressure_psi[0];
    } else if (valve_deg >= cal.valve_deg[n - 1]) {
        interp_psi = cal.pressure_psi[n - 1];
    } else {
        interp_psi = cal.pressure_psi[n - 1];  // fallback if no bracket found
        for (int i = 0; i < n - 1; i++) {
            float lo = cal.valve_deg[i];
            float hi = cal.valve_deg[i + 1];
            if (valve_deg >= lo && valve_deg <= hi && hi > lo) {
                float t = (valve_deg - lo) / (hi - lo);
                interp_psi = cal.pressure_psi[i]
                           + t * (cal.pressure_psi[i + 1] - cal.pressure_psi[i]);
                break;
            }
        }
    }

    float f = interp_psi / max_psi;
    if (f < 0.05f) return 0.0f;  // valve too closed for meaningful estimate
    return nozzle_psi / f;
}

// b284: Cal-time supply anchor for the pressure_map_t cal -- simply the
// max pressure in the table (the value we anchor f=1.0 to). Used as the
// reference for the rings_supply_limited gap analysis (b282), so it
// matches the f() source we're using for runtime supply estimates.
static float cal_anchor_supply_pressure_map(void)
{
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 1) return 0.0f;
    float max_psi = 0.0f;
    for (int i = 0; i < cal.num_points; i++) {
        if (cal.pressure_psi[i] > max_psi) max_psi = cal.pressure_psi[i];
    }
    return max_psi;
}

// b296: Option C-Full -- supply-aware ring scheduler for smooth mode.
// Replaces the fixed-order ring iteration within each pass with a
// scheduler that picks the next ring based on observed supply state.
//
// Hypothesis (validated by b281/b282 across three zones): outer rings
// (long throw, need high nozzle pressure) under-deliver when the pump
// is depleted; inner rings (short throw, restricted valve) don't care
// either way. Watering outer rings preferentially when supply is fresh
// AND inner rings when supply has sagged should reduce the
// rings_supply_limited count by matching each ring to a supply state
// it can actually use.
//
// Algorithm per ring decision:
//   1. Classify current supply: HIGH if current_supply >= cal_supply * 0.9
//   2. For each unvisited ring with depth_remaining > 0:
//        ring_needs_high = (throw_mm[ring] >= median_throw)
//        supply_match    = (ring_needs_high == supply_high) ? 1.0 : 0.4
//        score           = deficit_frac * supply_match
//   3. Pick highest-score ring.
//
// Bootstrap: first ring of pass 0 has no supply estimate -- caller
// passes cal_supply > 0 == false to indicate "supply unknown," and we
// default to HIGH (pump assumed fresh).
//
// Visited[]: prevents picking the same ring twice within a pass. Across
// passes the caller resets visited[] so rings can be revisited if they
// still have deficit. Preserves the "one logical pass = each ring at
// most once" semantics that other code depends on (calibration
// updates, valve_corr machinery, termination check).
//
// Returns ring index 0..num_rings-1, or -1 if no remaining work.
static int smooth_scheduler_pick(const bool *visited, int num_rings,
                                  const float *ring_throws,
                                  const float *last_actual_throw,
                                  const float *cum_depth, float depth_target,
                                  float current_supply, float cal_supply,
                                  float median_throw, int pass)
{
    // b297: bootstrap fix. The check `current_supply >= cal_supply * 0.90`
    // evaluates to false when current_supply is 0 (no readings yet), which
    // misclassified pass-0 as LOW supply and caused pass 0 to start with
    // a LOW-need (mid) ring instead of the outer ring. Default HIGH when
    // we have no real reading yet (current_supply <= 0).
    bool supply_high = (current_supply <= 0.0f || cal_supply <= 0.5f)
                       ? true
                       : (current_supply >= cal_supply * 0.90f);

    int best = -1;
    float best_score = -1.0f;

    for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
        if (visited[i]) continue;

        float depth_deficit = depth_target - cum_depth[i];
        float depth_def_frac = depth_deficit > 0.0f ? depth_deficit / depth_target : 0.0f;
        if (depth_def_frac > 1.0f) depth_def_frac = 1.0f;

        // b297: throw-aware re-pick. After pass 0 every ring has a recorded
        // actual_throw. If a ring under-threw (typically because it fired
        // at low supply), keep it eligible for re-firing even when its
        // cumulative depth target is met -- the final supply-limited
        // diagnostic is gated on BOTH throw_ratio<0.90 AND supply_ratio
        // <0.90 from the LAST fire, so re-firing at higher supply will
        // unset the flag (and the corr machinery gets another data point).
        float throw_def_frac = 0.0f;
        if (pass > 0 && ring_throws[i] >= 100.0f && last_actual_throw != NULL) {
            float at = last_actual_throw[i];
            if (at >= 100.0f) {
                float ratio = at / ring_throws[i];
                if (ratio < 0.90f) {
                    throw_def_frac = 1.0f - ratio;
                    if (throw_def_frac > 1.0f) throw_def_frac = 1.0f;
                }
            }
        }

        float deficit_frac = depth_def_frac;
        if (throw_def_frac > deficit_frac) deficit_frac = throw_def_frac;
        if (deficit_frac <= 0.0f) continue;   // both depth and throw satisfied

        bool ring_needs_high = (ring_throws[i] >= median_throw);
        float supply_match = (ring_needs_high == supply_high) ? 1.0f : 0.4f;
        float score = deficit_frac * supply_match;

        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

// b282: Returns the average cal-time supply pressure (PSI), computed as
// the mean nozzle pressure at the most-open valve_deg in the cal walk
// (where f~=1 so nozzle_psi is a clean supply proxy). Used as the
// reference against which runtime supply estimates are compared to flag
// "supply-limited" rings. Returns 0.0 if no anchors found.
static float cal_estimate_cal_time_supply_psi(const zone_perimeter_t *z)
{
    if (!z || z->num_points < 1) return 0.0f;
    float max_vd = -1e9f;
    for (int i = 0; i < z->num_points; i++) {
        if (z->points[i].valve_deg > max_vd) max_vd = z->points[i].valve_deg;
    }
    if (max_vd < 0.0f) return 0.0f;
    float sum = 0.0f; int n = 0;
    for (int i = 0; i < z->num_points; i++) {
        if (z->points[i].valve_deg >= max_vd - 0.5f &&
            z->points[i].pressure_psi > 0.05f) {
            sum += z->points[i].pressure_psi;
            n++;
        }
    }
    return n > 0 ? sum / n : 0.0f;
}

// b282: Decision logic for "is this ring under-delivering because the supply
// was depleted at the time it fired?" Two-condition gate:
//   1. actual throw fell more than 10% short of target
//   2. supply pressure during this ring was more than 10% below cal-time supply
// Both must be true to flag. A ring that under-threw at full supply (case 1
// without case 2) is a valve-correction problem, not a supply problem -- the
// existing smooth_valve_corr[] machinery handles that. A ring that operated
// at low supply but still hit its throw target (case 2 without case 1) didn't
// suffer -- maybe its throw target was modest enough that even depleted
// supply was sufficient.
//
// b283 will use the same predicate to decide which rings to re-water.
static bool ring_is_supply_limited(const water_ring_data_t *r,
                                    float cal_time_supply_psi)
{
    if (!r || r->throw_mm < 100.0f) return false;
    if (cal_time_supply_psi < 0.5f) return false; // no reference, can't judge
    if (r->supply_psi_est < 0.5f)   return false; // cal couldn't compute supply
    float throw_ratio  = r->actual_throw_mm / r->throw_mm;
    float supply_ratio = r->supply_psi_est / cal_time_supply_psi;
    return (throw_ratio < 0.90f) && (supply_ratio < 0.90f);
}

// Nozzle flow model: Q [mL/min] = k * psi^n  (measured once, applies to all units)
#define NOZZLE_FLOW_K  4542.0f   // mL/min at 1 PSI
#define NOZZLE_FLOW_N     0.566f // flow exponent (ideal orifice = 0.5)

// Precipitation depth [mm] deposited per pass by one ring.
// r_outer_mm -- outer radius of the ring's annular region [mm]
// r_inner_mm -- inner radius (= next ring's throw, or r_outer*0.92 for innermost) [mm]
// dps        -- nozzle sweep speed for this ring [deg/s]
// avg_psi    -- measured line pressure during this ring [PSI]
// Active arc angle cancels (more arc = more water AND more area), so depth
// depends only on flow rate, nozzle speed, and ring annular area.
// Uses the same exact annular area as the dps formula for consistency.
static float nozzle_precip_depth_mm(float r_outer_mm, float r_inner_mm,
                                     float dps, float avg_psi)
{
    float area_diff = r_outer_mm * r_outer_mm - r_inner_mm * r_inner_mm; /* mm² */
    if (dps < 0.01f || area_diff < 1.0f || avg_psi < 0.05f)
        return 0.0f;
    float Q = NOZZLE_FLOW_K * powf(avg_psi, NOZZLE_FLOW_N); /* mL/min */
    /* depth = Q[mL/min] * 6000 / (dps[deg/s] * pi * (r_out^2 - r_in^2)[mm^2]) */
    return Q * 6000.0f / (dps * (float)M_PI * area_diff);
}

// Zone perimeter definition
// ─────────────────────────────────────────────────────────────────────────────

// perimeter_point_t and zone_perimeter_t defined in irrigoto_types.h

// Forward declarations for LittleFS-first zone helpers
static esp_err_t zone_load_primary(uint16_t id, zone_perimeter_t *z);
static esp_err_t zone_save_primary(uint16_t id, const char *name_override, const zone_perimeter_t *z);
static bool      nozzle_sweep_pulse(float, float, bool, float, uint16_t, uint32_t,
                                    TickType_t, int, int, int, float, float, float*, int*, bool);

// ---- Last-watering record (actual per-ring PSI, DPS, active arc) --------
// water_ring_data_t and water_run_t defined in irrigoto_types.h

static water_run_t s_last_water_run;
// Completion-status snapshot. s_water_abort gets cleared at the start of
// the next run, so we capture it (and the active mode) at the moment a
// run ends so the HA "watering_complete" event can report accurately.
static bool        s_last_water_aborted = false;
static int         s_last_water_mode    = 0;
static uint16_t    s_last_water_zone_id = 0;

// Granular completion status. Set by water_set_status() during the run;
// snapshotted into s_last_water_status_code at run end.
// Codes:
//   0 = completed (default if nothing set abnormally)
//   1 = cancelled       (user pressed stop in UI/HA/UART)
//   2 = valve_fault     (valve motor failed to reach target — wiring TBD)
//   3 = nozzle_fault    (nozzle motor stalled — wiring TBD)
//   4 = water_loss      (supply pressure dropped mid-run after good flow)
#define WATER_STATUS_COMPLETED    0
#define WATER_STATUS_CANCELLED    1
#define WATER_STATUS_VALVE_FAULT  2
#define WATER_STATUS_NOZZLE_FAULT 3
#define WATER_STATUS_WATER_LOSS   4
static int  s_water_status_code      = WATER_STATUS_COMPLETED;
static int  s_last_water_status_code = WATER_STATUS_COMPLETED;
// Unix epoch when the last watering ended (set in the same block that
// snapshots zone_id/status). 0 = no watering completed since boot.
// Used by the landing-page schedule card to show "Last: <zone> ago Xm".
static time_t s_last_water_finish_epoch = 0;

// NVS-backed metadata so the "last completed zone" tag survives a
// deep-sleep wake. Persisted in lockstep with the in-RAM snapshot
// (zone_id / status_code / finish_epoch) at the end of every
// watering run, and restored once at irrigoto_init. Stored as a
// fixed-size blob under key "last_water" in the OtO namespace.
// The full s_last_water_run struct is NOT persisted here — it's
// big (rings, areas, throws) and only needed at the moment of
// completion to fire the HA event; after that, dashboards only
// care about zone+status+time, which fit in 16 bytes.
typedef struct {
    uint16_t zone_id;       // 0-based zone id, matches s_last_water_zone_id
    uint8_t  status_code;   // 0=completed,1=cancelled,...
    uint8_t  reserved;
    int64_t  finish_epoch;  // unix time when the run ended
} __attribute__((packed)) last_water_meta_t;

static void last_water_save_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    last_water_meta_t m = {
        .zone_id      = s_last_water_zone_id,
        .status_code  = (uint8_t)s_last_water_status_code,
        .reserved     = 0,
        .finish_epoch = (int64_t)s_last_water_finish_epoch,
    };
    nvs_set_blob(h, "last_water", &m, sizeof(m));
    nvs_commit(h);
    nvs_close(h);
}

static void last_water_load_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    last_water_meta_t m = {0};
    size_t sz = sizeof(m);
    if (nvs_get_blob(h, "last_water", &m, &sz) == ESP_OK && sz == sizeof(m)) {
        s_last_water_zone_id      = m.zone_id;
        s_last_water_status_code  = (int)m.status_code;
        s_last_water_finish_epoch = (time_t)m.finish_epoch;
    }
    nvs_close(h);
}
static bool s_water_run_had_flow     = false;
// Consecutive-no-flow streak. A single low PSI reading can be transient
// (water hammer, brief supply dip, sprinkler in friction-pause); require
// two rings in a row with no flow before declaring water_loss.
static int  s_water_no_flow_streak   = 0;
#define WATER_LOSS_STREAK_THRESHOLD 2

// Walk all polygon edges; find the one that straddles ring_throw and whose
// nozzle_deg crossing is closest to search_b (within search_range_deg).
// Returns search_b unchanged if no such edge exists (safe no-op fallback).
static float zone_refine_arc_bound(const zone_perimeter_t *zone, float ring_throw,
                                   float search_b, float search_range_deg)
{
    int n = zone->num_points;
    float best = search_b;
    float best_dist = search_range_deg + 1.0f;
    for (int i = 0; i < n; i++) {
        int   j  = (i + 1) % n;
        float b1 = zone->points[i].nozzle_deg, b2 = zone->points[j].nozzle_deg;
        float t1 = zone->points[i].throw_mm,   t2 = zone->points[j].throw_mm;
        if (fabsf(t2 - t1) < 1.0f) continue;
        if (ring_throw < fminf(t1, t2) || ring_throw > fmaxf(t1, t2)) continue;
        float frac = (ring_throw - t1) / (t2 - t1);
        float db = b2 - b1;
        if (db >  180.0f) db -= 360.0f;
        if (db < -180.0f) db += 360.0f;
        float cross_b = fmodf(b1 + frac * db + 360.0f, 360.0f);
        float dist = fminf(fmodf(cross_b - search_b + 360.0f, 360.0f),
                           fmodf(search_b - cross_b + 360.0f, 360.0f));
        if (dist < best_dist) { best_dist = dist; best = cross_b; }
    }
    return best;
}

// Build a named CSV path: /lfs/water/water_001_myzone.csv (or bare if no name).
static void water_csv_path_named(uint16_t id, char *buf, size_t len)
{
    // Look up zone name via public storage API.
    char zname[32] = {0};
    storage_zone_load(id, zname, sizeof(zname), NULL);
    // Build slug (lowercase, spaces→underscores).
    char slug[25] = {0}; size_t j = 0;
    for (size_t i = 0; zname[i] && j < sizeof(slug)-1; i++) {
        char c = zname[i];
        if (c >= 'a' && c <= 'z') { slug[j++] = c; }
        else if (c >= 'A' && c <= 'Z') { slug[j++] = c - 'A' + 'a'; }
        else if (c >= '0' && c <= '9') { slug[j++] = c; }
        else if ((c == ' ' || c == '-' || c == '_') && j > 0 && slug[j-1] != '_') {
            slug[j++] = '_';
        }
    }
    while (j > 0 && slug[j-1] == '_') j--;
    slug[j] = 0;
    if (slug[0])
        snprintf(buf, len, "/lfs/water/water_%03u_%s.wbin", id, slug);
    else
        snprintf(buf, len, "/lfs/water/water_%03u.wbin", id);
}

// Find an existing water log for zone id: prefers .wbin (binary), falls back to .csv (legacy text).
static void water_csv_path_find(uint16_t id, char *buf, size_t len)
{
    buf[0] = '\0';
    char prefix[13]; snprintf(prefix, sizeof(prefix), "water_%03u", id);
    size_t plen = strlen(prefix);
    char csv_fallback[300] = {0};
    DIR *d = opendir("/lfs/water");
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d))) {
            if (strncmp(ent->d_name, prefix, plen) != 0) continue;
            if (ent->d_name[plen] != '.' && ent->d_name[plen] != '_') continue;
            if (strstr(ent->d_name, ".wbin")) {
                snprintf(buf, len, "/lfs/water/%s", ent->d_name);
                closedir(d); return;   // binary wins immediately
            }
            if (!csv_fallback[0] && strstr(ent->d_name, ".csv"))
                snprintf(csv_fallback, sizeof(csv_fallback), "/lfs/water/%s", ent->d_name);
        }
        closedir(d);
    }
    if (csv_fallback[0]) { snprintf(buf, len, "%s", csv_fallback); return; }
    snprintf(buf, len, "/lfs/water/water_%03u.wbin", id);   // fallback (new file)
}

static void url_decode(char *buf, size_t len) {
    char tmp[64]; int j=0;
    for (int i=0; buf[i] && j<(int)len-1; i++) {
        if (buf[i]=='%' && buf[i+1] && buf[i+2]) {
            char hex[3]={buf[i+1],buf[i+2],0};
            tmp[j++]=(char)strtol(hex,NULL,16);
            i+=2;
        } else if (buf[i]=='+') {
            tmp[j++]=' ';
        } else {
            tmp[j++]=buf[i];
        }
    }
    tmp[j]=0; strncpy(buf,tmp,len-1); buf[len-1]=0;
}

static void zone_name_save_nvs(uint16_t id, const char *name) {
    char key[16]; snprintf(key, sizeof(key), "zname_%u", id);
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h)!=ESP_OK) return;
    nvs_set_str(h, key, name); nvs_commit(h); nvs_close(h);
}
static void zone_name_load_nvs(uint16_t id, char *buf, size_t len) {
    char key[16]; snprintf(key, sizeof(key), "zname_%u", id);
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h)!=ESP_OK) return;
    nvs_get_str(h, key, buf, &len); nvs_close(h);
}

// Resolve zone name: LittleFS header is primary; NVS is fallback.
static void zone_name_resolve(uint16_t id, const char *lfs_name,
                               const char *fallback, char *out, size_t len)
{
    if (lfs_name && lfs_name[0]) {
        strncpy(out, lfs_name, len-1); out[len-1]=0;
    } else {
        // LittleFS has no name -- try NVS migration fallback
        char nvs_name[32] = {0};
        zone_name_load_nvs(id, nvs_name, sizeof(nvs_name));
        if (nvs_name[0]) {
            strncpy(out, nvs_name, len-1); out[len-1]=0;
            // Promote NVS name into LittleFS so future loads use LittleFS
            if (storage_ready()) storage_zone_rename(id, nvs_name);
        } else {
            strncpy(out, fallback ? fallback : "Zone", len-1); out[len-1]=0;
        }
    }
}

static void device_name_save_nvs(const char *name) {
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h)!=ESP_OK) return;
    nvs_set_str(h, "device_name", name); nvs_commit(h); nvs_close(h);
}
static void device_name_load_nvs(char *buf, size_t len) {
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h)!=ESP_OK) return;
    nvs_get_str(h, "device_name", buf, &len); nvs_close(h);
}
static void device_name_init(void) {
    device_name_load_nvs(s_device_name, sizeof(s_device_name));
    if (!s_device_name[0]) {
        uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(s_device_name, sizeof(s_device_name), "irrigoto-%02X%02X", mac[4], mac[5]);
    }
}
// Sanitize a display name to a valid hostname (lowercase, spaces→dashes, drop other chars)
static void name_to_hostname(const char *name, char *out, size_t len) {
    int j=0;
    for (int i=0; name[i] && j<(int)len-1; i++) {
        char c=name[i];
        if (c>='A'&&c<='Z') c+=32;
        if ((c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-') out[j++]=c;
        else if (c==' '||c=='_') out[j++]='-';
    }
    if (!j) { strncpy(out,"oto",len-1); j=3; }
    out[j]=0;
}

// Average valve_deg from zone walk points whose throw is at or below a threshold.
// For circular zones all throws are equal and this returns the constant walk valve_deg.
// For non-uniform zones it averages the minimum-throw points (innermost boundary).
static float zone_avg_valve_deg(const zone_perimeter_t *z) {
    if (!z || z->num_points < 1) return -1.0f;
    float min_t = 1e9f;
    for (int i = 0; i < z->num_points; i++)
        if (z->points[i].throw_mm > 0 && z->points[i].throw_mm < min_t)
            min_t = z->points[i].throw_mm;
    float thr = min_t * 1.15f;  // include points within 15% of minimum
    float sum = 0; int cnt = 0;
    for (int i = 0; i < z->num_points; i++)
        if (z->points[i].throw_mm <= thr && z->points[i].valve_deg > 0)
            { sum += z->points[i].valve_deg; cnt++; }
    return cnt > 0 ? sum / cnt : z->points[0].valve_deg;
}

static void water_run_save_nvs(const water_run_t *r) {
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "last_water_run", r, sizeof(*r));
    nvs_commit(h); nvs_close(h);
}

static void water_run_load_nvs(water_run_t *r) {
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    // Check stored size matches current struct before loading.
    // A struct layout change (new fields) makes old blobs invalid.
    size_t stored_sz = 0;
    nvs_get_blob(h, "last_water_run", NULL, &stored_sz);  // size probe
    if (stored_sz == sizeof(*r)) {
        size_t sz = sizeof(*r);
        nvs_get_blob(h, "last_water_run", r, &sz);
    } else if (stored_sz > 0) {
        ESP_LOGW(TAG, "last_water_run NVS size mismatch (%u vs %u) -- ignoring stale data",
                 (unsigned)stored_sz, (unsigned)sizeof(*r));
    }
    nvs_close(h);
}

static esp_err_t zone_save_nvs(const zone_perimeter_t *z)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_blob(h, "zone_perimeter", z, sizeof(*z));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}

static esp_err_t zone_load_nvs(zone_perimeter_t *z)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    size_t sz = sizeof(*z);
    r = nvs_get_blob(h, "zone_perimeter", z, &sz);
    nvs_close(h);
    return r;
}

static void zone_print_points(const zone_perimeter_t *z)
{
    if (z->num_points == 0) {
        INFO("  (no points set)");
        return;
    }
    INFO("  Pt  Walk  Nozzle(deg)  Valve(deg)  Pressure(PSI)  Throw(mm)  Throw(ft)");
    INFO("  --  ----  -----------  ----------  -------------  ---------  ---------");
    for (int i = 0; i < z->num_points; i++) {
        float tm = z->points[i].throw_mm;
        if (tm <= 0.0f)
            tm = cal_pressure_to_throw_mm(z->points[i].pressure_psi);
        INFO("  %2d   %3u    %7.1f      %7.1f        %6.3f        %6.0f     %5.1f",
             i+1, (unsigned)z->points[i].walk_idx,
             z->points[i].nozzle_deg, z->points[i].valve_deg,
             z->points[i].pressure_psi, tm, tm / 304.8f);
    }
}


// Move valve to a target pressure expressed as % of calibrated max pressure.
// Max and min pressure come from stored calibration data.
static void phase_valve_pct(void)
{
    STEP("Valve Pressure % Target");

    // Load calibration
    pressure_map_t map = {0};
    if (cal_load_primary(&map) != ESP_OK || map.num_points < 2) {
        INFO("No calibration data found -- run pressure scan ('x') first.");
        return;
    }

    // Find min and max pressure from calibration
    float psi_min = map.pressure_psi[0];
    float psi_max = map.pressure_psi[0];
    for (int i = 1; i < map.num_points; i++) {
        if (map.pressure_psi[i] < psi_min) psi_min = map.pressure_psi[i];
        if (map.pressure_psi[i] > psi_max) psi_max = map.pressure_psi[i];
    }

    INFO("Calibration pressure range: %.3f PSI (min) to %.3f PSI (max)", psi_min, psi_max);
    INFO("Enter target as %% of max pressure (0-100), then Enter. 'q' to quit.");
    INFO("  0%%   = %.3f PSI (closed)", psi_min);
    INFO("  100%% = %.3f PSI (fully open)", psi_max);

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    // Close valve before starting
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);

    char buf[16];
    while (true) {
        INFO("Pct%%> ");
        int i = 0;
        memset(buf, 0, sizeof(buf));
        while (i < (int)sizeof(buf) - 1) {
            int c = uart_getchar(30000);
            if (c == 0 || c == 'q' || c == 'Q') goto done;
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) {
                if (i > 0) { i--; buf[i] = 0; printf(" "); }
                continue;
            }
            buf[i++] = (char)c;
            printf("%c", c);
            TOUCH_ACTIVITY();
        }
        if (i == 0) continue;

        float pct = atof(buf);
        if (pct < 0.0f || pct > 100.0f) {
            INFO("Invalid -- enter 0 to 100.");
            continue;
        }

        // Calculate target PSI
        float target_psi = psi_min + (pct / 100.0f) * (psi_max - psi_min);
        INFO("Target: %.1f%% = %.3f PSI", pct, target_psi);

        // Look up valve angle from calibration table
        float target_deg = cal_pressure_to_valve_deg(target_psi);
        if (target_deg < 0) {
            INFO("Cannot interpolate -- check calibration data.");
            continue;
        }

        INFO("Moving valve to %.1f deg (interpolated from cal table)...", target_deg);
        valve_goto(target_deg, 2.0f, 15000, false);

        // Settle and read actual pressure
        vTaskDelay(pdMS_TO_TICKS(1000));
        float actual_psi = 0;
        // Average a few readings
        float sum = 0; int ok = 0;
        for (int j = 0; j < 8; j++) {
            float p = 0;
            if (mprls_read(&p)) { sum += p; ok++; }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        actual_psi = ok ? sum / ok : -1.0f;

        float actual_pct = (psi_max > psi_min) ?
            ((actual_psi - psi_min) / (psi_max - psi_min)) * 100.0f : 0.0f;

        INFO("Actual: %.3f PSI = %.1f%% (target was %.1f%%)",
             actual_psi, actual_pct, pct);

        // Show throw estimate if available
        float throw_mm = cal_pressure_to_throw_mm(actual_psi);
        if (throw_mm > 0)
            INFO("Estimated throw: %.0f mm (%.1f ft)",
                 throw_mm, throw_mm / 304.8f);

        result(fabsf(actual_pct - pct) <= 10.0f, "Pressure within 10%% of target");
    }

done:
    motor_rail_off();
    INFO("Done. Valve left at last commanded position.");
}

// -----------------------------------------------------------------------
// Nozzle speed calibration
// Measures deg/sec at each PWM duty level, stores to NVS
// -----------------------------------------------------------------------




// Interpolate PWM duty for a target deg/sec from speed calibration table.
// Returns 0 if no calibration data.
static uint16_t __attribute__((unused)) spd_deg_per_sec_to_duty(float target_dps)
{
    speed_map_t m = {0};
    if (spd_load_primary(&m) != ESP_OK || m.num_points < 2) return 0;
    int n = m.num_points;
    // Clamp to measured range
    if (target_dps <= m.deg_per_sec[0])   return m.duty[0];
    if (target_dps >= m.deg_per_sec[n-1]) return m.duty[n-1];
    for (int i = 0; i < n - 1; i++) {
        float d0 = m.deg_per_sec[i],   d1 = m.deg_per_sec[i+1];
        float v0 = (float)m.duty[i],   v1 = (float)m.duty[i+1];
        if (target_dps >= d0 && target_dps <= d1) {
            float t = (target_dps - d0) / (d1 - d0);
            return (uint16_t)(v0 + t * (v1 - v0));
        }
    }
    return 0;
}

// Run nozzle at fixed duty for a timed window, return deg/sec.
// Uses a fixed 5-second measurement window for all duty levels.
// This handles slow speeds that would timeout waiting for a full rotation.
static float spd_measure_rotation(uint16_t duty)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    const uint32_t MEASURE_MS = 5000;  // fixed window -- works at any speed
    const float    MIN_DEG    = 2.0f;  // must move at least this much to count

    // Read starting position
    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) return -1.0f;
    float start_deg = raw * DEG_PER_COUNT;
    float travelled = 0.0f;
    float prev_deg  = start_deg;

    // Start nozzle motor at given duty (CW direction; polarity-resolved b419)
    gpio_set_direction(NOZZLE_PIN_REV, GPIO_MODE_OUTPUT);
    gpio_set_level(NOZZLE_PIN_REV, 0);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .period_ticks = 500,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&tcfg, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t ocfg = { .group_id = 0 };
    mcpwm_new_operator(&ocfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper, &ccfg, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, duty);
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gcfg = { .gen_gpio_num = NOZZLE_PIN_FWD };
    mcpwm_new_generator(oper, &gcfg, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // Kickstart: briefly run at 40% duty to overcome static friction,
    // then drop to target duty before measuring.
    if (duty < 150) {  // only needed below ~30% where static friction matters
        mcpwm_comparator_set_compare_value(cmpr, 200);  // 40% kickstart
        vTaskDelay(pdMS_TO_TICKS(150));                 // 150ms kick
        mcpwm_comparator_set_compare_value(cmpr, duty); // drop to target
        vTaskDelay(pdMS_TO_TICKS(200));                 // settle at target speed
    }

    // Read position after kickstart to establish clean baseline
    as5600_read(ADDR_AS5600, &raw, NULL, NULL);
    prev_deg = raw * DEG_PER_COUNT;
    travelled = 0.0f;

    TickType_t t_start = xTaskGetTickCount();

    while (pdTICKS_TO_MS(xTaskGetTickCount() - t_start) < MEASURE_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) break;
        float cur_deg = raw * DEG_PER_COUNT;
        float delta = cur_deg - prev_deg;
        if (delta < -180.0f) delta += 360.0f;
        if (delta >  180.0f) delta -= 360.0f;
        travelled += delta;
        prev_deg = cur_deg;
    }

    // Stop motor
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(GPIO_NFWD, 0);
    gpio_set_level(GPIO_NREV, 0);

    if (fabsf(travelled) < MIN_DEG) return -1.0f;  // no movement -- stalled

    float elapsed_sec = MEASURE_MS / 1000.0f;
    return fabsf(travelled) / elapsed_sec;
}

// Same as spd_measure_rotation but drives NREV (CCW direction).
static float spd_measure_rotation_ccw(uint16_t duty)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    const uint32_t MEASURE_MS = 5000;
    const float    MIN_DEG    = 2.0f;

    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) return -1.0f;
    float start_deg = raw * DEG_PER_COUNT;
    float travelled = 0.0f;
    float prev_deg  = start_deg;

    // Drive NREV (CCW), hold NFWD low (polarity-resolved b419)
    gpio_set_direction(NOZZLE_PIN_FWD, GPIO_MODE_OUTPUT);
    gpio_set_level(NOZZLE_PIN_FWD, 0);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .period_ticks = 500,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&tcfg, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t ocfg = { .group_id = 0 };
    mcpwm_new_operator(&ocfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper, &ccfg, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, duty);
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gcfg = { .gen_gpio_num = NOZZLE_PIN_REV };
    mcpwm_new_generator(oper, &gcfg, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // Kickstart
    mcpwm_comparator_set_compare_value(cmpr, 480);
    vTaskDelay(pdMS_TO_TICKS(60));
    mcpwm_comparator_set_compare_value(cmpr, duty);
    vTaskDelay(pdMS_TO_TICKS(200));

    uint32_t t_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t_start) * portTICK_PERIOD_MS < MEASURE_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        uint16_t r2 = 0;
        if (!as5600_read(ADDR_AS5600, &r2, NULL, NULL)) continue;
        float cur_deg = r2 * DEG_PER_COUNT;
        float delta = cur_deg - prev_deg;
        if (delta < -180.0f) delta += 360.0f;
        if (delta >  180.0f) delta -= 360.0f;
        travelled += delta;
        prev_deg = cur_deg;
    }

    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(GPIO_NREV, 0);
    gpio_set_level(GPIO_NFWD, 0);

    if (fabsf(travelled) < MIN_DEG) return -1.0f;
    return fabsf(travelled) / (MEASURE_MS / 1000.0f);
}

static void phase_jog_pulse_cal(void)
{
    STEP("Nozzle Jog Pulse Calibration (G)");
    INFO("Sweeps the nozzle 30 deg CW+CCW under water load to measure deg/pulse.");
    INFO("Valve opens to ~40%% pressure -- connect water supply.");
    INFO("Run AFTER nozzle speed cal (k).");
    if (!s_jog_web_mode) {
        INFO("Press Enter to start, q to abort.");
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q' || c == 0) { INFO("Aborted."); return; }
    }

    sensor_rail_on();
    motor_rail_on();
    adc_setup();

    // Load speed cal
    speed_map_t m = {0};
    bool have_spd = (spd_load_primary(&m) == ESP_OK && m.num_points > 0);
    uint16_t pulse_duty = (have_spd && m.jog_pulse_duty > 0) ? m.jog_pulse_duty : 200;
    uint32_t pulse_ms   = (have_spd && m.jog_pulse_ms   > 0) ? (uint32_t)m.jog_pulse_ms : 80;
    INFO("jog_pulse_duty=%u  jog_pulse_ms=%u", pulse_duty, pulse_ms);

    // Open valve to ~40%% pressure
    pressure_map_t pcal = {0};
    bool have_pcal = (cal_load_primary(&pcal) == ESP_OK && pcal.num_points >= 2);
    float psi_min = 0.05f, psi_max = 8.0f;
    if (have_pcal) {
        psi_min = psi_max = pcal.pressure_psi[0];
        for (int ci = 1; ci < pcal.num_points; ci++) {
            if (pcal.pressure_psi[ci] < psi_min) psi_min = pcal.pressure_psi[ci];
            if (pcal.pressure_psi[ci] > psi_max) psi_max = pcal.pressure_psi[ci];
        }
        float target = psi_max * 0.40f;
        INFO("Opening valve to %.3f PSI (40%% of max)...", target);
        water_hold_pressure(target, psi_min, psi_max);
        INFO("Actual: %.3f PSI", cal_read_pressure_avg());
    } else {
        INFO("No pressure cal -- measuring dry.");
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    // Measure CW: sweep 30 deg using nozzle_sweep_pulse, count pulses via deg_per_pulse log
    float TEST_ARC = 30.0f;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Opening valve and sweeping CW 30 deg...");
    s_wcal.progress = 20;
    INFO("Sweeping CW %.0f deg via pulse mode...", TEST_ARC);

    uint16_t raw0 = 0;
    as5600_read(ADDR_AS5600, &raw0, NULL, NULL);
    float origin = raw0 * (360.0f / 4096.0f);
    TickType_t t0 = xTaskGetTickCount();

    float psi_sum = 0.0f; int psi_n = 0;
    nozzle_sweep_pulse(origin, TEST_ARC, true, 1.0f,
                       pulse_duty, pulse_ms,
                       t0, 0, 1, 0,
                       VALVE_CLOSED_DEG,
                       0.0f, &psi_sum, &psi_n, false);

    uint16_t raw1 = 0;
    as5600_read(ADDR_AS5600, &raw1, NULL, NULL);
    float end_pos = raw1 * (360.0f / 4096.0f);
    float actual_cw = fmodf(end_pos - origin + 360.0f, 360.0f);
    float elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
    float pulses_est = elapsed_ms / (float)(pulse_ms + 120 + 20); // approx pulse count
    float dpp_cw = (pulses_est > 0) ? actual_cw / pulses_est : 0;

    INFO("CW: swept %.1f deg in %.0f ms  (~%.0f pulses)  => %.3f deg/pulse",
         actual_cw, elapsed_ms, pulses_est, dpp_cw);

    // Measure CCW
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Sweeping CCW 30 deg...");
    s_wcal.progress = 60;
    INFO("Sweeping CCW %.0f deg...", TEST_ARC);
    as5600_read(ADDR_AS5600, &raw0, NULL, NULL);
    origin = raw0 * (360.0f / 4096.0f);
    t0 = xTaskGetTickCount();
    psi_sum = 0.0f; psi_n = 0;
    nozzle_sweep_pulse(origin, TEST_ARC, false, 1.0f,
                       pulse_duty, pulse_ms,
                       t0, 0, 1, 0,
                       VALVE_CLOSED_DEG,
                       0.0f, &psi_sum, &psi_n, false);

    as5600_read(ADDR_AS5600, &raw1, NULL, NULL);
    float end_ccw = raw1 * (360.0f / 4096.0f);
    float actual_ccw = fmodf(origin - end_ccw + 360.0f, 360.0f);
    elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
    pulses_est = elapsed_ms / (float)(pulse_ms + 120 + 20);
    float dpp_ccw = (pulses_est > 0) ? actual_ccw / pulses_est : 0;

    INFO("CCW: swept %.1f deg in %.0f ms  (~%.0f pulses)  => %.3f deg/pulse",
         actual_ccw, elapsed_ms, pulses_est, dpp_ccw);

    float avg_dpp = ((dpp_cw > 0 ? dpp_cw : 0) + (dpp_ccw > 0 ? dpp_ccw : 0));
    int cnt = (dpp_cw > 0 ? 1 : 0) + (dpp_ccw > 0 ? 1 : 0);
    if (cnt > 0) avg_dpp /= cnt;

    INFO("Average deg/pulse: %.3f", avg_dpp);
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();
    if (have_spd && avg_dpp > 0.05f) {
        m.jog_deg_per_pulse = avg_dpp;
        spd_save_primary(&m);
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
            "Done. %.3f deg/pulse", avg_dpp);
        s_wcal.state = WCAL_DONE;
        s_wcal.progress = 100;
        PASS("jog_deg_per_pulse saved: %.3f deg/pulse", avg_dpp);
    } else {
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
            "Could not measure -- check pulse duty/ms.");
        s_wcal.state = WCAL_ERROR;
        WARN("Could not measure -- check duty/ms settings.");
    }
    INFO("Done.");
    // vTaskDelete only when running as a FreeRTOS task (web-triggered)
    // The wcal_reset() caller (web handler) set state before xTaskCreate,
    // so if state is still JOG_RUNNING here we know we're in a task context.
    // vTaskDelete handled by wcal_jog_task wrapper when called from web
}

static void wcal_jog_task(void *arg)
{
    s_wcal.state    = WCAL_JOG_RUNNING;
    s_wcal.progress = 5;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Starting jog pulse cal...");

    // --- Run the jog cal inline (no Enter prompt for web) ---
    STEP("Nozzle Jog Pulse Calibration (G) [web]");
    INFO("Sweeps nozzle 30 deg CW+CCW under 40%% pressure to measure deg/pulse.");

    sensor_rail_on();
    motor_rail_on();
    adc_setup();

    speed_map_t m = {0};
    bool have_spd = (spd_load_primary(&m) == ESP_OK && m.num_points > 0);
    uint16_t pulse_duty = (have_spd && m.jog_pulse_duty > 0) ? m.jog_pulse_duty : 200;
    uint32_t pulse_ms   = (have_spd && m.jog_pulse_ms   > 0) ? (uint32_t)m.jog_pulse_ms : 80;
    INFO("jog_pulse_duty=%u  jog_pulse_ms=%u", pulse_duty, pulse_ms);

    pressure_map_t pcal = {0};
    bool have_pcal = (cal_load_primary(&pcal) == ESP_OK && pcal.num_points >= 2);
    float psi_min = 0.05f, psi_max = 8.0f;
    if (have_pcal) {
        psi_min = psi_max = pcal.pressure_psi[0];
        for (int ci = 1; ci < pcal.num_points; ci++) {
            if (pcal.pressure_psi[ci] < psi_min) psi_min = pcal.pressure_psi[ci];
            if (pcal.pressure_psi[ci] > psi_max) psi_max = pcal.pressure_psi[ci];
        }
        float target = psi_max * 0.40f;
        INFO("Opening valve to %.3f PSI (40%% of max)...", target);
        water_hold_pressure(target, psi_min, psi_max);
        INFO("Actual: %.3f PSI", cal_read_pressure_avg());
    } else {
        INFO("No pressure cal -- measuring dry.");
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    float TEST_ARC = 30.0f;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Sweeping CW 30 deg...");
    s_wcal.progress = 20;
    INFO("Sweeping CW %.0f deg via pulse mode...", TEST_ARC);

    uint16_t raw0 = 0;
    as5600_read(ADDR_AS5600, &raw0, NULL, NULL);
    float origin = raw0 * (360.0f / 4096.0f);
    TickType_t t0 = xTaskGetTickCount();
    float psi_sum = 0.0f; int psi_n = 0;
    // Pass actual valve_deg and matching ring_throw so PID holds 30%
    // pressure instead of trying to close the valve (VALVE_CLOSED_DEG
    // + ring_throw=0 gives target_psi=0 which fights the open valve).
    float jog_ring_throw = have_pcal ? cal_pressure_to_throw_mm(psi_max * 0.30f) : 0.0f;
    float jog_valve_deg  = have_pcal ? cal_pressure_to_valve_deg(psi_max * 0.30f)
                                     : VALVE_CLOSED_DEG;
    nozzle_sweep_pulse(origin, TEST_ARC, true, 1.0f,
                       pulse_duty, pulse_ms, t0, 0, 1, 0,
                       jog_valve_deg, jog_ring_throw, &psi_sum, &psi_n, false);

    uint16_t raw1 = 0;
    as5600_read(ADDR_AS5600, &raw1, NULL, NULL);
    float end_cw = raw1 * (360.0f / 4096.0f);
    float actual_cw = fmodf(end_cw - origin + 360.0f, 360.0f);
    float elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
    float pulses_est = elapsed_ms / (float)(pulse_ms + 120 + 20);
    float dpp_cw = (pulses_est > 0) ? actual_cw / pulses_est : 0;
    INFO("CW: swept %.1f deg in %.0f ms (~%.0f pulses) => %.3f deg/pulse",
         actual_cw, elapsed_ms, pulses_est, dpp_cw);

    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Sweeping CCW 30 deg...");
    s_wcal.progress = 60;
    INFO("Sweeping CCW %.0f deg...", TEST_ARC);
    as5600_read(ADDR_AS5600, &raw0, NULL, NULL);
    origin = raw0 * (360.0f / 4096.0f);
    t0 = xTaskGetTickCount();
    psi_sum = 0.0f; psi_n = 0;
    nozzle_sweep_pulse(origin, TEST_ARC, false, 1.0f,
                       pulse_duty, pulse_ms, t0, 0, 1, 0,
                       jog_valve_deg, jog_ring_throw, &psi_sum, &psi_n, false);

    as5600_read(ADDR_AS5600, &raw1, NULL, NULL);
    float end_ccw = raw1 * (360.0f / 4096.0f);
    float actual_ccw = fmodf(origin - end_ccw + 360.0f, 360.0f);
    elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
    pulses_est = elapsed_ms / (float)(pulse_ms + 120 + 20);
    float dpp_ccw = (pulses_est > 0) ? actual_ccw / pulses_est : 0;
    INFO("CCW: swept %.1f deg in %.0f ms (~%.0f pulses) => %.3f deg/pulse",
         actual_ccw, elapsed_ms, pulses_est, dpp_ccw);

    float avg_dpp = ((dpp_cw > 0 ? dpp_cw : 0) + (dpp_ccw > 0 ? dpp_ccw : 0));
    int cnt = (dpp_cw > 0 ? 1 : 0) + (dpp_ccw > 0 ? 1 : 0);
    if (cnt > 0) avg_dpp /= cnt;
    INFO("Average deg/pulse: %.3f", avg_dpp);

    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();

    if (have_spd && avg_dpp > 0.05f) {
        m.jog_deg_per_pulse = avg_dpp;
        spd_save_primary(&m);
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Done: %.3f deg/pulse", avg_dpp);
        s_wcal.state = WCAL_DONE;
        s_wcal.progress = 100;
        PASS("jog_deg_per_pulse saved: %.3f deg/pulse", avg_dpp);
    } else {
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Could not measure -- check pulse duty/ms.");
        s_wcal.state = WCAL_ERROR;
        WARN("Could not measure -- check duty/ms settings.");
    }
    INFO("Done.");
    vTaskDelete(NULL);
}

static void phase_nozzle_speed_cal(void)
{
    STEP("Nozzle Speed Calibration");
    // b360: cal at ~70% pressure instead of ~40%. The motor's effective load
    // is the water-jet reaction force on the nozzle bearing, which scales
    // with flow rate. Mid-throw watering typically operates at 60-80% of cal
    // max pressure, so 40% cal was systematically under-loading the motor
    // and over-predicting dps for typical watering use (TopCorner R12-R20
    // ran at 1/9th the predicted dps at duty=70). 70% better matches the
    // mid-throw operating point. Inner rings still see less load than this;
    // the b360 mid-sweep dps refit handles those at runtime.
    INFO("Measures nozzle rotation speed with valve open to ~70%% pressure.");
    INFO("Connect water supply. Nozzle will sweep -- point safely.");
    if (s_wcal.state == WCAL_IDLE) {
        INFO("Press Enter to start, q to abort.");
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q' || c == 0) { INFO("Aborted."); return; }
    }

    sensor_rail_on();
    motor_rail_on();
    adc_setup();

    // Open valve to ~70%% of max cal pressure for wet measurement.
    pressure_map_t pcal = {0};
    bool have_pcal = (cal_load_primary(&pcal) == ESP_OK && pcal.num_points >= 2);
    float psi_min = 0.05f, psi_max = 8.0f;  // fallback if no pressure cal
    if (have_pcal) {
        psi_min = psi_max = pcal.pressure_psi[0];
        for (int ci = 1; ci < pcal.num_points; ci++) {
            if (pcal.pressure_psi[ci] < psi_min) psi_min = pcal.pressure_psi[ci];
            if (pcal.pressure_psi[ci] > psi_max) psi_max = pcal.pressure_psi[ci];
        }
    }
    float target_psi = have_pcal ? psi_max * 0.70f : psi_max * 0.70f;
    if (have_pcal) {
        INFO("Opening valve to ~70%% pressure (%.3f PSI)...", target_psi);
        water_hold_pressure(target_psi, psi_min, psi_max);
        INFO("Valve open: %.3f PSI actual", cal_read_pressure_avg());
    } else {
        INFO("No pressure cal -- measuring dry. Run x first for best results.");
    }
    vTaskDelay(pdMS_TO_TICKS(500));


    // Duty levels: start at confirmed start threshold (12%=60), then 2% steps
    // up to 20%, then 10% steps to 100%. Covers full usable range.
    uint16_t duties[] = { 70, 80, 90, 100, 150, 200, 250, 300, 350, 400, 450, 500 };
    int num_duties = sizeof(duties) / sizeof(duties[0]);

    speed_map_t map = {0};
    int n = 0;

    INFO("  Duty   %%PWM   deg/sec   sec/rev");
    INFO("  -----  -----  -------   -------");

    for (int i = 0; i < num_duties && n < SPD_MAX_POINTS; i++) {
        uint16_t duty = duties[i];
        float pct = duty * 100.0f / 500.0f;
        INFO("  Testing duty=%d (%.0f%%)...", duty, pct);
        vTaskDelay(pdMS_TO_TICKS(SPD_SETTLE_MS));

        float dps = spd_measure_rotation(duty);

        if (dps < 0) {
            WARN("  Timeout at duty=%d -- motor may have stalled, skipping.", duty);
            continue;
        }

        float sec_per_rev = 360.0f / dps;
        INFO("  %5d  %4.0f%%  %7.1f   %7.2f", duty, pct, dps, sec_per_rev);

        map.duty[n]        = duty;
        map.deg_per_sec[n] = dps;
        n++;

        result(dps > 0, "Nozzle rotation measured");
    }

    map.num_points = n;

    // --- CCW (NREV) direction measurement ---
    INFO("\nNow measuring CCW (reverse) direction...");
    INFO("  Duty   %%PWM   deg/sec   sec/rev");
    INFO("  -----  -----  -------   -------");
    int nccw = 0;
    for (int i = 0; i < num_duties && nccw < SPD_MAX_POINTS; i++) {
        uint16_t duty = duties[i];
        float pct = duty * 100.0f / 500.0f;
        INFO("  Testing duty=%d (%.0f%%) CCW...", duty, pct);
        vTaskDelay(pdMS_TO_TICKS(500));
        float dps = spd_measure_rotation_ccw(duty);
        if (dps < 0) {
            WARN("  Timeout at duty=%d CCW -- skipping.", duty);
            continue;
        }
        float sec_per_rev = 360.0f / dps;
        INFO("  %5d  %4.0f%%  %7.1f   %7.2f", duty, pct, dps, sec_per_rev);
        map.duty_ccw[nccw]        = duty;
        map.deg_per_sec_ccw[nccw] = dps;
        nccw++;
    }
    map.num_points_ccw = nccw;
    if (nccw > 0)
        INFO("CCW: %.1f dps min, %.1f dps max",
             map.deg_per_sec_ccw[0], map.deg_per_sec_ccw[nccw-1]);

    // Set jog defaults -- tune with G menu option after testing
    map.min_continuous_dps = (n > 0) ? map.deg_per_sec[0] : 10.0f;
    map.jog_pulse_duty     = 200;   // 40% -- adjust if stall occurs
    map.jog_pulse_ms       = 50;
    map.jog_deg_per_pulse  = 0.0f;  // measured by G calibration
    map.use_bump_stop      = 0;

    INFO("\nSpeed calibration summary (%d points):", n);
    INFO("  Min speed: %.1f deg/sec (%.1f sec/rev) at duty %d",
         map.deg_per_sec[0], 360.0f/map.deg_per_sec[0], map.duty[0]);
    INFO("  Max speed: %.1f deg/sec (%.1f sec/rev) at duty %d",
         map.deg_per_sec[n-1], 360.0f/map.deg_per_sec[n-1], map.duty[n-1]);

    esp_err_t r = spd_save_primary(&map);
    if (r == ESP_OK) {
        PASS("Nozzle speed calibration saved");
    } else {
        ESP_LOGE(TAG, "Save failed: %s", esp_err_to_name(r));
        FAIL("Speed calibration save");
    }

    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();
}

static void phase_nozzle_speed_view(void)
{
    STEP("View Nozzle Speed Calibration");
    speed_map_t m = {0};
    if (spd_load_primary(&m) != ESP_OK || m.num_points == 0) {
        INFO("No speed calibration data. Run speed cal first ('N').");
        return;
    }
    INFO("Nozzle speed calibration (%d points):", m.num_points);
    INFO("  Pt  Duty   %%PWM   deg/sec  sec/rev");
    INFO("  --  -----  -----  -------  -------");
    for (int i = 0; i < m.num_points; i++) {
        float dps = m.deg_per_sec[i];
        INFO("  %2d  %5d  %4.0f%%  %7.1f  %7.2f",
             i+1, m.duty[i], m.duty[i]*100.0f/500.0f, dps, 360.0f/dps);
    }
}

// Pressure monitoring over time -- logs CSV to UART/TCP.
// Holds valve at fixed % of calibrated range, samples at configurable interval.
// Press any key to stop.
static void phase_pressure_monitor(void)
{
    STEP("Pressure Monitor");

    pressure_map_t map = {0};
    bool have_cal = (cal_load_primary(&map) == ESP_OK && map.num_points > 0);
    float psi_min = 0.0f, psi_max = zone_get_psi_max();
    if (have_cal) {
        psi_min = map.pressure_psi[0]; psi_max = map.pressure_psi[0];
        for (int i = 1; i < map.num_points; i++) {
            if (map.pressure_psi[i] < psi_min) psi_min = map.pressure_psi[i];
            if (map.pressure_psi[i] > psi_max) psi_max = map.pressure_psi[i];
        }
        INFO("Calibration range: %.3f to %.3f PSI", psi_min, psi_max);
    }

    char buf[16];
    INFO("Valve position %% of max pressure (0-100, default 50): ");
    memset(buf, 0, sizeof(buf)); int n = 0;
    while (n < (int)sizeof(buf) - 1) {
        int c = uart_getchar(5000);
        if (c == 0) break;
        if (c == '\r' || c == '\n') { printf("\r\n"); break; }
        if (c == 8 || c == 127) { if (n > 0) { n--; buf[n]=0; printf("\b \b"); } continue; }
        buf[n++] = (char)c; printf("%c", c); TOUCH_ACTIVITY();
    }
    float pct = (n > 0) ? atof(buf) : 50.0f;
    if (pct < 0.0f || pct > 100.0f) pct = 50.0f;

    INFO("Sample interval in seconds (default 10): ");
    memset(buf, 0, sizeof(buf)); n = 0;
    while (n < (int)sizeof(buf) - 1) {
        int c = uart_getchar(5000);
        if (c == 0) break;
        if (c == '\r' || c == '\n') { printf("\r\n"); break; }
        if (c == 8 || c == 127) { if (n > 0) { n--; buf[n]=0; printf("\b \b"); } continue; }
        buf[n++] = (char)c; printf("%c", c); TOUCH_ACTIVITY();
    }
    uint32_t interval_ms = (n > 0 && atoi(buf) > 0) ?
        (uint32_t)(atoi(buf) * 1000) : 10000;

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    float target_psi = psi_min + (pct / 100.0f) * (psi_max - psi_min);
    float target_deg = have_cal ? cal_pressure_to_valve_deg(target_psi) : VALVE_CLOSED_DEG;
    if (target_deg < 0) target_deg = VALVE_CLOSED_DEG;

    INFO("Setting valve to %.1f%% = %.3f PSI (%.1f deg)...", pct, target_psi, target_deg);
    valve_goto(target_deg, 2.0f, 15000, false);
    vTaskDelay(pdMS_TO_TICKS(2000));

    INFO("Logging. Press any key to stop.");
    tprintf("sample,time_s,psi,pct_of_range,valve_deg\r\n");

    TickType_t t_start = xTaskGetTickCount();
    uint32_t sample = 0;
    TickType_t next_sample = t_start;

    while (uart_getchar(0) == 0) {
        if (xTaskGetTickCount() < next_sample) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        next_sample += pdMS_TO_TICKS(interval_ms);
        TOUCH_ACTIVITY();

        float psi_sum = 0; int psi_ok = 0;
        for (int i = 0; i < 4; i++) {
            float p = 0;
            if (mprls_read(&p)) { psi_sum += p; psi_ok++; }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        float psi_avg  = psi_ok ? psi_sum / psi_ok : -1.0f;
        float pct_act  = (psi_max > psi_min && psi_avg >= 0) ?
            ((psi_avg - psi_min) / (psi_max - psi_min)) * 100.0f : 0.0f;
        // Read actual valve angle from AS5600
        uint16_t raw = 0;
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        float valve_deg = raw * (360.0f / 4096.0f);
        uint32_t elapsed_s = pdTICKS_TO_MS(xTaskGetTickCount() - t_start) / 1000;
        sample++;

        tprintf("%lu,%lu,%.4f,%.2f,%.2f\r\n",
            (unsigned long)sample, (unsigned long)elapsed_s,
            psi_avg, pct_act, valve_deg);
        INFO("  [%4lu] t=%lus  %.4f PSI (%.1f%%)  valve=%.1f deg",
            (unsigned long)sample, (unsigned long)elapsed_s,
            psi_avg, pct_act, valve_deg);
    }

    uint32_t total_s = pdTICKS_TO_MS(xTaskGetTickCount() - t_start) / 1000;
    INFO("Monitor stopped. %lu samples, %lu seconds.",
         (unsigned long)sample, (unsigned long)total_s);
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);
    motor_rail_off();
}

static void phase_zone_setup(void)
{
    STEP("Zone Perimeter Setup");
    INFO("Define zone boundary by setting perimeter points.");
    INFO("Adjust nozzle direction and throw distance until spray");
    INFO("just reaches the zone boundary, then set the point.");
    INFO("");
    INFO("Commands:");
    INFO("  l  Rotate nozzle left  (CCW %.0f deg)", ZONE_ROTATE_STEP_DEG);
    INFO("  r  Rotate nozzle right (CW  %.0f deg)", ZONE_ROTATE_STEP_DEG);
    INFO("  +  Increase pressure (open valve %.0f deg)", ZONE_PRESSURE_STEP_DEG);
    INFO("  -  Decrease pressure (close valve %.0f deg)", ZONE_PRESSURE_STEP_DEG);
    INFO("  s  Set point (record current position)");
    INFO("  d  Delete last point");
    INFO("  p  Print all points");
    INFO("  v  View current position (nozzle angle + pressure)");
    INFO("  S  Save zone and exit");
    INFO("  q  Abort (discard all points)");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    // Reset direction trackers -- first move always does backlash compensation
    s_nozzle_last_dir = 0;
    s_valve_last_dir  = 0;

    // Ensure valve is closed before starting
    INFO("Closing valve before zone setup...");
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);

    // Load calibration for pressure readings
    pressure_map_t cal = {0};
    bool have_cal = (cal_load_primary(&cal) == ESP_OK && cal.num_points > 0);
    if (!have_cal) {
        WARN("No calibration data found. Pressure readings will be raw PSI only.");
        WARN("Run pressure calibration ('x') first for best results.");
    }

    zone_perimeter_t zone = {0};

    // Read starting position
    uint16_t raw = 0;
    as5600_read(ADDR_AS5600, &raw, NULL, NULL);
    float nozzle_deg = raw * (360.0f / 4096.0f);

    as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
    float valve_deg = raw * (360.0f / 4096.0f);

    float psi = 0;
    mprls_read(&psi);

    INFO("Starting position: nozzle=%.1f deg  valve=%.1f deg  pressure=%.3f PSI",
         nozzle_deg, valve_deg, psi);
    INFO("");

    while (true) {
        INFO("[l/r/+/-/s/d/p/v/S/q]> ");
        int c = uart_getchar(60000);
        if (c == 0) {
            INFO("Timeout -- aborting zone setup.");
            goto abort;
        }
        printf("%c\r\n", c);

        // Read current state
        as5600_read(ADDR_AS5600, &raw, NULL, NULL);
        nozzle_deg = raw * (360.0f / 4096.0f);
        mprls_read(&psi);

        if (c == 'l' || c == 'L') {
            float target = nozzle_deg - ZONE_ROTATE_STEP_DEG;
            if (target < 0) target += 360.0f;
            INFO("Rotating left to %.1f deg...", target);
            nozzle_goto_ex(target, 2.0f, 10000, false, s_nozzle_last_dir);
            s_nozzle_last_dir = -1;

        } else if (c == 'r' || c == 'R') {
            float target = nozzle_deg + ZONE_ROTATE_STEP_DEG;
            if (target >= 360.0f) target -= 360.0f;
            INFO("Rotating right to %.1f deg...", target);
            nozzle_goto_ex(target, 2.0f, 10000, false, s_nozzle_last_dir);
            s_nozzle_last_dir = 1;

        } else if (c == '+') {
            // Increase pressure by fixed PSI step
            // Use calibration table to find valve angle for target pressure
            mprls_read(&psi);
            float target_psi = psi + ZONE_PRESSURE_STEP_PSI;
            float target_deg = cal_pressure_to_valve_deg(target_psi);
            if (target_deg < 0) {
                // No calibration -- fall back to fixed angle step
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                target_deg = raw * (360.0f / 4096.0f) + ZONE_PRESSURE_STEP_DEG;
                if (target_deg > 360.0f) target_deg = 360.0f;
                INFO("No cal data -- opening valve to %.1f deg...", target_deg);
            } else {
                INFO("Increasing pressure %.3f -> %.3f PSI (valve %.1f deg)...",
                     psi, target_psi, target_deg);
            }
            valve_goto_ex(target_deg, 2.0f, 10000, false, s_valve_last_dir);
            s_valve_last_dir = 1;  // opening = increasing angle
            vTaskDelay(pdMS_TO_TICKS(500));
            mprls_read(&psi);
            INFO("  Actual pressure: %.3f PSI", psi);

        } else if (c == '-') {
            // Decrease pressure by fixed PSI step
            mprls_read(&psi);
            float target_psi = psi - ZONE_PRESSURE_STEP_PSI;
            if (target_psi < 0) target_psi = 0;
            float target_deg = cal_pressure_to_valve_deg(target_psi);
            if (target_deg < 0) {
                // No calibration -- fall back to fixed angle step
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                target_deg = raw * (360.0f / 4096.0f) - ZONE_PRESSURE_STEP_DEG;
                if (target_deg < 0) target_deg = 0;
                INFO("No cal data -- closing valve to %.1f deg...", target_deg);
            } else {
                INFO("Decreasing pressure %.3f -> %.3f PSI (valve %.1f deg)...",
                     psi, target_psi, target_deg);
            }
            valve_goto_ex(target_deg, 2.0f, 10000, false, s_valve_last_dir);
            s_valve_last_dir = -1;  // closing = decreasing angle
            vTaskDelay(pdMS_TO_TICKS(500));
            mprls_read(&psi);
            INFO("  Actual pressure: %.3f PSI", psi);

        } else if (c == 's') {
            // Set point
            if (zone.num_points >= ZONE_MAX_PERIM_POINTS) {
                WARN("Maximum %d points reached.", ZONE_MAX_PERIM_POINTS);
            } else {
                as5600_read(ADDR_AS5600, &raw, NULL, NULL);
                nozzle_deg = raw * (360.0f / 4096.0f);
                mprls_read(&psi);
                zone.points[zone.num_points].nozzle_deg   = nozzle_deg;
                zone.points[zone.num_points].pressure_psi = psi;
                zone.points[zone.num_points].throw_mm     = cal_pressure_to_throw_mm(psi);
                { uint16_t _vr = 0; as5600_read(ADDR_AS5600L, &_vr, NULL, NULL);
                  zone.points[zone.num_points].valve_deg = _vr * (360.0f / 4096.0f); }
                zone.num_points++;
                float throw_est = cal_pressure_to_throw_mm(psi);
                if (throw_est > 0)
                    INFO("[POINT %d SET] nozzle=%.1f deg  pressure=%.3f PSI  throw~%.0f mm",
                         zone.num_points, nozzle_deg, psi, throw_est);
                else
                    INFO("[POINT %d SET] nozzle=%.1f deg  pressure=%.3f PSI",
                         zone.num_points, nozzle_deg, psi);
            }

        } else if (c == 'd' || c == 'D') {
            if (zone.num_points == 0) {
                INFO("No points to delete.");
            } else {
                zone.num_points--;
                INFO("Deleted point %d. %d points remaining.",
                     zone.num_points + 1, zone.num_points);
            }

        } else if (c == 'p' || c == 'P') {
            INFO("Current perimeter (%d points):", zone.num_points);
            zone_print_points(&zone);

        } else if (c == 'v' || c == 'V') {
            as5600_read(ADDR_AS5600, &raw, NULL, NULL);
            nozzle_deg = raw * (360.0f / 4096.0f);
            as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
            valve_deg = raw * (360.0f / 4096.0f);
            mprls_read(&psi);
            // Pressure is the key value -- valve angle shown for diagnostics only
            INFO("Nozzle: %.1f deg  Pressure: %.3f PSI  (valve: %.1f deg)",
                 nozzle_deg, psi, valve_deg);

        } else if (c == 'S') {
            // Save
            if (zone.num_points < 3) {
                WARN("Need at least 3 points to define a zone (have %d).", zone.num_points);
            } else {
                INFO("Saving zone with %d perimeter points...", zone.num_points);
                zone_print_points(&zone);
                zone_save_primary(0, NULL, &zone);
                esp_err_t r = ESP_OK;
                if (r == ESP_OK) {
                    PASS("Zone saved");
                } else {
                    ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(r));
                    FAIL("Zone NVS save");
                }
                goto done;
            }

        } else if (c == 'q' || c == 'Q') {
            INFO("Aborting -- discarding %d points.", zone.num_points);
            goto abort;
        }
    }

abort:
    INFO("Zone setup aborted.");
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);  // close valve
    motor_rail_off();
    return;

done:
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);  // close valve
    motor_rail_off();
    INFO("Zone setup complete.");
}

static void phase_zone_view(void)
{
    STEP("View Zone Perimeter");
    sensor_rail_on();

    zone_perimeter_t zone = {0};
    if (zone_load_primary(0, &zone) != ESP_OK || zone.num_points == 0) {
        INFO("No zone defined. Use 'z' to set up a zone.");
        return;
    }

    INFO("Stored zone perimeter (%d points):", zone.num_points);
    zone_print_points(&zone);
}

static void phase_nozzle_goto(void)
{
    STEP("Nozzle Go-To Position");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    uint16_t raw = 0;
    as5600_read(ADDR_AS5600, &raw, NULL, NULL);
    float current_deg = raw * (360.0f / 4096.0f);
    INFO("Current nozzle position: %.1f deg", current_deg);
    INFO("Enter target angle (0-360), Enter. 'q' to quit.");

    char buf[16];
    while (true) {
        INFO("Target deg> ");
        int i = 0;
        memset(buf, 0, sizeof(buf));
        while (i < (int)sizeof(buf)-1) {
            int c = uart_getchar(30000);
            if (c == 0 || c == 'q' || c == 'Q') goto done;
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) { if (i > 0) { i--; buf[i] = 0; printf("\b \b"); } continue; }
            buf[i++] = (char)c;
            printf("%c", c);
        }
        if (i == 0) continue;

        float target = atof(buf);
        if (target < 0.0f || target > 360.0f) {
            INFO("Invalid angle %.1f -- enter 0-360", target);
            continue;
        }

        INFO("Moving nozzle to %.1f deg...", target);
        bool ok = nozzle_goto(target, 2.0f, 10000, false);
        result(ok, "Nozzle reached target position");

        as5600_read(ADDR_AS5600, &raw, NULL, NULL);
        current_deg = raw * (360.0f / 4096.0f);
        INFO("Position: %.1f deg", current_deg);
    }
done:
    motor_rail_off();
    INFO("Done.");
}

static void phase_kill(void)
{
    STEP("Kill / Brake Signal (GPIO13)");
    WARN("Valve motor will run briefly. 2 second warning...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    adc_setup();
    motor_rail_on();

    gpio_set_level(GPIO_VFWD, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    uint32_t running = adc_mv(ADC_CH_VCUR);

    gpio_set_level(GPIO_K, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t braked = adc_mv(ADC_CH_VCUR);
    gpio_set_level(GPIO_VFWD, 0);
    gpio_set_level(GPIO_K,    0);

    INFO("Current running: %lu mV (%.0f mA)", running, CURRENT_MA(running));
    INFO("Current braked:  %lu mV (%.0f mA)", braked,  CURRENT_MA(braked));
    result(running > 30, "Motor current while running");
    result(true, "K signal exercised (check H-bridge datasheet for brake behavior)");

    motor_rail_off();
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase S: Sleep/wake
// ─────────────────────────────────────────────────────────────────────────────
static void phase_toggle_3v3sen(void)
{
    STEP("3V3Sen Rail Toggle");
    INFO("Toggling 3V3Sen (GPIO4) every 2 seconds. Measure with multimeter.");
    INFO("Press any key to stop.");
    INFO("");

    bool state = false;
    int count = 0;
    while (true) {
        if (uart_getchar(10) != 0) break;

        state = !state;
        gpio_set_level(GPIO_3V3SEN, state ? 1 : 0);
        count++;

        if (state) {
            INFO("[%d] 3V3Sen HIGH -> Rail should be ON  -> SDA/SCL should be ~3.3V", count);
        } else {
            INFO("[%d] 3V3Sen LOW  -> Rail should be OFF -> SDA/SCL should be ~0V", count);
        }

        // update our rail state tracking
        if (state && !s_sensor_rail) {
            i2c_bus_init();
            s_sensor_rail = true;
        } else if (!state && s_sensor_rail) {
            i2c_bus_deinit();
            s_sensor_rail = false;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    INFO("Stopped. 3V3Sen left %s", state ? "HIGH (rail ON)" : "LOW (rail OFF)");
}

static void phase_toggle_9v(void)
{
    STEP("9V Motor Rail Toggle");
    INFO("Toggling 9V_EN (GPIO18) and K/brake (GPIO13) every 2 seconds.");
    INFO("Measure J10 connector pins with multimeter for 9V.");
    INFO("Press any key to stop.");
    INFO("");

    bool state = false;
    int count = 0;
    while (true) {
        if (uart_getchar(10) != 0) break;

        state = !state;
        if (state) {
            gpio_set_level(GPIO_9V_EN, 1);
            gpio_set_level(GPIO_K,     0);   // release brake
            INFO("[%d] 9V_EN HIGH, K LOW  -> Motor rail ON  -> J10 should show 9V", count++);
        } else {
            gpio_set_level(GPIO_K,     1);   // brake on
            gpio_set_level(GPIO_9V_EN, 0);
            INFO("[%d] 9V_EN LOW,  K HIGH -> Motor rail OFF -> J10 should show 0V", count++);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Leave rail off when done
    gpio_set_level(GPIO_K,     1);
    gpio_set_level(GPIO_9V_EN, 0);
    INFO("9V rail left OFF.");
}

static void phase_jog_valve(void)
{
    STEP("Valve Jog (200ms pulses)");
    INFO("Use 'f' for forward, 'r' for reverse, 'q' to quit.");
    INFO("Current position shown after each jog.");
    INFO("Valve angle range: 0-4095 counts (0-360 deg)");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    uint16_t pos = 0;
    as5600_read(ADDR_AS5600L, &pos, NULL, NULL);
    INFO("Current angle: %u (%.1f deg)", pos, pos * 360.0f / 4096.0f);

    while (true) {
        INFO("  [f]=fwd  [r]=rev  [q]=quit");
        int c = uart_getchar(30000);   // block up to 30s waiting for keypress
        if (c == 'q' || c == 'Q' || c == 0) break;
        if (c == 'f' || c == 'F') {
            motor_run(GPIO_VFWD, GPIO_VREV, "Jog FWD", ADC_CH_VCUR, "VCur", 200);
            as5600_read(ADDR_AS5600L, &pos, NULL, NULL);
            INFO("  -> %u (%.1f deg)", pos, pos * 360.0f / 4096.0f);
        } else if (c == 'r' || c == 'R') {
            motor_run(GPIO_VREV, GPIO_VFWD, "Jog REV", ADC_CH_VCUR, "VCur", 200);
            as5600_read(ADDR_AS5600L, &pos, NULL, NULL);
            INFO("  -> %u (%.1f deg)", pos, pos * 360.0f / 4096.0f);
        }
    }

    motor_rail_off();
    INFO("Jog done.");
}

static void phase_led_test(void)
{
    STEP("LED Colour Test");
    sensor_rail_on();

    // Ensure TCA is in output mode
    if (!s_tca_outputs) {
        uint8_t off = LED_OFF;
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &off, 1);
        uint8_t all_out = 0x00;
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_CONFIG, &all_out, 1);
        s_tca_outputs = true;
    }

    INFO("LED test -- press key to cycle:");
    INFO("  0=Off  1=Bit0(0xFE)  2=Bit1(0xFD)  3=Bit2(0xFB)");
    INFO("  4=Bits0+1(0xFC)  5=Bits0+2(0xFA)  6=Bits1+2(0xF9)  7=All(0xF8)");
    INFO("  g=Green(0xFD)  b=Blue(0xFB)  c=Cyan(0xF9)  q=Quit");

    const struct { char key; uint8_t val; const char *name; } entries[] = {
        {'0', 0xFF, "Off"},
        {'1', 0xFE, "Bit0 only"},
        {'2', 0xFD, "Bit1 only"},
        {'3', 0xFB, "Bit2 only"},
        {'4', 0xFC, "Bits 0+1"},
        {'5', 0xFA, "Bits 0+2"},
        {'6', 0xF9, "Bits 1+2"},
        {'7', 0xF8, "All bits 0-2"},
        {'g', LED_GREEN, "GREEN"},
        {'b', LED_BLUE,  "BLUE"},
        {'c', LED_CYAN,  "CYAN"},
    };

    while (true) {
        int ch = uart_getchar(30000);
        if (ch == 'q' || ch == 'Q' || ch == 0) break;
        bool found = false;
        for (int i = 0; i < (int)(sizeof(entries)/sizeof(entries[0])); i++) {
            if (ch == entries[i].key) {
                i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT,
                                  &entries[i].val, 1);
                INFO("  %c -> 0x%02x [%s]", ch, entries[i].val, entries[i].name);
                found = true;
                break;
            }
        }
        if (!found) INFO("  Unknown key '%c' -- 0=off g=green b=blue c=cyan q=quit", ch);
    }

    // Leave LEDs off
    uint8_t off = LED_OFF;
    i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &off, 1);
    INFO("LEDs off.");
}


// Pressure calibration

// Internal: move directly to target with no backlash compensation
static bool valve_goto_direct(float target_deg, float tolerance_deg,
                               uint32_t timeout_ms, bool verbose);

// valve_goto_ex: backlash compensation only when direction reverses.
// prev_dir: +1=last move increasing angle, -1=decreasing, 0=unknown (always compensate)
// -----------------------------------------------------------------------
// valve_goto_jog: precise positioning in the hydrodynamic resistance zone
// (270-286 deg) using short pulses + encoder check.
// This zone is hydrodynamic, not mechanical friction: water pressure on
// the partially-open ball face resists opening until ~50% port area, then
// releases suddenly. The motor cannot step smoothly in the opening direction
// through this zone -- it stalls, kicks, and overshoots.
// Solution: always approach from the HIGH (open) side in the CLOSING direction.
// The closing direction has no hydrodynamic resistance (torque assists or is
// neutral), so the jog settles cleanly regardless of target position.
// -----------------------------------------------------------------------
#define VALVE_FRICTION_LO  (270.0f + g_valve_offset_deg)  // hydrodynamic zone start (deg, frame-relative)
#define VALVE_FRICTION_HI  (286.0f + g_valve_offset_deg)  // hydrodynamic zone end (deg, frame-relative)
// ---------------------------------------------------------------------------
// Pressure-control experiments (draft 2026-06-06). Each flag defaults to 0 ==
// today's behavior; flip exactly ONE, recompile, A/B against baseline, revert.
//
// KEY FACT that shapes these: the AS5600L sits on the valve OUTPUT shaft, so
// the position loop already closes on the ball -- gear lash is INVISIBLE to
// encoder counting and does NOT cost final position. Lash instead costs us
// (a) TIME on direction reversal and (b) pressure HYSTERESIS. Menu 'd' measures
// both directly. These two toggles tune the spots where that cost shows up.
//   EXP_SNAP_JOG_TUNE     -- snap zone (270-286 deg): not torque-limited, it's
//                            deliberately under-driven; try slowing the creep so
//                            the hydraulic lurch can't outrun the encoder loop.
//   EXP_TRIM_NO_OVERSHOOT -- stop a sub-degree REVERSING pressure-trim nudge from
//                            triggering the full 15 deg consistent-approach swing.
// ---------------------------------------------------------------------------
#define EXP_SNAP_JOG_TUNE        0     // softer/shorter jog pulses in the snap zone
#define EXP_TRIM_NO_OVERSHOOT    0     // skip 15deg approach-comp for tiny trim nudges
#define EXP_TRIM_SMALL_CORR_DEG  1.0f  // "tiny" threshold for EXP_TRIM_NO_OVERSHOOT

#if EXP_SNAP_JOG_TUNE
#define VALVE_JOG_PULSE_MS   15     // EXP: slower creep -- lurch can't outrun the loop
#define VALVE_JOG_DUTY      180     // EXP: ~36% -- softer break-through
#else
#define VALVE_JOG_PULSE_MS   25     // ms per jog pulse
#define VALVE_JOG_DUTY      220     // ~44% -- enough to move but not blast past
#endif
#define VALVE_JOG_MAX_PULSES 60     // safety cap

static bool valve_goto_jog(float target_deg, float tolerance_deg,
                            uint32_t timeout_ms)
{
    const float DEG_PER_COUNT = 360.0f / 4096.0f;

    // Determine approach direction.
    // Opening (increasing deg) approaches from below target.
    // Closing (decreasing deg) approaches from above target.
    // For the friction zone, always close past target then open back up.
    // This gives consistent hysteresis and avoids the stall-kick-overshoot cycle.

    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) return false;
    float pos = raw * DEG_PER_COUNT;

    // Always approach target from the HIGH (open) side so the final jog
    // is in the closing direction (decreasing angle).
    // If below target: overshoot to VALVE_FRICTION_HI + 2 deg to complete
    // the hydraulic snap-through, then jog closing to target.
    // If already above target: jog closing directly -- no overshoot needed.
    bool need_overshoot = (pos < target_deg - 0.5f);
    if (need_overshoot) {
        float overshoot = VALVE_FRICTION_HI + 2.0f;  // past snap-through point
        valve_goto_direct(overshoot, 2.0f, timeout_ms / 2, false);
        vTaskDelay(pdMS_TO_TICKS(300));
        if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) return false;
        pos = raw * DEG_PER_COUNT;
    }

    float error = target_deg - pos;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    if (fabsf(error) <= tolerance_deg) return true;

    // Jog direction: closing = increase deg = VREV drives positive
    bool closing = (error > 0);
    gpio_num_t drive_pin = closing ? VALVE_PIN_REV : VALVE_PIN_FWD;
    gpio_num_t dir_pin   = closing ? VALVE_PIN_FWD : VALVE_PIN_REV;

    // Build MCPWM for jog pulses
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tc = { .group_id=0, .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz=10000000, .period_ticks=500, .count_mode=MCPWM_TIMER_COUNT_MODE_UP };
    mcpwm_new_timer(&tc, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oc = { .group_id=0 };
    mcpwm_new_operator(&oc, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cc = { .flags.update_cmp_on_tez=true };
    mcpwm_new_comparator(oper, &cc, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, VALVE_JOG_DUTY);
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gc = { .gen_gpio_num=drive_pin };
    mcpwm_new_generator(oper, &gc, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));
    gpio_set_direction(dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dir_pin, 0);
    mcpwm_timer_enable(timer);

    bool reached = false;
    int pulses = 0;
    uint32_t elapsed = 0;

    while (pulses < VALVE_JOG_MAX_PULSES && elapsed < timeout_ms) {
        // Pulse
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
        vTaskDelay(pdMS_TO_TICKS(VALVE_JOG_PULSE_MS));
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        vTaskDelay(pdMS_TO_TICKS(20));  // brief stop for encoder read
        elapsed += VALVE_JOG_PULSE_MS + 20;
        pulses++;

        if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) break;
        pos = raw * DEG_PER_COUNT;
        error = target_deg - pos;
        if (error >  180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (fabsf(error) <= tolerance_deg) { reached = true; break; }
        // Overshoot: passed through target
        if ((closing && error < -tolerance_deg) ||
            (!closing && error > tolerance_deg)) { reached = true; break; }
    }

    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(drive_pin, 0);
    gpio_set_level(dir_pin, 0);

    ESP_LOGI(TAG, "  valve_jog: %.1f deg (%d pulses, err %.2f)",
             pos, pulses, error);
    return reached;
}

// Is a given valve target angle in the hydrodynamic resistance zone?
static inline bool valve_in_friction_zone(float deg) {
    return (deg >= VALVE_FRICTION_LO && deg <= VALVE_FRICTION_HI);
}

static bool valve_goto_ex(float target_deg, float tolerance_deg,
                           uint32_t timeout_ms, bool verbose, int prev_dir)
{
    // Route friction-zone targets through jog-step control for precision.
    if (valve_in_friction_zone(target_deg)) {
        if (verbose) ESP_LOGI(TAG, "  valve_goto_ex: friction zone -> jog mode");
        return valve_goto_jog(target_deg, tolerance_deg, timeout_ms);
    }

    const float DEG_PER_COUNT = 360.0f / 4096.0f;
    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) return false;
    float current_deg = raw * DEG_PER_COUNT;

    float error = target_deg - current_deg;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= tolerance_deg) {
        if (verbose) ESP_LOGI(TAG, "  Already within tolerance");
        return true;
    }

    int this_dir = (error > 0) ? 1 : -1;
    bool need_backlash = (this_dir < 0) && (prev_dir >= 0);

    if (need_backlash) {
        float overshoot = target_deg - VALVE_BACKLASH_DEG;
        if (overshoot < 0) overshoot += 360.0f;
        if (verbose)
            ESP_LOGI(TAG, "  Backlash: overshooting to %.1f then approaching from below",
                     overshoot);
        if (!valve_goto_direct(overshoot, tolerance_deg, timeout_ms / 2, verbose))
            return false;
    }

    return valve_goto_direct(target_deg, tolerance_deg, timeout_ms / 2, verbose);
}

// Standard valve_goto always applies backlash compensation (safe default for general use)
static bool valve_goto(float target_deg, float tolerance_deg,
                       uint32_t timeout_ms, bool verbose)
{
    return valve_goto_ex(target_deg, tolerance_deg, timeout_ms, verbose, 1);
}

// Internal direct move -- no backlash compensation
static bool valve_goto_direct(float target_deg, float tolerance_deg,
                               uint32_t timeout_ms, bool verbose)
{
    const float DEG_PER_COUNT  = 360.0f / 4096.0f;
    const float MIN_MA         = 32.0f;  // just above idle baseline of ~28mA
    const float STALL_MA       = 500.0f;
    const uint32_t POLL_MS     = 10;    // 100 Hz
    // b416: 0.04 -> 0.02. /exp/lash Section C measured effective coast time at
    // ~0.013-0.022s (opening tight ~0.013), so 0.04 over-predicted stopping
    // distance ~3x -> predictive brake cut too early -> systematic stop-short
    // (the from-above ~2.8 deg bias in Section B). 0.02 stays just above the
    // measured coast (errs toward stop-short, never overshoot).
    const float COAST_S        = 0.02f;

    uint16_t raw = 0;
    if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) {
        ESP_LOGE(TAG, "valve_goto: sensor read failed");
        return false;
    }
    float current_deg = raw * DEG_PER_COUNT;

    // Shortest-path error with 360 wrap
    float error = target_deg - current_deg;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    if (verbose)
        ESP_LOGI(TAG, "valve_goto: %.1f -> %.1f deg (error %.1f)",
                 current_deg, target_deg, error);

    if (fabsf(error) <= tolerance_deg) {
        if (verbose) ESP_LOGI(TAG, "  Already within tolerance");
        return true;
    }

    // Reverse increases angle, forward decreases angle (confirmed empirically;
    // VALVE_PIN_* resolves the physical GPIO per stored motor polarity, b418)
    bool forward = (error < 0);
    gpio_num_t drive_pin = forward ? VALVE_PIN_FWD : VALVE_PIN_REV;
    gpio_num_t dir_pin   = forward ? VALVE_PIN_REV : VALVE_PIN_FWD;

    // Start motor
    gpio_set_direction(dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dir_pin, 0);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .period_ticks = 500,  // 20kHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_cfg, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_new_operator(&oper_cfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr);
    mcpwm_comparator_set_compare_value(cmpr, 200);  // initial duty, updated in loop @ 20kHz
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = drive_pin };
    mcpwm_new_generator(oper, &gen_cfg, &gen);
    mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmpr, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // Kick duration scales tightly with move size.
    // At 480 duty the valve moves ~1.5 deg in 50ms -- more than one inter-ring step.
    // Short moves need a proportionally shorter kick.
    float abs_err_kick = fabsf(error);
    uint32_t kick_ms = (abs_err_kick <  1.0f) ?  8 :
                       (abs_err_kick <  2.0f) ? 12 :
                       (abs_err_kick <  5.0f) ? 20 :
                       (abs_err_kick < 15.0f) ? 30 : 20;
    mcpwm_comparator_set_compare_value(cmpr, 480); // ~96% duty kickstart
    vTaskDelay(pdMS_TO_TICKS(kick_ms));
    mcpwm_comparator_set_compare_value(cmpr, 200); // back to run duty

    // Poll position and current while motor runs
    uint32_t elapsed = kick_ms + POLL_MS;
    bool reached = false;
    bool stalled  = false;
    float peak_ma = 0;
    int low_current_count = 0;
    int kick_throughs = 0;  // dead-spot kickthrough count
    float prev_deg = current_deg;  // for velocity estimation

    while (elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        elapsed += POLL_MS;

        // Sample current
        uint32_t mv = adc_mv(ADC_CH_VCUR);
        float ma = CURRENT_MA(mv);
        if (ma > peak_ma) peak_ma = ma;

        if (ma < MIN_MA) {
            if (++low_current_count >= 5) {  // 50ms of low current
                // Only kick if error is significant -- inter-ring steps (0.3-0.6 deg)
                // reach target and naturally stall; 100ms kick would blast past them.
                // Seg-2 reopens from closed arrive at friction zone with ~4 deg error.
                if (fabsf(error) > tolerance_deg && fabsf(error) > 1.5f && kick_throughs < 3) {
                    // Friction/gear tight spot -- apply burst to push through.
                    kick_throughs++;
                    uint32_t kick_t = 100u;  // 100ms -- targeting ~0 error (extrapolated from 40/60/80ms data)
                    ESP_LOGW(TAG, "  Dead-spot kick %d at %.1f deg (err %.1f) -- boosting %dms",
                             kick_throughs, current_deg, error, kick_t);
                    mcpwm_comparator_set_compare_value(cmpr, 480);
                    vTaskDelay(pdMS_TO_TICKS(kick_t));
                    mcpwm_comparator_set_compare_value(cmpr, 200);
                    elapsed += kick_t;
                    low_current_count = 0;
                    prev_deg = current_deg;  // reset velocity after kick
                } else {
                    ESP_LOGW(TAG, "  Low current %.0f mA -- motor not running?", ma);
                    break;
                }
            }
        } else {
            low_current_count = 0;
        }

        if (ma > STALL_MA) {
            stalled = true;
            ESP_LOGW(TAG, "  Stall %.0f mA", ma);
            break;
        }

        // Read position
        if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) break;
        float new_deg = raw * DEG_PER_COUNT;

        // Velocity estimate (deg/s), wrap-safe
        float delta = new_deg - prev_deg;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        float vel_dps = delta / (POLL_MS / 1000.0f);
        prev_deg    = new_deg;
        current_deg = new_deg;

        error = target_deg - current_deg;
        if (error >  180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        float abs_err = fabsf(error);

        // Predictive braking
        float stopping_dist = 0.0f;
        if (elapsed > kick_ms + 3*POLL_MS)
            stopping_dist = fabsf(vel_dps) * COAST_S;

        // Proportional duty
        uint32_t new_duty;
        if      (abs_err > 10.0f) new_duty = 350;
        else if (abs_err >  5.0f) new_duty = 175;
        else                      new_duty = 125;
        mcpwm_comparator_set_compare_value(cmpr, new_duty);

        if (verbose)
            ESP_LOGI(TAG, "  pos=%.1f err=%.1f vel=%.1f stop=%.1f ma=%.0f",
                     current_deg, error, vel_dps, stopping_dist, ma);

        // Stop conditions
        if (abs_err <= tolerance_deg)                        { reached = true; break; }
        if (forward  && error > 0)                           { reached = true; break; }
        if (!forward && error < 0)                           { reached = true; break; }
        if (stopping_dist >= abs_err + tolerance_deg * 0.5f) { reached = true; break; }
    }

    // Stop motor
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(timer);
    mcpwm_del_generator(gen);
    mcpwm_del_comparator(cmpr);
    mcpwm_del_operator(oper);
    mcpwm_del_timer(timer);
    gpio_set_level(drive_pin, 0);
    gpio_set_level(dir_pin, 0);

    if (!reached && !stalled)
        ESP_LOGW(TAG, "  Timeout after %lu ms", elapsed);

    ESP_LOGI(TAG, "  Final: %.1f deg (target %.1f, error %.1f, peak %.0f mA)",
             current_deg, target_deg, error, peak_ma);

    s_valve_deg_cached = current_deg;  // confirmed position for HA valve_open sensor

    return reached && !stalled && peak_ma >= MIN_MA;
}

static void phase_valve_goto(void)
{
    STEP("Valve Go-To Position");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    uint16_t raw = 0;
    as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
    float current_deg = raw * (360.0f / 4096.0f);
    INFO("Current valve position: %.1f deg", current_deg);
    INFO("Enter target angle in degrees (0-360), then Enter. 'q' to quit.");

    char buf[16];
    while (true) {
        INFO("Target deg> ");
        // Read a line
        int i = 0;
        memset(buf, 0, sizeof(buf));
        while (i < (int)sizeof(buf)-1) {
            int c = uart_getchar(30000);
            if (c == 0 || c == 'q' || c == 'Q') goto done;
            if (c == '\r' || c == '\n') { printf("\r\n"); break; }
            if (c == 8 || c == 127) {  // backspace
                if (i > 0) { i--; buf[i] = 0; printf("\b \b"); }
                continue;
            }
            buf[i++] = (char)c;
            printf("%c", c);  // echo
        }
        if (i == 0) continue;

        float target = atof(buf);
        if (target < 0.0f || target > 360.0f) {
            INFO("Invalid angle %.1f -- enter 0-360", target);
            continue;
        }

        INFO("Moving to %.1f deg...", target);
        bool ok = valve_goto(target, 2.0f, 10000, false);
        result(ok, "Valve reached target position");

        // Re-read final position
        as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
        current_deg = raw * (360.0f / 4096.0f);
        INFO("Position: %.1f deg", current_deg);
    }
done:
    motor_rail_off();
    INFO("Done.");
}

static void phase_sleep(void)
{
    STEP("Deep Sleep / Wake Test");

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        PASS("Woke from RTC timer -- timer wake works");
        return;
    }
    // Note: EXT0 (Hall sensor) wake not used -- Hall sensor interrupts power causing reboot

    INFO("NOTE: Hall sensor interrupts power -- device will reboot, not wake.");
    INFO("Enter sleep duration in seconds (default=300): ");

    char buf[16];
    int i = 0;
    memset(buf, 0, sizeof(buf));
    while (i < (int)sizeof(buf)-1) {
        int c = uart_getchar(10000);
        if (c == 0) break;
        if (c == '\r' || c == '\n') { printf("\r\n"); break; }
        if (c == 8 || c == 127) { if (i > 0) { i--; buf[i] = 0; printf("\b \b"); } continue; }
        buf[i++] = (char)c;
        printf("%c", c);
    }
    uint32_t sleep_sec = (i > 0) ? (uint32_t)atoi(buf) : 300;
    if (sleep_sec == 0) sleep_sec = 300;
    INFO("Sleeping for %lu seconds...", sleep_sec);
    vTaskDelay(pdMS_TO_TICKS(2000));

    motor_rail_off();

    // Set TCA6408A to LED_OFF before powering off sensor rail:
    // bit0=HIGH (red off via TCA -- BQ25504 charger drives red independently during sleep)
    // bit1=LOW, bit2=LOW (green and blue off)
    // During sleep: red lights only when solar charging is active.
    tca_led_set(LED_OFF);

    sensor_rail_off();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Only timer wakeup -- Hall sensor interrupts power causing a full reboot.
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_sec * 1000000ULL);

    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase M: Continuous monitor
// ─────────────────────────────────────────────────────────────────────────────
static void phase_monitor(void)
{
    STEP("Continuous Monitor (any key to stop)");
    sensor_rail_on();
    adc_setup();

    while (true) {
        if (uart_getchar(10) != 0) break;

        uint16_t va = 0, na = 0;
        uint8_t  vs = 0, ns = 0;
        as5600_read(ADDR_AS5600L, &va, NULL, &vs);
        as5600_read(ADDR_AS5600,  &na, NULL, &ns);

        float psi = 0.0f;
        mprls_read(&psi);

        uint32_t vb  = adc_mv(ADC_CH_VBATT);
        uint32_t vc  = adc_mv(ADC_CH_VCUR);
        uint32_t nc  = adc_mv(ADC_CH_NCUR);
        uint32_t chg = adc_mv(ADC_CH_CHARGE);

        tprintf("\r  V:%.1f N:%.1f P:%.2fpsi Bat:%.2fV Chg:%.0fmA Vc:%.0fmA Nc:%.0fmA   ",
               va * 360.0f / 4096.0f,
               na * 360.0f / 4096.0f,
               psi,
               vb * VBATT_DIVIDER_RATIO / 1000.0f,
               CURRENT_MA(chg),
               CURRENT_MA(vc),
               CURRENT_MA(nc));
        vTaskDelay(pdMS_TO_TICKS(480));
    }
    tprintf("\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TCP Terminal Server
// ─────────────────────────────────────────────────────────────────────────────
static void tcp_server_task(void *arg)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "TCP: socket create failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "TCP: bind failed");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "TCP terminal listening on port %d", TCP_PORT);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // If another client is already connected, close the old one
        if (s_tcp_client_fd >= 0) {
            close(s_tcp_client_fd);
        }

        ESP_LOGI(TAG, "TCP client connected from " IPSTR,
                 IP2STR((esp_ip4_addr_t *)&client_addr.sin_addr));

        s_tcp_client_fd = client_fd;

        // Send a welcome
        const char *welcome = "\r\n=== OtO Bring-up ===\r\n"
                              "Type a menu key to run a test.\r\n\r\n";
        send(client_fd, welcome, strlen(welcome), 0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi + Blue LED + OTA
// ─────────────────────────────────────────────────────────────────────────────
static void tca_led_set(uint8_t val)
{
    sensor_rail_on();
    if (!s_tca_outputs) {
        // First call: pre-load output latch HIGH, then switch to outputs
        uint8_t off = LED_OFF;
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &off, 1);
        uint8_t all_out = 0x00;
        i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_CONFIG, &all_out, 1);
        s_tca_outputs = true;
    }
    // Config already all-outputs -- just write the output register
    i2c_bus_write_reg(ADDR_TCA6408A, TCA6408A_REG_OUTPUT, &val, 1);
}

static void led_blink_task(void *arg)
{
    bool state = false;
    while (!s_wifi_connected) {
        state = !state;
        if (state) {
            // Turning ON: first explicitly write OFF to ensure clean transition
            tca_led_set(LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(10));
            tca_led_set(LED_BLUE);
        } else {
            tca_led_set(LED_OFF);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // WiFi connected: solid blue
    tca_led_set(LED_BLUE);
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        s_wifi_ip[0] = '\0';
        if (s_wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGI(TAG, "WiFi retry %d/%d...", s_wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected! IP: %s", s_wifi_ip);
        ESP_LOGI(TAG, "OTA push:  curl -X POST --data-binary @build/oto_bringup.bin http://%s:%d/update",
                 s_wifi_ip, OTA_HTTP_PORT);
        ESP_LOGI(TAG, "Zone UI:   http://%s/  (or /zone)", s_wifi_ip);
        s_wifi_retry_count = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

#ifndef ESPHOME_COMPONENT
static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_netif = esp_netif_create_default_wifi_sta();
    char _wifi_hn[32]={0}; name_to_hostname(s_device_name, _wifi_hn, sizeof(_wifi_hn));
    esp_netif_set_hostname(s_wifi_netif, _wifi_hn);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = CONFIG_OTO_WIFI_SSID,
            .password = CONFIG_OTO_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA starting, SSID: %s", CONFIG_OTO_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected to %s", CONFIG_OTO_WIFI_SSID);
    } else {
        ESP_LOGW(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
    }
}
#endif /* !ESPHOME_COMPONENT */

// ─────────────────────────────────────────────────────────────────────────────
// HTTP OTA Server (push-based: curl -X POST --data-binary @firmware.bin http://<ip>:8080/update)
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t ota_update_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    ESP_LOGI(TAG, "OTA push update started, content length: %d", req->content_len);
    s_ota_in_progress = true;
    TOUCH_ACTIVITY();  // reset inactivity timer at OTA start

    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (update_part == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        s_ota_in_progress = false;
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        s_ota_in_progress = false;
        return ESP_FAIL;
    }

    char buf[1024];
    int total = 0;
    int ret;
    while ((ret = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        err = esp_ota_write(ota_handle, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            s_ota_in_progress = false;
            return ESP_FAIL;
        }
        total += ret;
        if ((total % (64 * 1024)) < (int)sizeof(buf)) {
            ESP_LOGI(TAG, "OTA progress: %d bytes", total);
        }
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "OTA receive error");
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        s_ota_in_progress = false;
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed (bad image?)");
        s_ota_in_progress = false;
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        s_ota_in_progress = false;
        return ESP_FAIL;
    }

    s_ota_in_progress = false;  // about to reboot
    ESP_LOGI(TAG, "OTA update complete (%d bytes). Rebooting...", total);
    const char *resp = "OTA OK, rebooting...\n";
    httpd_resp_send(req, resp, strlen(resp));

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// -----------------------------------------------------------------------
// Zone editor web UI  (port 80)
// Endpoints:
//   GET  /            -> zone_editor HTML page

static void ota_http_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port       = OTA_HTTP_PORT;
    config.ctrl_port         = 32769;
    config.max_open_sockets  = 3;    // OTA needs 1; give extra for retries
    config.lru_purge_enable  = true;
    config.recv_wait_timeout = 60;   // large binary over WiFi needs time
    config.send_wait_timeout = 10;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "OTA HTTP server failed to start");
        return;
    }

    httpd_uri_t ota_uri = {
        .uri      = "/update",
        .method   = HTTP_POST,
        .handler  = ota_update_handler,
    };
    httpd_register_uri_handler(server, &ota_uri);
    ESP_LOGI(TAG, "OTA HTTP server on port %d (POST /update)", OTA_HTTP_PORT);
}

// ─────────────────────────────────────────────────────────────────────────────
// Run all
// ─────────────────────────────────────────────────────────────────────────────
static void run_all(void)
{
    phase_i2c_scan();
    phase_valve_sensor();
    phase_nozzle_sensor();
    phase_pressure();
    phase_tca6408a();
    phase_adc();
    phase_valve_motor();
    phase_nozzle_motor();
    phase_kill();
    tprintf("\n================================\n");
    tprintf("  All phases complete\n");
    tprintf("  PASS: %d   FAIL: %d\n", s_pass, s_fail);
    tprintf("================================\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// Menu
// ─────────────────────────────────────────────────────────────────────────────
static void print_menu(void)
{
    tprintf("\n");
    tprintf("========================================\n");
    tprintf("  OtO Hardware Bring-up    P:%-3d F:%-3d\n", s_pass, s_fail);
    tprintf("  WiFi: %s\n", s_wifi_connected ? "connected" : "not connected");
    tprintf("========================================\n");
    tprintf("  --- Sensors ---\n");
    tprintf("  1  I2C bus scan\n");
    tprintf("  2  AS5600L valve position\n");
    tprintf("  3  AS5600  nozzle position\n");
    tprintf("  4  MPRLS pressure sensor\n");
    tprintf("  6  ADC battery + current sense\n");
    tprintf("  --- Actuators ---\n");
    tprintf("  7  Valve motor test (both directions)\n");
    tprintf("  8  Nozzle motor test (both directions)\n");
    tprintf("  9  Kill/brake signal test\n");
    tprintf("  j  Jog valve motor (f=fwd r=rev q=quit)\n");
    tprintf("  B  Valve jog resolution explorer (CSV)\n");
    tprintf("  d  Valve reversal dead-time + hysteresis (EXP, CSV)\n");
    tprintf("  h  Encoder health scan (AGC/field vs valve angle)\n");
    tprintf("  p  Move valve to position (degrees)\n");
    tprintf("  n  Move nozzle to position (degrees)\n");
    tprintf("  x  Pressure calibration scan\n");
    tprintf("  %%  Valve pressure goto (open-loop %% of max)\n");
    tprintf("  g  Pressure monitor (CSV log over time)\n");
    tprintf("  i  Water zone (1/8\" / 1/4\" / demo)\n");
    tprintf("  z  Zone perimeter setup\n");
    tprintf("  k  Nozzle speed calibration (wet - connect water first)\n");
    tprintf("  G  Nozzle jog pulse calibration (deg/pulse, wet)\n");
    tprintf("  f  Nozzle stall point finder\n");
    tprintf("  e  View nozzle speed calibration\n");
    tprintf("  o  View zone perimeter\n");
    tprintf("  c  View / edit calibration table\n");
    tprintf("  --- LEDs ---\n");
    tprintf("  5  TCA6408A LED scan (all bits)\n");
    tprintf("  l  LED colour test (0-7 g b c q)\n");
    tprintf("  --- Power / Debug ---\n");
    tprintf("  t  Toggle 3V3Sen rail\n");
    tprintf("  v  Toggle 9V motor rail\n");
    tprintf("  s  Deep sleep / wake test\n");
    tprintf("  u  Auto-sleep: %s\n",
            s_sleep_disabled ? "DISABLED (press u to re-enable)"
                             : "enabled  (press u to disable)");
    tprintf("  m  Continuous monitor\n");
    tprintf("  w  WiFi + OTA status\n");
    tprintf("  a  Run all phases\n");
    tprintf("  r  Reset pass/fail counters\n");
    tprintf("  R  Reboot\n");
    tprintf("========================================\n");
    tprintf("Select: ");
    fflush(stdout);
}

// ─────────────────────────────────────────────────────────────────────────────
// app_main
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// Power management helpers
// ─────────────────────────────────────────────────────────────────────────────

// Forward declarations for power-management helpers defined further down
// (next to the auto-sleep getters/setters). pm_record_reason is called from
// sleep_forever / check_inactivity which sit above the implementations.
static void pm_record_reason(const char *reason);
static void pm_nvs_save_u8(const char *key, uint8_t v);

// Record a granular watering-completion reason. First non-zero code wins
// (so a water-loss detection earlier in a run isn't overwritten by the
// "cancelled" call from stop_and_close when it tears the run down).
// Always also flags s_water_abort = true so the loop unwinds cleanly.
static inline void water_set_status(int code)
{
    if (s_water_status_code == WATER_STATUS_COMPLETED) s_water_status_code = code;
    s_water_abort = true;
}

// Safe pre-sleep prep: close valve, blank LED, power down rails. Used by
// every sleep path so the device always sleeps in a known-safe state.
static void prepare_for_sleep(void)
{
    motor_rail_on();
    sensor_rail_on();
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    tca_led_set(LED_OFF);
    sensor_rail_off();
    motor_rail_off();
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void sleep_forever(const char *reason)
{
    ESP_LOGW(TAG, "Sleeping forever: %s", reason);
    pm_record_reason(reason);
    prepare_for_sleep();
    // No wakeup source -- only a power cycle or Hall sensor reset will wake.
    // Used for low-battery emergency: this runs from check_battery_on_boot()
    // BEFORE the ESPHome loop is up, so we have to sleep directly.
    esp_deep_sleep_start();
}

// b347: persistent boot diagnostics. Captures what the boot-time battery +
// valve checks observed and writes the record to NVS BEFORE the device can
// reach the working state (and BEFORE sleep_forever_quiet on a low-battery
// abort, since that path never returns). Backstop for the 2026-05-26
// incident where a TaskWDT during smooth watering rebooted the device but
// left the valve open through the reboot; we couldn't tell whether the
// boot-time close attempt fired because every diagnostic line went to UART.
// Kept small (~48 B per entry) so the 5-deep ring fits comfortably in NVS.
#define BOOT_DIAG_RING_N 5

#define BOOT_DIAG_FLAG_CLOSE_ATTEMPTED     0x01
#define BOOT_DIAG_FLAG_CLOSE_OK            0x02
#define BOOT_DIAG_FLAG_BATT_SKIPPED_VALVE  0x04
#define BOOT_DIAG_FLAG_BATT_TOO_LOW        0x08
#define BOOT_DIAG_FLAG_SENSOR_READ_FAILED  0x10

typedef struct {
    uint32_t boot_seq;                 // monotonic across boots
    int64_t  boot_epoch;               // unix time at write; 0 if clock not synced
    uint16_t fw_build;
    int8_t   reset_reason;             // esp_reset_reason() (ESP_RST_* enum)
    uint8_t  flags;                    // BOOT_DIAG_FLAG_*
    float    batt_v_at_boot;
    float    batt_v_after_close;       // post valve_goto reading (sag witness); 0 if N/A
    float    valve_deg_at_boot;        // -1.0 if read failed
    float    valve_err_at_boot;        // wrapped (-180, 180]
    float    valve_deg_after_close;    // -1.0 if no drive attempted
} __attribute__((packed)) boot_diag_t;

typedef struct {
    uint16_t version;                  // 1
    uint16_t count;                    // 0..BOOT_DIAG_RING_N
    uint16_t next_idx;                 // next slot to write (mod N)
    uint16_t reserved;
    boot_diag_t ring[BOOT_DIAG_RING_N];
} __attribute__((packed)) boot_diag_blob_t;

static void boot_diag_load_all(boot_diag_blob_t *out)
{
    memset(out, 0, sizeof(*out));
    out->version = 1;
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(*out);
    esp_err_t r = nvs_get_blob(h, "boot_diag", out, &sz);
    nvs_close(h);
    if (r != ESP_OK || sz != sizeof(*out) || out->version != 1) {
        memset(out, 0, sizeof(*out));
        out->version = 1;
    }
}

static uint32_t boot_diag_next_seq(void)
{
    boot_diag_blob_t blob;
    boot_diag_load_all(&blob);
    uint32_t mx = 0;
    for (int i = 0; i < blob.count; i++) {
        if (blob.ring[i].boot_seq > mx) mx = blob.ring[i].boot_seq;
    }
    return mx + 1;
}

static void boot_diag_append_and_save(const boot_diag_t *entry)
{
    boot_diag_blob_t blob;
    boot_diag_load_all(&blob);
    if (blob.next_idx >= BOOT_DIAG_RING_N) blob.next_idx = 0;
    blob.ring[blob.next_idx] = *entry;
    blob.next_idx = (blob.next_idx + 1) % BOOT_DIAG_RING_N;
    if (blob.count < BOOT_DIAG_RING_N) blob.count++;
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "boot_diag", &blob, sizeof(blob));
    nvs_commit(h);
    nvs_close(h);
}

// Reset reason → short string (mirrors the names used by the reset_reason
// text_sensor in esphome/irrigoto-core.yaml so HA users see the same labels).
static const char *boot_diag_reset_reason_str(int r)
{
    switch ((esp_reset_reason_t)r) {
        case ESP_RST_POWERON:   return "PowerOn";
        case ESP_RST_EXT:       return "ExternalPin";
        case ESP_RST_SW:        return "Software";
        case ESP_RST_PANIC:     return "Panic/Crash";
        case ESP_RST_INT_WDT:   return "InterruptWDT";
        case ESP_RST_TASK_WDT:  return "TaskWDT";
        case ESP_RST_WDT:       return "OtherWDT";
        case ESP_RST_DEEPSLEEP: return "DeepSleepWake";
        case ESP_RST_BROWNOUT:  return "Brownout";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "Unknown";
    }
}

// b327: Quiet variant of sleep_forever -- no motor activity. Used when
// boot-time battery is critically low: powering the motor rail and driving
// the valve under low battery can brown out the regulator and make things
// worse (or fail mid-move leaving the valve in an unknown state). Better
// to leave the valve where it is and just back out cleanly.
static void sleep_forever_quiet(const char *reason)
{
    ESP_LOGW(TAG, "Sleeping forever (quiet, no motors): %s", reason);
    pm_record_reason(reason);
    tca_led_set(LED_OFF);
    sensor_rail_off();
    motor_rail_off();
    vTaskDelay(pdMS_TO_TICKS(100));
    // No wakeup source -- physical reset required to revive. Matches
    // sleep_forever()'s rationale: waking again under the same low battery
    // would just measure low and sleep again, burning the dregs.
    esp_deep_sleep_start();
}

// b327: On a healthy-battery wake, verify the valve is closed. Catches
// any prior unclean shutdown (brownout / panic / watchdog / forced reset)
// that may have left the valve open with water flowing. Requires enough
// battery headroom that the close-attempt itself won't trip the brownout
// detector.
//
// b347: diag is optional — if non-NULL, the valve angle observed, the
// close-attempt outcome, and the post-drive battery sag are recorded
// into the boot_diag entry so /api/boot_diag can surface them later.
static void check_valve_closed_on_boot(boot_diag_t *diag)
{
    motor_rail_on();    // also releases the brake (GPIO_K low)
    // sensor_rail_on() already on from check_battery_on_boot above
    vTaskDelay(pdMS_TO_TICKS(150));   // let rails + AS5600L settle

    uint16_t v_raw = 0;
    // b348: as5600_read returns bool (true on success), NOT esp_err_t.
    // The original b327 check_valve_closed_on_boot used `!= ESP_OK`, which
    // meant `true != 0` → ALWAYS treated success as failure → silently
    // skipped the valve-close every single boot. boot_diag (b347) caught
    // this with sensor_read_failed=true on 5/5 captured boots even though
    // the sensor was fine. Fixed here.
    if (!as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL)) {
        ESP_LOGW(TAG, "Wake valve check: AS5600L read failed -- skipping");
        if (diag) diag->flags |= BOOT_DIAG_FLAG_SENSOR_READ_FAILED;
        motor_rail_off();
        return;
    }
    float cur_valve = v_raw * (360.0f / 4096.0f);
    s_valve_deg_cached = cur_valve;  // seed confirmed-state cache at boot

    // Tolerance check around VALVE_CLOSED_DEG (231). Wrap to (-180, 180]
    // so a small drift across the 0-360 boundary doesn't read as huge.
    const float CLOSE_TOL_DEG = 3.0f;
    float err = cur_valve - VALVE_CLOSED_DEG;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    if (diag) {
        diag->valve_deg_at_boot = cur_valve;
        diag->valve_err_at_boot = err;
    }

    if (fabsf(err) <= CLOSE_TOL_DEG) {
        ESP_LOGI(TAG, "Wake valve check: %.1f deg (closed, err %.1f) -- ok",
                 cur_valve, err);
        motor_rail_off();
        return;
    }

    ESP_LOGW(TAG, "Wake valve check: %.1f deg (err %.1f from closed) -- driving closed",
             cur_valve, err);
    bool drove_ok = valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);

    // Re-read the sensor to confirm we actually got there. valve_goto's
    // internal tolerance check could be fooled by a transient sensor
    // glitch; the second read is the post-mortem ground truth that the
    // user will see in /api/boot_diag.
    uint16_t v_raw2 = 0;
    float cur_valve2 = -1.0f, err2 = 0.0f;
    if (as5600_read(ADDR_AS5600L, &v_raw2, NULL, NULL)) {
        cur_valve2 = v_raw2 * (360.0f / 4096.0f);
        err2 = cur_valve2 - VALVE_CLOSED_DEG;
        while (err2 >  180.0f) err2 -= 360.0f;
        while (err2 < -180.0f) err2 += 360.0f;
    }

    // Sample battery again — the motor pull during valve_goto sags the
    // pack. A 4.0V battery dropping to 3.4V here is a useful flag that
    // the cell is loaded down even though steady-state V looked fine.
    uint32_t mv_after = adc_mv(ADC_CH_VBATT);
    float v_after = (mv_after * VBATT_DIVIDER_RATIO) / 1000.0f;

    if (diag) {
        diag->flags |= BOOT_DIAG_FLAG_CLOSE_ATTEMPTED;
        if (drove_ok && cur_valve2 >= 0.0f && fabsf(err2) <= CLOSE_TOL_DEG) {
            diag->flags |= BOOT_DIAG_FLAG_CLOSE_OK;
        }
        diag->valve_deg_after_close = cur_valve2;
        diag->batt_v_after_close    = v_after;
    }

    motor_rail_off();
    ESP_LOGI(TAG, "Wake valve check: post-drive %.1f deg (err %.1f), batt %.2fV%s",
             cur_valve2, err2, v_after, drove_ok ? "" : " [drive timed out]");
}

static void check_battery_on_boot(void)
{
    sensor_rail_on();
    adc_setup();
    vTaskDelay(pdMS_TO_TICKS(100));  // let rail settle

    uint32_t mv = adc_mv(ADC_CH_VBATT);
    float batt_v = (mv * VBATT_DIVIDER_RATIO) / 1000.0f;
    ESP_LOGI(TAG, "Battery on boot: %.2fV", batt_v);

    // b347: build the boot_diag record as we go. Written to NVS before
    // returning OR before sleep_forever_quiet on the low-batt branch
    // (which never returns), so post-mortem inspection always has
    // SOMETHING to read even when the device immediately re-sleeps.
    boot_diag_t diag = {0};
    diag.boot_seq              = boot_diag_next_seq();
    diag.fw_build              = (uint16_t)FW_BUILD;
    diag.reset_reason          = (int8_t)esp_reset_reason();
    diag.batt_v_at_boot        = batt_v;
    diag.valve_deg_at_boot     = -1.0f;
    diag.valve_deg_after_close = -1.0f;
    // boot_epoch will usually be 0 here — system clock hasn't synced
    // yet via SNTP. HA-side rendering treats 0 as "unsynced at write"
    // and falls back to boot_seq ordering.
    diag.boot_epoch            = (int64_t)time(NULL);

    if (batt_v < BATT_MIN_VOLTAGE_V && batt_v > 0.5f) {
        // > 0.5V check avoids false trigger if ADC not ready.
        // Quiet sleep -- do NOT try to close the valve. Motor draw under
        // low battery can brown out the regulator; valve stays where it is.
        ESP_LOGW(TAG, "Battery %.2fV below %.1fV threshold -- quiet sleep, no motor activity",
                 batt_v, BATT_MIN_VOLTAGE_V);
        diag.flags |= BOOT_DIAG_FLAG_BATT_TOO_LOW;
        boot_diag_append_and_save(&diag);
        sleep_forever_quiet("low battery");
    }

    // Battery is at least marginal. If we have enough headroom for a motor
    // move without dipping near the brownout point, verify the valve is in
    // the closed position. Backstop against unclean shutdowns (brownout /
    // panic / WDT / forced reset) that may have left the valve open mid-
    // watering. Skipped between 3.6 and 3.9 V -- device boots normally but
    // we don't risk a sag below 3.6 from a motor pull.
    if (batt_v >= BATT_VALVE_SAFE_VOLTAGE_V) {
        check_valve_closed_on_boot(&diag);
    } else if (batt_v > 0.5f) {
        ESP_LOGI(TAG, "Battery %.2fV between %.1fV and %.1fV -- skipping wake-time valve check",
                 batt_v, BATT_MIN_VOLTAGE_V, BATT_VALVE_SAFE_VOLTAGE_V);
        diag.flags |= BOOT_DIAG_FLAG_BATT_SKIPPED_VALVE;
    }

    boot_diag_append_and_save(&diag);
}

static void check_inactivity(void)
{
    if (s_last_activity == 0) return;  // not yet initialised
    if (s_ota_in_progress) return;  // never sleep during OTA
    if (s_sleep_disabled)  return;  // user disabled auto-sleep
    // Suppress sleep while a web client is actively polling.
    // s_last_web_req_tick is set by WEB_TOUCH() in every zone web handler.
    // A client polling every 900ms will keep this fresh; a closed browser
    // stops updating it, and after WEB_CLIENT_TIMEOUT_MS sleep is allowed.
    if (s_last_web_req_tick > 0) {
        TickType_t web_idle = xTaskGetTickCount() - s_last_web_req_tick;
        if (web_idle < pdMS_TO_TICKS(WEB_CLIENT_TIMEOUT_MS)) return;
    }
    TickType_t idle = xTaskGetTickCount() - s_last_activity;
    if (idle >= pdMS_TO_TICKS(s_inactivity_ms)) {
        // Schedule-aware deferral: if a scheduled run is imminent, don't
        // sleep — let the schedule executor fire it on the next poll.
        // Without this guard, the inactivity timer could win a tight race
        // against the scheduler's poll cadence and put the device to sleep
        // ~seconds before the schedule was supposed to run.
        time_t now_t = time(NULL);
        if (now_t > 1700000000) {
            time_t next_t = 0;
            int    next_zone = 0;
            // Defer if scheduled run is within 120 s — gives the
            // schedule task time to poll and fire instead.
            if (irrigoto_schedule_next_run(now_t, &next_t, &next_zone)
                    && next_t - now_t < (time_t)120) {
                INFO("Inactivity due, but zone %d schedule fires in %lds — staying awake",
                     next_zone, (long)(next_t - now_t));
                // Touch activity so we don't re-trigger this every tick;
                // we'll re-evaluate after the next inactivity window.
                TOUCH_ACTIVITY();
                return;
            }
        }
        tprintf("\nNo activity for %u min -- sleeping.\n",
                (unsigned)(s_inactivity_ms / 60000u));
        vTaskDelay(pdMS_TO_TICKS(200));
#ifdef ESPHOME_COMPONENT
        // Route through ESPHome's deep_sleep so HA gets a clean disconnect.
        irrigoto_sleep_now_with_reason(s_sleep_dur_s, "inactivity");
#else
        sleep_forever("inactivity timeout");
#endif
    }
}

// -----------------------------------------------------------------------
// -------------------------------------------------------------------
// Water zone constants
// -------------------------------------------------------------------
#define WATER_SECTOR_DEG            10.0f
#define WATER_SECTORS               36
#define WATER_MAX_RINGS_CAL         36
#define WATER_RING_SPACING         700.0f
#define WATER_MIN_RING_SPACING      80.0f
#define WATER_PRESSURE_TOL           0.15f
#define WATER_PRESSURE_ITER          8
#define WATER_MAX_THROW_MM        8534.0f
#define WATER_MIN_THROW_MM         461.0f
#define WATER_MIN_ELLIPSE_THROW_MM 1829.0f
#define WATER_INNER_SPLASH_RADIUS_MM 300.0f

// Convert a geometric-annulus deposited depth into the physically-true depth
// that matches the footprint the dps solver actually sized the sweep for.
//
// For rings inside WATER_MIN_ELLIPSE_THROW_MM the solver (see ~line 9279)
// spreads the stream over a fixed WATER_INNER_SPLASH_RADIUS_MM-wide splash
// band -- a stream landing that close wets ~300mm radially; it cannot lay a
// stripe into the sub-100mm geometric gap between adjacent inner rings. But
// the cumulative-depth tracker (and hence the CSV / heatmap) credits that
// water to the NARROW geometric annulus, inflating reported depth 2-4x and
// painting the bogus "hot core". (The tracker's own header comment claimed it
// "uses the same exact annular area as the dps formula" -- false for these
// rings; this is the fix.)
//
// depth = water / area, and both A_geo and A_splash are pass-invariant
// constants for a ring, so scaling the accumulated geometric depth by
// A_geo/A_splash exactly reconstructs the full multi-pass splash-area depth
// without needing per-pass history. Outer rings (>= threshold) already used
// the geometric annulus in the solver, so they pass through unchanged.
//
// DISPLAY ONLY: callers apply this to the value written to the CSV / depth_mm,
// never to the in-loop smooth_cumulative_depth that drives pass decisions --
// the adaptive loop's watering behavior is intentionally left untouched.
static inline float smooth_display_depth_mm(float geo_depth,
                                            float ring_throw_mm,
                                            float inner_throw_mm)
{
    if (ring_throw_mm >= WATER_MIN_ELLIPSE_THROW_MM) return geo_depth;
    float a_geo    = ring_throw_mm * ring_throw_mm
                   - inner_throw_mm * inner_throw_mm;
    float ro       = ring_throw_mm + WATER_INNER_SPLASH_RADIUS_MM * 0.5f;
    float ri       = fmaxf(0.0f, ring_throw_mm - WATER_INNER_SPLASH_RADIUS_MM * 0.5f);
    float a_splash = ro * ro - ri * ri;
    if (a_geo <= 1.0f || a_splash <= 1.0f) return geo_depth;
    return geo_depth * (a_geo / a_splash);
}

// Read actual max and min throw from the calibration table.
// Returns the measured throw at full open / closed from stored throw_mm values.
// Falls back to the compile-time constants if the cal table has no throw data.
static float cal_get_max_throw_mm(void)
{
    pressure_map_t c = {0};
    if (cal_load_primary(&c) == ESP_OK && c.num_points > 0) {
        float mx = 0;
        for (int i = 0; i < c.num_points; i++)
            if (c.throw_mm[i] > mx) mx = c.throw_mm[i];
        if (mx > 100.0f) return mx;
    }
    return WATER_MAX_THROW_MM;  // fallback to compile-time constant
}

static float cal_get_min_throw_mm(void)
{
    pressure_map_t c = {0};
    if (cal_load_primary(&c) == ESP_OK && c.num_points > 0) {
        float mn = 99999.0f;
        for (int i = 0; i < c.num_points; i++)
            if (c.throw_mm[i] > 100.0f && c.throw_mm[i] < mn)
                mn = c.throw_mm[i];
        if (mn < 99000.0f) return mn;
    }
    return WATER_MIN_THROW_MM;  // fallback
}

static float water_perimeter_throw(const zone_perimeter_t *z,
                                   float bearing_deg,
                                   float psi_min, float psi_max)
{
    if (!z || z->num_points < 2) return cal_get_max_throw_mm();
    int n = z->num_points;

    // --- Gap detection: find largest bearing gap -> inactive arc ---
    float sb[ZONE_MAX_PERIM_POINTS];
    for (int i = 0; i < n; i++) sb[i] = z->points[i].nozzle_deg;
    for (int i = 0; i < n-1; i++)
        for (int j = i+1; j < n; j++)
            if (sb[j] < sb[i]) { float t=sb[i]; sb[i]=sb[j]; sb[j]=t; }

    float max_gap=0, gap_lo=0, gap_hi=360.0f;
    for (int i = 0; i < n; i++) {
        float b0 = sb[i];
        float b1 = (i < n-1) ? sb[i+1] : sb[0] + 360.0f;
        if (b1 - b0 > max_gap) { max_gap=b1-b0; gap_lo=b0; gap_hi=b1; }
    }
    bool in_gap = false;
    if (max_gap > 45.0f) {
        if (gap_hi <= 360.0f) in_gap = (bearing_deg > gap_lo && bearing_deg < gap_hi);
        else { float hw=gap_hi-360.0f; in_gap=(bearing_deg>gap_lo||bearing_deg<hw); }
    }
    // b435: an in-gap bearing is NOT automatically outside the zone. A single
    // long straight edge (e.g. an 80-deg chord between two far corners) can
    // legitimately span the largest vertex-bearing gap; zeroing that wedge
    // outright dried out ~40% of such a zone. Fall through to the exact ray
    // cast and zero only if the polygon has no waterable extent at this
    // bearing (a true wedge back side: closing edges hug the head, so the
    // ray cast lands below the nozzle's minimum throw).

    // --- Ray-polygon intersection ---
    // Cast a ray from the origin at bearing_deg and intersect every edge of the
    // perimeter polygon. Return the MAXIMUM intersection distance (outer boundary).
    // This correctly handles notches, steps, and concave shapes: each polygon edge
    // is evaluated independently in Cartesian space, so same-bearing step points
    // (which caused the old MAX-throw heuristic to skip notches) are handled exactly.
    //
    // Coordinate system: x = East (throw * sin(bearing)), y = North (throw * cos(bearing))
    // Ray: P(s) = s * (bsin, bcos),  s > 0
    // Edge: Q(u) = (x1,y1) + u*(dx,dy),  0 <= u <= 1
    // Solve: bsin*s - dx*u = x1
    //        bcos*s - dy*u = y1
    // det = bcos*dx - bsin*dy
    // s = (dx*y1 - dy*x1) / det
    // u = (bsin*y1 - bcos*x1) / det
    float max_mm = cal_get_max_throw_mm();
    float bsin = sinf(bearing_deg * ((float)M_PI / 180.0f));
    float bcos = cosf(bearing_deg * ((float)M_PI / 180.0f));
    float best = 0.0f;

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;

        // Throw at each endpoint (use stored throw_mm, fall back to PSI-derived)
        float ti = (z->points[i].throw_mm > 100.0f) ? z->points[i].throw_mm
                 : (psi_max > 0 ? z->points[i].pressure_psi / psi_max * max_mm : 0.0f);
        float tj = (z->points[j].throw_mm > 100.0f) ? z->points[j].throw_mm
                 : (psi_max > 0 ? z->points[j].pressure_psi / psi_max * max_mm : 0.0f);

        // Cartesian endpoints
        float ai = z->points[i].nozzle_deg * ((float)M_PI / 180.0f);
        float aj = z->points[j].nozzle_deg * ((float)M_PI / 180.0f);
        float x1 = ti * sinf(ai), y1 = ti * cosf(ai);
        float x2 = tj * sinf(aj), y2 = tj * cosf(aj);
        float dx = x2 - x1, dy = y2 - y1;

        float det = bcos * dx - bsin * dy;
        if (fabsf(det) < 1e-6f) continue;  // ray parallel to edge

        float s = (dx * y1 - dy * x1) / det;
        float u = (bsin * y1 - bcos * x1) / det;

        // Valid if: ray travels forward (s > 0) and intersection on edge (0 <= u <= 1)
        if (s > 0.5f && u >= 0.0f && u <= 1.0f && s > best)
            best = s;
    }

    if (in_gap && best < WATER_MIN_THROW_MM) return 0.0f;
    return best;
}

// b435: companion to the gap logic above for callers that restrict the sweep
// arc to the complement of the largest vertex-bearing gap. Returns true only
// if the gap is genuinely empty space (a wedge zone's inactive back side).
// Probes three bearings inside the gap; any waterable extent means the gap is
// spanned by a real polygon edge and the arc must NOT be restricted.
static bool zone_gap_is_empty(const zone_perimeter_t *z, float gap_lo,
                              float gap_deg, float psi_min, float psi_max)
{
    for (int k = 1; k <= 3; k++) {
        float b = fmodf(gap_lo + gap_deg * (float)k / 4.0f + 360.0f, 360.0f);
        if (water_perimeter_throw(z, b, psi_min, psi_max) >= WATER_MIN_THROW_MM)
            return false;
    }
    return true;
}

// Move valve to target_psi using feedforward + proportional correction.
// Returns actual PSI achieved.
static float water_hold_pressure(float target_psi, float psi_min, float psi_max)
{
    float deg = cal_pressure_to_valve_deg(target_psi);
    if (deg < 0) deg = VALVE_CAL_START_DEG;
    valve_goto(deg, 2.0f, 10000, false);
    vTaskDelay(pdMS_TO_TICKS(800));

    float actual = target_psi;
    for (int i = 0; i < WATER_PRESSURE_ITER; i++) {
        if (!mprls_read(&actual)) break;
        float err = target_psi - actual;
        if (fabsf(err) <= WATER_PRESSURE_TOL) break;
        float err_pct = err / (psi_max - psi_min) * 100.0f;
        deg += err_pct * 0.3f;
        if (deg < VALVE_CAL_START_DEG) deg = VALVE_CAL_START_DEG;
        if (deg > VALVE_OPEN_DEG)      deg = VALVE_OPEN_DEG;
        valve_goto_ex(deg, 2.0f, 5000, false, s_valve_last_dir);
        s_valve_last_dir = (err > 0) ? 1 : -1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return actual;
}

// b367: pressure-feedback valve seating for low-pressure inner rings.
// The valve->throw cal is unreliable near the closing point: gear backlash
// means reaching a low opening by CLOSING down from a higher ring lands the
// valve seat in a dead spot (no measurable flow) even when the encoder reads
// the commanded angle. Seat at VALVE_CLOSED_DEG and open UPWARD in small steps
// until measured pressure reaches target_psi -- this matches the calibration
// direction and the manual technique that produces flow down to ~1.5 ft.
// Returns the final valve angle; writes the achieved pressure to *out_psi.
static float water_seat_valve_from_closed(float target_psi, float *out_psi)
{
    // Seat fully closed so the approach is unambiguously from below.
    valve_goto_direct(VALVE_CLOSED_DEG, 2.0f, 8000, false);
    s_valve_last_dir = -1;
    vTaskDelay(pdMS_TO_TICKS(400));

    // Never aim below the detectable-flow floor.
    float want = target_psi;
    if (want < WATER_MIN_FLOW_PSI + 0.10f) want = WATER_MIN_FLOW_PSI + 0.10f;

    float deg = VALVE_CAL_START_DEG;   // first opening step lands at flow-start
    float psi = 0.0f;
    int i = 0;
    // 0.4 deg step with 0.2 deg per-step tolerance: the tolerance MUST stay
    // below the step size, otherwise valve_goto_direct sees each commanded step
    // as "already within tolerance" and never moves. 0.2 deg is ~2 encoder
    // counts (0.088 deg/count) -- near the noise floor but still reliable.
    for (; i < 128 && deg <= VALVE_OPEN_DEG; i++) {
        valve_goto_direct(deg, 0.2f, 4000, false);
        s_valve_last_dir = 1;          // every step opens further (anti-backlash)
        vTaskDelay(pdMS_TO_TICKS(220));
        if (!mprls_read_quiet(&psi)) break;
        if (psi >= want) break;
        deg += 0.4f;
    }
    INFO("  Seat-from-closed: %.2f PSI at %.1f deg (target %.2f, %d steps)",
         psi, deg, want, i + 1);
    if (out_psi) *out_psi = psi;
    return deg;
}

// -----------------------------------------------------------------------
// Water zone -- zone-aware, multi-arc serpentine
// Correctly handles active sectors that wrap through 0/360 and
// zones with multiple disconnected active arcs per ring (e.g. notch bumps).
// -----------------------------------------------------------------------

// One contiguous run of active sectors on a ring.
// first_sec..last_sec traced CW; last_sec may be < first_sec if the arc
// wraps through the 0/360 boundary.

typedef struct {
    int  first_sec;
    int  last_sec;
    bool wraps;
    int  num_secs;
} water_arc_t;

#define WATER_MAX_ARCS_PER_RING 6

static int water_find_arcs(bool active[WATER_SECTORS],
                            water_arc_t arcs[WATER_MAX_ARCS_PER_RING])
{
    int total = 0;
    for (int s = 0; s < WATER_SECTORS; s++) if (active[s]) total++;
    if (total == 0) return 0;
    if (total == WATER_SECTORS) {
        arcs[0].first_sec=0; arcs[0].last_sec=WATER_SECTORS-1;
        arcs[0].wraps=false; arcs[0].num_secs=WATER_SECTORS;
        return 1;
    }
    int scan_start = 0;
    for (int s = 0; s < WATER_SECTORS; s++) {
        if (!active[s]) { scan_start = s; break; }
    }
    int n = 0; bool in_arc = false;
    for (int i = 0; i < WATER_SECTORS; i++) {
        int s = (scan_start + i) % WATER_SECTORS;
        if (active[s] && !in_arc) {
            if (n < WATER_MAX_ARCS_PER_RING) {
                arcs[n].first_sec=s; arcs[n].last_sec=s;
                arcs[n].wraps=false; arcs[n].num_secs=1;
            }
            in_arc = true;
        } else if (active[s] && in_arc) {
            if (n < WATER_MAX_ARCS_PER_RING) {
                if (s < arcs[n].first_sec) arcs[n].wraps = true;
                arcs[n].last_sec = s;
                arcs[n].num_secs++;
            }
        } else if (!active[s] && in_arc) {
            n++; in_arc = false;
        }
    }
    if (in_arc && n < WATER_MAX_ARCS_PER_RING) n++;
    return n;
}

// -----------------------------------------------------------------------
// Water zone -- smooth open-loop sweep
//
// Strategy:
//   1. Build ring list from zone perimeter (inner to outer OR outer to inner).
//   2. For each ring, look up valve angle directly from cal table by throw_mm.
//   3. Command valve angle open-loop (tiny move from prior ring).
//   4. Immediately start nozzle sweep -- no waiting for pressure to settle.
//   5. Valve stays open throughout. No correction loops, no close/reopen.
//
// The valve moves only a few degrees between adjacent rings. It is already
// at approximately the right angle before the nozzle starts sweeping.
// -----------------------------------------------------------------------

// Interpolate valve_deg for a given throw_mm from the cal table.
// Returns -1 if cal table has no throw data.
static float cal_throw_to_valve_deg(float throw_mm)
{
    pressure_map_t c = {0};
    if (cal_load_primary(&c) != ESP_OK || c.num_points < 2) return -1.0f;

    // Find throw range in table
    float max_throw = 0, min_throw = 99999;
    for (int i = 0; i < c.num_points; i++) {
        if (c.throw_mm[i] > max_throw) max_throw = c.throw_mm[i];
        if (c.throw_mm[i] > 100 && c.throw_mm[i] < min_throw) min_throw = c.throw_mm[i];
    }
    if (max_throw < 100) return -1.0f;

    // b303: when throw is beyond cal range, extrapolate toward the pressure-
    // peak valve angle (306.7 deg). Cal tables typically end at ~298 deg,
    // leaving ~8.6 deg of unused valve-open range. For zone perimeters that
    // demand max reach (outermost rings of large zones), this unlocks extra
    // nozzle pressure -- previously cal_throw_to_valve_deg saturated at the
    // cal table's last valve_deg even when the request was clearly out of
    // range, so the valve never opened past calibrated max even though
    // there was real head room. Interpolate within (cal_max_valve,
    // VALVE_PEAK_DEG) so a small over-request gets a small extra opening
    // and a large over-request snaps to the peak. Above peak, pressure
    // drops, so we never go past it.
    if (throw_mm >= max_throw) {
        float cal_max_valve = c.valve_deg[c.num_points-1];
        if (cal_max_valve >= VALVE_PEAK_DEG) return cal_max_valve;
        // Interpolate: 0% over -> cal_max_valve, 30%+ over -> VALVE_PEAK_DEG.
        // Linear ramp from 0 to 30% over-request.
        float over_frac = (throw_mm - max_throw) / max_throw;
        if (over_frac < 0.0f) over_frac = 0.0f;
        if (over_frac > 0.30f) over_frac = 0.30f;
        float t = over_frac / 0.30f;
        return cal_max_valve + t * (VALVE_PEAK_DEG - cal_max_valve);
    }
    if (throw_mm <= min_throw) return c.valve_deg[0];

    // Linear interpolation between adjacent cal points by throw_mm
    // Cal table is ordered by valve_deg (increasing), throw_mm also increases
    for (int i = 0; i < c.num_points-1; i++) {
        float t0 = c.throw_mm[i], t1 = c.throw_mm[i+1];
        if (t0 <= 0 || t1 <= 0) continue;
        if (throw_mm >= t0 && throw_mm <= t1) {
            float frac = (t1 > t0) ? (throw_mm - t0) / (t1 - t0) : 0;
            return c.valve_deg[i] + frac * (c.valve_deg[i+1] - c.valve_deg[i]);
        }
    }
    return c.valve_deg[c.num_points-1];
}

// Sweep nozzle continuously from sweep_origin for arc_deg in direction cw.
// Position-based termination via AS5600 encoder -- handles 0/360 wrap cleanly.
// Returns false if user pressed q (abort).

// -----------------------------------------------------------------------
// nozzle_sweep_pulse -- uniform-coverage sweep using discrete pulses.
//
// Used when the ideal nozzle speed (from ring area calculation) is below
// min_continuous_dps. Each pulse advances the nozzle a small measured
// amount; the remaining dwell time is spent with the motor stopped so
// water deposits uniformly. Effective speed = deg/pulse / (pulse_ms + dwell_ms).
// deg_per_pulse is measured from the encoder after every pulse.
// -----------------------------------------------------------------------
static bool nozzle_sweep_pulse(
        float sweep_origin, float arc_deg, bool cw,
        float target_dps,
        uint16_t pulse_duty, uint32_t pulse_ms,
        TickType_t t_start,
        int ring, int arc_label, int first_sec,
        float valve_deg, float ring_throw,
        float *psi_sum_out, int *psi_n_out, bool direct_valve)
{
    const uint32_t SETTLE_MS = 120;

    // Same sweep_adj logic as continuous mode
    {
        uint16_t act_raw = 0;
        if (as5600_read(ADDR_AS5600, &act_raw, NULL, NULL)) {
            float act_pos = act_raw * (360.0f / 4096.0f);
            float behind = cw
                ? fmodf(sweep_origin - act_pos + 360.0f, 360.0f)
                : fmodf(act_pos - sweep_origin + 360.0f, 360.0f);
            if (behind > 180.0f) behind = 0.0f;
            if (behind > 0.5f && behind < 90.0f) {
                INFO("  Sweep adj (pulse): nozzle %.1f deg behind -- arc %.1f->%.1f",
                     behind, arc_deg, arc_deg + behind);
                arc_deg += behind;
                sweep_origin = act_pos;
            } else {
                float prog_now = cw
                    ? fmodf(act_pos - sweep_origin + 360.0f, 360.0f)
                    : fmodf(sweep_origin - act_pos + 360.0f, 360.0f);
                if (prog_now > 0.5f && prog_now < 180.0f) {
                    float remaining = arc_deg - prog_now;
                    if (remaining <= 0.5f) {
                        float ep = cw
                            ? fmodf(sweep_origin + arc_deg, 360.0f)
                            : fmodf(sweep_origin - arc_deg + 360.0f, 360.0f);
                        nozzle_goto_direct(ep, 1.5f, 5000, false);
                        return true;
                    }
                    arc_deg = remaining;
                }
            }
        }
    }

    gpio_num_t drv_pin = cw ? NOZZLE_PIN_FWD : NOZZLE_PIN_REV;
    gpio_num_t off_pin = cw ? NOZZLE_PIN_REV : NOZZLE_PIN_FWD;
    gpio_set_level(off_pin, 0);
    gpio_set_level(GPIO_K, 0);

    // Set up MCPWM -- duty starts at 0 (off), toggled per pulse
    mcpwm_timer_handle_t n_timer = NULL;
    mcpwm_oper_handle_t  n_oper  = NULL;
    mcpwm_cmpr_handle_t  n_cmpr  = NULL;
    mcpwm_gen_handle_t   n_gen   = NULL;
    mcpwm_timer_config_t n_tc = {
        .group_id=0, .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz=10000000, .count_mode=MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks=500 };
    mcpwm_new_timer(&n_tc, &n_timer);
    mcpwm_operator_config_t n_oc = {.group_id=0};
    mcpwm_new_operator(&n_oc, &n_oper);
    mcpwm_operator_connect_timer(n_oper, n_timer);
    mcpwm_comparator_config_t n_cc = {.flags.update_cmp_on_tez=true};
    mcpwm_new_comparator(n_oper, &n_cc, &n_cmpr);
    mcpwm_generator_config_t n_gc = {.gen_gpio_num=(int)drv_pin};
    mcpwm_new_generator(n_oper, &n_gc, &n_gen);
    mcpwm_generator_set_action_on_timer_event(n_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(n_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        n_cmpr, MCPWM_GEN_ACTION_LOW));
    mcpwm_comparator_set_compare_value(n_cmpr, 0);
    mcpwm_timer_enable(n_timer);
    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_START_NO_STOP);

    float    progress     = 0.0f;
    float    deg_per_pulse = 0.0f;
    uint32_t elapsed      = 0;
    uint32_t last_csv_ms  = 0;
    int      csv_step     = 0;
    bool     ok           = true;
    int      stall_count  = 0;
    uint32_t timeout_ms   = (uint32_t)(arc_deg / target_dps * 4000.0f);
    if (timeout_ms < 60000UL) timeout_ms = 60000UL;

    // Pressure PID state
    const float PSI_KP        = 0.8f;   // valve deg per PSI error (conservative)
    const float PSI_DEADBAND  = 0.08f;  // PSI -- skip tiny corrections
    const float PSI_MAX_CORR  = 2.0f;   // max valve correction per pulse (deg)
    const uint32_t VCORR_MS   = 600;    // max ms budget for valve correction
    // direct_valve: use walk valve_deg as-is; skip pressure settle and PID entirely.
    float target_psi = direct_valve ? 0.0f : cal_throw_to_psi(ring_throw);
    float valve_live = valve_deg;        // running PID-corrected setpoint
    int   valve_dir  = 1;               // last direction for backlash tracking

    if (!direct_valve) {
        // Wait for hydraulic pressure to stabilise after valve positioning.
        // Phase 1: wait until two consecutive reads agree within 0.05 PSI.
        // Phase 2: if settled pressure is >0.3 PSI from target, correct valve once.
        float psi_settled = 0.0f;
        float psi_prev = 0.0f;
        mprls_read_quiet(&psi_prev);
        for (int sw = 0; sw < 10 && ok; sw++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            elapsed += 100;
            if (uart_getchar(0) != 0 || s_water_abort) { ok = false; break; }
            TOUCH_ACTIVITY();
            float psi_now = 0.0f;
            mprls_read_quiet(&psi_now);
            if (psi_now > 0.05f && fabsf(psi_now - psi_prev) < 0.05f) {
                psi_settled = psi_now;
                INFO("  Pressure settled: %.3f PSI (target %.3f, err %.3f) after %d reads",
                     psi_now, target_psi, psi_now - target_psi, sw + 1);
                break;
            }
            psi_prev = psi_now;
        }
        if (ok && psi_settled > 0.05f && fabsf(psi_settled - target_psi) > 0.30f) {
            float ideal_deg = cal_pressure_to_valve_deg(target_psi);
            float new_deg   = valve_deg + 0.25f * (ideal_deg - valve_deg);
            if (new_deg < VALVE_CLOSED_DEG) new_deg = VALVE_CLOSED_DEG;
            if (new_deg > VALVE_OPEN_DEG)   new_deg = VALVE_OPEN_DEG;
            valve_deg  = new_deg;
            valve_live = new_deg;
            valve_goto_ex(new_deg, 0.3f, 4000, false, s_valve_last_dir);
            vTaskDelay(pdMS_TO_TICKS(400));
            elapsed += 400;
            float psi_check = 0.0f;
            mprls_read_quiet(&psi_check);
            INFO("  Pressure corrected: %.3f->%.3f PSI (target %.3f)",
                 psi_settled, psi_check, target_psi);
        }
    }

    while (progress < arc_deg - 0.3f && elapsed < timeout_ms) {
        if (uart_getchar(0) != 0 || s_water_abort) { ok = false; break; }
        TOUCH_ACTIVITY();

        uint16_t n_raw_b = 0;
        as5600_read(ADDR_AS5600, &n_raw_b, NULL, NULL);
        float pos_before = n_raw_b * (360.0f / 4096.0f);

        // Fire pulse
        mcpwm_comparator_set_compare_value(n_cmpr, pulse_duty);
        vTaskDelay(pdMS_TO_TICKS(pulse_ms));
        mcpwm_comparator_set_compare_value(n_cmpr, 0);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
        elapsed += pulse_ms + SETTLE_MS;

        uint16_t n_raw_a = 0;
        as5600_read(ADDR_AS5600, &n_raw_a, NULL, NULL);
        float pos_after = n_raw_a * (360.0f / 4096.0f);

        float delta = cw
            ? fmodf(pos_after - pos_before + 360.0f, 360.0f)
            : fmodf(pos_before - pos_after + 360.0f, 360.0f);

        if (delta < 0.15f) {
            if (++stall_count >= 3) {
                WARN("  Pulse mode: stall at progress=%.1f deg", progress);
                break;
            }
            continue;
        }
        stall_count = 0;
        deg_per_pulse = (deg_per_pulse < 0.1f)
            ? delta : (0.8f*deg_per_pulse + 0.2f*delta);
        progress += delta;

        // Dwell so total time per step = delta / target_dps
        float   step_ms_ideal = delta / target_dps * 1000.0f;
        int32_t dwell_ms      = (int32_t)step_ms_ideal
                              - (int32_t)(pulse_ms + SETTLE_MS);

        // Pressure P-control: nudge valve to maintain target_psi.
        // Valve uses GPIO (not MCPWM) so no conflict with nozzle timer.
        if (target_psi > 0.1f && dwell_ms > (int32_t)VCORR_MS) {
            float act_psi = 0.0f;
            if (mprls_read_quiet(&act_psi) && act_psi > 0.1f) {
                float err = target_psi - act_psi;
                if (fabsf(err) > PSI_DEADBAND) {
                    float corr = PSI_KP * err;
                    if (corr >  PSI_MAX_CORR) corr =  PSI_MAX_CORR;
                    if (corr < -PSI_MAX_CORR) corr = -PSI_MAX_CORR;
                    float nv = valve_live + corr;
                    if (nv < VALVE_CAL_START_DEG) nv = VALVE_CAL_START_DEG;
                    if (nv > VALVE_OPEN_DEG)      nv = VALVE_OPEN_DEG;
                    if (fabsf(nv - valve_live) > 0.2f) {
                        TickType_t tc = xTaskGetTickCount();
                        int ndir = (nv > valve_live) ? 1 : -1;
#if EXP_TRIM_NO_OVERSHOOT
                        // A sub-degree reversing nudge doesn't need the 15deg
                        // consistent-approach excursion -- the continuous trim
                        // re-corrects next pulse. Skipping it keeps the move inside
                        // the VCORR_MS budget and avoids a self-inflicted pressure
                        // transient. Larger corrections still get the full comp.
                        if (fabsf(corr) < EXP_TRIM_SMALL_CORR_DEG)
                            valve_goto_direct(nv, 0.4f, VCORR_MS, false);
                        else
                            valve_goto_ex(nv, 0.4f, VCORR_MS, false, valve_dir);
#else
                        valve_goto_ex(nv, 0.4f, VCORR_MS, false, valve_dir);
#endif
                        valve_live = nv;
                        valve_dir  = ndir;
                        uint32_t spent=(xTaskGetTickCount()-tc)*portTICK_PERIOD_MS;
                        dwell_ms -= (int32_t)spent;
                        elapsed  += spent;
                    }
                }
            }
        }

        if (dwell_ms > 20) {
            uint32_t rem = (uint32_t)dwell_ms;
            while (rem > 0 && ok) {
                uint32_t chunk = rem > 100 ? 100 : rem;
                vTaskDelay(pdMS_TO_TICKS(chunk));
                rem -= chunk; elapsed += chunk;
                if (uart_getchar(0) != 0 || s_water_abort) ok = false;
                TOUCH_ACTIVITY();
            }
        }

        // Log every 2 deg of sweep (5x more than 1-per-sector)
        // giving ~17 rows per ring for a 35 deg zone.
        uint32_t mps = (uint32_t)(2.0f / target_dps * 1000.0f);
        if (mps < 200) mps = 200;
        if (elapsed - last_csv_ms >= mps) {
            last_csv_ms = elapsed;
            float t_s = (float)(xTaskGetTickCount()-t_start)/(float)configTICK_RATE_HZ;
            float cur_deg = cw
                ? fmodf(sweep_origin+progress, 360.0f)
                : fmodf(sweep_origin-progress+360.0f, 360.0f);
            uint16_t v_raw=0;
            as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
            float csv_psi=0.0f; mprls_read_quiet(&csv_psi);
            if (psi_sum_out && csv_psi > 0.1f) { *psi_sum_out += csv_psi; (*psi_n_out)++; }
            water_trace_sample(csv_psi);  // b283: 1Hz-rate-limited, harmless no-op when inactive
            // Sector from actual bearing so column stays correct at higher sample rate
            int sec = (int)(cur_deg / WATER_SECTOR_DEG);
            {
                int   _sn  = (sec+WATER_SECTORS)%WATER_SECTORS;
                float _tr  = cur_deg*(float)M_PI/180.0f;
                float _ar  = cur_deg*(float)M_PI/180.0f;
                float _at2 = cal_pressure_to_throw_mm(csv_psi);
                float _tp  = cal_throw_to_psi(ring_throw);
                water_csv_write_row(s_water_csv_f, t_s, ring, arc_label, _sn,
                    (float)_sn * WATER_SECTOR_DEG, cur_deg,
                    valve_live, v_raw * (360.0f / 4096.0f),
                    ring_throw, _at2, _tp, csv_psi, _tr, s_csv_pass_type);
                tprintf("%.2f,%d,%d,%d,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%.3f,%.3f,%.0f,%.0f,%.0f,%.0f,1\r\n",
                    t_s, ring, arc_label, _sn,
                    (float)_sn*WATER_SECTOR_DEG, cur_deg,
                    valve_live, v_raw*(360.0f/4096.0f),
                    ring_throw, _at2, _tp, csv_psi,
                    ring_throw*sinf(_tr), ring_throw*cosf(_tr),
                    _at2*sinf(_ar), _at2*cosf(_ar));
            }
            csv_step++;
        }
    }

    float err = progress - arc_deg;
    INFO("  Arc end (pulse): %.1f deg swept (target %.1f, err %.1f)  "
         "~%.2f deg/pulse  valve %.1f->%.1f (PID %.2f deg)",
         progress, arc_deg, err, deg_per_pulse,
         valve_deg, valve_live, valve_live - valve_deg);

    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(n_timer);
    mcpwm_del_generator(n_gen);
    mcpwm_del_comparator(n_cmpr);
    mcpwm_del_operator(n_oper);
    mcpwm_del_timer(n_timer);
    gpio_set_level(drv_pin, 0);
    gpio_set_level(off_pin, 0);
    s_nozzle_last_dir = cw ? 1 : -1;
    return ok;
}


static bool nozzle_sweep_continuous(
        float sweep_origin, float arc_deg, bool cw, uint16_t run_duty,
        float nozzle_dps, TickType_t t_start,
        int ring, int arc_label, int first_sec,
        float valve_deg, float ring_throw)
{
    // Read actual nozzle position before the sweep.
    // If nozzle_goto left the nozzle short of sweep_origin, adjust
    // sweep_origin and arc_deg so the sweep still ends at the intended
    // endpoint. Without this a 22 deg miss causes ~295 deg progress
    // error and the arc fires immediately after the 500ms guard.
    {
        uint16_t act_raw = 0;
        if (as5600_read(ADDR_AS5600, &act_raw, NULL, NULL)) {
            float act_pos = act_raw * (360.0f / 4096.0f);
            // How far is the nozzle ahead or behind sweep_origin?
            // Positive = behind (needs catch-up), negative-ish = ahead.
            float progress = cw
                ? fmodf(act_pos - sweep_origin + 360.0f, 360.0f)
                : fmodf(sweep_origin - act_pos + 360.0f, 360.0f);
            float behind = cw
                ? fmodf(sweep_origin - act_pos + 360.0f, 360.0f)
                : fmodf(act_pos - sweep_origin + 360.0f, 360.0f);
            if (behind > 180.0f) behind = 0.0f;

            if (behind > 0.5f && behind < 90.0f) {
                // Nozzle is behind sweep_origin: extend arc and move origin back.
                INFO("  Sweep adj: nozzle %.1f deg behind -- arc %.1f->%.1f",
                     behind, arc_deg, arc_deg + behind);
                arc_deg     += behind;
                sweep_origin = act_pos;
            } else if (progress > 0.5f && progress < 180.0f) {
                // Nozzle is ahead of sweep_origin (already into the arc).
                // Clip arc_deg to the remaining travel.
                float remaining = arc_deg - progress;
                if (remaining <= 0.5f) {
                    // Nozzle already past arc endpoint.
                    // Move to arc endpoint so subsequent rings start from the right place.
                    // Without this, each skipped arc accumulates a positional offset
                    // that causes all later rings to trim their sweeps incorrectly.
                    float endpoint = cw
                        ? fmodf(sweep_origin + arc_deg, 360.0f)
                        : fmodf(sweep_origin - arc_deg + 360.0f, 360.0f);
                    INFO("  Sweep adj: nozzle %.1f deg past endpoint -- repositioning to %.1f",
                         progress, endpoint);
                    nozzle_goto_direct(endpoint, 1.5f, 5000, false);
                    return true;
                } else {
                    INFO("  Sweep adj: nozzle %.1f deg ahead -- arc %.1f->%.1f",
                         progress, arc_deg, remaining);
                    arc_deg = remaining;
                }
            }
        }
    }

    uint32_t arc_ms     = (uint32_t)(arc_deg / nozzle_dps * 1000.0f);
    uint32_t arc_ms_max = arc_ms * 2;

    gpio_num_t drv_pin = cw ? NOZZLE_PIN_FWD : NOZZLE_PIN_REV;
    gpio_num_t off_pin = cw ? NOZZLE_PIN_REV : NOZZLE_PIN_FWD;
    gpio_set_level(off_pin, 0);
    gpio_set_level(GPIO_K, 0);

    mcpwm_timer_handle_t n_timer = NULL;
    mcpwm_oper_handle_t  n_oper  = NULL;
    mcpwm_cmpr_handle_t  n_cmpr  = NULL;
    mcpwm_gen_handle_t   n_gen   = NULL;

    mcpwm_timer_config_t n_tc = {
        .group_id=0, .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz=10000000, .count_mode=MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks=500 };
    mcpwm_new_timer(&n_tc, &n_timer);
    mcpwm_operator_config_t n_oc = {.group_id=0};
    mcpwm_new_operator(&n_oc, &n_oper);
    mcpwm_operator_connect_timer(n_oper, n_timer);
    mcpwm_comparator_config_t n_cc = {.flags.update_cmp_on_tez=true};
    mcpwm_new_comparator(n_oper, &n_cc, &n_cmpr);
    mcpwm_generator_config_t n_gc = {.gen_gpio_num=(int)drv_pin};
    mcpwm_new_generator(n_oper, &n_gc, &n_gen);
    mcpwm_generator_set_action_on_timer_event(n_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(n_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        n_cmpr, MCPWM_GEN_ACTION_LOW));

    // Scale kickstart with run_duty. Full 480 for normal/fast sweeps;
    // gentler burst for slow mode so short arcs don't overshoot.
    uint16_t kick_duty = (run_duty < 200) ? (uint16_t)(run_duty + 100u) : 480u;
    mcpwm_comparator_set_compare_value(n_cmpr, kick_duty);
    mcpwm_timer_enable(n_timer);
    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_START_NO_STOP);
    vTaskDelay(pdMS_TO_TICKS(30));
    mcpwm_comparator_set_compare_value(n_cmpr, run_duty);

    // Kickstart moves the nozzle before sweep tracking begins. Re-read actual
    // position and shrink arc_deg so the intended endpoint is preserved.
    {
        uint16_t _pk_raw = 0;
        if (as5600_read(ADDR_AS5600, &_pk_raw, NULL, NULL)) {
            float _pk_pos     = _pk_raw * (360.0f / 4096.0f);
            float _kick_travel = cw
                ? fmodf(_pk_pos - sweep_origin + 360.0f, 360.0f)
                : fmodf(sweep_origin - _pk_pos + 360.0f, 360.0f);
            if (_kick_travel > 0.5f && _kick_travel < arc_deg - 1.0f) {
                arc_deg     -= _kick_travel;
                sweep_origin = _pk_pos;
            }
        }
    }

    uint32_t elapsed = 30, last_csv_ms = 0;
    int      csv_step = 0;
    bool     ok = true;

    while (elapsed < arc_ms_max) {
        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed += 50;
        TOUCH_ACTIVITY();
        if (uart_getchar(0) != 0 || s_water_abort) { ok = false; break; }

        uint16_t n_raw=0, v_raw=0;
        as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
        as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
        float cur_deg = n_raw * (360.0f / 4096.0f);
        float progress = cw ?
            fmodf(cur_deg - sweep_origin + 360.0f, 360.0f) :
            fmodf(sweep_origin - cur_deg + 360.0f, 360.0f);

        uint32_t arc_guard_ms = (uint32_t)(arc_deg / nozzle_dps * 500.0f);
        if (arc_guard_ms < 80) arc_guard_ms = 80;   // floor: at least 80ms
        if (elapsed > arc_guard_ms && progress >= arc_deg - 0.18f) {
            float err = progress - arc_deg;
            if (fabsf(err) < 3.0f)
                INFO("    Arc end: %.1f deg swept (target %.1f, err %.1f)", progress, arc_deg, err);
            else if (arc_deg >= 15.0f)
                INFO("    *** Arc overshoot %.1f deg -- rerun speed cal ***", err);
            else
                INFO("    Arc end (short arc): %.1f deg swept (target %.1f, err %.1f)",
                     progress, arc_deg, err);
            break;
        }
        if (elapsed >= arc_ms_max)
            INFO("    *** Arc timeout: %.1f of %.1f deg -- motor slow? ***", progress, arc_deg);

        uint32_t mps = (uint32_t)(WATER_SECTOR_DEG / nozzle_dps * 1000.0f);
        if (mps < 50) mps = 50;
        if (elapsed - last_csv_ms >= mps) {
            last_csv_ms = elapsed;
            float t_s = (float)(xTaskGetTickCount() - t_start) / (float)configTICK_RATE_HZ;
            float csv_psi2=0.0f; mprls_read_quiet(&csv_psi2);
            int sec = first_sec + (cw ? csv_step : -csv_step);
            tprintf("%.2f,%d,%d,%d,%.1f,%.2f,%.2f,%.2f,%.0f,%.3f,1\r\n",
                    t_s, ring, arc_label, (sec+WATER_SECTORS)%WATER_SECTORS,
                    (float)((sec+WATER_SECTORS)%WATER_SECTORS)*WATER_SECTOR_DEG,
                    cur_deg, v_raw*(360.0f/4096.0f), valve_deg, ring_throw, csv_psi2);
            csv_step++;
        }
    }

    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(n_timer);
    mcpwm_del_generator(n_gen);
    mcpwm_del_comparator(n_cmpr);
    mcpwm_del_operator(n_oper);
    mcpwm_del_timer(n_timer);
    gpio_set_level(drv_pin, 0);
    gpio_set_level(GPIO_K, 1); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GPIO_K, 0);

    return ok;
}

// Encoder-position-based continuous sweep for gentle mode.
// Motor runs at full run_duty; encoder is polled every 10ms and the motor
// is cut when progress reaches arc_deg - COAST_DEG (1.5 deg) to account
// for coasting. CSV rows are written to file and terminal every 2 deg so
// the heatmap gets real data (unlike nozzle_sweep_continuous which never
// writes to s_water_csv_f).
static bool nozzle_sweep_encoder_gentle(
        float sweep_origin, float arc_deg, bool cw, uint16_t run_duty,
        float nozzle_dps, TickType_t t_start,
        int ring, int arc_label, int first_sec,
        float valve_deg, float ring_throw,
        float *psi_sum_out, int *psi_n_out,
        uint32_t *elapsed_ms_out)
{
    // Adjust sweep_origin/arc_deg if nozzle is behind or ahead of start.
    {
        uint16_t act_raw = 0;
        if (as5600_read(ADDR_AS5600, &act_raw, NULL, NULL)) {
            float act_pos = act_raw * (360.0f / 4096.0f);
            float progress = cw
                ? fmodf(act_pos - sweep_origin + 360.0f, 360.0f)
                : fmodf(sweep_origin - act_pos + 360.0f, 360.0f);
            float behind = cw
                ? fmodf(sweep_origin - act_pos + 360.0f, 360.0f)
                : fmodf(act_pos - sweep_origin + 360.0f, 360.0f);
            if (behind > 180.0f) behind = 0.0f;

            if (behind > 0.5f && behind < 90.0f) {
                // b310: don't extend arc backwards -- that bypasses the
                // polygon clip applied to sweep_origin by the caller. Reposition
                // the nozzle FORWARD to sweep_origin and start the sweep there.
                // Symptom this fixes: zone with several skipped rings (e.g. N3
                // ring 1, 30, 31 clipped to zero width by polygon) leaves the
                // nozzle wherever the last unclipped ring ended (often 30+ deg
                // behind the next ring's clipped seg_origin). The old code
                // extended arc_deg by that gap, sweeping with valve open through
                // bearings the polygon clip just excluded.
                INFO("  Sweep adj: nozzle %.1f deg behind -- repositioning to %.1f",
                     behind, sweep_origin);
                nozzle_goto_direct(sweep_origin, 0.5f, 5000, false);
            } else if (progress > 0.5f && progress < 180.0f) {
                float remaining = arc_deg - progress;
                if (remaining <= 0.5f) {
                    float endpoint = cw
                        ? fmodf(sweep_origin + arc_deg, 360.0f)
                        : fmodf(sweep_origin - arc_deg + 360.0f, 360.0f);
                    INFO("  Sweep adj: nozzle %.1f deg past endpoint -- repositioning to %.1f",
                         progress, endpoint);
                    nozzle_goto_direct(endpoint, 1.5f, 5000, false);
                    return true;
                } else {
                    INFO("  Sweep adj: nozzle %.1f deg ahead -- arc %.1f->%.1f",
                         progress, arc_deg, remaining);
                    arc_deg = remaining;
                }
            }
        }
    }

    const float COAST_DEG = 1.5f;
    uint32_t arc_ms_max = (uint32_t)(arc_deg / nozzle_dps * 1000.0f) * 3 + 500;

    gpio_num_t drv_pin = cw ? NOZZLE_PIN_FWD : NOZZLE_PIN_REV;
    gpio_num_t off_pin = cw ? NOZZLE_PIN_REV : NOZZLE_PIN_FWD;
    gpio_set_level(off_pin, 0);
    gpio_set_level(GPIO_K, 0);

    mcpwm_timer_handle_t n_timer = NULL;
    mcpwm_oper_handle_t  n_oper  = NULL;
    mcpwm_cmpr_handle_t  n_cmpr  = NULL;
    mcpwm_gen_handle_t   n_gen   = NULL;

    mcpwm_timer_config_t n_tc = {
        .group_id=0, .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz=10000000, .count_mode=MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks=500 };
    mcpwm_new_timer(&n_tc, &n_timer);
    mcpwm_operator_config_t n_oc = {.group_id=0};
    mcpwm_new_operator(&n_oc, &n_oper);
    mcpwm_operator_connect_timer(n_oper, n_timer);
    mcpwm_comparator_config_t n_cc = {.flags.update_cmp_on_tez=true};
    mcpwm_new_comparator(n_oper, &n_cc, &n_cmpr);
    mcpwm_generator_config_t n_gc = {.gen_gpio_num=(int)drv_pin};
    mcpwm_new_generator(n_oper, &n_gc, &n_gen);
    mcpwm_generator_set_action_on_timer_event(n_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(n_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
        n_cmpr, MCPWM_GEN_ACTION_LOW));

    // Smooth-overshoot kickstart. The legacy fixed kick (run_duty+100 for
    // 30 ms then instantaneous drop to run_duty) reliably broke static
    // friction but produced ~5-6 deg of velocity overshoot that read as a
    // jerk at every ring entry. A plain ramp up to run_duty was visually
    // smoother but failed to break stiction at the low run_duty values
    // (70-80) used in smooth-mode watering.
    //
    // This profile keeps the same peak torque as the legacy kick but
    // smooths both the rising and falling edges: 4 steps up to PEAK
    // (= run_duty + boost), then 3 steps down to run_duty. Total ~48 ms,
    // no instantaneous PWM step in either direction.
    const uint16_t RUN_DUTY  = run_duty;
    const uint16_t BOOST     = (run_duty < 200u) ? 100u : (uint16_t)(480u - run_duty);
    const uint16_t PEAK_DUTY = (uint16_t)(RUN_DUTY + BOOST);
    const uint16_t START_DUTY = (run_duty > 80u) ? (uint16_t)(run_duty / 2u) : 40u;
    const int      UP_STEPS    = 4;
    const uint32_t UP_STEP_MS  = 6;
    const int      DOWN_STEPS   = 3;
    const uint32_t DOWN_STEP_MS = 8;

    mcpwm_comparator_set_compare_value(n_cmpr, START_DUTY);
    mcpwm_timer_enable(n_timer);
    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_START_NO_STOP);
    for (int _s = 1; _s <= UP_STEPS; _s++) {
        uint16_t _d = (uint16_t)(START_DUTY + (PEAK_DUTY - START_DUTY) * _s / UP_STEPS);
        mcpwm_comparator_set_compare_value(n_cmpr, _d);
        vTaskDelay(pdMS_TO_TICKS(UP_STEP_MS));
    }
    for (int _s = 1; _s <= DOWN_STEPS; _s++) {
        uint16_t _d = (uint16_t)(PEAK_DUTY - (PEAK_DUTY - RUN_DUTY) * _s / DOWN_STEPS);
        mcpwm_comparator_set_compare_value(n_cmpr, _d);
        vTaskDelay(pdMS_TO_TICKS(DOWN_STEP_MS));
    }

    // Shrink arc_deg by kickstart travel so the endpoint is preserved.
    {
        uint16_t _pk_raw = 0;
        if (as5600_read(ADDR_AS5600, &_pk_raw, NULL, NULL)) {
            float _pk_pos      = _pk_raw * (360.0f / 4096.0f);
            float _kick_travel = cw
                ? fmodf(_pk_pos - sweep_origin + 360.0f, 360.0f)
                : fmodf(sweep_origin - _pk_pos + 360.0f, 360.0f);
            if (_kick_travel > 0.5f && _kick_travel < arc_deg - 1.0f) {
                arc_deg     -= _kick_travel;
                sweep_origin = _pk_pos;
            }
        }
    }

    // Kickstart consumed UP_STEPS*UP_STEP_MS + DOWN_STEPS*DOWN_STEP_MS
    // = 4*6 + 3*8 = 48 ms. arc_ms_max has a 3x safety multiplier so
    // small accounting drift doesn't matter.
    uint32_t elapsed = (uint32_t)(UP_STEPS * UP_STEP_MS + DOWN_STEPS * DOWN_STEP_MS);
    float    last_csv_deg = 0.0f;
    bool     ok = true;
    // Wall-clock reference for actual sweep duration.
    // elapsed += 10 is NOT reliable because CSV writes (fprintf + tprintf every
    // 2 deg) make each loop iteration ~50ms actual vs the 10ms it counts.
    // xTaskGetTickCount() measures real time; used for elapsed_ms_out only.
    TickType_t t_sweep_start = xTaskGetTickCount();
    // Stall-detection state. Tracks the last encoder reading that showed
    // measurable motion (> ~1 encoder count). If no motion is seen for
    // STALL_TIMEOUT_MS while the sweep is well clear of the coast region,
    // the nozzle motor is mechanically stuck.
    TickType_t last_motion_tick = t_sweep_start;
    float      prev_progress    = 0.0f;
    // Generous thresholds — only a clearly-stuck motor should fault.
    // At the slowest expected sweep rate (~3 deg/s) we still see
    // ~2.4 deg of travel in 800 ms; legitimate motion always clears
    // the 0.1 deg motion threshold well before STALL_TIMEOUT_MS hits.
    const uint32_t STALL_TIMEOUT_MS  = 800u;   // no-motion window before checking current
    const uint32_t STALL_GRACE_MS    = 250u;   // settle after kickstart
    // Current-based fault confirmation. Two failure modes:
    //   1. Mechanical stall: drive on, motor blocked. Current spikes
    //      (typically 400+ mA). High-current AND no-motion = jam.
    //   2. Open circuit: winding broken / wire failure. PWM driving but
    //      no current path. Near-zero current AND no-motion = open.
    //      Only checked when run_duty is high enough that we'd
    //      expect non-trivial current draw — at very low duty (<120
    //      i.e. 24% PWM), normal motor draw can be under 50 mA and
    //      a low reading doesn't indicate an open.
    const float    STALL_CURRENT_MA  = 400.0f;  // upper gate: jam
    const float    OPEN_CURRENT_MA   = 15.0f;   // lower gate: winding open
    const uint16_t OPEN_CHECK_MIN_DUTY = 120;   // skip open check below this
    // Independent dwell watchdog — backstop in case the stall+current
    // checks miss a failure mode (frozen encoder, subtle stalls, etc.).
    // Caps the time the nozzle can sit near one bearing with the valve
    // open. Timeout is the "definitely failed" ceiling, not a plant-
    // protection precision timer — the stall+current detection (~800 ms)
    // handles fast common-case shutdown. Configurable from HA, persisted
    // in NVS as "pm_dwell_s"; defaults to 30 s for standalone.
    TickType_t last_dwell_move_tick = t_sweep_start;
    float      last_dwell_pos       = sweep_origin;  // post-kickstart baseline
    const uint32_t DWELL_TIMEOUT_MS  = s_dwell_timeout_ms;
    const float    DWELL_MOVE_DEG    = 3.0f;    // min motion to reset
    // Adaptive recovery: when stall detection fires with current in the
    // normal-running range (motor not jammed, not open, just stuck at
    // very low duty), briefly boost PWM to unstick before resetting the
    // timer. Caps the number of recoveries per sweep so a permanently-
    // stuck motor still hits the dwell watchdog eventually.
    int            recovery_kicks_used = 0;
    const int      MAX_RECOVERY_KICKS  = 6;

    // b360: track whether nozzle has clearly left sweep_origin. The old code
    // gated arc-end and stall-detection on `progress < 180`, intending to
    // reject encoder quantization noise that reads slightly-behind-origin as
    // progress=359°. That works for arcs ≤ 180° but kills both checks for
    // arcs > 180° (sprinkler-inside-polygon zones with zone_arc_deg=360 set
    // every inner ring's active arc to ~220°). Symptom on TopCorner: arc-end
    // never fired, motor swept until arc_ms_max timeout, overshooting up to
    // a full revolution past intended end and spraying through the polygon
    // "gap" sectors. Latch: once progress has been clearly above the start
    // (> 2 × COAST_DEG), trust later progress readings — including > 180°.
    bool progress_cleared_start = false;
    const float PROGRESS_CLEAR_DEG = 2.0f * COAST_DEG;  // 3.0 deg
    // b360: per-sweep dps refit. The speed cal can be wrong under load
    // (TopCorner R12 ran at 0.58 dps vs cal-predicted 5.4 dps at duty 70),
    // letting the motor crawl while the valve sprays. After a short window
    // of real motion, if actual dps is well below target, bump PWM duty up
    // so the sweep finishes on time. Single shot per sweep to avoid runaway.
    bool dps_refit_done = false;
    const uint32_t DPS_REFIT_AFTER_MS = 4000u;   // give kickstart + stiction time
    const float    DPS_REFIT_MIN_PROG  = 6.0f;    // need real motion to measure
    const float    DPS_REFIT_RATIO     = 0.5f;    // bump if < 50% of target
    const float    DPS_REFIT_DUTY_MULT = 1.5f;    // 50% PWM bump
    const uint16_t DPS_REFIT_DUTY_CAP  = 480u;    // mcpwm period_ticks=500, headroom

    while (elapsed < arc_ms_max) {
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
        TOUCH_ACTIVITY();
        if (uart_getchar(0) != 0 || s_water_abort) { ok = false; break; }

        uint16_t n_raw = 0;
        as5600_read(ADDR_AS5600, &n_raw, NULL, NULL);
        float cur_deg  = n_raw * (360.0f / 4096.0f);
        float progress = cw
            ? fmodf(cur_deg - sweep_origin + 360.0f, 360.0f)
            : fmodf(sweep_origin - cur_deg + 360.0f, 360.0f);

        // b360: latch once we've clearly moved off start so quantization-noise
        // values (progress=358..360 when nozzle is slightly behind sweep_origin)
        // don't satisfy the stop check at the very start of a sweep.
        if (progress > PROGRESS_CLEAR_DEG && progress < 180.0f)
            progress_cleared_start = true;

        float stop_at = arc_deg - COAST_DEG;
        if (stop_at < 0.0f) stop_at = 0.0f;
        // Fire end-of-arc only after the nozzle has clearly cleared the
        // start region. This rejects encoder quantization noise at sweep
        // start (intent of the old `progress < 180` guard) while still
        // letting legitimate arcs > 180° terminate at the right bearing.
        if (progress_cleared_start && progress >= stop_at) {
            float err = progress - arc_deg;
            INFO("  Arc end (gentle): %.1f deg swept (target %.1f, coast-stop err %.1f)",
                 progress, arc_deg, err);
            break;
        }
        if (elapsed >= arc_ms_max)
            INFO("  *** Gentle arc timeout: %.1f of %.1f deg ***", progress, arc_deg);

        // b360: mid-sweep dps refit. If the motor is moving much slower than
        // the speed cal predicted for the chosen duty, bump duty up. This
        // self-heals when the cal was made at different load (e.g. inner
        // rings see different valve drag than the cal pressure).
        if (!dps_refit_done && progress_cleared_start
                && progress > DPS_REFIT_MIN_PROG) {
            uint32_t ms_so_far = (uint32_t)((xTaskGetTickCount() - t_sweep_start)
                                            * portTICK_PERIOD_MS);
            if (ms_so_far > DPS_REFIT_AFTER_MS) {
                float actual_dps = progress * 1000.0f / (float)ms_so_far;
                if (nozzle_dps > 0.1f
                        && actual_dps < nozzle_dps * DPS_REFIT_RATIO) {
                    uint32_t new_duty32 = (uint32_t)((float)run_duty
                                                     * DPS_REFIT_DUTY_MULT);
                    if (new_duty32 > DPS_REFIT_DUTY_CAP)
                        new_duty32 = DPS_REFIT_DUTY_CAP;
                    if (new_duty32 > run_duty) {
                        ESP_LOGI(TAG, "  Mid-sweep dps refit: actual %.2f dps "
                                      "vs target %.2f -- duty %u -> %u",
                                 actual_dps, nozzle_dps,
                                 (unsigned)run_duty, (unsigned)new_duty32);
                        run_duty = (uint16_t)new_duty32;
                        mcpwm_comparator_set_compare_value(n_cmpr, run_duty);
                    }
                }
                dps_refit_done = true;
            }
        }

        // Stall detection: only check away from arc end (motor may legitimately
        // coast slowly) and after the kickstart grace period. Encoder LSB is
        // ~0.088 deg, so a 0.1 deg threshold reliably distinguishes "actually
        // moving" from "quantization noise on a stalled motor".
        uint32_t since_start = (uint32_t)((xTaskGetTickCount() - t_sweep_start)
                                          * portTICK_PERIOD_MS);
        // Dwell watchdog: applies even during grace period and near
        // arc end. The valve is open the entire time this sweep is
        // running, so the worst-case water-in-one-spot exposure is
        // DWELL_TIMEOUT_MS regardless of where the failure is.
        {
            float dwell_delta = fabsf(cur_deg - last_dwell_pos);
            if (dwell_delta > 180.0f) dwell_delta = 360.0f - dwell_delta;
            if (dwell_delta > DWELL_MOVE_DEG) {
                last_dwell_move_tick = xTaskGetTickCount();
                last_dwell_pos       = cur_deg;
            } else {
                uint32_t dwell_ms = (uint32_t)((xTaskGetTickCount() - last_dwell_move_tick)
                                               * portTICK_PERIOD_MS);
                if (dwell_ms > DWELL_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "Nozzle dwell watchdog: %u ms at ~%.1f deg "
                                  "with valve open — slamming valve closed",
                             (unsigned)dwell_ms, cur_deg);
                    water_set_status(WATER_STATUS_NOZZLE_FAULT);
                    // Stop nozzle drive immediately and close the valve from
                    // within this function so water flow ends ASAP, before
                    // the caller's abort-label unwind reaches valve_goto.
                    mcpwm_comparator_set_compare_value(n_cmpr, 0);
                    valve_goto(VALVE_CLOSED_DEG, 2.0f, 5000, false);
                    ok = false;
                    break;
                }
            }
        }

        // End-of-arc skip widened to 10 deg so the natural coast-down
        // deceleration (which can briefly look like a stall in encoder
        // counts) doesn't trip the timer. b360: replaced raw `progress < 180`
        // guard with the cleared-start latch so stall detection (and its
        // recovery kicks) still fires for arcs > 180°.
        if (since_start > STALL_GRACE_MS && progress_cleared_start
                && progress < stop_at - 10.0f) {
            if (fabsf(progress - prev_progress) > 0.1f) {
                last_motion_tick = xTaskGetTickCount();
                prev_progress    = progress;
            } else {
                uint32_t stall_ms = (uint32_t)((xTaskGetTickCount() - last_motion_tick)
                                               * portTICK_PERIOD_MS);
                if (stall_ms > STALL_TIMEOUT_MS) {
                    // Sample NCUR (4 reads * 5 ms apart, averaged) and
                    // decide based on both extremes:
                    //   high current + no motion = mechanical jam
                    //   near-zero current + no motion = winding open / wire fail
                    //   normal current + no motion  = slow but moving; reset
                    // DC motors typically fail open, so the low-current gate
                    // catches the most common failure mode that the high-
                    // current check on its own would miss.
                    uint32_t sum_mv = 0;
                    for (int _s = 0; _s < 4; _s++) {
                        sum_mv += adc_mv(ADC_CH_NCUR);
                        vTaskDelay(pdMS_TO_TICKS(5));
                    }
                    float ma = CURRENT_MA(sum_mv / 4u);
                    if (ma >= STALL_CURRENT_MA) {
                        ESP_LOGW(TAG, "Nozzle fault (jam): stalled at %.1f deg "
                                      "(target %.1f), no motion %u ms, "
                                      "current %.0f mA",
                                 progress, arc_deg, (unsigned)stall_ms, ma);
                        water_set_status(WATER_STATUS_NOZZLE_FAULT);
                        ok = false;
                        break;
                    } else if (ma <= OPEN_CURRENT_MA
                                && run_duty >= OPEN_CHECK_MIN_DUTY) {
                        ESP_LOGW(TAG, "Nozzle fault (open): no motion %u ms at "
                                      "%.1f deg, current only %.0f mA at duty %u "
                                      "(driving but no current path)",
                                 (unsigned)stall_ms, progress, ma, (unsigned)run_duty);
                        water_set_status(WATER_STATUS_NOZZLE_FAULT);
                        ok = false;
                        break;
                    } else {
                        // Motor is drawing current but not moving — typically
                        // the run_duty is too low for the current load.
                        // Try a brief boost to push past stiction.
                        if (recovery_kicks_used < MAX_RECOVERY_KICKS) {
                            uint16_t kick_d = (run_duty < 200u)
                                              ? (uint16_t)(run_duty + 100u)
                                              : (uint16_t)PEAK_DUTY;
                            ESP_LOGI(TAG, "Nozzle slow at %.1f deg (%u ms, %.0f mA) "
                                          "— recovery kick %d/%d at duty %u",
                                     progress, (unsigned)stall_ms, ma,
                                     recovery_kicks_used + 1, MAX_RECOVERY_KICKS,
                                     (unsigned)kick_d);
                            mcpwm_comparator_set_compare_value(n_cmpr, kick_d);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            mcpwm_comparator_set_compare_value(n_cmpr, run_duty);
                            recovery_kicks_used++;
                        } else {
                            ESP_LOGW(TAG, "Nozzle slow at %.1f deg, exhausted "
                                          "%d recovery kicks — leaving to dwell watchdog",
                                     progress, MAX_RECOVERY_KICKS);
                        }
                        last_motion_tick = xTaskGetTickCount();
                        prev_progress    = progress;
                    }
                }
            }
        } else {
            // In grace period or near arc end — keep last_motion_tick fresh
            // so a legitimate coast-down doesn't trip the timer.
            last_motion_tick = xTaskGetTickCount();
            prev_progress    = progress;
        }

        // CSV row every 2 deg of encoder-measured progress.
        if (progress >= last_csv_deg + 2.0f) {
            last_csv_deg = progress;
            float t_s = (float)(xTaskGetTickCount() - t_start) / (float)configTICK_RATE_HZ;
            uint16_t v_raw = 0;
            as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
            float csv_psi = 0.0f; mprls_read_quiet(&csv_psi);
            if (psi_sum_out && csv_psi > 0.1f) { *psi_sum_out += csv_psi; (*psi_n_out)++; }
            water_trace_sample(csv_psi);  // b283: 1Hz-rate-limited, harmless no-op when inactive
            float valve_live = v_raw * (360.0f / 4096.0f);
            float _at2 = cal_pressure_to_throw_mm(csv_psi);
            float _tp  = cal_throw_to_psi(ring_throw);
            float _tr  = cur_deg * (float)M_PI / 180.0f;
            int   _sn  = ((int)(cur_deg / WATER_SECTOR_DEG) + WATER_SECTORS) % WATER_SECTORS;
                water_csv_write_row(s_water_csv_f, t_s, ring, arc_label, _sn,
                    (float)_sn * WATER_SECTOR_DEG, cur_deg,
                    valve_deg, valve_live,
                    ring_throw, _at2, _tp, csv_psi, _tr, s_csv_pass_type);
            tprintf("%.2f,%d,%d,%d,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%.3f,%.3f,%.0f,%.0f,%.0f,%.0f,1\r\n",
                t_s, ring, arc_label, _sn,
                (float)_sn * WATER_SECTOR_DEG, cur_deg,
                valve_deg, valve_live,
                ring_throw, _at2, _tp, csv_psi,
                ring_throw * sinf(_tr), ring_throw * cosf(_tr),
                _at2 * sinf(_tr), _at2 * cosf(_tr));
        }
    }

    mcpwm_timer_start_stop(n_timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(n_timer);
    mcpwm_del_generator(n_gen);
    mcpwm_del_comparator(n_cmpr);
    mcpwm_del_operator(n_oper);
    mcpwm_del_timer(n_timer);
    gpio_set_level(drv_pin, 0);
    gpio_set_level(GPIO_K, 1); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GPIO_K, 0);

    if (elapsed_ms_out) {
        TickType_t t_sweep_end = xTaskGetTickCount();
        *elapsed_ms_out = (uint32_t)((t_sweep_end - t_sweep_start) * portTICK_PERIOD_MS);
    }
    return ok;
}

// Sort zone perimeter points into walk order (by walk_idx) in-place.
// The NVS stores points sorted by bearing for lookup; polygon-dependent
// operations (ray intersection, edge-perpendicular, point-in-polygon)
// must use the user's actual walk order to get the correct polygon shape.
// LittleFS-first zone load. Falls back to NVS only for zone 0 (legacy migration).
static esp_err_t zone_load_primary(uint16_t id, zone_perimeter_t *z)
{
    if (storage_ready()) {
        char _name[32]={0};
        if (storage_zone_load(id, _name, sizeof(_name), z) == ESP_OK)
            return ESP_OK;
    }
    if (id == 0) return zone_load_nvs(z);  // NVS fallback for zone 0 only
    return ESP_ERR_NOT_FOUND;
}

// Save zone to LittleFS (primary) AND NVS (fallback).
// name_override: use this name if non-NULL/non-empty; otherwise preserve existing LittleFS name.
static esp_err_t zone_save_primary(uint16_t id, const char *name_override, const zone_perimeter_t *z)
{
    if (id == 0) zone_save_nvs(z);  // NVS fallback maintained for zone 0 only
    if (!storage_ready()) {
        ESP_LOGE(TAG, "zone_save_primary: storage not ready (zone id=%u not persisted to FS)", id);
        return ESP_ERR_INVALID_STATE;
    }
    char zname[32] = {0};
    if (name_override && name_override[0]) {
        strncpy(zname, name_override, sizeof(zname)-1);
    } else {
        char lfs_name[32]={0};
        zone_perimeter_t _tmp = {0};
        storage_zone_load(id, lfs_name, sizeof(lfs_name), &_tmp);  // name probe
        zone_name_resolve(id, lfs_name, "Zone", zname, sizeof(zname));
    }
    return storage_zone_save(id, zname, z);
}

static void zone_sort_walk_order(zone_perimeter_t *z)
{
    if (!z || z->num_points < 2) return;
    int n = z->num_points;
    // Insertion sort by walk_idx
    for (int i = 1; i < n; i++) {
        perimeter_point_t key = z->points[i];
        int j = i - 1;
        while (j >= 0 && z->points[j].walk_idx > key.walk_idx) {
            z->points[j+1] = z->points[j];
            j--;
        }
        z->points[j+1] = key;
    }
}

// Compute minimum throw needed to cover the complete zone polygon.
// Uses the minimum perpendicular distance from the origin (sprinkler) to
// any polygon EDGE -- not just the nearest vertex. This ensures the innermost
// rings reach the true inner boundary of the zone, including the gap that
// would otherwise exist between the nearest vertex and the closest edge midpoint.
static float zone_get_min_throw_mm(const zone_perimeter_t *z) {
    if (!z || z->num_points < 2) return cal_get_min_throw_mm();
    float min_dist = 99999.0f;
    int n = z->num_points;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float b1 = z->points[i].nozzle_deg * ((float)M_PI / 180.0f);
        float b2 = z->points[j].nozzle_deg * ((float)M_PI / 180.0f);
        float r1 = z->points[i].throw_mm;
        float r2 = z->points[j].throw_mm;
        if (r1 < 50.0f || r2 < 50.0f) continue;
        // Cartesian coords of the two vertices
        float x1 = r1*sinf(b1), y1 = r1*cosf(b1);
        float x2 = r2*sinf(b2), y2 = r2*cosf(b2);
        // Project origin onto the line segment, clamp to [0,1]
        float dx = x2-x1, dy = y2-y1;
        float len2 = dx*dx + dy*dy;
        float dist;
        if (len2 < 1.0f) {
            dist = sqrtf(x1*x1 + y1*y1);
        } else {
            float t = -(x1*dx + y1*dy) / len2;
            if (t <= 0.0f) {
                dist = sqrtf(x1*x1 + y1*y1);
            } else if (t >= 1.0f) {
                dist = sqrtf(x2*x2 + y2*y2);
            } else {
                float px = x1 + t*dx, py = y1 + t*dy;
                dist = sqrtf(px*px + py*py);
            }
        }
        if (dist < min_dist) min_dist = dist;
    }
    // Apply a small inward margin so the innermost arc actually lands inside
    // the polygon rather than exactly on the boundary edge.
    float result = (min_dist < 99000.0f) ? (min_dist * 0.85f) : 0.0f;
    float cal_min = cal_get_min_throw_mm();
    return fmaxf(result, cal_min);
}


// Ray-cast from origin to determine if the sprinkler (at origin) is inside
// the zone polygon. Returns true if origin is inside.
static bool zone_origin_inside(const zone_perimeter_t *z)
{
    if (!z || z->num_points < 3) return false;
    int n = z->num_points;
    int crossings = 0;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float b1 = z->points[i].nozzle_deg * ((float)M_PI / 180.0f);
        float b2 = z->points[j].nozzle_deg * ((float)M_PI / 180.0f);
        float x1 = z->points[i].throw_mm * sinf(b1);
        float y1 = z->points[i].throw_mm * cosf(b1);
        float x2 = z->points[j].throw_mm * sinf(b2);
        float y2 = z->points[j].throw_mm * cosf(b2);
        // Ray from origin in +x direction: count edges that straddle y=0 with x_int > 0
        if ((y1 > 0.0f) == (y2 > 0.0f)) continue;
        float x_int = x1 + (0.0f - y1) * (x2 - x1) / (y2 - y1);
        if (x_int > 0.0f) crossings++;
    }
    return (crossings % 2) == 1;
}

// True if (bearing_deg, r_mm) lies inside the zone polygon. Standard
// ray-cast PIP: cast a horizontal ray east in Cartesian space from
// the test point and count edge crossings; odd = inside.
//
// Used by zone_clip_arc_to_polygon to tighten sweep arcs so the
// nozzle doesn't aim at bearings where the polygon's radial extent
// at the ring's throw distance is outside the zone (e.g. zones with
// same-bearing inner+outer vertex pairs forming a narrow radial
// strip -- a sector grid centered at that bearing has a valid
// "outer" intersection but no polygon between origin and that
// outer for OTHER ring throws).
static bool zone_contains_point(const zone_perimeter_t *z,
                                 float bearing_deg, float r_mm)
{
    if (!z || z->num_points < 3) return false;
    float bsin = sinf(bearing_deg * ((float)M_PI / 180.0f));
    float bcos = cosf(bearing_deg * ((float)M_PI / 180.0f));
    float px = r_mm * bsin;
    float py = r_mm * bcos;
    int n = z->num_points;
    int crossings = 0;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float b1 = z->points[i].nozzle_deg * ((float)M_PI / 180.0f);
        float b2 = z->points[j].nozzle_deg * ((float)M_PI / 180.0f);
        float x1 = z->points[i].throw_mm * sinf(b1);
        float y1 = z->points[i].throw_mm * cosf(b1);
        float x2 = z->points[j].throw_mm * sinf(b2);
        float y2 = z->points[j].throw_mm * cosf(b2);
        if ((y1 > py) == (y2 > py)) continue;
        float x_int = x1 + (py - y1) * (x2 - x1) / (y2 - y1);
        if (x_int > px) crossings++;
    }
    return (crossings % 2) == 1;
}

// Tighten an arc [lo, hi] to the LONGEST contiguous run of bearings
// whose (bearing, ring_throw) point lies inside the polygon. Scans
// [lo, hi] at step_deg increments. If no bearing satisfies PIP,
// returns *lo == *hi (zero-width) so the caller can skip the segment.
//
// Returning a fully-inside contiguous sub-arc -- rather than only
// tightening the two endpoints -- is what guarantees the swept span
// never crosses outside the polygon. The previous endpoint-walk
// assumed the span between two inside endpoints was itself inside
// ("simply-connected"). That fails when the polygon touches ring_throw
// only at isolated vertices: e.g. a flat band of equal-throw perimeter
// points produced by a perimeter walked above the throw-calibration
// range. There the straight edges between the equal-throw vertices sag
// just inside ring_throw, PIP is true only in sub-degree vertex slivers,
// and the endpoint-walk leapfrogs between two different slivers and
// bridges them the long way around -- commanding a near-360deg
// wrap-around sweep almost entirely outside the polygon.
//
// Also fixes the original N3-style spillover (b309): polar-interp
// refinement / snap-to-zone-arc-end producing lo/hi at bearings whose
// radial extent at ring_throw is outside the polygon.
static void zone_clip_arc_to_polygon(const zone_perimeter_t *z,
                                      float ring_throw,
                                      float *lo, float *hi,
                                      float step_deg)
{
    if (!z || !lo || !hi || z->num_points < 3) return;
    float span = fmodf(*hi - *lo + 360.0f, 360.0f);
    if (span < 0.01f) return;
    if (step_deg <= 0.0f) step_deg = 0.5f;
    int n_steps = (int)(span / step_deg);

    float best_lo = 0.0f, best_hi = 0.0f, best_len = -1.0f;
    bool  in_run  = false;
    float run_lo  = 0.0f, run_prev = 0.0f;
    for (int i = 0; i <= n_steps; i++) {
        float b = fmodf(*lo + (float)i * step_deg + 360.0f, 360.0f);
        bool  inside = zone_contains_point(z, b, ring_throw);
        if (inside) {
            if (!in_run) { in_run = true; run_lo = b; }
            run_prev = b;
        }
        if ((!inside || i == n_steps) && in_run) {
            float run_len = fmodf(run_prev - run_lo + 360.0f, 360.0f);
            if (run_len > best_len) {
                best_len = run_len; best_lo = run_lo; best_hi = run_prev;
            }
            in_run = false;
        }
    }
    if (best_len < 0.0f) {
        // No bearing in the arc contains ring_throw. Collapse.
        *hi = *lo;
        return;
    }
    *lo = best_lo;
    *hi = best_hi;
}

// b312: Chase mode -- entertainment watering, continuous PWM control.
//
// Goal: a smoothly-moving water stream confined to the zone polygon for
// a settable duration (1..10 min). Designed for dogs/kids: the stream
// should glide, not jerk -- both nozzle bearing and valve throw advance
// continuously with constant motor PWM, only reversing direction when
// a look-ahead position would leave the polygon.
//
// Approach: two motors, both running concurrently with persistent PWM.
//   - Nozzle motor: NFWD / NREV pins, one MCPWM operator with two
//                   comparator/generator pairs (one per direction).
//                   Only one comparator nonzero at a time -- no
//                   shoot-through risk.
//   - Valve  motor: same pattern on VFWD / VREV.
// Every 50 ms tick: read both encoders, predict ~200 ms ahead in the
// current direction, and reverse the axis whose predicted point would
// fall outside the zone arc OR fail PIP at the other axis's actual
// current position.
//
// No closed-loop position control inside the loop. Velocity stays
// constant (constant duty) which is what makes the motion visually
// smooth -- the previous b311 implementation called nozzle_goto_direct
// every tick which spun the motor up and down each call, producing the
// stepped feel the user reported.
//
// Reversal is "ramp current direction to 0, brief settle, ramp new
// direction up" -- ~150 ms transition, again to keep motion smooth.

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmpr_fwd;
    mcpwm_cmpr_handle_t  cmpr_rev;
    mcpwm_gen_handle_t   gen_fwd;
    mcpwm_gen_handle_t   gen_rev;
    int                  dir;       // +1 fwd, -1 rev, 0 stopped
    uint16_t             duty;      // current PWM compare value on active side
    gpio_num_t           fwd_pin;
    gpio_num_t           rev_pin;
} chase_motor_t;

static bool chase_motor_setup(chase_motor_t *m,
                              gpio_num_t fwd_pin, gpio_num_t rev_pin)
{
    memset(m, 0, sizeof(*m));
    m->fwd_pin = fwd_pin;
    m->rev_pin = rev_pin;

    mcpwm_timer_config_t tc = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .period_ticks  = 500,       // 20 kHz
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    if (mcpwm_new_timer(&tc, &m->timer) != ESP_OK) return false;

    mcpwm_operator_config_t oc = { .group_id = 0 };
    if (mcpwm_new_operator(&oc, &m->oper) != ESP_OK) return false;
    mcpwm_operator_connect_timer(m->oper, m->timer);

    mcpwm_comparator_config_t cc = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(m->oper, &cc, &m->cmpr_fwd);
    mcpwm_new_comparator(m->oper, &cc, &m->cmpr_rev);
    mcpwm_comparator_set_compare_value(m->cmpr_fwd, 0);
    mcpwm_comparator_set_compare_value(m->cmpr_rev, 0);

    mcpwm_generator_config_t gcf = { .gen_gpio_num = (int)fwd_pin };
    mcpwm_new_generator(m->oper, &gcf, &m->gen_fwd);
    mcpwm_generator_set_action_on_timer_event(m->gen_fwd,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m->gen_fwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->cmpr_fwd, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_config_t gcr = { .gen_gpio_num = (int)rev_pin };
    mcpwm_new_generator(m->oper, &gcr, &m->gen_rev);
    mcpwm_generator_set_action_on_timer_event(m->gen_rev,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m->gen_rev,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->cmpr_rev, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(m->timer);
    mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP);
    return true;
}

// Apply a target (dir, duty) instantly. dir = +1 forward, -1 reverse,
// 0 = both off (stop). duty 0..500 (period_ticks). Avoids shoot-through
// by always clearing the OFF side first.
static void chase_motor_apply(chase_motor_t *m, int dir, uint16_t duty)
{
    if (dir > 0) {
        mcpwm_comparator_set_compare_value(m->cmpr_rev, 0);
        mcpwm_comparator_set_compare_value(m->cmpr_fwd, duty);
    } else if (dir < 0) {
        mcpwm_comparator_set_compare_value(m->cmpr_fwd, 0);
        mcpwm_comparator_set_compare_value(m->cmpr_rev, duty);
    } else {
        mcpwm_comparator_set_compare_value(m->cmpr_fwd, 0);
        mcpwm_comparator_set_compare_value(m->cmpr_rev, 0);
    }
    m->dir  = dir;
    m->duty = (dir == 0) ? 0 : duty;
}

// Smooth direction reversal with cosine-eased deceleration / acceleration.
//
// Profile:
//   1. Decel: ease duty down from current to 0 over ~200 ms using a
//      cosine curve (slow near full duty, faster near zero). Smoother
//      than a linear ramp because the duty derivative is continuous.
//   2. Coast: brief zero-duty interval (~80 ms) for the motor to actually
//      stop. The mechanical reversal happens during this window thanks to
//      inertia + friction; no abrupt direction-pin step under load.
//   3. Kickstart: a single boosted pulse (run_duty * 1.6) for ~40 ms to
//      break static friction in the new direction. Without this the
//      ramp-up below stalls at low duty.
//   4. Accel: ease duty from boost back down to run_duty over ~150 ms,
//      again cosine-shaped. The motor is now moving smoothly in the new
//      direction, and the loop's proximity-based duty scaling takes
//      over.
// Total ~470 ms; the motor traces a smooth U-turn instead of jolting.
static void chase_motor_reverse(chase_motor_t *m, uint16_t run_duty)
{
    int old_dir = (m->dir != 0) ? m->dir : +1;
    int new_dir = -old_dir;
    uint16_t start_duty = m->duty;

    // Phase 1: cosine-ease decel to zero (~200 ms, 10 steps × 20 ms).
    if (m->dir != 0 && start_duty > 0) {
        const int STEPS = 10;
        for (int i = 1; i <= STEPS; i++) {
            float t = (float)i / (float)STEPS;          // 0..1
            float eased = 0.5f * (1.0f + cosf((float)M_PI * t));  // 1..0 smooth
            uint16_t d = (uint16_t)((float)start_duty * eased);
            chase_motor_apply(m, old_dir, d);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    chase_motor_apply(m, 0, 0);

    // Phase 2: coast (motor stops via friction + inertia).
    vTaskDelay(pdMS_TO_TICKS(80));

    // Phase 3: kickstart boost on new side to overcome static friction.
    // Cap at 500 since that's the period_ticks ceiling.
    uint16_t boost = (uint16_t)((unsigned)run_duty + 60u);
    if (boost > 500) boost = 500;
    chase_motor_apply(m, new_dir, boost);
    vTaskDelay(pdMS_TO_TICKS(40));

    // Phase 4: cosine-ease from boost down to run_duty (~150 ms, 8 × 19 ms).
    const int A_STEPS = 8;
    for (int i = 1; i <= A_STEPS; i++) {
        float t = (float)i / (float)A_STEPS;
        float eased = 0.5f * (1.0f - cosf((float)M_PI * t));  // 0..1 smooth (from boost end → run end)
        // Interpolate from boost to run_duty
        uint16_t d = (uint16_t)((float)boost - eased * (float)(boost - run_duty));
        chase_motor_apply(m, new_dir, d);
        vTaskDelay(pdMS_TO_TICKS(19));
    }
    chase_motor_apply(m, new_dir, run_duty);
}

// Shortened reversal (~150 ms total) used during SINUSOID character mode.
// The full chase_motor_reverse takes ~470 ms which is too slow for the
// 2-3 second sine half-periods we want for a snappy "wiggling" stream.
// Same shape -- decel / coast / kickstart / accel -- just compressed.
static void chase_motor_reverse_fast(chase_motor_t *m, uint16_t run_duty)
{
    int old_dir = (m->dir != 0) ? m->dir : +1;
    int new_dir = -old_dir;
    uint16_t start_duty = m->duty;

    if (m->dir != 0 && start_duty > 0) {
        const int STEPS = 5;
        for (int i = 1; i <= STEPS; i++) {
            float t = (float)i / (float)STEPS;
            float eased = 0.5f * (1.0f + cosf((float)M_PI * t));
            uint16_t d = (uint16_t)((float)start_duty * eased);
            chase_motor_apply(m, old_dir, d);
            vTaskDelay(pdMS_TO_TICKS(10));        // 50 ms decel
        }
    }
    chase_motor_apply(m, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(30));                // 30 ms coast
    uint16_t boost = (uint16_t)((unsigned)run_duty + 50u);
    if (boost > 500) boost = 500;
    chase_motor_apply(m, new_dir, boost);
    vTaskDelay(pdMS_TO_TICKS(30));                // 30 ms boost
    chase_motor_apply(m, new_dir, run_duty);      // ~10 ms transition
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void chase_motor_teardown(chase_motor_t *m)
{
    chase_motor_apply(m, 0, 0);
    gpio_set_level(m->fwd_pin, 0);
    gpio_set_level(m->rev_pin, 0);
    mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(m->timer);
    if (m->gen_fwd)  mcpwm_del_generator(m->gen_fwd);
    if (m->gen_rev)  mcpwm_del_generator(m->gen_rev);
    if (m->cmpr_fwd) mcpwm_del_comparator(m->cmpr_fwd);
    if (m->cmpr_rev) mcpwm_del_comparator(m->cmpr_rev);
    if (m->oper)     mcpwm_del_operator(m->oper);
    if (m->timer)    mcpwm_del_timer(m->timer);
    memset(m, 0, sizeof(*m));
}

// ─────────────────────────────────────────────────────────────────────────────
// b423: Serpentine mode -- continuous serpentine coverage watering (mode 8 / 'n').
//
// The OEM firmware waters irregular zones in one continuous boustrophedon:
// concentric arcs alternating direction, joined by smooth turns -- the
// stream never stops. Smooth mode (mode 7) already PLANS exactly those
// arcs (ring_throws + polygon clip) but executes them with brake +
// nozzle_goto reposition + valve jog between rings, and the b296 scheduler
// hops between non-adjacent rings. Serpentine = smooth's ring plan + the b417
// zone_trace glide motion: outermost ring -> innermost, sweep direction
// alternating per ring, the valve gliding to the next ring's throw during
// each turn. Out->in ordering preserves the validated b297 supply insight
// by construction (long-throw rings fire while supply is fresh).
//
// Phase A (this build): single pass at a fixed conservative speed
// (default 8 dps, speed= param 3..20). Inner/direct-valve rings are
// deferred (b367: gliding the valve down into the sub-cal band risks the
// seat dead-band no-flow spot; a seated smooth-style tail is Phase B).
// Depth IS credited per sweep leg into the cumulative tracker (gentle-
// style: every pass, no throw correction) so the shared closeout's
// heatmap / last_water plumbing works unchanged. dry=1 runs the full
// motion with the valve held closed -- a polygon-containment rehearsal
// before water flows.
//
// Runs INSIDE phase_water_zone's water task: branches just before the
// smooth/gentle pass loop and rejoins at the shared abort:/closeout
// (valve close, aggregate flush, depth compute, run summary).

typedef enum {
    SERPENTINE_LEG_REPOS = 0,   // dry full-speed aim to the path start, valve held
    SERPENTINE_LEG_TURN,        // ring-to-ring turn / radial valve glide
    SERPENTINE_LEG_SWEEP,       // watering sweep across one ring arc
    SERPENTINE_LEG_TRANSIT,     // dry hop across an inactive gap (multi-arc rings)
} serpentine_leg_kind_t;

typedef struct {
    float    b1_deg;       // leg end bearing (start = ACTUAL current bearing)
    float    v1_deg;       // valve target at leg end; < 0 = hold (dry reposition)
    uint16_t duty;         // nozzle PWM for SWEEP/TURN legs (speed-map solved)
    float    dps;          // planned deg/s (timeout sizing + measured-dps fallback)
    int8_t   ring;         // ring index for depth/CSV attribution; -1 = none
    int8_t   dir;          // +1 CW, -1 CCW (sweep/transit); 0 = shortest path
    uint8_t  kind;         // serpentine_leg_kind_t
    float    ring_throw;   // mm, CSV target columns (SWEEP legs)
} serpentine_leg_t;

// 36 rings x (turn + sweep) + boundary-hug waypoint (b426) and multi-arc
// slack. Static: the water task stack is 16 KB and has history of running
// close (b285).
#define SERPENTINE_MAX_LEGS 256
// b426: boundary-hugging turns. Waypoint pitch along the connector, and how
// far inside the polygon's radial extent the hug path rides. The hug radius
// at each waypoint is min(linear glide, perimeter extent - margin); a turn
// goes dry only when that falls below the waterable floor (true exclusion).
#define SERPENTINE_TURN_STEP_DEG   4.0f
#define SERPENTINE_TURN_MARGIN_MM  200.0f
#define SERPENTINE_TURN_MAX_WPS    64
static serpentine_leg_t s_serpentine_legs[SERPENTINE_MAX_LEGS];

// b423: directional speed-map duty lookup, extracted verbatim from the
// smooth-mode ring loop (CCW/NREV can run at a very different speed than
// CW/NFWD at the same duty -- use the CCW-specific table when available).
// Returns 250 when no usable map exists (smooth's historical default).
static uint16_t spd_duty_for_dps(const speed_map_t *spd, bool have_spd,
                                 float dps, bool cw)
{
    uint16_t run_duty = 250;
    bool use_ccw = !cw && have_spd && (spd->num_points_ccw >= 2);
    int             np = use_ccw ? spd->num_points_ccw  : spd->num_points;
    const uint16_t *dt = use_ccw ? spd->duty_ccw        : spd->duty;
    const float    *ds = use_ccw ? spd->deg_per_sec_ccw : spd->deg_per_sec;
    if (have_spd && np >= 2) {
        if (dps <= ds[0]) {
            run_duty = dt[0];
        } else if (dps >= ds[np-1]) {
            run_duty = dt[np-1];
        } else {
            for (int k = 0; k < np-1; k++) {
                if (dps >= ds[k] && dps <= ds[k+1]) {
                    float fr = (dps - ds[k]) / (ds[k+1] - ds[k]);
                    run_duty = (uint16_t)(dt[k] + fr*(dt[k+1]-dt[k]));
                    break;
                }
            }
        }
    }
    if (use_ccw)
        INFO("  (CCW table: duty=%u for %.1fdps)", run_duty, dps);
    return run_duty;
}

// b423: per-ring clipped arc bounds in CW order along the zone arc.
// Mirrors the smooth ring loop's per-arc bound derivation (sector activity
// bitmap -> water_find_arcs -> snap-to-zone-edge -> zone_refine_arc_bound
// -> zone_clip_arc_to_polygon) but standalone -- the inline original is
// interleaved with pass state. Drops arcs whose fully-inside span is below
// half a sector (b309: never re-expand a clipped arc). Returns arc count.
static int serpentine_arc_bounds(const zone_perimeter_t *zone, bool have_zone,
                            float ring_throw, const float *sector_throw,
                            float act_max_throw,
                            float zone_arc_start, float zone_arc_end,
                            float zone_arc_deg,
                            float lo_out[WATER_MAX_ARCS_PER_RING],
                            float hi_out[WATER_MAX_ARCS_PER_RING])
{
    bool active[WATER_SECTORS];
    int  active_count = 0;
    for (int s = 0; s < WATER_SECTORS; s++) {
        float tol = WATER_RING_SPACING * (ring_throw / act_max_throw) * 0.5f;
        active[s] = (ring_throw <= sector_throw[s] + tol);
        if (active[s]) active_count++;
    }
    if (active_count == 0) return 0;

    water_arc_t arcs[WATER_MAX_ARCS_PER_RING];
    int num_arcs = water_find_arcs(active, arcs);

    // Sort by CW distance of first_sec from zone_arc_start (handles zones
    // crossing 0 deg; see the smooth loop's sort).
    for (int a = 0; a < num_arcs-1; a++)
        for (int b = a+1; b < num_arcs; b++) {
            float s_a = arcs[a].first_sec * WATER_SECTOR_DEG;
            float s_b = arcs[b].first_sec * WATER_SECTOR_DEG;
            float da = fmodf(s_a - zone_arc_start + 360.0f, 360.0f);
            float db = fmodf(s_b - zone_arc_start + 360.0f, 360.0f);
            if (db < da) { water_arc_t tmp=arcs[a]; arcs[a]=arcs[b]; arcs[b]=tmp; }
        }

    int n_out = 0;
    for (int ai = 0; ai < num_arcs && n_out < WATER_MAX_ARCS_PER_RING; ai++) {
        float first_b = (float)(arcs[ai].first_sec) * WATER_SECTOR_DEG;
        float last_b  = (float)((arcs[ai].last_sec + 1) % WATER_SECTORS) * WATER_SECTOR_DEG;

        float _lo_cw_from_start = fmodf(first_b - zone_arc_start + 360.0f, 360.0f);
        bool  _first_in_zone    = (_lo_cw_from_start <= zone_arc_deg + 0.5f);
        float left_gap = fminf(_lo_cw_from_start,
                               fmodf(zone_arc_start - first_b + 360.0f, 360.0f));
        float lo = (!_first_in_zone || left_gap < 0.5f * WATER_SECTOR_DEG)
                   ? zone_arc_start : first_b;

        float _hi_cw_from_start = fmodf(last_b - zone_arc_start + 360.0f, 360.0f);
        bool  _last_in_zone     = (_hi_cw_from_start <= zone_arc_deg + 0.5f);
        float right_gap = fminf(fmodf(last_b      - zone_arc_end + 360.0f, 360.0f),
                                fmodf(zone_arc_end - last_b      + 360.0f, 360.0f));
        float hi = (!_last_in_zone || right_gap < 0.5f * WATER_SECTOR_DEG)
                   ? zone_arc_end : last_b;

        // Full-circle arc (all sectors active): lo == hi after the snaps and
        // every fmodf span collapses to 0. Represent it as a 359.5 deg sweep
        // so the clip and the glide engine see a real span.
        if (active_count == WATER_SECTORS &&
            fmodf(hi - lo + 360.0f, 360.0f) < 0.01f) {
            hi = fmodf(lo + 359.5f, 360.0f);
        }

        if (have_zone && lo == first_b)
            lo = zone_refine_arc_bound(zone, ring_throw, first_b, WATER_SECTOR_DEG);
        if (have_zone && hi == last_b)
            hi = zone_refine_arc_bound(zone, ring_throw, last_b, WATER_SECTOR_DEG);
        // b309: tighten to the longest contiguous fully-inside sub-arc.
        if (have_zone)
            zone_clip_arc_to_polygon(zone, ring_throw, &lo, &hi, 0.5f);

        float seg_deg = fmodf(hi - lo + 360.0f, 360.0f);
        if (seg_deg < 0.5f * WATER_SECTOR_DEG) continue;   // b309: don't re-expand
        lo_out[n_out] = fmodf(lo + 360.0f, 360.0f);
        hi_out[n_out] = fmodf(hi + 360.0f, 360.0f);
        n_out++;
    }
    return n_out;
}

// b423: ring throw -> valve angle, mirroring the smooth ring loop's lookup:
// band-anchored direct-valve interpolation for sub-cal rings (b382), the
// throw-cal table otherwise, pressure-ratio fallback when uncalibrated.
// pressure_scale applies smooth's pass-0 global supply correction (open
// further when today's supply is below cal). Clamped to the valve's range.
static float serpentine_ring_valve_deg(float ring_throw, bool direct_ring,
                                  float ref_throw_direct, bool have_throw_cal,
                                  float act_max_throw, float pressure_scale,
                                  float psi_min, float psi_max)
{
    // b428: do NOT apply the global pressure_scale here. It is measured at
    // FULL OPEN, where a well/tank supply sags hardest; at partial openings
    // the supply barely sags (b427 wet run: back-computed supply 5.7 PSI at
    // full-flow rings vs 13.2 PSI at barely-open ones), so scaling every
    // lookup throw by 1/scale -- smooth's pass-0 trick -- over-opened every
    // ring and landed water at 1.4x (outer) to 2.3x (inner) of each ring's
    // radius. The cal curve already embeds cal-day partial-open supply
    // behavior; day-to-day correction belongs to per-ring measured corr
    // (Phase B), not a full-open global multiplier. Smooth survives the
    // same scale because its corr fixes pass 1+ and one pass is 1/N of the
    // run; serpentine's single coat is 100% of it.
    (void)pressure_scale;
    float vrt = ring_throw;
    float v;
    if (direct_ring) {
        float ref_valve = (ref_throw_direct > 0.0f)
            ? cal_throw_to_valve_deg(ref_throw_direct) : VALVE_CAL_START_DEG;
        v = (ref_throw_direct > 0.0f)
            ? VALVE_CAL_START_DEG
              + (ref_valve - VALVE_CAL_START_DEG) * (vrt / ref_throw_direct)
            : VALVE_CAL_START_DEG;
    } else if (have_throw_cal) {
        v = cal_throw_to_valve_deg(vrt);
    } else {
        float ring_pct = vrt / act_max_throw * 100.0f;
        float ring_psi = psi_min + (ring_pct / 100.0f) * (psi_max - psi_min);
        v = cal_pressure_to_valve_deg(ring_psi);
    }
    if (v < VALVE_CAL_START_DEG) v = VALVE_CAL_START_DEG;
    return fmaxf(VALVE_CLOSED_DEG, fminf(VALVE_OPEN_DEG, v));
}

// b429: flow-solved sweep speed -- one sweep of this ring deposits
// per_pass_depth at the calibration-reference PSI (smooth's solver: dps =
// Q_ref * active_deg / (depth * 60000 * ring_area), annular area with the
// splash-radius special case for short throws). Clamped to the speed map's
// continuous band; a ceiling-pinned inner ring over-deposits once and is
// then satisfied by the cumulative tracker (the b382 TopCorner lesson:
// never skip, always water).
static float serpentine_ring_dps(float ring_throw, float inner_throw,
                            float active_deg, float per_pass_depth,
                            const speed_map_t *spd, bool have_spd)
{
    float r_outer = ring_throw  / 1000.0f;
    float r_inner = inner_throw / 1000.0f;
    if (ring_throw < WATER_MIN_ELLIPSE_THROW_MM) {
        float sr = WATER_INNER_SPLASH_RADIUS_MM / 1000.0f;
        r_outer = ring_throw / 1000.0f + sr * 0.5f;
        r_inner = fmaxf(0.0f, ring_throw / 1000.0f - sr * 0.5f);
    }
    float arc_fraction = active_deg / 360.0f;
    float ring_area = 3.14159f * (r_outer * r_outer - r_inner * r_inner)
                      * arc_fraction;
    float dps = 12.0f;
    float ref_psi = cal_throw_to_psi(ring_throw);
    if (ring_area > 1e-4f && per_pass_depth > 0.001f && ref_psi > 0.1f) {
        float Q_ref = NOZZLE_FLOW_K * powf(ref_psi, NOZZLE_FLOW_N);
        dps = Q_ref * active_deg / (per_pass_depth * 60000.0f * ring_area);
    }
    float min_dps = have_spd ? spd->min_continuous_dps : 10.9f;
    float max_dps = (have_spd && spd->num_points > 0)
                    ? spd->deg_per_sec[spd->num_points - 1] : 118.0f;
    if (dps < min_dps) dps = min_dps;
    if (dps > max_dps) dps = max_dps;
    return dps;
}

// b423: build the serpentine leg list for ONE pass into s_serpentine_legs[].
// Ring order out->in (or reversed); rings with skip[] set emit nothing --
// the next ring's connector absorbs the radial distance. First emitted
// ring: dry REPOS to the entry bearing + radial open. Every other arc is
// reached by a b426 boundary-hugging wet connector (see the inline comment
// in the loop), then swept to its far bound, direction alternating per
// ring. The valve closes only when a connector crosses a true exclusion
// (polygon extent below the waterable floor). Returns the leg count.
static int serpentine_build_pass_plan(
    const zone_perimeter_t *zone, bool have_zone,
    const float *sector_throw,
    const float *ring_throws, const bool *ring_direct,
    const float *ring_ref_throw, int num_rings,
    const bool *skip, bool out_to_in, bool start_cw,
    float zone_arc_start, float zone_arc_end, float zone_arc_deg,
    float act_max_throw, bool have_throw_cal, float pressure_scale,
    float psi_min, float psi_max,
    const speed_map_t *spd, bool have_spd,
    float serpentine_dps,            // > 0 = manual override; 0 = flow-solved (b429)
    float depth_mm,             // b429: per-pass deposit target for the solver
    const float *valve_corr)    // b429: per-ring lookup-throw multiplier
{
    int  n     = 0;
    bool cw    = start_cw;
    bool first = true;
    float prev_exit_b = -1.0f;   // b425: bearing where the previous sweep ended
    float prev_throw  = 0.0f;    // b425: its ring throw, for the connector
    // b426: below this throw a boundary-hug waypoint can't water anyway --
    // the connector goes dry instead (true exclusions stay dry).
    float turn_floor = fmaxf(cal_get_min_throw_mm(), 500.0f);

#define SERPENTINE_EMIT(_b, _v, _duty, _dps, _ring, _dir, _kind, _throw) do {      \
        if (n >= SERPENTINE_MAX_LEGS) {                                            \
            INFO("Serpentine plan: leg cap (%d) hit -- truncating", SERPENTINE_MAX_LEGS);\
            return n;                                                         \
        }                                                                     \
        s_serpentine_legs[n++] = (serpentine_leg_t){ .b1_deg=(_b), .v1_deg=(_v),        \
            .duty=(_duty), .dps=(_dps), .ring=(int8_t)(_ring),                \
            .dir=(int8_t)(_dir), .kind=(uint8_t)(_kind),                      \
            .ring_throw=(_throw) };                                           \
    } while (0)

    for (int ri = 0; ri < num_rings && ri < WATER_RUN_MAX_RINGS; ri++) {
        int ring = out_to_in ? ri : (num_rings - 1 - ri);
        if (ring >= WATER_RUN_MAX_RINGS || skip[ring]) continue;
        float ring_throw = ring_throws[ring];

        float lo[WATER_MAX_ARCS_PER_RING], hi[WATER_MAX_ARCS_PER_RING];
        int na = serpentine_arc_bounds(zone, have_zone, ring_throw, sector_throw,
                                  act_max_throw, zone_arc_start, zone_arc_end,
                                  zone_arc_deg, lo, hi);
        if (na == 0) continue;

        // b429: flow-solved per-ring speed (or the manual speed= override),
        // and the per-ring corr applied to the valve lookup throw (b303
        // clamp so corr can engage the beyond-cal extrapolation).
        float active_deg = 0.0f;
        for (int k = 0; k < na; k++)
            active_deg += fmodf(hi[k] - lo[k] + 360.0f, 360.0f);
        float inner_throw = (ring == num_rings - 1) ? ring_throw * 0.92f
                                                    : ring_throws[ring + 1];
        float dps_ring = (serpentine_dps > 0.0f)
            ? serpentine_dps
            : serpentine_ring_dps(ring_throw, inner_throw, active_deg, depth_mm,
                             spd, have_spd);
        float lookup_throw = fminf(ring_throw * valve_corr[ring],
                                   act_max_throw * 1.5f);
        float valve = serpentine_ring_valve_deg(lookup_throw, ring_direct[ring],
                                           ring_ref_throw[ring], have_throw_cal,
                                           act_max_throw, pressure_scale,
                                           psi_min, psi_max);
        uint16_t duty = spd_duty_for_dps(spd, have_spd, dps_ring, cw);

        // Arcs in physical sweep order for this ring's direction.
        for (int k = 0; k < na; k++) {
            int   ai    = cw ? k : (na - 1 - k);
            float entry = cw ? lo[ai] : hi[ai];
            float exitb = cw ? hi[ai] : lo[ai];

            if (first) {
                SERPENTINE_EMIT(entry, -1.0f, 0, 0.0f, -1, 0, SERPENTINE_LEG_REPOS, 0.0f);
                SERPENTINE_EMIT(entry, valve, 0, 0.0f, ring, 0, SERPENTINE_LEG_TURN, ring_throw);
                first = false;
            } else {
                // b426: boundary-hugging wet connector -- used for BOTH
                // ring-to-ring turns and same-ring lobe gaps. The OEM
                // serpentine never shuts the stream off at a turn: when the
                // direct glide would exit the polygon, the path rides just
                // inside the boundary instead (radius = min(linear glide,
                // perimeter extent - margin) at ~4 deg waypoints; the glide
                // engine carries through same-direction waypoint chains, so
                // it stays one continuous motion). The valve closes only
                // when the polygon's extent at some bearing drops below the
                // waterable floor -- a true exclusion (driveway-style notch)
                // that must stay dry. Each waypoint is PIP-verified; any
                // failure also falls back to the dry hop (b425 trio).
                float d = entry - prev_exit_b;
                while (d >  180.0f) d -= 360.0f;
                while (d < -180.0f) d += 360.0f;
                int nw = (int)(fabsf(d) / SERPENTINE_TURN_STEP_DEG) + 1;
                if (nw > SERPENTINE_TURN_MAX_WPS) nw = SERPENTINE_TURN_MAX_WPS;
                float wpb[SERPENTINE_TURN_MAX_WPS], wpr[SERPENTINE_TURN_MAX_WPS];
                bool wet_ok = true;
                for (int wi = 1; wi <= nw; wi++) {
                    float f = (float)wi / (float)nw;
                    float b = fmodf(prev_exit_b + d * f + 360.0f, 360.0f);
                    float r = prev_throw + (ring_throw - prev_throw) * f;
                    if (have_zone) {
                        float ext = water_perimeter_throw(zone, b, psi_min, psi_max)
                                    - SERPENTINE_TURN_MARGIN_MM;
                        if (ext < r) r = ext;
                        if (!zone_contains_point(zone, b, r))
                            r -= SERPENTINE_TURN_MARGIN_MM;   // one nudge inward
                        if (r < turn_floor || !zone_contains_point(zone, b, r)) {
                            wet_ok = false;
                            break;
                        }
                    }
                    wpb[wi - 1] = b;
                    wpr[wi - 1] = r;
                }
                if (wet_ok) {
                    for (int wi = 0; wi < nw; wi++) {
                        bool last = (wi == nw - 1);
                        float wv = last ? valve
                            : serpentine_ring_valve_deg(wpr[wi], false, 0.0f,
                                                   have_throw_cal, act_max_throw,
                                                   pressure_scale, psi_min, psi_max);
                        SERPENTINE_EMIT(last ? entry : wpb[wi], wv, duty, dps_ring,
                                   ring, 0, SERPENTINE_LEG_TURN, ring_throw);
                    }
                    if (nw > 2)
                        INFO("Serpentine plan r%d: boundary-hug turn %.1f -> %.1f "
                             "(%d waypoints)", ring + 1, prev_exit_b, entry, nw);
                } else {
                    // True exclusion: close IN PLACE, hop dry, reopen at the
                    // entry (upward, the b367-safe direction).
                    SERPENTINE_EMIT(prev_exit_b, VALVE_CAL_START_DEG, 0, 0.0f, -1, 0,
                               SERPENTINE_LEG_TURN, 0.0f);              // radial close
                    SERPENTINE_EMIT(entry, VALVE_CAL_START_DEG, 0, 0.0f, -1, 0,
                               SERPENTINE_LEG_TRANSIT, 0.0f);           // dry hop
                    SERPENTINE_EMIT(entry, valve, 0, 0.0f, ring, 0,
                               SERPENTINE_LEG_TURN, ring_throw);        // reopen
                    INFO("Serpentine plan r%d: connector %.1f -> %.1f below waterable "
                         "floor -- dry hop", ring + 1, prev_exit_b, entry);
                }
            }
            SERPENTINE_EMIT(exitb, valve, duty, dps_ring, ring, cw ? +1 : -1,
                       SERPENTINE_LEG_SWEEP, ring_throw);
            prev_exit_b = exitb;                                   // b425
            prev_throw  = ring_throw;   // b426: per ARC (lobe-gap connectors)
            float span = fmodf(hi[ai] - lo[ai] + 360.0f, 360.0f);
            INFO("Serpentine plan r%d arc%d: %.1f -> %.1f %s (%.1f deg) v=%.1f "
                 "duty=%u %.1fdps", ring + 1, ai + 1, entry, exitb,
                 cw ? "CW" : "CCW", span, valve, duty, dps_ring);
        }
        cw = !cw;
    }
#undef SERPENTINE_EMIT
    return n;
}

// b423: the glide executor -- generalized from zone_trace_run's leg loop
// (b417) with per-leg duty, a 2-deg CSV/trace sampler on wet sweep legs
// (feeds s_smooth_accum in RAM; no file I/O mid-run, b285/b288), per-leg
// measured dps + avg PSI feeding the cumulative depth tracker, stiction
// recovery kicks and the dwell watchdog (gentle-sweep patterns), and
// abort via s_water_abort / uart 'q' (serpentine IS the watering run, so no
// yield-to-watering check). Motion model unchanged from b417: every leg
// derived from the ACTUAL current bearing, the valve chasing a target
// interpolated along the leg's measured sweep progress, carry-through at
// same-direction joints, cosine-ease braking only into reversals/radials.
// Does NOT close the valve or touch the rails on exit -- the caller's
// shared closeout owns teardown.
static bool serpentine_glide_legs(const serpentine_leg_t *legs, int n,
                             const float *ring_throws, int num_rings,
                             float *cumulative_depth,
                             float *run_psi_sum, int *run_psi_n,
                             int *ring_sweeps_out,
                             TickType_t t_start, bool dry)
{
    const uint16_t N_MIN_DUTY    = 70;
    const uint16_t N_ALIGN_DUTY  = 380;   // dry reposition/transit (aim speed)
    const float    N_TOL_DEG     = 1.0f;
    const float    MIN_SWEEP_DEG = 1.0f;  // below this a leg is radial
    const uint16_t V_CHASE_DUTY  = 220;
    const uint16_t V_MIN_DUTY    = 70;
    const float    V_TOL_DEG     = 0.8f;
    const float    V_DECEL_DEG   = 4.0f;
    const float    V_LAG_DEG     = 5.0f;
    const float    V_NOM_DPS     = 8.0f;  // timeout estimates only
    const uint32_t TICK_MS       = 30;
    const int      VALVE_OPEN_DIR = -1;   // FWD pin closes valve (b318/b418)

    chase_motor_t nm = {0}, vm = {0};
    if (!chase_motor_setup(&nm, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
        INFO("serpentine: nozzle PWM setup failed");
        return false;
    }
    if (!chase_motor_setup(&vm, VALVE_PIN_FWD, VALVE_PIN_REV)) {
        INFO("serpentine: valve PWM setup failed");
        chase_motor_teardown(&nm);
        return false;
    }

    bool ok = true;
    int  v_dir = 0;
    bool flow_checked    = false;
    // b427: the no-flow gate accumulates samples ACROSS legs -- Edge's outer
    // arcs are 6-10 deg (~3 samples each), too few to judge supply on one.
    float flow_psi_sum   = 0.0f;
    int   flow_psi_n     = 0;
    int  last_trace_ring = -1;
    TickType_t t_run = xTaskGetTickCount();

    for (int leg = 0; leg < n && ok; leg++) {
        const serpentine_leg_t *L = &legs[leg];
        uint16_t n_raw = 0, v_raw = 0;
        as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
        as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
        float cur_b = n_raw * (360.0f / 4096.0f);
        float v0    = v_raw * (360.0f / 4096.0f);

        // Leg geometry from the ACTUAL current position (positioning error
        // never accumulates -- b417). Sweeps/transits carry an explicit
        // direction: ring arcs can exceed 180 deg (sprinkler-inside zones),
        // where shortest-path derivation would sweep the wrong way.
        float span; int dirn;
        if (L->dir > 0) {
            span = fmodf(L->b1_deg - cur_b + 360.0f, 360.0f); dirn = +1;
        } else if (L->dir < 0) {
            span = fmodf(cur_b - L->b1_deg + 360.0f, 360.0f); dirn = -1;
        } else {
            float d = L->b1_deg - cur_b;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            span = fabsf(d);
            dirn = (d >= 0.0f) ? +1 : -1;
        }
        if (span > 355.0f) span = 0.0f;   // explicit-dir wrap noise: already there
        bool  radial  = (span < MIN_SWEEP_DEG);
        bool  dry_leg = (L->v1_deg < 0.0f) || dry;
        float v1      = dry_leg ? v0 : L->v1_deg;
        bool  sweep   = (L->kind == SERPENTINE_LEG_SWEEP);
        uint16_t leg_duty =
            (L->kind == SERPENTINE_LEG_REPOS || L->kind == SERPENTINE_LEG_TRANSIT)
                ? N_ALIGN_DUTY
                : (L->duty > 0 ? L->duty : N_ALIGN_DUTY);
        if (leg_duty < N_MIN_DUTY) leg_duty = N_MIN_DUTY;
        // Speed-scaled decel band: ~0.3 s of travel at the leg's planned
        // speed (zone_trace's fixed 12/5 deg was tuned for <= 25 dps).
        float decel_band = fmaxf(5.0f, (L->dps > 0.1f ? L->dps : 25.0f) * 0.3f);

        // Brake into this leg's end unless the path carries straight
        // through into the next leg (same direction, non-radial).
        bool brake_end = true;
        if (!radial && leg + 1 < n) {
            const serpentine_leg_t *NL = &legs[leg + 1];
            float dn = NL->b1_deg - L->b1_deg;
            while (dn >  180.0f) dn -= 360.0f;
            while (dn < -180.0f) dn += 360.0f;
            int ndir = (NL->dir != 0) ? NL->dir : ((dn >= 0.0f) ? +1 : -1);
            brake_end = (fabsf(dn) < MIN_SWEEP_DEG) || (ndir != dirn);
        }

        float eff_dps = (L->dps > 0.5f) ? L->dps : (float)leg_duty / 15.0f;
        float exp_s = radial ? 0.0f : span / eff_dps;
        float vexp  = fabsf(v1 - v0) / V_NOM_DPS;
        if (vexp > exp_s) exp_s = vexp;
        uint32_t leg_timeout = (uint32_t)(exp_s * 2000.0f) + 4000;

        // b283: tag trace samples with the ring about to be swept.
        if (sweep && !dry_leg && L->ring >= 0 && L->ring != last_trace_ring) {
            water_trace_mark_ring(L->ring, L->v1_deg);
            last_trace_ring = L->ring;
        }

        if (radial) {
            chase_motor_apply(&nm, 0, 0);
        } else if (nm.dir != dirn || nm.duty == 0) {
            // Entering a fresh direction. If the motor is still turning the
            // other way, the eased reverse handles it; from rest, a brief
            // boost breaks stiction at low sweep duties (gentle kickstart).
            if (nm.dir != 0 && nm.dir != dirn) {
                chase_motor_reverse_fast(&nm, leg_duty);
            } else {
                uint16_t kick = (uint16_t)(leg_duty + 100u);
                if (kick > 480u) kick = 480u;
                chase_motor_apply(&nm, dirn, kick);
                vTaskDelay(pdMS_TO_TICKS(60));
                chase_motor_apply(&nm, dirn, leg_duty);
            }
        }

        float leg_psi_sum = 0.0f; int leg_psi_n = 0;
        float last_csv_prog = 0.0f;
        float prev_b = cur_b, traveled = 0.0f;
        TickType_t t_leg = xTaskGetTickCount();
        TickType_t last_motion_tick = t_leg;
        float      last_motion_b    = cur_b;
        int        kicks_used       = 0;
        TickType_t last_dwell_tick  = t_leg;
        float      last_dwell_b     = cur_b;

        for (;;) {
            if (uart_getchar(0) != 0 || s_water_abort) {
                s_water_abort = true;
                ok = false;
                break;
            }
            if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_leg) >= leg_timeout) {
                INFO("serpentine: leg %d timeout (%.1f / %.1f deg) -- continuing",
                     leg, fabsf(traveled), span);
                break;   // next leg re-derives from the actual position
            }
            vTaskDelay(pdMS_TO_TICKS(TICK_MS));
            TOUCH_ACTIVITY();

            as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
            as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
            cur_b = n_raw * (360.0f / 4096.0f);
            float cur_v = v_raw * (360.0f / 4096.0f);

            float db = cur_b - prev_b;
            while (db >  180.0f) db -= 360.0f;
            while (db < -180.0f) db += 360.0f;
            traveled += db;
            prev_b    = cur_b;

            float progress = radial ? span : traveled * (float)dirn;
            if (progress < 0.0f)  progress = 0.0f;
            if (progress > span)  progress = span;
            float frac = (radial || span < 0.01f) ? 1.0f : progress / span;

            // Valve: chase the along-leg interpolated target.
            float v_tgt = v0 + (v1 - v0) * frac;
            float v_err = v_tgt - cur_v;
            float v_abs = fabsf(v_err);
            if (v_abs < V_TOL_DEG) {
                if (v_dir) { chase_motor_apply(&vm, 0, 0); v_dir = 0; }
            } else {
                int want = (v_err >= 0.0f) ? VALVE_OPEN_DIR : -VALVE_OPEN_DIR;
                uint16_t vd;
                if (v_abs < V_DECEL_DEG) {
                    float t    = v_abs / V_DECEL_DEG;
                    float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                    vd = (uint16_t)((float)V_MIN_DUTY
                            + ease * (float)(V_CHASE_DUTY - V_MIN_DUTY));
                } else {
                    vd = V_CHASE_DUTY;
                }
                if (v_dir && want != v_dir) {
                    chase_motor_reverse_fast(&vm, vd);
                    v_dir = vm.dir;
                } else {
                    chase_motor_apply(&vm, want, vd);
                    v_dir = want;
                }
                s_valve_last_dir = v_dir;
            }

            // Dwell watchdog: cap valve-open time near one bearing (the
            // valve is open for the entire run; gentle-sweep pattern).
            if (!dry_leg) {
                float dd = fabsf(cur_b - last_dwell_b);
                if (dd > 180.0f) dd = 360.0f - dd;
                if (dd > 3.0f) {
                    last_dwell_tick = xTaskGetTickCount();
                    last_dwell_b    = cur_b;
                } else if (!radial &&
                           (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - last_dwell_tick)
                               > s_dwell_timeout_ms) {
                    ESP_LOGW(TAG, "serpentine: dwell watchdog at ~%.1f deg with valve "
                                  "open -- slamming valve closed", cur_b);
                    water_set_status(WATER_STATUS_NOZZLE_FAULT);
                    chase_motor_apply(&nm, 0, 0);
                    chase_motor_apply(&vm, 0, 0);
                    valve_goto(VALVE_CLOSED_DEG, 2.0f, 8000, false);
                    ok = false;
                    break;
                }
            }

            if (radial) {
                if (v_abs < V_TOL_DEG) break;   // valve settled -> leg done
                continue;
            }

            // Nozzle: constant duty, eased only into braking ends.
            float to_go = span - progress;
            if (to_go <= N_TOL_DEG) {
                if (brake_end) chase_motor_apply(&nm, 0, 0);
                break;
            }
            uint16_t nd = leg_duty;
            if (brake_end && to_go < decel_band) {
                float t    = to_go / decel_band;
                float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                nd = (uint16_t)((float)N_MIN_DUTY
                        + ease * (float)(leg_duty - N_MIN_DUTY));
            }
            if (v_abs > V_LAG_DEG) nd = N_MIN_DUTY;   // let the valve catch up
            chase_motor_apply(&nm, dirn, nd);
            s_nozzle_last_dir = dirn;

            // Stiction recovery: no encoder motion mid-leg -> boost kick
            // (gentle-sweep recovery pattern; the dwell watchdog above and
            // the leg timeout are the fault backstops).
            {
                float md = fabsf(cur_b - last_motion_b);
                if (md > 180.0f) md = 360.0f - md;
                if (md > 0.1f) {
                    last_motion_tick = xTaskGetTickCount();
                    last_motion_b    = cur_b;
                } else if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - last_motion_tick)
                               > 800u
                           && to_go > decel_band && kicks_used < 6) {
                    uint16_t kick = (uint16_t)(leg_duty + 100u);
                    if (kick > 480u) kick = 480u;
                    INFO("serpentine: leg %d slow at %.1f deg -- recovery kick %d/6",
                         leg, cur_b, kicks_used + 1);
                    chase_motor_apply(&nm, dirn, kick);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    chase_motor_apply(&nm, dirn, nd);
                    kicks_used++;
                    last_motion_tick = xTaskGetTickCount();
                }
            }

            // Sampler: every 2 deg of a wet sweep leg, mirror the gentle
            // sweep's CSV/trace row. In smooth-aggregate mode this lands in
            // the RAM buckets; ring index is 0-based to match the flush.
            if (sweep && !dry_leg && progress >= last_csv_prog + 2.0f) {
                last_csv_prog = progress;
                float t_s = (float)(xTaskGetTickCount() - t_start)
                            / (float)configTICK_RATE_HZ;
                float csv_psi = 0.0f; mprls_read_quiet(&csv_psi);
                if (csv_psi > 0.1f) {
                    leg_psi_sum += csv_psi; leg_psi_n++;
                    if (run_psi_sum) { *run_psi_sum += csv_psi; (*run_psi_n)++; }
                }
                water_trace_sample(csv_psi);
                float _at2 = cal_pressure_to_throw_mm(csv_psi);
                float _tp  = cal_throw_to_psi(L->ring_throw);
                float _tr  = cur_b * (float)M_PI / 180.0f;
                int   _sn  = ((int)(cur_b / WATER_SECTOR_DEG) + WATER_SECTORS)
                             % WATER_SECTORS;
                water_csv_write_row(s_water_csv_f, t_s, L->ring, 1, _sn,
                    (float)_sn * WATER_SECTOR_DEG, cur_b,
                    v1, cur_v,
                    L->ring_throw, _at2, _tp, csv_psi, _tr, s_csv_pass_type);
            }
        }   // tick loop

        if (!ok) break;

        // b427: PSI settle after a flow-starting valve open (run start, or
        // reopen after a dry hop). The valve chase reaches the ring target
        // in ~3 s but supply pressure needs ~1.5 s more to stabilize (b361
        // waits 1500 ms in the supply check for the same reason). Without
        // this, the first 6-deg outer arc sampled 0.26 PSI of just-arriving
        // water and tripped the no-flow abort.
        if (radial && !dry_leg
                && v0 <= VALVE_CAL_START_DEG + 2.0f
                && v1 >  VALVE_CAL_START_DEG + 2.0f) {
            vTaskDelay(pdMS_TO_TICKS(1800));
            TOUCH_ACTIVITY();
        }

        // Wet sweep leg done: measured speed -> depth credit + ring data.
        if (sweep && !dry_leg && L->ring >= 0 && L->ring < WATER_RUN_MAX_RINGS) {
            uint32_t leg_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_leg);
            float meas_deg  = fabsf(traveled);
            float meas_dps  = (leg_ms > 200 && meas_deg > 0.5f)
                              ? meas_deg * 1000.0f / (float)leg_ms
                              : (L->dps > 0.1f ? L->dps : 8.0f);
            float avg_psi   = (leg_psi_n > 0) ? leg_psi_sum / (float)leg_psi_n
                                              : 0.0f;

            // First flow of the run: no detectable flow means supply is off
            // / valve fault -- stop rather than dry-trace the whole zone
            // (WATER_MIN_FLOW_PSI gate, smooth's no-flow check). b427:
            // judge on >= 4 samples accumulated across however many sweep
            // legs it takes, not one tiny outer arc's worth.
            if (!flow_checked) {
                flow_psi_sum += leg_psi_sum;
                flow_psi_n   += leg_psi_n;
                if (flow_psi_n >= 4) {
                    flow_checked = true;
                    float fav = flow_psi_sum / (float)flow_psi_n;
                    if (fav < WATER_MIN_FLOW_PSI) {
                        INFO("serpentine: no flow over first %d samples (%.3f PSI "
                             "< %.2f) -- stopping", flow_psi_n, fav,
                             WATER_MIN_FLOW_PSI);
                        water_set_status(WATER_STATUS_WATER_LOSS);
                        ok = false;
                        break;
                    }
                    s_water_run_had_flow = true;
                }
            }

            int   ring = L->ring;
            float ro   = ring_throws[ring];
            float ri   = (ring == num_rings - 1) ? ro * 0.92f
                                                 : ring_throws[ring + 1];
            cumulative_depth[ring] +=
                nozzle_precip_depth_mm(ro, ri, meas_dps, avg_psi);

            float _at = (avg_psi > 0.1f) ? cal_pressure_to_throw_mm(avg_psi) : ro;
            float _arc_s = (dirn > 0)
                ? fmodf(L->b1_deg - meas_deg + 360.0f, 360.0f) : L->b1_deg;
            float _arc_e = (dirn > 0)
                ? L->b1_deg : fmodf(L->b1_deg + meas_deg, 360.0f);
            s_last_water_run.rings[ring] = (water_ring_data_t){
                .throw_mm        = ro,
                .avg_psi         = avg_psi,
                .actual_throw_mm = _at,
                .dps             = meas_dps,
                .active_deg      = meas_deg,
                .arc_start_deg   = _arc_s,
                .arc_end_deg     = _arc_e,
                .valve_deg       = L->v1_deg,
            };
            if (ring_sweeps_out) (*ring_sweeps_out)++;
            INFO("serpentine: r%d sweep: %.1f deg in %u ms (%.2f dps) %.2f PSI "
                 "cum %.2fmm", ring + 1, meas_deg, (unsigned)leg_ms,
                 meas_dps, avg_psi, cumulative_depth[ring]);
        }
    }   // leg loop

    chase_motor_apply(&nm, 0, 0);
    chase_motor_apply(&vm, 0, 0);
    chase_motor_teardown(&nm);
    chase_motor_teardown(&vm);

    INFO("serpentine: pass %s after %u ms", ok ? "complete" : "stopped",
         (unsigned)pdTICKS_TO_MS(xTaskGetTickCount() - t_run));
    return ok;
}

// b423: serpentine pass driver -- called from phase_water_zone in place of the
// smooth/gentle pass loop. Builds a skip mask (unwaterable, inner/direct
// rings deferred in Phase A, satisfied rings on later passes), plans one
// serpentine per pass, executes it, and dumps per-ring cumulative depth in
// smooth's catch-cup format. Alternates out->in / in->out per pass so pass
// N+1 starts at the ring where pass N ended. Teardown (valve close, rails,
// aggregate flush, summary) belongs to the caller's shared closeout.
static void water_serpentine_passes(
    const zone_perimeter_t *zone, bool have_zone,
    const float *sector_throw,
    const float *ring_throws, const bool *ring_direct,
    const float *ring_ref_throw, int num_rings,
    const bool *ring_unwaterable,
    float *cumulative_depth,
    float zone_arc_start, float zone_arc_end, float zone_arc_deg,
    float act_max_throw, float act_min_throw, bool have_throw_cal,
    float pressure_scale, float psi_min, float psi_max,
    const speed_map_t *spd, bool have_spd,
    float depth_mm, int passes, float serpentine_dps, bool dry,
    int *ring_sweeps_out, float *run_psi_sum, int *run_psi_n,
    TickType_t t_start)
{
    // b429: speed= is now a manual override (testing); 0/absent means
    // flow-solved per ring in the planner.
    if (serpentine_dps > 0.0f) {
        if (serpentine_dps < 3.0f)  serpentine_dps = 3.0f;
        if (serpentine_dps > 20.0f) serpentine_dps = 20.0f;
    }

    if (dry) passes = 1;   // b429: rehearsal -- nothing credits depth, one
                           // pass of motion is the whole point

    // b429: per-ring valve throw correction, smooth's iterative scheme
    // (blend 0.6 old / 0.4 new of target/actual, clamped 0.5..1.6).
    // Replaces the global pressure_scale that b428 removed: corr learns
    // each ring's real supply behavior from its own measured throw.
    float corr[WATER_RUN_MAX_RINGS];
    for (int i = 0; i < WATER_RUN_MAX_RINGS; i++) corr[i] = 1.0f;

    // b432: inner/direct-valve rings are NOT deferred anymore. The throw cal
    // reaches ~330 mm on this fleet, so nearly every ring is cal-addressable
    // (direct rings use the b382 band-anchored interpolation). The b367
    // seat-dead-band worry is covered structurally: in->out passes open the
    // valve upward from closed through every inner ring's angle (the safe
    // direction), and a dead-band miss on an out->in pass reads ~0 PSI, earns
    // no depth credit, and is simply re-attempted next pass.
    (void)act_min_throw;
    // b434: throw-convergence retire (smooth's b297 insight, bounded). A
    // depth-satisfied ring whose TRUSTWORTHY measured throw is still short
    // of target landed its water inward of where it belongs -- the b432
    // Edge run left the boundary band ~14% light on 4 supply-limited outer
    // rings. Re-fire such rings (corr keeps climbing between passes) up to
    // 3 extra times, then accept. The +/-25% plausibility window keeps the
    // low-PSI inner-ring throw artifact (reads up to 2x) from ever causing
    // re-fire loops; over-throw is not re-fired (beyond +25% it's artifact,
    // within it the water still landed inside the splash band).
    uint8_t throw_retries[WATER_RUN_MAX_RINGS] = {0};
    for (int pass = 0; pass < passes; pass++) {
        bool skip[WATER_RUN_MAX_RINGS];
        int  todo = 0;
        for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
            bool depth_ok = (pass > 0 && cumulative_depth[i] >= depth_mm);
            if (depth_ok && !ring_unwaterable[i]) {
                water_ring_data_t *r = &s_last_water_run.rings[i];
                if (r->throw_mm > 100.0f && r->actual_throw_mm > 100.0f) {
                    float ratio = r->actual_throw_mm / r->throw_mm;
                    if (ratio >= 0.75f && ratio < 0.90f && throw_retries[i] < 3) {
                        throw_retries[i]++;
                        depth_ok = false;   // re-fire for throw convergence
                        INFO("Serpentine ring %d: depth ok but throw %.2fx -- "
                             "re-fire %u/3", i + 1, ratio, throw_retries[i]);
                    }
                }
            }
            skip[i] = ring_unwaterable[i] || depth_ok;
            if (!skip[i]) todo++;
        }
        if (todo == 0) {
            INFO("Serpentine: all rings satisfied after pass %d -- done", pass);
            break;
        }
        bool out_to_in = (pass % 2 == 0);
        int n = serpentine_build_pass_plan(zone, have_zone, sector_throw,
                                      ring_throws, ring_direct, ring_ref_throw,
                                      num_rings, skip, out_to_in,
                                      /*start_cw=*/out_to_in,
                                      zone_arc_start, zone_arc_end, zone_arc_deg,
                                      act_max_throw, have_throw_cal,
                                      pressure_scale, psi_min, psi_max,
                                      spd, have_spd, serpentine_dps, depth_mm, corr);
        if (n == 0) {
            // b434: the only rings left are polygon-clipped to zero width
            // (e.g. the outermost ring on a spike-shaped zone) -- they can
            // never be watered, so this IS completion, not an error.
            INFO("Serpentine: all waterable rings satisfied after pass %d "
                 "(remaining rings clipped to zero by polygon) -- done", pass);
            break;
        }
        if (serpentine_dps > 0.0f)
            INFO("--- Serpentine pass %d: %d legs, %d ring(s), %s, manual %.1f dps%s ---",
                 pass + 1, n, todo, out_to_in ? "out->in" : "in->out",
                 serpentine_dps, dry ? " [DRY]" : "");
        else
            INFO("--- Serpentine pass %d: %d legs, %d ring(s), %s, flow-solved%s ---",
                 pass + 1, n, todo, out_to_in ? "out->in" : "in->out",
                 dry ? " [DRY]" : "");
        s_csv_pass_type = (uint8_t)(pass + 1);
        if (!serpentine_glide_legs(s_serpentine_legs, n, ring_throws, num_rings,
                              cumulative_depth, run_psi_sum, run_psi_n,
                              ring_sweeps_out, t_start, dry)) {
            return;   // aborted/faulted -- shared closeout tears down
        }
        // b429: iterative per-ring throw correction, smooth's scheme verbatim
        // (psi_min gate rejects cold-start under-throw; clamp 0.5..1.6 so
        // persistent under-throw can engage the beyond-cal extrapolation).
        if (!dry) {
            for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                water_ring_data_t *r = &s_last_water_run.rings[i];
                if (skip[i] || r->throw_mm < 1.0f || r->actual_throw_mm < 100.0f)
                    continue;
                bool _under = r->actual_throw_mm < r->throw_mm;
                if (_under && r->avg_psi < psi_min) continue;   // cold-start
                float new_corr = fmaxf(0.5f, fminf(1.6f,
                                       r->throw_mm / r->actual_throw_mm));
                float old_corr = corr[i];
                corr[i] = old_corr * 0.6f + new_corr * 0.4f;
                if (fabsf(corr[i] - 1.0f) > 0.05f)
                    INFO("  Serpentine ring %d corr %.3f->%.3f (%.0f->%.0fmm, psi %.2f)",
                         i + 1, old_corr, corr[i],
                         r->throw_mm, r->actual_throw_mm, r->avg_psi);
            }
            // Pass 0: un-credit rings whose water landed in the wrong ring
            // (smooth's seed gate -- |corr-1| > 0.20 means the deposit
            // geometry was off by more than a ring's worth).
            if (pass == 0) {
                for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                    if (fabsf(corr[i] - 1.0f) > 0.20f && cumulative_depth[i] > 0.0f) {
                        INFO("  Serpentine ring %d: pass 0 depth not credited (corr %.3f)",
                             i + 1, corr[i]);
                        cumulative_depth[i] = 0.0f;
                    }
                }
            }
        }

        // Per-ring cumulative depth dump (catch-cup aid, smooth's format;
        // b429: chunked 12 rings/line -- one 400-byte line truncated at r13).
        for (int i0 = 0; i0 < num_rings && i0 < WATER_RUN_MAX_RINGS; i0 += 12) {
            char _depbuf[260];
            int _off = 0;
            int _hi = i0 + 12;
            for (int i = i0; i < _hi && i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                if (ring_throws[i] < 1.0f) continue;
                int _n = snprintf(_depbuf + _off, sizeof(_depbuf) - _off,
                                  " %d=%.1fm:%.2fmm", i + 1,
                                  ring_throws[i] / 1000.0f, cumulative_depth[i]);
                if (_n < 0 || _n >= (int)(sizeof(_depbuf) - _off)) break;
                _off += _n;
            }
            INFO("Serpentine pass %d ring depths [%d-%d]:%s",
                 pass + 1, i0 + 1, _hi, _depbuf);
        }
    }
}

static void phase_chase_water_zone(void)
{
    int duration_min = s_chase_duration_min;
    if (duration_min < CHASE_MIN_MINUTES) duration_min = CHASE_MIN_MINUTES;
    if (duration_min > CHASE_MAX_MINUTES) duration_min = CHASE_MAX_MINUTES;

    zone_perimeter_t zone = {0};
    if (zone_load_primary(s_water_zone_id, &zone) != ESP_OK || zone.num_points < 3) {
        INFO("Chase: zone %u has no perimeter -- aborting.", (unsigned)s_water_zone_id);
        return;
    }
    zone_sort_walk_order(&zone);

    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) {
        INFO("Chase: no pressure calibration -- aborting.");
        return;
    }

    // --- Zone arc + throw envelope ---
    float min_throw =  1e9f, max_throw = 0.0f;
    for (int i = 0; i < zone.num_points; i++) {
        float t = zone.points[i].throw_mm;
        if (t > 100.0f) {
            if (t < min_throw) min_throw = t;
            if (t > max_throw) max_throw = t;
        }
    }
    if (min_throw > max_throw) {
        min_throw = cal_get_min_throw_mm();
        max_throw = cal_get_max_throw_mm();
    }

    float sb[ZONE_MAX_PERIM_POINTS];
    int n = zone.num_points;
    for (int i = 0; i < n; i++) sb[i] = zone.points[i].nozzle_deg;
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (sb[j] < sb[i]) { float t = sb[i]; sb[i] = sb[j]; sb[j] = t; }
    float max_gap = 0, gap_lo = 0, gap_hi = 360.0f;
    for (int i = 0; i < n; i++) {
        float b0 = sb[i];
        float b1 = (i < n - 1) ? sb[i + 1] : sb[0] + 360.0f;
        if (b1 - b0 > max_gap) { max_gap = b1 - b0; gap_lo = b0; gap_hi = b1; }
    }
    float cpsi_min = cal.pressure_psi[0], cpsi_max = cal.pressure_psi[0];
    for (int i = 1; i < cal.num_points; i++) {
        if (cal.pressure_psi[i] < cpsi_min) cpsi_min = cal.pressure_psi[i];
        if (cal.pressure_psi[i] > cpsi_max) cpsi_max = cal.pressure_psi[i];
    }
    float arc_start, arc_end, arc_deg;
    // b435: only restrict the chase envelope if the gap is genuinely empty --
    // a long straight boundary edge can span a >45deg vertex gap.
    if (max_gap > 45.0f &&
        zone_gap_is_empty(&zone, gap_lo, max_gap, cpsi_min, cpsi_max)) {
        arc_start = fmodf(gap_hi, 360.0f);
        arc_end   = gap_lo;
        arc_deg   = fmodf(arc_end - arc_start + 360.0f, 360.0f);
    } else {
        arc_start = 0.0f; arc_end = 360.0f; arc_deg = 360.0f;
    }
    float arc_center = fmodf(arc_start + arc_deg * 0.5f + 360.0f, 360.0f);

    // Inset the safe envelope from the polygon's literal boundaries so the
    // look-ahead reversal triggers a few degrees / mm before water actually
    // spills over the edge.
    const float ARC_MARGIN_DEG    = 3.0f;
    const float THROW_MARGIN_MM   = 250.0f;
    float arc_safe_lo  = fmodf(arc_start + ARC_MARGIN_DEG + 360.0f, 360.0f);
    float arc_safe_hi  = fmodf(arc_end   - ARC_MARGIN_DEG + 360.0f, 360.0f);
    float arc_safe_span = fmodf(arc_safe_hi - arc_safe_lo + 360.0f, 360.0f);
    float throw_safe_lo = min_throw + THROW_MARGIN_MM;
    float throw_safe_hi = max_throw - THROW_MARGIN_MM;
    if (throw_safe_hi < throw_safe_lo + 200.0f) {
        // very narrow polygon -- use a tighter inset
        throw_safe_lo = min_throw + 100.0f;
        throw_safe_hi = max_throw - 100.0f;
    }

    INFO("=== Chase mode: zone %u for %d min ===",
         (unsigned)s_water_zone_id, duration_min);
    INFO("Chase: arc [%.1f, %.1f] (span %.1f), throw [%.0f, %.0f] mm",
         arc_start, arc_end, arc_deg, min_throw, max_throw);
    INFO("Chase: safe arc [%.1f, %.1f], safe throw [%.0f, %.0f] mm",
         arc_safe_lo, arc_safe_hi, throw_safe_lo, throw_safe_hi);

    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(300));

    s_water_est_min = duration_min;

    // --- Position at chase center before opening the valve ---
    float r_center = (throw_safe_lo + throw_safe_hi) * 0.5f;
    for (int tries = 0; tries < 20; tries++) {
        if (zone_contains_point(&zone, arc_center, r_center)) break;
        r_center = throw_safe_lo + (r_center - throw_safe_lo) * 0.9f;
    }
    float valve_start_deg = cal_throw_to_valve_deg(r_center);
    if (valve_start_deg < VALVE_CAL_START_DEG) valve_start_deg = VALVE_CAL_START_DEG;
    if (valve_start_deg > VALVE_OPEN_DEG)      valve_start_deg = VALVE_OPEN_DEG;

    nozzle_goto(arc_center, 1.5f, 10000, false);
    s_nozzle_last_dir = 1;
    valve_goto_ex(valve_start_deg, 0.8f, 10000, false, 1);
    s_valve_last_dir = 1;
    vTaskDelay(pdMS_TO_TICKS(300));

    // --- Set up persistent PWM for both motors ---
    chase_motor_t nm = {0}, vm = {0};
    if (!chase_motor_setup(&nm, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
        INFO("Chase: nozzle PWM setup failed");
        return;
    }
    if (!chase_motor_setup(&vm, VALVE_PIN_FWD, VALVE_PIN_REV)) {
        INFO("Chase: valve PWM setup failed");
        chase_motor_teardown(&nm);
        return;
    }

    // Slow continuous duty values. Tuned for visually-smooth motion that
    // still reliably overcomes static friction at the start of each leg.
    // Period_ticks=500 -> duty/500 = % of cycle on.
    //   90  -> 18 % -> ~10-15 dps nozzle (depends on supply voltage)
    //  130  -> 26 % -> ~2-4 dps valve (~20 sec to traverse cal range)
    const uint16_t NOZZLE_DUTY     = 90;
    const uint16_t VALVE_DUTY      = 130;
    // Minimum duty used at boundary-approach. The motor still moves at
    // this duty (slower) so encoder progress keeps the lookahead alive.
    // Set ~65% of run duty: enough to maintain motion, not so high that
    // the slowdown is invisible.
    const uint16_t NOZZLE_MIN_DUTY = 60;
    const uint16_t VALVE_MIN_DUTY  = 90;
    // Distance over which the cosine ramp goes from MIN to RUN. Past
    // EASE_DEG from a boundary -> full duty; inside EASE_DEG -> easing.
    const float    NOZZLE_EASE_DEG = 15.0f;
    const float    VALVE_EASE_DEG  = 6.0f;
    // Tick + look-ahead: 50 ms tick, 250 ms look-ahead means we reverse
    // when we'd cross the safe boundary within the next 5 ticks.
    const uint32_t TICK_MS         = 50;
    const float    LOOK_AHEAD_S    = 0.25f;
    // Nominal velocities used for look-ahead prediction. These don't have
    // to be exact -- a slight over-estimate just turns the motor around a
    // bit early, which is the safe direction.
    const float    NOZZLE_LOOKAHEAD_DPS = 18.0f;
    const float    VALVE_LOOKAHEAD_DPS  = 4.0f;   // deg of valve angle / sec
    // After a reversal, ignore further reversals on the same axis for
    // this many ticks. Without this, encoder quantization + look-ahead can
    // toggle direction repeatedly right at the boundary ("buzzing").
    const int      REVERSAL_LOCKOUT_TICKS = 12;   // 600 ms

    int  nm_lockout = 0;
    int  vm_lockout = 0;

    // Initial directions: nozzle CW, valve opening. If currently near the
    // upper boundary of either axis, start in the opposite direction so the
    // first move isn't immediately a reversal.
    uint16_t n_raw = 0, v_raw = 0;
    as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
    as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
    float cur_bearing   = n_raw * (360.0f / 4096.0f);
    float cur_valve_deg = v_raw * (360.0f / 4096.0f);

    // CW = increasing bearing.
    // For zone arcs, distance from current bearing to arc_safe_hi (CW) and
    // arc_safe_lo (CCW) decides initial direction.
    float cw_room  = fmodf(arc_safe_hi - cur_bearing + 360.0f, 360.0f);
    float ccw_room = fmodf(cur_bearing - arc_safe_lo + 360.0f, 360.0f);
    int n_dir = (cw_room >= ccw_room) ? +1 : -1;
    int v_dir = (cur_valve_deg < (VALVE_CAL_START_DEG + VALVE_OPEN_DEG) * 0.5f) ? +1 : -1;

    chase_motor_apply(&nm, n_dir, NOZZLE_DUTY);
    chase_motor_apply(&vm, v_dir, VALVE_DUTY);

    TickType_t t_start    = xTaskGetTickCount();
    uint32_t   total_ms   = (uint32_t)duration_min * 60u * 1000u;
    uint32_t   last_log_s = 0;

    // --- Character state machine ----------------------------------------
    // NORMAL  : the smooth proximity-eased back-and-forth above.
    // SINUSOID: nozzle wiggles in a small arc around a center bearing
    //           while the valve continues its slow open/close. Draws
    //           a sine along the radius as the stream's reach changes.
    // DASH    : nozzle gets boosted duty for a brief sprint across the
    //           zone arc -- triggered when the stream is far out and
    //           close to one of the side edges, so the dash is a quick
    //           crossing toward the far side.
    enum { CS_NORMAL = 0, CS_SINUSOID, CS_DASH } cstate = CS_NORMAL;
    TickType_t cstate_t0       = xTaskGetTickCount();
    uint32_t   cstate_dur_ms   = 0;
    TickType_t last_special_t  = xTaskGetTickCount() - pdMS_TO_TICKS(60000);  // allow special on first eligible tick
    float      sin_center_b    = 0.0f;
    float      sin_amp_deg     = 0.0f;
    uint32_t   sin_half_period = 1500;
    // b315: events spaced ~5-10 s apart so the watcher actually sees them.
    // Per-tick probabilities tuned so the first eligible tick after the
    // cooldown almost always fires (cooldown is the dominant gate).
    const uint32_t SPECIAL_COOLDOWN_MS = 5000;
    // 500 = 100% duty (period_ticks). Motor reaches its full no-load
    // speed; combined with the dash-aware lookahead boost below, the
    // boundary reversal still fires in time.
    const uint16_t DASH_DUTY           = 500;

    while (!s_water_abort) {
        uint32_t elapsed_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_start);
        if (elapsed_ms >= total_ms) break;

        uint32_t remaining_ms = total_ms - elapsed_ms;
        int rem_min = (int)((remaining_ms + 59999u) / 60000u);
        if (rem_min < 1) rem_min = 1;
        s_water_est_min = rem_min;

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        TOUCH_ACTIVITY();

        as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
        as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
        cur_bearing   = n_raw * (360.0f / 4096.0f);
        cur_valve_deg = v_raw * (360.0f / 4096.0f);
        float cur_throw = cal_valve_deg_to_throw_mm(cur_valve_deg);

        // Predict ~LOOK_AHEAD_S ahead at nominal velocity. During DASH the
        // motor is at full duty (~30 dps) so use a longer effective horizon
        // -- otherwise the reversal fires after we've already crossed the
        // safe arc boundary at speed, and the ~470 ms chase_motor_reverse
        // ramp lets the stream spill outside the polygon.
        float n_lookahead_dps = (cstate == CS_DASH) ? 35.0f : NOZZLE_LOOKAHEAD_DPS;
        float n_lookahead_s   = (cstate == CS_DASH) ? 0.45f : LOOK_AHEAD_S;
        float predict_b   = fmodf(cur_bearing
                            + (float)n_dir * n_lookahead_dps * n_lookahead_s
                            + 360.0f, 360.0f);
        float predict_v   = cur_valve_deg
                            + (float)v_dir * VALVE_LOOKAHEAD_DPS * LOOK_AHEAD_S;
        float predict_t   = cal_valve_deg_to_throw_mm(predict_v);

        if (nm_lockout > 0) nm_lockout--;
        if (vm_lockout > 0) vm_lockout--;

        // --- Nozzle boundary check ---
        bool reverse_n = false;
        // Out-of-arc test using safe band. CW distance from arc_safe_lo to
        // predicted bearing must be in [0, arc_safe_span].
        float cw_from_lo = fmodf(predict_b - arc_safe_lo + 360.0f, 360.0f);
        if (cw_from_lo > arc_safe_span) reverse_n = true;
        // PIP test at predicted bearing, current throw (if we know it).
        if (!reverse_n && cur_throw > 100.0f &&
            !zone_contains_point(&zone, predict_b, cur_throw)) {
            reverse_n = true;
        }

        // --- Valve boundary check ---
        bool reverse_v = false;
        if (predict_t < throw_safe_lo || predict_t > throw_safe_hi) reverse_v = true;
        if (predict_v < VALVE_CAL_START_DEG + 1.0f ||
            predict_v > VALVE_OPEN_DEG     - 1.0f)                  reverse_v = true;
        // PIP test at current bearing, predicted throw.
        if (!reverse_v && predict_t > 100.0f &&
            !zone_contains_point(&zone, cur_bearing, predict_t)) {
            reverse_v = true;
        }

        // Normal boundary-driven reversal. In CS_DASH this also serves as
        // the dash's natural exit -- it sprints until it hits the far edge.
        // In CS_SINUSOID we suppress this; the sinusoid's small amplitude
        // keeps the stream well inside the polygon (we verified at trigger
        // time), and the sinusoid controls the nozzle direction itself.
        if (reverse_n && nm_lockout == 0 && cstate != CS_SINUSOID) {
            INFO("  Chase: nozzle reverse at b=%.1f (pred %.1f), dir %d->%d",
                 cur_bearing, predict_b, n_dir, -n_dir);
            chase_motor_reverse(&nm, NOZZLE_DUTY);
            n_dir = nm.dir;
            nm_lockout = REVERSAL_LOCKOUT_TICKS;
            if (cstate == CS_DASH) {
                INFO("  Chase: DASH ended by boundary");
                cstate = CS_NORMAL;
                cstate_t0 = xTaskGetTickCount();
            }
        }
        if (reverse_v && vm_lockout == 0) {
            INFO("  Chase: valve reverse at v=%.1f (pred %.1f), dir %d->%d",
                 cur_valve_deg, predict_v, v_dir, -v_dir);
            chase_motor_reverse(&vm, VALVE_DUTY);
            v_dir = vm.dir;
            vm_lockout = REVERSAL_LOCKOUT_TICKS;
        }

        // --- Character state machine -----------------------------------
        uint32_t cstate_elapsed  = (uint32_t)pdTICKS_TO_MS(
                                       xTaskGetTickCount() - cstate_t0);
        uint32_t since_special   = (uint32_t)pdTICKS_TO_MS(
                                       xTaskGetTickCount() - last_special_t);

        // Time-based exits for special states.
        if (cstate == CS_DASH && cstate_elapsed >= cstate_dur_ms) {
            INFO("  Chase: DASH end");
            cstate = CS_NORMAL;
            cstate_t0 = xTaskGetTickCount();
        }
        if (cstate == CS_SINUSOID && cstate_elapsed >= cstate_dur_ms) {
            INFO("  Chase: SINUSOID end");
            cstate = CS_NORMAL;
            cstate_t0 = xTaskGetTickCount();
        }

        // Maybe trigger a new special state from NORMAL. b315: probabilities
        // boosted so the cooldown floor is the dominant gate -- expect a
        // special event very shortly after each cooldown expires.
        if (cstate == CS_NORMAL && nm_lockout == 0 &&
            since_special >= SPECIAL_COOLDOWN_MS) {
            float throw_span = throw_safe_hi - throw_safe_lo;
            // "Valve fairly open" -- the dash trigger per user spec.
            bool  far_throw  = (cur_throw > throw_safe_lo + throw_span * 0.50f);
            float dist_to_lo = fmodf(cur_bearing - arc_safe_lo + 360.0f, 360.0f);
            float dist_to_hi = fmodf(arc_safe_hi - cur_bearing + 360.0f, 360.0f);
            float side_dist  = fminf(dist_to_lo, dist_to_hi);

            uint32_t r = esp_random();
            // 50/50 between DASH (if eligible) and SINUSOID. ~5 % per tick
            // when eligible means within ~1 s of cooldown expiring there's
            // typically a trigger.
            if (far_throw && (r % 100u) < 5u) {
                // Direction: AWAY from whichever edge is closer.
                int dash_dir = (dist_to_lo < dist_to_hi) ? +1 : -1;
                if (dash_dir != n_dir) {
                    // Fast flip so the lead-in to the dash is snappy.
                    chase_motor_reverse_fast(&nm, NOZZLE_DUTY);
                    n_dir = nm.dir;
                    nm_lockout = REVERSAL_LOCKOUT_TICKS / 3;
                }
                cstate = CS_DASH;
                cstate_dur_ms = 1500u + (esp_random() % 1500u);   // 1.5-3 s
                cstate_t0 = xTaskGetTickCount();
                last_special_t = cstate_t0;
                INFO("  Chase: DASH start dir=%+d dur=%ums (b=%.1f thro=%.0f valve %.1f)",
                     n_dir, (unsigned)cstate_dur_ms, cur_bearing, cur_throw, cur_valve_deg);
            }
            else {
                // SINUSOID amplitude trimmed to whatever fits in the safe arc.
                float amp = 12.0f + (float)((esp_random() >> 4) % 10u);  // 12..21 deg
                float max_amp = fmaxf(6.0f, side_dist - 4.0f);
                if (amp > max_amp) amp = max_amp;
                if ((r % 100u) < 5u && amp >= 6.0f) {
                    cstate = CS_SINUSOID;
                    cstate_dur_ms   = 10000u + (esp_random() % 8000u); // 10-18 s
                    cstate_t0       = xTaskGetTickCount();
                    last_special_t  = cstate_t0;
                    sin_center_b    = cur_bearing;
                    sin_amp_deg     = amp;
                    sin_half_period = 900u + (esp_random() % 600u);    // 0.9-1.5 s
                    INFO("  Chase: SINUSOID start amp=%.0f hp=%ums dur=%ums (c=%.1f valve %.1f)",
                         sin_amp_deg, (unsigned)sin_half_period,
                         (unsigned)cstate_dur_ms, sin_center_b, cur_valve_deg);
                }
            }
        }

        // --- Per-state nozzle motor control ----------------------------
        if (cstate == CS_DASH) {
            // Full-throttle sprint. Override proximity scaling. Boundary
            // reversal (handled above) is the natural exit.
            if (nm_lockout == 0) {
                chase_motor_apply(&nm, n_dir, DASH_DUTY);
            }
        } else if (cstate == CS_SINUSOID) {
            // Velocity follows cos(pi * t / half_period). When cos crosses
            // zero -> reverse direction (fast). Velocity magnitude scales
            // the duty: full at center, decel toward edges -> sin-shaped
            // bearing trajectory.
            float phase   = (float)cstate_elapsed / (float)sin_half_period;
            float vel     = cosf(phase * (float)M_PI);            // -1..1
            int   want_dir = (vel > 0.0f) ? +1 : -1;
            if (want_dir != n_dir && nm_lockout == 0) {
                chase_motor_reverse_fast(&nm, NOZZLE_DUTY);
                n_dir = nm.dir;
                nm_lockout = REVERSAL_LOCKOUT_TICKS / 3;          // ~200 ms
            }
            if (nm_lockout == 0) {
                // |vel| in [0, 1]; min duty floor so the motor doesn't stall
                // at the zero-crossing.
                float mag = fabsf(vel);
                float mix = 0.4f + 0.6f * mag;     // 0.4..1.0
                uint16_t target = (uint16_t)((float)NOZZLE_DUTY * mix);
                if (target < NOZZLE_MIN_DUTY) target = NOZZLE_MIN_DUTY;
                chase_motor_apply(&nm, n_dir, target);
            }
        } else {
            // CS_NORMAL: proximity-based duty scaling (smooth decel into turn).
            if (nm_lockout == 0) {
                float d;
                if (n_dir > 0) {
                    d = fmodf(arc_safe_hi - cur_bearing + 360.0f, 360.0f);
                    if (d > arc_safe_span + 5.0f) d = 0.0f;
                } else {
                    d = fmodf(cur_bearing - arc_safe_lo + 360.0f, 360.0f);
                    if (d > arc_safe_span + 5.0f) d = 0.0f;
                }
                float t = d / NOZZLE_EASE_DEG;
                if (t > 1.0f) t = 1.0f;
                if (t < 0.0f) t = 0.0f;
                float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                uint16_t target = (uint16_t)((float)NOZZLE_MIN_DUTY
                                  + ease * (float)(NOZZLE_DUTY - NOZZLE_MIN_DUTY));
                chase_motor_apply(&nm, n_dir, target);
            }
        }

        // Valve control:
        //   - CS_DASH: HOLD the valve still (duty 0) so throw freezes at the
        //              spot where the sprint started -- visually clearer that
        //              the nozzle is the one moving fast.
        //   - otherwise: normal proximity-eased open/close.
        if (cstate == CS_DASH) {
            chase_motor_apply(&vm, 0, 0);
        } else if (vm_lockout == 0) {
            float d = (v_dir > 0)
                ? (VALVE_OPEN_DEG     - cur_valve_deg)
                : (cur_valve_deg     - VALVE_CAL_START_DEG);
            if (d < 0.0f) d = 0.0f;
            float t = d / VALVE_EASE_DEG;
            if (t > 1.0f) t = 1.0f;
            if (t < 0.0f) t = 0.0f;
            float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
            uint16_t target = (uint16_t)((float)VALVE_MIN_DUTY
                              + ease * (float)(VALVE_DUTY - VALVE_MIN_DUTY));
            chase_motor_apply(&vm, v_dir, target);
        }

        s_nozzle_last_dir = n_dir;
        s_valve_last_dir  = v_dir;

        uint32_t now_s = elapsed_ms / 1000u;
        if (now_s >= last_log_s + 10u) {
            const char *mode_str = (cstate == CS_DASH) ? " DASH" :
                                   (cstate == CS_SINUSOID) ? " SIN" : "";
            INFO("  Chase t=%us b=%.1f(%+d) v=%.1f(%+d) rem=%d min%s",
                 (unsigned)now_s, cur_bearing, n_dir, cur_valve_deg, v_dir,
                 rem_min, mode_str);
            last_log_s = now_s;
        }
    }

    // --- Cleanup ---
    INFO("Chase: shutting down");
    // Ramp nozzle down smoothly.
    if (nm.dir != 0 && nm.duty > 0) {
        for (int i = 4; i >= 0; i--) {
            chase_motor_apply(&nm, nm.dir, (uint16_t)(NOZZLE_DUTY * i / 5));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    chase_motor_apply(&nm, 0, 0);
    // Smoothly close valve via existing closed-loop primitive (it ramps).
    // First disable valve PWM so it doesn't fight valve_goto_ex.
    chase_motor_teardown(&vm);
    chase_motor_teardown(&nm);
    valve_goto_ex(VALVE_CLOSED_DEG, 2.0f, 10000, false, -1);
    s_valve_last_dir = -1;

    s_water_est_min = 0;
    s_eta_anchor_tick = 0;   // b362
    INFO("Chase: complete (%u s elapsed)",
         (unsigned)((xTaskGetTickCount() - t_start) * portTICK_PERIOD_MS / 1000u));
}

// Water just the perimeter boundary of the stored zone.
// Walks each edge of the perimeter polygon in order:
//   - Arc edges: stepped nozzle waypoints (~5 deg) with interpolated valve throw
//   - Radial walls (same bearing, different throw): valve-only adjustment
// This lets the user verify the zone boundary and deliver a light perimeter soak.
static void phase_perim_water(void)
{
    zone_perimeter_t zone = {0};
    if (zone_load_primary(s_water_zone_id, &zone) != ESP_OK || zone.num_points < 3) {
        INFO("No zone perimeter saved -- run 'z' first."); return;
    }
    // Sort into walk order so polygon edges match the shape the user defined.
    zone_sort_walk_order(&zone);
    int n = zone.num_points;

    pressure_map_t cal = {0};
    float psi_max = 6.28f;
    if (cal_load_primary(&cal) == ESP_OK && cal.num_points >= 2)
        psi_max = cal.pressure_psi[cal.num_points - 1];

    // Pre-compute valve_deg for each perimeter point
    float vdeg[ZONE_MAX_PERIM_POINTS];
    for (int i = 0; i < n; i++) {
        float t = (zone.points[i].throw_mm > 100.0f) ? zone.points[i].throw_mm
                : zone.points[i].pressure_psi / psi_max * cal_get_max_throw_mm();
        vdeg[i] = cal_throw_to_valve_deg(t);
        if (vdeg[i] < VALVE_CAL_START_DEG) vdeg[i] = VALVE_CAL_START_DEG;
        if (vdeg[i] > VALVE_OPEN_DEG)      vdeg[i] = VALVE_OPEN_DEG;
    }

    // Nozzle waypoint spacing -- used only for steep variable-throw edges
    const float STEP_DEG = 8.0f;

    // Load speed cal for continuous sweep on constant-throw edges
    speed_map_t spd = {0};
    bool have_spd = (spd_load_primary(&spd) == ESP_OK && spd.num_points >= 2);
    float sweep_dps   = have_spd ? spd.min_continuous_dps : 10.9f;
    uint16_t sweep_duty = (have_spd && spd.num_points >= 1) ? spd.duty[0] : 250;

    // Speed selection: slow uses half the nozzle duty (still above stall) and
    // adds a dwell after each waypoint so the valve has time to settle.
    INFO("  Speed: 1=slow  2=normal (default)");
    int spd_sel = uart_getchar(10000);
    printf("%c\r\n", spd_sel > 0 ? spd_sel : '2');
    bool slow_mode = (spd_sel == '1');
    if (slow_mode) {
        // Floor duty at jog_pulse_duty+20 to avoid stall
        uint16_t floor_duty = have_spd ? (spd.jog_pulse_duty + 20) : 180;
        uint16_t half_duty  = (uint16_t)(sweep_duty * 0.5f);
        sweep_duty = (half_duty > floor_duty) ? half_duty : floor_duty;
        sweep_dps  = sweep_dps * 0.5f;
    }
    uint32_t waypoint_dwell_ms = slow_mode ? 500 : 0;
    TickType_t t_start = xTaskGetTickCount();

    INFO("Watering zone perimeter (%d edges, %s) -- press q to stop.",
         n, slow_mode ? "slow" : "normal");
    adc_setup();
    sensor_rail_on();
    s_water_abort = false;  // clear any stale abort from previous run
    s_water_status_code   = WATER_STATUS_COMPLETED;
    s_water_run_had_flow  = false;
    s_water_no_flow_streak = 0;
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(300));

    // Position to first perimeter point with valve closed, then open
    nozzle_goto(zone.points[0].nozzle_deg, 1.0f, 10000, false);
    valve_goto_ex(vdeg[0], 1.0f, 10000, false, 1);
    if (uart_getchar(0) != 0 || s_water_abort) goto abort;

    // Walk every edge of the perimeter polygon
    for (int i = 0; i < n; i++) {
        if (uart_getchar(0) != 0 || s_water_abort) { INFO("Stopped."); goto abort; }
        TOUCH_ACTIVITY();

        int   j  = (i + 1) % n;
        float b0 = zone.points[i].nozzle_deg;
        float b1 = zone.points[j].nozzle_deg;
        float v0 = vdeg[i];
        float v1 = vdeg[j];

        if (fabsf(b1 - b0) < 0.5f) {
            // Radial wall: same bearing, step valve only (no nozzle move)
            INFO("  Edge %d->%d: radial wall at %.1f deg  valve %.1f->%.1f deg",
                 i+1, j+1, b0, v0, v1);
            valve_goto_ex(v1, 1.0f, 5000, false, v1 > v0 ? 1 : -1);
            continue;
        }

        // Determine shorter path direction (CW or CCW)
        float delta_cw  = fmodf(b1 - b0 + 360.0f, 360.0f);
        float delta_ccw = fmodf(b0 - b1 + 360.0f, 360.0f);
        bool  cw        = (delta_cw <= delta_ccw);
        float arc_deg   = cw ? delta_cw : delta_ccw;

        INFO("  Edge %d->%d: %s  %.1f->%.1f deg  %.1f arc  valve %.1f->%.1f",
             i+1, j+1, cw ? "CW" : "CCW", b0, b1, arc_deg, v0, v1);

        // Choose sweep strategy based on valve change across this arc:
        //   Gradual throw (|v1-v0| < 10 deg): continuous MCPWM sweep.
        //     Set valve to midpoint, let nozzle run freely. Fast, no backlash zigzag.
        //   Steep throw change (|v1-v0| >= 10 deg): waypoint stepping.
        //     Valve updated at each 8-deg nozzle waypoint; uses larger steps than
        //     before to keep moves long enough to avoid CW overshoot accumulation.
        if (fabsf(v1 - v0) < 10.0f && arc_deg >= 15.0f) {
            float mid_v = (v0 + v1) * 0.5f;
            if (mid_v < VALVE_CAL_START_DEG) mid_v = VALVE_CAL_START_DEG;
            if (mid_v > VALVE_OPEN_DEG)      mid_v = VALVE_OPEN_DEG;
            valve_goto_ex(mid_v, 1.0f, 5000, false, 1);
            nozzle_goto(b0, 1.0f, 10000, false);
            if (!nozzle_sweep_continuous(
                    b0, arc_deg, cw, sweep_duty, sweep_dps,
                    t_start, i+1, j+1, (int)(b0 / WATER_SECTOR_DEG),
                    mid_v,
                    (zone.points[i].throw_mm + zone.points[j].throw_mm) * 0.5f)) {
                INFO("Stopped."); goto abort;
            }
        } else {
            // Variable-throw ramp: waypoint stepping
            int steps = (int)ceilf(arc_deg / STEP_DEG);
            if (steps < 1) steps = 1;
            float cur_v = v0;
            for (int s = 1; s <= steps; s++) {
                if (uart_getchar(0) != 0 || s_water_abort) { INFO("Stopped."); goto abort; }
                TOUCH_ACTIVITY();
                float frac = (float)s / (float)steps;
                float tb = cw ? fmodf(b0 + frac * arc_deg, 360.0f)
                              : fmodf(b0 - frac * arc_deg + 360.0f, 360.0f);
                float tv = v0 + frac * (v1 - v0);
                if (tv < VALVE_CAL_START_DEG) tv = VALVE_CAL_START_DEG;
                if (tv > VALVE_OPEN_DEG)      tv = VALVE_OPEN_DEG;
                if (fabsf(tv - cur_v) > 0.5f) {
                    valve_goto_ex(tv, 1.0f, 3000, false, v1 > v0 ? 1 : -1);
                    cur_v = tv;
                }
                nozzle_goto(tb, 0.5f, 5000, false);
                if (waypoint_dwell_ms > 0)
                    vTaskDelay(pdMS_TO_TICKS(waypoint_dwell_ms));
            }
        }
    }

    INFO("Perimeter complete.");

abort:
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);
    motor_rail_off();
    sensor_rail_off();
}

// ─────────────────────────────────────────────────────────────────────────────
// b292: Watering-mode WiFi pause + RAM log buffer
// ─────────────────────────────────────────────────────────────────────────────
// During a smooth/gentle watering run we shut down WiFi entirely
// (esp_wifi_stop) to eliminate the WiFi-PHY <-> SPI-flash-cache race that
// has been crashing the device mid-run since b285. With WiFi off, the PHY
// cannot do beacon/AMPDU/channel-scan work that requires flash access,
// so there's no opportunity for the cache-disable race.
//
// HA loses live observability for the ~20-min run duration. To make crash
// diagnostics still possible, all ESP_LOG output during this window is
// also captured into a 24 KB RAM ring buffer. After the run, WiFi comes
// back up and the buffer is served via /zone/last_log so the user can
// pull the full run's log via HTTP for post-mortem analysis.
//
// Trade-off: HA history shows pre-watering value -> ~20 min gap ->
// post-watering value (which is what the b287 Quiet mode was already
// doing for pressure/battery/throw). Web UI is unreachable during the
// gap. USB serial console (if attached) still gets logs in real time.

#define WATER_LOG_BUFFER_SIZE  24576   // 24 KB, ~500-700 log lines typical

static char     s_watering_log_buf[WATER_LOG_BUFFER_SIZE];
static volatile uint16_t s_watering_log_pos = 0;
static volatile bool     s_watering_log_wrapped = false;
static volatile bool     s_watering_log_active  = false;
static vprintf_like_t    s_prev_vprintf = NULL;

// Custom vprintf hook installed via esp_log_set_vprintf. Captures every
// ESP_LOG* line into our ring buffer (oldest rolls off on overflow) while
// still letting the prior handler print to console/UART. Thread safety:
// log calls can come from any task; we accept occasional torn writes
// rather than locking (worst case is a few mangled chars in the dump).
static int water_log_vprintf(const char *fmt, va_list args)
{
    // Format into a stack buffer once. We must consume args via va_list
    // before passing through, since va_list state is single-use.
    char line[256];
    va_list args_copy;
    va_copy(args_copy, args);
    int len = vsnprintf(line, sizeof(line), fmt, args_copy);
    va_end(args_copy);

    // Pass to previous handler for console output (uses original args).
    int n = s_prev_vprintf ? s_prev_vprintf(fmt, args) : 0;

    if (!s_watering_log_active || len <= 0) return n;
    if (len > (int)sizeof(line) - 1) len = sizeof(line) - 1;

    // Append to ring buffer
    for (int i = 0; i < len; i++) {
        s_watering_log_buf[s_watering_log_pos] = line[i];
        uint16_t next = s_watering_log_pos + 1;
        if (next >= WATER_LOG_BUFFER_SIZE) {
            next = 0;
            s_watering_log_wrapped = true;
        }
        s_watering_log_pos = next;
    }
    return n;
}

static void watering_log_capture_start(void)
{
    s_watering_log_pos = 0;
    s_watering_log_wrapped = false;
    s_watering_log_active = true;
    s_prev_vprintf = esp_log_set_vprintf(water_log_vprintf);
    INFO("Watering RAM log buffer armed (%u bytes)", WATER_LOG_BUFFER_SIZE);
}

static void watering_log_capture_stop(void)
{
    if (!s_watering_log_active) return;
    s_watering_log_active = false;
    if (s_prev_vprintf) {
        esp_log_set_vprintf(s_prev_vprintf);
        s_prev_vprintf = NULL;
    }
    INFO("Watering RAM log buffer captured %u bytes (wrapped=%d)",
         s_watering_log_wrapped ? WATER_LOG_BUFFER_SIZE
                                : (unsigned)s_watering_log_pos,
         s_watering_log_wrapped ? 1 : 0);
}

// b293: Stop WiFi for the duration of a watering run via ESPHome's
// WiFiComponent::disable(), which:
//   - Sets the component state to DISABLED so its loop() won't try to
//     reconnect (the b292 problem -- esp_wifi_stop() alone got reverted
//     by ESPHome's reconnect logic within milliseconds).
//   - Calls wifi_disconnect_() and wifi_mode_(false, false) internally,
//     which is the proper IDF teardown.
// On resume, WiFiComponent::enable() puts the state back to OFF and
// kicks start() which restarts the full STA flow including AP scan,
// auth, and DHCP -- takes ~3-10 sec to fully reconnect.
//
// Bridged via irrigoto_wifi_disable_component / _enable_component in
// irrigoto.cpp because the WiFiComponent class lives in C++ namespace
// esphome::wifi and isn't directly callable from C.
extern void irrigoto_wifi_disable_component(void);  // bridge from irrigoto.cpp
extern void irrigoto_wifi_enable_component(void);   // bridge from irrigoto.cpp

static void watering_wifi_pause(void)
{
    irrigoto_wifi_disable_component();
    INFO("WiFi paused for watering (ESPHome WiFiComponent disabled)");
}

static void watering_wifi_resume(void)
{
    irrigoto_wifi_enable_component();
    INFO("WiFi resume requested (ESPHome WiFiComponent enabled; "
         "STA reconnect typically takes 3-10 sec)");
}

// ─────────────────────────────────────────────────────────────────────────────
// b283: Per-watering pressure trace recorder
// ─────────────────────────────────────────────────────────────────────────────
// Captures (time_s, psi, valve_deg) tuples at ~1 Hz during smooth/gentle runs.
// Used by post-run analysis (in HA, external scripts, or future firmware
// builds b284+) to detect pump-cycle transitions and refine f(valve_deg).
//
// Design:
//   - Fixed RAM ring buffer sized for ~20-min run at 1 Hz (1200 samples).
//   - water_trace_sample(psi) is rate-limited internally; callers can invoke
//     it on every MPRLS read without filling the buffer with redundant data.
//   - valve_deg snapshot is captured at ring boundaries (smooth/gentle hold
//     the valve fixed per-ring), so per-sample valve_deg is the most recent
//     ring-boundary value.
//   - Saved at end of run as a binary file alongside the existing water_*.json
//     for that zone; served via /zone/water_trace?id=N.
//   - Pure data collection: no behavior change, no firmware decisions made
//     from the trace within this build.
#define WATER_TRACE_MAX_SAMPLES   1500     // 25 min @ 1 Hz, generous headroom
// b289: bumped from WATER_RUN_MAX_RINGS (36) to 400. A smooth-mode run can do
// up to GENTLE_MAX_PASSES (~30) passes x 13 rings = 390 ring transitions, each
// emitting a boundary marker. The original 36 was sized for "rings per pass,"
// not "rings across all passes" -- so after pass 3 the buffer filled and all
// subsequent ring boundaries were silently dropped. Samples kept recording
// (separate 1500-entry buffer) but without their ring/valve_deg context, so
// the served trace appeared to "stop watering" at the 3rd-pass mark. RAM cost
// of the bump: 400 x 6 bytes = 2400 bytes (was 216), trivial.
#define WATER_TRACE_MAX_RINGS     400
#define WATER_TRACE_INTERVAL_MS   1000

typedef struct __attribute__((packed)) {
    uint16_t time_s;       // seconds since run start
    uint16_t psi_c;        // pressure x 100
    uint16_t valve_deg_d;  // valve_deg x 10 (the value held when sampled)
} water_trace_sample_t;

typedef struct __attribute__((packed)) {
    uint16_t ring;          // 0-based ring index
    uint16_t time_s_start;  // seconds since run start when this ring began
    uint16_t valve_deg_d;   // valve_deg x 10 held during this ring's sweep
} water_trace_ring_t;

static water_trace_sample_t s_trace_samples[WATER_TRACE_MAX_SAMPLES];
static water_trace_ring_t   s_trace_rings  [WATER_TRACE_MAX_RINGS];
static volatile uint16_t    s_trace_n_samples = 0;
static volatile uint8_t     s_trace_n_rings   = 0;
static volatile bool        s_trace_active    = false;
static          TickType_t  s_trace_t_start         = 0;
static          TickType_t  s_trace_t_last_sample   = 0;
static volatile uint16_t    s_trace_current_valve_d = 0;
static volatile uint16_t    s_trace_zone_id   = 0;

// Reset trace state at the start of a watering run. Call once from
// phase_water_zone() before the pass loop.
static void water_trace_reset(uint16_t zone_id)
{
    s_trace_n_samples = 0;
    s_trace_n_rings   = 0;
    s_trace_t_start   = xTaskGetTickCount();
    s_trace_t_last_sample   = 0;
    s_trace_current_valve_d = 0;
    s_trace_zone_id   = zone_id;
    s_trace_active    = true;
}

// Record a ring boundary -- called when each ring's valve has been positioned.
// Updates current valve_deg so subsequent samples are tagged with it.
static void water_trace_mark_ring(int ring_idx, float valve_deg)
{
    if (!s_trace_active) return;
    uint16_t vd_d = (uint16_t)fminf(valve_deg * 10.0f + 0.5f, 65535.0f);
    s_trace_current_valve_d = vd_d;
    if (s_trace_n_rings >= WATER_TRACE_MAX_RINGS) return;
    uint16_t t_s = (uint16_t)fminf(
        (xTaskGetTickCount() - s_trace_t_start) * portTICK_PERIOD_MS / 1000,
        65535.0f);
    s_trace_rings[s_trace_n_rings++] = (water_trace_ring_t){
        .ring         = (uint16_t)ring_idx,
        .time_s_start = t_s,
        .valve_deg_d  = vd_d,
    };
}

// Append a pressure sample if at least WATER_TRACE_INTERVAL_MS has elapsed
// since the last one. Cheap call -- safe to invoke from any sweep function's
// per-sample loop without flooding the buffer.
static void water_trace_sample(float psi)
{
    if (!s_trace_active) return;
    if (s_trace_n_samples >= WATER_TRACE_MAX_SAMPLES) return;
    TickType_t now = xTaskGetTickCount();
    uint32_t since_ms = (now - s_trace_t_last_sample) * portTICK_PERIOD_MS;
    if (s_trace_t_last_sample != 0 && since_ms < WATER_TRACE_INTERVAL_MS) return;
    s_trace_t_last_sample = now;
    uint16_t t_s = (uint16_t)fminf(
        (now - s_trace_t_start) * portTICK_PERIOD_MS / 1000,
        65535.0f);
    s_trace_samples[s_trace_n_samples++] = (water_trace_sample_t){
        .time_s      = t_s,
        .psi_c       = (uint16_t)fminf(fmaxf(psi, 0.0f) * 100.0f + 0.5f, 65535.0f),
        .valve_deg_d = s_trace_current_valve_d,
    };
}

// b284: Post-run f(valve_deg) refit using the b283 trace buffer.
// Firmware port of tools/analyze_water_trace.py. Two-pass algorithm:
//   Pass 1: per sample, back-compute supply using current f() (from
//           pressure_map_t cal).
//   Pass 2: smooth supply via rolling median over [-W, +W] seconds.
//   Pass 3: per sample, f_obs = psi / smoothed_supply; bin by valve_deg.
//   Output: log per-bin (cal_f, fit_f, diff, n) to console.
//
// b284 LOGS ONLY -- the refined f() is computed but not persisted or used
// in subsequent supply estimates. b285 will add the persistence + use
// path. The log line lets us validate the on-device refit matches what
// the Python analyzer produces, before we trust the refit to act on its
// own output.
#define F_REFIT_BIN_SIZE_DEG     2.0f
#define F_REFIT_SMOOTH_WINDOW_S  30
#define F_REFIT_MIN_SAMPLES_BIN  3
#define F_REFIT_MAX_BINS         40

static void water_trace_refit_f_and_log(void)
{
    uint16_t n = s_trace_n_samples;
    if (n < 10) {
        INFO("f() refit: skipped (trace has only %u samples)", (unsigned)n);
        return;
    }
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) {
        INFO("f() refit: skipped (cal unavailable)");
        return;
    }
    float cal_max_psi = 0.0f;
    for (int i = 0; i < cal.num_points; i++) {
        if (cal.pressure_psi[i] > cal_max_psi) cal_max_psi = cal.pressure_psi[i];
    }
    if (cal_max_psi < 0.5f) {
        INFO("f() refit: skipped (cal max pressure too low)");
        return;
    }

    // Pass 1: per-sample supply estimate using current cal-based f().
    // Static buffers: same size as trace, avoids stack overflow on watering task.
    static float supply_raw     [WATER_TRACE_MAX_SAMPLES];
    static float supply_smoothed[WATER_TRACE_MAX_SAMPLES];
    for (uint16_t i = 0; i < n; i++) {
        float psi = s_trace_samples[i].psi_c       * 0.01f;
        float vd  = s_trace_samples[i].valve_deg_d * 0.1f;
        supply_raw[i] = supply_psi_from_pressure_map(vd, psi);
    }

    // Pass 2: rolling median within [-W, +W] seconds of each sample.
    // Window size is small (sample density ~3s avg -> ~20 samples per window
    // at 30s half-width), so simple selection sort is fine.
    for (uint16_t i = 0; i < n; i++) {
        uint16_t t_i = s_trace_samples[i].time_s;
        float window[64];
        int wn = 0;
        for (uint16_t j = 0; j < n && wn < 64; j++) {
            int dt = (int)s_trace_samples[j].time_s - (int)t_i;
            if (dt < -F_REFIT_SMOOTH_WINDOW_S || dt > F_REFIT_SMOOTH_WINDOW_S) continue;
            if (supply_raw[j] < 0.1f) continue;
            window[wn++] = supply_raw[j];
        }
        if (wn == 0) { supply_smoothed[i] = 0.0f; continue; }
        // Selection sort (small N, no qsort dependency)
        for (int a = 0; a < wn - 1; a++) {
            for (int b = a + 1; b < wn; b++) {
                if (window[b] < window[a]) {
                    float tmp = window[a]; window[a] = window[b]; window[b] = tmp;
                }
            }
        }
        supply_smoothed[i] = window[wn / 2];
    }

    // Pass 3: bin by valve_deg, accumulate f_obs = psi / smoothed_supply.
    struct { float vd_center; float f_sum; uint16_t n; } bins[F_REFIT_MAX_BINS] = {0};
    int n_bins = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (supply_smoothed[i] < 0.5f) continue;
        float psi = s_trace_samples[i].psi_c * 0.01f;
        if (psi < 0.1f) continue;
        float vd  = s_trace_samples[i].valve_deg_d * 0.1f;
        float f_obs = psi / supply_smoothed[i];
        if (f_obs < 0.05f || f_obs > 1.5f) continue;  // implausible -- skip
        float bin_center = roundf(vd / F_REFIT_BIN_SIZE_DEG) * F_REFIT_BIN_SIZE_DEG;
        int slot = -1;
        for (int b = 0; b < n_bins; b++) {
            if (fabsf(bins[b].vd_center - bin_center) < 0.1f) { slot = b; break; }
        }
        if (slot < 0) {
            if (n_bins >= F_REFIT_MAX_BINS) continue;
            slot = n_bins++;
            bins[slot].vd_center = bin_center;
        }
        bins[slot].f_sum += f_obs;
        bins[slot].n++;
    }

    // Sort bins by vd_center ascending (insertion sort, small N)
    for (int a = 1; a < n_bins; a++) {
        int b = a - 1;
        float vd_k = bins[a].vd_center;
        float fs_k = bins[a].f_sum;
        uint16_t n_k = bins[a].n;
        while (b >= 0 && bins[b].vd_center > vd_k) {
            bins[b + 1] = bins[b]; b--;
        }
        bins[b + 1].vd_center = vd_k;
        bins[b + 1].f_sum     = fs_k;
        bins[b + 1].n         = n_k;
    }

    // Helper: cal_f at a query valve_deg via linear interp of cal table
    INFO("f() refit (b284, log only):  bin  cal_f  fit_f  diff       n");
    int reported = 0;
    for (int b = 0; b < n_bins; b++) {
        if (bins[b].n < F_REFIT_MIN_SAMPLES_BIN) continue;
        float vd = bins[b].vd_center;
        float interp_psi = cal.pressure_psi[cal.num_points - 1];
        if (vd <= cal.valve_deg[0])
            interp_psi = cal.pressure_psi[0];
        else if (vd >= cal.valve_deg[cal.num_points - 1])
            interp_psi = cal.pressure_psi[cal.num_points - 1];
        else for (int i = 0; i < cal.num_points - 1; i++) {
            float lo = cal.valve_deg[i], hi = cal.valve_deg[i + 1];
            if (vd >= lo && vd <= hi && hi > lo) {
                float t = (vd - lo) / (hi - lo);
                interp_psi = cal.pressure_psi[i]
                           + t * (cal.pressure_psi[i + 1] - cal.pressure_psi[i]);
                break;
            }
        }
        float cal_f = interp_psi / cal_max_psi;
        float fit_f = bins[b].f_sum / (float)bins[b].n;
        INFO("                          %5.1f  %5.3f  %5.3f  %+5.3f  %5u",
             vd, cal_f, fit_f, fit_f - cal_f, (unsigned)bins[b].n);
        reported++;
    }
    INFO("f() refit: %d/%d bins reported (>=%d samples each, %u total samples)",
         reported, n_bins, F_REFIT_MIN_SAMPLES_BIN, (unsigned)n);
}

// Save the trace as CSV to LittleFS at /lfs/water/water_NNN_trace.csv.
// Returns true on success. Logs sample/ring counts regardless.
// b286: Trace is now RAM-only. Writing the ~10KB CSV to LittleFS at
// run-end was contributing to a flash-cache race that crashed the device
// (PC in ROM, BT through spi_flash_disable_interrupts_caches + lfs_*).
// The trace is a pure diagnostic -- not needed for normal watering -- so
// we keep the in-RAM buffer and serve it on demand via /zone/water_trace.
// User pulls the trace via HTTP within minutes of run-end if they want it
// for off-device analysis (analyze_water_trace.py). After the next
// watering, the buffer is overwritten. Trace is lost on reboot, which is
// acceptable for a diagnostic.
//
// The previously-used /lfs/water/water_NNN_trace.csv files (from b283-b285)
// are now stale; left in place so old downloads remain valid, but no longer
// produced. Could be cleaned up manually if storage pressure mounts.
static bool water_trace_save(uint16_t zone_id)
{
    (void)zone_id;  // RAM-only -- no per-zone file naming needed
    if (!s_trace_active) return false;
    s_trace_active = false;
    INFO("Pressure trace: %u samples, %u ring boundaries, %u total seconds "
         "(RAM-only -- fetch via /zone/water_trace before next run)",
         (unsigned)s_trace_n_samples, (unsigned)s_trace_n_rings,
         s_trace_n_samples > 0
            ? (unsigned)s_trace_samples[s_trace_n_samples - 1].time_s : 0u);
    return true;
}

static void phase_water_zone(void)
{
    STEP("Water Zone");

    // --- Load calibration ---
    pressure_map_t cal = {0};
    if (cal_load_primary(&cal) != ESP_OK || cal.num_points < 2) {
        INFO("No pressure calibration -- run x first."); return;
    }
    float psi_min = cal.pressure_psi[0], psi_max = cal.pressure_psi[0];
    for (int i = 1; i < cal.num_points; i++) {
        if (cal.pressure_psi[i] < psi_min) psi_min = cal.pressure_psi[i];
        if (cal.pressure_psi[i] > psi_max) psi_max = cal.pressure_psi[i];
    }

    float act_max_throw = cal_get_max_throw_mm();
    bool  have_throw_cal = (act_max_throw > 1000.0f);

    // --- Load speed calibration ---
    speed_map_t spd = {0};
    bool have_spd = (spd_load_primary(&spd) == ESP_OK && spd.num_points > 0);

    // --- Load zone perimeter ---
    zone_perimeter_t zone = {0};
    bool have_zone = (zone_load_primary(s_water_zone_id, &zone) == ESP_OK && zone.num_points >= 2);
    if (have_zone) zone_sort_walk_order(&zone);
    INFO("Firmware build: %d", FW_BUILD);
    if (have_zone) INFO("Zone perimeter: %d points.", zone.num_points);
    else           INFO("No zone defined -- watering full circle at max throw.");
    // Use perimeter inner boundary as min throw; falls back to cal table minimum.
    float act_min_throw = have_zone ? zone_get_min_throw_mm(&zone)  // const ok
                                    : cal_get_min_throw_mm();

    if (!have_throw_cal) {
        INFO("WARNING: No throw_mm data in cal table.");
        INFO("Run pressure scan 'x' and enter max throw distance for best results.");
        INFO("Continuing with pressure-only mode (less accurate valve positioning).");
    }

    // b381: arm the RAM log capture HERE (moved up from the smooth/gentle
    // block below) so the buffer catches the ring-generation decisions --
    // "Zone throw", "Rings: N (..outer to ..inner)", and the inner-ring
    // summaries -- which print before mode selection. /zone/last_log was
    // missing all of them because capture used to arm after those lines.
    // Armed unconditionally for every run; the hook is a cheap RAM mirror.
    watering_log_capture_start();

    // --- Sample perimeter throw at each sector ---
    float sector_throw[WATER_SECTORS];
    float max_throw = 0.0f, min_throw = act_max_throw;
    for (int s = 0; s < WATER_SECTORS; s++) {
        float bearing = s * WATER_SECTOR_DEG;
        sector_throw[s] = water_perimeter_throw(&zone, bearing, psi_min, psi_max);
        if (sector_throw[s] > max_throw) max_throw = sector_throw[s];
        if (sector_throw[s] < min_throw) min_throw = sector_throw[s];
    }
    if (min_throw < act_min_throw) min_throw = act_min_throw;

    INFO("Zone throw: %.0fmm (%.1fft) to %.0fmm (%.1fft)",
         min_throw, min_throw/304.8f, max_throw, max_throw/304.8f);

    // Compute exact arc boundary angles from zone gap analysis.
    // These replace sector-quantized boundaries giving ~3 deg more accuracy.
    float zone_arc_start = 0.0f;    // CW sweep start bearing (left zone edge)
    float zone_arc_end   = 360.0f;  // CW sweep end bearing (right zone edge)
    float zone_arc_deg   = 360.0f;  // total active arc (degrees)
    if (have_zone && zone.num_points >= 2) {
        float sb[ZONE_MAX_PERIM_POINTS];
        for (int i = 0; i < zone.num_points; i++) sb[i] = zone.points[i].nozzle_deg;
        for (int i = 0; i < zone.num_points-1; i++)
            for (int j = i+1; j < zone.num_points; j++)
                if (sb[j] < sb[i]) { float t=sb[i]; sb[i]=sb[j]; sb[j]=t; }
        float max_gap=0, gap_lo_z=0, gap_hi_z=360;
        for (int i = 0; i < zone.num_points; i++) {
            float b0=sb[i], b1=(i<zone.num_points-1)?sb[i+1]:sb[0]+360.0f;
            if (b1-b0>max_gap){max_gap=b1-b0;gap_lo_z=b0;gap_hi_z=b1;}
        }
        // b435: only restrict the sweep arc if the gap is genuinely empty --
        // a long straight boundary edge can span a >45deg vertex gap.
        if (max_gap > 45.0f &&
            zone_gap_is_empty(&zone, gap_lo_z, max_gap, psi_min, psi_max)) {
            zone_arc_start = fmodf(gap_hi_z, 360.0f); // left zone edge
            zone_arc_end   = gap_lo_z;                // right zone edge
            zone_arc_deg   = fmodf(gap_lo_z - gap_hi_z + 360.0f, 360.0f);
            INFO("Zone arc: %.1f deg to %.1f deg (%.1f deg span)",
                 zone_arc_start, zone_arc_end, zone_arc_deg);
        }
    }
    // Sprinkler-in-center zones: gap in walk bearings is just the start/end
    // of the walk path, not a real exclusion. Force full 360 deg sweep.
    if (have_zone && max_throw < act_min_throw) {
        zone_arc_start = 0.0f;
        zone_arc_end   = 360.0f;
        zone_arc_deg   = 360.0f;
        INFO("Sprinkler-in-center zone: forcing full 360 deg arc");
    }

    // --- Build ring list with proportional spacing ---
    float ring_throws[WATER_MAX_RINGS_CAL];
    bool  ring_direct[WATER_MAX_RINGS_CAL];    // true = use walk valve_deg, skip PID
    float ring_ref_throw[WATER_MAX_RINGS_CAL]; // reference throw for direct-valve interp
    int   num_rings = 0;
    {
        float t = max_throw;
        while (t >= act_min_throw && num_rings < WATER_MAX_RINGS_CAL) {
            ring_throws[num_rings]    = t;
            ring_direct[num_rings]    = false;
            ring_ref_throw[num_rings] = max_throw;
            num_rings++;
            float sp = WATER_RING_SPACING * (t / act_max_throw);
            if (sp < WATER_MIN_RING_SPACING) sp = WATER_MIN_RING_SPACING;
            t -= sp;
        }
    }
    // If the zone boundary is below the cal minimum throw (e.g. sprinkler-in-center
    // zones), sweep rings from zone_max_throw down toward the sprinkler using the
    // walk valve_deg as the reference and interpolating linearly toward closed.
    // No PID -- just open the valve to the proportional angle for each ring.
    if (have_zone && max_throw < act_min_throw) {
        float walk_valve = zone_avg_valve_deg(&zone);
        if (walk_valve > 0 && max_throw > 10.0f) {
            float t = max_throw;
            int added = 0;
            while (t > WATER_MIN_THROW_MM && num_rings < WATER_MAX_RINGS_CAL) {
                ring_throws[num_rings]    = t;
                ring_direct[num_rings]    = true;
                ring_ref_throw[num_rings] = max_throw;
                num_rings++;
                added++;
                float sp = WATER_RING_SPACING * (t / act_max_throw);
                if (sp < WATER_MIN_RING_SPACING) sp = WATER_MIN_RING_SPACING;
                t -= sp;
            }
            INFO("Zone boundary (%.0fmm/%.1fft) below cal min -- added %d direct-valve rings (walk angle %.1f deg, interpolated inward).",
                 max_throw, max_throw/304.8f, added, walk_valve);
        }
    }
    // Sprinkler-inside-polygon: origin is inside zone polygon but max_throw is within
    // cal range. The bearing-gap arc detection incorrectly restricts coverage to the
    // walk-bearing arc; force full 360 deg so the polygon boundary (via sector_throw
    // pip logic) determines coverage instead. Outer rings at large throw are still
    // naturally clipped because sector_throw is small in interior-only directions.
    if (have_zone && max_throw >= act_min_throw && zone_origin_inside(&zone)) {
        zone_arc_start = 0.0f;
        zone_arc_end   = 360.0f;
        zone_arc_deg   = 360.0f;
        INFO("Origin inside polygon: forcing full 360 deg arc");
    }
    // Inner rings for sprinkler-inside-polygon zones: normal rings stop at act_min_throw;
    // add direct-valve inner rings from there down toward the sprinkler.
    if (have_zone && max_throw >= act_min_throw && zone_origin_inside(&zone)) {
        float ref_valve = zone_avg_valve_deg(&zone);
        // Inner rings between act_min_throw and the cal-min throw are still
        // within the valve->throw calibration range, so they can use the same
        // PID/cal valve control as normal rings (ring_direct=false). Only rings
        // below cal-min throw fall back to direct-valve interpolation -- the cal
        // table can't map a throw it never measured. Previously ALL inner rings
        // were forced direct, and the linear direct-valve formula collapsed the
        // valve below the cal-start (flow) angle, so every inner ring landed at
        // VALVE_CLOSED_DEG and produced no water (the dry center hole).
        float cal_min_throw = cal_get_min_throw_mm();
        if (ref_valve > 0.0f && act_min_throw > WATER_MIN_THROW_MM) {
            float sp = WATER_RING_SPACING * (act_min_throw / act_max_throw);
            if (sp < WATER_MIN_RING_SPACING) sp = WATER_MIN_RING_SPACING;
            float t = act_min_throw - sp;
            int added = 0, added_cal = 0;
            while (t > WATER_MIN_THROW_MM && num_rings < WATER_MAX_RINGS_CAL) {
                bool below_cal = (t < cal_min_throw);
                ring_throws[num_rings]    = t;
                ring_direct[num_rings]    = below_cal;
                ring_ref_throw[num_rings] = below_cal ? cal_min_throw : 0.0f;
                num_rings++;
                added++;
                if (!below_cal) added_cal++;
                sp = WATER_RING_SPACING * (t / act_max_throw);
                if (sp < WATER_MIN_RING_SPACING) sp = WATER_MIN_RING_SPACING;
                t -= sp;
            }
            INFO("Origin inside polygon: added %d inner rings (%d cal, %d direct) "
                 "below %.0fmm (cal-min %.0fmm, ref valve %.1f deg).",
                 added, added_cal, added - added_cal, act_min_throw,
                 cal_min_throw, ref_valve);
        }
    }
    // Zero sector_throw for sectors outside the zone arc. For sprinkler-inside-polygon
    // zones, all bearings have non-zero sector_throw (polygon wraps around origin).
    // Without zeroing, inner rings would spray through bearings outside the zone arc.
    if (zone_arc_deg < 359.0f) {
        for (int s = 0; s < WATER_SECTORS; s++) {
            float sb       = (float)s * WATER_SECTOR_DEG;
            float cw_dist  = fmodf(sb - zone_arc_start + 360.0f, 360.0f);
            if (cw_dist > zone_arc_deg + WATER_SECTOR_DEG * 0.5f)
                sector_throw[s] = 0.0f;
        }
    }
    if (num_rings < 1) {
        INFO("ERROR: No rings generated (max_throw=%.0fmm act_min=%.0fmm). Aborting.", max_throw, act_min_throw);
        s_web_water_mode = 0;
        return;
    }
    INFO("Rings: %d  (%.0fft outer to %.0fft inner)",
         num_rings, ring_throws[0]/304.8f, ring_throws[num_rings-1]/304.8f);

    // --- Mode selection ---
    INFO("Select mode:");
    INFO("  1  Water 1/8 inch (1 pass)");
    INFO("  2  Water 1/4 inch (1 pass)");
    INFO("  3  Water 1/8 inch x2 passes");
    INFO("  4  Water 1/4 inch x2 passes");
    INFO("  5  Gentle 1/8 inch (seed-safe, continuous, up to %d adaptive passes)", GENTLE_MAX_PASSES);
    INFO("  6  Gentle 1/4 inch (seed-safe, continuous, up to %d adaptive passes)", GENTLE_MAX_PASSES);
    INFO("  s  Smooth 1/8 inch (open-loop, valve steps at reversals, 1 pass)");
    INFO("  d  Demo (max speed)");
    INFO("  p  Perimeter only");
    INFO("  q  Cancel");
    INFO(">>> Press q at any time to stop <<<");

    s_water_abort = false;  // clear stale abort from previous run or web cancel
    s_water_status_code   = WATER_STATUS_COMPLETED;
    s_water_run_had_flow  = false;
    s_water_no_flow_streak = 0;
    // Web-triggered: s_web_water_mode already set, skip interactive prompt
    int sel;
    if (s_web_water_mode > 0) {
        // 0=idle, 1-4=metered, 5-6=gentle, 7=smooth, 8=serpentine (b423), 99=demo
        sel = (s_web_water_mode == 99) ? 'd' :
              (s_web_water_mode == 8)  ? 'n' :
              (s_web_water_mode == 7)  ? 's' : ('0' + s_web_water_mode);
        INFO("Web mode: %c", sel);
    } else {
        sel = uart_getchar(30000);
        printf("%c\r\n", sel);
        if (sel == 'q' || sel == 'Q' || sel == 0) { INFO("Cancelled."); return; }
        // Flush any trailing CR/LF left in the UART buffer by the Enter key
        { char _c; while ((_c = uart_getchar(0)) != 0 && (_c == '\r' || _c == '\n')) {} }
    }

    if (sel == 'p' || sel == 'P') { phase_perim_water(); return; }
    bool  demo_mode   = (sel == 'd' || sel == 'D');
    bool  gentle_mode = (sel == '5' || sel == '6');
    bool  smooth_mode = (sel == 's' || sel == 'S');
    bool  serpentine_mode  = (sel == 'n' || sel == 'N');   // b423: web-only mode 8
    // b423: serpentine params, consumed once like depth_eighths below.
    float serpentine_dps = s_web_serpentine_dps; s_web_serpentine_dps = 0.0f;
    bool  serpentine_dry = s_web_serpentine_dry; s_web_serpentine_dry = false;
    // Depth target in eighths of an inch (1..8 = 1/8".. 1"), supplied by the
    // web/schedule path and consumed once here. 0 = legacy digit-encoded depth.
    // Pulse repeats the base 1/8" pass N times; gentle/smooth target the same
    // depth_mm with their own adaptive multipass (so the heatmap's expected
    // tracks N x 1/8" automatically).
    int   depth8      = s_web_water_depth_eighths;
    s_web_water_depth_eighths = 0;
    bool  use_eighths = (depth8 >= 1 && depth8 <= 8) && !demo_mode;
    float depth_mm    = use_eighths ? (float)depth8 * 3.175f :
                        (sel == '2' || sel == '4' || sel == '6') ? 6.35f : 3.175f;
    int   passes      = gentle_mode ? GENTLE_MAX_PASSES :
                        use_eighths  ? depth8 :                 // pulse: N x 1/8"
                        (sel == '3' || sel == '4') ? 2 : 1;
    if (demo_mode)   { depth_mm = 0.0f; passes = 1; }
    if (smooth_mode) { passes = 30; } // adaptive; depth_mm target set above
    if (serpentine_mode)  { passes = 30; } // b429: adaptive, exits when all rings satisfied

    // b283: arm pressure trace recorder for smooth/gentle runs only (pulse
    // mode's per-arc valve PID would make valve_deg snapshots meaningless).
    // Demo mode also skipped -- no real watering, nothing to characterize.
    if ((smooth_mode || gentle_mode || serpentine_mode) && !demo_mode) {
        water_trace_reset(s_water_zone_id);
    }

    // b287: Enter "Watering Quiet" mode for smooth/gentle runs. Throttles
    // ESPHome's high-rate sensor publishes (pressure/battery/throw/cal)
    // while we're driving motors and watching MPRLS at high rate, so the
    // WiFi/NVS-write pipe stays calm. Progress sensors (watering, status,
    // zone) continue to publish at full rate so HA still sees the run.
    // Also bump this task's priority from 10 -> 15 to keep tighter
    // contiguous CPU during the watering loop -- doesn't fix the flash/WiFi
    // race directly but reduces the number of moments where the two
    // subsystems happen to interleave.
    if ((smooth_mode || gentle_mode || serpentine_mode) && !demo_mode) {
        s_watering_quiet = true;
        UBaseType_t _old_prio = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, 15);
        INFO("Watering Quiet ON: throttling sensor publishes; "
             "task priority %u -> 15 for the run",
             (unsigned)_old_prio);
        // b292: arm RAM log capture. b381: moved earlier (before ring
        // generation) so the buffer captures the ring-gen decision lines;
        // re-arming here would wipe them, so the call is gone from this block.
        // b292/b293: previously called watering_wifi_pause() here to shut
        // WiFi down during the run.
        // b295: removed. b294 (moving water_task to PRO_CPU so it shares
        // a core with ESPHome and WiFi) eliminated the cross-core cache-
        // disable race that crashed the device. With that fix, WiFi can
        // stay up during watering -- HA keeps live observability of the
        // watering binary, status text, and zone name (b287 Quiet still
        // throttles the high-rate numeric publishes to keep history clean).
        // If we discover a mid-run crash returns, the pause is one line
        // to re-enable here.
    }

    // Seed the UI estimate from the last completed run for this zone before the
    // ~10s pressure-check valve operation runs.  total_time_s == 0 on older runs
    // or aborted runs, so the fallback (13 min placeholder) still applies then.
    // Adding 5% accounts for day-to-day supply pressure variation.
    //
    // b298: the history-seed value is now retained through the run. Previously
    // the model estimator at the bottom of this block unconditionally
    // overwrote it -- but the model assumes every pass sweeps every ring,
    // which is true only for pass 0; later passes only fire rings that still
    // have deficit, so smooth-mode totals were 3-4x overestimated (e.g. N2
    // showed 130 min when actual is ~30 min). When we have history, history
    // wins. pressure_scale (computed below) refines it for today's supply.
    bool history_seed_valid = false;
    float history_seed_total_s = 0.0f;
    if (!demo_mode && storage_ready()) {
        water_run_t _prev; memset(&_prev, 0, sizeof(_prev));
        if (storage_water_load(s_water_zone_id, &_prev) == ESP_OK
                && _prev.total_time_s > 60.0f) {
            history_seed_total_s = _prev.total_time_s;
            history_seed_valid = true;
            s_water_est_min = (int)(_prev.total_time_s * 1.05f / 60.0f) + 1;
            // b362: arm tick-down anchor so HA "minutes remaining" decreases
            // between the bigger pass-boundary refreshes below.
            s_eta_anchor_secs = _prev.total_time_s * 1.05f;
            s_eta_anchor_tick = xTaskGetTickCount();
            INFO("Est from last run: %d min (prev %.0fs +5%%)",
                 s_water_est_min, _prev.total_time_s);
        }
    }

    // Power on early so we can measure live supply pressure before the estimate.
    adc_setup();
    sensor_rail_on();
    motor_rail_on();
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 12000, false);
    s_valve_last_dir  = -1;
    s_nozzle_last_dir = 0;

    // b363: aim the nozzle into the zone before the run-start pressure check.
    // The check opens the valve to full for ~12s; without repositioning, that
    // spray went in whatever direction the nozzle was left from the previous
    // run (could be the house, driveway, etc). Pick the bearing with the
    // largest sector_throw -- the polygon's deepest radial extent and the
    // direction most likely to keep the full-open spray inside the zone (or
    // at worst at its outer edge). Skip when no zone is defined: the user
    // is then operating without a polygon and we have no notion of "inside."
    // b365: also remember the aim bearing so the pressure-check loop can
    // waggle the nozzle around it instead of hammering one spot.
    float aim_deg            = -1.0f;
    float waggle_half_swing  = 0.0f;
    if (have_zone) {
        float aim_throw = 0.0f;
        for (int s = 0; s < WATER_SECTORS; s++) {
            if (sector_throw[s] > aim_throw) {
                aim_throw = sector_throw[s];
                aim_deg   = ((float)s + 0.5f) * WATER_SECTOR_DEG;
            }
        }
        if (aim_deg >= 0.0f) {
            // Waggle range: nominally ±20°, but capped at 25% of the zone's
            // active arc so we don't waggle outside the polygon on narrow
            // zones. For sprinkler-inside zones (zone_arc_deg=360) this stays
            // at 20°.
            waggle_half_swing = fminf(20.0f, zone_arc_deg * 0.25f);
            INFO("Pre-pressure-check: aiming nozzle to %.1f deg "
                 "(deepest polygon extent %.0fmm); waggle range +/-%.1f deg",
                 aim_deg, aim_throw, waggle_half_swing);
            nozzle_goto(aim_deg, 2.0f, 12000, false);
            s_nozzle_last_dir = 0;
        }
    }

    // Live supply pressure check: open valve to full and read actual PSI for
    // ~12s, then close. Well-and-tank systems cycle in 1-5 min (pump on at
    // 45 PSI, off at 60 PSI for typical setups), so a 600ms snapshot lands
    // on one arbitrary moment in the cycle and locks pressure_scale to it
    // for the entire run. b361: sample for ~12s (120 reads at 10Hz) to get
    // a cycle mean, plus min/max so the log surfaces today's cycle range.
    // 12s at full open drains ~0.7 gal -- 3.4% of a 20 gal pressure tank,
    // negligible. The smooth-mode per-ring iterative valve_corr (b297) and
    // the b296 supply-aware scheduler handle the moment-to-moment cycling
    // during the run itself; pressure_scale just sets the pass-0 baseline
    // and the time-estimate scaling.
    // b365: during the sample, drive the nozzle smoothly back and forth
    // around aim_deg so the 12s of full-open spray spreads over a ~40deg
    // arc instead of hammering one spot. Encoder-based reversal at swing
    // endpoints, plus a 3s safety fallback in case encoder reads fail.
    float pressure_scale = 1.0f;
    // b424: serpentine dry rehearsal must not open the valve -- skip the 12s
    // full-open supply check (pressure_scale stays 1.0; no water flows).
    if (!demo_mode && !serpentine_dry && psi_max > 0.5f) {
        valve_goto(VALVE_OPEN_DEG, 1.0f, 8000, false);
        vTaskDelay(pdMS_TO_TICKS(1500));   // settle after valve open

        // b365: spin up the waggle motor. NOZZLE_DUTY=90 matches chase mode --
        // smooth at typical loads, low enough that reversals are quick. Setup
        // failure (e.g. mcpwm exhausted) silently degrades to the b364
        // hold-still behavior.
        const uint16_t WAGGLE_DUTY        = 90;
        const uint32_t WAGGLE_FALLBACK_MS = 3000;
        chase_motor_t waggle_m = {0};
        bool       waggle_active = false;
        int        waggle_dir    = +1;
        TickType_t waggle_last_reverse_tick = xTaskGetTickCount();
        if (aim_deg >= 0.0f && waggle_half_swing > 0.5f) {
            if (chase_motor_setup(&waggle_m, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
                chase_motor_apply(&waggle_m, waggle_dir, WAGGLE_DUTY);
                waggle_active = true;
                INFO("Waggle ON: %.1f deg +/-%.1f, duty %u",
                     aim_deg, waggle_half_swing, (unsigned)WAGGLE_DUTY);
            } else {
                INFO("Waggle setup failed -- holding aim bearing for read");
            }
        }

        const int      PRES_SAMPLE_N  = 120;
        const uint32_t PRES_SAMPLE_DT = 100;   // ms between reads -> 12s total
        float _psum = 0.0f;
        float _pmin = 1e9f, _pmax = -1e9f;
        int   _pn = 0;
        int   _waggle_reversals = 0;
        for (int _i = 0; _i < PRES_SAMPLE_N; _i++) {
            float _p = 0.0f;
            if (mprls_read(&_p) && _p > 0.1f) {
                _psum += _p;
                if (_p < _pmin) _pmin = _p;
                if (_p > _pmax) _pmax = _p;
                _pn++;
            }

            // Waggle reversal: at swing endpoint, or after fallback timeout
            // if encoder reads aren't triggering one.
            if (waggle_active) {
                bool reverse = false;
                uint16_t n_raw = 0;
                if (as5600_read(ADDR_AS5600, &n_raw, NULL, NULL)) {
                    float cur_deg = n_raw * (360.0f / 4096.0f);
                    float delta   = cur_deg - aim_deg;
                    if (delta > 180.0f)        delta -= 360.0f;
                    else if (delta < -180.0f)  delta += 360.0f;
                    if (waggle_dir > 0 && delta >=  waggle_half_swing) reverse = true;
                    if (waggle_dir < 0 && delta <= -waggle_half_swing) reverse = true;
                }
                uint32_t since_rev_ms =
                    (uint32_t)((xTaskGetTickCount() - waggle_last_reverse_tick)
                               * portTICK_PERIOD_MS);
                if (since_rev_ms > WAGGLE_FALLBACK_MS) reverse = true;
                if (reverse) {
                    chase_motor_reverse_fast(&waggle_m, WAGGLE_DUTY);
                    waggle_dir = -waggle_dir;
                    waggle_last_reverse_tick = xTaskGetTickCount();
                    _waggle_reversals++;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(PRES_SAMPLE_DT));
            TOUCH_ACTIVITY();   // keep inactivity sleep at bay
            // Respect web/UART cancel during the longer sample window.
            if (uart_getchar(0) != 0 || s_water_abort) {
                s_water_abort = true;
                break;
            }
        }

        if (waggle_active) {
            chase_motor_apply(&waggle_m, 0, 0);
            chase_motor_teardown(&waggle_m);
            s_nozzle_last_dir = 0;
            INFO("Waggle OFF (%d reversals during sample)", _waggle_reversals);
        }

        valve_goto(VALVE_CLOSED_DEG, 2.0f, 8000, false);
        s_valve_last_dir = -1;
        if (_pn >= 10) {
            float live_psi = _psum / _pn;
            float range    = _pmax - _pmin;
            pressure_scale = fmaxf(0.5f, fminf(1.5f, live_psi / psi_max));
            INFO("Supply: %.2f PSI mean (%.2f-%.2f over %.1fs, range %.2f), "
                 "cal max %.2f PSI, scale %.2fx",
                 live_psi, _pmin, _pmax,
                 _pn * PRES_SAMPLE_DT / 1000.0f, range,
                 psi_max, pressure_scale);
            if (range > 1.0f)
                INFO("Supply cycling detected (range %.2f PSI / %.0f%% of mean) "
                     "-- smooth-mode per-ring corr will track per-cycle.",
                     range, range / fmaxf(live_psi, 0.1f) * 100.0f);
        } else {
            INFO("Supply pressure read failed -- using calibrated estimate");
        }
    }

    // --- Pre-compute time estimate for web UI display ---
    // Flow is scaled by live pressure ratio so estimate reflects today's supply.
    // b298: if we have a history seed (previous run's total_time_s), prefer
    // that over the model estimate. The model assumes every pass sweeps every
    // ring; smooth-mode reality is later passes fire only rings with deficit,
    // which the per-pass adaptive estimate at end-of-pass discovers but the
    // pre-run model can't predict. History captures the truth in one number.
    if (history_seed_valid) {
        // Apply pressure_scale: lower supply today => proportionally longer.
        // pressure_scale is clamped to [0.5, 1.5] above, so adjustment is bounded.
        float scale_adj = (pressure_scale > 0.1f) ? (1.0f / pressure_scale) : 1.0f;
        float est_s = history_seed_total_s * 1.05f * scale_adj;
        s_water_est_min = (int)(est_s / 60.0f) + 1;
        s_eta_anchor_secs = est_s;                      // b362
        s_eta_anchor_tick = xTaskGetTickCount();
        INFO("Estimated watering time (history-based): %d min "
             "(prev %.0fs * 1.05 * %.2f pressure-adj)",
             s_water_est_min, history_seed_total_s, scale_adj);
    } else if (!demo_mode) {
        float est_sweep = 0.0f;    // per-pass ring sweep time (continuous-sweep modes)
        float est_overhead = 0.0f; // per-pass inter-ring overhead
        float _arc_frac = zone_arc_deg / 360.0f;
        // Smooth/gentle: accumulate flow-model depth/pass to derive a realistic
        // pass count estimate instead of using the adaptive cap (which would
        // wildly overstate the time). pressure_scale adjusts calibrated PSI to
        // today's supply.
        float smooth_est_dp_sum = 0.0f; int smooth_est_dp_n = 0;
        for (int r = 0; r < num_rings; r++) {
            if (smooth_mode || gentle_mode) {
                // Estimator dps matches the actual run's flow-model formula
                // (see nozzle_dps calc ~line 6722). Smooth's per_pass_depth=depth_mm
                // makes the unclamped dps tiny, so it pins to min_continuous_dps
                // (the old magic 13 dps). Gentle's per_pass_depth is ~5× shallower,
                // so its dps lands ~5× higher — and the old hardcoded 13 was wrong
                // by that factor for both sweep time AND per-pass depth.
                // Inter-ring overhead ~1.5s (no pressure-settle wait).
                float per_pass_depth_est = smooth_mode ? depth_mm
                                                       : GENTLE_PER_PASS_DEPTH_MM;
                float min_dps_est = have_spd ? spd.min_continuous_dps : 10.9f;
                float _rt  = ring_throws[r];
                float _ri  = (r == num_rings-1) ? _rt * 0.92f : ring_throws[r+1];
                float _ro_m = _rt / 1000.0f, _ri_m = _ri / 1000.0f;
                float _ring_area = (float)M_PI *
                                   (_ro_m*_ro_m - _ri_m*_ri_m) * _arc_frac;
                float _rp  = cal_throw_to_psi(_rt) * pressure_scale;
                float _est_dps = min_dps_est;
                if (_rp > 0.1f && _ring_area > 1e-6f && per_pass_depth_est > 0.001f) {
                    float _Q = NOZZLE_FLOW_K * powf(_rp, NOZZLE_FLOW_N);
                    _est_dps = _Q * zone_arc_deg /
                               (per_pass_depth_est * 60000.0f * _ring_area);
                    if (_est_dps < min_dps_est) _est_dps = min_dps_est;
                }
                est_sweep    += zone_arc_deg / _est_dps;           // seconds
                est_overhead += (r == 0) ? 6.0f : 1.5f;
                if (_rp > 0.1f) {
                    float _dp = nozzle_precip_depth_mm(_rt, _ri, _est_dps, _rp);
                    if (_dp > 0.001f) { smooth_est_dp_sum += _dp; smooth_est_dp_n++; }
                }
            } else {
                float rt = ring_throws[r];
                float ri = (r == num_rings-1) ? act_min_throw : ring_throws[r+1];
                float ro = rt / 1000.0f, rinn = ri / 1000.0f;
                float ring_area = 3.14159f * (ro*ro - rinn*rinn) * _arc_frac;
                float ring_pct  = rt / act_max_throw * 100.0f * pressure_scale;
                float t_min     = cal_watering_time_min(ring_area, depth_mm, ring_pct);
                if (t_min < 0.05f) t_min = 0.05f;
                est_sweep    += t_min * 60.0f;
                // Per-ring overhead: ring 0 valve open ~6s, CW rings ~13s, CCW ~18s
                if (r == 0)       est_overhead += 6.0f;
                else if (r & 1)   est_overhead += 13.0f;
                else              est_overhead += 18.0f;
            }
        }
        // b302: hybrid volume + overhead initial estimate.
        // Replaces the pass-count-based formula with the physics:
        //   pumping_s = sum over rings of (ring_volume / ring_flow_rate)
        //   overhead_s = ~10s per pass (valve close+reposition, motor settle,
        //                dead-spot kicks, pass-boundary transitions)
        // Validated against Zone/N1/N2 b297 runs:
        //   Zone   8 rings:   actual  6 min, model ~5 min
        //   N1    15 rings:   actual 12 min, model ~11 min
        //   N2    23 rings:   actual 30 min, model ~28 min
        // After pass 0 finishes the volume+measured-overhead update below
        // takes over with empirical flow rate.
        float est_s;
        if (smooth_mode || gentle_mode) {
            float avg_dp = smooth_est_dp_n > 0
                         ? smooth_est_dp_sum / smooth_est_dp_n : 0.0f;
            float est_passes_f = (avg_dp > 0.001f)
                                ? ceilf(depth_mm / avg_dp) : 12.0f;
            // Clamp: outer rings typically need more passes than the average predicts
            // (they see higher PSI when inner rings drop out); add a 25% margin and
            // cap between 5 and the adaptive ceiling (smooth: 25, gentle: GENTLE_MAX_PASSES).
            float pass_ceiling = smooth_mode ? 25.0f : (float)GENTLE_MAX_PASSES;
            est_passes_f = fmaxf(5.0f, fminf(pass_ceiling, est_passes_f * 1.25f));
            INFO("%s est: %.2fmm/pass avg -> %.0f passes",
                 smooth_mode ? "Smooth" : "Gentle", avg_dp, est_passes_f);

            // Volume + per-ring flow rate
            float _arc_frac = zone_arc_deg / 360.0f;
            float total_pumping_s = 0.0f;
            float total_vol_L = 0.0f;
            for (int r = 0; r < num_rings; r++) {
                if (ring_throws[r] < 1.0f) continue;
                float ro_m = ring_throws[r] / 1000.0f;
                float ri_m = (r == num_rings - 1)
                           ? (act_min_throw / 1000.0f)
                           : (ring_throws[r+1] / 1000.0f);
                float ring_area_m2 = (float)M_PI *
                                     (ro_m*ro_m - ri_m*ri_m) * _arc_frac;
                if (ring_area_m2 < 1e-4f) continue;
                float ring_vol_L = ring_area_m2 * depth_mm;  // m^2 * mm = L
                total_vol_L += ring_vol_L;

                float ring_psi = cal_throw_to_psi(ring_throws[r]) * pressure_scale;
                if (ring_psi < 0.5f) ring_psi = 1.0f;
                float ring_flow_lpm =
                    NOZZLE_FLOW_K * powf(ring_psi, NOZZLE_FLOW_N) / 1000.0f;
                if (ring_flow_lpm < 0.05f) continue;
                total_pumping_s += (ring_vol_L / ring_flow_lpm) * 60.0f;
            }

            // Overhead: per-pass cost (valve operations + transitions).
            // 10s/pass tuned against b297 data across the three zones.
            // b304: add 30s cleanup tail (valve close, save, log refit, LFS
            // settle) so the no-history initial estimate matches the
            // per-pass adaptive (which also includes this tail).
            float overhead_s = 10.0f * est_passes_f;
            const float CLEANUP_TAIL_S = 30.0f;
            est_s = total_pumping_s + overhead_s + CLEANUP_TAIL_S;
            INFO("Vol est: %.1fL pumping %.0fs + overhead %.0fs (%.0f passes) "
                 "+ tail %.0fs = %d min",
                 total_vol_L, total_pumping_s, overhead_s,
                 est_passes_f, CLEANUP_TAIL_S, (int)(est_s / 60.0f) + 1);
        } else {
            // Pulse / other: original formula.
            est_s = (est_sweep + est_overhead + 20.0f) * passes;
        }
        s_water_est_min = (int)(est_s / 60.0f) + 1;
        s_eta_anchor_secs = est_s;                      // b362
        s_eta_anchor_tick = xTaskGetTickCount();
    } else {
        s_water_est_min = 2;
        s_eta_anchor_secs = 120.0f;                     // b362: demo 2min
        s_eta_anchor_tick = xTaskGetTickCount();
    }
    INFO("Estimated watering time: %d min", s_water_est_min);

    // Open watering CSV log on LittleFS.
    // b288: in smooth-aggregate mode the fopen is DEFERRED to run-end. Holding the
    // wbin file open during a long run keeps its CTZ metadata block dirty, so
    // concurrent reads from elsewhere (schedule_api zone_load every 30 s, fs_browse,
    // etc.) can trigger lfs_dir_compact which races with WiFi PHY cache-disable on
    // the ESP32 v3 silicon. Aggregate writes happen in one burst at run-end during
    // the b287 Quiet-mode window where WiFi is idle.
    s_last_depth_mm    = depth_mm;
    s_csv_pass_type    = 1;
    s_smooth_wbin_path[0] = '\0';
    bool _defer_open = (smooth_mode || serpentine_mode) && !s_water_detail_log;  // b423
    // b424: serpentine dry rehearsal writes no watering log -- without this gate
    // it would remove the zone's previous wbin and flush an empty aggregate
    // over the last real run's heatmap data.
    if (!demo_mode && !serpentine_dry && storage_ready()) {
        storage_ensure_dirs();  // /lfs/water must exist before fopen
        // Remove any stale CSV for this zone first to reclaim LittleFS space.
        char _old_csv[64]; water_csv_path_find(s_water_zone_id, _old_csv, sizeof(_old_csv));
        remove(_old_csv);
        char _csv_path[64]; water_csv_path_named(s_water_zone_id, _csv_path, sizeof(_csv_path));
        // Make room: aggregate smooth runs need ~41 KB max; per-pass modes need ~300 KB.
        // Smooth aggregates per-sector summaries (~41KB regardless of passes).
        // Gentle now writes per-pass per-sample rows over up to GENTLE_MAX_PASSES
        // passes -- bump allocation accordingly. Pulse stays at 300KB (1-2 passes).
        storage_make_room((smooth_mode || serpentine_mode) ? 64 * 1024
                                       : (gentle_mode ? 600 * 1024 : 300 * 1024),
                          s_water_zone_id);
        // Skip if still < 64 KB free.
        bool _csv_ok = true;
        { size_t _used = 0, _total = 0;
          if (storage_usage(&_used, &_total) == ESP_OK && _total > 0 && (_total - _used) < 65536) {
              ESP_LOGW(TAG, "LittleFS low space (%uKB free) -- skipping binary log",
                       (unsigned)((_total - _used) / 1024));
              _csv_ok = false;
          } }
        if (_csv_ok) {
            if (_defer_open) {
                // Stash the path; aggregate-flush block at run-end will fopen.
                strncpy(s_smooth_wbin_path, _csv_path, sizeof(s_smooth_wbin_path) - 1);
                s_smooth_wbin_path[sizeof(s_smooth_wbin_path) - 1] = '\0';
            } else {
                s_water_csv_f = fopen(_csv_path, "wb");
                if (s_water_csv_f) {
                    uint32_t _magic = WATER_BIN_MAGIC;
                    if (fwrite(&_magic, sizeof(_magic), 1, s_water_csv_f) != 1) {
                        ESP_LOGW(TAG, "Binary log header write failed -- disabling log");
                        fclose(s_water_csv_f); s_water_csv_f = NULL;
                    }
                } else {
                    ESP_LOGW(TAG, "Could not open %s", _csv_path);
                }
            }
        }
    }

    // CSV header for post-run analysis
    TickType_t t_start = xTaskGetTickCount();
    tprintf("time_s,ring,arc,sector,nozzle_deg_target,nozzle_deg_actual,"
            "valve_deg_target,valve_deg_actual,throw_mm_target,throw_mm_actual,"
            "pressure_psi_target,pressure_psi_actual,"
            "target_x,target_y,actual_x,actual_y,active\r\n");

    float total_time = 0.0f;
    int   rings_done = 0;

    // Smooth multipass: first-pass depth/ring is measured and used to
    // compute total passes needed to reach depth_mm target.
    // Indexed [0..num_rings-1]; populated after pass 0 completes.
    // smooth_depth_per_pass[i]: depth delivered per ring during pass 0.
    // Used as the per-pass rate estimate for the adaptive termination check.
    float smooth_depth_per_pass[WATER_RUN_MAX_RINGS];
    memset(smooth_depth_per_pass, 0, sizeof(smooth_depth_per_pass));
    // smooth_cumulative_depth[i]: total depth accumulated across all passes.
    // Replaces the old passes_completed * depth_per_pass calculation.
    // Rings are skipped once their cumulative reaches depth_mm.
    float smooth_cumulative_depth[WATER_RUN_MAX_RINGS];
    memset(smooth_cumulative_depth, 0, sizeof(smooth_cumulative_depth));
    // Per-ring valve throw correction from first-pass encoder measurement.
    float smooth_valve_corr[WATER_RUN_MAX_RINGS];
    for (int i = 0; i < WATER_RUN_MAX_RINGS; i++) smooth_valve_corr[i] = 1.0f;

    // b364: persistent "ring is unwaterable" flag, set when zone_clip_arc_to_polygon
    // collapses ALL of a ring's arcs to zero width. The clip is deterministic
    // in (polygon, ring_throw) so the result is the same on every pass --
    // without this flag the b296 scheduler keeps picking the same ring every
    // pass (deficit stays at full target), the arc loop runs and skips every
    // segment, and the run spins through its pass cap doing nothing. Symptom
    // on N1 b361: passes 21-30 all picked rings 1, 14, 15 in turn, every arc
    // logged "clipped to zero width by polygon, skipping segment", deficit
    // never closed, ~5-10 minutes wasted before the cap kicked in.
    bool ring_unwaterable[WATER_RUN_MAX_RINGS] = {0};

    // Smooth mode: accumulate sector measurements instead of writing per-pass rows.
    // When s_water_detail_log is on (set from landing page), write per-pass records
    // instead so the full 2°-sample data is available for debugging and evaluation.
    if ((smooth_mode || serpentine_mode) && !s_water_detail_log) {   // b423
        s_smooth_accum_mode = true;
        memset(s_smooth_accum, 0, sizeof(s_smooth_accum));
    }

    // b296 (Option C-Full): supply-aware ring scheduler state (smooth mode).
    //  - sched_median_throw: partition rings into HIGH-need (long throw) vs
    //    LOW-need (short throw) for the supply-match heuristic. Computed
    //    once at run start; ring count and throws don't change mid-run.
    //  - sched_supply_est: rolling estimate of pre-valve supply pressure,
    //    updated after each ring's avg_psi is computed. Bootstrap value
    //    of 0.0 signals "no measurement yet, use HIGH-supply default."
    //  - sched_cal_supply: anchor reference for HIGH/LOW classification
    //    (cal-time max nozzle pressure ~= cal-time supply).
    float sched_median_throw = ring_throws[num_rings / 2];   // middle ring's throw
    if (num_rings >= 2) {
        // Simple median via sorted copy (small N, safe to do once at start)
        float _sorted[WATER_RUN_MAX_RINGS];
        int _n_sort = num_rings < WATER_RUN_MAX_RINGS ? num_rings : WATER_RUN_MAX_RINGS;
        memcpy(_sorted, ring_throws, _n_sort * sizeof(float));
        for (int a = 1; a < _n_sort; a++) {
            float k = _sorted[a]; int b = a - 1;
            while (b >= 0 && _sorted[b] > k) { _sorted[b+1] = _sorted[b]; b--; }
            _sorted[b+1] = k;
        }
        sched_median_throw = _sorted[_n_sort / 2];
    }
    float sched_supply_est  = 0.0f;  // 0 = bootstrap, assume HIGH on first pick
    float sched_cal_supply  = cal_anchor_supply_pressure_map();
    if (smooth_mode) {
        INFO("b296 scheduler armed: median_throw=%.0fmm cal_supply=%.2f PSI",
             sched_median_throw, sched_cal_supply);
    }

    // b423: serpentine mode -- continuous serpentine. Branches here, after the
    // shared preamble (cal/zone load, ring plan, pressure check, log
    // setup), and rejoins at the shared abort:/closeout below. The
    // smooth/gentle pass loop never runs; the b296 scheduler is replaced
    // by monotonic out->in ordering (supply-aligned by construction).
    if (serpentine_mode) {
        int _serpentine_sweeps = 0;
        float _spsi_sum = 0.0f; int _spsi_n = 0;
        water_serpentine_passes(&zone, have_zone, sector_throw,
                           ring_throws, ring_direct, ring_ref_throw, num_rings,
                           ring_unwaterable, smooth_cumulative_depth,
                           zone_arc_start, zone_arc_end, zone_arc_deg,
                           act_max_throw, act_min_throw, have_throw_cal,
                           pressure_scale, psi_min, psi_max, &spd, have_spd,
                           depth_mm, passes, serpentine_dps, serpentine_dry,
                           &_serpentine_sweeps, &_spsi_sum, &_spsi_n, t_start);
        rings_done += _serpentine_sweeps;
        (void)_spsi_sum; (void)_spsi_n;
        goto abort;   // shared closeout: valve close, flush, depth, summary
    }

    for (int pass = 0; pass < passes; pass++) {
        // Track pass number in CSV (1=initial, 2+=subsequent)
        if (smooth_mode) s_csv_pass_type = (uint8_t)(pass + 1);
        if (smooth_mode) INFO("--- Pass %d ---", pass+1);
        else if (passes > 1) INFO("--- Pass %d/%d ---", pass+1, passes);
        // Smooth: alternate start direction each pass so pass N+1 begins from
        // where pass N ended, saving nozzle repositioning and ensuring both
        // arc ends get equal coverage over the full run.
        // Gentle joins smooth in alternating sweep direction so multi-pass runs
        // don't pay nozzle-reposition overhead between every pass.
        bool  cw             = (smooth_mode || gentle_mode) ? (pass % 2 == 0) : true;
        float prev_valve_deg = VALVE_CLOSED_DEG;
        bool  pass_first     = true;

        // b296: reset scheduler visited[] at the start of each pass so every
        // ring is eligible to be picked exactly once in this pass.
        // b364: pre-mark unwaterable rings as visited so the scheduler can't
        // pick them (and waste the pass running the no-op arc loop).
        bool sched_visited[WATER_RUN_MAX_RINGS] = {0};
        for (int _i = 0; _i < num_rings && _i < WATER_RUN_MAX_RINGS; _i++) {
            if (ring_unwaterable[_i]) sched_visited[_i] = true;
        }

        // b297: snapshot last actual_throw_mm for each ring so the scheduler
        // can apply throw-aware re-pick (keeps under-thrown rings eligible
        // even after their cumulative depth target is met).
        float sched_last_throw[WATER_RUN_MAX_RINGS];
        for (int _i = 0; _i < WATER_RUN_MAX_RINGS; _i++) {
            sched_last_throw[_i] = s_last_water_run.rings[_i].actual_throw_mm;
        }

    for (int _iter = 0; _iter < num_rings; _iter++) {
        // b296: smooth mode uses the supply-aware scheduler. Other modes
        // (gentle, pulse) keep the original sequential iteration order --
        // gentle has no per-ring valve correction, so reordering it has
        // no benefit, and pulse uses a completely different loop above.
        int ring;
        if (smooth_mode) {
            ring = smooth_scheduler_pick(sched_visited, num_rings,
                                          ring_throws, sched_last_throw,
                                          smooth_cumulative_depth,
                                          depth_mm, sched_supply_est,
                                          sched_cal_supply, sched_median_throw,
                                          pass);
            if (ring < 0) {
                INFO("Sched: no remaining deficit -- pass %d ended early", pass + 1);
                break;
            }
            sched_visited[ring] = true;
            float _def  = depth_mm - smooth_cumulative_depth[ring];
            // b297: same bootstrap fix as in smooth_scheduler_pick so the
            // log line agrees with what the scheduler actually used.
            bool _shi   = (sched_supply_est <= 0.0f || sched_cal_supply <= 0.5f)
                          ? true
                          : (sched_supply_est >= sched_cal_supply * 0.90f);
            bool _rh    = (ring_throws[ring] >= sched_median_throw);
            // b297: indicate whether re-pick was driven by throw shortfall
            // (depth was already met -- we're firing again for throw recovery).
            float _at = s_last_water_run.rings[ring].actual_throw_mm;
            bool _re_throw = (pass > 0 && _def <= 0.0f &&
                              _at >= 100.0f && ring_throws[ring] >= 100.0f &&
                              _at < ring_throws[ring] * 0.90f);
            INFO("Sched: pick ring %d (deficit %.2fmm, throw %.1fm, %s-need, "
                 "supply %s=%.2f, cal=%.2f%s)",
                 ring + 1, _def, ring_throws[ring] / 1000.0f,
                 _rh ? "HIGH" : "LOW",
                 _shi ? "HIGH" : "LOW",
                 sched_supply_est, sched_cal_supply,
                 _re_throw ? ", RE-THROW" : "");
        } else {
            ring = _iter;
            // b364: sequential modes (gentle, pulse) -- skip rings that earlier
            // passes marked unwaterable. Smooth mode handles this via the
            // sched_visited[] pre-mark above.
            if (ring < WATER_RUN_MAX_RINGS && ring_unwaterable[ring]) continue;
        }
        if (uart_getchar(0) != 0 || s_water_abort) { INFO("Stopped."); goto abort; }
        TOUCH_ACTIVITY();

        // Adaptive: skip rings whose cumulative depth already meets target.
        // Smooth also requires throw to be accurate (within 20%); gentle has no
        // iterative throw correction so only the depth criterion applies.
        // Still flip cw to keep the alternating-direction pattern intact for
        // whichever rings DO sweep this pass.
        if ((smooth_mode || gentle_mode) && pass > 0 && ring < WATER_RUN_MAX_RINGS) {
            bool _depth_ok = smooth_cumulative_depth[ring] >= depth_mm;
            bool _throw_ok = true;
            if (smooth_mode) {
                // b297: tighten throw_ok to require BOTH no-overshoot AND
                // no-under-throw. Previously only checked actual <= target*1.20
                // (overshoot guard), which meant under-thrown rings were still
                // "throw_ok" and got skipped once depth was met -- defeating
                // the scheduler's b297 throw-aware re-pick. Now an under-thrown
                // ring (<90% of target) is NOT throw_ok and will be re-fired
                // to give the supply-limited diagnostic + valve_corr machinery
                // a chance to recover.
                float _at = s_last_water_run.rings[ring].actual_throw_mm;
                _throw_ok = (_at < 100.0f) ||
                            (_at >= ring_throws[ring] * 0.90f &&
                             _at <= ring_throws[ring] * 1.20f);
            }
            if (_depth_ok && _throw_ok) { cw = !cw; continue; }
        }

        float ring_throw  = ring_throws[ring];
        bool  direct_ring = ring_direct[ring];

        // --- Look up valve angle ---
        float valve_target_deg;
        // Smooth mode: no per-arc PID, so pre-scale the effective throw used for
        // valve lookup by 1/pressure_scale. This opens the valve further to
        // compensate for lower supply pressure measured at startup.
        float valve_ring_throw = ring_throw;
        if (smooth_mode) {
            if (pass == 0) {
                // Pass 0: global supply pressure correction only.
                // Handles low/high supply pressure vs calibration max at startup.
                if (pressure_scale > 0.1f && fabsf(pressure_scale - 1.0f) > 0.05f)
                    valve_ring_throw = fminf(ring_throw / pressure_scale, act_max_throw);
            } else {
                // Passes 1+: use per-ring iterative correction exclusively.
                // smooth_valve_corr initializes to 1.0 (no effect) and is updated
                // each pass from measured throw ratios; pressure_scale is NOT applied
                // here to avoid re-introducing cold-start startup-PSI error.
                // b303: previously this clamped valve_ring_throw at
                // act_max_throw (the cal table's max throw), which meant
                // corr could never push the valve past its calibrated
                // setting -- defeating the purpose of corr for rings beyond
                // cal range. Loosen to act_max_throw * 1.5 so a corr that
                // wants more aggressive opening can reach the new extended
                // range in cal_throw_to_valve_deg (up to VALVE_PEAK_DEG).
                valve_ring_throw = fminf(ring_throw * smooth_valve_corr[ring],
                                         act_max_throw * 1.5f);
            }
        }

        if (direct_ring) {
            // Direct-valve mode for rings below the cal table's min throw.
            // b382: anchor the interpolation on the FLOW band, not the origin.
            // The old formula walk_valve*(throw/ref_throw) ran a line through
            // (0mm, 0deg), so every sub-cal-min throw fell below VALVE_CAL_START_DEG
            // and clamped to it -- all inner rings collapsed onto the 263 deg
            // no-flow threshold (the dry-center hole). It also paired walk_valve
            // (zone_avg_valve_deg ~= the valve for the zone's *perimeter* throw,
            // ~271 deg) with ref_throw=cal_min (~982mm), a >2x mismatch that put
            // the anchor in the wrong place. Anchor instead on the physically
            // correct (ref_throw -> cal valve) pair and floor at the flow angle,
            // so inner rings spread linearly across the flowing band (~263..265
            // deg) and actually reach the center instead of piling on 263.
            float ref_throw = ring_ref_throw[ring];
            float ref_valve = (ref_throw > 0.0f)
                ? cal_throw_to_valve_deg(ref_throw) : VALVE_CAL_START_DEG;
            valve_target_deg = (ref_throw > 0.0f)
                ? VALVE_CAL_START_DEG
                  + (ref_valve - VALVE_CAL_START_DEG) * (valve_ring_throw / ref_throw)
                : VALVE_CAL_START_DEG;
            if (valve_target_deg < VALVE_CAL_START_DEG)
                valve_target_deg = VALVE_CAL_START_DEG;
            INFO("Ring %2d: direct-valve %.1f deg (band-anchored: ref %.0fmm -> %.1f deg, no PID)",
                 ring+1, valve_target_deg, ref_throw, ref_valve);
        } else if (have_throw_cal) {
            valve_target_deg = cal_throw_to_valve_deg(valve_ring_throw);
            if (valve_target_deg < VALVE_CAL_START_DEG)
                valve_target_deg = VALVE_CAL_START_DEG;
        } else {
            // Fallback: use pressure ratio
            float ring_pct = valve_ring_throw / act_max_throw * 100.0f;
            float ring_psi = psi_min + (ring_pct/100.0f) * (psi_max - psi_min);
            valve_target_deg = cal_pressure_to_valve_deg(ring_psi);
            if (valve_target_deg < VALVE_CAL_START_DEG)
                valve_target_deg = VALVE_CAL_START_DEG;
        }
        valve_target_deg = fmaxf(VALVE_CLOSED_DEG,
                           fminf(VALVE_OPEN_DEG, valve_target_deg));

        // b283: tag the trace with this ring's valve_deg. Trace samples
        // arriving until the next ring boundary will be attributed to this
        // ring's valve position. No-op if trace recorder isn't armed
        // (pulse/demo modes), which is the desired behavior.
        water_trace_mark_ring(ring, valve_target_deg);

        // --- Sector activity ---
        water_arc_t arcs[WATER_MAX_ARCS_PER_RING];
        bool active[WATER_SECTORS];
        int  active_count = 0;
        for (int s = 0; s < WATER_SECTORS; s++) {
            float tol = WATER_RING_SPACING * (ring_throw/act_max_throw) * 0.5f;
            active[s] = (ring_throw <= sector_throw[s] + tol);
            if (active[s]) active_count++;
        }
        if (active_count == 0) {
            INFO("Ring %2d: throw=%.0fmm -- no active sectors, skipping.",
                 ring+1, ring_throw);
            continue;
        }

        int num_arcs = water_find_arcs(active, arcs);
        // Sort arcs in physical sweep order relative to the zone arc start/end.
        // CW:  ascending CW distance of first_sec from zone_arc_start.
        // CCW: ascending CCW distance of last_sec from zone_arc_end.
        // Using absolute sector indices (old code) breaks for zones that cross
        // 0 degrees: e.g. arc 334->70 has sectors 34,35,0,1,2,3 -- sector 3
        // sorts before 34 numerically but is physically LAST in CW order.
        for (int a = 0; a < num_arcs-1; a++)
            for (int b = a+1; b < num_arcs; b++) {
                float da, db;
                if (cw) {
                    // CW distance of arc first_sec from zone_arc_start
                    float s_a = arcs[a].first_sec * WATER_SECTOR_DEG;
                    float s_b = arcs[b].first_sec * WATER_SECTOR_DEG;
                    da = fmodf(s_a - zone_arc_start + 360.0f, 360.0f);
                    db = fmodf(s_b - zone_arc_start + 360.0f, 360.0f);
                } else {
                    // CCW distance of arc last_sec from zone_arc_end
                    float e_a = arcs[a].last_sec * WATER_SECTOR_DEG;
                    float e_b = arcs[b].last_sec * WATER_SECTOR_DEG;
                    da = fmodf(zone_arc_end - e_a + 360.0f, 360.0f);
                    db = fmodf(zone_arc_end - e_b + 360.0f, 360.0f);
                }
                if (db < da) { water_arc_t tmp=arcs[a]; arcs[a]=arcs[b]; arcs[b]=tmp; }
            }

        // --- Nozzle speed ---
        float arc_fraction = (float)active_count / (float)WATER_SECTORS;
        // Use 92% of ring throw for innermost ring inner boundary (matches cleanup
        // analysis formula). act_min_throw is the zone edge -- often within 50mm of
        // the ring throw -- giving near-zero annular area and hitting the time_min floor.
        float inner_throw  = (ring == num_rings-1) ? ring_throw * 0.92f : ring_throws[ring+1];
        float r_outer = ring_throw  / 1000.0f;
        float r_inner = inner_throw / 1000.0f;
        bool  below_e = (ring_throw < WATER_MIN_ELLIPSE_THROW_MM);
        if (below_e) {
            float sr = WATER_INNER_SPLASH_RADIUS_MM / 1000.0f;
            r_outer = ring_throw/1000.0f + sr*0.5f;
            r_inner = fmaxf(0, ring_throw/1000.0f - sr*0.5f);
        }
        float ring_area = 3.14159f*(r_outer*r_outer - r_inner*r_inner)*arc_fraction;

        float nozzle_dps;
        bool  use_pulse_mode = false;
        if (demo_mode) {
            nozzle_dps = 118.0f;
        } else {
            float active_deg = active_count * WATER_SECTOR_DEG;
            float time_min;
            // Per-pass target depth — what one nozzle sweep should deposit on this ring.
            //   smooth: depth_mm per pass (adaptive multi-pass tops up via cumulative tracker)
            //   gentle: GENTLE_PER_PASS_DEPTH_MM (~0.635mm, seed-safe; adaptive multi-pass)
            //   pulse:  depth_mm / passes (1- or 2-pass open-loop hits target exactly)
            float per_pass_depth;
            if (smooth_mode)       per_pass_depth = depth_mm;
            else if (gentle_mode)  per_pass_depth = GENTLE_PER_PASS_DEPTH_MM;
            else                   per_pass_depth = depth_mm / (float)(passes > 0 ? passes : 1);

            // Flow-model dps: dps = Q[mL/min]*active_deg / (per_pass_depth*60000*ring_area[m2])
            // delivers per_pass_depth in one sweep at the reference PSI.
            // Smooth uses calibration-reference PSI (per-ring iterative throw correction
            // converges to live conditions); gentle/pulse use live supply scale so the
            // first pass already targets today's flow rate.
            bool flow_override = false;
            if (ring_area > 0.0f && per_pass_depth > 0.001f) {
                float ref_psi = cal_throw_to_psi(ring_throw);
                if (!smooth_mode) ref_psi *= pressure_scale;
                if (ref_psi > 0.1f) {
                    float Q_ref = NOZZLE_FLOW_K * powf(ref_psi, NOZZLE_FLOW_N);
                    nozzle_dps  = Q_ref * active_deg / (per_pass_depth * 60000.0f * ring_area);
                    time_min    = active_deg / (nozzle_dps * 60.0f);
                    flow_override = true;
                }
            }
            if (!flow_override) {
                // Fallback: legacy linear flow model (only hit when cal_throw_to_psi
                // returns ~0, e.g. uncalibrated install).
                float ring_pct = ring_throw / act_max_throw * 100.0f * pressure_scale;
                time_min = cal_watering_time_min(ring_area, per_pass_depth, ring_pct);
                if (time_min < 0.05f) time_min = 0.05f;
                nozzle_dps = active_deg / (time_min * 60.0f);
            }

            float min_dps = have_spd ? spd.min_continuous_dps : 10.9f;
            float max_dps = have_spd ? spd.deg_per_sec[spd.num_points-1] : 118.0f;
            // Pulse mode: encoder-feedback pauses at each sector (accurate but stationary).
            // Gentle and smooth modes force continuous sweep so the nozzle never stops.
            use_pulse_mode = !demo_mode && !gentle_mode && !smooth_mode;
            if (!use_pulse_mode && nozzle_dps < min_dps) nozzle_dps = min_dps;
            if (nozzle_dps > max_dps) nozzle_dps = max_dps;
            total_time += time_min;

            // Inner rings (inside ~6 ft) have such a small annular area that the
            // flow-solved sweep speed pins at max_dps and still over-deposits
            // relative to per_pass_depth. b360/b382 tried to *skip* those rings
            // when a predicted deposit exceeded 1.5x target, but that prediction
            // used an area inconsistent with the dps solver above, so it fired on
            // rings that actually deposit at/under target -- and a ceiling-pinned
            // inner ring has no faster pass to defer to anyway. The skip left the
            // center permanently dry (the TopCorner dry-hole bug). The dps solver
            // already delivers exactly per_pass_depth whenever the sweep is
            // unclamped; a ceiling-pinned ring over-deposits once and is then
            // satisfied by cumulative-depth tracking. So we always water every
            // ring -- no skip.
        }

        float ring_psi_sum = 0.0f; int ring_psi_n = 0;
        bool  ring_no_flow = false;   // set if no detectable flow on first arc
        INFO("Ring %2d/%d  throw=%.0fmm  valve=%.1fdeg (was %.1f, delta %.1f)  "
             "%d arc(s)  %.1fdps  %s  %s",
             ring+1, num_rings, ring_throw,
             valve_target_deg, prev_valve_deg, valve_target_deg - prev_valve_deg,
             num_arcs, nozzle_dps, cw ? "CW" : "CCW",
             use_pulse_mode ? "[PULSE]" : gentle_mode ? "[GENTLE]" : smooth_mode ? "[SMOOTH]" : "");

        // Lookup run_duty from the appropriate directional speed table.
        // b423: extracted verbatim to spd_duty_for_dps() (shared with serpentine).
        uint16_t run_duty = spd_duty_for_dps(&spd, have_spd, nozzle_dps, cw);
        if (demo_mode) run_duty = spd.num_points>0 ? spd.duty[spd.num_points-1] : 480;

        // --- Sweep each arc segment ---
        // Each segment is swept continuously at constant valve pressure.
        // Between segments: valve closes, nozzle transits, valve reopens.
        // This correctly handles zones with notches, driveways, walkways etc.
        // Hoisted to ring scope so ring-data recording can reference last arc's values.
        float    seg_origin = 0.0f, seg_deg = 0.0f;
        // b364: count arcs that passed the polygon clip and reached the sweep.
        // If this is still 0 after the arc loop completes, every arc was
        // clipped to zero -- ring is unwaterable, mark it so subsequent
        // passes skip it.
        int      arcs_swept_this_ring = 0;
        // Accumulators for smooth-mode actual speed measurement.
        // Sum up the encoder-timed elapsed ms and degrees across all arcs of
        // this ring so we can compute the true average dps after the arc loop.
        uint32_t ring_sweep_ms       = 0;
        float    ring_arc_deg_swept  = 0.0f;
        for (int ai = 0; ai < num_arcs; ai++) {
            if (uart_getchar(0) != 0 || s_water_abort) { INFO("Stopped."); goto abort; }
            TOUCH_ACTIVITY();

            water_arc_t *arc = &arcs[ai];
            bool is_first_arc = (ai == 0);
            bool is_last_arc  = (ai == num_arcs - 1);

            seg_origin = 0.0f; seg_deg = 0.0f;  // reset each arc
            // Compute segment sweep boundaries.
            // Use exact zone boundary angles only when the arc's sectors are
            // within 2 sectors (20 deg) of the zone edge -- not just because
            // it happens to be the first/last arc in the loop.
            // This prevents outer rings from sweeping the full zone span when
            // only a small sub-arc (e.g. 20-30 deg) is actually active.
            {
                float first_b = (float)(arc->first_sec) * WATER_SECTOR_DEG;
                float last_b  = (float)((arc->last_sec + 1) % WATER_SECTORS) * WATER_SECTOR_DEG;

                // Left boundary: snap first_b to zone_arc_start only when the
                // arc genuinely starts at the CCW zone edge (within half a sector).
                // The old 2-sector (20 deg) threshold was too wide: outer rings
                // whose first active sector is 10+ deg inside the zone were still
                // snapped to zone_arc_start, making them sweep bearings where their
                // throw exceeds the zone polygon's outer boundary.
                float _lo_cw_from_start = fmodf(first_b - zone_arc_start + 360.0f, 360.0f);
                bool  _first_in_zone    = (_lo_cw_from_start <= zone_arc_deg + 0.5f);
                float left_gap = fminf(_lo_cw_from_start,
                                       fmodf(zone_arc_start - first_b + 360.0f, 360.0f));
                float lo = (!_first_in_zone || left_gap < 0.5f * WATER_SECTOR_DEG)
                           ? zone_arc_start : first_b;

                // Right boundary: same tightened threshold.
                float _hi_cw_from_start = fmodf(last_b - zone_arc_start + 360.0f, 360.0f);
                bool  _last_in_zone     = (_hi_cw_from_start <= zone_arc_deg + 0.5f);
                float right_gap = fminf(fmodf(last_b      - zone_arc_end + 360.0f, 360.0f),
                                        fmodf(zone_arc_end - last_b      + 360.0f, 360.0f));
                float hi = (!_last_in_zone || right_gap < 0.5f * WATER_SECTOR_DEG)
                           ? zone_arc_end : last_b;

                // Refine sector-quantized boundaries to the exact polygon
                // crossing bearing for this ring's throw. This corrects the
                // ~5-10 deg overshoot/undershoot caused by snapping to 10-deg
                // sector edges when the true boundary falls mid-sector.
                if (have_zone && lo == first_b)
                    lo = zone_refine_arc_bound(&zone, ring_throw, first_b, WATER_SECTOR_DEG);
                if (have_zone && hi == last_b)
                    hi = zone_refine_arc_bound(&zone, ring_throw, last_b, WATER_SECTOR_DEG);

                // b309: tighten lo/hi to bearings where the polygon
                // actually contains (bearing, ring_throw). Catches:
                //   - linear-interp refinement landing at polygon-edge
                //     bearings that are wrong side of the polygon at
                //     this ring's throw
                //   - snap-to-zone_arc_start/end pulling sweep into a
                //     region where polygon has dropped radially below
                //     ring_throw (e.g. N3-style narrow strips at
                //     same-bearing inner+outer vertex pairs)
                // Step 0.5 deg = fine enough to nail mid-sector crossings
                // without burning cycles on every ring.
                if (have_zone)
                    zone_clip_arc_to_polygon(&zone, ring_throw, &lo, &hi, 0.5f);

                seg_deg = fmodf(hi - lo + 360.0f, 360.0f);
                // The polygon clip returns a fully-inside contiguous sub-arc.
                // If that real inside span is below the minimum sweepable arc,
                // skip the segment. Do NOT floor it up to WATER_SECTOR_DEG:
                // that re-expands the sweep past the inside region and spills
                // water outside the polygon -- the failure mode on a flat
                // equal-throw vertex band (perimeter walked above the throw-
                // cal range). Sweeping the true sub-sector inside span is
                // safe; expanding it is not.
                if (seg_deg < 0.5f * WATER_SECTOR_DEG) {
                    INFO("Ring %2d arc %d: inside span %.1fdeg < min, skipping segment.",
                         ring+1, ai, seg_deg);
                    continue;
                }
                arcs_swept_this_ring++;   // b364: past the polygon-clip gate
                if (seg_deg > zone_arc_deg + 1.0f) seg_deg = zone_arc_deg;
                // CW rings consistently overshoot arc end by ~7 deg due to
                // nozzle inertia. Pre-compensate so water stays inside zone.
                // Gentle mode uses encoder-based stopping (COAST_DEG), so no
                // pre-compensation needed there.
                if (cw && !gentle_mode && seg_deg > WATER_SECTOR_DEG + 1.0f)
                    seg_deg -= 7.0f;
                seg_origin = cw ? lo : hi;
            }

            // Position nozzle:
            //   First arc of each pass  -> goto (nozzle starting from rest)
            //   Subsequent arcs in ring -> goto (transit after valve-close gap)
            //   First arc, ring > 0     -> no goto unless nozzle is far ahead
            if (pass_first || !is_first_arc) {
                nozzle_goto(seg_origin, 1.0f, 10000, false);
                s_nozzle_last_dir = cw ? 1 : -1;
                // Gentle mode: valve is still closed here -- do a precise second
                // goto if nozzle is still behind arc start. Initial goto from
                // 15 deg out often undershoots 2-3 deg, causing the valve-open
                // pressure-settle dwell to spray outside the zone (5 passes × 1.5s).
                if (gentle_mode) {
                    uint16_t _chk = 0;
                    if (as5600_read(ADDR_AS5600, &_chk, NULL, NULL)) {
                        float _cpos = _chk * (360.0f / 4096.0f);
                        float _behind = cw
                            ? fmodf(seg_origin - _cpos + 360.0f, 360.0f)
                            : fmodf(_cpos - seg_origin + 360.0f, 360.0f);
                        if (_behind > 180.0f) _behind = 0.0f;
                        if (_behind > 0.5f && _behind < 20.0f)
                            nozzle_goto_direct(seg_origin, 0.5f, 3000, false);
                    }
                }
                if (pass_first) pass_first = false;
            } else {
                // First arc of ring > 0: nozzle should be near seg_origin.
                // Reposition under valve-closed if nozzle is more than a few
                // degrees off in EITHER direction. b310: previously only the
                // "ahead" and CCW-behind cases triggered a reposition; CW rings
                // following clipped rings could find themselves 30+ deg BEHIND
                // seg_origin with no reposition, then have the sweep extend
                // backwards (bypassing the polygon clip).
                uint16_t _nr = 0;
                if (as5600_read(ADDR_AS5600, &_nr, NULL, NULL)) {
                    float _npos = _nr * (360.0f / 4096.0f);
                    // "Ahead" = past seg_origin in sweep direction.
                    float _ahead = cw
                        ? fmodf(_npos - seg_origin + 360.0f, 360.0f)
                        : fmodf(seg_origin - _npos + 360.0f, 360.0f);
                    // "Behind" = before seg_origin in sweep direction.
                    float _behind = cw
                        ? fmodf(seg_origin - _npos + 360.0f, 360.0f)
                        : fmodf(_npos - seg_origin + 360.0f, 360.0f);
                    if (_ahead  > 180.0f) _ahead  = 0.0f;
                    if (_behind > 180.0f) _behind = 0.0f;
                    bool _need_goto = (_ahead > 6.0f && _ahead < 180.0f)
                        || (_behind > 3.0f && _behind < 180.0f);
                    if (_need_goto) {
                        INFO("  Nozzle %s arc start by %.1f deg -- repositioning to %.1f deg",
                             _behind > _ahead ? "behind" : "ahead",
                             _behind > _ahead ? _behind : _ahead, seg_origin);
                        nozzle_goto(seg_origin, 1.5f, 10000, false);
                        s_nozzle_last_dir = cw ? 1 : -1;
                    }
                }
            }

            // Open valve to ring target.
            // b367: low-pressure inner rings (throw below the zone's normal-ring
            // boundary) cannot be reached reliably by closing the valve down from
            // a higher ring -- gear backlash leaves the seat in a dead spot and
            // the sprinkler reads no flow even at the commanded angle. Seat from
            // closed and open upward to the ring's target pressure instead. The
            // achieved angle replaces valve_target_deg for downstream logging/sweep.
            bool inner_ring = (ring_throw < act_min_throw - 1.0f);
            if (inner_ring) {
                if (is_first_arc) {
                    float _seat_psi = 0.0f;
                    float _tp = cal_throw_to_psi(ring_throw);
                    valve_target_deg = water_seat_valve_from_closed(_tp, &_seat_psi);
                } else {
                    // Re-open from below: the valve was driven closed during the
                    // inter-arc nozzle transit, so a direct goto opens up into it.
                    valve_goto_direct(valve_target_deg, 0.3f, 6000, false);
                }
                s_valve_last_dir = 1;
            } else {
                // Compute direction from actual positions: if we just reduced to
                // inner boundary (s_valve_last_dir=-1), passing that to valve_goto_ex
                // applies backlash by closing further -- hitting the mechanical min
                // and stalling. Always pass the true required direction.
                uint16_t _vr=0; as5600_read(ADDR_AS5600L, &_vr, NULL, NULL);
                float _cur = _vr * (360.0f/4096.0f);
                int   _dir = (valve_target_deg > _cur) ? 1 : -1;
                bool _vok = valve_goto_ex(valve_target_deg, 0.3f, 8000, false, _dir);
                s_valve_last_dir = _dir;
                // Valve-fault detection: if valve_goto_ex returned false the
                // motor failed to reach the target within tolerance/timeout.
                // Verify with a fresh encoder read (the function can return
                // false on transient I2C glitches even when the valve is
                // actually at target) before flagging a fault.
                if (!_vok) {
                    uint16_t _vr2=0;
                    if (as5600_read(ADDR_AS5600L, &_vr2, NULL, NULL)) {
                        float _post = _vr2 * (360.0f/4096.0f);
                        float _err  = fabsf(_post - valve_target_deg);
                        if (_err > 180.0f) _err = 360.0f - _err;
                        // 8 deg is well beyond the 0.3 deg working tolerance
                        // and the ~2 deg friction-zone jog precision — only
                        // an unmistakable mechanical failure should land here.
                        if (_err > 8.0f) {
                            ESP_LOGW(TAG, "Valve fault: target %.1f reached %.1f (err %.1f)",
                                     valve_target_deg, _post, _err);
                            water_set_status(WATER_STATUS_VALVE_FAULT);
                            goto abort;
                        }
                    }
                }
            }
            s_valve_last_dir = (valve_target_deg > prev_valve_deg) ? 1 : -1;

            // b305: scaled pressure-settle wait at ring entry.
            // The previous fixed 150ms wait was not long enough after a
            // large valve move -- ring 1's CSV showed throw rising
            // monotonically across the sweep (5191 -> 6338 -> 6350 -> 7373
            // mm on the Zone b302 run), evidence that pressure was still
            // building when the nozzle started moving. The water column,
            // valve mechanical settling, and pump pressure recovery all
            // need more time after a big angle change; after a small
            // delta (consecutive rings with similar throws), 150ms is fine.
            //
            // Scale by |valve_target - prev_valve|:
            //   > 5 deg  -> 600 ms (big move, e.g. inner ring -> outer ring)
            //   1-5 deg  -> 300 ms (moderate)
            //   < 1 deg  -> 150 ms (same angle, e.g. multi-arc same ring)
            //
            // Read prev_valve_deg BEFORE the is_first_arc update below.
            // Only on the first arc of a ring -- subsequent arcs reuse the
            // same valve position so no settle is needed.
            if (smooth_mode && is_first_arc) {
                float _vdelta = fabsf(valve_target_deg - prev_valve_deg);
                int _settle_ms = 150;
                if (_vdelta > 5.0f)      _settle_ms = 600;
                else if (_vdelta > 1.0f) _settle_ms = 300;
                vTaskDelay(pdMS_TO_TICKS(_settle_ms));
            }

            if (is_first_arc) prev_valve_deg = valve_target_deg;

            // On the first arc of each ring: measure actual settled pressure and
            // compensate nozzle_dps for the actual vs planned flow rate.
            // Smooth mode skips this entirely -- open-loop by design; no settle wait
            // means the valve is still moving when the sweep starts.
            if (is_first_arc && !smooth_mode) {
                if (valve_in_friction_zone(valve_target_deg)) {
                    INFO("  DPS adj ring %d: skipped (hydrodynamic zone %.1f deg)",
                         ring+1, valve_target_deg);
                    // Pressure trim: wait for hydraulic settle, then nudge valve
                    // angle to bring actual PSI toward target. Higher angle = more
                    // open = higher pressure, so close (decrease) if too high.
                    // Two correction passes; cap 3 deg per pass. valve_target_deg
                    // is updated so the sweep PID starts from the corrected position.
                    {
                        vTaskDelay(pdMS_TO_TICKS(800));
                        float _tp = cal_throw_to_psi(ring_throw);
                        float _ap = 0.0f;
                        mprls_read(&_ap);
                        if (_ap > 0.5f && _tp > 0.5f) {
                            float _ratio = _ap / _tp;
                            float _adj = 0.0f;
                            if (_ratio > 1.15f)
                                _adj = -fminf(3.0f, (_ratio - 1.0f) * 8.0f); // close
                            else if (_ratio < 0.85f)
                                _adj = fminf(3.0f, (1.0f - _ratio) * 8.0f);  // open
                            if (fabsf(_adj) > 0.1f) {
                                valve_target_deg += _adj;
                                INFO("  Friction trim ring %d: %.2f PSI vs %.2f target (%.0f%%) -- adj %.1f deg (->%.1f)",
                                     ring+1, _ap, _tp, _ratio*100.0f,
                                     _adj, valve_target_deg);
                                valve_goto_jog(valve_target_deg, 0.5f, 4000);
                                s_valve_last_dir = (_adj > 0) ? 1 : -1;
                                // Second pass for large deficits
                                if (fabsf(_adj) > 1.0f) {
                                    vTaskDelay(pdMS_TO_TICKS(600));
                                    float _ap2 = 0.0f;
                                    mprls_read(&_ap2);
                                    if (_ap2 > 0.5f) {
                                        float _ratio2 = _ap2 / _tp;
                                        float _adj2 = 0.0f;
                                        if (_ratio2 > 1.10f)
                                            _adj2 = -fminf(2.0f, (_ratio2 - 1.0f) * 8.0f);
                                        else if (_ratio2 < 0.90f)
                                            _adj2 = fminf(2.0f, (1.0f - _ratio2) * 8.0f);
                                        if (fabsf(_adj2) > 0.1f) {
                                            valve_target_deg += _adj2;
                                            INFO("  Friction trim2 ring %d: %.2f PSI (%.0f%%) -- adj2 %.1f deg (->%.1f)",
                                                 ring+1, _ap2, _ratio2*100.0f,
                                                 _adj2, valve_target_deg);
                                            valve_goto_jog(valve_target_deg, 0.5f, 4000);
                                            s_valve_last_dir = (_adj2 > 0) ? 1 : -1;
                                        } else {
                                            INFO("  Friction trim2 ring %d: %.2f PSI (%.0f%%) -- OK",
                                                 ring+1, _ap2, _ratio2*100.0f);
                                        }
                                    }
                                }
                            } else {
                                INFO("  Friction trim ring %d: %.2f PSI vs %.2f target -- OK",
                                     ring+1, _ap, _tp);
                            }
                        }
                    }
                } else {
                    float _tp  = cal_throw_to_psi(ring_throw);
                    float _ap  = 0.0f;
                    if (ring == 0) {
                        if (_tp > 0.5f)
                            _ap = water_hold_pressure(_tp, psi_min, psi_max);
                        else
                            vTaskDelay(pdMS_TO_TICKS(1500));
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(200));
                        mprls_read(&_ap);
                    }
                    if (_ap > 0.5f && _tp > 0.5f) {
                        float _af = cal_pressure_to_flow_lpm(_ap / psi_max * 100.0f);
                        float _tf = cal_pressure_to_flow_lpm(_tp / psi_max * 100.0f);
                        if (_tf > 0.1f) {
                            float _r = fmaxf(0.5f, fminf(2.0f, _af / _tf));
                            if (fabsf(_r - 1.0f) > 0.05f) {
                                INFO("  DPS adj ring %d: %.2f->%.2f (actual %.2f vs target %.2f PSI)",
                                     ring+1, nozzle_dps, nozzle_dps * _r, _ap, _tp);
                                nozzle_dps *= _r;
                            }
                        }
                    }
                }
            }

            // No-flow check (first arc only): if valve is open but PSI is below
            // the detectable-flow threshold the ring is at the inner limit of what
            // the sprinkler can water.  Close valve and skip this ring entirely.
            // Smooth/gentle have no prior settle, so add a brief wait before reading.
            if (is_first_arc) {
                float _nf = 0.0f;
                if (smooth_mode || gentle_mode)
                    vTaskDelay(pdMS_TO_TICKS(200));
                mprls_read_quiet(&_nf);
                if (_nf < WATER_MIN_FLOW_PSI) {
                    INFO("Ring %d: no flow (%.3f PSI < %.2f) -- skipping ring",
                         ring+1, _nf, WATER_MIN_FLOW_PSI);
                    valve_goto_ex(VALVE_CLOSED_DEG, 2.0f, 8000, false, -1);
                    s_valve_last_dir = -1;
                    ring_no_flow = true;
                    // Water-loss detection: needs sustained no-flow after we'd
                    // already seen good flow earlier. A single dry ring can be
                    // a transient supply blip; require WATER_LOSS_STREAK_THRESHOLD
                    // consecutive dry rings to flag the run as water_loss.
                    if (s_water_run_had_flow) {
                        s_water_no_flow_streak++;
                        if (s_water_no_flow_streak >= WATER_LOSS_STREAK_THRESHOLD) {
                            INFO("  -> Lost flow mid-run (%d consecutive dry rings)",
                                 s_water_no_flow_streak);
                            water_set_status(WATER_STATUS_WATER_LOSS);
                        }
                    }
                    break;  // break arc loop; ring_no_flow guards data recording below
                }
                // Flow restored — reset both the "had flow" tracker (latched
                // true on first detection) and the dry-streak counter.
                s_water_run_had_flow   = true;
                s_water_no_flow_streak = 0;
            }

            // Sweep: gentle/smooth use encoder-position-based continuous sweep;
            // pulse mode for slow rings; continuous for fast/demo.
            bool swept_ok;
            if (gentle_mode || smooth_mode) {
                uint32_t arc_elapsed_ms = 0;
                swept_ok = nozzle_sweep_encoder_gentle(
                    seg_origin, seg_deg, cw, run_duty,
                    nozzle_dps,
                    t_start, ring+1, ai+1, arc->first_sec,
                    valve_target_deg, ring_throw,
                    &ring_psi_sum, &ring_psi_n,
                    &arc_elapsed_ms);
                // Accumulate actual sweep time for smooth-mode dps measurement.
                if (smooth_mode && arc_elapsed_ms > 0) {
                    ring_sweep_ms      += arc_elapsed_ms;
                    ring_arc_deg_swept += seg_deg;
                }
            } else if (use_pulse_mode) {
                // Use calibrated pulse params or safe defaults if cal not available
                uint16_t  eff_duty = (have_spd && spd.jog_pulse_duty > 0)
                                     ? spd.jog_pulse_duty : 200;
                uint32_t  eff_ms   = (have_spd && spd.jog_pulse_ms > 0)
                                     ? (uint32_t)spd.jog_pulse_ms : 80;
                swept_ok = nozzle_sweep_pulse(
                    seg_origin, seg_deg, cw,
                    nozzle_dps,
                    eff_duty, eff_ms,
                    t_start, ring+1, ai+1, arc->first_sec,
                    valve_target_deg, ring_throw,
                    &ring_psi_sum, &ring_psi_n, direct_ring);
            } else {
                swept_ok = nozzle_sweep_continuous(
                    seg_origin, seg_deg, cw, run_duty,
                    demo_mode ? 118.0f : nozzle_dps,
                    t_start, ring+1, ai+1, arc->first_sec,
                    valve_target_deg, ring_throw);
            }
            if (!swept_ok) { INFO("Stopped."); goto abort; }
            s_nozzle_last_dir = cw ? 1 : -1;

            // Between segments: close the valve while the nozzle transits the
            // inactive arc gap. Applies to all modes including smooth -- gap
            // zones (driveways, walkways) must stay dry.
            if (!is_last_arc) {
                INFO("  Seg %d done -- closing valve for nozzle transit to seg %d",
                     ai+1, ai+2);
                valve_goto_ex(VALVE_CLOSED_DEG, 2.0f, 8000, false, -1);
                s_valve_last_dir = -1;
            }
        }  // arc loop

        // b364: every arc clipped to zero by the polygon -- ring isn't
        // reachable at this throw given the polygon shape. Mark it so the
        // scheduler doesn't waste subsequent passes re-trying. zone_clip_arc_to_polygon
        // is deterministic in (polygon, ring_throw) so re-checking would
        // produce the same result every pass.
        if (num_arcs > 0 && arcs_swept_this_ring == 0
                && ring < WATER_RUN_MAX_RINGS && !ring_unwaterable[ring]) {
            ring_unwaterable[ring] = true;
            INFO("Ring %d: all %d arc(s) clipped to zero by polygon -- "
                 "marked unwaterable, excluded from further passes.",
                 ring + 1, num_arcs);
        }

        // Continuous-sweep modes (smooth, gentle): replace theoretical nozzle_dps
        // with the speed actually observed by the encoder timer. ring_sweep_ms is
        // the sum of elapsed[] from every nozzle_sweep_encoder_gentle call for
        // this ring; dividing the total degrees swept by that time gives the
        // true average dps. This corrects depth_mm, which is computed post-loop
        // from r->dps -- without it the cumulative tracker would drift.
        if ((smooth_mode || gentle_mode) && ring_sweep_ms > 0 && ring_arc_deg_swept > 1.0f) {
            float actual_dps = ring_arc_deg_swept / (ring_sweep_ms / 1000.0f);
            INFO("  Ring %d actual dps: %.2f (theoretical %.2f, %.0f deg in %u ms)",
                 ring+1, actual_dps, nozzle_dps, ring_arc_deg_swept, ring_sweep_ms);
            nozzle_dps = actual_dps;
        }

        // Record actual ring data for /zone/last_water
        if (!ring_no_flow && ring < WATER_RUN_MAX_RINGS) {
            {
                float _avg = ring_psi_n > 0 ? ring_psi_sum / ring_psi_n : 0.0f;
                float _at  = _avg > 0.1f ? cal_pressure_to_throw_mm(_avg) : ring_throw;
                // Derive arc extents from last arc's seg_origin/seg_deg
                float _arc_s = cw ? seg_origin
                                  : fmodf(seg_origin - seg_deg + 360.0f, 360.0f);
                float _arc_e = cw ? fmodf(seg_origin + seg_deg, 360.0f)
                                  : seg_origin;
                s_last_water_run.rings[ring] = (water_ring_data_t){
                    .throw_mm        = ring_throw,
                    .avg_psi         = _avg,
                    .actual_throw_mm = _at,
                    // In smooth mode .dps is the encoder-measured actual speed
                    // (computed above); in other modes it is the theoretical target.
                    .dps             = nozzle_dps,
                    .active_deg      = active_count * (float)WATER_SECTOR_DEG,
                    .arc_start_deg   = _arc_s,
                    .arc_end_deg     = _arc_e,
                    .valve_deg       = valve_target_deg,  // b281: for supply back-calc
                };
                // b296: feed the scheduler's rolling supply estimate. Updated
                // every ring (not just per-pass) so the scheduler reacts to
                // pump cycles within a pass rather than lagging by a full
                // pass. Smooth mode only -- other modes don't consult it.
                if (smooth_mode && _avg > 0.1f && valve_target_deg > 0.5f) {
                    float _new_supply = supply_psi_from_pressure_map(
                                            valve_target_deg, _avg);
                    if (_new_supply > 0.1f) sched_supply_est = _new_supply;
                }
            }
        }
        // Adaptive: accumulate depth delivered this sweep.
        //   Smooth: passes 1+ only -- pass 0 is seeded below after throw-correction
        //           gating (so cold-start over/under-throw passes don't double-credit).
        //   Gentle: every pass, no throw correction -- credit each sweep directly.
        bool _accum_now = !ring_no_flow && ring < WATER_RUN_MAX_RINGS &&
                          ((smooth_mode && pass > 0) || gentle_mode);
        if (_accum_now) {
            water_ring_data_t *_r = &s_last_water_run.rings[ring];
            float _ro = ring_throw;
            float _ri = (ring == num_rings - 1) ? ring_throw * 0.92f
                                                 : ring_throws[ring + 1];
            smooth_cumulative_depth[ring] +=
                nozzle_precip_depth_mm(_ro, _ri, _r->dps, _r->avg_psi);
        }
        rings_done++;
        cw = !cw;
    }  // ring loop
        valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);

        // Adaptive: after pass 0, calibrate per-ring corrections (smooth only --
        // gentle has no throw correction) and seed the cumulative depth tracker.
        // After every pass, check whether all rings have reached their target
        // and update the time estimate.
        if ((smooth_mode || gentle_mode) && !s_water_abort) {
            const char *_mode_tag = smooth_mode ? "Smooth" : "Gentle";

            if (pass == 0) {
                // --- Pass 0: compute depth/pass rate for each ring ---
                // Used as the per-pass rate estimate for the adaptive termination
                // check. May be inaccurate if pass 0 ran cold (low supply PSI).
                float sum_dp = 0.0f; int cnt_dp = 0;
                for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                    water_ring_data_t *r = &s_last_water_run.rings[i];
                    if (r->throw_mm < 1.0f) continue;
                    float r_outer = r->throw_mm;
                    float r_inner = (i == num_rings - 1)
                                  ? r->throw_mm * 0.92f
                                  : s_last_water_run.rings[i + 1].throw_mm;
                    float dppass = nozzle_precip_depth_mm(r_outer, r_inner,
                                                          r->dps, r->avg_psi);
                    smooth_depth_per_pass[i] = dppass;
                    if (dppass > 0.005f) { sum_dp += dppass; cnt_dp++; }
                }
                float avg_dp = cnt_dp > 0 ? sum_dp / cnt_dp : 0.0f;
                INFO("%s pass 1 done: avg depth/pass %.3fmm, target %.3fmm",
                     _mode_tag, avg_dp, depth_mm);
            }

            // --- Iterative per-ring throw correction (smooth only, every pass) ---
            // Updates smooth_valve_corr[] using a blended throw-ratio so the
            // correction converges over multiple passes rather than locking in
            // one bad pass-0 measurement.
            // PSI gate: skip cold-start under-throw when avg_psi is below the
            // calibration minimum (psi_min). Below that the pressure sensor and
            // valve model are both unreliable -- pump still pressurizing.
            // Blend weights 0.6 old / 0.4 new: slower to respond to single-pass
            // PSI swings (e.g. supply pressure rises as inner rings drop out)
            // while still converging over the multi-pass run.
            if (smooth_mode) {
                for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                    water_ring_data_t *r = &s_last_water_run.rings[i];
                    if (r->throw_mm < 1.0f || r->actual_throw_mm < 100.0f) continue;
                    bool _underthrow = r->actual_throw_mm < r->throw_mm;
                    if (_underthrow && r->avg_psi < psi_min) continue; // cold-start, skip
                    // b303: raise upper clamp 1.4 -> 1.6 so persistently
                    // under-throwing rings (those at the physical reach edge)
                    // can drive valve_ring_throw far enough above cal_max to
                    // engage the VALVE_PEAK_DEG extrapolation in
                    // cal_throw_to_valve_deg. Lower clamp 0.5 unchanged --
                    // over-throw recovery doesn't need help.
                    float new_corr = fmaxf(0.5f, fminf(1.6f,
                                           r->throw_mm / r->actual_throw_mm));
                    float old_corr = smooth_valve_corr[i];
                    smooth_valve_corr[i] = old_corr * 0.6f + new_corr * 0.4f;
                    if (fabsf(smooth_valve_corr[i] - 1.0f) > 0.05f)
                        INFO("  Ring %d corr %.3f->%.3f (%.0f->%.0fmm, psi %.2f)",
                             i+1, old_corr, smooth_valve_corr[i],
                             r->throw_mm, r->actual_throw_mm, r->avg_psi);
                }

                if (pass == 0) {
                    // Seed cumulative depth from pass 0 (smooth only -- gentle
                    // already accumulated each sweep above).
                    // Credit only rings where throw was within 20% of target
                    // (|corr-1| <= 0.20). Excludes cold-start over-correction
                    // (corr ~1.4) AND severe overshoot (corr ~0.67) -- both cases
                    // mean water landed in the wrong ring.
                    for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                        if (fabsf(smooth_valve_corr[i] - 1.0f) <= 0.20f) {
                            smooth_cumulative_depth[i] = smooth_depth_per_pass[i];
                        } else {
                            smooth_cumulative_depth[i] = 0.0f;
                            INFO("  Ring %d: pass 0 depth not credited (corr %.3f)",
                                 i+1, smooth_valve_corr[i]);
                        }
                    }
                }
            }

            // --- Per-pass adaptive termination and estimate update ---
            // b297: a ring is "done" only when BOTH cumulative depth target
            // is met AND last actual_throw is within 10% of target. The
            // scheduler will re-pick under-thrown rings (at preferred high-
            // supply moments) until both conditions are met, or until the
            // outer pass cap (passes=30 for smooth) is hit.
            int rings_remaining = 0;
            float max_more_passes = 0.0f;
            for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                // b364: unwaterable rings produce no progress no matter how
                // many more passes we run -- exclude them from the remaining-
                // work count so the run can terminate early once everything
                // reachable has converged.
                if (ring_unwaterable[i]) continue;
                float deficit = depth_mm - smooth_cumulative_depth[i];
                bool depth_done = (deficit <= 0.0f);
                bool throw_done = true;
                if (smooth_mode) {
                    float _at = s_last_water_run.rings[i].actual_throw_mm;
                    if (_at >= 100.0f && ring_throws[i] >= 100.0f) {
                        throw_done = (_at >= ring_throws[i] * 0.90f);
                    }
                }
                if (depth_done && throw_done) continue;
                rings_remaining++;
                if (!depth_done) {
                    float dppass = smooth_depth_per_pass[i];
                    if (dppass > 0.005f) {
                        float np = ceilf(deficit / dppass);
                        if (np > max_more_passes) max_more_passes = np;
                    } else {
                        if (max_more_passes < 3.0f) max_more_passes = 3.0f;
                    }
                } else {
                    // b304: throw-only deficit needs more than 1-2 passes to
                    // converge corr to a useful value. Empirical from b297
                    // data: corr blend is 0.6 old / 0.4 new, so closing a
                    // 10-15% throw gap takes ~4-5 passes typically. Bumping
                    // from 2 to 4 reduces the case where est_min hits 0
                    // while the scheduler is still chasing throw deficits
                    // (observed on Zone b302: estimate hit 0 with 2-3 min
                    // of throw recovery still in progress).
                    if (max_more_passes < 4.0f) max_more_passes = 4.0f;
                }
            }
            if (rings_remaining == 0) {
                INFO("%s: all %d rings satisfied after pass %d -- done",
                     _mode_tag, num_rings, pass+1);
                break;  // exit pass loop early
            }
            INFO("%s pass %d done: %d/%d rings still need water"
                 " (~%.0f more pass(es))",
                 _mode_tag, pass+1, rings_remaining, num_rings, max_more_passes);
            // --- Per-ring cumulative depth dump (catch-cup calibration aid) ---
            // Logs each active ring's center throw distance and the model's
            // predicted cumulative depth at that radius. Compare against catch-
            // cup measurements (cups placed at the ring throw distances) to
            // validate NOZZLE_FLOW_K / depth claim per ring. Only meaningful
            // for adaptive modes that maintain smooth_cumulative_depth[]
            // (smooth pass 1+ and gentle every pass).
            {
                char _depbuf[400];
                int _off = 0;
                for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                    water_ring_data_t *_r = &s_last_water_run.rings[i];
                    if (_r->throw_mm < 1.0f) continue;
                    int _n = snprintf(_depbuf + _off, sizeof(_depbuf) - _off,
                                      " %d=%.1fm:%.2fmm",
                                      i + 1, _r->throw_mm / 1000.0f,
                                      smooth_cumulative_depth[i]);
                    if (_n < 0 || _n >= (int)(sizeof(_depbuf) - _off)) break;
                    _off += _n;
                }
                INFO("%s pass %d ring depths (r#=throw:cum_depth):%s",
                     _mode_tag, pass + 1, _depbuf);
            }
            // b302: per-pass adaptive estimate uses volume + measured overhead.
            //   pumping_remaining = remaining_volume / measured_flow_rate
            //   overhead_remaining = (elapsed - pumping_done)/passes_done
            //                        * max_more_passes
            // Flow rate is derived from measured avg_psi across rings that
            // have fired (orifice model). Overhead is back-computed from
            // total elapsed minus modeled pumping time, then extrapolated
            // forward at the same per-pass rate. This decouples the
            // estimate from the scheduler's choices and tracks the actual
            // run physics directly.
            TickType_t _now = xTaskGetTickCount();
            float _elapsed_s = (float)(_now - t_start)
                             / (float)configTICK_RATE_HZ;

            float _arc_frac = zone_arc_deg / 360.0f;
            float total_vol_L = 0.0f;
            float done_vol_L = 0.0f;
            float psi_sum = 0.0f; int psi_n = 0;
            for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
                if (ring_throws[i] < 1.0f) continue;
                float ro_m = ring_throws[i] / 1000.0f;
                float ri_m = (i == num_rings - 1)
                           ? (act_min_throw / 1000.0f)
                           : (ring_throws[i+1] / 1000.0f);
                float ring_area_m2 = (float)M_PI *
                                     (ro_m*ro_m - ri_m*ri_m) * _arc_frac;
                if (ring_area_m2 < 1e-4f) continue;
                total_vol_L += ring_area_m2 * depth_mm;
                float done_d = smooth_cumulative_depth[i];
                if (done_d < 0.0f) done_d = 0.0f;
                // Cap at 1.5x target to avoid overcounting from over-thrown
                // rings (otherwise we'd compute "negative remaining vol").
                if (done_d > depth_mm * 1.5f) done_d = depth_mm * 1.5f;
                done_vol_L += ring_area_m2 * done_d;

                water_ring_data_t *_r = &s_last_water_run.rings[i];
                if (_r->avg_psi > 0.5f) {
                    psi_sum += _r->avg_psi; psi_n++;
                }
            }
            float remaining_vol_L = total_vol_L - done_vol_L;
            if (remaining_vol_L < 0.0f) remaining_vol_L = 0.0f;

            float avg_psi = (psi_n > 0) ? (psi_sum / (float)psi_n) : 5.0f;
            float flow_lpm =
                NOZZLE_FLOW_K * powf(avg_psi, NOZZLE_FLOW_N) / 1000.0f;
            if (flow_lpm < 0.05f) flow_lpm = 1.0f;

            float pumping_remaining_s = (remaining_vol_L / flow_lpm) * 60.0f;
            float pumping_done_s = (done_vol_L / flow_lpm) * 60.0f;
            float overhead_so_far_s = _elapsed_s - pumping_done_s;
            if (overhead_so_far_s < 0.0f) overhead_so_far_s = 0.0f;
            float overhead_per_pass_s = (pass + 1 > 0)
                ? overhead_so_far_s / (float)(pass + 1)
                : 10.0f;
            // Floor per-pass overhead at 5s to avoid runaway-low estimates
            // if pumping_done_s overshoots elapsed (rounding / over-throw).
            if (overhead_per_pass_s < 5.0f) overhead_per_pass_s = 5.0f;
            float overhead_remaining_s = overhead_per_pass_s * max_more_passes;

            // b304: add a fixed cleanup tail (valve close, trace flush,
            // storage save, f() refit logging, LFS settle). On small zones
            // (Zone, 8 rings) this can be a substantial fraction of the
            // remaining time near end-of-run; on large zones it's negligible.
            const float CLEANUP_TAIL_S = 30.0f;
            float secs_remaining = pumping_remaining_s + overhead_remaining_s
                                 + CLEANUP_TAIL_S;
            s_water_est_min = (int)(secs_remaining / 60.0f) + 1;
            // b362: re-anchor the live tick-down so HA sees the corrected
            // estimate decrease through the upcoming pass too.
            s_eta_anchor_secs = secs_remaining;
            s_eta_anchor_tick = xTaskGetTickCount();
            INFO("%s pass %d done: vol %.1f/%.1fL @ %.2f psi (%.1f L/min), "
                 "rem pump %.0fs + ovh %.0fs (%.0fs/pass x %.0f) + tail %.0fs = %d min",
                 _mode_tag, pass + 1, done_vol_L, total_vol_L,
                 avg_psi, flow_lpm,
                 pumping_remaining_s, overhead_remaining_s,
                 overhead_per_pass_s, max_more_passes, CLEANUP_TAIL_S,
                 s_water_est_min);
        }
    }  // pass loop

abort:
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);

    // Smooth mode: flush per-(ring,sector) accumulators as summary records.
    // One record per sector, pass_type=0xFF.  time_ds repurposed: depth_mm × 100.
    // Works for both normal completion and early abort -- writes whatever was
    // accumulated up to the point of exit.
    //
    // b288: open the wbin file HERE rather than at run-start. The aggregate
    // burst (magic header + per-sector records, ~41 KB total) is the only
    // LittleFS write the smooth path needs. By deferring fopen until the
    // pass loop has fully exited and WiFi is in Quiet-mode (low-rate),
    // we keep the loop's tail free of metadata-relocation events that
    // race with WiFi PHY cache-disable.
    if ((smooth_mode || serpentine_mode) && s_smooth_accum_mode && s_smooth_wbin_path[0] && !s_water_csv_f) {
        s_water_csv_f = fopen(s_smooth_wbin_path, "wb");
        if (s_water_csv_f) {
            uint32_t _magic = WATER_BIN_MAGIC;
            if (fwrite(&_magic, sizeof(_magic), 1, s_water_csv_f) != 1) {
                ESP_LOGW(TAG, "Smooth aggregate: magic header write failed");
                fclose(s_water_csv_f); s_water_csv_f = NULL;
            }
        } else {
            ESP_LOGW(TAG, "Smooth aggregate: could not open %s", s_smooth_wbin_path);
        }
    }
    if ((smooth_mode || serpentine_mode) && s_smooth_accum_mode && s_water_csv_f) {
        int _aggr_n = 0;
        for (int _r = 0; _r < num_rings && _r < WATER_RUN_MAX_RINGS; _r++) {
            // Report the splash-true depth (display only -- control loop used
            // the raw geometric accumulation). See smooth_display_depth_mm.
            float _inner = (_r == num_rings - 1) ? ring_throws[_r] * 0.92f
                                                 : ring_throws[_r + 1];
            float _dep = smooth_display_depth_mm(smooth_cumulative_depth[_r],
                                                 ring_throws[_r], _inner);
            uint16_t _dep_ds = (uint16_t)fminf(_dep * 100.0f + 0.5f, 65535.0f);
            float _rt = ring_throws[_r];
            for (int _s = 0; _s < WATER_SECTORS; _s++) {
                const water_sector_accum_t *_a = &s_smooth_accum[_r][_s];
                if (!_a->count) continue;
                float _adeg  = (float)_a->sum_actual_deg_d / (10.0f * (float)_a->count);
                float _athr  = (float)_a->sum_actual_throw /          (float)_a->count;
                float _apsi  = (float)_a->sum_psi_c        / (100.0f * (float)_a->count);
                float _brad  = _adeg * (float)M_PI / 180.0f;
                float _tdeg  = (float)_s * WATER_SECTOR_DEG;
                float _trad  = _tdeg * (float)M_PI / 180.0f;
                water_row_t _wr = {
                    .time_ds             = _dep_ds,
                    .ring                = (uint8_t)_r,
                    .arc                 = 0,
                    .sector              = (uint16_t)_s,
                    .nozzle_deg_target_d = (uint16_t)(_tdeg * 10.0f + 0.5f),
                    .nozzle_deg_actual_d = (uint16_t)fminf(_adeg * 10.0f + 0.5f, 65535.0f),
                    .valve_deg_target_d  = 0,
                    .valve_deg_actual_d  = 0,
                    .throw_mm_target     = (uint16_t)fminf(_rt   + 0.5f, 65535.0f),
                    .throw_mm_actual     = (uint16_t)fminf(_athr + 0.5f, 65535.0f),
                    .psi_target_c        = 0,
                    .psi_actual_c        = (uint16_t)fminf(_apsi * 100.0f + 0.5f, 65535.0f),
                    .target_x  = (int16_t)fmaxf(-32767.f, fminf(_rt   * sinf(_trad), 32767.f)),
                    .target_y  = (int16_t)fmaxf(-32767.f, fminf(_rt   * cosf(_trad), 32767.f)),
                    .actual_x  = (int16_t)fmaxf(-32767.f, fminf(_athr * sinf(_brad), 32767.f)),
                    .actual_y  = (int16_t)fmaxf(-32767.f, fminf(_athr * cosf(_brad), 32767.f)),
                    .pass_type = 0xFF,
                    .active    = 1,
                };
                fwrite(&_wr, sizeof(_wr), 1, s_water_csv_f);
                _aggr_n++;
            }
        }
        INFO("Smooth aggregate: %d summary records written (~%u bytes)",
             _aggr_n, (unsigned)(_aggr_n * sizeof(water_row_t) + 4));
    }
    s_smooth_accum_mode = false;

    if (s_water_csv_f) { fclose(s_water_csv_f); s_water_csv_f = NULL; }
    motor_rail_off();

    // Compute per-ring precipitation depth for the water JSON / heatmap.
    // Adaptive modes (smooth, gentle): use the live cumulative tracker, which
    // sums actual depth from every pass. Smooth excludes pass-0 contribution
    // for overshooting rings; gentle credits every pass.
    // Other modes (pulse, demo): compute from last-pass dps/psi using flow model.
    for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
        water_ring_data_t *r = &s_last_water_run.rings[i];
        float r_inner = (i == num_rings - 1)
                      ? r->throw_mm * 0.92f
                      : s_last_water_run.rings[i + 1].throw_mm;
        if ((smooth_mode || gentle_mode || serpentine_mode) && smooth_cumulative_depth[i] > 0.005f) {
            // Splash-true depth for the heatmap (control used raw geometric).
            r->depth_mm = smooth_display_depth_mm(smooth_cumulative_depth[i],
                                                  r->throw_mm, r_inner);
        } else {
            r->depth_mm = smooth_display_depth_mm(
                nozzle_precip_depth_mm(r->throw_mm, r_inner, r->dps, r->avg_psi),
                r->throw_mm, r_inner);
        }
    }

    // b281+b284: per-ring supply pressure estimation + run-level summary.
    // For each ring, back-compute the (pre-valve) supply pressure that the
    // pump/tank was providing while that ring was being watered, using the
    // valve transmission function f(valve_deg) derived from the device's
    // pressure_map_t cal (b284 switched from zone-perim cal -- denser data,
    // single source-of-truth, cleaner interpolation). Surfaces well-pump
    // cycling dynamics to HA without needing a separate supply-side sensor.
    {
        float sup_min = 1e9f, sup_max = -1e9f, sup_sum = 0.0f;
        int sup_n = 0;
        for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
            water_ring_data_t *r = &s_last_water_run.rings[i];
            r->supply_psi_est = 0.0f;
            if (r->avg_psi < 0.1f || r->valve_deg < 1.0f) continue;
            float sup = supply_psi_from_pressure_map(r->valve_deg, r->avg_psi);
            if (sup < 0.1f) continue;  // cal couldn't classify
            r->supply_psi_est = sup;
            if (sup < sup_min) sup_min = sup;
            if (sup > sup_max) sup_max = sup;
            sup_sum += sup;
            sup_n++;
        }
        if (sup_n > 0) {
            s_last_water_run.supply_psi_min = sup_min;
            s_last_water_run.supply_psi_max = sup_max;
            s_last_water_run.supply_psi_avg = sup_sum / sup_n;
            INFO("Supply pressure (back-computed from f(valve_deg)): "
                 "min=%.2f max=%.2f avg=%.2f PSI across %d active rings",
                 sup_min, sup_max, sup_sum / sup_n, sup_n);
        } else {
            s_last_water_run.supply_psi_min = 0.0f;
            s_last_water_run.supply_psi_max = 0.0f;
            s_last_water_run.supply_psi_avg = 0.0f;
        }

        // b282+b284: Gap analysis -- flag rings whose under-throw correlates
        // with depleted supply. b284 switches the cal-time supply reference
        // to the pressure_map_t cal's max-pressure anchor (same source as
        // supply_psi_from_pressure_map above), so the comparison is honest.
        int n_flagged = 0;
        float cal_supply = cal_anchor_supply_pressure_map();
        for (int i = 0; i < num_rings && i < WATER_RUN_MAX_RINGS; i++) {
            water_ring_data_t *r = &s_last_water_run.rings[i];
            if (ring_is_supply_limited(r, cal_supply)) {
                float throw_pct  = 100.0f * r->actual_throw_mm / r->throw_mm;
                float supply_pct = 100.0f * r->supply_psi_est  / cal_supply;
                INFO("Ring %2d supply-limited (would cleanup-retry): "
                     "throw %.0f%% of target, supply %.0f%% of cal "
                     "(actual_throw=%.0fmm target=%.0fmm, "
                     "supply_est=%.2f cal_supply=%.2f PSI)",
                     i+1, throw_pct, supply_pct,
                     r->actual_throw_mm, r->throw_mm,
                     r->supply_psi_est, cal_supply);
                n_flagged++;
            }
        }
        s_last_water_run.rings_supply_limited = (uint8_t)n_flagged;
        if (cal_supply > 0.5f) {
            INFO("Gap analysis: %d ring(s) supply-limited "
                 "(cal-time supply ref: %.2f PSI)", n_flagged, cal_supply);
        } else {
            INFO("Gap analysis: skipped (no cal supply reference)");
        }
    }

    // b283: finalize pressure trace -- writes /lfs/water/water_NNN_trace.csv
    // and logs sample/ring counts. No-op if recorder wasn't armed
    // (pulse/demo modes).
    // b284: also run the on-device f(valve_deg) refit + log results. Logs
    // only -- the refined values aren't persisted or used yet (b285's job).
    // The log lets us validate the on-device refit matches the off-device
    // analyze_water_trace.py output before trusting it to drive behavior.
    if ((smooth_mode || gentle_mode || serpentine_mode) && !demo_mode && rings_done > 0) {
        water_trace_save(s_water_zone_id);
        water_trace_refit_f_and_log();
    } else if (s_trace_active) {
        // Defensive: in case watering aborted before save reached, still
        // clear the active flag so a subsequent run starts clean.
        s_trace_active = false;
    }

    // Persist last-watering data: LittleFS primary, NVS as fallback
    if (!demo_mode && rings_done > 0) {
        s_last_water_run.fw_build      = FW_BUILD;
        s_last_water_run.num_rings     = (uint16_t)num_rings;
        s_last_water_run.arc_start_deg = zone_arc_start;
        s_last_water_run.arc_span_deg  = zone_arc_deg;
        s_last_water_run.target_depth_mm = depth_mm;  // N x 1/8" target (heatmap baseline)
        // Record actual wall-clock duration even for aborted runs so HA
        // can report partial run time. (Previously: 0 for aborted runs
        // so it didn't poison the UI's "estimated time" cache — the UI
        // path can still detect aborted=true and skip stale aborts.)
        float _elapsed = (float)(xTaskGetTickCount() - t_start)
                         * portTICK_PERIOD_MS / 1000.0f;
        s_last_water_run.total_time_s = s_water_abort ? 0.0f : _elapsed;
        if (storage_water_save(s_water_zone_id, &s_last_water_run) != ESP_OK) {
            water_run_save_nvs(&s_last_water_run);  // FS unavailable
        }
    }

    // b287: Exit Watering Quiet mode. Wait briefly so any in-flight LFS
    // operations from storage_water_save above can fully settle before
    // ESPHome's sensor pipe wakes back up and starts pushing pressure /
    // battery / throw updates again (which would otherwise hit the WiFi/
    // flash race window). Then drop priority back to the normal 10.
    // b292: also stop the RAM log capture and restart WiFi here, in the
    // same settle window. Order matters: stop log capture FIRST (so the
    // "WiFi resume" line doesn't go into the captured buffer -- HA will
    // see that line live), then resume WiFi. ESPHome's API will
    // reconnect to HA over the next 3-10 sec; the watering_complete
    // event payload (b281/b282 supply data) will fire once reconnect
    // completes, and /zone/last_log will then serve the run's captured
    // log on demand.
    if (s_watering_quiet) {
        INFO("Watering complete -- holding Quiet mode 3s for LFS settle");
        vTaskDelay(pdMS_TO_TICKS(3000));
        watering_log_capture_stop();
        // b295: removed watering_wifi_resume() -- WiFi was never paused
        // since b294 made it unnecessary. See watering_wifi_pause site
        // for full rationale.
        s_watering_quiet = false;
        vTaskPrioritySet(NULL, 10);
        INFO("Watering Quiet OFF: sensor publishes resume, task priority -> 10");
    }
    // Snapshot completion status for the HA "watering_complete" event.
    // s_water_abort is reset at next start, so we capture the value now.
    s_last_water_aborted = s_water_abort;
    s_last_water_mode    = s_web_water_mode;
    s_last_water_zone_id = s_water_zone_id;
    // If no granular reason was set, the run completed normally even if
    // s_water_abort was flipped late by stop_and_close-style teardown.
    s_last_water_status_code = s_water_status_code;
    s_last_water_finish_epoch = time(NULL);
    // Persist the trio across deep-sleep wake so the landing-page
    // "Last completed" tag survives. NVS write is ~6 ms; happens
    // at most once per watering completion.
    last_water_save_nvs();
    int expected_rings = num_rings * passes;
    if (demo_mode) {
        INFO("Demo complete -- %d/%d rings.", rings_done, num_rings);
    } else if (serpentine_mode) {
        // b423: serpentine skips inner rings (Phase A) so the expected_rings
        // identity doesn't hold; also must never fall into the pulse
        // cleanup-pass branch below.
        INFO("Serpentine %s -- %d ring sweep(s), %.3fmm target.",
             s_water_abort ? "stopped" : "complete", rings_done, depth_mm);
    } else if (rings_done == expected_rings) {
        INFO("Watering complete. %.3fmm applied. ~%.0f min.", depth_mm, total_time);
        if (gentle_mode) {
            INFO("Gentle mode: %d continuous passes applied.", passes);
        } else if (smooth_mode) {
            INFO("Smooth mode complete -- review heatmap before running cleanup passes.");
        } else {
            INFO("Analysing coverage for cleanup passes...");
            for (int _cp = 1; _cp <= CLEANUP_MAX_PASSES && !s_water_abort; _cp++) {
                s_water_cleanup_pass = _cp;
                water_cleanup_pass(&zone, zone_arc_start, zone_arc_deg,
                                   act_max_throw, act_min_throw,
                                   depth_mm, psi_min, psi_max, _cp);
            }
            s_water_cleanup_pass = 0;
        }
    } else {
        INFO("Stopped after %d/%d rings.", rings_done, expected_rings);
    }
    s_web_water_mode = 0;
    s_water_est_min  = 0;
    s_eta_anchor_tick = 0;   // b362: stop ticking once the run is done
}

// -----------------------------------------------------------------------
// water_cleanup_pass: re-water rings that delivered < CLEANUP_THRESHOLD.
// Fix 1: accumulated_depth[] tracks depth added by previous cleanup passes
//         so re-analysis sees the running total, not just the initial run.
// Fix 2: throw accuracy (actual_throw_mm vs throw_mm) is factored in so
//         rings with correct pressure but wrong throw are still flagged.
// -----------------------------------------------------------------------
static void water_cleanup_pass(
    const zone_perimeter_t *zone,
    float zone_arc_start, float zone_arc_deg,
    float act_max_throw,  float act_min_throw,
    float depth_mm,
    float psi_min,        float psi_max,
    int   pass_num)
{
    const water_run_t *run = &s_last_water_run;
    int n = (int)run->num_rings;
    if (n < 1) return;

    // Accumulated extra depth per ring from previous cleanup passes.
    // Static so it persists across calls within the same watering session.
    // Reset to zero on the first pass.
    static float s_cleanup_extra_mm[WATER_RUN_MAX_RINGS];
    if (pass_num == 1) memset(s_cleanup_extra_mm, 0, sizeof(s_cleanup_extra_mm));

    // Helper: compute depth delivered by a ring given its stored data.
    // Uses actual_throw_mm to detect throw-error rings (correct PSI, wrong throw).
    // A ring that threw 85% of target distance gets credit for less area coverage.
    #define ring_depth(r, rO_m, rI_m) (         (r)->avg_psi > 0.05f && (r)->dps > 0.01f && (r)->active_deg > 0.5f ?         (0.12f * ((r)->avg_psi / 5.034f * 100.0f) + 1.0f) / 60.0f / 1000.0f         * ((r)->active_deg / (r)->dps)         / fmaxf(3.14159f * ((rO_m)*(rO_m) - (rI_m)*(rI_m)) * ((r)->active_deg / 360.0f), 1e-4f)         * 1000.0f         * fminf(1.0f, (r)->actual_throw_mm > 100.0f                       ? (r)->actual_throw_mm / fmaxf((r)->throw_mm, 1.0f) : 1.0f)         : 0.0f )

    // --- Analyse each ring (initial depth + any prior cleanup depth) ---
    bool any_under = false;
    for (int i = 0; i < n; i++) {
        const water_ring_data_t *r = &run->rings[i];
        float rO_m = ((i == 0) ? r->throw_mm * 1.08f
                                : (run->rings[i-1].throw_mm + r->throw_mm) * 0.5f) / 1000.0f;
        float rI_m = ((i == n-1) ? r->throw_mm * 0.92f
                                  : (r->throw_mm + run->rings[i+1].throw_mm) * 0.5f) / 1000.0f;
        float depth_initial = ring_depth(r, rO_m, rI_m);
        float depth_total   = depth_initial + s_cleanup_extra_mm[i];
        float pct = depth_total / depth_mm;
        INFO("  Ring %2d: %.2f+%.2f=%.2fmm / %.2fmm = %.0f%%%s",
             i+1, depth_initial, s_cleanup_extra_mm[i], depth_total, depth_mm,
             pct * 100.0f, pct < CLEANUP_THRESHOLD ? " <-- UNDER" : "");
        if (pct < CLEANUP_THRESHOLD) any_under = true;
    }

    if (!any_under) {
        INFO("Cleanup pass %d: all rings >= %.0f%% -- done.",
             pass_num, CLEANUP_THRESHOLD * 100.0f);
        return;
    }
    INFO("Starting cleanup pass %d...", pass_num);

    // --- Open CSV in append mode ---
    s_csv_pass_type = (uint8_t)(pass_num + 1);
    if (storage_ready()) {
        char _csv_path[64]; water_csv_path_find(s_water_zone_id, _csv_path, sizeof(_csv_path));
        s_water_csv_f = fopen(_csv_path, "ab");
        if (!s_water_csv_f)
            ESP_LOGW(TAG, "cleanup: could not append to %s", _csv_path);
    }

    sensor_rail_on();
    motor_rail_on();
    adc_setup();
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 12000, false);
    s_valve_last_dir  = -1;
    s_nozzle_last_dir = 0;

    TickType_t t_start = xTaskGetTickCount();
    int rings_cleaned  = 0;
    bool cw = true;

    for (int i = 0; i < n; i++) {
        if (uart_getchar(0) != 0 || s_water_abort) break;
        TOUCH_ACTIVITY();

        const water_ring_data_t *r = &run->rings[i];
        float rO_m = ((i == 0) ? r->throw_mm * 1.08f
                                : (run->rings[i-1].throw_mm + r->throw_mm) * 0.5f) / 1000.0f;
        float rI_m = ((i == n-1) ? r->throw_mm * 0.92f
                                  : (r->throw_mm + run->rings[i+1].throw_mm) * 0.5f) / 1000.0f;
        float depth_total = ring_depth(r, rO_m, rI_m) + s_cleanup_extra_mm[i];
        if (depth_total / depth_mm >= CLEANUP_THRESHOLD) { cw = !cw; continue; }

        float ring_throw   = r->throw_mm;
        float target_psi_r = cal_throw_to_psi(ring_throw);
        float valve_deg_r  = cal_pressure_to_valve_deg(target_psi_r);
        float seg_origin   = cw ? zone_arc_start
                                : fmodf(zone_arc_start + zone_arc_deg, 360.0f);
        float seg_deg_r    = zone_arc_deg;
        float area_m2      = 3.14159f * (rO_m*rO_m - rI_m*rI_m) * (zone_arc_deg / 360.0f);
        float ring_pct_r   = ring_throw / act_max_throw * 100.0f;
        float deficit_mm   = fmaxf(0.1f, depth_mm - depth_total);
        float t_min_r      = cal_watering_time_min(area_m2, deficit_mm, ring_pct_r);
        if (t_min_r < 0.05f) t_min_r = 0.05f;
        float nozzle_dps   = fmaxf(0.3f, fminf(zone_arc_deg / (t_min_r * 60.0f), 5.0f));

        INFO("  Cleanup ring %d: throw=%.0fmm %s %.2fdps",
             i+1, ring_throw, cw ? "CW" : "CCW", nozzle_dps);

        valve_goto_ex(valve_deg_r, 0.3f, 8000, false,
                      valve_deg_r > VALVE_CLOSED_DEG ? 1 : -1);
        nozzle_goto(seg_origin, 1.5f, 10000, false);
        s_nozzle_last_dir = cw ? 1 : -1;
        water_hold_pressure(target_psi_r, psi_min, psi_max);
        vTaskDelay(pdMS_TO_TICKS(300));

        speed_map_t spd = {0};
        bool have_spd = (spd_load_primary(&spd) == ESP_OK && spd.num_points >= 2);
        uint16_t eff_duty = (have_spd && spd.jog_pulse_duty > 0) ? spd.jog_pulse_duty : 200;
        uint32_t eff_ms   = (have_spd && spd.jog_pulse_ms > 0) ? (uint32_t)spd.jog_pulse_ms : 80;

        float psi_sum = 0.0f; int psi_n = 0;
        nozzle_sweep_pulse(seg_origin, seg_deg_r, cw, nozzle_dps,
                           eff_duty, eff_ms, t_start, i+1, 1, 0,
                           valve_deg_r, ring_throw, &psi_sum, &psi_n, false);

        // Accumulate delivered depth for next pass re-analysis
        if (psi_n > 0) {
            float avg_psi_c  = psi_sum / psi_n;
            float flow_c     = 0.12f * (avg_psi_c / 5.034f * 100.0f) + 1.0f;
            float depth_c    = (flow_c / 60.0f / 1000.0f)
                             * (zone_arc_deg / nozzle_dps)
                             / fmaxf(area_m2, 1e-4f) * 1000.0f;
            s_cleanup_extra_mm[i] += depth_c;
        }
        rings_cleaned++;
        cw = !cw;
    }

    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    if (s_water_csv_f) { fclose(s_water_csv_f); s_water_csv_f = NULL; }
    motor_rail_off();
    INFO("Cleanup pass %d done: %d ring(s) re-watered.", pass_num, rings_cleaned);
    #undef ring_depth
}

// Web-triggered watering: mode 1-4 matches menu, 99 = demo
static void phase_water_zone_mode(int mode)
{
    if (mode == WATER_MODE_CHASE) {
        // b311: chase mode bypasses the depth/ring/pass machinery entirely.
        phase_chase_water_zone();
        return;
    }
    if (mode==99) {
        // Demo: temporarily set demo_mode via a shim -- just call with demo flag
        // For now reuse interactive path with demo selection
        s_web_water_mode = 99;  // signal to phase_water_zone
    }
    // Patch: store mode in a global, let phase_water_zone read it
    // This avoids duplicating the entire watering function.
    // The existing phase_water_zone reads s_web_water_mode.
    phase_water_zone();
}
// Return minimum throw_mm from zone perimeter inner boundary points.
// Falls back to cal_get_min_throw_mm() if no throw data stored.

static float zone_get_psi_max(void) {
    pressure_map_t _c={0};
    if(cal_load_primary(&_c)==ESP_OK && _c.num_points>0) {
        float mx=_c.pressure_psi[0];
        for(int i=1;i<_c.num_points;i++) if(_c.pressure_psi[i]>mx) mx=_c.pressure_psi[i];
        return mx;
    }
    return 7.2f;
}

// -----------------------------------------------------------------------

// Encoder health scan: sweep valve slowly across full range and log
// position, AGC, and status as CSV. AGC is inversely proportional to
// field strength (lower = stronger). A sinusoidal variation across the
// sweep indicates magnet eccentricity; a sudden dip/spike at one angle
// indicates a magnetic anomaly. Flat AGC = healthy mount.
// Also logs motor current so dead spots (low mA, no movement) are visible.
static void phase_encoder_health(void)
{
    INFO("Encoder health scan -- sweeping valve open then closed.");
    INFO("CSV: direction,valve_deg,agc,status,current_ma");
    tprintf("direction,valve_deg,agc,status,current_ma\r\n");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(300));

    // Two passes: opening (CW) then closing (CCW)
    for (int pass = 0; pass < 2; pass++) {
        const char *dir = pass == 0 ? "open" : "close";
        gpio_num_t drv = pass == 0 ? GPIO_VFWD : GPIO_VREV;
        gpio_num_t off = pass == 0 ? GPIO_VREV : GPIO_VFWD;

        gpio_set_level(off, 0);
        gpio_set_level(GPIO_K, 0);
        gpio_set_level(drv, 1);

        uint32_t elapsed = 0;
        float prev_deg = -1;

        while (elapsed < 15000) {
            vTaskDelay(pdMS_TO_TICKS(20));
            elapsed += 20;
            if (uart_getchar(0) != 0 || s_water_abort) goto abort;

            uint16_t raw = 0;
            uint8_t agc = 0, status = 0;
            as5600_read(ADDR_AS5600L, &raw, &agc, &status);
            float deg = raw * (360.0f / 4096.0f);

            uint32_t cur_mv = adc_mv(ADC_CH_VCUR);
            float ma = CURRENT_MA(cur_mv);

            // Log every 0.5 deg of movement to keep output manageable
            if (prev_deg < 0 || fabsf(deg - prev_deg) >= 0.5f) {
                tprintf("%s,%.2f,%d,0x%02x,%.0f\r\n",
                        dir, deg, agc, status, ma);
                prev_deg = deg;
            }

            // Stop at mechanical limits
            if (pass == 0 && deg >= VALVE_OPEN_DEG  - 0.5f) break;
            if (pass == 1 && deg <= VALVE_CLOSED_DEG + 0.5f) break;
        }

        // Brief brake between passes
        gpio_set_level(drv, 0);
        gpio_set_level(GPIO_K, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(GPIO_K, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    INFO("Scan complete. Paste CSV into a spreadsheet and plot AGC vs valve_deg.");
    INFO("Look for: sinusoidal variation (eccentricity) or step dip (dead spot).");

abort:
    gpio_set_level(GPIO_VFWD, 0);
    gpio_set_level(GPIO_VREV, 0);
    gpio_set_level(GPIO_K, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(GPIO_K, 0);
    motor_rail_off();
    sensor_rail_off();
}

// Valve jog resolution explorer
// Finds minimum pulse duration / duty that produces reliable position change.
// Logs CSV of: duty_pct, pulse_ms, pos_before_deg, pos_after_deg,
//              delta_deg, psi_before, psi_after to terminal.
// Useful for tuning valve open-loop step resolution.
// -----------------------------------------------------------------------
// ---------------------------------------------------------------------------
// EXP (2026-06-06): raw constant-duty valve drive (no position loop), used by
// phase_valve_hysteresis to time first-motion on reversal. MCPWM group 1 so it
// never clashes with valve_goto_direct (group 0).
// ---------------------------------------------------------------------------
typedef struct {
    mcpwm_timer_handle_t t; mcpwm_oper_handle_t o;
    mcpwm_cmpr_handle_t  c; mcpwm_gen_handle_t  g;
    gpio_num_t drive, dir;
} exp_drv_t;

static void exp_drv_start(exp_drv_t *d, bool closing, uint16_t duty)
{
    // Ground truth (valve_goto_direct): VFWD decreases angle, VREV increases it.
    // CLOSED=231 (low) .. OPEN=308 (high), so closing = decreasing angle = VFWD,
    // opening = increasing angle = VREV. (b414 had these swapped, which inverted
    // Sections A and C; fixed b415.)
    d->drive = closing ? VALVE_PIN_FWD : VALVE_PIN_REV;
    d->dir   = closing ? VALVE_PIN_REV : VALVE_PIN_FWD;
    gpio_set_direction(d->dir, GPIO_MODE_OUTPUT);
    gpio_set_level(d->dir, 0);
    gpio_set_level(GPIO_K, 0);              // release brake (coast-capable)
    mcpwm_timer_config_t tc = { .group_id=1, .clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz=10000000, .period_ticks=500, .count_mode=MCPWM_TIMER_COUNT_MODE_UP };
    mcpwm_new_timer(&tc, &d->t);
    mcpwm_operator_config_t oc = { .group_id=1 };
    mcpwm_new_operator(&oc, &d->o);
    mcpwm_operator_connect_timer(d->o, d->t);
    mcpwm_comparator_config_t cc = { .flags.update_cmp_on_tez=true };
    mcpwm_new_comparator(d->o, &cc, &d->c);
    mcpwm_generator_config_t gc = { .gen_gpio_num=d->drive };
    mcpwm_new_generator(d->o, &gc, &d->g);
    mcpwm_generator_set_action_on_timer_event(d->g,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(d->g,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       d->c, MCPWM_GEN_ACTION_LOW));
    mcpwm_comparator_set_compare_value(d->c, duty);
    mcpwm_timer_enable(d->t);
    mcpwm_timer_start_stop(d->t, MCPWM_TIMER_START_NO_STOP);
}

static void exp_drv_stop(exp_drv_t *d)
{
    mcpwm_timer_start_stop(d->t, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(d->t);
    mcpwm_del_generator(d->g);
    mcpwm_del_comparator(d->c);
    mcpwm_del_operator(d->o);
    mcpwm_del_timer(d->t);
    gpio_set_level(d->drive, 0);
    gpio_set_level(d->dir, 0);
}

// EXP: quantify the two real costs of gear lash given an output-shaft encoder.
//   Section 1/A (DRY): reversal dead-time -- how long the motor spins after a
//     direction flip before the ball (encoder) actually moves. This is the
//     penalty that eats the 600ms per-pulse trim budget on reversals. Multiply
//     by motor deg/s (speed cal) to get motor-side lost motion.
//   Section 2/B (WET): pressure hysteresis -- settled PSI at the SAME commanded
//     angle approached from below vs from above. This is exactly what the 15deg
//     consistent-approach overshoot exists to suppress; if it's small, that
//     overshoot is overkill and can shrink (faster, smoother trim).
//
// Non-interactive: emits each row via INFO("LASHCSV,..") so results land in BOTH
// the UART console AND the RAM log buffer -- fetch over the network with
// `GET /zone/last_log` and filter for lines containing "LASHCSV,". Honors
// s_water_abort for an emergency stop. Caller owns the busy interlock.
static void exp_lash_run(int section)
{
    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    const float    DPC       = 360.0f / 4096.0f;
    const uint16_t LOW_DUTY  = 160;   // gentle: reveal first motion, not a lurch
    const float    start_deg = VALVE_CAL_START_DEG;  // geared region, below snap
    uint16_t raw = 0;

    if (section == 1) {
        // ---- Section A: reversal dead-time (DRY) ----
        INFO("LASH Section A: reversal dead-time (water should be OFF)");
        INFO("LASHCSV,section,trial,deadtime_ms,start_deg");
        for (int trial = 0; trial < 5 && !s_water_abort; trial++) {
            TOUCH_ACTIVITY();
            // Seat the slack on the OPENING side, then settle.
            valve_goto_direct(start_deg + 4.0f, 0.5f, 8000, false);
            vTaskDelay(pdMS_TO_TICKS(600));
            as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
            float p0 = raw * DPC;

            // Reverse (closing) at low duty; time until output moves >= 0.5 deg.
            exp_drv_t d;
            exp_drv_start(&d, true /*closing*/, LOW_DUTY);
            TickType_t t0 = xTaskGetTickCount();
            uint32_t dead_ms = 9999;
            while (((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS) < 1500) {
                vTaskDelay(pdMS_TO_TICKS(3));
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float dm = raw * DPC - p0;
                if (dm >  180.0f) dm -= 360.0f;
                if (dm < -180.0f) dm += 360.0f;
                if (fabsf(dm) >= 0.5f) {
                    dead_ms = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
                    break;
                }
            }
            exp_drv_stop(&d);
            vTaskDelay(pdMS_TO_TICKS(400));
            INFO("LASHCSV,A,%d,%lu,%.2f", trial + 1, (unsigned long)dead_ms, p0);
        }
        INFO("LASH A done. dead_ms x motor deg/s ~= motor-side lost motion.");
    } else if (section == 2) {
        // ---- Section B: pressure hysteresis (WET) ----
        INFO("LASH Section B: pressure hysteresis (water should be ON)");
        INFO("LASHCSV,section,target_deg,pos_up,psi_up,pos_down,psi_down,psi_hyst");
        const float lo = VALVE_CAL_START_DEG + 3.0f;
        const float hi = VALVE_OPEN_DEG - 3.0f;
        const int   N  = 6;
        for (int i = 0; i < N && !s_water_abort; i++) {
            TOUCH_ACTIVITY();
            float tgt = lo + (hi - lo) * i / (float)(N - 1);
            // Skip the hydrodynamic snap zone -- its hysteresis is a separate beast.
            if (tgt >= VALVE_FRICTION_LO && tgt <= VALVE_FRICTION_HI) continue;

            float below = tgt - 8.0f; if (below < VALVE_CAL_START_DEG) below = VALVE_CAL_START_DEG;
            float above = tgt + 8.0f; if (above > VALVE_OPEN_DEG)      above = VALVE_OPEN_DEG;

            // Approach from BELOW (final motion = opening).
            valve_goto_direct(below, 0.5f, 8000, false);
            vTaskDelay(pdMS_TO_TICKS(300));
            valve_goto_direct(tgt, 0.3f, 8000, false);
            vTaskDelay(pdMS_TO_TICKS(900));
            as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
            float pos_up = raw * DPC, psi_up = -1.0f;
            mprls_read_quiet(&psi_up);

            // Approach from ABOVE (final motion = closing).
            valve_goto_direct(above, 0.5f, 8000, false);
            vTaskDelay(pdMS_TO_TICKS(300));
            valve_goto_direct(tgt, 0.3f, 8000, false);
            vTaskDelay(pdMS_TO_TICKS(900));
            as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
            float pos_dn = raw * DPC, psi_dn = -1.0f;
            mprls_read_quiet(&psi_dn);

            INFO("LASHCSV,B,%.1f,%.2f,%.3f,%.2f,%.3f,%.3f",
                 tgt, pos_up, psi_up, pos_dn, psi_dn,
                 (psi_up >= 0 && psi_dn >= 0) ? (psi_up - psi_dn) : -99.0f);
        }
        INFO("LASH B done. psi_hyst large => 15deg approach earns keep; small => shrink it.");
    } else if (section == 3) {
        // ---- Section C: coast distance per direction (predictive-braking check) ----
        // valve_goto_direct cuts the motor when |vel|*COAST_S >= remaining error
        // (COAST_S=0.04). If the real coast differs by direction, the stop lands
        // short (closing) or long. Measure coast_deg vs pre-cut velocity to get an
        // effective COAST_S per direction. Run in the clean upper region (above the
        // 286 friction HI) so the snap zone doesn't contaminate the coast.
        // Best run DRY; water load changes coast (label the run accordingly).
        INFO("LASH Section C: coast per direction (clean region 288-308 deg)");
        INFO("LASHCSV,section,dir,duty,v_before_dps,coast_deg,coast_s_eff");
        const uint16_t duties[] = {200, 300, 400};
        for (int di = 0; di < 3 && !s_water_abort; di++) {
            for (int dir = 0; dir < 2 && !s_water_abort; dir++) {
                bool closing = (dir == 1);               // closing = decreasing deg
                TOUCH_ACTIVITY();
                // Home with room to drive+coast without hitting a limit.
                float home = closing ? (VALVE_OPEN_DEG - 2.0f)
                                     : (VALVE_FRICTION_HI + 2.0f);
                valve_goto(home, 0.5f, 10000, false);
                vTaskDelay(pdMS_TO_TICKS(500));

                // Kick to break stiction, then drop to the run duty and settle.
                exp_drv_t d;
                exp_drv_start(&d, closing, 480);
                vTaskDelay(pdMS_TO_TICKS(30));
                mcpwm_comparator_set_compare_value(d.c, duties[di]);
                vTaskDelay(pdMS_TO_TICKS(160));

                // Velocity over a 30ms window just before the cut.
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float p_prev = raw * DPC;
                vTaskDelay(pdMS_TO_TICKS(30));
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float p_cut = raw * DPC;
                float dv = p_cut - p_prev;
                if (dv >  180.0f) dv -= 360.0f;
                if (dv < -180.0f) dv += 360.0f;
                float v_before = dv / 0.030f;            // deg/s (signed)

                // CUT to coast (exp_drv_stop leaves the brake released).
                exp_drv_stop(&d);

                // Track until motion stops (4 consecutive <0.03 deg reads).
                float p_last = p_cut; int still = 0;
                TickType_t t0 = xTaskGetTickCount();
                while (((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS) < 800) {
                    vTaskDelay(pdMS_TO_TICKS(5));
                    as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                    float pp = raw * DPC;
                    float dd = pp - p_last;
                    if (dd >  180.0f) dd -= 360.0f;
                    if (dd < -180.0f) dd += 360.0f;
                    p_last = pp;
                    if (fabsf(dd) < 0.03f) { if (++still >= 4) break; } else still = 0;
                }
                float coast = p_last - p_cut;
                if (coast >  180.0f) coast -= 360.0f;
                if (coast < -180.0f) coast += 360.0f;
                float seff = (fabsf(v_before) > 1.0f) ? fabsf(coast) / fabsf(v_before) : 0.0f;
                INFO("LASHCSV,C,%s,%u,%.1f,%.2f,%.4f",
                     closing ? "close" : "open", (unsigned)duties[di],
                     v_before, coast, seff);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }
        INFO("LASH C done. Compare coast_s_eff open vs close vs COAST_S=0.04.");
    }

    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);
    INFO("LASH valve closed; section %d complete.", section);
}

// EXP background task -- mirrors water_task's proven shape: raise priority to
// reduce timing jitter, arm the RAM log capture (NO log streaming, WiFi stays
// up -- see b295), run, then tear down. Triggered by POST /exp/lash.
static void exp_task(void *arg)
{
    (void)arg;
    s_water_abort = false;
    UBaseType_t old_prio = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, 15);
    watering_log_capture_start();
    INFO("LASH experiment start: section %d", s_exp_section);
    exp_lash_run(s_exp_section);
    INFO("LASH experiment end: section %d", s_exp_section);
    watering_log_capture_stop();
    vTaskPrioritySet(NULL, old_prio);
    s_exp_section = 0;
    s_exp_running = false;
    vTaskDelete(NULL);
}

// Terminal-menu wrapper (still works over the USB-UART adapter). Network users
// should POST /exp/lash?section=A|B instead.
static void phase_valve_hysteresis(void)
{
    STEP("Valve Reversal Dead-Time + Hysteresis (EXP)");
    INFO("Rows tagged LASHCSV. Section A=dead-time (DRY), B=hysteresis (WET).");
    INFO("Ensure water OFF. Enter to run Section A, q to abort.");
    while (true) {
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q') { INFO("Aborted."); return; }
        if (c == '\r' || c == '\n') break;
    }
    exp_lash_run(1);
    INFO("Turn water ON. Enter to run Section B, q to skip.");
    while (true) {
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q') { INFO("Skipped Section B."); return; }
        if (c == '\r' || c == '\n') break;
    }
    exp_lash_run(2);
}

static void phase_valve_jog_explore(void)
{
    STEP("Valve Jog Resolution Explorer");
    INFO("Sweeps increasing pulse durations at multiple duty levels.");
    INFO("Measures minimum detectable valve movement and pressure change.");
    INFO("Connect water supply. Press Enter to start, q to abort.");
    INFO("Output is CSV -- capture with serial monitor for graphing.");

    while (true) {
        int c = uart_getchar(30000);
        if (c == 'q' || c == 'Q') { INFO("Aborted."); return; }
        if (c == '\r' || c == '\n') break;
    }

    adc_setup();
    sensor_rail_on();
    motor_rail_on();

    // Start from a known mid-range position with pressure active
    INFO("Moving to start position (%.1f deg)...", VALVE_CAL_START_DEG + 10.0f);
    valve_goto(VALVE_CAL_START_DEG + 10.0f, 1.0f, 15000, false);
    vTaskDelay(pdMS_TO_TICKS(1500));

    // CSV header
    tprintf("duty_pct,pulse_ms,dir,pos_before_deg,pos_after_deg,"
            "delta_deg,psi_before,psi_after,psi_delta\r\n");

    // Sweep: duty levels x pulse durations, both directions
    const uint16_t duties[]    = {160, 200, 240, 280, 320, 400, 480};  // of 500
    const uint32_t pulses_ms[] = {5, 10, 15, 20, 30, 40, 60, 80, 100, 150};
    const int n_duties  = sizeof(duties)   / sizeof(duties[0]);
    const int n_pulses  = sizeof(pulses_ms)/ sizeof(pulses_ms[0]);

    for (int di = 0; di < n_duties; di++) {
        uint16_t duty = duties[di];
        float duty_pct = duty * 100.0f / 500.0f;

        for (int pi = 0; pi < n_pulses; pi++) {
            uint32_t pulse_ms = pulses_ms[pi];

            // Allow user abort between measurements
            if (uart_getchar(0) != 0) { INFO("Aborted."); goto done; }
            TOUCH_ACTIVITY();

            // Re-home to mid-range before each measurement group
            valve_goto(VALVE_CAL_START_DEG + 10.0f, 1.0f, 15000, false);
            vTaskDelay(pdMS_TO_TICKS(800));

            // --- OPENING direction pulse ---
            {
                uint16_t raw = 0;
                float psi_before = -1.0f;
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float pos_before = raw * (360.0f / 4096.0f);
                mprls_read(&psi_before);

                // Fire pulse: set duty, wait pulse_ms, stop
                gpio_set_level(GPIO_VFWD, 1);
                gpio_set_level(GPIO_VREV, 0);
                gpio_set_level(GPIO_K,    0);

                // Use MCPWM via valve motor setup
                mcpwm_timer_handle_t  timer = NULL;
                mcpwm_oper_handle_t   oper  = NULL;
                mcpwm_cmpr_handle_t   cmpr  = NULL;
                mcpwm_gen_handle_t    gen   = NULL;

                mcpwm_timer_config_t tc = {
                    .group_id      = 1,
                    .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
                    .resolution_hz = 10000000,
                    .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
                    .period_ticks  = 500,
                };
                mcpwm_new_timer(&tc, &timer);
                mcpwm_operator_config_t oc = {.group_id = 1};
                mcpwm_new_operator(&oc, &oper);
                mcpwm_operator_connect_timer(oper, timer);
                mcpwm_comparator_config_t cc = {.flags.update_cmp_on_tez = true};
                mcpwm_new_comparator(oper, &cc, &cmpr);
                mcpwm_generator_config_t gc = {.gen_gpio_num = GPIO_VFWD};
                mcpwm_new_generator(oper, &gc, &gen);
                mcpwm_generator_set_action_on_timer_event(gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
                mcpwm_generator_set_action_on_compare_event(gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                    cmpr, MCPWM_GEN_ACTION_LOW));
                mcpwm_comparator_set_compare_value(cmpr, duty);
                mcpwm_timer_enable(timer);
                mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

                vTaskDelay(pdMS_TO_TICKS(pulse_ms));

                mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
                mcpwm_timer_disable(timer);
                mcpwm_del_generator(gen);
                mcpwm_del_comparator(cmpr);
                mcpwm_del_operator(oper);
                mcpwm_del_timer(timer);
                gpio_set_level(GPIO_VFWD, 0);
                gpio_set_level(GPIO_K, 1);
                vTaskDelay(pdMS_TO_TICKS(20));
                gpio_set_level(GPIO_K, 0);

                vTaskDelay(pdMS_TO_TICKS(500));  // settle

                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float pos_after = raw * (360.0f / 4096.0f);
                float psi_after = -1.0f;
                mprls_read(&psi_after);

                tprintf("%.1f,%u,open,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
                        duty_pct, (unsigned)pulse_ms,
                        pos_before, pos_after, pos_after - pos_before,
                        psi_before, psi_after,
                        (psi_after >= 0 && psi_before >= 0) ?
                            psi_after - psi_before : -99.0f);
            }

            vTaskDelay(pdMS_TO_TICKS(200));

            // --- CLOSING direction pulse (from same re-homed position) ---
            valve_goto(VALVE_CAL_START_DEG + 10.0f, 1.0f, 15000, false);
            vTaskDelay(pdMS_TO_TICKS(800));
            {
                uint16_t raw = 0;
                float psi_before = -1.0f;
                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float pos_before = raw * (360.0f / 4096.0f);
                mprls_read(&psi_before);

                gpio_set_level(GPIO_VREV, 1);
                gpio_set_level(GPIO_VFWD, 0);
                gpio_set_level(GPIO_K,    0);
                vTaskDelay(pdMS_TO_TICKS(pulse_ms));
                gpio_set_level(GPIO_VREV, 0);
                gpio_set_level(GPIO_K, 1);
                vTaskDelay(pdMS_TO_TICKS(20));
                gpio_set_level(GPIO_K, 0);

                vTaskDelay(pdMS_TO_TICKS(500));

                as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
                float pos_after = raw * (360.0f / 4096.0f);
                float psi_after = -1.0f;
                mprls_read(&psi_after);

                tprintf("%.1f,%u,close,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
                        duty_pct, (unsigned)pulse_ms,
                        pos_before, pos_after, pos_after - pos_before,
                        psi_before, psi_after,
                        (psi_after >= 0 && psi_before >= 0) ?
                            psi_after - psi_before : -99.0f);
            }
        }
        INFO("Duty %.0f%% complete.", duty_pct);
    }

done:
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 15000, false);
    motor_rail_off();
    INFO("Jog exploration complete. Copy CSV from terminal for analysis.");
    INFO("Minimum reliable delta_deg and corresponding pulse_ms/duty_pct");
    INFO("are the parameters to tune ZONE_WEB_PRES_STEP and kickstart timing.");
}

// -----------------------------------------------------------------------
// Zone Setup Web Server
// Served on port 80. Mobile-friendly radar UI for perimeter definition.
// Access: http://<ip>/  or  http://<ip>/zone
//
// GET  /           -> redirect to /zone
// GET  /zone       -> HTML page (embedded from zone_setup_html.h)
// GET  /zone/state -> JSON state (bearing, throw, pressure_pct, at_min, at_max, points[])
// POST /zone/act?cmd=X -> action, returns updated JSON state
//
// Commands:
//   nozzle_cw / nozzle_ccw  rotate nozzle 2.5 deg per press
//   pres_up   / pres_dn     adjust pressure 2% per press (open-loop via cal table)
//   water_toggle             open/close valve at current pressure
//   trim_pres                one-shot closed-loop pressure correction
//   add_pt                   stamp current bearing+throw as perimeter point
//   undo                     remove last point
//   clear                    remove all points
//   save                     sort by bearing, write to NVS, close valve
//   cancel                   discard edits, reload saved zone, close valve
// -----------------------------------------------------------------------

#ifndef WATER_MAX_THROW_MM
#define WATER_MAX_THROW_MM   8534.0f
#endif

static zone_perimeter_t   s_web_zone          = {0};
static float              s_web_pres_pct      = 50.0f;
static bool               s_web_water         = false;
static float              s_web_valve_deg     = -1.0f;
static float              s_web_meas_psi      = 0.0f;   // last measured PSI (0 when water off)
static float              s_web_meas_throw_mm = 0.0f;   // last measured throw (0 when water off)
static httpd_handle_t     s_zone_server     = NULL;
static SemaphoreHandle_t  s_zone_mutex      = NULL;
// b406: sized to hold the full state JSON for a max-size zone. The points
// array alone is up to ZONE_MAX_PERIM_POINTS(36) * ~56 bytes ≈ 2 KB; the
// old 1800 truncated at 32 points (Patio), yielding invalid JSON that made
// the zone-setup page show "offline" + 0/36. 36 is a hard cap, so this is a
// provable upper bound (unlike /api/all, which scales with zone count).
static char               s_zone_json_buf[3072];

#define ZONE_WEB_PORT            80
#define ZONE_WEB_CTRL_PORT    32770
#define ZONE_WEB_STEP_DEG       2.5f
#define ZONE_WEB_PRES_STEP      2.0f
#define ZONE_WEB_PRES_MIN       5.4f
#define ZONE_WEB_PRES_MAX     100.0f
#define ZONE_WEB_PSI_SCALE      6.28f
#define ZONE_WEB_VALVE_STEP_DEG 0.5f  // open-loop valve step per button press (deg)
#define ZONE_WEB_SETTLE_MS      300   // settle time after valve move before reading PSI

static const char s_zone_html[] =
#include "zone_setup_html.h"
;

static const char s_landing_html[] =
#include "landing_html.h"
;

static const char s_fs_html[] =
#include "fs_html.h"
;
#define LANDING_HTML s_landing_html

static const char s_cal_html[] =
#include "cal_html.h"
;

static const char s_schedule_html[] =
#include "schedule_html.h"
;

// Open-loop valve move using calibration table
#define ZONE_VALVE_OPEN_LOOP() do { \
    if (s_web_water) { \
        float _deg = cal_pressure_to_valve_deg( \
            s_web_pres_pct / 100.0f * ZONE_WEB_PSI_SCALE); \
        if (_deg < VALVE_CAL_START_DEG) \
            _deg = VALVE_CLOSED_DEG + \
                   (s_web_pres_pct / 100.0f) * (VALVE_OPEN_DEG - VALVE_CLOSED_DEG); \
        _deg = fmaxf(VALVE_CLOSED_DEG, fminf(VALVE_OPEN_DEG, _deg)); \
        s_web_valve_deg = _deg; \
        valve_goto_ex(_deg, 2.0f, 8000, false, s_valve_last_dir); \
        s_valve_last_dir = (s_web_pres_pct > 50.0f) ? 1 : -1; \
    } \
} while(0)

static int zone_build_json(char *buf, int maxlen)
{
    uint16_t raw = 0;
    as5600_read(ADDR_AS5600, &raw, NULL, NULL);
    float nozzle_deg = raw * (360.0f / 4096.0f);

    // Live pressure read first -- used for both the throw readout bar and the dot.
    // Reading it here means the display self-corrects on every poll: pressure that
    // was still building when water first opened will catch up within a few polls
    // rather than staying frozen at the one-shot value from the toggle command.
    float live_psi = 0.0f;
    float actual_throw_mm = 0.0f;
    if (mprls_read_quiet(&live_psi) && live_psi > 0.2f) {
        actual_throw_mm = cal_pressure_to_throw_mm(live_psi);
        // Extrapolate beyond cal table if supply pressure exceeds calibrated maximum.
        // Uses slope of last two cal segments so higher supply pressure yields a
        // proportionally larger throw estimate rather than clamping at the cal max.
        { pressure_map_t _ec = {0};
          if (cal_load_primary(&_ec) == ESP_OK && _ec.num_points >= 2) {
              int _n = _ec.num_points;
              float _pmax = _ec.pressure_psi[_n-1], _plo = _ec.pressure_psi[_n-2];
              float _tmax = _ec.throw_mm[_n-1],     _tlo = _ec.throw_mm[_n-2];
              if (live_psi > _pmax && (_pmax - _plo) > 0.01f) {
                  float _slope = (_tmax - _tlo) / (_pmax - _plo);
                  actual_throw_mm = _tmax + _slope * (live_psi - _pmax);
              }
          }
        }
    }

    // Throw display: when water is ON, show live/extrapolated throw (self-corrects
    // every poll as pressure builds, and adjusts above cal table if supply is higher).
    // When water is OFF:
    //   s_web_valve_deg < 0  → fresh page load / valve never positioned this session:
    //                          show cal max as a preview (water_toggle opens to max).
    //   s_web_valve_deg < VALVE_CAL_START_DEG  → valve returned to closed after water off:
    //                          show 0 so the user sees the valve has shut.
    //   s_web_valve_deg in cal range  → user manually adjusted with valve buttons:
    //                          show interpolated throw for that position.
    float throw_mm;
    if (s_web_water && actual_throw_mm > 10.0f) {
        throw_mm = actual_throw_mm;             // live + extrapolated -- updates every poll
    } else if (s_web_water && s_web_meas_throw_mm > 10.0f) {
        throw_mm = s_web_meas_throw_mm;         // fallback: cached if PSI read failed
    } else if (s_web_valve_deg < 0.0f) {
        throw_mm = cal_get_max_throw_mm();      // initial preview: water_toggle opens to max
    } else if (s_web_valve_deg < VALVE_CAL_START_DEG) {
        throw_mm = 0.0f;                        // valve at closed position -- water is off
    } else {
        throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg);  // manual valve position
    }
    float throw_ft = throw_mm / 304.8f;

    // Limit indicators based on valve degree position
    bool at_min = (s_web_valve_deg > 0.0f &&
                   s_web_valve_deg <= VALVE_CAL_START_DEG + ZONE_WEB_VALVE_STEP_DEG);
    bool at_max = (s_web_valve_deg >= VALVE_OPEN_DEG - ZONE_WEB_VALVE_STEP_DEG);

    // b406: up to ZONE_MAX_PERIM_POINTS(36) points * ~56 bytes + brackets.
    // The old 1536 truncated Patio's 32 points mid-object (invalid JSON).
    static char pts[2304];
    pts[0] = '['; pts[1] = '\0';
    for (int i = 0; i < s_web_zone.num_points; i++) {
        char pt[96];
        snprintf(pt, sizeof(pt),
            "%s{\"deg\":%.1f,\"throw_mm\":%.0f,\"psi\":%.3f,\"widx\":%u}",
            i > 0 ? "," : "",
            s_web_zone.points[i].nozzle_deg,
            s_web_zone.points[i].throw_mm,
            s_web_zone.points[i].pressure_psi,
            (unsigned)s_web_zone.points[i].walk_idx);
        strncat(pts, pt, sizeof(pts) - strlen(pts) - 1);
    }
    strncat(pts, "]", sizeof(pts) - strlen(pts) - 1);

    // Compute act_min_throw using edge-perpendicular method so web UI
    // path visualization matches actual firmware ring generation.
    zone_perimeter_t _z = {0};
    bool _hz = (zone_load_primary(s_web_zone_id, &_z) == ESP_OK && _z.num_points >= 2);
    if (_hz) zone_sort_walk_order(&_z);
    float act_min_throw_mm = _hz ? zone_get_min_throw_mm(&_z) : cal_get_min_throw_mm();

    // act_max_throw: when water is on and supply pressure exceeds the cal table, use the
    // extrapolated actual throw so JS ring calculations are correct for higher pressure.
    float act_max_throw_mm = cal_get_max_throw_mm();
    if (s_web_water && actual_throw_mm > act_max_throw_mm)
        act_max_throw_mm = actual_throw_mm;

    return snprintf(buf, maxlen,
        "{\"bearing\":%.1f,\"throw_mm\":%.0f,\"throw_ft\":%.2f,"
        "\"pressure_pct\":%.1f,\"water\":%s,"
        "\"at_min\":%s,\"at_max\":%s,\"points\":%s,"
        "\"act_max_throw\":%.0f,\"fw_build\":%d,"
        "\"actual_throw_mm\":%.0f,\"act_min_throw\":%.0f,"
        "\"name\":\"%s\"}",
        nozzle_deg, throw_mm, throw_ft, s_web_pres_pct,
        s_web_water ? "true" : "false",
        at_min ? "true" : "false",
        at_max ? "true" : "false",
        pts, act_max_throw_mm, FW_BUILD,
        actual_throw_mm, act_min_throw_mm,
        s_web_zone_name);
}

static esp_err_t zone_root_handler(httpd_req_t *req)
{
    WEB_TOUCH();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    HTTP_CONN_CLOSE(req);
    httpd_resp_sendstr(req, LANDING_HTML);
    return ESP_OK;
}

static esp_err_t zone_page_handler(httpd_req_t *req)
{
    WEB_TOUCH();
    // Parse ?id=N or ?id=new
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char ids[8]={0}; httpd_query_key_value(qs, "id", ids, sizeof(ids));
    if (strcmp(ids, "new") == 0) {
        // Find next available zone id
        uint16_t avail[STORAGE_MAX_ZONES]; int nc=0;
        storage_zone_list(avail, STORAGE_MAX_ZONES, &nc);
        s_web_zone_id = (uint16_t)nc;  // next id = count of existing zones
        s_web_zone_is_new = true;
        memset(&s_web_zone, 0, sizeof(s_web_zone));
        snprintf(s_web_zone_name, sizeof(s_web_zone_name), "Zone #%u", s_web_zone_id);
    } else {
        s_web_zone_id = ids[0] ? (uint16_t)atoi(ids) : 0;
        s_web_zone_is_new = false;
        char lfs_name[32] = {0};
        if (storage_ready() &&
            storage_zone_load(s_web_zone_id, lfs_name, sizeof(lfs_name), &s_web_zone) == ESP_OK) {
            zone_name_resolve(s_web_zone_id, lfs_name, "Zone", s_web_zone_name, sizeof(s_web_zone_name));
        } else {
            zone_load_nvs(&s_web_zone);
            zone_name_resolve(s_web_zone_id, NULL, "Zone", s_web_zone_name, sizeof(s_web_zone_name));
        }
    }
    // Reset valve/water web state on each page load so the throw preview is
    // correct when revisiting a zone.  s_web_valve_deg = -1 is the sentinel
    // meaning "valve position not yet set by this session".
    s_web_valve_deg     = -1.0f;
    s_web_water         = false;
    s_web_meas_psi      = 0.0f;
    s_web_meas_throw_mm = 0.0f;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, s_zone_html, strlen(s_zone_html));
    return ESP_OK;
}

static esp_err_t zone_state_handler(httpd_req_t *req)
{
    WEB_TOUCH();
    if (s_zone_mutex) xSemaphoreTake(s_zone_mutex, pdMS_TO_TICKS(500));
    zone_build_json(s_zone_json_buf, sizeof(s_zone_json_buf));
    if (s_zone_mutex) xSemaphoreGive(s_zone_mutex);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, s_zone_json_buf, strlen(s_zone_json_buf));
    return ESP_OK;
}

static esp_err_t zone_act_handler(httpd_req_t *req)
{
    WEB_TOUCH();

    char query[128] = "", cmd[32] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        httpd_query_key_value(query, "cmd", cmd, sizeof(cmd));

    // Optional &deg= parameter for pres_up/pres_dn (hold-acceleration from UI)
    float step_deg = ZONE_WEB_VALVE_STEP_DEG;
    { char deg_str[12] = "";
      if (httpd_query_key_value(query, "deg", deg_str, sizeof(deg_str)) == ESP_OK) {
          float d = strtof(deg_str, NULL);
          if (d >= 0.25f && d <= 2.0f) step_deg = d;
      } }

    if (s_ota_in_progress) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"ota_in_progress\"}");
        return ESP_OK;
    }

    if (s_zone_mutex) xSemaphoreTake(s_zone_mutex, pdMS_TO_TICKS(2000));

    sensor_rail_on();
    motor_rail_on();
    adc_setup();

    if (strcmp(cmd, "nozzle_cw") == 0) {
        uint16_t r = 0;
        as5600_read(ADDR_AS5600, &r, NULL, NULL);
        float deg = r * (360.0f / 4096.0f) + ZONE_WEB_STEP_DEG;
        if (deg >= 360.0f) deg -= 360.0f;
        nozzle_goto_ex(deg, 1.5f, 3000, false, s_nozzle_last_dir);
        s_nozzle_last_dir = 1;

    } else if (strcmp(cmd, "nozzle_ccw") == 0) {
        uint16_t r = 0;
        as5600_read(ADDR_AS5600, &r, NULL, NULL);
        float deg = r * (360.0f / 4096.0f) - ZONE_WEB_STEP_DEG;
        if (deg < 0.0f) deg += 360.0f;
        nozzle_goto_ex(deg, 1.5f, 3000, false, s_nozzle_last_dir);
        s_nozzle_last_dir = -1;

    } else if (strcmp(cmd, "pres_up") == 0) {
        // Open-loop: step valve angle up, then read PSI to derive throw
        if (s_web_valve_deg < VALVE_CLOSED_DEG)
            s_web_valve_deg = VALVE_CAL_START_DEG;   // first press: start at pressure threshold
        s_web_valve_deg += step_deg;
        if (s_web_valve_deg > VALVE_OPEN_DEG) s_web_valve_deg = VALVE_OPEN_DEG;
        if (s_web_water) {
            valve_goto_ex(s_web_valve_deg, 2.0f, 8000, false, 1);
            s_valve_last_dir = 1;
            vTaskDelay(pdMS_TO_TICKS(ZONE_WEB_SETTLE_MS));
            float psi = 0.0f;
            if (mprls_read_quiet(&psi) && psi > 0.1f) {
                s_web_meas_psi      = psi;
                s_web_meas_throw_mm = cal_pressure_to_throw_mm(psi);
            }
        } else {
            s_web_meas_psi      = 0.0f;
            s_web_meas_throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg);
        }
        { float mx = cal_get_max_throw_mm();
          s_web_pres_pct = (mx > 0) ? fmaxf(ZONE_WEB_PRES_MIN,
              fminf(ZONE_WEB_PRES_MAX, s_web_meas_throw_mm / mx * 100.0f)) : s_web_pres_pct; }

    } else if (strcmp(cmd, "pres_dn") == 0) {
        // Open-loop: step valve angle down, then read PSI to derive throw
        if (s_web_valve_deg < VALVE_CLOSED_DEG)
            s_web_valve_deg = VALVE_CAL_START_DEG;
        s_web_valve_deg -= step_deg;
        if (s_web_valve_deg < VALVE_CLOSED_DEG) s_web_valve_deg = VALVE_CLOSED_DEG;
        if (s_web_water) {
            valve_goto_ex(s_web_valve_deg, 2.0f, 8000, false, -1);
            s_valve_last_dir = -1;
            vTaskDelay(pdMS_TO_TICKS(ZONE_WEB_SETTLE_MS));
            float psi = 0.0f;
            if (mprls_read_quiet(&psi) && psi > 0.1f) {
                s_web_meas_psi      = psi;
                s_web_meas_throw_mm = cal_pressure_to_throw_mm(psi);
            } else {
                s_web_meas_psi      = 0.0f;
                s_web_meas_throw_mm = 0.0f;
            }
        } else {
            s_web_meas_psi      = 0.0f;
            s_web_meas_throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg);
        }
        { float mx = cal_get_max_throw_mm();
          s_web_pres_pct = (mx > 0) ? fmaxf(ZONE_WEB_PRES_MIN,
              fminf(ZONE_WEB_PRES_MAX, s_web_meas_throw_mm / mx * 100.0f)) : s_web_pres_pct; }

    } else if (strcmp(cmd, "pres_up_move") == 0) {
        // Hold-repeat fast variant: move valve only, no PSI settle or read.
        // Used by JS for steps 2+ during hold so response is ~motor-time only.
        if (s_web_valve_deg < VALVE_CLOSED_DEG) s_web_valve_deg = VALVE_CAL_START_DEG;
        s_web_valve_deg += step_deg;
        if (s_web_valve_deg > VALVE_OPEN_DEG) s_web_valve_deg = VALVE_OPEN_DEG;
        if (s_web_water) { valve_goto_ex(s_web_valve_deg, 2.0f, 8000, false,  1); s_valve_last_dir =  1; }
        else { s_web_meas_throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg); }

    } else if (strcmp(cmd, "pres_dn_move") == 0) {
        // Hold-repeat fast variant: move valve only, no PSI settle or read.
        if (s_web_valve_deg < VALVE_CLOSED_DEG) s_web_valve_deg = VALVE_CAL_START_DEG;
        s_web_valve_deg -= step_deg;
        if (s_web_valve_deg < VALVE_CLOSED_DEG) s_web_valve_deg = VALVE_CLOSED_DEG;
        if (s_web_water) { valve_goto_ex(s_web_valve_deg, 2.0f, 8000, false, -1); s_valve_last_dir = -1; }
        else { s_web_meas_throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg); }

    } else if (strcmp(cmd, "water_toggle") == 0) {
        s_web_water = !s_web_water;
        if (s_web_water) {
            // Open to full flow; user trims down with valve buttons.
            // Use tight tolerance (1.0 deg) so predictive braking doesn't
            // stop the motor 1-2 deg short of VALVE_OPEN_DEG.
            s_web_pres_pct  = ZONE_WEB_PRES_MAX;
            s_web_valve_deg = VALVE_OPEN_DEG;
            valve_goto(VALVE_OPEN_DEG, 1.0f, 10000, false);
            s_valve_last_dir = 1;
            // Read initial PSI so throw display is live from the start
            vTaskDelay(pdMS_TO_TICKS(500));
            float psi = 0.0f;
            if (mprls_read_quiet(&psi) && psi > 0.1f) {
                s_web_meas_psi      = psi;
                s_web_meas_throw_mm = cal_pressure_to_throw_mm(psi);
            }
        } else {
            valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
            s_web_valve_deg     = VALVE_CLOSED_DEG;
            s_web_meas_psi      = 0.0f;
            s_web_meas_throw_mm = 0.0f;
        }

    } else if (strcmp(cmd, "trim_pres") == 0) {
        if (s_web_water) {
            pressure_map_t cal = {0};
            float psi_min = 0.20f, psi_max = ZONE_WEB_PSI_SCALE;
            if (cal_load_primary(&cal) == ESP_OK && cal.num_points >= 2) {
                psi_min = cal.pressure_psi[0]; psi_max = cal.pressure_psi[0];
                for (int i = 1; i < cal.num_points; i++) {
                    if (cal.pressure_psi[i] < psi_min) psi_min = cal.pressure_psi[i];
                    if (cal.pressure_psi[i] > psi_max) psi_max = cal.pressure_psi[i];
                }
            }
            float target_psi  = s_web_pres_pct / 100.0f * psi_max;
            float actual_psi  = water_hold_pressure(target_psi, psi_min, psi_max);
            float achieved    = (psi_max > psi_min) ?
                (actual_psi - psi_min) / (psi_max - psi_min) * 100.0f : s_web_pres_pct;
            if (achieved < ZONE_WEB_PRES_MIN) achieved = ZONE_WEB_PRES_MIN;
            if (achieved > ZONE_WEB_PRES_MAX) achieved = ZONE_WEB_PRES_MAX;
            s_web_pres_pct  = achieved;
            s_web_valve_deg = cal_pressure_to_valve_deg(actual_psi);
        }

    } else if (strcmp(cmd, "add_pt") == 0) {
        if (s_web_zone.num_points < ZONE_MAX_PERIM_POINTS) {
            uint16_t r = 0;
            as5600_read(ADDR_AS5600, &r, NULL, NULL);
            float deg = r * (360.0f / 4096.0f);

            // Compute throw using exactly the same path as zone_build_json so the
            // stamped point always lands at the green dot the user sees on screen.
            float throw_mm, psi = 0.0f;
            if (s_web_water) {
                // Water ON: live PSI read + extrapolation beyond cal table
                if (mprls_read_quiet(&psi) && psi > 0.2f) {
                    throw_mm = cal_pressure_to_throw_mm(psi);
                    pressure_map_t _ec = {0};
                    if (cal_load_primary(&_ec) == ESP_OK && _ec.num_points >= 2) {
                        int _n = _ec.num_points;
                        float _pmax = _ec.pressure_psi[_n-1], _plo = _ec.pressure_psi[_n-2];
                        float _tmax = _ec.throw_mm[_n-1],     _tlo = _ec.throw_mm[_n-2];
                        if (psi > _pmax && (_pmax - _plo) > 0.01f) {
                            float _slope = (_tmax - _tlo) / (_pmax - _plo);
                            throw_mm = _tmax + _slope * (psi - _pmax);
                        }
                    }
                } else {
                    // PSI read failed: fall back to cached measurement
                    psi      = s_web_meas_psi;
                    throw_mm = (s_web_meas_throw_mm > 10.0f) ?
                                s_web_meas_throw_mm : cal_get_max_throw_mm();
                }
            } else {
                // Water OFF: derive throw from commanded valve position, same as
                // zone_build_json (s_web_valve_deg < 0 = initial/preview state).
                if (s_web_valve_deg < 0.0f || s_web_valve_deg < VALVE_CAL_START_DEG)
                    throw_mm = cal_get_max_throw_mm();
                else
                    throw_mm = cal_valve_deg_to_throw_mm(s_web_valve_deg);
            }

            int n = s_web_zone.num_points++;
            s_web_zone.points[n].nozzle_deg   = deg;
            s_web_zone.points[n].throw_mm     = throw_mm;
            s_web_zone.points[n].pressure_psi = psi;
            s_web_zone.points[n].walk_idx     = (uint8_t)n;
            s_web_zone.points[n].valve_deg    = (s_web_valve_deg > 0.0f) ?
                                                 s_web_valve_deg : VALVE_CAL_START_DEG;
            ESP_LOGI(TAG, "Zone pt %d: deg=%.1f psi=%.3f throw=%.0fmm",
                     n+1, deg, psi, throw_mm);
        }

    } else if (strcmp(cmd, "undo") == 0) {
        if (s_web_zone.num_points > 0) s_web_zone.num_points--;

    } else if (strcmp(cmd, "clear") == 0) {
        memset(&s_web_zone, 0, sizeof(s_web_zone));

    } else if (strcmp(cmd, "save") == 0) {
        // Parse optional name from query (URL-encoded)
        char name_param[32] = {0};
        httpd_query_key_value(query, "name", name_param, sizeof(name_param));
        url_decode(name_param, sizeof(name_param));
        if (name_param[0]) strncpy(s_web_zone_name, name_param, sizeof(s_web_zone_name)-1);
        esp_err_t _save_err = zone_save_primary(s_web_zone_id, s_web_zone_name, &s_web_zone);
        if (_save_err != ESP_OK) {
            ESP_LOGE(TAG, "Zone %u save FAILED: %s", s_web_zone_id, esp_err_to_name(_save_err));
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_status(req, "500 Internal Server Error");
            char err_buf[128];
            snprintf(err_buf, sizeof(err_buf),
                     "{\"ok\":false,\"error\":\"zone save failed: %s\"}",
                     esp_err_to_name(_save_err));
            httpd_resp_sendstr(req, err_buf);
            return ESP_OK;
        }
        s_web_zone_is_new = false;
        ESP_LOGI(TAG, "Zone %u saved: %s (%d pts)", s_web_zone_id, s_web_zone_name, s_web_zone.num_points);
        // Zone changed -- watering history is now stale (ring layout, arc, throw
        // targets all depend on zone geometry).  Delete it so the next run uses
        // the flow-model estimate rather than a cached time from the old layout.
        if (storage_ready()) storage_water_delete(s_web_zone_id);
        if (s_web_water) {
            s_web_water = false;
            valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
        }

    } else if (strcmp(cmd, "cancel") == 0) {
        if (s_web_zone_is_new)
            memset(&s_web_zone, 0, sizeof(s_web_zone));
        else
            zone_load_primary(s_web_zone_id, &s_web_zone);
        if (s_web_water) {
            s_web_water = false;
            valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
        }
    }

    zone_build_json(s_zone_json_buf, sizeof(s_zone_json_buf));
    if (s_zone_mutex) xSemaphoreGive(s_zone_mutex);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, s_zone_json_buf, strlen(s_zone_json_buf));
    return ESP_OK;
}

// GET /api/zone?id=N  -- zone points for mini radar preview
static esp_err_t api_zone_delete_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    char body[32]={0};
    httpd_req_recv(req, body, sizeof(body)-1);
    char ids[8]={0};
    httpd_query_key_value(body, "id", ids, sizeof(ids));
    uint16_t zone_id = (uint16_t)atoi(ids);
    ESP_LOGI(TAG, "Delete zone %u (storage_ready=%d)", zone_id, storage_ready());
    // Delete from LittleFS -- treat NOT_FOUND as success (file may already be gone)
    esp_err_t r = ESP_OK;
    if (storage_ready()) {
        r = storage_zone_delete(zone_id);
        if (r == ESP_FAIL) ESP_LOGW(TAG, "storage_zone_delete failed for zone %u", zone_id);
        if (r != ESP_OK) r = ESP_OK;  // treat any LittleFS error as non-fatal
        storage_water_delete(zone_id);  // remove watering history (CSV + run JSON)
    }
    // Clear NVS name entry
    char key[16]; snprintf(key, sizeof(key), "zname_%u", zone_id);
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, key); nvs_commit(h); nvs_close(h);
    }
    // Clear zone from NVS blob too
    if (zone_id == 0) {
        nvs_handle_t zh;
        if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &zh) == ESP_OK) {
            nvs_erase_key(zh, "zone_perimeter"); nvs_commit(zh); nvs_close(zh);
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_zone_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char ids[8]={0}; httpd_query_key_value(qs, "id", ids, sizeof(ids));
    uint16_t zone_id = (uint16_t)atoi(ids);
    zone_perimeter_t zp={0}; char name[32]="Zone";
    bool ok = false;
    if (storage_ready()) {
        char lfsname[32]={0};
        if (storage_zone_load(zone_id, lfsname, sizeof(lfsname), &zp)==ESP_OK) {
            zone_name_resolve(zone_id, lfsname, "Zone", name, sizeof(name));
            ok=true;
        }
    }
    if (!ok && zone_id==0) {
        zone_load_nvs(&zp);  // pure NVS fallback when LittleFS has no zone
        zone_name_resolve(0, NULL, "Zone", name, sizeof(name));
        ok = zp.num_points>0;
    }
    char buf[1024]; int n=0;
    n+=snprintf(buf+n,sizeof(buf)-n,"{\"id\":%u,\"name\":\"%s\",\"num_points\":%u,\"points\":[",
                zone_id, name, ok?zp.num_points:0);
    for(int i=0;i<zp.num_points&&n<(int)sizeof(buf)-80;i++){
        n+=snprintf(buf+n,sizeof(buf)-n,"%s{\"deg\":%.1f,\"mm\":%.0f,\"widx\":%d}",
            i?",":"",zp.points[i].nozzle_deg,zp.points[i].throw_mm,zp.points[i].walk_idx);
    }
    n+=snprintf(buf+n,sizeof(buf)-n,"]}");
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
    HTTP_CONN_CLOSE(req);
    httpd_resp_send(req,buf,n); return ESP_OK;
}

// POST /api/zone/name  body: id=N&name=MyZone
static esp_err_t api_zone_name_handler(httpd_req_t *req)
{
    char body[128]={0};
    int len=httpd_req_recv(req,body,sizeof(body)-1);
    if(len<=0){httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"bad body");return ESP_OK;}
    char ids[8]={0}, name[32]={0};
    httpd_query_key_value(body,"id",ids,sizeof(ids));
    httpd_query_key_value(body,"name",name,sizeof(name));
    url_decode(name, sizeof(name));
    uint16_t zone_id=(uint16_t)atoi(ids);
    // Primary: update LittleFS header name in-place (no zone data reload needed)
    if (storage_ready() && storage_zone_rename(zone_id, name) == ESP_OK) {
        // LittleFS updated -- NVS not needed but write anyway for migration
        zone_name_save_nvs(zone_id, name);
    } else {
        // LittleFS unavailable or zone not yet saved -- NVS only
        zone_name_save_nvs(zone_id, name);
    }
    httpd_resp_sendstr(req,"{\"ok\":true}"); return ESP_OK;
}

// POST /api/device/name  body: name=MyOtO
static esp_err_t api_device_name_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    char body[128]={0};
    int len=httpd_req_recv(req,body,sizeof(body)-1);
    if(len<=0){httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"bad body");return ESP_OK;}
    char name[32]={0};
    httpd_query_key_value(body,"name",name,sizeof(name));
    url_decode(name, sizeof(name));
    if(!name[0]){httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"name required");return ESP_OK;}
    strncpy(s_device_name, name, sizeof(s_device_name)-1);
    device_name_save_nvs(s_device_name);
    if (s_wifi_netif) {
        char _hn[32]={0}; name_to_hostname(s_device_name, _hn, sizeof(_hn));
        esp_netif_set_hostname(s_wifi_netif, _hn);
    }
    ESP_LOGI(TAG, "Device name set to: %s", s_device_name);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req,"{\"ok\":true}"); return ESP_OK;
}

// POST /zone/water  body: id=N&mode=1|2|3|4|5|6|7|8|c|d  (8=serpentine, b431)
// Triggers a watering run in a one-shot task (non-blocking to HTTP handler)
static void water_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(200));  // brief yield so HTTP response can send
    phase_water_zone_mode(s_web_water_mode);
    s_web_water_mode     = 0;
    s_water_est_min      = 0;
    s_water_cleanup_pass = 0;
    s_eta_anchor_tick    = 0;   // b362
    vTaskDelete(NULL);
}
static esp_err_t zone_water_cancel_handler(httpd_req_t *req)
{
    water_set_status(WATER_STATUS_CANCELLED);
    httpd_resp_set_type(req, "application/json");
    HTTP_CONN_CLOSE(req);
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t zone_water_handler(httpd_req_t *req)
{
    char body[96]={0};
    httpd_req_recv(req,body,sizeof(body)-1);
    char mode_s[4]={0}, id_s[8]={0}, dur_s[8]={0};
    httpd_query_key_value(body,"mode",mode_s,sizeof(mode_s));
    httpd_query_key_value(body,"id",id_s,sizeof(id_s));
    httpd_query_key_value(body,"duration",dur_s,sizeof(dur_s));
    // b311: 'c'/'C' = chase mode (requires duration=1..10).
    // b423: '8' = serpentine (continuous serpentine).
    int mode = mode_s[0]>'0'&&mode_s[0]<='6' ? mode_s[0]-'0' :
               mode_s[0]=='7'                 ? 7  :
               mode_s[0]=='8'                 ? 8  :
               mode_s[0]=='c'||mode_s[0]=='C' ? WATER_MODE_CHASE :
               mode_s[0]=='d'||mode_s[0]=='D' ? 99 : 0;
    uint16_t zone_id = (uint16_t)atoi(id_s);
    if(mode==0){
        httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"invalid mode");
        return ESP_OK;
    }
    int duration_min = 0;
    if (mode == WATER_MODE_CHASE) {
        duration_min = atoi(dur_s);
        if (duration_min < CHASE_MIN_MINUTES || duration_min > CHASE_MAX_MINUTES) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "chase duration must be 1..10 minutes");
            return ESP_OK;
        }
    }
    if(s_web_water_mode!=0 || s_exp_running){
        httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"already running");
        return ESP_OK;
    }
    s_water_abort = false;
    s_water_status_code   = WATER_STATUS_COMPLETED;
    s_water_run_had_flow  = false;
    s_water_no_flow_streak = 0;
    s_water_est_min = (mode==99)                  ? 2 :
                      (mode==WATER_MODE_CHASE)    ? duration_min : 13;
    s_water_zone_id  = zone_id;
    s_chase_duration_min = (mode == WATER_MODE_CHASE) ? duration_min : 0;
    // Optional depth target in eighths of an inch (1..8 = 1/8".. 1"). Absent or
    // out-of-range -> 0 = legacy digit-encoded depth. Chase ignores depth.
    char depth_q[4]={0};
    httpd_query_key_value(body,"depth",depth_q,sizeof(depth_q));
    int depth8_req = atoi(depth_q);
    s_web_water_depth_eighths = (mode == WATER_MODE_CHASE || depth8_req < 1 || depth8_req > 8)
                                  ? 0 : depth8_req;
    // b423: serpentine tuning params. speed = nozzle dps (3..20; absent/0 ->
    // default 8). dry=1 runs the full serpentine with the valve held
    // closed -- a polygon-containment rehearsal before water flows.
    {
        char spd_q[8]={0}, dry_q[4]={0};
        httpd_query_key_value(body,"speed",spd_q,sizeof(spd_q));
        httpd_query_key_value(body,"dry",dry_q,sizeof(dry_q));
        s_web_serpentine_dps = (mode == 8) ? strtof(spd_q, NULL) : 0.0f;
        s_web_serpentine_dry = (mode == 8 && dry_q[0] == '1');
    }
    s_web_water_mode = mode;
    // b294: moved from APP_CPU (core 1) to PRO_CPU (core 0). The original
    // pin-to-core-1 reasoning was "isolate motion control from WiFi/lwIP
    // on core 0." But after b292/b293 the WiFi/PHY isn't running during
    // watering anyway, AND the b294 crash analysis showed a NEW race
    // mode: ESPHome's preferences/NVS code on core 0 doing flash reads
    // while our watering task on core 1 does flash writes -- the
    // cross-core cache-disable handshake faults at cache_utils.c:176.
    // Moving both flash users to the SAME core eliminates the cross-core
    // sync entirely (cache-disable on a single core is trivial). Motion
    // control jitter risk is low because b287 already bumped this task's
    // priority to 15 (higher than ESPHome's main loop), so it preempts
    // ESPHome whenever it needs CPU.
    //
    // b285: bumped from 8192 -> 16384 to fix stack overflow observed at end
    // of long smooth-mode runs on b284. Smooth's deep call chain (motor
    // control + I2C + b283 trace hooks + post-run refit) was very close to
    // the 8KB limit and finally pushed past it. 16KB gives ~2x headroom.
    xTaskCreatePinnedToCore(water_task,"water_web",16384,NULL,10,NULL,PRO_CPU_NUM);
    httpd_resp_set_type(req,"application/json");
    HTTP_CONN_CLOSE(req);
    httpd_resp_sendstr(req,"{\"ok\":true,\"started\":true}");
    return ESP_OK;
}

// EXP: POST /exp/lash?section=A|B  -- spawn the lash experiment task.
// A = dry reversal dead-time, B = wet pressure hysteresis. Mutually exclusive
// with watering and calibration. Results: GET /zone/last_log, filter "LASHCSV,".
static esp_err_t exp_lash_handler(httpd_req_t *req)
{
    char qs[64] = {0};
    httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char sec[4] = {0};
    httpd_query_key_value(qs, "section", sec, sizeof(sec));
    int section = (sec[0]=='A'||sec[0]=='a'||sec[0]=='1') ? 1 :
                  (sec[0]=='B'||sec[0]=='b'||sec[0]=='2') ? 2 :
                  (sec[0]=='C'||sec[0]=='c'||sec[0]=='3') ? 3 : 0;
    if (section == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "section must be A (dead-time), B (hysteresis), or C (coast)");
        return ESP_OK;
    }
    bool busy = (s_web_water_mode != 0) || s_exp_running ||
                (s_wcal.state != WCAL_IDLE && s_wcal.state != WCAL_DONE);
    if (busy) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "device busy (watering / calibration / experiment)");
        return ESP_OK;
    }
    s_water_abort = false;
    s_exp_section = section;
    s_exp_running = true;
    // PRO_CPU + 16KB stack, same as water_task (b294/b285 rationale applies).
    xTaskCreatePinnedToCore(exp_task, "exp_lash", 16384, NULL, 10, NULL, PRO_CPU_NUM);
    httpd_resp_set_type(req, "application/json");
    HTTP_CONN_CLOSE(req);
    httpd_resp_sendstr(req,
        "{\"ok\":true,\"started\":true,\"results\":\"GET /zone/last_log, filter LASHCSV\"}");
    return ESP_OK;
}

static void wcal_pressure_task(void *arg)
{
    s_wcal.state    = WCAL_PRESSURE_SCANNING;
    s_wcal.progress = 0;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Starting scan...");

    adc_setup();
    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(300));

    pressure_map_t map = {0};
    int n = cal_do_pressure_scan(&map, true);
    if (n < 0) {
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
            "No pressure found. Check water supply is connected.");
        s_wcal.state = WCAL_ERROR;
        motor_rail_off(); sensor_rail_off();
        vTaskDelete(NULL); return;
    }

    s_wcal.pmap = map;
    s_wcal.progress = 100;
    // b385: two-anchor throw cal. Park at a SHORT opening first (approached
    // from closed for anti-backlash repeatability) so the user measures the
    // low throw; the high-throw handler then opens fully for the max throw.
    // Two measured anchors give the throw-vs-pressure fit a real slope AND
    // intercept instead of one proportional ray through the origin.
    float low_psi = 0.0f;
    water_seat_valve_from_closed(WATER_MIN_FLOW_PSI + 0.20f, &low_psi);
    s_wcal.pmap_low_psi = low_psi;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg),
        "Scan done (%d pts). Valve open a little (%.2f PSI) -- measure the SHORT throw.",
        n, low_psi);
    s_wcal.state = WCAL_PRESSURE_AWAIT_THROW_LOW;
    vTaskDelete(NULL);
}

// b418/b419: open-loop motor-polarity probe, shared by the valve and nozzle
// cals. Pulses the PHYSICAL fwd-labelled pin and returns the SIGN of the
// resulting encoder delta: +1 (angle increased), -1 (decreased), 0 (never
// moved -- inconclusive, store nothing). The caller maps the sign onto its
// reference convention (VFWD decreases / NFWD increases). Runs BEFORE any
// closed-loop move so a motor with swapped +/- leads can't servo away from
// target -- the goto loops' wrap guards would fire half a revolution out
// and falsely report "reached". Pulse widths grow past the lash takeup
// (~67 ms on the valve, /exp/lash Section A) in case the first pulse is
// eaten by a reversal. Jogs back open-loop (opposite pin, same total drive
// time) so the cal starts roughly where the motor was.
static int motor_dir_probe(gpio_num_t pulse_pin, gpio_num_t other_pin,
                           uint8_t enc_addr, float *delta_out)
{
    uint16_t raw = 0;
    if (!as5600_read(enc_addr, &raw, NULL, NULL)) return 0;
    float start = raw * (360.0f / 4096.0f);

    float delta = 0.0f;
    uint32_t driven_ms = 0;
    static const uint32_t PULSE_MS[3] = {100, 250, 500};
    // A prior goto may have left a pin bound to an MCPWM generator --
    // reclaim both for plain GPIO (same defense as exp_drv_start).
    gpio_set_direction(pulse_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(other_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(other_pin, 0);
    gpio_set_level(GPIO_K, 0);
    for (int i = 0; i < 3; i++) {
        gpio_set_level(pulse_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(PULSE_MS[i]));
        gpio_set_level(pulse_pin, 0);
        driven_ms += PULSE_MS[i];
        vTaskDelay(pdMS_TO_TICKS(200));  // coast + settle
        if (!as5600_read(enc_addr, &raw, NULL, NULL)) break;
        delta = raw * (360.0f / 4096.0f) - start;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        if (fabsf(delta) >= 0.5f) break;
    }

    int sense = 0;
    if      (delta >=  0.5f) sense = +1;
    else if (delta <= -0.5f) sense = -1;

    if (sense != 0) {
        // Return jog: opposite physical pin, same total drive time. Lash
        // makes this slightly short -- fine, the cal starts from "current".
        gpio_set_level(other_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(driven_ms));
        gpio_set_level(other_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (delta_out) *delta_out = delta;
    ESP_LOGI(TAG, "Motor polarity probe (pin %d, enc 0x%02x): pulse moved %.2f deg -> sense %+d",
             (int)pulse_pin, enc_addr, delta, sense);
    return sense;
}

static void wcal_nozzle_task(void *arg)
{
    s_wcal.state    = WCAL_NOZZLE_RUNNING;
    s_wcal.progress = 0;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Starting nozzle speed cal...");

    sensor_rail_on();
    motor_rail_on();
    adc_setup();
    vTaskDelay(pdMS_TO_TICKS(300));

    // b419: polarity probe FIRST (dry, before the valve opens) -- the speed
    // maps below are direction-labelled (CW vs CCW) and the valve open
    // itself servos closed-loop.
    int nsense = motor_dir_probe(GPIO_NFWD, GPIO_NREV, ADDR_AS5600, NULL);
    if (nsense == 0) {
        motor_rail_off(); sensor_rail_off();
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
                 "Polarity probe: nozzle never moved -- check motor power/wiring.");
        s_wcal.state = WCAL_ERROR;
        vTaskDelete(NULL);
    }
    if (nsense != g_nozzle_motor_dir)
        ESP_LOGW(TAG, "Nozzle motor polarity changed: %+d -> %+d",
                 g_nozzle_motor_dir, nsense);
    g_nozzle_motor_dir = (int8_t)nsense;  // reference wiring: NFWD INCREASES angle (CW)
    motor_dir_nvs_save("ndir", (int8_t)nsense);

    // Open valve to ~30% pressure for wet measurement
    pressure_map_t _pcal = {0};
    bool _have_pcal = (cal_load_primary(&_pcal) == ESP_OK && _pcal.num_points >= 2);
    if (_have_pcal) {
        float _psi_min = _pcal.pressure_psi[0], _psi_max = _pcal.pressure_psi[0];
        for (int _ci = 1; _ci < _pcal.num_points; _ci++) {
            if (_pcal.pressure_psi[_ci] < _psi_min) _psi_min = _pcal.pressure_psi[_ci];
            if (_pcal.pressure_psi[_ci] > _psi_max) _psi_max = _pcal.pressure_psi[_ci];
        }
        float _target_psi = _psi_max * 0.30f;
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Opening valve to %.2f PSI (30%%)...", _target_psi);
        water_hold_pressure(_target_psi, _psi_min, _psi_max);
    } else {
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "No pressure cal -- running dry.");
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    uint16_t duties[] = {70,80,90,100,150,200,250,300,350,400,450,500};
    int nd = 12;
    speed_map_t map = {0};
    int n = 0, nccw = 0;

    for (int i = 0; i < nd && n < SPD_MAX_POINTS; i++) {
        s_wcal.progress = i * 45 / nd;
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "CW duty %d (%d/%d)...", duties[i], i+1, nd);
        vTaskDelay(pdMS_TO_TICKS(SPD_SETTLE_MS));
        float dps = spd_measure_rotation(duties[i]);
        if (dps < 0) continue;
        map.duty[n] = duties[i];
        map.deg_per_sec[n] = dps;
        n++;
    }
    map.num_points = n;

    for (int i = 0; i < nd && nccw < SPD_MAX_POINTS; i++) {
        s_wcal.progress = 50 + i * 45 / nd;
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "CCW duty %d (%d/%d)...", duties[i], i+1, nd);
        vTaskDelay(pdMS_TO_TICKS(500));
        float dps = spd_measure_rotation_ccw(duties[i]);
        if (dps < 0) continue;
        map.duty_ccw[nccw] = duties[i];
        map.deg_per_sec_ccw[nccw] = dps;
        nccw++;
    }
    map.num_points_ccw = nccw;

    spd_save_primary(&map);
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off(); sensor_rail_off();
    s_wcal.progress = 100;
    snprintf(s_wcal.msg, sizeof(s_wcal.msg),
        "Done. CW: %d pts, CCW: %d pts.", n, nccw);
    s_wcal.state = WCAL_DONE;
    vTaskDelete(NULL);
}

static esp_err_t api_all_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    // Status fields
    size_t used=0, total=0;
    if (storage_ready()) storage_usage(&used, &total);

    httpd_resp_set_type(req, "application/json");
    // b306: cross-origin GETs from HA dashboards (e.g. the heatmap
    // Lovelace card) need an explicit Allow-Origin header. The other
    // /api/* and /zone/* endpoints already set this; /api/all was the
    // odd one out, blocking the card's first request and short-
    // circuiting the rest.
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // b405: emit the response as HTTP chunks instead of building it in one
    // fixed buffer. The old char buf[3072] silently truncated once the
    // zones array filled it: at 6 zones on ba1f88, Patio (id=5, 32 pts)
    // pushed past the buffer and the loop's `n<sizeof(buf)-400` guard
    // dropped it entirely, so it never appeared in the web UI. Chunking
    // means zone count/size can no longer truncate the JSON. A small
    // scratch buffer is reused per snprintf; nothing here exceeds it.
    char buf[512]; int n;
    uint32_t bat_mv = (uint32_t)(adc_mv(ADC_CH_VBATT) * VBATT_DIVIDER_RATIO);
    // mDNS / WiFi-station hostname (ESPHome owns this in component mode;
    // standalone sets it in wifi_init). Distinct from the user-editable
    // s_device_name above.
    const char *hostname = "";
    esp_netif_t *_nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (_nif) esp_netif_get_hostname(_nif, &hostname);
    n = snprintf(buf, sizeof(buf),
        "{\"fw_build\":%u,\"wifi_rssi\":%d,\"wifi_ip\":\"%s\","
        "\"hostname\":\"%s\","
        "\"bat_mv\":%lu,"
        "\"storage_used_kb\":%u,\"storage_total_kb\":%u,"
        "\"watering\":%s,\"water_mode\":%d,\"water_est_min\":%d,"
        "\"water_zone_id\":%u,\"cleanup_pass\":%d,"
        "\"detail_log\":%s,"
        "\"device_name\":\"%s\",\"zones\":[",
        FW_BUILD, wifi_get_rssi(), s_wifi_ip,
        hostname ? hostname : "",
        bat_mv,
        (unsigned)(used/1024),(unsigned)(total/1024),
        s_web_water_mode?"true":"false",s_web_water_mode,s_water_est_min,
        (unsigned)s_water_zone_id, s_water_cleanup_pass,
        s_water_detail_log?"true":"false",
        s_device_name);
    httpd_resp_send_chunk(req, buf, n);

    int count=0;
    if (storage_ready()) {
        uint16_t ids[STORAGE_MAX_ZONES]; int nc=0;
        storage_zone_list(ids, STORAGE_MAX_ZONES, &nc);
        for (int i=0; i<nc; i++) {
            zone_perimeter_t zp={0};
            char _zn[32]={0};
            if (storage_zone_load(ids[i],_zn,sizeof(_zn),&zp)!=ESP_OK) continue;
            float mx=0,mn=1e9f,alo=999,ahi=-999;
            for(int j=0;j<zp.num_points;j++){
                if(zp.points[j].throw_mm>mx)mx=zp.points[j].throw_mm;
                if(zp.points[j].throw_mm<mn)mn=zp.points[j].throw_mm;
                if(zp.points[j].nozzle_deg<alo)alo=zp.points[j].nozzle_deg;
                if(zp.points[j].nozzle_deg>ahi)ahi=zp.points[j].nozzle_deg;
            }
            char zrname[32], _zdef[16];
            snprintf(_zdef, sizeof(_zdef), "Zone #%u", ids[i]);
            zone_name_resolve(ids[i], _zn, _zdef, zrname, sizeof(zrname));
            n=snprintf(buf,sizeof(buf),
                "%s{\"id\":%u,\"name\":\"%s\",\"num_points\":%u,"
                "\"min_ft\":%.1f,\"max_ft\":%.1f,\"arc_deg\":%.1f,\"points\":[",
                count?",":"",ids[i],zrname,
                zp.num_points,mn/304.8f,mx/304.8f,ahi-alo);
            httpd_resp_send_chunk(req, buf, n);
            for(int j=0;j<zp.num_points;j++){
                n=snprintf(buf,sizeof(buf),"%s{\"deg\":%.1f,\"mm\":%.0f,\"widx\":%d}",
                    j?",":"",zp.points[j].nozzle_deg,zp.points[j].throw_mm,
                    zp.points[j].walk_idx);
                httpd_resp_send_chunk(req, buf, n);
            }
            httpd_resp_send_chunk(req, "]}", 2);
            count++;
        }
    }
    if (count==0) {
        zone_perimeter_t zp={0};
        if (zone_load_nvs(&zp)==ESP_OK && zp.num_points>=2) {  // NVS fallback
            float mx=0,mn=1e9f,alo=999,ahi=-999;
            for(int j=0;j<zp.num_points;j++){
                if(zp.points[j].throw_mm>mx)mx=zp.points[j].throw_mm;
                if(zp.points[j].throw_mm<mn)mn=zp.points[j].throw_mm;
                if(zp.points[j].nozzle_deg<alo)alo=zp.points[j].nozzle_deg;
                if(zp.points[j].nozzle_deg>ahi)ahi=zp.points[j].nozzle_deg;
            }
            char zname[32]="Zone 1";
            zone_name_load_nvs(0, zname, sizeof(zname));
            n=snprintf(buf,sizeof(buf),
                "{\"id\":0,\"name\":\"%s\",\"num_points\":%u,"
                "\"min_ft\":%.1f,\"max_ft\":%.1f,\"arc_deg\":%.1f}",
                zname,zp.num_points,mn/304.8f,mx/304.8f,ahi-alo);
            httpd_resp_send_chunk(req, buf, n);
        }
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// b347: snapshot the boot_diag ring as JSON. Latest-first ordering so a
// human eyeballing the response sees the most-recent boot at index 0
// regardless of where the write pointer happens to be. Each entry
// reflects what the boot-time battery + valve checks observed; useful
// when a TaskWDT / panic / brownout rebooted the device with the valve
// open and the user wants to confirm whether the boot-time close fired.
static esp_err_t api_boot_diag_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    boot_diag_blob_t blob;
    boot_diag_load_all(&blob);

    // b407: each entry's JSON is ~400 bytes and the ring holds up to
    // BOOT_DIAG_RING_N(5), so a full ring is ~2 KB — the old 1536 truncated
    // it to ~3 entries (invalid JSON). 5 is a hard cap, so this is a provable
    // upper bound (same reasoning as the b406 /zone/state buffer).
    static char buf[2560]; int n = 0;
    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"count\":%u,\"latest_idx\":%u,\"fw_build\":%u,\"entries\":[",
        blob.count, blob.next_idx, (unsigned)FW_BUILD);
    for (int i = 0; i < blob.count; i++) {
        // Walk newest -> oldest. Newest sits at next_idx-1, then go backwards.
        int slot = ((int)blob.next_idx - 1 - i + 2 * BOOT_DIAG_RING_N) % BOOT_DIAG_RING_N;
        const boot_diag_t *e = &blob.ring[slot];
        const char *rstr = boot_diag_reset_reason_str(e->reset_reason);
        n += snprintf(buf+n, sizeof(buf)-n,
            "%s{"
              "\"boot_seq\":%u,"
              "\"boot_epoch\":%lld,"
              "\"fw_build\":%u,"
              "\"reset_reason\":\"%s\","
              "\"reset_reason_code\":%d,"
              "\"flags\":%u,"
              "\"close_attempted\":%s,"
              "\"close_ok\":%s,"
              "\"batt_skipped_valve\":%s,"
              "\"batt_too_low\":%s,"
              "\"sensor_read_failed\":%s,"
              "\"batt_v_at_boot\":%.3f,"
              "\"batt_v_after_close\":%.3f,"
              "\"valve_deg_at_boot\":%.2f,"
              "\"valve_err_at_boot\":%.2f,"
              "\"valve_deg_after_close\":%.2f"
            "}",
            i ? "," : "",
            (unsigned)e->boot_seq,
            (long long)e->boot_epoch,
            (unsigned)e->fw_build,
            rstr,
            (int)e->reset_reason,
            (unsigned)e->flags,
            (e->flags & BOOT_DIAG_FLAG_CLOSE_ATTEMPTED)    ? "true" : "false",
            (e->flags & BOOT_DIAG_FLAG_CLOSE_OK)           ? "true" : "false",
            (e->flags & BOOT_DIAG_FLAG_BATT_SKIPPED_VALVE) ? "true" : "false",
            (e->flags & BOOT_DIAG_FLAG_BATT_TOO_LOW)       ? "true" : "false",
            (e->flags & BOOT_DIAG_FLAG_SENSOR_READ_FAILED) ? "true" : "false",
            e->batt_v_at_boot,
            e->batt_v_after_close,
            e->valve_deg_at_boot,
            e->valve_err_at_boot,
            e->valve_deg_after_close);
    }
    n += snprintf(buf+n, sizeof(buf)-n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// Toggle or set s_water_detail_log.  GET /api/detail_log         → toggle.
// GET /api/detail_log?on=1  → enable.  GET /api/detail_log?on=0 → disable.
static esp_err_t api_detail_log_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char val[4]={0};
    if (httpd_query_key_value(qs, "on", val, sizeof(val)) == ESP_OK)
        s_water_detail_log = (val[0] == '1');
    else
        s_water_detail_log = !s_water_detail_log;
    INFO("Detail log %s", s_water_detail_log ? "ENABLED" : "disabled");
    char resp[40];
    snprintf(resp, sizeof(resp), "{\"detail_log\":%s}", s_water_detail_log?"true":"false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static int wifi_get_rssi(void) {
    if (!s_wifi_connected) return 1;  // 1 = disconnected sentinel (RSSI always negative when connected)
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK && ap.rssi < 0) return ap.rssi;
    return -65;  // connected but RSSI unavailable -- return plausible default
}

static esp_err_t api_cal_clear_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    // Remove LittleFS file
    if (storage_ready()) remove("/lfs/cal/pressure.json");
    // Erase NVS backup
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "pressure_map");
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Pressure cal cleared (LittleFS + NVS)");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_cal_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    pressure_map_t map = {0};
    cal_load_primary(&map);
    static char buf[2048]; int n = 0;
    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"num_points\":%u,\"valve_deg\":[", map.num_points);
    for (int i = 0; i < (int)map.num_points; i++)
        n += snprintf(buf+n, sizeof(buf)-n, "%s%.2f", i?",":"", map.valve_deg[i]);
    n += snprintf(buf+n, sizeof(buf)-n, "],\"pressure_psi\":[");
    for (int i = 0; i < (int)map.num_points; i++)
        n += snprintf(buf+n, sizeof(buf)-n, "%s%.4f", i?",":"", map.pressure_psi[i]);
    n += snprintf(buf+n, sizeof(buf)-n, "],\"throw_mm\":[");
    for (int i = 0; i < (int)map.num_points; i++)
        n += snprintf(buf+n, sizeof(buf)-n, "%s%.1f", i?",":"", map.throw_mm[i]);
    n += snprintf(buf+n, sizeof(buf)-n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// GET /api/theme  -> {"dark":true|false}
// POST /api/theme  body: dark=1 or dark=0  -> {"ok":true,"dark":bool}
static esp_err_t api_theme_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    httpd_resp_set_type(req, "application/json");
    if (req->method == HTTP_POST) {
        char body[32]={0};
        int len = httpd_req_recv(req, body, sizeof(body)-1);
        if (len <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
            return ESP_OK;
        }
        char val[8]={0};
        if (httpd_query_key_value(body, "dark", val, sizeof(val)) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dark required");
            return ESP_OK;
        }
        bool dark = (val[0] == '1' || val[0] == 't' || val[0] == 'T');
        irrigoto_set_theme_dark(dark);
    }
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "{\"dark\":%s}",
                     irrigoto_get_theme_dark() ? "true" : "false");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t api_status_handler(httpd_req_t *req)
{
    size_t used=0, total=0;
    if (storage_ready()) storage_usage(&used, &total);
    char buf[256];
    HTTP_CONN_CLOSE(req);
    int n = snprintf(buf, sizeof(buf),
        "{\"fw_build\":%u,\"wifi_rssi\":%d,"
        "\"storage_used_kb\":%u,\"storage_total_kb\":%u,"
        "\"watering\":%s,\"water_mode\":%d,\"water_est_min\":%d,\"cleanup_pass\":%d}",
        FW_BUILD, wifi_get_rssi(),
        (unsigned)(used/1024), (unsigned)(total/1024),
        s_web_water_mode?"true":"false", s_web_water_mode, s_water_est_min,
        s_water_cleanup_pass);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t api_zones_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    char buf[1024]; int n=0, count=0;
    n += snprintf(buf+n, sizeof(buf)-n, "{\"zones\":[");
    if (storage_ready()) {
        uint16_t ids[STORAGE_MAX_ZONES]; int nc=0;
        storage_zone_list(ids, STORAGE_MAX_ZONES, &nc);
        for (int i=0; i<nc && n<(int)sizeof(buf)-200; i++) {
            zone_perimeter_t zp={0};
            char _zn2[32]={0};
            if (storage_zone_load(ids[i],_zn2,sizeof(_zn2),&zp)!=ESP_OK) continue;
            float mx=0,mn=1e9f,alo=999,ahi=-999;
            for(int j=0;j<zp.num_points;j++){
                if(zp.points[j].throw_mm>mx)mx=zp.points[j].throw_mm;
                if(zp.points[j].throw_mm<mn)mn=zp.points[j].throw_mm;
                if(zp.points[j].nozzle_deg<alo)alo=zp.points[j].nozzle_deg;
                if(zp.points[j].nozzle_deg>ahi)ahi=zp.points[j].nozzle_deg;
            }
            n += snprintf(buf+n, sizeof(buf)-n,
                "%s{\"id\":%u,\"name\":\"%s\",\"num_points\":%u,"
                "\"min_ft\":%.1f,\"max_ft\":%.1f,\"arc_deg\":%.1f}",
                count?",":"", ids[i], _zn2[0]?_zn2:"Zone",
                zp.num_points, mn/304.8f, mx/304.8f, ahi-alo);
            count++;
        }
    }
    if (count==0) {
        zone_perimeter_t zp={0};
        if (zone_load_nvs(&zp)==ESP_OK && zp.num_points>=2) {  // NVS fallback
            float mx=0,mn=1e9f,alo=999,ahi=-999;
            for(int j=0;j<zp.num_points;j++){
                if(zp.points[j].throw_mm>mx)mx=zp.points[j].throw_mm;
                if(zp.points[j].throw_mm<mn)mn=zp.points[j].throw_mm;
                if(zp.points[j].nozzle_deg<alo)alo=zp.points[j].nozzle_deg;
                if(zp.points[j].nozzle_deg>ahi)ahi=zp.points[j].nozzle_deg;
            }
            char zname[32]="Zone 1";
            zone_name_load_nvs(0, zname, sizeof(zname));
            n += snprintf(buf+n, sizeof(buf)-n,
                "{\"id\":0,\"name\":\"%s\",\"num_points\":%u,"
                "\"min_ft\":%.1f,\"max_ft\":%.1f,\"arc_deg\":%.1f}",
                zname, zp.num_points, mn/304.8f, mx/304.8f, ahi-alo);
        }
    }
    n += snprintf(buf+n, sizeof(buf)-n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t cal_status_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    const pressure_map_t *pm = &s_wcal.pmap;
    char buf[2048]; int n = 0;
    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"state\":%d,\"msg\":\"%s\",\"progress\":%d,"
        "\"open_psi\":%.3f,\"num_points\":%u,\"points\":[",
        (int)s_wcal.state, s_wcal.msg, s_wcal.progress,
        s_wcal.pmap_open_psi, pm->num_points);
    for (int i = 0; i < (int)pm->num_points && n < (int)sizeof(buf)-80; i++) {
        n += snprintf(buf+n, sizeof(buf)-n,
            "%s{\"v\":%.1f,\"p\":%.3f,\"t\":%.0f}",
            i?",":"", pm->valve_deg[i], pm->pressure_psi[i], pm->throw_mm[i]);
    }
    n += snprintf(buf+n, sizeof(buf)-n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t cal_pressure_start_handler(httpd_req_t *req)
{
    if (s_wcal.state == WCAL_PRESSURE_SCANNING ||
        s_wcal.state == WCAL_NOZZLE_RUNNING) {
        httpd_resp_sendstr(req, "{\"error\":\"calibration already running\"}");
        return ESP_OK;
    }
    // b380: coast by default (the A/B found no cal benefit from braking -- see
    // s_cal_brake notes). Optional ?brake=1 re-enables the K hold for experiments.
    s_cal_brake = false;
    char q[48];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char val[8] = {0};
        if (httpd_query_key_value(q, "brake", val, sizeof(val)) == ESP_OK)
            s_cal_brake = (val[0] != '0');
    }
    ESP_LOGI(TAG, "cal_pressure_start: brake=%d", s_cal_brake);
    wcal_reset();
    xTaskCreatePinnedToCore(wcal_pressure_task, "wcal_pres", 12288, NULL, 8, NULL, APP_CPU_NUM);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s_cal_brake ? "{\"ok\":true,\"brake\":true}"
                                        : "{\"ok\":true,\"brake\":false}");
    return ESP_OK;
}

// b385: low throw anchor. The valve is parked at a short opening (set by
// wcal_pressure_task via water_seat_valve_from_closed). The user measures the
// short throw and posts it here; we record (psi, throw) as the low anchor,
// then open fully and advance to the high-throw step.
static esp_err_t cal_pressure_throw_low_handler(httpd_req_t *req)
{
    if (s_wcal.state != WCAL_PRESSURE_AWAIT_THROW_LOW) {
        httpd_resp_sendstr(req, "{\"error\":\"not awaiting low throw\"}");
        return ESP_OK;
    }
    char body[64] = {0};
    httpd_req_recv(req, body, sizeof(body)-1);
    char val[16] = {0};
    httpd_query_key_value(body, "throw_mm", val, sizeof(val));
    float throw_mm = atof(val);
    if (val[0] && (val[strlen(val)-1] == 'f' || val[strlen(val)-1] == 'F')) throw_mm *= 304.8f;
    if (throw_mm < 100.0f || throw_mm > 15000.0f) {
        httpd_resp_sendstr(req, "{\"error\":\"throw_mm out of range (100-15000)\"}");
        return ESP_OK;
    }
    // Fresh pressure at the current short opening -- time-correlated with the
    // short throw the user just measured.
    float psi_now = cal_read_pressure_avg();
    s_wcal.pmap_low_psi   = psi_now;
    s_wcal.pmap_low_throw = throw_mm;
    ESP_LOGI(TAG, "cal low throw: %.0f mm at fresh PSI=%.3f", throw_mm, psi_now);
    // Open fully for the high anchor.
    valve_goto(VALVE_OPEN_DEG, 1.0f, 15000, false);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));
    s_wcal.pmap_open_psi = cal_read_pressure_avg();
    snprintf(s_wcal.msg, sizeof(s_wcal.msg),
        "Low throw saved (%.0fmm @ %.2f PSI). Valve full open -- measure the MAX throw.",
        throw_mm, psi_now);
    s_wcal.state = WCAL_PRESSURE_AWAIT_THROW;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t cal_pressure_throw_handler(httpd_req_t *req)
{
    if (s_wcal.state != WCAL_PRESSURE_AWAIT_THROW) {
        httpd_resp_sendstr(req, "{\"error\":\"not awaiting throw\"}");
        return ESP_OK;
    }
    char body[64] = {0};
    httpd_req_recv(req, body, sizeof(body)-1);
    char val[16] = {0};
    httpd_query_key_value(body, "throw_mm", val, sizeof(val));
    float throw_mm = atof(val);
    // Accept feet suffix: e.g. "20.5f"
    if (val[strlen(val)-1] == 'f' || val[strlen(val)-1] == 'F') throw_mm *= 304.8f;
    if (throw_mm < 100.0f || throw_mm > 15000.0f) {
        httpd_resp_sendstr(req, "{\"error\":\"throw_mm out of range (100-15000)\"}");
        return ESP_OK;
    }
    // Take a fresh pressure reading NOW — supply pressure can drift while
    // the user walks out, measures throw, and walks back. The valve is
    // still at VALVE_OPEN_DEG from when the low-throw step completed, so this
    // is the same operating point as the throw they just observed.
    pressure_map_t *pm = &s_wcal.pmap;
    float psi_now = cal_read_pressure_avg();
    s_wcal.pmap_max_throw = throw_mm;
    // b385: two-anchor fit -- low anchor from the AWAIT_THROW_LOW step, high
    // anchor measured now at full open. Fills throw_mm[] for every point.
    cal_apply_two_anchor_throw(pm,
        s_wcal.pmap_low_psi, s_wcal.pmap_low_throw,
        psi_now,             throw_mm);
    ESP_LOGI(TAG, "cal two-anchor: lo(%.3fPSI,%.0fmm) hi(%.3fPSI,%.0fmm)",
        s_wcal.pmap_low_psi, s_wcal.pmap_low_throw, psi_now, throw_mm);
    esp_err_t _save_err = cal_save_primary(pm);
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();
    sensor_rail_off();
    if (_save_err == ESP_OK) {
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
            "Saved. Max throw %.0fmm (%.1fft), %d points.",
            throw_mm, throw_mm/304.8f, pm->num_points);
        s_wcal.state = WCAL_DONE;
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "Cal save FAILED: %s", esp_err_to_name(_save_err));
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
            "FAILED to save: %s", esp_err_to_name(_save_err));
        s_wcal.state = WCAL_IDLE;
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, "500 Internal Server Error");
        char err_buf[128];
        snprintf(err_buf, sizeof(err_buf),
                 "{\"ok\":false,\"error\":\"cal save failed: %s\"}",
                 esp_err_to_name(_save_err));
        httpd_resp_sendstr(req, err_buf);
    }
    return ESP_OK;
}

static esp_err_t cal_pressure_cancel_handler(httpd_req_t *req)
{
    s_wcal.state = WCAL_IDLE;
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off(); sensor_rail_off();
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Cancelled.");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t cal_nozzle_start_handler(httpd_req_t *req)
{
    if (s_wcal.state == WCAL_PRESSURE_SCANNING ||
        s_wcal.state == WCAL_NOZZLE_RUNNING) {
        httpd_resp_sendstr(req, "{\"error\":\"calibration already running\"}");
        return ESP_OK;
    }
    wcal_reset();
    xTaskCreatePinnedToCore(wcal_nozzle_task, "wcal_nozzle", 8192, NULL, 8, NULL, APP_CPU_NUM);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// b370: valve open/close moved off the HA service bus to HTTP so the
// ListEntitiesServices response stays under the 2026.4.5 send-buffer
// ceiling once the Valve Open binary_sensor is added. The HA "Irrigoto
// Valve" template switch drives these; firmware confirms via valve_open.
static esp_err_t valve_open_handler(httpd_req_t *req)
{
    irrigoto_valve_open();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t valve_close_handler(httpd_req_t *req)
{
    irrigoto_valve_close();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// b372: dry-test / diagnostic valve positioning. GET /valve/goto?deg=X drives
// the valve to X (clamped 231-308) then reads the AS5600L back so the caller
// sees commanded vs achieved in one round-trip. Used to map the low-pressure
// "false cutoff" near the valve endpoints (run with water OFF). agc/status are
// the encoder's magnet diagnostics — watch for them drifting near the seats.
static esp_err_t valve_goto_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char deg_str[12]={0};
    if (httpd_query_key_value(qs, "deg", deg_str, sizeof(deg_str)) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"missing deg\"}");
        return ESP_OK;
    }
    float target = strtof(deg_str, NULL);
    if (target < VALVE_CLOSED_DEG) target = VALVE_CLOSED_DEG;
    if (target > VALVE_OPEN_DEG)   target = VALVE_OPEN_DEG;

    // b373: optional tol param tightens the settle tolerance for fine
    // diagnostic sweeps (default 2.0 matches the production valve_goto).
    char tol_str[12]={0};
    float tol = 2.0f;
    if (httpd_query_key_value(qs, "tol", tol_str, sizeof(tol_str)) == ESP_OK) {
        tol = strtof(tol_str, NULL);
        if (tol < 0.2f) tol = 0.2f;
        if (tol > 5.0f) tol = 5.0f;
    }

    sensor_rail_on();
    motor_rail_on();
    bool ok = valve_goto(target, tol, 10000, true);

    sensor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(50));
    uint16_t vraw=0; uint8_t vagc=0, vst=0;
    as5600_read(ADDR_AS5600L, &vraw, &vagc, &vst);

    char buf[180];
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":%s,\"commanded\":%.2f,\"tol\":%.2f,\"achieved\":%.2f,\"err\":%.2f,"
        "\"agc\":%u,\"status\":\"%02x\"}",
        ok ? "true" : "false", target, tol, vraw*(360.0f/4096.0f),
        vraw*(360.0f/4096.0f) - target, vagc, vst);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// b377: controlled brake-vs-coast A/B probe. GET /valve/probe?deg=X&brake=0|1
// takes ONE calibration-style sample: drives to X, optionally K-brakes the hold,
// settles, then reads achieved angle + pressure. Self-contained K control (not
// gated on s_cal_brake) so a host script can call it at a FIXED angle list,
// interleaving a coast read and a braked read back-to-back at each angle -- which
// nulls out the supply-pressure drift that confounds whole-scan comparisons.
// Optional &settle=<ms> (default CAL_SETTLE_MS) and &tol=<deg> (default 0.5).
static esp_err_t valve_probe_handler(httpd_req_t *req)
{
    char qs[64]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char v[12]={0};
    if (httpd_query_key_value(qs, "deg", v, sizeof(v)) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"missing deg\"}");
        return ESP_OK;
    }
    float target = strtof(v, NULL);
    if (target < VALVE_CLOSED_DEG) target = VALVE_CLOSED_DEG;
    if (target > VALVE_OPEN_DEG)   target = VALVE_OPEN_DEG;

    float tol = 0.5f;
    if (httpd_query_key_value(qs, "tol", v, sizeof(v)) == ESP_OK) {
        tol = strtof(v, NULL);
        if (tol < 0.2f) tol = 0.2f;
        if (tol > 5.0f) tol = 5.0f;
    }
    int settle_ms = CAL_SETTLE_MS;
    if (httpd_query_key_value(qs, "settle", v, sizeof(v)) == ESP_OK) {
        settle_ms = atoi(v);
        if (settle_ms < 50)   settle_ms = 50;
        if (settle_ms > 5000) settle_ms = 5000;
    }
    bool brake = false;
    if (httpd_query_key_value(qs, "brake", v, sizeof(v)) == ESP_OK)
        brake = (v[0] != '0');

    sensor_rail_on();
    motor_rail_on();
    bool ok = valve_goto(target, tol, 10000, false);
    if (brake) gpio_set_level(GPIO_K, 1);   // hold under short-brake during settle+read
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
    uint16_t vraw=0; uint8_t vagc=0, vst=0;
    as5600_read(ADDR_AS5600L, &vraw, &vagc, &vst);
    float psi = cal_read_pressure_avg();
    if (brake) gpio_set_level(GPIO_K, 0);   // release before returning so next move drives

    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":%s,\"commanded\":%.2f,\"achieved\":%.2f,\"err\":%.2f,\"psi\":%.4f,"
        "\"brake\":%s,\"settle\":%d,\"agc\":%u,\"status\":\"%02x\"}",
        ok ? "true" : "false", target, vraw*(360.0f/4096.0f),
        vraw*(360.0f/4096.0f) - target, psi,
        brake ? "true" : "false", settle_ms, vagc, vst);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t cal_encoder_handler(httpd_req_t *req)
{
    sensor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(50));
    uint16_t vraw=0, nraw=0; uint8_t vagc=0, nagc=0, vst=0, nst=0;
    as5600_read(ADDR_AS5600L, &vraw, &vagc, &vst);
    as5600_read(ADDR_AS5600,  &nraw, &nagc, &nst);
    char buf[256]; int n=0;
    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"valve\":{\"deg\":%.2f,\"agc\":%u,\"status\":\"%02x\"},"
        "\"nozzle\":{\"deg\":%.2f,\"agc\":%u,\"status\":\"%02x\"}}",
        vraw*(360.0f/4096.0f), vagc, vst,
        nraw*(360.0f/4096.0f), nagc, nst);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// b391: per-unit valve frame calibration. The valve has NO mechanical hard
// stop, so the frame is referenced to the pressure PEAK (ball-valve max-flow
// angle). REQUIRES WATER ON. Two-phase: a coarse settled-median sweep brackets
// the peak, then a fine re-probe refines it -- both use a stationary median so
// water-hammer transients can't latch the argmax (the b390 mean-based sweep
// overshot ba1f88's peak by ~6 deg off one spike). Sets g_valve_offset_deg =
// peak - 306.7, persists to NVS, parks the valve closed (peak - 90, geometry).
static esp_err_t cal_valve_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(150));

    // b418: polarity probe FIRST -- everything below servos closed-loop.
    int sense = motor_dir_probe(GPIO_VFWD, GPIO_VREV, ADDR_AS5600L, NULL);
    if (sense == 0) {
        motor_rail_off();
        httpd_resp_sendstr(req,
            "{\"error\":\"polarity probe: valve never moved -- check motor power/wiring, then retry\"}");
        return ESP_OK;
    }
    int mdir = -sense;   // reference wiring: VFWD DECREASES angle (closes)
    if (mdir != g_valve_motor_dir)
        ESP_LOGW(TAG, "Valve motor polarity changed: %+d -> %+d", g_valve_motor_dir, mdir);
    g_valve_motor_dir = (int8_t)mdir;
    motor_dir_nvs_save("dir", (int8_t)mdir);

    // Phase 1 -- coarse sweep from the current angle. Settle, then read a
    // SETTLED MEDIAN (not an in-motion mean) so transient pressure spikes can't
    // latch the argmax. No upfront flow gate: from closed, pressure is ~0 even
    // with water on; we instead require real flow somewhere in the sweep.
    uint16_t raw = 0; as5600_read(ADDR_AS5600L, &raw, NULL, NULL);
    float start = raw * (360.0f / 4096.0f);
    float deg = start;
    float best_deg = start, best_psi = -1.0f;
    int falling = 0;
    const float STEP = 3.0f;
    const float SPAN = 120.0f;             // sweep up to start + 120 deg
    for (int i = 0; i < 50 && deg < start + SPAN; i++) {
        valve_goto_direct(deg, 0.5f, 4000, false);
        vTaskDelay(pdMS_TO_TICKS(1500));            // let motion transients dissipate
        float psi = cal_pressure_settled_median(5);
        if (psi >= 0.0f) {
            if (psi > best_psi) { best_psi = psi; best_deg = deg; falling = 0; }
            else if (psi < best_psi - 0.40f) { if (++falling >= 3) break; }  // clearly past peak
        }
        deg += STEP;
    }

    // Must have seen real flow, else the supply is off / too low to calibrate.
    if (best_psi < 1.0f) {
        valve_goto_direct(start, 2.0f, 8000, false);
        motor_rail_off();
        char ebuf[150];
        int en = snprintf(ebuf, sizeof(ebuf),
            "{\"error\":\"no flow (max %.2f PSI) -- turn the water supply ON, then retry\"}",
            best_psi);
        httpd_resp_send(req, ebuf, en);
        return ESP_OK;
    }

    // Phase 2 -- refine. Re-probe a fine window around the coarse peak with a
    // longer settle + median. A one-off transient that won the coarse argmax
    // won't reproduce here, so the true broad peak wins.
    float peak = best_deg, peak_psi = best_psi;
    for (float d = best_deg - STEP; d <= best_deg + STEP + 0.01f; d += 1.0f) {
        valve_goto_direct(d, 0.4f, 4000, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        float psi = cal_pressure_settled_median(7);
        if (psi > peak_psi) { peak_psi = psi; peak = d; }
    }
    best_psi = peak_psi;

    float offset = peak - 306.7f;
    if (offset <= -60.0f || offset >= 60.0f) {
        // Implausible -- don't store, but still park closed by ball geometry.
        valve_goto_direct(peak - 90.0f, 2.0f, 10000, false);
        motor_rail_off();
        char ebuf[180];
        int en = snprintf(ebuf, sizeof(ebuf),
            "{\"error\":\"implausible offset %.2f (peak %.2f, max %.3f PSI) -- not stored\"}",
            offset, peak, best_psi);
        httpd_resp_send(req, ebuf, en);
        return ESP_OK;
    }

    g_valve_offset_deg = offset;
    valve_offset_nvs_save(offset);

    // Park fully closed: 90 deg back from the pressure peak (ball geometry).
    float closed = peak - 90.0f;
    valve_goto_direct(closed, 2.0f, 10000, true);
    motor_rail_off();

    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":true,\"peak_deg\":%.2f,\"offset\":%.2f,\"closed_deg\":%.2f,"
        "\"max_psi\":%.3f,\"motor_dir\":%d}",
        peak, offset, closed, best_psi, (int)g_valve_motor_dir);
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// b390: set the valve frame offset directly (no sweep). Used to push a known
// offset -- e.g. one decoded from the factory valve_home, or a hand-verified
// value -- and persist it. POST /cal/valve/set?offset=<deg>
static esp_err_t cal_valve_set_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    char qs[48] = {0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char v[16] = {0};
    if (httpd_query_key_value(qs, "offset", v, sizeof(v)) != ESP_OK) {
        httpd_resp_sendstr(req, "{\"error\":\"missing offset\"}");
        return ESP_OK;
    }
    float off = strtof(v, NULL);
    if (off <= -60.0f || off >= 60.0f) {
        httpd_resp_sendstr(req, "{\"error\":\"offset out of range (-60,60)\"}");
        return ESP_OK;
    }
    g_valve_offset_deg = off;
    valve_offset_nvs_save(off);
    char buf[140];
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":true,\"offset\":%.2f,\"closed_deg\":%.2f,\"open_deg\":%.2f,\"peak_deg\":%.2f}",
        off, 231.0f + off, 308.0f + off, 306.7f + off);
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

// b396: AP setup mode -- a direct phone<->device link for zone setup when the
// router hop is weak (e.g. a unit parked behind a rock). /wifi/ap_mode drops the
// station so the device's own AP becomes active (reach the UI at
// http://192.168.4.1); /wifi/reconnect reboots, which reloads the compiled WiFi
// creds and rejoins the home network. The reboot self-revert is the safety net
// -- the device can't be stranded in AP mode (a power-cycle also recovers it).
extern void irrigoto_wifi_ap_mode(void);  // bridge from irrigoto.cpp

static esp_err_t wifi_ap_mode_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req,
        "{\"ok\":true,\"msg\":\"AP mode: join the device's WiFi AP, then browse http://192.168.4.1\"}");
    // Keep the device awake so the AP stays up for setup. Set the flag DIRECTLY
    // (RAM-only) rather than irrigoto_set_auto_sleep_enabled(), which persists to
    // NVS -- we want the next reboot to restore the stored sleep setting.
    s_sleep_disabled = true;
    // Drop the station only AFTER the reply has flushed over the current link.
    vTaskDelay(pdMS_TO_TICKS(400));
    irrigoto_wifi_ap_mode();
    return ESP_OK;
}

static esp_err_t wifi_reconnect_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"Rebooting to reconnect to the network...\"}");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  // not reached
}

// b398: zone import/export -- the foundation for HA zone backup/restore and
// (later) editing. GET /zone/export?id=N emits a full, round-trippable zone
// JSON (every point field, unlike the lossy /api/all). POST /zone/import accepts
// that JSON, parses + validates it, and saves via the same zone_save_primary the
// web "save" path uses. Target id: an explicit URL query (?id=N overwrites/
// restores, ?id=new) is authoritative; otherwise the body's NUMERIC id is used.
// The default whenever no numeric target is given is "new" -- an import can
// never silently overwrite an existing zone (a non-numeric body id like "new"
// is numeric-guarded in storage_zone_parse_json, so it can't atoi to 0).
static esp_err_t zone_export_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char ids[8]={0}; httpd_query_key_value(qs, "id", ids, sizeof(ids));
    uint16_t zone_id = (uint16_t)atoi(ids);

    zone_perimeter_t zp={0}; char name[32]="Zone"; bool ok=false;
    if (storage_ready()) {
        char lfsname[32]={0};
        if (storage_zone_load(zone_id, lfsname, sizeof(lfsname), &zp)==ESP_OK) {
            zone_name_resolve(zone_id, lfsname, "Zone", name, sizeof(name));
            ok=true;
        }
    }
    if (!ok && zone_id==0) {  // pure-NVS fallback for zone 0 (matches api_zone_handler)
        zone_load_nvs(&zp);
        zone_name_resolve(0, NULL, "Zone", name, sizeof(name));
        ok = zp.num_points>0;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    HTTP_CONN_CLOSE(req);
    if (!ok) { httpd_resp_sendstr(req, "{\"error\":\"no such zone\"}"); return ESP_OK; }

    char *buf = malloc(4096);   // up to 36 pts x ~90B + header; heap (httpd stack ~8KB)
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_OK; }
    int n = snprintf(buf, 4096,
        "{\"id\":%u,\"name\":\"%s\",\"num_points\":%u,\"points\":[", zone_id, name, zp.num_points);
    for (int i = 0; i < zp.num_points && n < 4096-130; i++) {
        const perimeter_point_t *p = &zp.points[i];
        n += snprintf(buf+n, 4096-n,
            "%s{\"nozzle_deg\":%.2f,\"valve_deg\":%.2f,\"pressure\":%.4f,\"throw_mm\":%.1f,\"walk_idx\":%u}",
            i?",":"", p->nozzle_deg, p->valve_deg, p->pressure_psi, p->throw_mm, p->walk_idx);
    }
    n += snprintf(buf+n, 4096-n, "]}");
    httpd_resp_send(req, buf, n);
    free(buf);
    return ESP_OK;
}

static esp_err_t zone_import_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int clen = req->content_len;
    if (clen <= 0 || clen > 8192) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"empty or oversized body (max 8192)\"}");
        return ESP_OK;
    }
    char *body = malloc(clen+1);
    if (!body) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_OK; }
    int got = 0;
    while (got < clen) {
        int r = httpd_req_recv(req, body+got, clen-got);
        if (r <= 0) { free(body); httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"recv failed\"}"); return ESP_OK; }
        got += r;
    }
    body[got] = 0;

    int pid=-1, np=0; char name[32]="Zone";
    static zone_perimeter_t zimp;   // static: too big for the ~8KB httpd stack
    esp_err_t pe = storage_zone_parse_json(body, &pid, name, sizeof(name), &zimp, &np);
    free(body);
    if (pe != ESP_OK) { httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"parse failed\"}"); return ESP_OK; }
    if (np <= 0)      { httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no points\"}");    return ESP_OK; }

    // Target id: an explicit URL query (?id=N / ?id=new) is authoritative and
    // overrides the body's id. This is the safe, unambiguous control -- "new" is
    // a string compare (no atoi trap), and the default (no numeric target) is
    // ALWAYS "new", so an import can never SILENTLY overwrite an existing zone.
    char qs[24]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char qid[8]={0};
    if (httpd_query_key_value(qs, "id", qid, sizeof(qid)) == ESP_OK && qid[0]) {
        if (strcmp(qid, "new") == 0) pid = -1;
        else { int q = atoi(qid); pid = (qid[0]=='0' || q>0) ? q : -1; }
    }
    uint16_t zone_id;
    if (pid >= 0 && pid < STORAGE_MAX_ZONES) {
        zone_id = (uint16_t)pid;                 // explicit numeric id -> overwrite (restore)
    } else {                                     // "new"/absent/non-numeric -> next free id
        uint16_t avail[STORAGE_MAX_ZONES]; int nc=0;
        storage_zone_list(avail, STORAGE_MAX_ZONES, &nc);
        zone_id = (uint16_t)nc;
    }

    // b400: ?recompute=1 (the on-device drag editor passes this) re-derives each
    // point's pressure + valve angle from its (possibly dragged) throw via the
    // cal, so the point is self-consistent and actually waters to its new
    // distance -- watering derives throw from per-point pressure. Phase-1 restore
    // omits it, preserving stored pressure/valve exactly.
    char rec[4]={0};
    if (httpd_query_key_value(qs, "recompute", rec, sizeof(rec)) == ESP_OK && rec[0]=='1') {
        for (int i = 0; i < zimp.num_points; i++) {
            float t = zimp.points[i].throw_mm;
            zimp.points[i].pressure_psi = cal_throw_to_psi(t);
            zimp.points[i].valve_deg    = cal_throw_to_valve_deg(t);
        }
    }

    esp_err_t se = zone_save_primary(zone_id, name, &zimp);
    if (se != ESP_OK) {
        char eb[96]; int en = snprintf(eb, sizeof(eb),
            "{\"ok\":false,\"error\":\"save failed: %s\"}", esp_err_to_name(se));
        httpd_resp_sendstr(req, eb);
        return ESP_OK;
    }
    // b400: if the import targets the zone the web editor is currently showing,
    // sync the live in-memory buffer so /zone/state reflects the saved edit on
    // the next poll (otherwise the page would show the pre-edit s_web_zone).
    if (zone_id == s_web_zone_id) {
        s_web_zone = zimp;
        snprintf(s_web_zone_name, sizeof(s_web_zone_name), "%s", name);
    }

    char ob[96]; int on = snprintf(ob, sizeof(ob),
        "{\"ok\":true,\"id\":%u,\"num_points\":%d}", zone_id, np);
    ESP_LOGI(TAG, "Zone %u imported: %s (%d pts)", zone_id, name, np);
    httpd_resp_sendstr(req, ob);
    return ESP_OK;
}

// b417: POST /zone/trace?id=N[&speed=D][&loop=0|1] -- glide the stream
// around zone N's perimeter in one continuous motion (zone_trace_run;
// spawns a task and returns immediately). POST /zone/trace?stop=1 aborts
// an active glide -- the trace task closes the valve on its way out.
static esp_err_t zone_trace_handler(httpd_req_t *req)
{
    char qs[64] = {0};
    httpd_req_get_url_query_str(req, qs, sizeof(qs));
    httpd_resp_set_type(req, "application/json");
    HTTP_CONN_CLOSE(req);

    char stop_s[4] = {0};
    if (httpd_query_key_value(qs, "stop", stop_s, sizeof(stop_s)) == ESP_OK &&
        stop_s[0] == '1') {
        irrigoto_zone_trace_stop();
        httpd_resp_sendstr(req, "{\"ok\":true,\"stopping\":true}");
        return ESP_OK;
    }

    char id_s[8] = {0};
    if (httpd_query_key_value(qs, "id", id_s, sizeof(id_s)) != ESP_OK || !id_s[0]) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing id\"}");
        return ESP_OK;
    }
    float speed = 6.0f;
    char sp_s[8] = {0};
    if (httpd_query_key_value(qs, "speed", sp_s, sizeof(sp_s)) == ESP_OK && sp_s[0])
        speed = strtof(sp_s, NULL);
    bool loop = true;
    char lp_s[4] = {0};
    if (httpd_query_key_value(qs, "loop", lp_s, sizeof(lp_s)) == ESP_OK &&
        lp_s[0] == '0')
        loop = false;

    bool ok = irrigoto_zone_trace_start((uint16_t)atoi(id_s), speed, loop);
    httpd_resp_sendstr(req, ok ? "{\"ok\":true,\"started\":true}"
                               : "{\"ok\":false,\"error\":\"busy or no such zone\"}");
    return ESP_OK;
}

// ---- Filesystem browser handlers ----

static esp_err_t fs_page_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, s_fs_html);
    return ESP_OK;
}

static int fs_list_dir(const char *path, char *buf, int bsz, int n, int depth)
{
    DIR *d = opendir(path);
    if (!d) return n;
    bool first = true;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        if (!first && n < bsz-4) buf[n++] = ',';
        first = false;
        char full[384];
        snprintf(full, sizeof(full), "%s/%s", path, ent->d_name);
        // Use stat() to determine type -- ent->d_type is DT_UNKNOWN on LittleFS
        struct stat st={0};
        stat(full, &st);
        if (S_ISDIR(st.st_mode)) {
            n += snprintf(buf+n, bsz-n,
                "{\"type\":\"dir\",\"name\":\"%s\","
                "\"path\":\"%s\",\"children\":[",
                ent->d_name, full);
            n = fs_list_dir(full, buf, bsz, n, depth+1);
            if (n < bsz-4) { buf[n++]=']'; buf[n++]='}'; }
        } else {
            n += snprintf(buf+n, bsz-n,
                "{\"type\":\"file\",\"name\":\"%s\","
                "\"path\":\"%s\",\"size\":%ld}",
                ent->d_name, full, (long)st.st_size);
        }
    }
    closedir(d);
    return n;
}

static esp_err_t fs_list_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    if (!storage_ready()) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"not mounted\",\"entries\":[]}");
        return ESP_OK;
    }
    static char buf[6144];
    size_t used=0, total=0;
    storage_usage(&used, &total);
    int n = snprintf(buf, sizeof(buf),
        "{\"used\":%u,\"total\":%u,\"entries\":[",
        (unsigned)used, (unsigned)total);
    n = fs_list_dir("/lfs", buf, (int)sizeof(buf)-8, n, 0);
    buf[n++]=']'; buf[n++]='}'; buf[n]=0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t fs_download_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    char qs[256]={0};
    httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char path[128]={0};
    httpd_query_key_value(qs, "path", path, sizeof(path));
    url_decode(path, sizeof(path));
    if (!path[0] || strstr(path, "..")) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
        return ESP_OK;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_OK;
    }
    const char *fname = strrchr(path, '/');
    fname = fname ? fname+1 : path;
    // .wbin files: convert to CSV on the fly so the download is human-readable.
    size_t fnlen = strlen(fname);
    if (fnlen > 5 && strcmp(fname + fnlen - 5, ".wbin") == 0) {
        uint32_t magic = 0;
        fread(&magic, sizeof(magic), 1, f);
        if (magic == WATER_BIN_MAGIC) {
            char csv_fname[128];
            snprintf(csv_fname, sizeof(csv_fname), "%.*s.csv", (int)(fnlen - 5), fname);
            char disp[256];
            snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", csv_fname);
            httpd_resp_set_type(req, "text/csv");
            httpd_resp_set_hdr(req, "Content-Disposition", disp);
            httpd_resp_sendstr_chunk(req,
                "time_s,ring,arc,sector,"
                "nozzle_deg_target,nozzle_deg_actual,"
                "valve_deg_target,valve_deg_actual,"
                "throw_mm_target,throw_mm_actual,"
                "pressure_psi_target,pressure_psi_actual,"
                "target_x,target_y,actual_x,actual_y,pass_type,active\n");
            water_row_t row;
            char line[200];
            while (fread(&row, sizeof(row), 1, f) == 1) {
                int n = snprintf(line, sizeof(line),
                    "%.1f,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%d,%d,%.3f,%.3f,%d,%d,%d,%d,%u,%u\n",
                    row.time_ds  * 0.1f,
                    (int)row.ring, (int)row.arc, (int)row.sector,
                    row.nozzle_deg_target_d * 0.1f, row.nozzle_deg_actual_d * 0.1f,
                    row.valve_deg_target_d  * 0.1f, row.valve_deg_actual_d  * 0.1f,
                    (int)row.throw_mm_target, (int)row.throw_mm_actual,
                    row.psi_target_c * 0.01f, row.psi_actual_c * 0.01f,
                    (int)row.target_x, (int)row.target_y,
                    (int)row.actual_x, (int)row.actual_y,
                    (unsigned)row.pass_type, (unsigned)row.active);
                httpd_resp_send_chunk(req, line, n);
            }
            httpd_resp_send_chunk(req, NULL, 0);
            fclose(f);
            return ESP_OK;
        }
        fseek(f, 0, SEEK_SET);  // not a valid wbin -- fall through to raw download
    }
    char disp[256];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", fname);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);
    static uint8_t fbuf[512];
    size_t rd;
    while ((rd = fread(fbuf, 1, sizeof(fbuf), f)) > 0)
        httpd_resp_send_chunk(req, (char*)fbuf, rd);
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(f);
    return ESP_OK;
}

static esp_err_t fs_upload_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    bool upload_ok = false;
    char upload_err[96] = {0};
    int total = req->content_len;
    if (total <= 0 || total > 65536) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad size");
        return ESP_OK;
    }
    char *body = malloc(total + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }
    int received = 0, r;
    while (received < total) {
        r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) break;
        received += r;
    }
    body[received] = 0;

    // Parse multipart boundary
    char ct[128]={0};
    httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct));
    char *bnd = strstr(ct, "boundary=");
    if (!bnd) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no boundary");
        return ESP_OK;
    }
    char boundary[72]="--";
    strncat(boundary, bnd+9, 68);
    int blen = strlen(boundary);

    // Extract path field
    char dest[128]="/lfs/upload.bin";
    char *pp = strstr(body, "name=\"path\"");
    if (pp) {
        char *pdata = strstr(pp, "\r\n\r\n");
        if (pdata) {
            pdata += 4;
            char *pend = strstr(pdata, boundary);
            if (pend) {
                int len = (int)(pend - pdata) - 2;
                if (len > 0 && len < (int)sizeof(dest)) {
                    memcpy(dest, pdata, len);
                    dest[len] = 0;
                }
            }
        }
    }

    // Extract file data
    size_t file_size = 0;
    char *fp = strstr(body, "name=\"file\"");
    if (fp) {
        char *fdata = strstr(fp, "\r\n\r\n");
        if (fdata) {
            fdata += 4;
            char *fend = (char*)memmem(fdata, body+received-fdata, boundary, blen);
            if (fend) fend -= 2;
            else fend = body + received;
            file_size = (size_t)(fend - fdata);
            // Recursively create every parent dir of dest so uploads to
            // arbitrary nested paths (e.g. /lfs/a/b/c/file.txt) work.
            // Skips the leaf (everything after the final '/').
            char parent[128];
            strncpy(parent, dest, sizeof(parent)-1);
            parent[sizeof(parent)-1] = 0;
            char *leaf = strrchr(parent, '/');
            if (leaf && leaf != parent) {
                *leaf = 0;  // truncate to the directory
                for (char *p = parent + 1; *p; p++) {
                    if (*p == '/') {
                        *p = 0;
                        if (mkdir(parent, 0755) != 0 && errno != EEXIST)
                            ESP_LOGW(TAG, "fs_upload: mkdir('%s') failed: %s",
                                     parent, strerror(errno));
                        *p = '/';
                    }
                }
                if (mkdir(parent, 0755) != 0 && errno != EEXIST)
                    ESP_LOGW(TAG, "fs_upload: mkdir('%s') failed: %s",
                             parent, strerror(errno));
            }
            FILE *wf = fopen(dest, "wb");
            if (wf) {
                size_t wr = fwrite(fdata, 1, file_size, wf);
                fclose(wf);
                if (wr == file_size) {
                    ESP_LOGI(TAG, "fs_upload: wrote %u bytes to '%s'",
                             (unsigned)file_size, dest);
                    upload_ok = true;
                } else {
                    ESP_LOGE(TAG, "fs_upload: short write %u/%u to '%s'",
                             (unsigned)wr, (unsigned)file_size, dest);
                    snprintf(upload_err, sizeof(upload_err),
                             "short write %u/%u bytes",
                             (unsigned)wr, (unsigned)file_size);
                }
            } else {
                ESP_LOGE(TAG, "fs_upload: fopen('%s') failed: errno=%d (%s)",
                         dest, errno, strerror(errno));
                snprintf(upload_err, sizeof(upload_err),
                         "fopen failed: %s", strerror(errno));
            }
        }
    }
    free(body);
    char resp[200];
    httpd_resp_set_type(req, "application/json");
    if (upload_ok) {
        snprintf(resp, sizeof(resp),
            "{\"ok\":true,\"path\":\"%s\",\"size\":%u}",
            dest, (unsigned)file_size);
    } else {
        httpd_resp_set_status(req, "500 Internal Server Error");
        snprintf(resp, sizeof(resp),
            "{\"ok\":false,\"path\":\"%s\",\"error\":\"%s\"}",
            dest, upload_err[0] ? upload_err : "unknown failure");
    }
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t fs_delete_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    char body[256]={0};
    httpd_req_recv(req, body, sizeof(body)-1);
    char path[128]={0};
    httpd_query_key_value(body, "path", path, sizeof(path));
    url_decode(path, sizeof(path));
    if (!path[0] || strstr(path, "..") || !strstr(path, "/lfs/")) {
        httpd_resp_sendstr(req, "{\"error\":\"bad path\"}");
        return ESP_OK;
    }
    if (remove(path) == 0) {
        ESP_LOGI(TAG, "fs_delete: removed '%s'", path);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "fs_delete: remove('%s') failed: errno=%d (%s)",
                 path, errno, strerror(errno));
        httpd_resp_set_status(req, "500 Internal Server Error");
        char err_buf[160];
        snprintf(err_buf, sizeof(err_buf),
                 "{\"ok\":false,\"error\":\"delete failed: %s\"}",
                 strerror(errno));
        httpd_resp_sendstr(req, err_buf);
    }
    return ESP_OK;
}

static esp_err_t cal_jog_start_handler(httpd_req_t *req)
{
    if (s_wcal.state == WCAL_PRESSURE_SCANNING ||
        s_wcal.state == WCAL_NOZZLE_RUNNING ||
        s_wcal.state == WCAL_JOG_RUNNING) {
        httpd_resp_sendstr(req, "{\"error\":\"calibration already running\"}");
        return ESP_OK;
    }
    wcal_reset();
    s_wcal.state = WCAL_JOG_RUNNING;
    s_jog_web_mode = true;
    xTaskCreatePinnedToCore(wcal_jog_task, "wcal_jog", 8192, NULL, 8, NULL, APP_CPU_NUM);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t cal_jog_cancel_handler(httpd_req_t *req)
{
    s_wcal.state = WCAL_IDLE;
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off(); sensor_rail_off();
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Cancelled.");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t cal_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");
    httpd_resp_sendstr(req, s_cal_html);
    return ESP_OK;
}

// Forward decl — the implementation lives next to s_schedule so the static
// is in scope. Keeps the schedule data hidden from this section while still
// letting the web handler read a consistent snapshot.
static void schedule_snapshot(schedule_t *out);

// GET /schedule -> the standalone schedule editor page (embedded HTML).
static esp_err_t schedule_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");
    httpd_resp_sendstr(req, s_schedule_html);
    return ESP_OK;
}

// Build a JSON description of the current schedule for the editor page:
//   {fw_build, now, tz_offset_min, last_status, max_entries,
//    next_run:{zone,epoch} | null,
//    zones:[{id,name},...],
//    entries:[{zone,mode,depth,hour,minute,days_mask,enabled},...]}
//
// Posting to the same URL replaces the schedule. Body: text=ENCODED, using
// the same ';'-separated text format as the HA set_schedule service. We
// keep both representations in one shape so external callers (HA, scripts)
// can use whichever they have on hand without a separate API surface.
static esp_err_t api_schedule_handler(httpd_req_t *req)
{
    // The big working buffers (body, encoded, response JSON) are static —
    // the httpd task has only 8 KB of stack and three ~1-3 KB locals here
    // would risk overflow once snprintf + littlefs reads are stacked on
    // top. The httpd server processes URI handlers one at a time per
    // server instance, so a single shared scratch is safe.
    static char body[1024];
    static char encoded[1024];
    static char buf[3072];

    HTTP_CONN_CLOSE(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");

    if (req->method == HTTP_POST) {
        // Bound the read by content_len — otherwise the loop calls recv
        // again after the body is fully consumed, blocks for the server's
        // recv_wait_timeout (3 s), and only returns once the timeout fires.
        // Bodies > sizeof(body)-1 get truncated; the parser will reject
        // them downstream as a malformed schedule.
        int want = req->content_len;
        if (want < 0) want = 0;
        if (want > (int)sizeof(body) - 1) want = (int)sizeof(body) - 1;
        int total = 0;
        while (total < want) {
            int chunk = httpd_req_recv(req, body + total, want - total);
            if (chunk <= 0) {
                if (chunk == HTTPD_SOCK_ERR_TIMEOUT) continue;  // retry once
                break;
            }
            total += chunk;
        }
        body[total] = '\0';
        if (total <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
            return ESP_OK;
        }
        // The text= parameter is URL-encoded form data. An empty text=
        // (or missing field) is treated as "clear" so the editor can
        // wipe the schedule via the same endpoint.
        encoded[0] = '\0';
        httpd_query_key_value(body, "text", encoded, sizeof(encoded));
        url_decode(encoded, sizeof(encoded));
        if (encoded[0] == '\0') {
            irrigoto_schedule_clear();
        } else {
            (void)irrigoto_schedule_set_text(encoded);
            // Result reflected via s_sched_last_status below.
        }
    }

    // Build response JSON.
    int n = 0;
    // b407: emit the response as HTTP chunks. The zones+entries arrays scale
    // with zone count (up to STORAGE_MAX_ZONES) and SCHEDULE_MAX_ENTRIES, so
    // the old fixed buf[3072] silently truncated once both grew. buf is now a
    // reusable accumulator: flush it whenever it nears full (each zone/entry
    // is < 256 B, so one always fits after a flush), then a final flush +
    // terminator closes the stream. Header snprintfs stay bounded < buf.
    #define SCHED_FLUSH_IF_FULL() do { \
        if (n > (int)sizeof(buf) - 256) { httpd_resp_send_chunk(req, buf, n); n = 0; } \
    } while (0)

    time_t now = time(NULL);
    // tz offset: difference between local and UTC at "now" in minutes.
    // Convention: positive east of UTC, so EDT = -240, EST = -300.
    //
    // Compute by decomposing the same epoch with localtime_r and gmtime_r
    // and subtracting the wall-clock fields. This honors libc's POSIX TZ
    // DST rules (selected by localtime_r based on the date), unlike a
    // bare mktime call which has to be told the isdst flag externally.
    // Day-of-year wrap is the only edge case — handle it explicitly.
    int tz_off_min = 0;
    int diag_isdst = -2;
    if (now > 1700000000) {
        struct tm lt, gt;
        localtime_r(&now, &lt);
        gmtime_r(&now, &gt);
        diag_isdst = lt.tm_isdst;
        int diff_sec = (lt.tm_hour - gt.tm_hour) * 3600
                     + (lt.tm_min  - gt.tm_min)  * 60
                     + (lt.tm_sec  - gt.tm_sec);
        // tm_yday wraps at year boundary; clamp to ±1 day delta which is
        // the only physically possible difference between local and UTC
        // decompositions of the same instant.
        int diff_days = lt.tm_yday - gt.tm_yday;
        if (lt.tm_year != gt.tm_year) {
            diff_days = (lt.tm_year > gt.tm_year) ? 1 : -1;
        }
        diff_sec += diff_days * 86400;
        tz_off_min = diff_sec / 60;
    }
    // One-line diagnostic so we can verify libc TZ is honoring the
    // POSIX DST rules. Logged at INFO so it shows up in standard logs
    // — only fires on schedule-page loads, so it's not spammy.
    ESP_LOGI(TAG, "schedule api: now=%ld isdst=%d tz_off_min=%d",
             (long)now, diag_isdst, tz_off_min);

    char status[160] = {0};
    irrigoto_schedule_get_last_status(status, sizeof(status));

    time_t nr_t = 0; int nr_zone = 0;
    bool have_next = irrigoto_schedule_next_run(now, &nr_t, &nr_zone);

    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"fw_build\":%u,\"now\":%ld,\"tz_offset_min\":%d,"
        "\"max_entries\":%d,\"last_status\":\"",
        FW_BUILD, (long)now, tz_off_min, SCHEDULE_MAX_ENTRIES);
    // Escape status for JSON (it can contain quotes from parse-error
    // messages). Cheap: drop control chars, escape " and \.
    for (const char *p = status; *p && n < (int)sizeof(buf)-8; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') { buf[n++] = '\\'; buf[n++] = c; }
        else if (c >= 0x20)         { buf[n++] = c; }
    }
    n += snprintf(buf+n, sizeof(buf)-n, "\",");

    if (have_next) {
        n += snprintf(buf+n, sizeof(buf)-n,
            "\"next_run\":{\"zone\":%d,\"epoch\":%ld},",
            nr_zone, (long)nr_t);
    } else {
        n += snprintf(buf+n, sizeof(buf)-n, "\"next_run\":null,");
    }
    // Rain / wind delay state for the editor's "delay" card. Epoch
    // matches the schedule_changed event's delay_until field. 0 means
    // no delay active.
    n += snprintf(buf+n, sizeof(buf)-n,
        "\"delay_until\":%ld,",
        (long)irrigoto_schedule_get_delay_until());
    // Last completed watering — surfaces on the landing schedule card
    // and on the editor status header. Epoch 0 means nothing has run
    // since boot. zone is 1-based; status follows the watering_complete
    // event's vocabulary (completed / cancelled / valve_fault / ...).
    time_t lw_epoch = irrigoto_last_water_finish_epoch();
    if (lw_epoch > 1700000000) {
        char lw_zname[32] = {0};
        char lw_status[24] = {0};
        irrigoto_last_water_zone_name(lw_zname, sizeof(lw_zname));
        irrigoto_last_water_status_str(lw_status, sizeof(lw_status));
        n += snprintf(buf+n, sizeof(buf)-n,
            "\"last_run\":{\"zone\":%d,\"name\":\"",
            irrigoto_last_water_zone());
        for (const char *p = lw_zname; *p && n < (int)sizeof(buf)-8; p++) {
            unsigned char c = (unsigned char)*p;
            if (c == '"' || c == '\\') { buf[n++] = '\\'; buf[n++] = c; }
            else if (c >= 0x20)         { buf[n++] = c; }
        }
        n += snprintf(buf+n, sizeof(buf)-n,
            "\",\"epoch\":%ld,\"status\":\"%s\"},",
            (long)lw_epoch, lw_status);
    } else {
        n += snprintf(buf+n, sizeof(buf)-n, "\"last_run\":null,");
    }

    // Zone list — use the same lookup path as /api/all so names stay
    // consistent between landing page and schedule editor.
    n += snprintf(buf+n, sizeof(buf)-n, "\"zones\":[");
    int zcount = 0;
    if (storage_ready()) {
        uint16_t ids[STORAGE_MAX_ZONES]; int nc = 0;
        storage_zone_list(ids, STORAGE_MAX_ZONES, &nc);
        for (int i = 0; i < nc; i++) {
            // b407: flush before each zone so the response can't truncate.
            // This handler scales with BOTH zone count (up to STORAGE_MAX_
            // ZONES=100) and entry count, so its worst case far exceeds buf;
            // chunking it (like /api/all) removes the cap. One zone is < 256 B,
            // so flushing whenever n nears the top keeps every append in-bounds.
            SCHED_FLUSH_IF_FULL();
            zone_perimeter_t zp = {0};
            char zn[32] = {0};
            if (storage_zone_load(ids[i], zn, sizeof(zn), &zp) != ESP_OK) continue;
            char zdef[16]; snprintf(zdef, sizeof(zdef), "Zone #%u", ids[i]);
            char zname[32];
            zone_name_resolve(ids[i], zn, zdef, zname, sizeof(zname));
            // Escape name (commas and quotes are realistic — users can type anything).
            n += snprintf(buf+n, sizeof(buf)-n,
                "%s{\"id\":%u,\"name\":\"", zcount ? "," : "", ids[i]);
            for (const char *p = zname; *p && n < (int)sizeof(buf)-8; p++) {
                unsigned char c = (unsigned char)*p;
                if (c == '"' || c == '\\') { buf[n++] = '\\'; buf[n++] = c; }
                else if (c >= 0x20)         { buf[n++] = c; }
            }
            n += snprintf(buf+n, sizeof(buf)-n, "\"}");
            zcount++;
        }
    }
    n += snprintf(buf+n, sizeof(buf)-n, "],\"entries\":[");

    // Snapshot the live schedule. s_schedule is static further down in this
    // file; schedule_snapshot() is forward-declared above the handler and
    // defined alongside the schedule code so this handler can sit with the
    // rest of the web handlers.
    schedule_t snap = {0};
    schedule_snapshot(&snap);
    for (uint8_t i = 0; i < snap.count; i++) {
        // b407: flush before each entry (each is < 256 B). See zone loop above.
        SCHED_FLUSH_IF_FULL();
        const schedule_entry_t *e = &snap.entries[i];
        // b355: include id + last_modified + source so HA / web UI can
        // round-trip through sync_schedule without re-keying entries.
        n += snprintf(buf+n, sizeof(buf)-n,
            "%s{\"id\":%lu,\"last_modified\":%lu,\"client_tag\":%lu,"
            "\"zone\":%u,\"mode\":%u,\"depth\":%u,"
            "\"hour\":%u,\"minute\":%u,\"days_mask\":%u,\"enabled\":%u,"
            "\"source\":%u}",
            i ? "," : "",
            (unsigned long)e->id, (unsigned long)e->last_modified,
            (unsigned long)e->client_tag,
            e->zone, e->mode, e->depth,
            e->hour, e->minute, e->days_mask, e->enabled, e->source);
    }
    n += snprintf(buf+n, sizeof(buf)-n, "],");
    // b407: flush before the tail (schedule_version + ok + escaped status,
    // up to ~400 B) so a near-full buffer from the entries loop can't truncate
    // it. The escaped status string is the only remaining unbounded-ish field.
    SCHED_FLUSH_IF_FULL();
    // b355: schedule_version on the response so the editor can show
    // "behind by N revisions" if HA changed it mid-edit. Same number
    // surfaced by the schedule_version text_sensor to HA.
    n += snprintf(buf+n, sizeof(buf)-n, "\"schedule_version\":%lu",
                  (unsigned long)irrigoto_schedule_version());
    // (note: trailing comma intentionally absent; closing block below
    // appends "," then "ok"/"status" pair.)

    // The response shape doubles as the POST result: include {ok,status} so
    // the page can show the outcome without a second fetch.
    // A successful sync sets status = "sync ok (N entries, M applied)" (see
    // schedule_sync_apply), which starts with "sync", not "ok" -- so the old
    // check reported ok:false on every successful sync. Accept the "sync ok"
    // prefix too. All error statuses are "sync out-of-range/parse failed/bad
    // tombstone", none of which match "sync ok", so this stays strict.
    bool ok = (strncmp(status, "ok", 2) == 0) ||
              (strncmp(status, "sync ok", 7) == 0) ||
              (strcmp(status, "no schedule pushed yet") == 0);
    n += snprintf(buf+n, sizeof(buf)-n, ",\"ok\":%s,\"status\":\"",
                  ok ? "true" : "false");
    for (const char *p = status; *p && n < (int)sizeof(buf)-8; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') { buf[n++] = '\\'; buf[n++] = c; }
        else if (c >= 0x20)         { buf[n++] = c; }
    }
    n += snprintf(buf+n, sizeof(buf)-n, "\"}");

    if (n > 0) httpd_resp_send_chunk(req, buf, n);
    httpd_resp_send_chunk(req, NULL, 0);  // terminator
    #undef SCHED_FLUSH_IF_FULL
    return ESP_OK;
}

// POST /api/schedule/clear -> wipe all entries. Convenience for the
// "Clear all" button on the editor; same effect as POSTing an empty text=.
static esp_err_t api_schedule_clear_handler(httpd_req_t *req)
{
    HTTP_CONN_CLOSE(req);
    irrigoto_schedule_clear();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// POST /api/schedule/delay  body: hours=N
//   N == 0  -> cancel any active delay (resume schedule immediately)
//   N >  0  -> suspend firing for N hours from now
// Same code path as the HA delay_schedule_hours service so the two
// surfaces stay in lockstep. Returns {ok, delay_until} so the page
// can update without a follow-up GET.
static esp_err_t api_schedule_delay_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }
    char hours_s[16] = {0};
    if (httpd_query_key_value(body, "hours", hours_s, sizeof(hours_s)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "hours required");
        return ESP_OK;
    }
    int hours = atoi(hours_s);
    if (hours <= 0) {
        irrigoto_schedule_clear_delay();
    } else {
        irrigoto_schedule_set_delay_hours((uint32_t)hours);
    }
    HTTP_CONN_CLOSE(req);
    httpd_resp_set_type(req, "application/json");
    char resp[64];
    int n = snprintf(resp, sizeof(resp),
                     "{\"ok\":true,\"delay_until\":%ld}",
                     (long)irrigoto_schedule_get_delay_until());
    httpd_resp_send(req, resp, n);
    return ESP_OK;
}

// Serve the raw watering CSV log for heatmap rendering.
// Detects binary .wbin format (WATER_BIN_MAGIC header) and converts to CSV text on the fly.
static esp_err_t zone_water_csv_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char ids[8]={0}; httpd_query_key_value(qs, "id", ids, sizeof(ids));
    uint16_t csv_zone_id = ids[0] ? (uint16_t)atoi(ids) : s_web_zone_id;
    char csv_path[64]; water_csv_path_find(csv_zone_id, csv_path, sizeof(csv_path));
    FILE *f = fopen(csv_path, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No watering data");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    HTTP_CONN_CLOSE(req);
    uint32_t magic = 0;
    fread(&magic, sizeof(magic), 1, f);
    if (magic == WATER_BIN_MAGIC) {
        // Binary format: emit CSV header then convert each record.
        httpd_resp_sendstr_chunk(req,
            "time_s,ring,arc,sector,"
            "nozzle_deg_target,nozzle_deg_actual,"
            "valve_deg_target,valve_deg_actual,"
            "throw_mm_target,throw_mm_actual,"
            "pressure_psi_target,pressure_psi_actual,"
            "target_x,target_y,actual_x,actual_y,pass_type,active\n");
        water_row_t row;
        char line[200];
        while (fread(&row, sizeof(row), 1, f) == 1) {
            int n = snprintf(line, sizeof(line),
                "%.1f,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%d,%d,%.3f,%.3f,%d,%d,%d,%d,%u,%u\n",
                row.time_ds  * 0.1f,
                (int)row.ring, (int)row.arc, (int)row.sector,
                row.nozzle_deg_target_d * 0.1f, row.nozzle_deg_actual_d * 0.1f,
                row.valve_deg_target_d  * 0.1f, row.valve_deg_actual_d  * 0.1f,
                (int)row.throw_mm_target, (int)row.throw_mm_actual,
                row.psi_target_c * 0.01f, row.psi_actual_c * 0.01f,
                (int)row.target_x, (int)row.target_y,
                (int)row.actual_x, (int)row.actual_y,
                (unsigned)row.pass_type, (unsigned)row.active);
            httpd_resp_send_chunk(req, line, n);
        }
    } else {
        // Legacy text CSV: rewind and stream as-is.
        fseek(f, 0, SEEK_SET);
        char buf[256];
        while (fgets(buf, sizeof(buf), f))
            httpd_resp_sendstr_chunk(req, buf);
    }
    httpd_resp_sendstr_chunk(req, NULL);
    fclose(f);
    return ESP_OK;
}

// b283/b286: Serve the 1Hz pressure trace from RAM. b286 removed the
// LFS file write -- the buffer in RAM (s_trace_samples / s_trace_rings)
// is the source of truth and is overwritten on the next watering. The
// `id` query parameter is accepted for backward compatibility with old
// download URLs but ignored: there's only one trace buffer, holding the
// most recent run regardless of zone.
//
// Output format matches the b283 CSV exactly so existing tools
// (analyze_water_trace.py) work unchanged.
static esp_err_t zone_water_trace_handler(httpd_req_t *req)
{
    if (s_trace_n_samples == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
            "No trace in RAM -- run a smooth/gentle watering first "
            "(traces are RAM-only since b286; pull before next run)");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    HTTP_CONN_CLOSE(req);

    char line[64];
    httpd_resp_sendstr_chunk(req, "type,time_s,a,b\n");
    httpd_resp_sendstr_chunk(req,
        "# S,time_s,psi,valve_deg   R,time_s,ring,valve_deg\n");
    // Ring boundaries first (then samples) -- matches the b283 file format.
    for (uint16_t i = 0; i < s_trace_n_rings; i++) {
        const water_trace_ring_t *r = &s_trace_rings[i];
        int n = snprintf(line, sizeof(line), "R,%u,%u,%.1f\n",
                         (unsigned)r->time_s_start,
                         (unsigned)(r->ring + 1u),
                         r->valve_deg_d * 0.1f);
        if (n > 0) httpd_resp_send_chunk(req, line, n);
    }
    for (uint16_t i = 0; i < s_trace_n_samples; i++) {
        const water_trace_sample_t *s = &s_trace_samples[i];
        int n = snprintf(line, sizeof(line), "S,%u,%.2f,%.1f\n",
                         (unsigned)s->time_s,
                         s->psi_c * 0.01f,
                         s->valve_deg_d * 0.1f);
        if (n > 0) httpd_resp_send_chunk(req, line, n);
    }
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// b292: Serve the RAM-buffered log captured during the last watering.
// While WiFi is off mid-run, all ESP_LOG output is mirrored into a 24 KB
// ring buffer. On WiFi resume, the buffer holds the most recent log
// lines (oldest scrolled off on overflow). The buffer survives until
// either (a) the next watering re-arms capture, or (b) the device
// reboots. Cleared format: plain text, chronological order.
static esp_err_t zone_last_log_handler(httpd_req_t *req)
{
    if (s_watering_log_pos == 0 && !s_watering_log_wrapped) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
            "No watering log captured yet (b292 captures during smooth/gentle runs only)");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    HTTP_CONN_CLOSE(req);

    if (s_watering_log_wrapped) {
        // Buffer wrapped: send tail (older portion) first, then head.
        // Position 'write_pos' is where the next byte would go, so
        // [write_pos .. end) is the older fragment and
        // [0 .. write_pos) is the newer fragment.
        uint16_t pos = s_watering_log_pos;  // snapshot
        if (pos < WATER_LOG_BUFFER_SIZE) {
            httpd_resp_send_chunk(req, &s_watering_log_buf[pos],
                                  WATER_LOG_BUFFER_SIZE - pos);
        }
        if (pos > 0) {
            httpd_resp_send_chunk(req, &s_watering_log_buf[0], pos);
        }
    } else {
        // Not wrapped: just send [0 .. write_pos).
        httpd_resp_send_chunk(req, &s_watering_log_buf[0], s_watering_log_pos);
    }
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t zone_last_water_handler(httpd_req_t *req)
{
    char qs[32]={0}; httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char ids[8]={0}; httpd_query_key_value(qs, "id", ids, sizeof(ids));
    uint16_t lw_zone_id = ids[0] ? (uint16_t)atoi(ids) : s_web_zone_id;
    // b298: bump from 2048 -> 8192. The previous size truncated the
    // rings[] array at ~12 entries because each ring's JSON object is
    // ~150 chars and the guard `n < sizeof(buf) - 120` cut the loop
    // short for the 15-ring N1 and 23-ring N2 zones. 8KB comfortably
    // fits WATER_RUN_MAX_RINGS (36) rings: 36*170 + 250 header ~= 6.4KB.
    static char buf[8192];
    int n = 0;
    water_run_t _run = {0};
    if (storage_ready()) storage_water_load(lw_zone_id, &_run);
    const water_run_t *r = &_run;
    n += snprintf(buf+n, sizeof(buf)-n,
        "{\"fw_build\":%u,\"num_rings\":%u,"
        "\"arc_start\":%.1f,\"arc_span\":%.1f,"
        // b281: run-level supply pressure summary
        "\"supply_psi_min\":%.2f,\"supply_psi_max\":%.2f,\"supply_psi_avg\":%.2f,"
        // b282: gap analysis count
        "\"rings_supply_limited\":%u,"
        // b388: run target depth (N x 1/8") for heatmap normalization
        "\"target_depth_mm\":%.3f,"
        "\"rings\":[",
        r->fw_build, r->num_rings,
        r->arc_start_deg, r->arc_span_deg,
        r->supply_psi_min, r->supply_psi_max, r->supply_psi_avg,
        (unsigned)r->rings_supply_limited, r->target_depth_mm);
    for (int i = 0; i < (int)r->num_rings && i < WATER_RUN_MAX_RINGS
                     && n < (int)sizeof(buf)-120; i++) {
        const water_ring_data_t *rd = &r->rings[i];
        n += snprintf(buf+n, sizeof(buf)-n,
            "%s{\"throw_mm\":%.0f,\"avg_psi\":%.3f,"
            "\"dps\":%.2f,\"active_deg\":%.1f,\"actual_throw_mm\":%.0f,"
            "\"arc_s\":%.1f,\"arc_e\":%.1f,"
            // b281: per-ring valve angle + back-computed supply pressure
            "\"valve_deg\":%.2f,\"supply_psi_est\":%.2f}",
            i ? "," : "", rd->throw_mm, rd->avg_psi, rd->dps,
            rd->active_deg, rd->actual_throw_mm,
            rd->arc_start_deg, rd->arc_end_deg,
            rd->valve_deg, rd->supply_psi_est);
    }
    n += snprintf(buf+n, sizeof(buf)-n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    HTTP_CONN_CLOSE(req);
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static void zone_web_start(void)
{
    s_zone_mutex = xSemaphoreCreateMutex();
    zone_load_primary(0, &s_web_zone);  // load zone 0 as default at startup

    httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = ZONE_WEB_PORT;
    cfg.ctrl_port        = ZONE_WEB_CTRL_PORT;
    cfg.max_uri_handlers  = 64;  // MUST be set before httpd_start (cfg is copied there);
                                 // headroom over uris[] count -- _Static_assert below guards it.
                                 // (b378: was 40 and silently dropped every handler past #40,
                                 //  including /valve/probe and all /fs/* -- the post-start
                                 //  reassignment that "fixed" this was a no-op.)
    cfg.max_open_sockets  = 12;  // LwIP pool raised to 16; OTA+TCP terminal use ~4
    cfg.stack_size        = 8192;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 3;
    cfg.send_wait_timeout = 3;

    if (httpd_start(&s_zone_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Zone web server failed to start");
        return;
    }

    static const httpd_uri_t uris[] = {
        {.uri="/",           .method=HTTP_GET,  .handler=zone_root_handler},
        {.uri="/zone",       .method=HTTP_GET,  .handler=zone_page_handler},
        {.uri="/zone/state", .method=HTTP_GET,  .handler=zone_state_handler},
        {.uri="/zone/act",        .method=HTTP_POST, .handler=zone_act_handler},
        {.uri="/exp/lash",        .method=HTTP_POST, .handler=exp_lash_handler},   // EXP

        {.uri="/zone/last_water", .method=HTTP_GET,  .handler=zone_last_water_handler},
        {.uri="/zone/water_csv",   .method=HTTP_GET,  .handler=zone_water_csv_handler},
        {.uri="/zone/water_trace", .method=HTTP_GET,  .handler=zone_water_trace_handler}, // b283
        {.uri="/zone/last_log",    .method=HTTP_GET,  .handler=zone_last_log_handler},   // b292
        {.uri="/api/all",         .method=HTTP_GET,  .handler=api_all_handler},
        {.uri="/api/detail_log",  .method=HTTP_GET,  .handler=api_detail_log_handler},
        {.uri="/api/cal",         .method=HTTP_GET,  .handler=api_cal_handler},
        {.uri="/api/cal/clear",   .method=HTTP_POST, .handler=api_cal_clear_handler},
        {.uri="/api/status",      .method=HTTP_GET,  .handler=api_status_handler},
        {.uri="/api/boot_diag",   .method=HTTP_GET,  .handler=api_boot_diag_handler},  // b347
        {.uri="/api/theme",       .method=HTTP_GET,  .handler=api_theme_handler},
        {.uri="/api/theme",       .method=HTTP_POST, .handler=api_theme_handler},
        {.uri="/api/zones",       .method=HTTP_GET,  .handler=api_zones_handler},
        {.uri="/schedule",        .method=HTTP_GET,  .handler=schedule_page_handler},
        {.uri="/api/schedule",    .method=HTTP_GET,  .handler=api_schedule_handler},
        {.uri="/api/schedule",    .method=HTTP_POST, .handler=api_schedule_handler},
        {.uri="/api/schedule/clear", .method=HTTP_POST, .handler=api_schedule_clear_handler},
        {.uri="/api/schedule/delay", .method=HTTP_POST, .handler=api_schedule_delay_handler},
        {.uri="/api/zone",        .method=HTTP_GET,  .handler=api_zone_handler},
        {.uri="/api/zone/name",   .method=HTTP_POST, .handler=api_zone_name_handler},
        {.uri="/api/zone/delete", .method=HTTP_POST, .handler=api_zone_delete_handler},
        {.uri="/api/device/name", .method=HTTP_POST, .handler=api_device_name_handler},
        {.uri="/zone/water",      .method=HTTP_POST, .handler=zone_water_handler},
        {.uri="/cal",                   .method=HTTP_GET,  .handler=cal_page_handler},
        {.uri="/cal/status",            .method=HTTP_GET,  .handler=cal_status_handler},
        {.uri="/cal/pressure/start",    .method=HTTP_POST, .handler=cal_pressure_start_handler},
        {.uri="/cal/pressure/throw",    .method=HTTP_POST, .handler=cal_pressure_throw_handler},
        {.uri="/cal/pressure/throw_low",.method=HTTP_POST, .handler=cal_pressure_throw_low_handler},
        {.uri="/cal/pressure/cancel",   .method=HTTP_POST, .handler=cal_pressure_cancel_handler},
        {.uri="/cal/nozzle/start",      .method=HTTP_POST, .handler=cal_nozzle_start_handler},
        {.uri="/cal/jog/start",         .method=HTTP_POST, .handler=cal_jog_start_handler},
        {.uri="/cal/jog/cancel",        .method=HTTP_POST, .handler=cal_jog_cancel_handler},
        {.uri="/cal/encoder",           .method=HTTP_GET,  .handler=cal_encoder_handler},
        {.uri="/zone/water/cancel",     .method=HTTP_POST, .handler=zone_water_cancel_handler},
        {.uri="/valve/open",            .method=HTTP_POST, .handler=valve_open_handler},   // b370
        {.uri="/valve/close",           .method=HTTP_POST, .handler=valve_close_handler},  // b370
        {.uri="/valve/goto",            .method=HTTP_GET,  .handler=valve_goto_handler},   // b372
        {.uri="/valve/probe",           .method=HTTP_GET,  .handler=valve_probe_handler}, // b377
        {.uri="/wifi/ap_mode",          .method=HTTP_POST, .handler=wifi_ap_mode_handler},   // b396
        {.uri="/wifi/reconnect",        .method=HTTP_POST, .handler=wifi_reconnect_handler}, // b396
        {.uri="/zone/export",           .method=HTTP_GET,  .handler=zone_export_handler},    // b398
        {.uri="/zone/import",           .method=HTTP_POST, .handler=zone_import_handler},     // b398
        {.uri="/zone/trace",            .method=HTTP_POST, .handler=zone_trace_handler},      // b417
        {.uri="/cal/valve",             .method=HTTP_POST, .handler=cal_valve_handler},   // b389
        {.uri="/cal/valve/set",         .method=HTTP_POST, .handler=cal_valve_set_handler}, // b390
        {.uri="/fs",                    .method=HTTP_GET,  .handler=fs_page_handler},
        {.uri="/fs/list",               .method=HTTP_GET,  .handler=fs_list_handler},
        {.uri="/fs/download",           .method=HTTP_GET,  .handler=fs_download_handler},
        {.uri="/fs/upload",             .method=HTTP_POST, .handler=fs_upload_handler},
        {.uri="/fs/delete",             .method=HTTP_POST, .handler=fs_delete_handler},
    };
    _Static_assert(sizeof(uris)/sizeof(uris[0]) <= 64,
                   "uris[] exceeds cfg.max_uri_handlers -- raise it before httpd_start");
    for (size_t i = 0; i < sizeof(uris)/sizeof(uris[0]); i++)
        httpd_register_uri_handler(s_zone_server, &uris[i]);

    ESP_LOGI(TAG, "Zone web server on port %d", ZONE_WEB_PORT);
}


#ifndef ESPHOME_COMPONENT
void app_main(void)
{
    uart_setup();
    gpio_setup();
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Check battery voltage -- sleep forever if too low
    check_battery_on_boot();

    // Report wake cause if woken from sleep
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER || cause == ESP_SLEEP_WAKEUP_EXT0) {
        phase_sleep();
    }

    // Start blue LED blink task (flashes until WiFi connects)
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);

    // Mount LittleFS and load persisted last-watering data
    if (storage_init() == ESP_OK) {
        if (storage_water_load(0, &s_last_water_run) != ESP_OK) {
            // Migrate from NVS if FS file not yet written
            water_run_load_nvs(&s_last_water_run);
        }
    } else {
        water_run_load_nvs(&s_last_water_run);  // FS unavailable, use NVS
    }

    // Load device name before WiFi so hostname is set correctly
    device_name_init();

    // Connect to WiFi
    wifi_init();

    // Start TCP terminal server (after WiFi is up)
    s_tcp_mutex = xSemaphoreCreateMutex();
    if (s_wifi_connected) {
        xTaskCreate(tcp_server_task, "tcp_srv", 4096, NULL, 5, NULL);
        ota_http_start();
        zone_web_start();
    }

    // Redirect ESP_LOG output to both UART and TCP
    esp_log_set_vprintf(dual_vprintf);

    tprintf("\n\n");
    tprintf("========================================\n");
    tprintf("  OtO Hardware Bring-up  build %d\n", FW_BUILD);
    tprintf("  ESP-IDF %s\n", IDF_VER);
    tprintf("  WiFi: %s\n", s_wifi_connected ? "connected" : "NOT connected");
    if (s_wifi_connected) {
        tprintf("  TCP terminal: port %d\n", TCP_PORT);
        tprintf("  OTA push:     port %d  (POST /update)\n", OTA_HTTP_PORT);
    }
    tprintf("========================================\n");

    char buf[8];
    bool show_menu = true;
    s_last_activity = xTaskGetTickCount();  // start inactivity timer
    while (true) {
        if (show_menu) print_menu();
        show_menu = true;
        // When web watering runs, use short-timeout single-char reads
        // so pressing q alone (no Enter needed) triggers abort immediately.
        if (s_web_water_mode != 0) {
            buf[0] = 0;
            while (s_web_water_mode != 0) {
                char c = uart_getchar(200);  // 200ms poll
                if (c == 'q' || c == 'Q') {
                    water_set_status(WATER_STATUS_CANCELLED);
                    INFO("Watering abort via terminal.");
                    break;
                }
            }
            // Drain any extra bytes the user typed
            while (uart_getchar(0) != 0) {}
            show_menu = false;  // suppress menu reprint
            continue;
        }
        uart_readline(buf, sizeof(buf), 60000);

        // Update activity timestamp on any keypress
        if (buf[0] != 0) TOUCH_ACTIVITY();
        check_inactivity();

        // Preserve case for uppercase-only commands (B, G, R);
        // lowercase everything else so i/I, z/Z etc both work.
        char _raw = (buf[0] != 0) ? buf[0] : 0;
        char sel = (_raw == 'B' || _raw == 'G' || _raw == 'R')
                   ? _raw : (char)tolower((unsigned char)_raw);
        switch (sel) {
            case '1': phase_i2c_scan();     break;
            case '2': phase_valve_sensor(); break;
            case '3': phase_nozzle_sensor();break;
            case '4': phase_pressure();     break;
            case '5': phase_tca6408a();     break;
            case '6': phase_adc();          break;
            case '7': phase_valve_motor();  break;
            case '8': phase_nozzle_motor(); break;
            case '9': phase_kill();         break;
            case 't': phase_toggle_3v3sen(); break;
            case 'v': phase_toggle_9v();       break;
            case 'j': phase_jog_valve();     break;
            case 'b': phase_valve_jog_explore(); break;
            case 'd': phase_valve_hysteresis(); break;
            case 'h': phase_encoder_health(); break;
            case 'l': phase_led_test();      break;
            case 'p': phase_valve_goto();   break;
            case 'n': phase_nozzle_goto();  break;
            case 'x': phase_pressure_cal(); break;
            case '%': phase_valve_pct();    break;
            case 'g': phase_pressure_monitor(); break;
            case 'i': phase_water_zone();       break;
            case 'z': phase_zone_setup();   break;
            case 'k': phase_nozzle_speed_cal();  break;
            case 'G': phase_jog_pulse_cal();     break;
            case 'f': phase_nozzle_stall_find(); break;
            case 'e': phase_nozzle_speed_view(); break;
            case 'o': phase_zone_view();    break;
            case 'c': phase_cal_view();      break;
            case 's': phase_sleep();        break;
            case 'u':
                s_sleep_disabled = !s_sleep_disabled;
                tprintf("Auto-sleep %s\r\n",
                        s_sleep_disabled ? "DISABLED" : "enabled");
                break;
            case 'm': phase_monitor();      break;
            case 'w':
                INFO("WiFi: %s", s_wifi_connected ? "connected" : "NOT connected");
                break;
            case 'a': run_all();            break;
            case 'r':
                s_pass = 0; s_fail = 0;
                INFO("Counters reset");
                break;
            case 'R':
                INFO("Rebooting...");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
                break;
            case 0:
                show_menu = false;
                break; // timeout, don't redraw
            default:
                WARN("Unknown: '%c'", sel);
                break;
        }
    }
}
#endif /* !ESPHOME_COMPONENT */

// ─────────────────────────────────────────────────────────────────────────────
// ESPHome component API
// Compiled only when ESPHOME_COMPONENT is defined by the component CMakeLists.
// ─────────────────────────────────────────────────────────────────────────────
#ifdef ESPHOME_COMPONENT

// (irrigoto_api.h already included near the top of this file so callsites
//  above this block — e.g. check_inactivity() — see the prototypes.)

// Forward declarations for helpers defined further down in this file.
// (pm_record_reason is forward-declared globally near prepare_for_sleep.)
static void log_wake_cause(void);
static void pm_nvs_load(void);
static void schedule_load_nvs(void);
static void schedule_save_nvs(void);
static void schedule_delay_load_nvs(void);
static void last_water_load_nvs(void);
static void schedule_task(void *arg);

// Background task: maintains inactivity timer; no blocking uart loop needed
// because ESPHome owns the main loop (component loop() polls HA entities).
static void esphome_idle_task(void *arg)
{
    (void)arg;
    while (true) {
        check_inactivity();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void irrigoto_init(void)
{
    uart_setup();
    gpio_setup();
    // NOTE: nvs_flash_init() intentionally omitted — ESPHome calls it first.
    // NOTE: wifi_init() / esp_event_loop_create_default() omitted — ESPHome owns those.

    pm_nvs_load();        // restore auto_sleep + thresholds + last reason
    valve_offset_nvs_load();  // b389: per-unit valve frame, BEFORE boot valve-close
    check_battery_on_boot();
    log_wake_cause();

    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);

    if (storage_init() == ESP_OK) {
        if (storage_water_load(0, &s_last_water_run) != ESP_OK) {
            water_run_load_nvs(&s_last_water_run);
        }
    } else {
        water_run_load_nvs(&s_last_water_run);
    }

    device_name_init();

    // Register only the IP/disconnect events — ESPHome manages reconnection.
    s_wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &wifi_event_handler, NULL, NULL);

    // WiFi is already connected when irrigoto_init() is called from ESPHome setup().
    // Seed our state with the current IP.
    s_wifi_connected = true;
    esp_netif_t *_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (_netif) {
        esp_netif_ip_info_t _ip_info;
        if (esp_netif_get_ip_info(_netif, &_ip_info) == ESP_OK) {
            snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&_ip_info.ip));
        }
    }

    // Start web server. The standalone curl-push OTA endpoint is omitted in
    // ESPHome mode — ESPHome's native OTA replaces it, and the extra httpd
    // instance would consume 3 LWIP sockets we don't have to spare.
    s_tcp_mutex = xSemaphoreCreateMutex();
    zone_web_start();

    // NOTE: do NOT call esp_log_set_vprintf(dual_vprintf) in ESPHome mode.
    // ESPHome installs its own vprintf to forward IDF logs into its native
    // API logger (so `esphome logs`, HA log viewer, and the captive portal
    // all stream live logs). Overriding it here would silence those
    // channels — UART would still work but only with a physical cable.
    INFO("irrigoto ready (ESPHome component, build %d, ip %s)", FW_BUILD, s_wifi_ip);

    s_last_activity = xTaskGetTickCount();
    xTaskCreate(esphome_idle_task, "oto_idle", 4096, NULL, 3, NULL);
    // Schedule executor — separate task so a slow watering loop can't
    // delay the next-minute check. Low priority; just polls every 15 s.
    schedule_load_nvs();
    schedule_delay_load_nvs();   // restore rain/wind delay across reboot/wake
    last_water_load_nvs();       // restore "last completed" tag across deep-sleep wake
    xTaskCreate(schedule_task, "oto_sched", 3072, NULL, 2, NULL);
    INFO("Schedule loaded: %d entries", irrigoto_schedule_count());
}

// ── State getters ─────────────────────────────────────────────────────────────

float irrigoto_get_pressure_psi(void)
{
    // During active watering or calibration, motion control is reading the
    // MPRLS on its own schedule and the I2C bus mutex is hot. Don't compete:
    // serve the most recent cached value that mprls_read() already wrote.
    // When idle, take a fresh reading (which also updates the cache).
    bool busy = (s_web_water_mode != 0) ||
                (s_wcal.state != WCAL_IDLE && s_wcal.state != WCAL_DONE);
    if (!busy) {
        float psi = 0.0f;
        mprls_read_quiet(&psi);
    }
    return (s_cached_psi > 0.0f) ? s_cached_psi : 0.0f;
}

uint32_t irrigoto_get_battery_mv(void)
{
    return (uint32_t)(adc_mv(ADC_CH_VBATT) * VBATT_DIVIDER_RATIO);
}

float irrigoto_get_throw_mm(void)
{
    if (s_web_water && s_web_meas_throw_mm > 10.0f)
        return s_web_meas_throw_mm;
    if (s_web_valve_deg >= VALVE_CAL_START_DEG)
        return cal_valve_deg_to_throw_mm(s_web_valve_deg);
    return 0.0f;
}

bool irrigoto_is_watering(void)
{
    return (s_web_water_mode != 0);
}

// Estimated minutes remaining for the current watering run. 0 between
// runs. Exposed as an ESPHome sensor so HA dashboards can show a
// "time remaining" tile while watering.
//
// b362: smooth/gentle modes set s_water_est_min only at pass boundaries
// (every several minutes -- the per-pass adaptive estimate in
// phase_water_zone). Between those refreshes the value used to sit flat,
// so HA saw the same minutes-remaining for the entire pass. To make the
// HA tile decrease continuously we keep a (secs, tick) anchor that's
// refreshed alongside every s_water_est_min update inside phase_water_zone;
// this getter back-computes wall-clock decrement from the anchor. Floored
// at 1 min until the next pass-boundary refresh corrects the trajectory.
int irrigoto_get_water_minutes_remaining(void)
{
    if (s_eta_anchor_tick == 0 || s_web_water_mode == 0)
        return s_water_est_min;
    float elapsed_s = (float)(xTaskGetTickCount() - s_eta_anchor_tick)
                     / (float)configTICK_RATE_HZ;
    float remaining_s = s_eta_anchor_secs - elapsed_s;
    if (remaining_s < 60.0f) remaining_s = 60.0f;
    return (int)(remaining_s / 60.0f) + 1;
}

// b287: Watering Quiet mode -- true while a smooth/gentle run is in
// progress AND for ~3 sec after the post-run LFS writes complete. The
// ESPHome-side loop() consults this to throttle high-rate sensor
// publishes (pressure, battery, throw, cal_state) during the run,
// reducing WiFi/NVS-write load that has been racing with SPI flash
// access and crashing the device on long runs. Progress sensors
// (watering binary, status, zone name) continue to publish normally
// so HA still sees the run live.
bool irrigoto_is_watering_quiet(void)
{
    return s_watering_quiet;
}

int irrigoto_get_zone(void)
{
    return (int)s_water_zone_id + 1;  // 0-based → 1-based
}

int irrigoto_get_mode(void)
{
    // s_web_water_mode: 0=idle, 1-4=metered/pulse, 5-6=gentle, 7=smooth, 8=serpentine
    // Map to HA options: 0=Pulse, 1=Gentle, 2=Smooth, 3=Serpentine (b431)
    if (s_web_water_mode == 8)                     return 3;  // serpentine
    if (s_web_water_mode == 7)                     return 2;  // smooth
    if (s_web_water_mode == 5 || s_web_water_mode == 6) return 1; // gentle
    return 0;  // pulse / idle
}

void irrigoto_get_status(char *buf, size_t len)
{
    if (s_web_water_mode != 0) {
        const char *mname =
            (s_web_water_mode == 8)                    ? "serpentine"  :   // b423
            (s_web_water_mode == 7)                    ? "smooth" :
            (s_web_water_mode >= 5)                    ? "gentle" : "pulse";
        // b422: name-first -- the zone number is only a correlation key for
        // HA's static cards; the name is what the operator recognizes.
        char zn[32];
        irrigoto_zone_name_by_id((int)s_water_zone_id, zn, sizeof(zn));
        snprintf(buf, len, "Watering %s (%s)", zn, mname);
    } else {
        snprintf(buf, len, "Idle");
    }
}

// b422: resolve any zone id (0-based storage id) to its display name.
// Same precedence as everywhere else: LittleFS header -> NVS legacy slot ->
// "Zone #<id>" fallback. Display-only helper; ids on the wire unchanged.
void irrigoto_zone_name_by_id(int zone_id, char *buf, size_t len)
{
    char lfs_name[32] = {0};
    if (storage_ready()) {
        zone_perimeter_t _tmp;
        storage_zone_load((uint16_t)zone_id, lfs_name, sizeof(lfs_name), &_tmp);
    }
    char fallback[16];
    snprintf(fallback, sizeof(fallback), "Zone #%d", zone_id);
    zone_name_resolve((uint16_t)zone_id, lfs_name, fallback, buf, len);
}

void irrigoto_get_zone_name(char *buf, size_t len)
{
    // b420: self-heal a dangling selection. s_water_zone_id boots as 0 and
    // can be left pointing at a deleted zone; on units whose zone ids are
    // sparse (no zone 0 -- e.g. ba1f88 has 1/3/5) the resolved fallback
    // "Zone 1" is not a real zone, so the HA select rejects it on every
    // publish ("Invalid option"). When idle, snap to the first configured
    // zone instead. Never touched while watering.
    if (s_web_water_mode == 0 && storage_ready()) {
        uint16_t ids[STORAGE_MAX_ZONES]; int nc = 0;
        if (storage_zone_list(ids, STORAGE_MAX_ZONES, &nc) == ESP_OK && nc > 0) {
            bool exists = false;
            for (int i = 0; i < nc; i++)
                if (ids[i] == s_water_zone_id) { exists = true; break; }
            if (!exists) {
                ESP_LOGW(TAG, "Active zone id %u not configured -- snapping to %u",
                         (unsigned)s_water_zone_id, (unsigned)ids[0]);
                s_water_zone_id = ids[0];
            }
        }
    }
    // Web UI writes zone names to LittleFS; NVS is a migration fallback.
    // zone_name_resolve() handles the precedence (LittleFS -> NVS ->
    // default "Zone N"). Just reading NVS missed names set via the web UI.
    char lfs_name[32] = {0};
    if (storage_ready()) {
        zone_perimeter_t _tmp;
        storage_zone_load(s_water_zone_id, lfs_name, sizeof(lfs_name), &_tmp);
    }
    // b421: display ids are 0-based "#<id>" everywhere (HA schedule tab,
    // dashboard cards) -- fallback names follow suit. Wire encodings
    // (schedule entries 1-based, /zone/water 0-based) are untouched.
    char fallback[16];
    snprintf(fallback, sizeof(fallback), "Zone #%d", (int)s_water_zone_id);
    zone_name_resolve(s_water_zone_id, lfs_name, fallback, buf, len);
}

// ── Last-watering-run summary (consumed by the HA completion event) ──
// All values are read from s_last_water_run, snapshotted at the end of
// every watering pass. Units are metric and explicit in the field name.

int irrigoto_last_water_zone(void)         { return (int)s_last_water_zone_id + 1; }
int irrigoto_last_water_mode_code(void)    { return s_last_water_mode; }
int irrigoto_last_water_num_rings(void)    { return (int)s_last_water_run.num_rings; }
float irrigoto_last_water_duration_s(void) { return s_last_water_run.total_time_s; }
bool  irrigoto_last_water_aborted(void)    { return s_last_water_aborted; }
int   irrigoto_last_water_status_code(void){ return s_last_water_status_code; }

time_t irrigoto_last_water_finish_epoch(void)
{
    return s_last_water_finish_epoch;
}

void irrigoto_last_water_status_str(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    const char *s;
    switch (s_last_water_status_code) {
        case WATER_STATUS_COMPLETED:    s = "completed";    break;
        case WATER_STATUS_CANCELLED:    s = "cancelled";    break;
        case WATER_STATUS_VALVE_FAULT:  s = "valve_fault";  break;
        case WATER_STATUS_NOZZLE_FAULT: s = "nozzle_fault"; break;
        case WATER_STATUS_WATER_LOSS:   s = "water_loss";   break;
        default:                        s = "unknown";      break;
    }
    snprintf(buf, len, "%s", s);
}

void irrigoto_last_water_zone_name(char *buf, size_t len)
{
    if (buf == NULL || len == 0) return;
    char lfs_name[32] = {0};
    if (storage_ready()) {
        zone_perimeter_t _tmp;
        storage_zone_load(s_last_water_zone_id, lfs_name, sizeof(lfs_name), &_tmp);
    }
    char fallback[16];
    snprintf(fallback, sizeof(fallback), "Zone #%d", (int)s_last_water_zone_id);
    zone_name_resolve(s_last_water_zone_id, lfs_name, fallback, buf, len);
}

// Human-readable mode label for the event payload. Maps the internal
// s_web_water_mode (0=idle, 1-4=metered, 5-6=gentle, 7=smooth, ...).
void irrigoto_last_water_mode_label(char *buf, size_t len)
{
    if (buf == NULL || len == 0) return;
    const char *l;
    switch (s_last_water_mode) {
        case 1:  l = "Pulse 1/8 in";      break;
        case 2:  l = "Pulse 1/4 in";      break;
        case 3:  l = "Pulse 1/8 in x2";   break;
        case 4:  l = "Pulse 1/4 in x2";   break;
        case 5:  l = "Gentle 1/8 in";     break;
        case 6:  l = "Gentle 1/4 in";     break;
        case 7:  l = "Smooth 1/8 in";     break;
        case 8:  l = "Serpentine";             break;   // b431: depth via depth= param
        default: l = "unknown";           break;
    }
    snprintf(buf, len, "%s", l);
}

// Average pressure across rings that saw flow (avg_psi > 0).
float irrigoto_last_water_avg_psi(void)
{
    float sum = 0.0f; int n = 0;
    int nr = s_last_water_run.num_rings;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;
    for (int i = 0; i < nr; i++) {
        float p = s_last_water_run.rings[i].avg_psi;
        if (p > 0.0f) { sum += p; n++; }
    }
    return n ? sum / (float)n : 0.0f;
}

// Maximum throw distance observed across rings (mm, 0 if nothing reached).
float irrigoto_last_water_max_throw_mm(void)
{
    float mx = 0.0f;
    int nr = s_last_water_run.num_rings;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;
    for (int i = 0; i < nr; i++) {
        float t = s_last_water_run.rings[i].throw_mm;
        if (t > mx) mx = t;
    }
    return mx;
}

// Sum of estimated depth across rings (mm).
float irrigoto_last_water_total_depth_mm(void)
{
    float sum = 0.0f;
    int nr = s_last_water_run.num_rings;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;
    for (int i = 0; i < nr; i++) sum += s_last_water_run.rings[i].depth_mm;
    return sum;
}

// Area covered (m^2). Each ring is approximated as a thin annular sector
// of width WATER_RING_SPACING centered on the ring's throw distance.
// A_ring_mm2 = (active_deg * pi/180) * throw_mm * spacing_mm
// Summed across rings that actually ran (depth > 0 or active arc > 0).
float irrigoto_last_water_area_m2(void)
{
    float sum_mm2 = 0.0f;
    int nr = s_last_water_run.num_rings;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;
    for (int i = 0; i < nr; i++) {
        const water_ring_data_t *r = &s_last_water_run.rings[i];
        if (r->active_deg <= 0.0f || r->throw_mm <= 0.0f) continue;
        float arc_rad = r->active_deg * (float)M_PI / 180.0f;
        sum_mm2 += arc_rad * r->throw_mm * (float)WATER_RING_SPACING;
    }
    return sum_mm2 / 1.0e6f;  // mm^2 -> m^2
}

// Estimated water volume (liters). For each ring, volume = depth * area
// (where 1 mm * 1 m^2 = 1 L). Sum across rings.
float irrigoto_last_water_volume_l(void)
{
    float sum_L = 0.0f;
    int nr = s_last_water_run.num_rings;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;
    for (int i = 0; i < nr; i++) {
        const water_ring_data_t *r = &s_last_water_run.rings[i];
        if (r->active_deg <= 0.0f || r->throw_mm <= 0.0f) continue;
        float arc_rad = r->active_deg * (float)M_PI / 180.0f;
        float area_m2 = (arc_rad * r->throw_mm * (float)WATER_RING_SPACING)
                        / 1.0e6f;
        sum_L += r->depth_mm * area_m2;  // depth(mm) * area(m^2) = volume(L)
    }
    return sum_L;
}

// Target depth requested for the last run (3.175mm for 1/8", 6.35mm for 1/4").
// Set at start of phase_water_zone(); preserved across runs so HA / dashboards
// can show "Z3 1/8" attempted -> got 26%" for partial deliveries.
float irrigoto_last_water_target_depth_mm(void) { return s_last_depth_mm; }

// Average depth actually delivered across the zone polygon (mm).
//   volume_l / zone_area_m2  (1 L on 1 m² == 1 mm depth).
// This is the honest "did we hit target?" number -- compare against
// irrigoto_last_water_target_depth_mm(). 0 if zone polygon area is unknown
// (zone not loaded, < 3 points, etc.).
float irrigoto_last_water_actual_avg_depth_mm(void)
{
    float zone_area = irrigoto_last_water_zone_area_m2();
    if (zone_area <= 0.01f) return 0.0f;
    return irrigoto_last_water_volume_l() / zone_area;
}

// Sort zone perimeter points by walk_idx and convert to Cartesian. Used
// by both polygon area and the heatmap-based score, neither of which run
// often enough to justify caching the result.
typedef struct {
    bool   valid;
    int    n;
    float  x[ZONE_MAX_PERIM_POINTS];
    float  y[ZONE_MAX_PERIM_POINTS];
} zone_cart_t;

static void load_last_water_zone_cart(zone_cart_t *out)
{
    out->valid = false; out->n = 0;
    if (!storage_ready()) return;
    zone_perimeter_t zp;
    char _name[32];
    if (storage_zone_load(s_last_water_zone_id, _name, sizeof(_name), &zp) != ESP_OK)
        return;
    if (zp.num_points < 3) return;
    int n = zp.num_points;
    int order[ZONE_MAX_PERIM_POINTS];
    for (int i = 0; i < n; i++) order[i] = i;
    // Insertion sort by walk_idx (n <= 36, trivial cost).
    for (int i = 1; i < n; i++) {
        int t = order[i]; int j = i;
        while (j > 0 && zp.points[order[j-1]].walk_idx > zp.points[t].walk_idx) {
            order[j] = order[j-1]; j--;
        }
        order[j] = t;
    }
    for (int i = 0; i < n; i++) {
        const perimeter_point_t *p = &zp.points[order[i]];
        float a = p->nozzle_deg * (float)M_PI / 180.0f;
        out->x[i] = p->throw_mm * sinf(a);
        out->y[i] = p->throw_mm * cosf(a);
    }
    out->n = n; out->valid = true;
}

// Zone perimeter polygon area (m^2) via the shoelace formula.
float irrigoto_last_water_zone_area_m2(void)
{
    zone_cart_t zc; load_last_water_zone_cart(&zc);
    if (!zc.valid) return 0.0f;
    float sum = 0.0f;
    for (int i = 0, j = zc.n - 1; i < zc.n; j = i++) {
        sum += (zc.x[j] + zc.x[i]) * (zc.y[j] - zc.y[i]);
    }
    return fabsf(sum) * 0.5f / 1.0e6f;  // mm^2 -> m^2
}

// True if the ring's actual footprint contains the cell (bearing, r_mm).
// Ring footprint: bearing within [arc_start, arc_end] (handles wrap) and
// radial distance within +- WATER_RING_SPACING/2 of the ring's effective
// throw (actual if measured, else target).
static bool ring_covers(const water_ring_data_t *r, float bearing_deg, float r_mm)
{
    if (r->active_deg <= 0.0f || r->throw_mm <= 0.0f) return false;
    if (r->depth_mm   <= 0.01f)                       return false;
    bool in_arc;
    if (r->arc_start_deg <= r->arc_end_deg) {
        in_arc = (bearing_deg >= r->arc_start_deg && bearing_deg <= r->arc_end_deg);
    } else {
        in_arc = (bearing_deg >= r->arc_start_deg || bearing_deg <= r->arc_end_deg);
    }
    if (!in_arc) return false;
    // b433: trust the PSI-derived actual throw only within +/-25% of target
    // (the heatmap card's plausibility rule). Below ~1.5 PSI the pressure->
    // throw model over-reads up to 2x, which displaced inner-ring footprints
    // outward and deflated the serpentine run's coverage from a fair 87% to
    // 71.5% (the watered center rastered as "missed"). Genuine under-throw
    // (e.g. supply-limited outer rings at 0.86x) stays measured.
    float ref = r->throw_mm;
    if (r->actual_throw_mm > 100.0f && r->throw_mm > 100.0f) {
        float ratio = r->actual_throw_mm / r->throw_mm;
        if (ratio >= 0.75f && ratio <= 1.25f) ref = r->actual_throw_mm;
    }
    float half = (float)WATER_RING_SPACING * 0.5f;
    return (r_mm >= ref - half && r_mm <= ref + half);
}

// Rasterize the zone polygon (2 deg x 200 mm polar grid, ~7600 cells) and
// classify each cell against the last run's rings:
//   covered    = inside zone polygon AND reached by some ring's sweep
//   missed     = inside zone polygon AND NOT reached
//   overspray  = outside zone polygon AND reached
// Returns false (and zeroes the outputs) if the zone polygon can't be loaded
// or the last run had no rings. Used by both irrigoto_last_water_score()
// (F1 × depth_factor) and irrigoto_last_water_polygon_coverage_pct() (recall
// only) so the cell-counting logic stays in one place.
static bool last_water_count_cells(int *out_covered, int *out_missed,
                                   int *out_overspray)
{
    if (out_covered)   *out_covered   = 0;
    if (out_missed)    *out_missed    = 0;
    if (out_overspray) *out_overspray = 0;

    zone_cart_t zc; load_last_water_zone_cart(&zc);
    if (!zc.valid) return false;
    int nr = s_last_water_run.num_rings;
    if (nr <= 0) return false;
    if (nr > WATER_RUN_MAX_RINGS) nr = WATER_RUN_MAX_RINGS;

    const float BEARING_STEP = 2.0f;     // degrees
    const float RADIAL_STEP  = 200.0f;   // mm
    // Reach cap for the raster. Was a hardcoded 8534mm (28ft), which silently
    // truncated the covered/missed/overspray counts for any zone -- or supply
    // pressure -- reaching past 28ft, skewing the coverage % and score that are
    // published to HA. Derive it from what we're actually scoring: the polygon's
    // own extent, the calibrated reach, and the run's actual throw, + a margin
    // so overspray just beyond the boundary is still classified.
    float RADIAL_MAX = cal_get_max_throw_mm();
    float run_max = irrigoto_last_water_max_throw_mm();
    if (run_max > RADIAL_MAX) RADIAL_MAX = run_max;
    for (int i = 0; i < zc.n; i++) {
        float pr = sqrtf(zc.x[i]*zc.x[i] + zc.y[i]*zc.y[i]);
        if (pr > RADIAL_MAX) RADIAL_MAX = pr;
    }
    RADIAL_MAX += 2.0f * RADIAL_STEP;    // margin to catch overspray past the edge
    int covered = 0, missed = 0, overspray = 0;

    for (float bearing = 0.0f; bearing < 360.0f; bearing += BEARING_STEP) {
        float br = bearing * (float)M_PI / 180.0f;
        float sb = sinf(br), cb = cosf(br);
        for (float r_mm = RADIAL_STEP * 0.5f; r_mm < RADIAL_MAX; r_mm += RADIAL_STEP) {
            float x = r_mm * sb, y = r_mm * cb;
            // Ray-cast point-in-polygon (horizontal ray from (x,y)).
            bool inside = false;
            for (int i = 0, j = zc.n - 1; i < zc.n; j = i++) {
                if ((zc.y[i] > y) != (zc.y[j] > y) &&
                    (x < (zc.x[j] - zc.x[i]) * (y - zc.y[i])
                          / (zc.y[j] - zc.y[i]) + zc.x[i])) {
                    inside = !inside;
                }
            }
            bool actual = false;
            for (int k = 0; k < nr && !actual; k++) {
                actual = ring_covers(&s_last_water_run.rings[k], bearing, r_mm);
            }
            if (inside && actual) covered++;
            else if (inside)      missed++;
            else if (actual)      overspray++;
        }
    }
    if (out_covered)   *out_covered   = covered;
    if (out_missed)    *out_missed    = missed;
    if (out_overspray) *out_overspray = overspray;
    return true;
}

// Watering score, 0.0..5.0. From the rasterized polygon cell counts:
// Coverage  = covered / (covered + missed)        [recall]
// Precision = covered / (covered + overspray)
// F1        = 2 * coverage * precision / (coverage + precision)
//
// Depth factor (new in b276): min(1, actual_avg_depth / target_depth_mm).
// F1 alone says "did we hit every cell"; the depth factor says "with how much
// water". A run that sprayed the right places but delivered 26% of target
// depth (gentle-mode under-delivery bug) used to score full marks for spatial
// coverage -- now it drops to ~0.26 × F1 × 5, signaling that the run was
// thin. Over-delivery (>1.0) doesn't earn bonus points; the factor caps at 1.
//
// score = 5 * F1 * depth_factor
//
// 5.0 = every intended cell got water and at least target depth, no overspray.
// Drops linearly as coverage shrinks, overspray grows, OR depth falls short.
// Returns 0 if the zone can't be loaded (no storage, polygon < 3 points).
float irrigoto_last_water_score(void)
{
    int covered = 0, missed = 0, overspray = 0;
    if (!last_water_count_cells(&covered, &missed, &overspray)) return 0.0f;
    int intended = covered + missed;
    int actual   = covered + overspray;
    if (intended == 0) return 0.0f;
    float recall    = (float)covered / (float)intended;
    float precision = actual ? (float)covered / (float)actual : 0.0f;
    float f1 = (recall + precision > 0.0f)
               ? 2.0f * recall * precision / (recall + precision)
               : 0.0f;
    // Depth factor: ratio of delivered depth to target, capped at 1.0.
    // s_last_depth_mm holds the target the run aimed for (3.175 / 6.35); if
    // that's <=0 (very old data, demo mode) skip the depth penalty.
    float depth_factor = 1.0f;
    if (s_last_depth_mm > 0.01f) {
        float actual_depth = irrigoto_last_water_actual_avg_depth_mm();
        depth_factor = fminf(1.0f, actual_depth / s_last_depth_mm);
        if (depth_factor < 0.0f) depth_factor = 0.0f;
    }
    return 5.0f * f1 * depth_factor;
}

// Honest "what fraction of the zone got water" — recall side of the score
// computation, exposed on its own so HA can render a gauge that ONLY counts
// water landing inside the polygon. Returns 0..100.
//
// Contrast with sensor.irrigoto_last_watering_coverage_ratio (HA-side,
// area_m2 / zone_area_m2 capped at 100%): that one is an upper bound based
// on the total sprayed footprint, including over-spray past the polygon.
// This number is the truth.
float irrigoto_last_water_polygon_coverage_pct(void)
{
    int covered = 0, missed = 0, overspray = 0;
    if (!last_water_count_cells(&covered, &missed, &overspray)) return 0.0f;
    int intended = covered + missed;
    if (intended == 0) return 0.0f;
    return 100.0f * (float)covered / (float)intended;
}

// b281: Last-run supply pressure summary accessors. Computed at end of
// each watering run from per-ring avg_psi + zone cal f(valve_deg).
// Return 0.0 if the cal couldn't classify (insufficient cal points).
float irrigoto_last_water_supply_psi_min(void)
{
    return s_last_water_run.supply_psi_min;
}
float irrigoto_last_water_supply_psi_max(void)
{
    return s_last_water_run.supply_psi_max;
}
float irrigoto_last_water_supply_psi_avg(void)
{
    return s_last_water_run.supply_psi_avg;
}
// b282: Count of rings flagged as supply-limited after the last run.
int irrigoto_last_water_rings_supply_limited(void)
{
    return (int)s_last_water_run.rings_supply_limited;
}

// Look up the Nth configured zone (0-based idx). Returns the zone's 1-based
// number and fills name_buf with its configured name (falls back to
// "Zone N" if no name has been set anywhere). Returns -1 if idx is out of
// range or storage isn't ready. Used by the HA select entity to surface
// actual zone names instead of generic "Zone 1..4" labels.
int irrigoto_zone_at(int idx, char *name_buf, size_t name_len)
{
    if (idx < 0 || name_buf == NULL || name_len == 0) return -1;
    if (!storage_ready())                              return -1;
    uint16_t ids[STORAGE_MAX_ZONES]; int nc = 0;
    if (storage_zone_list(ids, STORAGE_MAX_ZONES, &nc) != ESP_OK) return -1;
    if (idx >= nc) return -1;
    uint16_t zid = ids[idx];  // 0-based zone id
    // Load the LittleFS-stored name, then resolve via the canonical helper
    // so we honor (in order): LittleFS header, NVS legacy slot, default
    // "Zone N". The web UI's api_zones_handler uses the same pattern.
    char lfs_name[32] = {0};
    zone_perimeter_t _tmp;
    storage_zone_load(zid, lfs_name, sizeof(lfs_name), &_tmp);
    char fallback[16];
    snprintf(fallback, sizeof(fallback), "Zone #%u", (unsigned)zid);
    zone_name_resolve(zid, lfs_name, fallback, name_buf, name_len);
    return (int)(zid + 1);
}

// ── Command setters ───────────────────────────────────────────────────────────

// Internal entry point that takes the granular web_mode value (1-7)
// directly. Used by both irrigoto_start_watering (HA-facing 0/1/2 mode)
// and the scheduler (which encodes mode+depth into a web_mode).
//   web_mode = style digit: 1=Pulse, 5=Gentle, 7=Smooth, 99=demo.
//   depth8   = depth target in eighths of an inch (1..8); 0 = legacy default.
static void start_watering_web_mode(int zone, int web_mode, int depth8)
{
    if (s_web_water_mode != 0) {
        ESP_LOGW(TAG, "start_watering: already running");
        return;
    }
    if (s_exp_running) {
        ESP_LOGW(TAG, "start_watering: lash experiment running -- refusing");
        return;
    }
    if (zone < 1) zone = 1;
    s_water_abort        = false;
    s_water_status_code  = WATER_STATUS_COMPLETED;
    s_water_run_had_flow = false;
    s_water_no_flow_streak = 0;
    s_water_est_min  = (web_mode == 99) ? 2 : 13;
    s_water_zone_id  = (uint16_t)(zone - 1);
    s_web_water_depth_eighths = (depth8 >= 1 && depth8 <= 8) ? depth8 : 0;
    s_web_water_mode = web_mode;
    // b294: moved to PRO_CPU (core 0). See water_web above for rationale.
    // b285: bumped from 8192 -> 16384, same reason as water_web above.
    xTaskCreatePinnedToCore(water_task, "water_ha", 16384, NULL, 10, NULL, PRO_CPU_NUM);
}

// Translate the schedule entry's mode into the style web_mode digit. Depth is
// now carried separately as eighths (see start_watering_web_mode), so this no
// longer encodes depth.
static int schedule_web_mode(uint8_t mode)
{
    switch (mode) {
        case 0:  return 1;  // Pulse
        case 1:  return 5;  // Gentle
        case 3:  return 8;  // Serpentine (b431)
        default: return 7;  // Smooth
    }
}

void irrigoto_start_watering(int zone, int mode, int duration_s)
{
    (void)duration_s;  // TODO: duration override — not wired yet
    // Legacy HA-facing entrypoint: default to 1/8" (1 eighth).
    start_watering_web_mode(zone, schedule_web_mode((uint8_t)mode), 1);
}

void irrigoto_stop_watering(void)
{
    water_set_status(WATER_STATUS_CANCELLED);
}

void irrigoto_set_zone(int zone)
{
    if (zone >= 1)
        s_water_zone_id = (uint16_t)(zone - 1);
}

void irrigoto_set_mode(int mode)
{
    // No-op while watering; the next call to irrigoto_start_watering passes mode.
    (void)mode;
}

// ── Watering schedule ────────────────────────────────────────────────────────
// Device-owned schedule. Any HA scheduler (built-in, Schedy, Scheduler Card,
// AppDaemon, custom) pushes the table here via irrigoto_schedule_set_text();
// the device runs entries autonomously based on the local clock. Storage is
// NVS blob "schedule" under the OtO namespace.
//
// Text format (one entry per line, ';' or '\n' separator):
//   zone,mode,depth,hour,minute,days_mask,enabled
// e.g. "2,2,0,06,30,127,1" = zone 2, smooth, 1/8", 6:30 AM, all days, enabled
//
// days_mask: bit0=Sun, bit1=Mon, ..., bit6=Sat (matches struct tm tm_wday)
// mode:      0=Pulse, 1=Gentle, 2=Smooth, 3=Serpentine (b431)
// depth:     0=1/8" (3.175 mm), 1=1/4" (6.35 mm).  Smooth always uses 1/8".
//
// Time source: the system clock is set by ESPHome's homeassistant time
// platform — no NTP, no internet. Until time is valid (epoch < 1.7e9) the
// scheduler does nothing.

static schedule_t s_schedule = { .count = 0 };
// Per-entry "last fire minute" (unix-epoch minute) so a single timestamp
// matching multiple poll iterations only fires once. RAM-only; on reboot
// any entry whose hour:minute hasn't changed in the current minute would
// re-fire, which is fine — that scenario is rare and harmless.
static uint32_t s_sched_last_fire_min[SCHEDULE_MAX_ENTRIES] = {0};
// Outcome of the most recent set_schedule call, surfaced to HA via
// text_sensor so you can see at a glance whether a push succeeded or
// why it was rejected.
static char s_sched_last_status[160] = "no schedule pushed yet";

// b355: device-wide monotonic schedule_version. Bumped on every mutation
// (set_text, sync_text, clear; NOT delay changes — those don't touch
// the entry list). HA tracks last_seen_schedule_version per device and
// pulls on mismatch; auto-synced on API reconnect via a text_sensor.
static uint32_t s_schedule_version = 0;
// b355: next sequential ID to assign to a newly-created entry. Persisted
// across reboots so IDs stay unique even if a freshly-allocated entry
// races with a power-cycle. Starts at 1 — 0 is reserved as the wire-
// protocol sentinel for "device, please allocate one."
static uint32_t s_schedule_id_next = 1;

// Rain / wind delay. Absolute epoch — schedule firing is suppressed
// until now >= s_sched_delay_until. 0 means no delay active. Persisted
// to NVS so it survives reboot / deep-sleep wake (otherwise a wake
// during a delay window would happily fire the entry we asked to skip).
static time_t s_sched_delay_until = 0;

static void schedule_delay_load_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    int64_t v = 0;
    if (nvs_get_i64(h, "sched_dly", &v) == ESP_OK) {
        s_sched_delay_until = (time_t)v;
    }
    nvs_close(h);
}

static void schedule_delay_save_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i64(h, "sched_dly", (int64_t)s_sched_delay_until);
    nvs_commit(h);
    nvs_close(h);
}

void irrigoto_schedule_set_delay_until(time_t t)
{
    time_t now = time(NULL);
    // Past / zero clears the delay. Future sets it.
    if (t <= now) {
        if (s_sched_delay_until != 0) {
            INFO("Schedule delay cleared");
            s_sched_delay_until = 0;
            schedule_delay_save_nvs();
            snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                     "delay cleared");
        }
        return;
    }
    s_sched_delay_until = t;
    schedule_delay_save_nvs();
    // Log + status using local time so the user sees what they set.
    struct tm lt; localtime_r(&t, &lt);
    INFO("Schedule delay set: suspended until %04d-%02d-%02d %02d:%02d",
         lt.tm_year+1900, lt.tm_mon+1, lt.tm_mday, lt.tm_hour, lt.tm_min);
    snprintf(s_sched_last_status, sizeof(s_sched_last_status),
             "delay until %04d-%02d-%02d %02d:%02d",
             lt.tm_year+1900, lt.tm_mon+1, lt.tm_mday,
             lt.tm_hour, lt.tm_min);
}

void irrigoto_schedule_set_delay_hours(uint32_t hours)
{
    if (hours == 0) {
        irrigoto_schedule_set_delay_until(0);
        return;
    }
    time_t now = time(NULL);
    if (now < 1700000000) {
        ESP_LOGW(TAG, "Schedule delay requested (%u h) but time not synced — ignoring",
                 (unsigned)hours);
        snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                 "delay rejected: time not synced");
        return;
    }
    // Clamp to a sane upper bound — anything beyond a couple weeks is
    // probably a unit mix-up and would also confuse the next_run walk
    // (which only looks 7 days out).
    if (hours > 14u * 24u) hours = 14u * 24u;
    irrigoto_schedule_set_delay_until(now + (time_t)hours * 3600);
}

void irrigoto_schedule_clear_delay(void)
{
    irrigoto_schedule_set_delay_until(0);
}

time_t irrigoto_schedule_get_delay_until(void)
{
    // Self-clear expired delays on read so callers always see 0 once
    // the window has passed. Don't bother re-saving NVS for the expiry
    // — next set/clear will write the right value, and a stale future
    // timestamp in NVS is harmless (next read also self-clears).
    if (s_sched_delay_until != 0) {
        time_t now = time(NULL);
        if (now > 1700000000 && now >= s_sched_delay_until) {
            s_sched_delay_until = 0;
        }
    }
    return s_sched_delay_until;
}

// Snapshot of the live schedule for the web UI. Plain copy under no lock
// — the scheduler only reads s_schedule from one task and writes it via
// irrigoto_schedule_set_text / _clear (atomic struct assignment). A torn
// read here just means the editor briefly displays an in-flight value;
// it doesn't crash and self-heals on the next poll.
static void schedule_snapshot(schedule_t *out)
{
    if (out) *out = s_schedule;
}

// b355: load the schedule, version counter, and next-id counter.
//
// Two compatibility paths:
//   - new schema blob (sizeof(schedule_t) == 1 + 32*16 + padding) — load directly.
//   - old schema blob (sizeof(schedule_v1_t) == 1 + 32*7 = 225 bytes) — load
//     as old struct, then promote each entry into the new schema with a
//     freshly-allocated ID, last_modified = time(NULL) or 0 if clock not
//     yet set, source = 0 (unknown). Schedule version starts at 1.
//
// Corrupt blob (wrong size, count over max) → schedule wiped, version=1.
// We intentionally don't try to recover individual entries from a torn
// blob — see open question #2 in docs/schedule_sync_design.md.
static void schedule_load_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;

    // First check the blob size to decide which schema to read.
    size_t blob_sz = 0;
    esp_err_t sr = nvs_get_blob(h, "schedule", NULL, &blob_sz);
    // b403: schema-version key. CRITICAL — the tagged entry (3×u32 + 8×u8 = 20B,
    // schedule_t = 644B) is the SAME SIZE as the pre-tag entry (2×u32 + 9×u8 incl.
    // _pad = 20B, also 644B). Size alone CANNOT tell them apart, so loading an old
    // blob directly would overlay client_tag onto the old source/zone/mode/depth
    // bytes — silent corruption. Only treat the blob as tagged when this key says
    // so; pre-b403 firmware never wrote it (schema stays 0 -> v2 migration below).
    uint32_t schema = 0;
    nvs_get_u32(h, "sched_schema", &schema);
    bool migrated = false;   // set by a v1/v2 migration -> resave (w/ schema) below
    if (sr == ESP_OK && schema >= 3 && blob_sz == sizeof(s_schedule)) {
        // New (tagged) schema — load straight into the live struct.
        size_t got = blob_sz;
        sr = nvs_get_blob(h, "schedule", &s_schedule, &got);
        if (sr != ESP_OK || got != sizeof(s_schedule) ||
                s_schedule.count > SCHEDULE_MAX_ENTRIES) {
            ESP_LOGW(TAG, "Schedule NVS read failed or corrupt (got %u, expected %u, count %u) -- wiping",
                     (unsigned)got, (unsigned)sizeof(s_schedule), s_schedule.count);
            memset(&s_schedule, 0, sizeof(s_schedule));
        }
    } else if (sr == ESP_OK && blob_sz == sizeof(schedule_v1_t)) {
        // Legacy schema — migrate in place.
        schedule_v1_t old; memset(&old, 0, sizeof(old));
        size_t got = blob_sz;
        sr = nvs_get_blob(h, "schedule", &old, &got);
        memset(&s_schedule, 0, sizeof(s_schedule));
        if (sr == ESP_OK && got == sizeof(old) && old.count <= SCHEDULE_MAX_ENTRIES) {
            time_t now = time(NULL);
            uint32_t lm = (now > 1700000000) ? (uint32_t)now : 0;
            s_schedule.count = old.count;
            uint32_t next_id = 1;
            for (uint8_t i = 0; i < old.count; i++) {
                schedule_entry_t *e = &s_schedule.entries[i];
                e->id            = next_id++;
                e->last_modified = lm;
                e->source        = 0;  // unknown
                e->zone          = old.entries[i].zone;
                e->mode          = old.entries[i].mode;
                e->depth         = old.entries[i].depth;
                e->hour          = old.entries[i].hour;
                e->minute        = old.entries[i].minute;
                e->days_mask     = old.entries[i].days_mask;
                e->enabled       = old.entries[i].enabled;
            }
            // s_schedule_id_next will be loaded below if the NVS key exists;
            // otherwise this seed value is used.
            s_schedule_id_next = next_id;
            migrated = true;
            INFO("Schedule NVS migrated from v1 schema: %u entries (lm=%u, "
                 "id_next=%u)", s_schedule.count, lm, s_schedule_id_next);
        } else {
            ESP_LOGW(TAG, "Legacy schedule read failed (count=%u) -- wiping",
                     old.count);
        }
    } else if (sr == ESP_OK && blob_sz == sizeof(schedule_v2_t)) {
        // b403 migration: pre-tag 16-byte entries -> tagged schema. Copy every
        // field; client_tag starts at 0 (HA re-stamps a tag on the next edit/
        // push; until then id/tuple dedup still applies). No data loss — without
        // this branch the size mismatch below would WIPE the stored schedule on
        // the upgrade boot.
        schedule_v2_t old; memset(&old, 0, sizeof(old));
        size_t got = blob_sz;
        sr = nvs_get_blob(h, "schedule", &old, &got);
        memset(&s_schedule, 0, sizeof(s_schedule));
        if (sr == ESP_OK && got == sizeof(old) && old.count <= SCHEDULE_MAX_ENTRIES) {
            s_schedule.count = old.count;
            for (uint8_t i = 0; i < old.count; i++) {
                schedule_entry_t *e = &s_schedule.entries[i];
                const schedule_entry_v2_t *o = &old.entries[i];
                e->id            = o->id;
                e->last_modified = o->last_modified;
                e->client_tag    = 0;
                e->source        = o->source;
                e->zone          = o->zone;
                e->mode          = o->mode;
                e->depth         = o->depth;
                e->hour          = o->hour;
                e->minute        = o->minute;
                e->days_mask     = o->days_mask;
                e->enabled       = o->enabled;
            }
            migrated = true;
            INFO("Schedule NVS migrated from v2 (pre-tag) schema: %u entries",
                 s_schedule.count);
        } else {
            ESP_LOGW(TAG, "v2 schedule read failed (count=%u) -- wiping", old.count);
        }
    } else if (sr == ESP_OK) {
        ESP_LOGW(TAG, "Schedule NVS blob has unexpected size %u -- wiping",
                 (unsigned)blob_sz);
        memset(&s_schedule, 0, sizeof(s_schedule));
    }
    // sr != ESP_OK and not ESP_ERR_NVS_NOT_FOUND would be unusual; either
    // way s_schedule stays zero-initialized.

    // Restore version + next-id counters. Defaults: version=1 if any
    // entries are present (so HA sees a "real" version on first contact),
    // 0 if empty. id_next at least max(entry.id)+1 so we never reuse.
    uint32_t v = 0;
    if (nvs_get_u32(h, "sched_ver", &v) == ESP_OK) {
        s_schedule_version = v;
    } else {
        s_schedule_version = (s_schedule.count > 0) ? 1 : 0;
    }
    v = 0;
    if (nvs_get_u32(h, "sched_idnxt", &v) == ESP_OK && v > 0) {
        s_schedule_id_next = v;
    }
    // Defensive: id_next must be > every entry's id.
    for (uint8_t i = 0; i < s_schedule.count; i++) {
        if (s_schedule.entries[i].id >= s_schedule_id_next) {
            s_schedule_id_next = s_schedule.entries[i].id + 1;
        }
    }
    if (s_schedule_id_next == 0) s_schedule_id_next = 1;

    // Depth-semantics migration (one-time, version-gated). The `depth` field
    // changed meaning from a 0/1 flag (1/8" / 1/4") to a count of eighths of an
    // inch (1..8). Remap legacy values once — 1/4"(1) -> 2 eighths, 1/8"(0) -> 1
    // — gated by "sched_depthv" so it never double-applies (re-applying would
    // double the watering depth). Persist the remapped blob + the version key
    // in one commit so a torn write simply retries on the next boot.
    uint32_t depthv = 0;
    nvs_get_u32(h, "sched_depthv", &depthv);
    nvs_close(h);
    if (depthv < 1) {
        for (uint8_t i = 0; i < s_schedule.count; i++) {
            schedule_entry_t *e = &s_schedule.entries[i];
            e->depth = (e->depth >= 1) ? 2 : 1;
        }
        nvs_handle_t wh;
        if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &wh) == ESP_OK) {
            nvs_set_blob(wh, "schedule", &s_schedule, sizeof(s_schedule));
            nvs_set_u32 (wh, "sched_depthv", 1);
            nvs_commit(wh);
            nvs_close(wh);
            INFO("Schedule depth migrated to eighths (%u entries)", s_schedule.count);
        }
    }
    // b403: if we migrated a v1/v2 blob, persist the tagged blob WITH the schema
    // key now (schedule_save_nvs stamps sched_schema=3). Otherwise the next boot
    // would re-run the migration, and — worse — a depth-migration resave above
    // could leave a tagged-layout blob on disk with schema still 0, which the
    // next load would misread as pre-tag. Persisting here closes both gaps.
    if (migrated) schedule_save_nvs();
}

// Persist schedule blob + version + id_next together so the three stay
// consistent across crashes. Called from every mutation path.
static void schedule_save_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "schedule", &s_schedule, sizeof(s_schedule));
    nvs_set_u32 (h, "sched_ver",   s_schedule_version);
    nvs_set_u32 (h, "sched_idnxt", s_schedule_id_next);
    // b403: stamp the schema version IN THE SAME COMMIT as the blob so the two
    // stay consistent. The tagged layout is byte-size-identical to the pre-tag
    // one (both 644B), so schedule_load_nvs() relies on this key — not the blob
    // size — to know the blob carries client_tag. (3 = tagged schema.)
    nvs_set_u32 (h, "sched_schema", 3);
    nvs_commit(h);
    nvs_close(h);
}

uint32_t irrigoto_schedule_version(void)
{
    return s_schedule_version;
}

// Estimate how long a scheduled entry will take, in minutes. Used by
// validation to detect overlapping entries. Prefers last-run wall-clock
// time from storage (the most accurate signal); falls back to mode-
// and depth-based conservative upper bounds when no history exists yet.
//
// Note: storage_water_load returns the last run regardless of which
// mode/depth was used, so the estimate inherits whatever depth produced
// it. Acceptable approximation for overlap-detection purposes.
static int schedule_estimate_duration_min(uint8_t zone, uint8_t mode, uint8_t depth)
{
    if (zone < 1) return 30;
    if (storage_ready()) {
        water_run_t prev; memset(&prev, 0, sizeof(prev));
        if (storage_water_load(zone - 1, &prev) == ESP_OK
                && prev.total_time_s > 60.0f) {
            return (int)((prev.total_time_s * 1.10f + 59.0f) / 60.0f);
        }
    }
    // Scale with the depth target (now eighths of an inch, 1..8): the per-eighth
    // base is the 1/8" time for each style, so deeper runs scale proportionally.
    // Rough by design — once a real run completes the measured time above wins.
    int d = (depth >= 1 && depth <= 8) ? depth : 1;
    int base;
    switch (mode) {
        case 2:  base = 30; break;  // smooth (adaptive)
        case 1:  base = 15; break;  // gentle, per 1/8"
        default: base = 8;  break;  // pulse, per 1/8"
    }
    return base * d;
}

// Validate that no two enabled entries overlap anywhere in the week,
// including across midnight and the Sat→Sun week-wrap. Each entry is
// projected onto a 0..10080 minute-of-week axis once per day it fires
// (per its days_mask); all (i_dow, j_dow) combinations are pairwise
// tested for time-window intersection given the entries' estimated
// durations. Fills err_buf with a description of the first overlap
// found (if any). Returns true if the schedule is safe to install.
static bool schedule_validate(const schedule_t *s, char *err_buf, size_t err_len)
{
    if (err_buf && err_len) err_buf[0] = '\0';
    if (!s || s->count < 2) return true;

    // Precompute per-entry duration estimates *before* the pair loop.
    // schedule_estimate_duration_min calls storage_water_load, which does
    // an opendir+readdir+fopen+fclose on littlefs. Doing that inside the
    // inner loop was O(n²) -- at SCHEDULE_MAX_ENTRIES=32 worst case it's
    // ~500 fs operations on the httpd task. Each fclose closes through
    // vfs_littlefs_close → settimeofday → __tzcalc_limits, which acquires
    // a newlib lock; under contention the Interrupt WDT trips CPU0 spinning
    // in xPortEnterCriticalTimeout (observed crash on Build 273). Hoisting
    // here keeps the load to exactly s->count storage_water_load calls.
    int dur[SCHEDULE_MAX_ENTRIES] = {0};
    for (uint8_t i = 0; i < s->count && i < SCHEDULE_MAX_ENTRIES; i++) {
        const schedule_entry_t *e = &s->entries[i];
        if (!e->enabled) continue;
        dur[i] = schedule_estimate_duration_min(e->zone, e->mode, e->depth);
    }

    // Project each (entry, day-of-week) firing onto a minute-of-week
    // axis [0..10080) and test overlap across all (dow_a, dow_b) day
    // combos for every entry pair. The wrap tests handle a Sat tail
    // spilling into Sun-of-next-week. The 7×7 inner is trivial — at
    // n=32 pairs that's ~25k integer compares, all on-stack.
    static const char *DOW_NAME[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    for (uint8_t i = 0; i < s->count; i++) {
        const schedule_entry_t *a = &s->entries[i];
        if (!a->enabled) continue;
        int a_sm = a->hour * 60 + a->minute;
        for (uint8_t j = (uint8_t)(i + 1); j < s->count; j++) {
            const schedule_entry_t *b = &s->entries[j];
            if (!b->enabled) continue;
            int b_sm = b->hour * 60 + b->minute;
            for (uint8_t da = 0; da < 7; da++) {
                if (!(a->days_mask & (1u << da))) continue;
                int as = da * 1440 + a_sm;
                int ae = as + dur[i];
                for (uint8_t db = 0; db < 7; db++) {
                    if (!(b->days_mask & (1u << db))) continue;
                    int bs = db * 1440 + b_sm;
                    int be = bs + dur[j];
                    // Standard interval overlap, plus week-end wrap:
                    // if A or B spills past minute 10080, also test
                    // the wrapped tail against the other range.
                    bool ov = (as < be && bs < ae);
                    if (!ov && ae > 10080)
                        ov = (0 < be && bs < (ae - 10080));
                    if (!ov && be > 10080)
                        ov = (as < (be - 10080) && 0 < ae);
                    if (ov) {
                        if (err_buf && err_len) {
                            // b421: zone shown as 0-based "#id" (entries'
                            // wire value is 1-based; display-only shift)
                            snprintf(err_buf, err_len,
                                "entry %u (zone #%u, %s %02u:%02u, est %d min) overlaps "
                                "entry %u (zone #%u, %s %02u:%02u, est %d min)",
                                (unsigned)i, (unsigned)(a->zone - 1), DOW_NAME[da],
                                a->hour, a->minute, dur[i],
                                (unsigned)j, (unsigned)(b->zone - 1), DOW_NAME[db],
                                b->hour, b->minute, dur[j]);
                        }
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

// Helper: try to find an existing entry with matching (zone,hour,minute,
// days_mask) so a legacy REPLACE-ALL push reuses the device's existing
// ID for that "slot" rather than churning a new ID every push. Returns
// the index in s_schedule.entries[] or -1.
//
// Rationale: HA's legacy pusher emits the full schedule on every change,
// without IDs. If we minted a fresh ID for every entry on every push the
// effective IDs would tumble each time, defeating the sync-by-ID design
// even for clients that haven't migrated to sync_text yet. Matching on
// the "intent" tuple keeps IDs stable across legacy pushes that didn't
// actually move the entry — small white lie that smooths the migration.
static int legacy_find_existing(uint8_t zone, uint8_t hour, uint8_t minute,
                                uint8_t days_mask)
{
    for (uint8_t i = 0; i < s_schedule.count; i++) {
        const schedule_entry_t *e = &s_schedule.entries[i];
        if (e->zone == zone && e->hour == hour && e->minute == minute &&
                e->days_mask == days_mask) {
            return (int)i;
        }
    }
    return -1;
}

bool irrigoto_schedule_set_text(const char *text)
{
    if (text == NULL) return false;
    schedule_t next = { .count = 0 };
    // Parse one entry at a time. Each entry: 7 comma-separated ints,
    // separated from other entries by ';' or '\n'.
    const char *p = text;
    while (*p && next.count < SCHEDULE_MAX_ENTRIES) {
        // Skip whitespace and separators between entries
        while (*p == ' ' || *p == '\t' || *p == ';' || *p == '\n' || *p == '\r') p++;
        if (!*p) break;
        int z, m, d, hh, mm, days, en;
        int n = sscanf(p, "%d,%d,%d,%d,%d,%d,%d", &z, &m, &d, &hh, &mm, &days, &en);
        if (n != 7) {
            snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                "parse failed at \"%.32s\" (expected 7 fields)", p);
            ESP_LOGW(TAG, "%s", s_sched_last_status);
            return false;
        }
        if (z < 1 || z > 250 || m < 0 || m > 2 || d < 0 || d > 8 ||
            hh < 0 || hh > 23 || mm < 0 || mm > 59 ||
            days < 0 || days > 127 || en < 0 || en > 1) {
            snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                "out-of-range z=%d m=%d d=%d %02d:%02d days=%d en=%d",
                z, m, d, hh, mm, days, en);
            ESP_LOGW(TAG, "%s", s_sched_last_status);
            return false;
        }
        next.entries[next.count++] = (schedule_entry_t){
            .id = 0,            // assigned below
            .last_modified = 0,
            .source = 0,
            .zone = (uint8_t)z, .mode = (uint8_t)m, .depth = (uint8_t)d,
            .hour = (uint8_t)hh, .minute = (uint8_t)mm,
            .days_mask = (uint8_t)days, .enabled = (uint8_t)en,
        };
        // Skip past this entry's tokens to the next separator
        while (*p && *p != ';' && *p != '\n') p++;
    }
    // Validate before installing — overlapping entries on shared days
    // would get the second one silently skipped at fire time, which is
    // a UX surprise. Reject the whole batch and keep the prior schedule.
    char verr[160] = {0};
    if (!schedule_validate(&next, verr, sizeof(verr))) {
        snprintf(s_sched_last_status, sizeof(s_sched_last_status),
            "rejected: %s", verr);
        ESP_LOGW(TAG, "Schedule rejected: %s", verr);
        return false;
    }
    // Assign IDs + timestamps. For each new entry, try to match the
    // existing schedule by (zone,hour,minute,days_mask) so unchanged
    // entries keep their ID; rewrite fields and bump last_modified only
    // if anything actually differs. Entries with no match get a fresh ID.
    time_t now = time(NULL);
    uint32_t lm_default = (now > 1700000000) ? (uint32_t)now : 0;
    for (uint8_t i = 0; i < next.count; i++) {
        schedule_entry_t *ne = &next.entries[i];
        int idx = legacy_find_existing(ne->zone, ne->hour, ne->minute,
                                       ne->days_mask);
        if (idx >= 0) {
            const schedule_entry_t *old = &s_schedule.entries[idx];
            bool changed = (old->mode != ne->mode) ||
                           (old->depth != ne->depth) ||
                           (old->enabled != ne->enabled);
            ne->id            = old->id ? old->id : s_schedule_id_next++;
            ne->source        = changed ? 0 : old->source;
            ne->last_modified = changed ? lm_default : old->last_modified;
        } else {
            ne->id            = s_schedule_id_next++;
            ne->last_modified = lm_default;
            ne->source        = 0;
        }
    }
    // Atomic swap + persist
    s_schedule = next;
    s_schedule_version++;
    memset(s_sched_last_fire_min, 0, sizeof(s_sched_last_fire_min));
    schedule_save_nvs();
    snprintf(s_sched_last_status, sizeof(s_sched_last_status),
        "ok (%u entries)", (unsigned)s_schedule.count);
    INFO("Schedule updated: %u entries (v=%u, persisted)",
         (unsigned)s_schedule.count, s_schedule_version);
    return true;
}

// b355: bidirectional sync entry-point. See docs/schedule_sync_design.md.
// Format (one entry per ';' or newline):
//   id,last_modified,zone,mode,depth,hour,minute,days_mask,enabled,source,tombstone
//
// Each incoming entry is applied independently to the current schedule:
//   - tombstone=1 with matching id → delete
//   - id=0                          → allocate new ID, insert
//   - id matches existing           → LWW: incoming wins if its
//                                     last_modified is strictly greater
//                                     than the device's; tie goes to
//                                     device. source updates with it.
//   - id !=0, no existing match     → insert with the supplied ID (treat
//                                     as "HA assigned this on its side")
//
// Entries in the device that HA doesn't mention are left alone. The whole
// payload is validated (no overlaps among the resulting set) before any
// persistence; if validation fails, the device rejects the merge and
// keeps the prior schedule. schedule_version increments once if any
// change actually landed.
bool irrigoto_schedule_sync_text(const char *text)
{
    if (text == NULL) return false;
    // Work on a copy so we can roll back cleanly if validation fails.
    schedule_t next = s_schedule;
    uint32_t   id_next_after = s_schedule_id_next;
    int        applied = 0;
    time_t     now = time(NULL);
    uint32_t   now_u32 = (now > 1700000000) ? (uint32_t)now : 0;

    const char *p = text;
    while (*p && next.count < SCHEDULE_MAX_ENTRIES) {
        while (*p == ' ' || *p == '\t' || *p == ';' || *p == '\n' || *p == '\r') p++;
        if (!*p) break;
        unsigned long id_u, lm_u, tag_u = 0;
        int z, m, d, hh, mm, days, en, src, tomb;
        // 11 required fields + an OPTIONAL 12th = client_tag (b403). Use %lu for
        // the unsigned-32 fields to keep parsing portable on the device's 32-bit
        // long. Accept both 11 (legacy/tagless pusher) and 12 fields.
        int parsed = sscanf(p, "%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu",
                            &id_u, &lm_u, &z, &m, &d, &hh, &mm,
                            &days, &en, &src, &tomb, &tag_u);
        if (parsed != 11 && parsed != 12) {
            snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                "sync parse failed at \"%.32s\" (expected 11-12 fields)", p);
            ESP_LOGW(TAG, "%s", s_sched_last_status);
            return false;
        }
        uint32_t client_tag = (parsed >= 12) ? (uint32_t)tag_u : 0;
        // Range-check non-id fields. id and last_modified are uint32 by
        // construction. tombstone=1 entries with id=0 are dropped (nonsensical).
        if (tomb != 0 && tomb != 1) {
            snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                "sync bad tombstone=%d", tomb);
            ESP_LOGW(TAG, "%s", s_sched_last_status);
            return false;
        }
        if (tomb == 0) {
            if (z < 1 || z > 250 || m < 0 || m > 2 || d < 0 || d > 8 ||
                hh < 0 || hh > 23 || mm < 0 || mm > 59 ||
                days < 0 || days > 127 || en < 0 || en > 1 ||
                src < 0 || src > 3) {
                snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                    "sync out-of-range id=%lu z=%d m=%d d=%d %02d:%02d days=%d en=%d src=%d",
                    id_u, z, m, d, hh, mm, days, en, src);
                ESP_LOGW(TAG, "%s", s_sched_last_status);
                return false;
            }
        }
        uint32_t id = (uint32_t)id_u;
        uint32_t lm = (uint32_t)lm_u;
        // Advance past this entry's tokens
        while (*p && *p != ';' && *p != '\n') p++;

        // Find an existing entry by ID (only meaningful if id != 0).
        int idx = -1;
        if (id != 0) {
            for (uint8_t i = 0; i < next.count; i++) {
                if (next.entries[i].id == id) { idx = (int)i; break; }
            }
        }

        if (tomb) {
            // Delete the matching entry (no-op if not found).
            if (idx >= 0) {
                for (uint8_t k = (uint8_t)idx; k + 1 < next.count; k++) {
                    next.entries[k] = next.entries[k + 1];
                }
                next.count--;
                applied++;
            }
            continue;
        }

        // b403: tag-dedup — the robust dedup key. A stable client_tag (assigned
        // by HA when the entry is created, PRESERVED across edits) survives a
        // change to the time/zone/days, unlike the (zone,hour,minute,days) tuple
        // below. So re-pushing an EDITED id=0 entry maps back to the same record
        // and UPDATES it, instead of allocating a duplicate that overlaps the
        // original and rejects the whole sync. Checked before the tuple fallback.
        if (idx < 0 && client_tag != 0) {
            for (uint8_t i = 0; i < next.count; i++) {
                if (next.entries[i].client_tag == client_tag) {
                    ESP_LOGI(TAG, "sync_schedule: tag-dedup, incoming id=%lu tag=%lu mapped to existing id=%lu",
                             (unsigned long)id, (unsigned long)client_tag,
                             (unsigned long)next.entries[i].id);
                    idx = (int)i;
                    break;
                }
            }
        }

        // b359: tuple-dedup fallback for non-tombstone inserts. If the
        // incoming entry didn't match an existing id (either id=0 from a
        // newly-created HA slot, or HA's stored id is stale because b357
        // removed text_full from the event payload and HA never learned
        // the assigned id), try to match by (zone, hour, minute,
        // days_mask). If a tuple match exists, upsert THAT entry instead
        // of allocating a new id — which would create a duplicate that
        // schedule_validate rejects, blocking the whole sync.
        //
        // The user-visible effect: HA can keep pushing id=0 indefinitely
        // for the same zone/time/days tuple; the device transparently
        // reuses the existing id. No rejection chain.
        if (idx < 0) {
            for (uint8_t i = 0; i < next.count; i++) {
                if (next.entries[i].zone      == (uint8_t)z  &&
                    next.entries[i].hour      == (uint8_t)hh &&
                    next.entries[i].minute    == (uint8_t)mm &&
                    next.entries[i].days_mask == (uint8_t)days) {
                    ESP_LOGI(TAG, "sync_schedule: tuple-dedup, incoming id=%lu mapped to existing id=%lu (zone=%u %02u:%02u days=%u)",
                             (unsigned long)id,
                             (unsigned long)next.entries[i].id,
                             (uint8_t)z, (uint8_t)hh, (uint8_t)mm, (uint8_t)days);
                    idx = (int)i;
                    break;
                }
            }
        }

        if (idx < 0) {
            // Insert. id=0 → allocate; non-zero → respect HA's assignment.
            if (next.count >= SCHEDULE_MAX_ENTRIES) {
                snprintf(s_sched_last_status, sizeof(s_sched_last_status),
                    "sync rejected: SCHEDULE_MAX_ENTRIES exceeded");
                ESP_LOGW(TAG, "%s", s_sched_last_status);
                return false;
            }
            uint32_t new_id = (id == 0) ? id_next_after++ : id;
            // If HA pushed an id outside our known range, bump id_next so
            // we don't accidentally collide with it later.
            if (new_id >= id_next_after) id_next_after = new_id + 1;
            schedule_entry_t e = {
                .id = new_id,
                .last_modified = lm ? lm : now_u32,
                .client_tag = client_tag,
                .source = (uint8_t)src,
                .zone = (uint8_t)z, .mode = (uint8_t)m, .depth = (uint8_t)d,
                .hour = (uint8_t)hh, .minute = (uint8_t)mm,
                .days_mask = (uint8_t)days, .enabled = (uint8_t)en,
            };
            next.entries[next.count++] = e;
            applied++;
            continue;
        }

        // Upsert by ID: last-writer-wins, device wins on tie.
        schedule_entry_t *cur = &next.entries[idx];
        if (lm > cur->last_modified) {
            cur->zone      = (uint8_t)z;
            cur->mode      = (uint8_t)m;
            cur->depth     = (uint8_t)d;
            cur->hour      = (uint8_t)hh;
            cur->minute    = (uint8_t)mm;
            cur->days_mask = (uint8_t)days;
            cur->enabled   = (uint8_t)en;
            cur->source    = (uint8_t)src;
            cur->last_modified = lm;
            // b403: adopt a (nonzero) incoming tag so an entry created on the
            // device web UI, or matched by id/tuple, learns its HA tag once HA
            // starts sending one. Never clobber an existing tag with 0.
            if (client_tag) cur->client_tag = client_tag;
            applied++;
        }
        // else: device's last_modified >= incoming → keep device.
    }

    // Validate the merged result. Reject the whole sync if it produces
    // an overlapping schedule (same rule as set_text). Prior schedule
    // is preserved on rejection.
    char verr[160] = {0};
    if (!schedule_validate(&next, verr, sizeof(verr))) {
        snprintf(s_sched_last_status, sizeof(s_sched_last_status),
            "sync rejected: %s", verr);
        ESP_LOGW(TAG, "Schedule sync rejected: %s", verr);
        return false;
    }

    if (applied == 0) {
        // No-op sync — leave version untouched so HA doesn't think it
        // missed a change. Update status so the user can tell the push
        // landed cleanly with nothing to do.
        snprintf(s_sched_last_status, sizeof(s_sched_last_status),
            "sync no-op (%u entries)", (unsigned)next.count);
        return true;
    }

    s_schedule = next;
    s_schedule_id_next = id_next_after;
    s_schedule_version++;
    memset(s_sched_last_fire_min, 0, sizeof(s_sched_last_fire_min));
    schedule_save_nvs();
    snprintf(s_sched_last_status, sizeof(s_sched_last_status),
        "sync ok (%u entries, %d applied)",
        (unsigned)s_schedule.count, applied);
    INFO("Schedule sync: applied=%d count=%u v=%u",
         applied, (unsigned)s_schedule.count, s_schedule_version);
    return true;
}

void irrigoto_schedule_get_last_status(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    snprintf(buf, len, "%s", s_sched_last_status);
}

void irrigoto_schedule_get_text(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    size_t off = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < s_schedule.count && off < len; i++) {
        const schedule_entry_t *e = &s_schedule.entries[i];
        int n = snprintf(buf + off, len - off, "%s%u,%u,%u,%u,%u,%u,%u",
                         (i == 0) ? "" : ";",
                         e->zone, e->mode, e->depth,
                         e->hour, e->minute, e->days_mask, e->enabled);
        if (n < 0 || (size_t)n >= len - off) break;
        off += (size_t)n;
    }
}

// b355: structured serialization including id + last_modified + source,
// used by the schedule_text text_sensor and the schedule_changed event
// so HA can reconcile by ID. Format per entry:
//   id,last_modified,zone,mode,depth,hour,minute,days_mask,enabled,source
// Entries separated by ';'. Empty schedule → empty string.
void irrigoto_schedule_get_text_full(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    size_t off = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < s_schedule.count && off < len; i++) {
        const schedule_entry_t *e = &s_schedule.entries[i];
        int n = snprintf(buf + off, len - off,
                         "%s%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u",
                         (i == 0) ? "" : ";",
                         (unsigned long)e->id,
                         (unsigned long)e->last_modified,
                         e->zone, e->mode, e->depth,
                         e->hour, e->minute, e->days_mask, e->enabled,
                         e->source);
        if (n < 0 || (size_t)n >= len - off) break;
        off += (size_t)n;
    }
}

void irrigoto_schedule_clear(void)
{
    bool had_entries = (s_schedule.count > 0);
    s_schedule.count = 0;
    memset(s_sched_last_fire_min, 0, sizeof(s_sched_last_fire_min));
    if (had_entries) s_schedule_version++;
    schedule_save_nvs();
    INFO("Schedule cleared (v=%u)", s_schedule_version);
}

int irrigoto_schedule_count(void)
{
    return (int)s_schedule.count;
}

// Compute the next firing time across the whole table. Returns true and
// fills *out_t (unix epoch) plus *out_zone if any entry will fire within
// the next 7 days. Returns false if schedule is empty / all disabled.
bool irrigoto_schedule_next_run(time_t now, time_t *out_t, int *out_zone)
{
    if (s_schedule.count == 0 || now < 1700000000) return false;
    // Honor an active delay — entries that would fire during the
    // suspension window are skipped so callers (next_run sensor,
    // sleep-shortening logic) all see the same effective "next fire".
    // Self-clears on expiry via the getter.
    time_t delay_until = irrigoto_schedule_get_delay_until();
    time_t earliest = (delay_until > now) ? delay_until : now;
    time_t best_t = 0;
    int    best_zone = 0;
    for (uint8_t i = 0; i < s_schedule.count; i++) {
        const schedule_entry_t *e = &s_schedule.entries[i];
        if (!e->enabled || e->days_mask == 0) continue;
        // Walk forward up to 14 days — needs to outrun the max delay
        // window (also 14d) so an entry isn't missed when a long delay
        // hides everything in the first week.
        for (int d = 0; d < 15; d++) {
            time_t t = earliest + d * 86400;
            struct tm lt;
            localtime_r(&t, &lt);
            lt.tm_hour = e->hour; lt.tm_min = e->minute; lt.tm_sec = 0;
            time_t fire_t = mktime(&lt);
            if (fire_t <= earliest) continue;  // already past or in delay
            // Re-check day-of-week of fire_t (DST shifts can reorder)
            struct tm flt; localtime_r(&fire_t, &flt);
            if (!(e->days_mask & (1u << flt.tm_wday))) continue;
            if (best_t == 0 || fire_t < best_t) {
                best_t = fire_t;
                best_zone = e->zone;
            }
            break;  // earliest day-of-week match wins for this entry
        }
    }
    if (best_t == 0) return false;
    if (out_t) *out_t = best_t;
    if (out_zone) *out_zone = best_zone;
    return true;
}

// Background task: poll the schedule once every SCHED_POLL_MS and fire any
// entry whose hour:minute matches the current local time, hasn't already
// been fired in this minute, and isn't already running. Runs forever.
#define SCHED_POLL_MS  5000u   // 5 s — tight enough to reliably catch minute boundary
#define SCHED_IMMINENT_S 120u  // schedule firing within this many s -> block sleep

static void schedule_task(void *arg)
{
    (void)arg;
    // Check immediately on task start (i.e. on every wake from deep
    // sleep, since irrigoto_init() respawns this task) so a wake
    // timed to fire a scheduled run doesn't waste 15 s waiting for
    // the first poll. After the first iteration, normal SCHED_POLL_MS
    // cadence resumes via the delay at the bottom of the loop.
    while (true) {
        time_t now = time(NULL);
        if (now < 1700000000) goto sleep_poll;  // time not synced yet
        // Rain / wind delay active — suppress all firing until expiry.
        // Cheap check; the getter self-clears expired delays.
        time_t delay_until = irrigoto_schedule_get_delay_until();
        if (delay_until > now) goto sleep_poll;
        struct tm lt; localtime_r(&now, &lt);
        uint32_t now_min = (uint32_t)(now / 60);
        for (uint8_t i = 0; i < s_schedule.count; i++) {
            const schedule_entry_t *e = &s_schedule.entries[i];
            if (!e->enabled) continue;
            if (lt.tm_hour != e->hour || lt.tm_min != e->minute) continue;
            if (!(e->days_mask & (1u << lt.tm_wday))) continue;
            if (s_sched_last_fire_min[i] == now_min) continue;  // already fired
            if (s_web_water_mode != 0) {
                ESP_LOGW(TAG, "Schedule: entry %u due (zone %u) but watering already "
                              "active — skipping this slot", i, e->zone);
                s_sched_last_fire_min[i] = now_min;  // don't retry this minute
                continue;
            }
            int wm = schedule_web_mode(e->mode);
            INFO("Schedule fires: entry %u -> zone %u mode %u depth %u/8\" "
                 "(web_mode %d) at %02u:%02u",
                 i, e->zone, e->mode, e->depth, wm,
                 e->hour, e->minute);
            s_sched_last_fire_min[i] = now_min;
            start_watering_web_mode(e->zone, wm, e->depth);
        }
sleep_poll:
        vTaskDelay(pdMS_TO_TICKS(SCHED_POLL_MS));
    }
}

// ── Power management ─────────────────────────────────────────────────────────

// Log boot wake cause without prompting (UART input doesn't work in ESPHome
// mode — replaces the standalone phase_sleep() interactive path that would
// hang on uart_getchar for non-timer wakes).
static void log_wake_cause(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_TIMER:     INFO("Woke from RTC timer"); break;
        case ESP_SLEEP_WAKEUP_EXT0:      INFO("Woke from EXT0 (Hall sensor)"); break;
        case ESP_SLEEP_WAKEUP_UNDEFINED: /* normal cold boot */ break;
        default:                         INFO("Woke from cause %d", (int)cause); break;
    }
}

// ── Power-management persistence (NVS namespace "OtO") ─────────────────────
// Keys: pm_disable (u8), pm_inact_s (u32), pm_dur_s (u32), pm_reason (str)

static void pm_nvs_load(void)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t  dis = 0;
    uint32_t v   = 0;
    if (nvs_get_u8 (h, "pm_disable",  &dis) == ESP_OK) s_sleep_disabled = (dis != 0);
    if (nvs_get_u32(h, "pm_inact_s",  &v)   == ESP_OK && v >= 30 && v <= 3600)
        s_inactivity_ms = v * 1000u;
    if (nvs_get_u32(h, "pm_dur_s",    &v)   == ESP_OK && v >= 30 && v <= 3600)
        s_sleep_dur_s = v;
    if (nvs_get_u32(h, "pm_dwell_s",  &v)   == ESP_OK && v >= 10 && v <= 300)
        s_dwell_timeout_ms = v * 1000u;
    size_t sz = sizeof(s_last_sleep_reason);
    if (nvs_get_str(h, "pm_reason", s_last_sleep_reason, &sz) != ESP_OK)
        s_last_sleep_reason[0] = '\0';
    uint8_t th = 1;
    if (nvs_get_u8(h, "ui_theme", &th) == ESP_OK) s_theme_dark = (th != 0);
    nvs_close(h);
}

bool irrigoto_get_theme_dark(void)
{
    return s_theme_dark;
}

void irrigoto_set_theme_dark(bool dark)
{
    s_theme_dark = dark;
    pm_nvs_save_u8("ui_theme", dark ? 1 : 0);
    INFO("Web UI theme set to %s (persisted)", dark ? "dark" : "light");
}

// Persist a single u8/u32/string key. Failures only logged, not propagated —
// settings just won't survive next reboot if NVS is full or hosed.
static void pm_nvs_save_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, key, v); nvs_commit(h); nvs_close(h);
}
static void pm_nvs_save_u32(const char *key, uint32_t v)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, key, v); nvs_commit(h); nvs_close(h);
}
static void pm_nvs_save_str(const char *key, const char *v)
{
    nvs_handle_t h;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, key, v); nvs_commit(h); nvs_close(h);
}

// Record a sleep reason (RAM + NVS) so HA can display "why did it sleep?"
// after the next wake. Called from every sleep entry point.
static void pm_record_reason(const char *reason)
{
    if (reason == NULL) reason = "unknown";
    strncpy(s_last_sleep_reason, reason, sizeof(s_last_sleep_reason) - 1);
    s_last_sleep_reason[sizeof(s_last_sleep_reason) - 1] = '\0';
    pm_nvs_save_str("pm_reason", s_last_sleep_reason);
}

void irrigoto_set_auto_sleep_enabled(bool enabled)
{
    s_sleep_disabled = !enabled;
    pm_nvs_save_u8("pm_disable", s_sleep_disabled ? 1 : 0);
    // Toggling the switch counts as user activity — reset the inactivity
    // timer so enabling auto-sleep after a long idle period doesn't
    // trip sleep immediately on already-accumulated idle time.
    TOUCH_ACTIVITY();
    INFO("Auto sleep %s (persisted, inactivity timer reset)",
         enabled ? "ENABLED" : "DISABLED");
}

bool irrigoto_get_auto_sleep_enabled(void)
{
    return !s_sleep_disabled;
}

uint32_t irrigoto_get_inactivity_minutes(void)
{
    return s_inactivity_ms / 60000u;
}

void irrigoto_set_inactivity_minutes(uint32_t minutes)
{
    if (minutes < 1u)  minutes = 1u;
    if (minutes > 60u) minutes = 60u;
    s_inactivity_ms = minutes * 60u * 1000u;
    pm_nvs_save_u32("pm_inact_s", minutes * 60u);
    // Reset the inactivity timer so a lowered threshold doesn't trip
    // immediately on already-accumulated idle time. User just touched
    // a setting -- give them the new window from now.
    TOUCH_ACTIVITY();
    INFO("Inactivity threshold set to %u min (persisted, timer reset)",
         (unsigned)minutes);
}

uint32_t irrigoto_get_sleep_duration_s(void)
{
    return s_sleep_dur_s;
}

void irrigoto_set_sleep_duration_s(uint32_t seconds)
{
    if (seconds < 30u)   seconds = 30u;
    if (seconds > 3600u) seconds = 3600u;
    s_sleep_dur_s = seconds;
    pm_nvs_save_u32("pm_dur_s", seconds);
    // Same intent as the inactivity setter: any PM-knob touch is
    // user activity, give them a fresh inactivity window.
    TOUCH_ACTIVITY();
    INFO("Sleep duration set to %u s (persisted, inactivity timer reset)",
         (unsigned)seconds);
}

uint32_t irrigoto_get_dwell_timeout_s(void)
{
    return s_dwell_timeout_ms / 1000u;
}

void irrigoto_set_dwell_timeout_s(uint32_t seconds)
{
    if (seconds < 10u)  seconds = 10u;
    if (seconds > 300u) seconds = 300u;
    s_dwell_timeout_ms = seconds * 1000u;
    pm_nvs_save_u32("pm_dwell_s", seconds);
    TOUCH_ACTIVITY();  // user-initiated change counts as activity
    INFO("Nozzle dwell watchdog set to %u s (persisted)", (unsigned)seconds);
}

void irrigoto_get_last_sleep_reason(char *buf, size_t len)
{
    if (buf == NULL || len == 0) return;
    if (s_last_sleep_reason[0] == '\0') {
        strncpy(buf, "none", len - 1);
    } else {
        strncpy(buf, s_last_sleep_reason, len - 1);
    }
    buf[len - 1] = '\0';
}

// Sleep request flag. Set by irrigoto_sleep_now() / check_inactivity(),
// consumed by IrrigotoComponent::loop() which hands off to ESPHome's
// deep_sleep component for graceful HA disconnect.
static volatile bool     s_sleep_requested  = false;
static volatile uint32_t s_sleep_duration_s = 0;

void irrigoto_sleep_now_with_reason(uint32_t duration_s, const char *reason)
{
    if (duration_s == 0) duration_s = s_sleep_dur_s;
    // Schedule-aware sleep: if a scheduled run is coming up sooner than
    // the configured sleep duration, shorten the sleep so we wake in
    // time. Wake 60 s before the scheduled minute so there's room for
    // WiFi/HA reconnect, time sync, and the scheduler's poll cadence.
    time_t now = time(NULL);
    if (now > 1700000000) {
        time_t next_t = 0;
        int    next_zone = 0;
        if (irrigoto_schedule_next_run(now, &next_t, &next_zone)) {
            const uint32_t WAKE_GRACE_S = 60u;
            time_t wake_target = next_t - (time_t)WAKE_GRACE_S;
            if (wake_target > now) {
                uint32_t until_wake = (uint32_t)(wake_target - now);
                if (until_wake < duration_s) {
                    INFO("Schedule-aware sleep: shortening %lu s -> %u s "
                         "(wake %u s before zone %d at scheduled time)",
                         (unsigned long)duration_s, (unsigned)until_wake,
                         (unsigned)WAKE_GRACE_S, next_zone);
                    duration_s = until_wake;
                }
            }
        }
    }
    INFO("Sleep requested (%lu s, reason=%s) — preparing safe state, handing to ESPHome",
         (unsigned long)duration_s, reason ? reason : "manual");
    pm_record_reason(reason ? reason : "manual");
    prepare_for_sleep();
    s_sleep_duration_s = duration_s;
    s_sleep_requested  = true;
    // The ESPHome component's loop() picks this up on its next tick and
    // calls deep_sleep->begin_sleep(), which announces to HA then sleeps.
}

void irrigoto_sleep_now(uint32_t duration_s)
{
    irrigoto_sleep_now_with_reason(duration_s, "manual");
}

bool irrigoto_sleep_request_pending(uint32_t *duration_s_out)
{
    if (!s_sleep_requested) return false;
    if (duration_s_out) *duration_s_out = s_sleep_duration_s;
    return true;
}

void irrigoto_sleep_request_clear(void)
{
    s_sleep_requested  = false;
    s_sleep_duration_s = 0;
}

// ── Diagnostics ──────────────────────────────────────────────────────────────

float irrigoto_read_pressure_now(void)
{
    float psi;
    return mprls_read(&psi) ? psi : -1.0f;
}

bool irrigoto_valve_goto(float target_deg)
{
    INFO("HA valve_goto: %.1f deg", target_deg);
    sensor_rail_on();   // AS5600L position sensor
    motor_rail_on();    // 9V to motor + release brake (GPIO_K low)
    return valve_goto(target_deg, 2.0f, 10000, true);
}

bool irrigoto_nozzle_goto(float target_deg)
{
    INFO("HA nozzle_goto: %.1f deg", target_deg);
    sensor_rail_on();
    motor_rail_on();
    return nozzle_goto(target_deg, 2.0f, 10000, true);
}

// ── Zone-trace glide state (b417) ────────────────────────────────────────────
// Shared with the aim_to / water_arc / valve open-close guards below;
// the implementation lives after water_arc.
static volatile bool s_zone_trace_busy  = false;
// 0 = run, 1 = stop (trace task closes the valve), 2 = stop only
// (stop_and_close owns the valve -- don't fight its valve_goto).
static volatile int  s_zone_trace_abort = 0;

// b325: aim_to -- drive nozzle and valve to their respective targets
// simultaneously, as quickly as the hardware allows.
//
// Differs from the existing nozzle_goto / valve_goto in three ways:
//   1. Both motors run in PARALLEL (single tick loop reading both encoders
//      and updating both motor PWMs per tick), not back-to-back.
//   2. Uses chase_motor_* high-frequency PWM primitives instead of the
//      slower closed-loop nozzle_goto / valve_goto with their built-in
//      ramps + backlash compensation. Trade ~2 deg of tolerance for speed.
//   3. No backlash / dead-spot kick logic -- this is positioning for "aim
//      the stream", not for accurate ring watering. If a target is
//      slightly off, the user re-aims.
//
// Valve direction polarity is INVERTED on this device (chase_motor_apply
// with dir=+1 closes the valve; verified b318); encoded as VALVE_OPEN_DIR
// below.
//
// Returns true if BOTH motors reached their targets within tolerance
// before the timeout; false otherwise. On failure, motors are stopped
// at their current position (rails left on -- same convention as
// valve_goto / nozzle_goto).
static bool aim_to(float target_bearing, float target_valve_deg)
{
    if (s_web_water_mode != 0) {
        INFO("aim: refusing -- watering in progress (mode %d)",
             s_web_water_mode);
        return false;
    }
    if (s_zone_trace_busy) {
        INFO("aim: refusing -- zone trace in progress");
        return false;
    }

    // Clamp targets into valid ranges.
    if (target_valve_deg < VALVE_CAL_START_DEG) target_valve_deg = VALVE_CAL_START_DEG;
    if (target_valve_deg > VALVE_OPEN_DEG)      target_valve_deg = VALVE_OPEN_DEG;
    while (target_bearing <   0.0f) target_bearing += 360.0f;
    while (target_bearing >= 360.0f) target_bearing -= 360.0f;

    INFO("aim: target bearing=%.1f valve=%.1f", target_bearing, target_valve_deg);

    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(150));

    chase_motor_t nm = {0}, vm = {0};
    if (!chase_motor_setup(&nm, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
        INFO("aim: nozzle PWM setup failed");
        return false;
    }
    if (!chase_motor_setup(&vm, VALVE_PIN_FWD, VALVE_PIN_REV)) {
        INFO("aim: valve PWM setup failed");
        chase_motor_teardown(&nm);
        return false;
    }

    // Tuning. High RUN_DUTY for speed; cosine-eased decel inside DECEL_DEG
    // of target so the approach stays smooth and doesn't overshoot.
    const uint16_t N_RUN_DUTY    = 380;
    const uint16_t V_RUN_DUTY    = 320;
    const uint16_t N_MIN_DUTY    = 80;
    const uint16_t V_MIN_DUTY    = 70;
    const float    N_TOL_DEG     = 1.5f;
    const float    V_TOL_DEG     = 1.0f;
    const float    N_DECEL_DEG   = 12.0f;
    const float    V_DECEL_DEG   = 5.0f;
    const uint32_t TICK_MS       = 20;
    const uint32_t TIMEOUT_MS    = 8000;
    const int      VALVE_OPEN_DIR = -1;   // FWD pin closes valve on this device

    uint16_t n_raw = 0, v_raw = 0;
    as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
    as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
    float cur_b = n_raw * (360.0f / 4096.0f);
    float cur_v = v_raw * (360.0f / 4096.0f);

    float err_b = target_bearing - cur_b;
    while (err_b >  180.0f) err_b -= 360.0f;
    while (err_b < -180.0f) err_b += 360.0f;
    float err_v = target_valve_deg - cur_v;

    int n_dir = (err_b >= 0.0f) ? +1 : -1;
    int v_dir = (err_v >= 0.0f) ? VALVE_OPEN_DIR : -VALVE_OPEN_DIR;

    bool n_done = (fabsf(err_b) < N_TOL_DEG);
    bool v_done = (fabsf(err_v) < V_TOL_DEG);

    chase_motor_apply(&nm, n_dir, n_done ? 0 : N_RUN_DUTY);
    chase_motor_apply(&vm, v_dir, v_done ? 0 : V_RUN_DUTY);
    s_nozzle_last_dir = n_dir;
    s_valve_last_dir  = v_dir;

    TickType_t t_start = xTaskGetTickCount();
    while (!n_done || !v_done) {
        if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_start) >= TIMEOUT_MS) {
            INFO("aim: timeout (n_done=%d v_done=%d)", n_done, v_done);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        TOUCH_ACTIVITY();

        as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
        as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
        cur_b = n_raw * (360.0f / 4096.0f);
        cur_v = v_raw * (360.0f / 4096.0f);

        // Nozzle
        if (!n_done) {
            err_b = target_bearing - cur_b;
            while (err_b >  180.0f) err_b -= 360.0f;
            while (err_b < -180.0f) err_b += 360.0f;
            float abs_e = fabsf(err_b);
            int new_dir = (err_b >= 0.0f) ? +1 : -1;
            if (abs_e < N_TOL_DEG) {
                chase_motor_apply(&nm, 0, 0);
                n_done = true;
            } else if (new_dir != n_dir) {
                // Overshot -- smooth reverse via the cosine-eased fast ramp.
                chase_motor_reverse_fast(&nm, N_RUN_DUTY);
                n_dir = nm.dir;
            } else if (abs_e < N_DECEL_DEG) {
                float t = abs_e / N_DECEL_DEG;
                float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                uint16_t d = (uint16_t)((float)N_MIN_DUTY
                                + ease * (float)(N_RUN_DUTY - N_MIN_DUTY));
                chase_motor_apply(&nm, n_dir, d);
            } else {
                chase_motor_apply(&nm, n_dir, N_RUN_DUTY);
            }
        }

        // Valve
        if (!v_done) {
            err_v = target_valve_deg - cur_v;
            float abs_e = fabsf(err_v);
            int new_dir = (err_v >= 0.0f) ? VALVE_OPEN_DIR : -VALVE_OPEN_DIR;
            if (abs_e < V_TOL_DEG) {
                chase_motor_apply(&vm, 0, 0);
                v_done = true;
            } else if (new_dir != v_dir) {
                chase_motor_reverse_fast(&vm, V_RUN_DUTY);
                v_dir = vm.dir;
            } else if (abs_e < V_DECEL_DEG) {
                float t = abs_e / V_DECEL_DEG;
                float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                uint16_t d = (uint16_t)((float)V_MIN_DUTY
                                + ease * (float)(V_RUN_DUTY - V_MIN_DUTY));
                chase_motor_apply(&vm, v_dir, d);
            } else {
                chase_motor_apply(&vm, v_dir, V_RUN_DUTY);
            }
        }

        s_nozzle_last_dir = n_dir;
        s_valve_last_dir  = v_dir;
    }

    chase_motor_apply(&nm, 0, 0);
    chase_motor_apply(&vm, 0, 0);
    chase_motor_teardown(&nm);
    chase_motor_teardown(&vm);

    as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
    as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
    float fin_b = n_raw * (360.0f / 4096.0f);
    float fin_v = v_raw * (360.0f / 4096.0f);
    uint32_t elapsed = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_start);
    INFO("aim: done in %u ms -- bearing %.1f (tgt %.1f) valve %.1f (tgt %.1f) ok=%d",
         (unsigned)elapsed, fin_b, target_bearing, fin_v, target_valve_deg,
         n_done && v_done);

    return n_done && v_done;
}

bool irrigoto_aim_valve(float bearing_deg, float valve_deg)
{
    INFO("HA aim_valve: bearing=%.1f valve=%.1f", bearing_deg, valve_deg);
    return aim_to(bearing_deg, valve_deg);
}

bool irrigoto_aim_throw(float bearing_deg, float throw_mm)
{
    INFO("HA aim_throw: bearing=%.1f throw=%.0f mm", bearing_deg, throw_mm);
    float valve_deg = cal_throw_to_valve_deg(throw_mm);
    return aim_to(bearing_deg, valve_deg);
}

// b329: forward declarations for the water_arc task plumbing -- the
// synchronous worker becomes static, the public entry point spawns a
// task so the ESPHome API thread doesn't block (which was knocking HA
// offline mid-arc).
typedef struct {
    float       throw_mm;
    float       arc_start_deg;
    float       arc_end_deg;
    float       speed_dps;
    char        direction[8];   // "cw" / "ccw" / "shortest" / ""
} water_arc_args_t;
static water_arc_args_t s_water_arc_args;
static volatile bool    s_water_arc_busy = false;
static bool water_arc_run(float throw_mm, float arc_start_deg,
                          float arc_end_deg, float speed_dps,
                          const char *direction);

// b326 / b328: water_arc -- sweep the nozzle from arc_start_deg to
// arc_end_deg at constant angular velocity (speed_dps) while holding
// the valve at the throw distance. Building block for HA automations
// doing arc-based watering ("water the edge at 6 m, 30 -> 90 deg CW,
// 2 dps").
//
// Phases:
//   1. aim_to(arc_start, throw -> valve_deg)   -- fast parallel positioning
//   2. Traverse nozzle at speed_dps with cosine-eased decel near arc_end
//   3. Valve LEFT OPEN at end -- caller chains arcs or calls close_valve
//      / stop to stop the flow.
//
// direction: "cw"  -- increasing bearing, possibly the long way around
//            "ccw" -- decreasing bearing, possibly the long way around
//            anything else (incl. "shortest", "" or NULL) -- shortest path
//            via (arc_end - arc_start) wrapped to (-180, 180]. Matches
//            the b326 default behavior.
//
// Distance tracking uses unwrapped per-tick deltas summed into a signed
// "traveled" accumulator -- this lets us correctly detect arc completion
// for long-way arcs that cross the 0/360 boundary or exceed 180 deg.
//
// speed_dps clamped to [1, 25]. Below ~2.5 dps the motor's stall band
// dominates so motion gets choppy -- caller's problem, not ours.
//
// Refuses if a watering run is in progress.
static bool water_arc_run(float throw_mm, float arc_start_deg,
                          float arc_end_deg, float speed_dps,
                          const char *direction)
{
    if (s_web_water_mode != 0) {
        INFO("water_arc: refusing -- watering in progress (mode %d)",
             s_web_water_mode);
        return false;
    }

    // Clamp + normalise inputs.
    if (speed_dps <  1.0f) speed_dps =  1.0f;
    if (speed_dps > 25.0f) speed_dps = 25.0f;
    while (arc_start_deg <   0.0f) arc_start_deg += 360.0f;
    while (arc_start_deg >= 360.0f) arc_start_deg -= 360.0f;
    while (arc_end_deg   <   0.0f) arc_end_deg   += 360.0f;
    while (arc_end_deg   >= 360.0f) arc_end_deg   -= 360.0f;

    // Direction-aware signed arc length. CW = positive, CCW = negative,
    // shortest path = whichever is <= 180 deg.
    float arc_diff;
    int   n_dir;
    const char *dir_label;
    if (direction && (strcasecmp(direction, "cw") == 0)) {
        arc_diff = arc_end_deg - arc_start_deg;
        if (arc_diff <= 0.0f) arc_diff += 360.0f;     // long way if needed
        n_dir = +1;
        dir_label = "CW";
    } else if (direction && (strcasecmp(direction, "ccw") == 0)) {
        arc_diff = arc_end_deg - arc_start_deg;
        if (arc_diff >= 0.0f) arc_diff -= 360.0f;
        n_dir = -1;
        dir_label = "CCW";
    } else {
        // "shortest" / "" / NULL / anything unrecognised -> shortest path
        arc_diff = arc_end_deg - arc_start_deg;
        while (arc_diff >  180.0f) arc_diff -= 360.0f;
        while (arc_diff < -180.0f) arc_diff += 360.0f;
        n_dir = (arc_diff >= 0.0f) ? +1 : -1;
        dir_label = "shortest";
    }
    float arc_span = fabsf(arc_diff);
    if (arc_span < 1.0f) {
        INFO("water_arc: refusing -- arc too short (%.2f deg, %s)",
             arc_diff, dir_label);
        return false;
    }

    // Throw -> valve angle (cal-based + safety clamp).
    float valve_deg = cal_throw_to_valve_deg(throw_mm);
    if (valve_deg < VALVE_CAL_START_DEG) valve_deg = VALVE_CAL_START_DEG;
    if (valve_deg > VALVE_OPEN_DEG)      valve_deg = VALVE_OPEN_DEG;

    INFO("water_arc: throw=%.0f mm (valve=%.1f), arc %.1f -> %.1f "
         "(%.1f deg %s, dir=%s), speed=%.1f dps",
         throw_mm, valve_deg, arc_start_deg, arc_end_deg, arc_span,
         (n_dir > 0) ? "+1" : "-1", dir_label, speed_dps);

    // Phase 1: parallel position to (arc_start_deg, valve_deg).
    INFO("water_arc: positioning...");
    if (!aim_to(arc_start_deg, valve_deg)) {
        INFO("water_arc: position failed -- not traversing");
        return false;
    }

    // Phase 2: constant-velocity sweep with decel at end.
    //
    // K_DUTY_PER_DPS from b318 empirical data (duty 120 -> ~8 dps for
    // nozzle, so 15 duty/dps). b346: duty 45 was BELOW the reliable
    // start threshold for the nozzle motor -- it stayed parked instead
    // of sweeping (b345 test). Chase mode uses cruise duty 90 with a
    // floor of 60. Bumped floor to 70 (well above stall + margin).
    // Cost: "slow" speeds (1-4 dps requested) all clamp up to the
    // motor's actual minimum (~5 dps from duty 70). Fine for fun mode.
    const float    K_DUTY_PER_DPS = 15.0f;
    const float    DECEL_DEG      = 5.0f;
    const float    TOL_DEG        = 1.5f;
    const uint16_t MIN_DUTY       = 70;   // also the decel floor
    const uint32_t TICK_MS        = 30;

    uint16_t run_duty = (uint16_t)(K_DUTY_PER_DPS * speed_dps);
    if (run_duty < MIN_DUTY) run_duty = MIN_DUTY;
    if (run_duty > 450)      run_duty = 450;

    chase_motor_t nm = {0};
    if (!chase_motor_setup(&nm, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
        INFO("water_arc: nozzle PWM setup failed");
        return false;
    }

    chase_motor_apply(&nm, n_dir, run_duty);
    s_nozzle_last_dir = n_dir;

    // Timeout: 2x expected duration + 3 s buffer, capped at 90 s for
    // long-way arcs (up to 359 deg at 4 dps = 90 s).
    uint32_t expected_ms = (uint32_t)(arc_span / speed_dps * 1000.0f);
    uint32_t timeout_ms  = expected_ms * 2 + 3000;
    if (timeout_ms > 90000) timeout_ms = 90000;

    INFO("water_arc: sweeping at duty=%u (expected ~%u ms, timeout %u ms)",
         (unsigned)run_duty, (unsigned)expected_ms, (unsigned)timeout_ms);

    // Track traveled angle using unwrapped per-tick deltas. Needed for
    // long-way arcs where the "remaining angle to arc_end" wrap would
    // otherwise become ambiguous past 180 deg.
    uint16_t n_raw0 = 0;
    as5600_read(ADDR_AS5600, &n_raw0, NULL, NULL);
    float prev_b   = n_raw0 * (360.0f / 4096.0f);
    float traveled = 0.0f;    // signed; sign matches n_dir

    TickType_t t_start = xTaskGetTickCount();
    bool reached = false;
    while (!reached) {
        if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_start) >= timeout_ms) {
            INFO("water_arc: timeout traversing arc (traveled %.1f / %.1f deg)",
                 fabsf(traveled), arc_span);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        TOUCH_ACTIVITY();

        uint16_t n_raw = 0;
        as5600_read(ADDR_AS5600, &n_raw, NULL, NULL);
        float cur_b = n_raw * (360.0f / 4096.0f);

        // Unwrapped per-tick delta -- adds the angular distance moved this
        // tick to the running total, regardless of 0/360 wrap.
        float delta = cur_b - prev_b;
        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        traveled += delta;
        prev_b    = cur_b;

        // Progress along the commanded direction (positive = on track).
        float progress = traveled * (float)n_dir;
        float to_go    = arc_span - progress;

        if (to_go < TOL_DEG) {
            chase_motor_apply(&nm, 0, 0);
            reached = true;
            break;
        }
        if (to_go < DECEL_DEG) {
            float t    = to_go / DECEL_DEG;
            float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
            uint16_t d = (uint16_t)((float)MIN_DUTY
                            + ease * (float)(run_duty - MIN_DUTY));
            chase_motor_apply(&nm, n_dir, d);
        } else {
            chase_motor_apply(&nm, n_dir, run_duty);
        }
    }

    chase_motor_apply(&nm, 0, 0);
    chase_motor_teardown(&nm);

    uint16_t n_raw = 0;
    as5600_read(ADDR_AS5600, &n_raw, NULL, NULL);
    float fin_b = n_raw * (360.0f / 4096.0f);
    uint32_t elapsed = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_start);
    INFO("water_arc: complete in %u ms, bearing %.1f (target %.1f), traveled "
         "%.1f / %.1f deg %s, reached=%d -- valve LEFT OPEN at throw %.0f mm "
         "(valve_deg %.1f)",
         (unsigned)elapsed, fin_b, arc_end_deg, fabsf(traveled), arc_span,
         dir_label, reached, throw_mm, valve_deg);

    return reached;
}

// b329: task wrapper. The ESPHome API service handler can't block while
// water_arc traverses (up to ~90 s at slow speeds) -- HA's keepalive
// times out and the device gets marked unavailable. Spawn a pinned task
// on PRO_CPU (per b294 flash-cache race rule) and return immediately.
static void water_arc_task(void *arg)
{
    (void)arg;  // args are in s_water_arc_args
    water_arc_args_t a = s_water_arc_args;   // local copy
    (void)water_arc_run(a.throw_mm, a.arc_start_deg, a.arc_end_deg,
                        a.speed_dps, a.direction);
    s_water_arc_busy = false;
    vTaskDelete(NULL);
}

bool irrigoto_water_arc(float throw_mm, float arc_start_deg,
                        float arc_end_deg, float speed_dps,
                        const char *direction)
{
    if (s_water_arc_busy) {
        INFO("water_arc: refusing -- water_arc task already in progress");
        return false;
    }
    if (s_web_water_mode != 0) {
        INFO("water_arc: refusing -- watering in progress (mode %d)",
             s_web_water_mode);
        return false;
    }
    if (s_zone_trace_busy) {
        INFO("water_arc: refusing -- zone trace in progress");
        return false;
    }
    s_water_arc_args.throw_mm      = throw_mm;
    s_water_arc_args.arc_start_deg = arc_start_deg;
    s_water_arc_args.arc_end_deg   = arc_end_deg;
    s_water_arc_args.speed_dps     = speed_dps;
    s_water_arc_args.direction[0]  = '\0';
    if (direction) {
        strncpy(s_water_arc_args.direction, direction,
                sizeof(s_water_arc_args.direction) - 1);
        s_water_arc_args.direction[sizeof(s_water_arc_args.direction) - 1] = '\0';
    }
    s_water_arc_busy = true;
    // b330: stack bumped 8192 -> 16384 to match the chase-mode water_task.
    // b329 used 8 KB and the device rolled back to b328 after the first
    // call -- the call chain (water_arc_run -> aim_to -> chase_motor_* +
    // multiple INFO format strings) is deeper than 8 KB safely allows.
    BaseType_t r = xTaskCreatePinnedToCore(
        water_arc_task, "water_arc_task", 16384, NULL, 9, NULL, PRO_CPU_NUM);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "water_arc: xTaskCreate failed");
        s_water_arc_busy = false;
        return false;
    }
    return true;
}

// b417: zone_trace -- glide the stream around a zone's stored perimeter in
// ONE continuous motion. Built after the HA-side water_arc-chaining attempt
// (script.irrigoto_glide_perimeter v1) proved jerky: HA can't see arc
// completion, so every sub-arc needed a timing margin that read as a pause,
// and the valve only re-aimed between arcs.
//
// Motion model:
//   - Waypoints are the zone's points in walk_idx order; close_loop appends
//     waypoint 0 again so the glide ends where it started.
//   - Each leg is derived from the ACTUAL current bearing (positioning error
//     never accumulates) and swept at constant duty, water_arc style.
//   - The valve chases a target interpolated along the leg's measured sweep
//     progress -- throw tracks the polygon edge, not a per-leg constant.
//   - Same-direction vertices carry straight through at full duty; the sweep
//     only brakes (cosine ease) where the walk reverses direction, goes
//     radial (bearing-change < 1 deg, valve-only leg), or ends.
//   - If the valve lags its target by > V_LAG_DEG the nozzle drops to its
//     floor duty until the valve catches up (keeps the landing point on the
//     edge through fast throw swings like Edge's 8 m -> 0.5 m corner run).
//   - Leg 0 is a dry reposition: full aim-speed duty to the first point with
//     the valve HELD (sentinel target < 0), then a radial "open to first
//     throw" leg. Water starts at the perimeter, not smeared across the yard.
//
// Ends (or aborts with s_zone_trace_abort == 1) by closing the valve and
// powering the rails down -- a self-contained show, unlike water_arc's
// chain-friendly valve-left-open. abort == 2 means stop_and_close is doing
// the teardown; release the motors and DON'T touch the valve.
typedef struct {
    uint16_t zone_id;
    float    speed_dps;
    bool     close_loop;
} zone_trace_args_t;
static zone_trace_args_t s_zone_trace_args;
static zone_perimeter_t  s_zone_trace_zp;   // static: keep the task stack lean

static void zone_trace_run(uint16_t zone_id, float speed_dps, bool close_loop)
{
    if (speed_dps <  1.0f) speed_dps =  1.0f;
    if (speed_dps > 25.0f) speed_dps = 25.0f;

    // Load the zone (same sources as /zone/export: LittleFS, NVS fallback
    // for zone 0).
    zone_perimeter_t *zp = &s_zone_trace_zp;
    memset(zp, 0, sizeof(*zp));
    bool ok = false;
    if (storage_ready()) {
        char lfsname[32] = {0};
        ok = (storage_zone_load(zone_id, lfsname, sizeof(lfsname), zp) == ESP_OK);
    }
    if (!ok && zone_id == 0) {
        zone_load_nvs(zp);
        ok = zp->num_points > 0;
    }
    if (!ok || zp->num_points < 2) {
        INFO("zone_trace: zone %u unavailable or has < 2 points", zone_id);
        return;
    }

    // Waypoint arrays in walk order. +2 slots: the "open valve at the first
    // point" radial leg and the optional close-the-loop repeat of point 0.
    // tv < 0 is the "hold valve" sentinel (leg 0 dry reposition).
    static float tb[ZONE_MAX_PERIM_POINTS + 2];
    static float tv[ZONE_MAX_PERIM_POINTS + 2];
    int n = 0;
    for (int want = 0; want < zp->num_points; want++) {   // O(n^2), n <= 36
        for (int i = 0; i < zp->num_points; i++) {
            if (zp->points[i].walk_idx == want) {
                tb[n] = zp->points[i].nozzle_deg;
                tv[n] = cal_throw_to_valve_deg(zp->points[i].throw_mm);
                n++;
                break;
            }
        }
    }
    if (n != zp->num_points) {   // walk_idx isn't a clean 0..n-1 permutation
        n = 0;
        for (int i = 0; i < zp->num_points; i++) {
            tb[n] = zp->points[i].nozzle_deg;
            tv[n] = cal_throw_to_valve_deg(zp->points[i].throw_mm);
            n++;
        }
    }
    for (int i = 0; i < n; i++) {
        if (tv[i] < VALVE_CAL_START_DEG) tv[i] = VALVE_CAL_START_DEG;
        if (tv[i] > VALVE_OPEN_DEG)      tv[i] = VALVE_OPEN_DEG;
    }
    if (close_loop) { tb[n] = tb[0]; tv[n] = tv[0]; n++; }
    // Prepend the dry reposition: waypoint 0 twice, first with valve held.
    for (int i = n; i > 0; i--) { tb[i] = tb[i - 1]; tv[i] = tv[i - 1]; }
    tv[0] = -1.0f;   // hold valve while flying to tb[0]
    n++;

    INFO("zone_trace: zone %u, %d waypoints (%s), speed %.1f dps",
         zone_id, n - 1, close_loop ? "closed loop" : "open path", speed_dps);

    // Tuning. Nozzle constants match water_arc (b318 K, b346 floor); valve
    // chase is gentler than aim_to's 320 -- it follows a slow-moving target,
    // not a step.
    const float    K_DUTY_PER_DPS = 15.0f;
    const uint16_t N_MIN_DUTY     = 70;
    const uint16_t N_ALIGN_DUTY   = 380;   // leg-0 dry reposition (aim_to speed)
    const float    N_TOL_DEG      = 1.0f;
    const float    MIN_SWEEP_DEG  = 1.0f;  // below this a leg is radial
    const uint16_t V_CHASE_DUTY   = 220;
    const uint16_t V_MIN_DUTY     = 70;
    const float    V_TOL_DEG      = 0.8f;
    const float    V_DECEL_DEG    = 4.0f;
    const float    V_LAG_DEG      = 5.0f;
    const float    V_NOM_DPS      = 8.0f;  // timeout estimates only
    const uint32_t TICK_MS        = 30;
    const int      VALVE_OPEN_DIR = -1;    // FWD pin closes valve (b318)

    uint16_t run_duty = (uint16_t)(K_DUTY_PER_DPS * speed_dps);
    if (run_duty < N_MIN_DUTY) run_duty = N_MIN_DUTY;
    if (run_duty > 450)        run_duty = 450;
    float eff_dps = (float)run_duty / K_DUTY_PER_DPS;

    sensor_rail_on();
    motor_rail_on();
    vTaskDelay(pdMS_TO_TICKS(150));

    chase_motor_t nm = {0}, vm = {0};
    if (!chase_motor_setup(&nm, NOZZLE_PIN_FWD, NOZZLE_PIN_REV)) {
        INFO("zone_trace: nozzle PWM setup failed");
        goto closeout;
    }
    if (!chase_motor_setup(&vm, VALVE_PIN_FWD, VALVE_PIN_REV)) {
        INFO("zone_trace: valve PWM setup failed");
        chase_motor_teardown(&nm);
        goto closeout;
    }

    {
        bool aborted = false;
        int  v_dir   = 0;   // current valve drive dir, 0 = stopped
        TickType_t t_trace = xTaskGetTickCount();

        for (int leg = 0; leg + 1 < n && !aborted; leg++) {
            uint16_t n_raw = 0, v_raw = 0;
            as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
            as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
            float cur_b = n_raw * (360.0f / 4096.0f);
            float v0    = v_raw * (360.0f / 4096.0f);

            // Leg geometry from the actual current position.
            float d = tb[leg + 1] - cur_b;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            float span   = fabsf(d);
            int   dirn   = (d >= 0.0f) ? +1 : -1;
            float v1     = (tv[leg + 1] < 0.0f) ? v0 : tv[leg + 1];
            bool  radial = (span < MIN_SWEEP_DEG);
            bool  dry    = (tv[leg + 1] < 0.0f);
            uint16_t leg_duty = dry ? N_ALIGN_DUTY : run_duty;
            float decel_band  = (leg_duty > 200) ? 12.0f : 5.0f;

            // Brake into this leg's end unless the sweep carries straight
            // into the next leg (same direction, non-radial).
            bool brake_end = true;
            if (!radial && leg + 2 < n) {
                float dn = tb[leg + 2] - tb[leg + 1];
                while (dn >  180.0f) dn -= 360.0f;
                while (dn < -180.0f) dn += 360.0f;
                brake_end = (fabsf(dn) < MIN_SWEEP_DEG) ||
                            (((dn >= 0.0f) ? +1 : -1) != dirn);
            }

            float exp_s = radial ? 0.0f : span / (dry ? (N_ALIGN_DUTY / K_DUTY_PER_DPS)
                                                      : eff_dps);
            float vexp  = fabsf(v1 - v0) / V_NOM_DPS;
            if (vexp > exp_s) exp_s = vexp;
            uint32_t leg_timeout = (uint32_t)(exp_s * 2000.0f) + 4000;

            if (radial) chase_motor_apply(&nm, 0, 0);

            float prev_b = cur_b, traveled = 0.0f;
            TickType_t t_leg = xTaskGetTickCount();

            for (;;) {
                if (s_zone_trace_abort) { aborted = true; break; }
                if (s_web_water_mode != 0) {
                    // A watering run started under us -- it owns the motors
                    // and the valve now. Release everything and vanish.
                    INFO("zone_trace: watering started -- yielding");
                    s_zone_trace_abort = 2;
                    aborted = true;
                    break;
                }
                if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t_leg) >= leg_timeout) {
                    INFO("zone_trace: leg %d timeout (%.1f / %.1f deg) -- continuing",
                         leg, fabsf(traveled), span);
                    break;   // next leg re-derives from the actual position
                }
                vTaskDelay(pdMS_TO_TICKS(TICK_MS));
                TOUCH_ACTIVITY();

                as5600_read(ADDR_AS5600,  &n_raw, NULL, NULL);
                as5600_read(ADDR_AS5600L, &v_raw, NULL, NULL);
                cur_b = n_raw * (360.0f / 4096.0f);
                float cur_v = v_raw * (360.0f / 4096.0f);

                float db = cur_b - prev_b;
                while (db >  180.0f) db -= 360.0f;
                while (db < -180.0f) db += 360.0f;
                traveled += db;
                prev_b    = cur_b;

                float progress = radial ? span : traveled * (float)dirn;
                if (progress < 0.0f)  progress = 0.0f;
                if (progress > span)  progress = span;
                float frac = (radial || span < 0.01f) ? 1.0f : progress / span;

                // Valve: chase the along-leg interpolated target.
                float v_tgt = v0 + (v1 - v0) * frac;
                float v_err = v_tgt - cur_v;
                float v_abs = fabsf(v_err);
                if (v_abs < V_TOL_DEG) {
                    if (v_dir) { chase_motor_apply(&vm, 0, 0); v_dir = 0; }
                } else {
                    int want = (v_err >= 0.0f) ? VALVE_OPEN_DIR : -VALVE_OPEN_DIR;
                    uint16_t vd;
                    if (v_abs < V_DECEL_DEG) {
                        float t    = v_abs / V_DECEL_DEG;
                        float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                        vd = (uint16_t)((float)V_MIN_DUTY
                                + ease * (float)(V_CHASE_DUTY - V_MIN_DUTY));
                    } else {
                        vd = V_CHASE_DUTY;
                    }
                    if (v_dir && want != v_dir) {
                        chase_motor_reverse_fast(&vm, vd);
                        v_dir = vm.dir;
                    } else {
                        chase_motor_apply(&vm, want, vd);
                        v_dir = want;
                    }
                    s_valve_last_dir = v_dir;
                }

                if (radial) {
                    if (v_abs < V_TOL_DEG) break;   // valve settled -> leg done
                    continue;
                }

                // Nozzle: constant duty, eased only into braking ends.
                float to_go = span - progress;
                if (to_go <= N_TOL_DEG) {
                    if (brake_end) chase_motor_apply(&nm, 0, 0);
                    break;
                }
                uint16_t nd = leg_duty;
                if (brake_end && to_go < decel_band) {
                    float t    = to_go / decel_band;
                    float ease = 0.5f * (1.0f - cosf((float)M_PI * t));
                    nd = (uint16_t)((float)N_MIN_DUTY
                            + ease * (float)(leg_duty - N_MIN_DUTY));
                }
                if (v_abs > V_LAG_DEG) nd = N_MIN_DUTY;   // let the valve catch up
                chase_motor_apply(&nm, dirn, nd);
                s_nozzle_last_dir = dirn;
            }
        }

        chase_motor_apply(&nm, 0, 0);
        chase_motor_apply(&vm, 0, 0);
        chase_motor_teardown(&nm);
        chase_motor_teardown(&vm);

        INFO("zone_trace: %s after %u ms",
             aborted ? "aborted" : "complete",
             (unsigned)pdTICKS_TO_MS(xTaskGetTickCount() - t_trace));
    }

closeout:
    if (s_zone_trace_abort != 2) {
        valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, true);
        motor_rail_off();
        sensor_rail_off();
    }
}

static void zone_trace_task(void *arg)
{
    (void)arg;   // args are in s_zone_trace_args
    zone_trace_args_t a = s_zone_trace_args;
    zone_trace_run(a.zone_id, a.speed_dps, a.close_loop);
    s_zone_trace_busy = false;
    vTaskDelete(NULL);
}

bool irrigoto_zone_trace_start(uint16_t zone_id, float speed_dps, bool close_loop)
{
    if (s_zone_trace_busy) {
        INFO("zone_trace: refusing -- trace already in progress");
        return false;
    }
    if (s_water_arc_busy) {
        INFO("zone_trace: refusing -- water_arc in progress");
        return false;
    }
    if (s_web_water_mode != 0) {
        INFO("zone_trace: refusing -- watering in progress (mode %d)",
             s_web_water_mode);
        return false;
    }
    if (s_exp_running || (s_wcal.state != WCAL_IDLE && s_wcal.state != WCAL_DONE)) {
        INFO("zone_trace: refusing -- calibration/experiment in progress");
        return false;
    }
    s_zone_trace_args.zone_id    = zone_id;
    s_zone_trace_args.speed_dps  = speed_dps;
    s_zone_trace_args.close_loop = close_loop;
    s_zone_trace_abort = 0;
    s_zone_trace_busy  = true;
    // Stack + core per water_arc (b330 depth lesson, b294 PRO_CPU rule).
    BaseType_t r = xTaskCreatePinnedToCore(
        zone_trace_task, "zone_trace", 16384, NULL, 9, NULL, PRO_CPU_NUM);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "zone_trace: xTaskCreate failed");
        s_zone_trace_busy = false;
        return false;
    }
    return true;
}

void irrigoto_zone_trace_stop(void)
{
    if (s_zone_trace_busy) s_zone_trace_abort = 1;
}

void irrigoto_stop_and_close(void)
{
    INFO("HA stop_all: aborting all activity and closing valve");

    // Watering loop — sets the abort flag the water_task checks.
    irrigoto_stop_watering();

    // b417: zone-trace glide — stop-only (2): we drive the valve below,
    // the trace task must release the motors and stay off it.
    if (s_zone_trace_busy) s_zone_trace_abort = 2;

    // Calibration / jog tasks — set state to IDLE. The jog task polls
    // state and exits. Pressure-scan and nozzle-cal tasks don't poll
    // mid-loop, but powering off the motor rail below halts their
    // motor moves; they'll error out and exit on their next step.
    bool was_cal = (s_wcal.state != WCAL_IDLE && s_wcal.state != WCAL_DONE);
    if (was_cal) {
        ESP_LOGI(TAG, "stop_all: cancelling calibration (state was %d)", (int)s_wcal.state);
        s_wcal.state = WCAL_IDLE;
        snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Stopped by user.");
    }

    // Let the loops observe the abort flags and release the motors
    // before we drive the valve ourselves.
    vTaskDelay(pdMS_TO_TICKS(500));

    sensor_rail_on();
    motor_rail_on();
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, true);
    motor_rail_off();
    sensor_rail_off();
}

void irrigoto_valve_close(void)
{
    if (s_zone_trace_busy) {
        // b417: closing the valve during a glide means "stop the show" --
        // signal the trace task; IT closes the valve (two concurrent
        // valve_goto calls would fight on the PWM pins).
        INFO("HA close_valve: zone trace active -- stopping it instead");
        irrigoto_zone_trace_stop();
        return;
    }
    INFO("HA close_valve");
    sensor_rail_on();
    motor_rail_on();
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, true);
}

void irrigoto_valve_open(void)
{
    if (s_zone_trace_busy) {
        INFO("HA open_valve: refusing -- zone trace owns the valve");
        return;
    }
    INFO("HA open_valve");
    sensor_rail_on();
    motor_rail_on();
    valve_goto(VALVE_OPEN_DEG, 1.0f, 15000, true);
}

// ── Calibration ──────────────────────────────────────────────────────────────

bool irrigoto_cal_pressure_start(void)
{
    if (s_wcal.state == WCAL_PRESSURE_SCANNING ||
        s_wcal.state == WCAL_NOZZLE_RUNNING) {
        ESP_LOGW(TAG, "cal_pressure_start ignored — calibration already running");
        return false;
    }
    INFO("HA cal_pressure_start");
    wcal_reset();
    xTaskCreatePinnedToCore(wcal_pressure_task, "wcal_pres", 12288, NULL, 8, NULL, APP_CPU_NUM);
    return true;
}

bool irrigoto_cal_pressure_throw(float throw_mm)
{
    if (s_wcal.state != WCAL_PRESSURE_AWAIT_THROW) {
        ESP_LOGW(TAG, "cal_pressure_throw ignored — not awaiting throw (state=%d)",
                 (int)s_wcal.state);
        return false;
    }
    if (throw_mm < 100.0f || throw_mm > 15000.0f) {
        ESP_LOGW(TAG, "cal_pressure_throw: %.0f mm out of range (100-15000)", throw_mm);
        return false;
    }
    INFO("HA cal_pressure_throw: %.0f mm", throw_mm);
    // Mirror cal_pressure_throw_handler's logic: take a fresh pressure
    // reading at the moment of throw entry, so it's time-correlated with
    // what the user just observed at the sprinkler.
    pressure_map_t *pm = &s_wcal.pmap;
    float psi_now = cal_read_pressure_avg();
    s_wcal.pmap_max_throw = throw_mm;
    // b385: two-anchor fit. Degenerates to the legacy proportional ray when no
    // low anchor was captured (pmap_low_psi/throw both 0), so the HA path stays
    // functional even though it has no low-throw service wired.
    cal_apply_two_anchor_throw(pm,
        s_wcal.pmap_low_psi, s_wcal.pmap_low_throw,
        psi_now,             throw_mm);
    INFO("HA cal two-anchor: lo(%.3fPSI,%.0fmm) hi(%.3fPSI,%.0fmm)",
        s_wcal.pmap_low_psi, s_wcal.pmap_low_throw, psi_now, throw_mm);
    esp_err_t _r = cal_save_primary(pm);
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();
    sensor_rail_off();
    if (_r != ESP_OK) {
        ESP_LOGE(TAG, "HA cal_pressure_throw: save FAILED: %s", esp_err_to_name(_r));
        snprintf(s_wcal.msg, sizeof(s_wcal.msg),
                 "FAILED to save: %s", esp_err_to_name(_r));
        s_wcal.state = WCAL_IDLE;
        return false;
    }
    snprintf(s_wcal.msg, sizeof(s_wcal.msg),
             "Saved. Max throw %.0fmm (%.1fft), %d points.",
             throw_mm, throw_mm / 304.8f, pm->num_points);
    s_wcal.state = WCAL_DONE;
    return true;
}

bool irrigoto_cal_pressure_cancel(void)
{
    INFO("HA cal_pressure_cancel");
    s_wcal.state = WCAL_IDLE;
    valve_goto(VALVE_CLOSED_DEG, 2.0f, 10000, false);
    motor_rail_off();
    sensor_rail_off();
    snprintf(s_wcal.msg, sizeof(s_wcal.msg), "Cancelled.");
    return true;
}

bool irrigoto_cal_nozzle_start(void)
{
    if (s_wcal.state == WCAL_PRESSURE_SCANNING ||
        s_wcal.state == WCAL_NOZZLE_RUNNING) {
        ESP_LOGW(TAG, "cal_nozzle_start ignored — calibration already running");
        return false;
    }
    INFO("HA cal_nozzle_start");
    wcal_reset();
    xTaskCreatePinnedToCore(wcal_nozzle_task, "wcal_nozzle", 8192, NULL, 8, NULL, APP_CPU_NUM);
    return true;
}

// Numeric cal state. Matches the wcal_state_t enum (kept opaque to C++).
// 0=Idle, 1=PressureScan, 2=AwaitingThrow, 3=NozzleScan,
// 4=JogRunning, 5=Done, 6=Error.
int irrigoto_cal_state_code(void)
{
    return (int)s_wcal.state;
}

void irrigoto_cal_state_str(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    const char *s = "Unknown";
    switch (s_wcal.state) {
        case WCAL_IDLE:                 s = "Idle"; break;
        case WCAL_PRESSURE_SCANNING:    s = "Pressure scanning"; break;
        case WCAL_PRESSURE_AWAIT_THROW: s = "Awaiting throw measurement"; break;
        case WCAL_NOZZLE_RUNNING:       s = "Nozzle calibration"; break;
        case WCAL_DONE:                 s = "Done"; break;
    }
    // Include the cal_status message if present for context.
    if (s_wcal.msg[0])
        snprintf(buf, len, "%s — %s", s, s_wcal.msg);
    else
        snprintf(buf, len, "%s", s);
}

float irrigoto_get_valve_deg(void)
{
    sensor_rail_on();
    uint16_t raw;
    if (!as5600_read(ADDR_AS5600L, &raw, NULL, NULL)) return -1.0f;
    float deg = raw * (360.0f / 4096.0f);
    s_valve_deg_cached = deg;
    return deg;
}

// Confirmed valve open/closed state from the cached angle (no I2C / rail power).
// "Open" = any non-closed position: more than 8 deg off the closed seat (231 deg),
// covering the fully-open 308 deg target and every intermediate watering throw.
// Returns 1=open, 0=closed, -1=unknown (no move/read since boot).
int irrigoto_valve_is_open(void)
{
    if (s_valve_deg_cached < 0.0f) return -1;
    return (s_valve_deg_cached > VALVE_CLOSED_DEG + 8.0f) ? 1 : 0;
}

float irrigoto_get_nozzle_deg(void)
{
    sensor_rail_on();
    uint16_t raw;
    if (!as5600_read(ADDR_AS5600, &raw, NULL, NULL)) return -1.0f;
    return raw * (360.0f / 4096.0f);
}

void irrigoto_set_led(const char *color)
{
    if (!color) return;
    static const struct { const char *name; uint8_t val; } map[] = {
        {"off",    LED_OFF},    {"red",    LED_RED},
        {"green",  LED_GREEN},  {"blue",   LED_BLUE},
        {"yellow", LED_YELLOW}, {"cyan",   LED_CYAN},
        {"purple", LED_PURPLE}, {"white",  LED_WHITE},
    };
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
        if (strcasecmp(color, map[i].name) == 0) {
            tca_led_set(map[i].val);
            INFO("LED set to %s (0x%02X)", map[i].name, map[i].val);
            return;
        }
    }
    ESP_LOGW(TAG, "irrigoto_set_led: unknown color '%s'", color);
}

// b347: one-line summary of the most recent boot_diag entry, sized to fit
// HA's 255-char text_sensor state limit. Empty string if the ring is empty.
void irrigoto_boot_diag_summary(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    buf[0] = '\0';
    boot_diag_blob_t blob;
    boot_diag_load_all(&blob);
    if (blob.count == 0) return;
    int last = ((int)blob.next_idx - 1 + BOOT_DIAG_RING_N) % BOOT_DIAG_RING_N;
    const boot_diag_t *e = &blob.ring[last];
    const char *rstr = boot_diag_reset_reason_str(e->reset_reason);

    // Compose the headline. Layout chosen so the most-actionable signals
    // (close-attempted? close-ok? big valve err?) are visible without
    // scrolling past low-priority context.
    if (e->flags & BOOT_DIAG_FLAG_BATT_TOO_LOW) {
        snprintf(buf, len,
                 "b%u #%u %s batt=%.2fV LOW -> quiet sleep",
                 (unsigned)e->fw_build, (unsigned)e->boot_seq,
                 rstr, e->batt_v_at_boot);
        return;
    }
    if (e->flags & BOOT_DIAG_FLAG_SENSOR_READ_FAILED) {
        snprintf(buf, len,
                 "b%u #%u %s batt=%.2fV sensor read FAILED",
                 (unsigned)e->fw_build, (unsigned)e->boot_seq,
                 rstr, e->batt_v_at_boot);
        return;
    }
    if (e->flags & BOOT_DIAG_FLAG_BATT_SKIPPED_VALVE) {
        snprintf(buf, len,
                 "b%u #%u %s batt=%.2fV (skipped valve check)",
                 (unsigned)e->fw_build, (unsigned)e->boot_seq,
                 rstr, e->batt_v_at_boot);
        return;
    }
    if (e->flags & BOOT_DIAG_FLAG_CLOSE_ATTEMPTED) {
        const char *ok = (e->flags & BOOT_DIAG_FLAG_CLOSE_OK) ? "ok" : "FAILED";
        snprintf(buf, len,
                 "b%u #%u %s batt=%.2fV valve=%.1f° err=%+.1f° -> drove %s post=%.1f° sag=%.2fV",
                 (unsigned)e->fw_build, (unsigned)e->boot_seq,
                 rstr, e->batt_v_at_boot,
                 e->valve_deg_at_boot, e->valve_err_at_boot,
                 ok, e->valve_deg_after_close, e->batt_v_after_close);
        return;
    }
    // Otherwise: valve was already within tolerance — clean wake.
    snprintf(buf, len,
             "b%u #%u %s batt=%.2fV valve=%.1f° (closed, err=%+.1f°)",
             (unsigned)e->fw_build, (unsigned)e->boot_seq,
             rstr, e->batt_v_at_boot,
             e->valve_deg_at_boot, e->valve_err_at_boot);
}

#endif /* ESPHOME_COMPONENT */
