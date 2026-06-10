#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// ─────────────────────────────────────────────────────────────────────────────
// irrigoto_api.h  —  C interface between the ESPHome C++ component and the
//                    irrigoto firmware (irrigoto.c).
//
// All functions are implemented in main/irrigoto.c, compiled with C linkage.
// The #ifdef ESPHOME_COMPONENT guard in irrigoto.c activates this API and
// deactivates app_main() / wifi_init() so ESPHome can own those.
// ─────────────────────────────────────────────────────────────────────────────

#ifdef __cplusplus
extern "C" {
#endif

// ── Initialisation ────────────────────────────────────────────────────────────

/**
 * irrigoto_init() — replaces app_main() when built as an ESPHome component.
 *
 * Does everything app_main() does EXCEPT:
 *   - nvs_flash_init()   (ESPHome calls this before components start)
 *   - wifi_init()        (ESPHome owns WiFi)
 *   - esp_event_loop_create_default()  (ESPHome creates it)
 *
 * Must be called from the ESPHome component setup().
 */
void irrigoto_init(void);

// ── State getters (polled every ~2 s by the component loop) ──────────────────

/** Live line pressure from MPRLS sensor, PSI.  Returns 0 if not readable. */
float irrigoto_get_pressure_psi(void);

/** Battery pack voltage in millivolts. */
uint32_t irrigoto_get_battery_mv(void);

/** Current throw distance in mm (valve-derived or live). 0 when idle. */
float irrigoto_get_throw_mm(void);

/** True while a watering run is active. */
bool irrigoto_is_watering(void);

/** Estimated minutes remaining for the current watering run.
 *  Updated continuously from the watering loop based on remaining
 *  arc/depth to deliver. 0 when not watering. */
int irrigoto_get_water_minutes_remaining(void);

// b287: Watering Quiet mode -- true while a smooth/gentle run is in flight
// (and for a brief settle period after the final LFS write). Returning true
// asks the ESPHome loop() to throttle high-rate sensor publishes, keeping
// WiFi/NVS traffic minimal so it doesn't race with SPI flash operations.
// Progress sensors (watering, status, zone) keep publishing -- only the
// noisy ones (pressure, battery, throw, cal_state) get gated.
bool irrigoto_is_watering_quiet(void);

/** 1-based zone index of the currently selected / active zone. */
int irrigoto_get_zone(void);

/** Water mode: 0 = Pulse, 1 = Gentle, 2 = Smooth. */
int irrigoto_get_mode(void);

/**
 * Human-readable status string (e.g. "Idle", "Watering zone 2 smooth").
 * buf must be at least 64 bytes.
 */
void irrigoto_get_status(char *buf, size_t len);

/**
 * Name of the currently selected zone (from NVS config), e.g. "Front lawn".
 * buf must be at least 32 bytes.
 */
void irrigoto_get_zone_name(char *buf, size_t len);

// ── Last-watering-run summary (used by the HA completion event) ─────────────
// All values reflect the most-recently-completed watering pass; cleared at
// the start of each new run. Metric, with explicit units in the field name.
int   irrigoto_last_water_zone(void);                       // 1-based zone id
int   irrigoto_last_water_mode_code(void);                  // 0=idle,1-4=pulse,5-6=gentle,7=smooth
int   irrigoto_last_water_num_rings(void);
float irrigoto_last_water_duration_s(void);
float irrigoto_last_water_avg_psi(void);
float irrigoto_last_water_max_throw_mm(void);
float irrigoto_last_water_total_depth_mm(void);
float irrigoto_last_water_area_m2(void);            // sum of per-ring sector areas
float irrigoto_last_water_volume_l(void);           // depth * area, summed
float irrigoto_last_water_zone_area_m2(void);       // polygon area of the zone (intended)
// Target depth requested by the run (1/8" = 3.175mm, 1/4" = 6.35mm). Comes from
// the mode (smooth always 1/8"; pulse/gentle from depth selector). Lets HA flag
// runs that fell short of target.
float irrigoto_last_water_target_depth_mm(void);
// Average depth actually delivered over the zone polygon (volume_l / zone_area_m2).
// 1 L on 1 m² = 1 mm depth, so this is the headline "did we hit target?" number.
// Returns 0 if zone_area_m2 unavailable.
float irrigoto_last_water_actual_avg_depth_mm(void);
float irrigoto_last_water_score(void);              // 0..5; F1 * depth_factor * 5
// "True" coverage: fraction (0..100) of the zone polygon that received spray,
// measured by rasterizing the polygon and counting cells inside∩sprayed
// vs inside∩missed. Excludes over-spray past the polygon (which shows up in
// the HA spray_spread sensor instead). The honest answer to "did my zone get
// watered" — pair with spray_spread for the full picture.
float irrigoto_last_water_polygon_coverage_pct(void);

// b281: Supply pressure summary for the last completed run. These are
// pre-valve supply pressure estimates (PSI), back-computed from each ring's
// avg_psi using the zone perimeter cal data's f(valve_deg) transmission
// function. On well-fed systems they reveal pump cycling: a wide max-min
// spread means supply was actively cycling; a low avg means the cell
// couldn't keep up with watering draw. All three return 0.0 if the zone
// has insufficient cal data to back-compute supply.
float irrigoto_last_water_supply_psi_min(void);
float irrigoto_last_water_supply_psi_max(void);
float irrigoto_last_water_supply_psi_avg(void);

// b282: Count of rings flagged as supply-limited after the last run
// (under-threw their target AND were operating at supply pressure
// noticeably below cal-time supply). These are candidates for the
// pressure-gated cleanup pass (b283). Returns 0 if no rings were
// flagged or if cal data was insufficient to make the determination.
int irrigoto_last_water_rings_supply_limited(void);

bool  irrigoto_last_water_aborted(void);
void  irrigoto_last_water_zone_name(char *buf, size_t len);
void  irrigoto_last_water_mode_label(char *buf, size_t len);

// Granular completion status. Codes:
//   0=completed, 1=cancelled, 2=valve_fault, 3=nozzle_fault, 4=water_loss
// status_str fills buf with the matching string ("completed"/etc).
int  irrigoto_last_water_status_code(void);
void irrigoto_last_water_status_str(char *buf, size_t len);

// Unix epoch when the last watering ended. 0 if no watering has
// completed since boot. Used by web/HA surfaces that want to render
// "last watered N minutes ago".
time_t irrigoto_last_water_finish_epoch(void);

/**
 * Look up the Nth configured zone (0-based idx) in storage.
 *
 * Returns the zone's 1-based number (1..STORAGE_MAX_ZONES) and fills
 * name_buf with its configured name; falls back to "Zone N" if the
 * name slot is empty.  Returns -1 if idx is out of range or storage
 * isn't ready yet — caller can iterate idx = 0, 1, 2, ... until -1.
 *
 * Used by the HA select entity to surface real zone names instead of
 * generic "Zone 1..4" labels, and to skip unconfigured slots entirely.
 */
int irrigoto_zone_at(int idx, char *name_buf, size_t name_len);

// ── Command setters ───────────────────────────────────────────────────────────

/**
 * Start a watering run.
 * zone     : 1-based zone index
 * mode     : 0=Pulse, 1=Gentle, 2=Smooth
 * duration : 0 = use the zone's configured duration
 */
void irrigoto_start_watering(int zone, int mode, int duration_s);

/** Abort any in-progress watering run. */
void irrigoto_stop_watering(void);

/** Change the selected zone without starting water. */
void irrigoto_set_zone(int zone);

/** Change the water mode without starting water. */
void irrigoto_set_mode(int mode);

// ── Diagnostics ───────────────────────────────────────────────────────────────

/** One-shot live MPRLS pressure read in PSI; returns -1.0f on I2C failure. */
float irrigoto_read_pressure_now(void);

/**
 * Set the TCA6408A RGB LED to a named color. Case-insensitive.
 * Accepted: "off", "red", "green", "blue", "yellow", "cyan", "purple", "white".
 * Unknown colors are logged and ignored.
 */
void irrigoto_set_led(const char *color);

/**
 * Drive the valve / nozzle motor to a target angle (0-360 degrees).
 * Returns true on success, false on timeout/failure. Uses default
 * tolerance (2.0 deg) and timeout (10 s).
 */
bool irrigoto_valve_goto(float target_deg);
bool irrigoto_nozzle_goto(float target_deg);

/**
 * Aim the stream at a position by moving NOZZLE bearing and VALVE
 * simultaneously. Both motors run in parallel via high-PWM PID-style
 * control with cosine-eased decel near the target. Refused if a watering
 * run is in progress.
 *
 * Optimised for speed (~1-2 s typical for small moves) rather than
 * accuracy: tolerance is ~1.5 deg bearing / ~1.0 deg valve, no backlash
 * compensation. Use the (slower) valve_goto / nozzle_goto if you need
 * sub-degree positioning.
 *
 * aim_valve : valve target in raw valve angle (231..308 deg, clamped).
 * aim_throw : valve target derived from throw distance via the pressure
 *             cal table (cal_throw_to_valve_deg).
 *
 * Returns true if BOTH axes reached tolerance before the 8-s timeout.
 */
bool irrigoto_aim_valve(float bearing_deg, float valve_deg);
bool irrigoto_aim_throw(float bearing_deg, float throw_mm);

/**
 * Water a single arc: parallel-position to (arc_start_deg, throw), then
 * sweep the nozzle to arc_end_deg at constant angular velocity. Valve
 * stays at the throw position for the whole sweep AND IS LEFT OPEN at
 * the end -- caller chains arcs by calling again, or stops the flow via
 * close_valve / stop_and_close.
 *
 * direction: "cw"       -- increasing bearing (may take the long way)
 *            "ccw"      -- decreasing bearing (may take the long way)
 *            "shortest" -- shortest path; same effect as "" or NULL
 *
 * speed_dps clamped to [1, 25]. Speeds below ~2.5 dps may stutter due to
 * the motor's stall band.
 *
 * Refused if a watering run is in progress. Returns true if the arc end
 * was reached within tolerance before timeout (2x expected, max 90 s).
 */
bool irrigoto_water_arc(float throw_mm, float arc_start_deg,
                        float arc_end_deg, float speed_dps,
                        const char *direction);

/**
 * b417: glide the stream around a stored zone's perimeter in ONE
 * continuous motion. Walks the zone's points in walk_idx order: the
 * nozzle sweeps each leg at constant angular speed while the valve
 * chases a throw target interpolated along the leg's ACTUAL sweep
 * progress, so the landing point tracks the polygon edge instead of
 * stepping radius at each vertex. Same-direction vertices are carried
 * through without stopping; the sweep only brakes where the walk
 * physically reverses (or goes radial).
 *
 * Spawns a task and returns immediately (same pattern as water_arc).
 * Dry-positions to the first point (valve held), opens to the first
 * point's throw, glides, then closes the valve and powers the rails
 * down when finished or stopped. speed_dps clamped to [1, 25]; the
 * motor duty floor makes the real minimum ~5 dps.
 *
 * Refused while watering / calibrating / another trace or arc runs.
 */
bool irrigoto_zone_trace_start(uint16_t zone_id, float speed_dps,
                               bool close_loop);

/** Abort an active perimeter glide; the trace task closes the valve. */
void irrigoto_zone_trace_stop(void);

/**
 * Emergency stop: abort any active watering, wait for the watering
 * loop to release the motors, then drive the valve to the closed
 * position. Safe to call at any time.
 */
void irrigoto_stop_and_close(void);

/** Drive the valve to VALVE_CLOSED_DEG (231 deg). */
void irrigoto_valve_close(void);

/** Drive the valve to VALVE_OPEN_DEG (308 deg, pressure peak). */
void irrigoto_valve_open(void);

// ── Calibration ──────────────────────────────────────────────────────────────

/**
 * Pressure-curve calibration. Same state machine the web UI uses.
 *
 * Flow:
 *   cal_pressure_start()             -> WCAL_PRESSURE_SCANNING (background task)
 *   ...wait for state to become WCAL_PRESSURE_AWAIT_THROW...
 *   (user physically measures throw distance with a tape measure)
 *   cal_pressure_throw(throw_mm)     -> records & saves, state = WCAL_DONE
 *
 * Or at any time: cal_pressure_cancel() -> WCAL_IDLE.
 *
 * Returns false if calibration is already running.
 */
bool irrigoto_cal_pressure_start(void);
bool irrigoto_cal_pressure_throw(float throw_mm);
bool irrigoto_cal_pressure_cancel(void);

/** Start the nozzle calibration sequence (background task). */
bool irrigoto_cal_nozzle_start(void);

/** Current calibration state as a short string for HA text_sensor. */
void irrigoto_cal_state_str(char *buf, size_t len);

/**
 * Numeric calibration state, useful for transition detection without
 * parsing strings:
 *   0 = Idle
 *   1 = PressureScanning
 *   2 = AwaitingThrow
 *   3 = NozzleRunning
 *   4 = JogRunning
 *   5 = Done
 *   6 = Error
 */
int irrigoto_cal_state_code(void);

/**
 * Read current AS5600L / AS5600 raw angles in degrees (0-360).
 * Returns -1.0f on I2C failure.
 */
float irrigoto_get_valve_deg(void);
float irrigoto_get_nozzle_deg(void);

/** Confirmed valve open/closed state from the cached angle (no I2C, no rail
 * power). "Open" = any non-closed position (>8 deg off the 231 deg seat).
 * Returns 1=open, 0=closed, -1=unknown (no move/read since boot). */
int irrigoto_valve_is_open(void);

// ── Power management ──────────────────────────────────────────────────────────

/**
 * Enable or disable the 5-minute inactivity auto-deep-sleep.
 * Default at boot: enabled (preserves standalone behaviour).
 * Disable when the device should stay HA-reachable indefinitely.
 */
void irrigoto_set_auto_sleep_enabled(bool enabled);

/** True if auto sleep is currently enabled. */
bool irrigoto_get_auto_sleep_enabled(void);

/**
 * Request a deep sleep with timer wake after `duration_s` seconds.
 * duration_s = 0 -> use the configured default (irrigoto_get_sleep_duration_s).
 *
 * Performs the hardware safe-state prep (valve closed, rails off, LED blank)
 * synchronously, then sets a flag the ESPHome component's loop() consumes.
 * The component then hands off to ESPHome's deep_sleep component, which
 * notifies HA before actually sleeping (so HA keeps last-known states
 * instead of marking the device unavailable).
 *
 * Returns immediately — the actual sleep happens on the ESPHome task tick.
 *
 * irrigoto_sleep_now() stamps reason="manual"; the _with_reason variant
 * is for internal callers (e.g. inactivity watchdog) to label why the
 * device chose to sleep — surfaced to HA via the last-sleep-reason
 * text_sensor on the next wake.
 */
void irrigoto_sleep_now(uint32_t duration_s);
void irrigoto_sleep_now_with_reason(uint32_t duration_s, const char *reason);

// ── Persistent power-management settings (NVS-backed) ───────────────────────

/** Inactivity threshold before auto-sleep, minutes (1–60). */
uint32_t irrigoto_get_inactivity_minutes(void);
void     irrigoto_set_inactivity_minutes(uint32_t minutes);

/** Sleep duration on auto-sleep wake, seconds (30–3600). */
uint32_t irrigoto_get_sleep_duration_s(void);
void     irrigoto_set_sleep_duration_s(uint32_t seconds);

/**
 * Nozzle dwell-watchdog timeout, seconds (10–300, default 30).
 * Hard ceiling on how long the nozzle can sit within ~3° of one bearing
 * with the valve open before nozzle_fault is declared and the valve is
 * slammed closed. Backstop for the stall+current fault detection.
 */
uint32_t irrigoto_get_dwell_timeout_s(void);
void     irrigoto_set_dwell_timeout_s(uint32_t seconds);

/**
 * Last sleep reason recorded before deep sleep, restored from NVS on boot.
 * Values: "inactivity", "manual", "low battery", "none" (never slept yet).
 * buf must be at least 32 bytes.
 */
void irrigoto_get_last_sleep_reason(char *buf, size_t len);

/**
 * b347: one-line summary of the most recent boot_diag entry (NVS-persisted
 * record of what the boot-time battery + valve checks observed). Sized to
 * fit HA's 255-char text_sensor state limit. Empty string if no boot has
 * been captured yet. Full per-entry detail is available via GET
 * /api/boot_diag on the device's HTTP server.
 *
 * buf should be at least 192 bytes.
 */
void irrigoto_boot_diag_summary(char *buf, size_t len);

// ── Watering schedule (device-owned, NVS-persisted) ─────────────────────────
//
// The device runs its own schedule autonomously. Two push paths:
//
//   irrigoto_schedule_set_text()  : LEGACY REPLACE-ALL. The text payload is
//        a complete schedule; the device drops what it had and installs the
//        new list. Used by older HA pushers and the web-UI editor. Each
//        installed entry is stamped with a fresh ID (assigned from
//        schedule_id_next) and last_modified=time(NULL), so the new schema
//        keeps working even when the legacy push path is used. Deprecated
//        in favor of irrigoto_schedule_sync_text() — kept for compatibility.
//
//   irrigoto_schedule_sync_text() : NEW MERGE / LWW. The payload is a list of
//        per-entry deltas. For each incoming entry the device:
//          - id=0          → allocate a new ID, insert
//          - id matches, !tombstone → upsert with last-writer-wins
//                                     (incoming wins strictly newer than
//                                     stored; device wins on tie)
//          - id matches, tombstone=1 → delete
//          - id unknown, !tombstone → insert with the supplied ID (the
//                                     device tracks this so future syncs
//                                     can reference it)
//          - id unknown, tombstone   → no-op
//        Entries the device has but HA didn't mention are preserved — so
//        device-only entries (added via the web UI before HA learned of them)
//        survive a sync push. Any actual change increments schedule_version.
//
// Legacy text format (set_text):
//   zone,mode,depth,hour,minute,days_mask,enabled
//
// Sync text format (sync_text), one entry per ';' or newline:
//   id,last_modified,zone,mode,depth,hour,minute,days_mask,enabled,source,tombstone
//
// Returns false if any entry fails to parse or is out of range. Prior schedule
// is preserved on parse failure.
bool irrigoto_schedule_set_text(const char *text);
bool irrigoto_schedule_sync_text(const char *text);

/** Serialize the current schedule into the legacy 7-field text format
 *  (no IDs / timestamps). Used by the web UI's /api/schedule endpoint
 *  which has its own JSON path for the full structured data. */
void irrigoto_schedule_get_text(char *buf, size_t len);

/** Serialize the current schedule including per-entry id + last_modified +
 *  source. Format (one entry per ';'):
 *    id,last_modified,zone,mode,depth,hour,minute,days_mask,enabled,source
 *  Used by the HA schedule_text text_sensor and the schedule_changed event
 *  payload so HA can reconcile by ID without polling /api/schedule. */
void irrigoto_schedule_get_text_full(char *buf, size_t len);

/** Wipe all entries (and persist the empty table). Increments
 *  schedule_version. */
void irrigoto_schedule_clear(void);

/** Number of entries currently in the schedule (enabled or not). */
int irrigoto_schedule_count(void);

/** Device-wide monotonic counter, persisted in NVS. Increments every time
 *  the schedule is mutated (set_text, sync_text, clear, delay-changes do
 *  NOT count). Surfaced to HA via a text_sensor so HA can compare its
 *  last_seen_version and pull on mismatch without event-replay logic. */
uint32_t irrigoto_schedule_version(void);

/**
 * Outcome of the most recent set_schedule call. Useful for surfacing in
 * HA so users can see at a glance whether their push succeeded or got
 * rejected (and why). Values look like:
 *   "ok (3 entries)"
 *   "parse failed at \"1,0,..\" (expected 7 fields)"
 *   "rejected: entry 0 ... overlaps entry 1 ..."
 *   "no schedule pushed yet" (initial state)
 */
void irrigoto_schedule_get_last_status(char *buf, size_t len);

/**
 * Find the soonest upcoming firing across the whole schedule.
 * Walks each entry forward up to 7 days.  Returns true and fills
 * *out_t (unix epoch) plus *out_zone when something is upcoming;
 * false if the schedule is empty / all disabled / time not synced.
 * Entries that would fire during an active delay window are skipped —
 * the result is always the next *actual* firing the executor will do.
 */
#include <time.h>
bool irrigoto_schedule_next_run(time_t now, time_t *out_t, int *out_zone);

/**
 * Rain / wind delay — suspend all schedule firing until a specific time
 * (or for N hours from now). Used by automations that want to skip the
 * next watering because of weather, manual override, etc., without
 * editing or losing the schedule.
 *
 * irrigoto_schedule_set_delay_until(t):
 *   Suspend firing until the given epoch. t <= now (or 0) cancels any
 *   active delay. NVS-persisted, survives reboot / deep-sleep wake.
 *
 * irrigoto_schedule_set_delay_hours(h):
 *   Convenience wrapper. h == 0 cancels; otherwise delays for h hours
 *   from current device time. No-op if device time hasn't been synced.
 *
 * irrigoto_schedule_clear_delay():
 *   Resume the schedule immediately (same as set_delay_hours(0)).
 *
 * irrigoto_schedule_get_delay_until():
 *   Returns the absolute delay-until epoch, or 0 if no delay is active
 *   (which includes expired delays — they self-clear lazily on read).
 */
void   irrigoto_schedule_set_delay_until(time_t t);
void   irrigoto_schedule_set_delay_hours(uint32_t hours);
void   irrigoto_schedule_clear_delay(void);
time_t irrigoto_schedule_get_delay_until(void);

/** Web UI theme. true=dark (legacy), false=light. Persists to NVS. */
bool irrigoto_get_theme_dark(void);
void irrigoto_set_theme_dark(bool dark);

/**
 * Polled by the ESPHome component each loop iteration. Returns true and
 * fills *duration_s_out if a sleep was requested via irrigoto_sleep_now()
 * or by the inactivity watchdog. Caller should clear via
 * irrigoto_sleep_request_clear() once handled.
 */
bool irrigoto_sleep_request_pending(uint32_t *duration_s_out);

/** Clear the pending sleep request (called after begin_sleep is invoked). */
void irrigoto_sleep_request_clear(void);

#ifdef __cplusplus
}
#endif
