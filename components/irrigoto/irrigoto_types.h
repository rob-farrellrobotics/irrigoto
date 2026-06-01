#pragma once
/*
 * irrigoto_types.h -- shared struct definitions used by both irrigoto.c and storage.c
 */
#include <stdint.h>

#define SPD_MAX_POINTS       16
#define CAL_MAX_POINTS       48
#define ZONE_MAX_PERIM_POINTS 36
#define WATER_RUN_MAX_RINGS  36

typedef struct {
    uint8_t  num_points;
    uint16_t duty[SPD_MAX_POINTS];
    float    deg_per_sec[SPD_MAX_POINTS];
    uint8_t  num_points_ccw;
    uint16_t duty_ccw[SPD_MAX_POINTS];
    float    deg_per_sec_ccw[SPD_MAX_POINTS];
    float    min_continuous_dps;
    uint16_t jog_pulse_duty;
    uint16_t jog_pulse_ms;
    float    jog_deg_per_pulse;
    uint8_t  use_bump_stop;
} speed_map_t;

typedef struct {
    uint8_t  num_points;
    float    valve_deg[CAL_MAX_POINTS];
    float    pressure_psi[CAL_MAX_POINTS];
    float    throw_mm[CAL_MAX_POINTS];
} pressure_map_t;

typedef struct {
    float   nozzle_deg;
    float   valve_deg;
    float   pressure_psi;
    float   throw_mm;
    uint8_t walk_idx;
} perimeter_point_t;

typedef struct {
    uint8_t           num_points;
    perimeter_point_t points[ZONE_MAX_PERIM_POINTS];
} zone_perimeter_t;

typedef struct {
    float throw_mm;
    float avg_psi;
    float actual_throw_mm;
    float dps;
    float active_deg;
    float arc_start_deg;   // actual CW sweep start for this ring
    float arc_end_deg;     // actual CW sweep end for this ring
    float depth_mm;        // estimated precipitation depth per pass [mm]
    float valve_deg;       // b281: valve angle held for this ring (for supply back-calc)
    float supply_psi_est;  // b281: cal-back-computed supply pressure during this ring
                           //       (avg_psi / f(valve_deg)); 0.0 if cal insufficient.
                           //       Diagnoses pump-supply dynamics on well-fed systems.
} water_ring_data_t;

typedef struct {
    uint16_t          fw_build;
    uint16_t          num_rings;
    float             arc_start_deg;  // zone arc CW start bearing
    float             arc_span_deg;   // zone arc total span
    float             total_time_s;   // actual wall-clock run duration (0 if unknown/aborted)
    // b281: run-level supply pressure summary (back-computed from per-ring
    // avg_psi using f(valve_deg) extracted from zone cal data). Lets HA
    // dashboards show pump-supply dynamics without needing a flow meter.
    float             supply_psi_min; // minimum per-ring supply estimate across run
    float             supply_psi_max; // maximum per-ring supply estimate
    float             supply_psi_avg; // mean per-ring supply estimate (active rings only)
    // b282: gap analysis -- count of rings flagged as supply-limited:
    // under-threw their target AND were operating at a supply pressure
    // measurably below cal-time supply. These are the rings that b283
    // would re-water once supply recovers.
    uint8_t           rings_supply_limited;
    // Target depth this run aimed for, in mm (N x 1/8" = N x 3.175). Lets the
    // heatmap card normalize actual-vs-expected to the chosen depth instead of
    // a fixed 1/8". 0 when loading a pre-b388 file (card falls back to config).
    float             target_depth_mm;
    water_ring_data_t rings[WATER_RUN_MAX_RINGS];
} water_run_t;

// ── Watering schedule ────────────────────────────────────────────────────────
// One entry per "fire this zone at this time on these days" rule. Stored in
// NVS as a single blob; pushed/queried via HA service or web UI.
//
// b355: schema grew from 7 bytes/entry → 16 bytes/entry to support bidirectional
// HA<->device sync (docs/schedule_sync_design.md):
//   - id          : sequential, stable across edits, assigned at creation.
//                   0 is reserved for "unassigned" on the wire — the device
//                   allocates on first sync. Persisted with the entry.
//   - last_modified : unix epoch when this entry was last edited. LWW conflict
//                   resolution uses this; device wins on tie.
//   - source      : 0=unknown, 1=user_device, 2=user_ha, 3=algorithm. No
//                   behavior wired up yet — informational only.
//
// Field order is chosen so the struct packs to 16 bytes on ESP32 (4-byte
// aligned uint32_t members first, then the existing uint8_t cluster).
// Old NVS blobs (7 bytes/entry) are auto-migrated on first boot — see
// schedule_load_nvs() in irrigoto.c.
#define SCHEDULE_MAX_ENTRIES 32
typedef struct {
    uint32_t id;            // 1-based, stable across edits. 0 = unassigned (wire only).
    uint32_t last_modified; // unix epoch of last edit (0 if never set)
    uint8_t  source;        // 0=unknown, 1=user_device, 2=user_ha, 3=algorithm
    uint8_t  zone;          // 1-based zone number (0 = empty/unused slot)
    uint8_t  mode;          // 0=Pulse, 1=Gentle, 2=Smooth
    uint8_t  depth;         // depth target in eighths of an inch (1..8 = 1/8".. 1";
                            //   each eighth = 3.175 mm). Migrated from the old
                            //   0/1 flag on first boot (see schedule_load_nvs).
    uint8_t  hour;          // 0-23 local
    uint8_t  minute;        // 0-59
    uint8_t  days_mask;     // bit0=Sun, bit1=Mon, ..., bit6=Sat (matches tm_wday)
    uint8_t  enabled;       // 0/1 — disabled entries stay in storage but don't fire
    uint8_t  _pad;          // explicit padding so sizeof() is a stable 16
} schedule_entry_t;

typedef struct {
    uint8_t          count;
    schedule_entry_t entries[SCHEDULE_MAX_ENTRIES];
} schedule_t;

// Old-schema entry as it lived in NVS pre-b355. Kept here so the migration
// path in schedule_load_nvs() can recognize the legacy blob size and
// promote each entry into the new schema with sequential IDs.
typedef struct {
    uint8_t zone;
    uint8_t mode;
    uint8_t depth;
    uint8_t hour;
    uint8_t minute;
    uint8_t days_mask;
    uint8_t enabled;
} schedule_entry_v1_t;

typedef struct {
    uint8_t              count;
    schedule_entry_v1_t  entries[SCHEDULE_MAX_ENTRIES];
} schedule_v1_t;
