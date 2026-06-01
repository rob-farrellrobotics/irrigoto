#pragma once
/*
 * storage.h -- LittleFS-backed persistent storage (JSON format).
 *
 * Filesystem layout:
 *   /lfs/zones/zone_000_front_yard.json  -- zone perimeter (name embedded in filename)
 *   /lfs/cal/pressure.json               -- pressure->throw calibration
 *   /lfs/cal/speed.json                  -- nozzle speed calibration
 *   /lfs/water/zone_000_front_yard.json  -- last watering run (matches zone filename)
 *   /lfs/logs/YYYYMMDD.log               -- rotating daily log
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "irrigoto_types.h"

/* ---- Init ---- */
esp_err_t storage_init(void);
void      storage_deinit(void);
bool      storage_ready(void);

/* (Re-)create the four top-level dirs (/lfs/zones, /lfs/cal,
 * /lfs/water, /lfs/logs). Idempotent. Call this before any direct
 * fopen() from outside storage.c. */
void      storage_ensure_dirs(void);

/* ---- Zones ---- */
#define STORAGE_MAX_ZONES 100
esp_err_t storage_zone_list(uint16_t *ids_out, int max, int *count_out);
esp_err_t storage_zone_load(uint16_t id, char *name_out, size_t name_len,
                             zone_perimeter_t *zone_out);
esp_err_t storage_zone_save(uint16_t id, const char *name,
                             const zone_perimeter_t *zone);
esp_err_t storage_zone_delete(uint16_t id);
esp_err_t storage_zone_rename(uint16_t id, const char *name);
/* b398: parse a zone JSON (export/import shape) into a zone_perimeter_t without
 * saving. out_id = JSON "id" if numeric, else -1. out_np = points parsed. */
esp_err_t storage_zone_parse_json(const char *json, int *out_id,
                                  char *name_out, size_t name_len,
                                  zone_perimeter_t *zone_out, int *out_np);

/* ---- Calibration ---- */
esp_err_t storage_cal_save(const pressure_map_t *map);
esp_err_t storage_cal_load(pressure_map_t *map);

/* ---- Speed map ---- */
esp_err_t storage_spd_save(const speed_map_t *map);
esp_err_t storage_spd_load(speed_map_t *map);

/* ---- Water run ---- */
esp_err_t storage_water_save(uint16_t zone_id, const water_run_t *run);
esp_err_t storage_water_load(uint16_t zone_id, water_run_t *run);
esp_err_t storage_water_delete(uint16_t zone_id);

/* ---- Logging ---- */
esp_err_t storage_log(const char *line);

/* ---- Usage ---- */
esp_err_t storage_usage(size_t *used_out, size_t *total_out);

/* ---- Space management ---- */
// Delete water log files for other zones (binary first, then JSON) until
// at least bytes_needed bytes are free.  skip_zone_id is never deleted.
esp_err_t storage_make_room(size_t bytes_needed, uint16_t skip_zone_id);

/* ---- Schedule (kept for future use) ---- */
#define STORAGE_MAX_SCHEDULES 64
typedef struct __attribute__((packed)) {
    uint16_t zone_id;
    uint8_t  days_mask;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  mode;
    uint8_t  enabled;
    uint8_t  _pad;
} storage_schedule_entry_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  version;
    uint8_t  count;
    uint8_t  _pad[2];
    storage_schedule_entry_t entries[STORAGE_MAX_SCHEDULES];
} storage_schedule_t;

esp_err_t storage_schedule_load(storage_schedule_t *out);
esp_err_t storage_schedule_save(const storage_schedule_t *sched);
