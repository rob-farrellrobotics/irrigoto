/*
 * storage.c -- LittleFS-backed persistent storage (JSON format).
 *
 * All data stored as human-readable JSON files under /lfs/:
 *   /lfs/zones/zone_000.json
 *   /lfs/cal/pressure.json
 *   /lfs/cal/speed.json
 *   /lfs/water/water_000.json
 *   /lfs/logs/YYYYMMDD.log
 */

#include "storage.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <math.h>

/* Pull in struct definitions from irrigoto.c via a shared header. */
#include "irrigoto_types.h"
#include "storage.h"

static const char *TAG = "storage";

#define MOUNT_POINT   "/lfs"
#define ZONES_DIR     MOUNT_POINT "/zones"
#define CAL_DIR       MOUNT_POINT "/cal"
#define WATER_DIR     MOUNT_POINT "/water"
#define LOGS_DIR      MOUNT_POINT "/logs"
#define PRES_JSON     CAL_DIR "/pressure.json"
#define SPD_JSON      CAL_DIR "/speed.json"
#define SCHED_JSON    MOUNT_POINT "/schedule.json"
#define MAX_LOG_FILES 14

static bool s_ready = false;

/* ── helpers ───────────────────────────────────────────────────────── */

static void mkdir_p(const char *path)
{
    char tmp[64]; strncpy(tmp, path, sizeof(tmp)-1);
    for (char *p = tmp+1; *p; p++) {
        if (*p == '/') { *p = 0; mkdir(tmp, 0755); *p = '/'; }
    }
    int r = mkdir(tmp, 0755);
    if (r != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "mkdir('%s') failed: errno=%d (%s)",
                 tmp, errno, strerror(errno));
    } else if (r == 0) {
        // Only log on actual creation; "already exists" is the common case
        // and would flood the log on every save.
        ESP_LOGI(TAG, "mkdir_p('%s') created", tmp);
    }
}

/* Convert zone name to a filesystem-safe lowercase slug (max 24 chars). */
static void zone_name_slug(const char *name, char *slug, size_t len)
{
    size_t j = 0;
    for (size_t i = 0; name[i] && j < len - 1; i++) {
        char c = name[i];
        if (c >= 'a' && c <= 'z') { slug[j++] = c; }
        else if (c >= 'A' && c <= 'Z') { slug[j++] = c - 'A' + 'a'; }
        else if (c >= '0' && c <= '9') { slug[j++] = c; }
        else if ((c == ' ' || c == '-' || c == '_') && j > 0 && slug[j-1] != '_') {
            slug[j++] = '_';
        }
    }
    while (j > 0 && slug[j-1] == '_') j--;
    slug[j] = 0;
}

/* Build path for a zone using its name: zone_000_front_yard.json */
static void zone_path_named(uint16_t id, const char *name, char *buf, size_t len)
{
    if (name && name[0]) {
        char slug[25] = {0};
        zone_name_slug(name, slug, sizeof(slug));
        if (slug[0]) { snprintf(buf, len, ZONES_DIR "/zone_%03u_%s.json", id, slug); return; }
    }
    snprintf(buf, len, ZONES_DIR "/zone_%03u.json", id);
}

/* Find existing zone file for id by scanning the directory (handles rename). */
static void zone_path_find(uint16_t id, char *buf, size_t len)
{
    char prefix[12]; snprintf(prefix, sizeof(prefix), "zone_%03u", id);
    DIR *d = opendir(ZONES_DIR);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d))) {
            size_t plen = strlen(prefix);
            if (strncmp(ent->d_name, prefix, plen) == 0 &&
                (ent->d_name[plen] == '.' || ent->d_name[plen] == '_') &&
                strstr(ent->d_name, ".json")) {
                snprintf(buf, len, ZONES_DIR "/%s", ent->d_name);
                closedir(d); return;
            }
        }
        closedir(d);
    }
    snprintf(buf, len, ZONES_DIR "/zone_%03u.json", id);
}

/* Build path for a water run using zone name: zone_000_front_yard.json */
static void water_path_named(uint16_t id, const char *name, char *buf, size_t len)
{
    if (name && name[0]) {
        char slug[25] = {0};
        zone_name_slug(name, slug, sizeof(slug));
        if (slug[0]) { snprintf(buf, len, WATER_DIR "/water_%03u_%s.json", id, slug); return; }
    }
    snprintf(buf, len, WATER_DIR "/water_%03u.json", id);
}

/* Find existing water file for id by scanning the directory. */
static void water_path_find(uint16_t id, char *buf, size_t len)
{
    char prefix[13]; snprintf(prefix, sizeof(prefix), "water_%03u", id);
    DIR *d = opendir(WATER_DIR);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d))) {
            size_t plen = strlen(prefix);
            if (strncmp(ent->d_name, prefix, plen) == 0 &&
                (ent->d_name[plen] == '.' || ent->d_name[plen] == '_') &&
                strstr(ent->d_name, ".json")) {
                snprintf(buf, len, WATER_DIR "/%s", ent->d_name);
                closedir(d); return;
            }
        }
        closedir(d);
    }
    snprintf(buf, len, WATER_DIR "/water_%03u.json", id);
}

/* Read entire file into a malloc'd buffer (caller must free). Returns NULL on error. */
static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long sz = ftell(f); rewind(f);
    if (sz <= 0 || sz > 32768) { fclose(f); return NULL; }
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return NULL; }
    fread(buf, 1, sz, f); fclose(f);
    buf[sz] = 0;
    return buf;
}

/* Minimal JSON helpers -- no external library needed for simple flat/array JSON */

/* Find value after "key": in json, copy up to len-1 chars into out. */
static bool json_get_str(const char *json, const char *key, char *out, size_t len)
{
    char search[64]; snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p == '"') {
        p++;
        size_t i = 0;
        while (*p && *p != '"' && i < len-1) out[i++] = *p++;
        out[i] = 0; return true;
    }
    return false;
}

static bool json_get_int(const char *json, const char *key, int *out)
{
    char search[64]; snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = atoi(p); return true;
}

static bool json_get_float(const char *json, const char *key, float *out)
{
    char search[64]; snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = (float)atof(p); return true;
}

/* Parse a JSON float array: "key":[1.0,2.0,...] into out[], up to max elements */
static int json_get_float_array(const char *json, const char *key, float *out, int max)
{
    char search[64]; snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '[') return 0;
    p++;
    int n = 0;
    while (*p && *p != ']' && n < max) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']') break;
        out[n++] = (float)atof(p);
        while (*p && *p != ',' && *p != ']') p++;
    }
    return n;
}

static int json_get_int_array(const char *json, const char *key, int *out, int max)
{
    char search[64]; snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '[') return 0;
    p++;
    int n = 0;
    while (*p && *p != ']' && n < max) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']') break;
        out[n++] = atoi(p);
        while (*p && *p != ',' && *p != ']') p++;
    }
    return n;
}

/* ── Init ───────────────────────────────────────────────────────────── */

esp_err_t storage_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path              = MOUNT_POINT,
        .partition_label        = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    mkdir_p(ZONES_DIR);
    mkdir_p(CAL_DIR);
    mkdir_p(WATER_DIR);
    mkdir_p(LOGS_DIR);
    s_ready = true;
    size_t used=0, total=0;
    esp_littlefs_info("littlefs", &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    return ESP_OK;
}

void storage_deinit(void)
{
    if (s_ready) { esp_vfs_littlefs_unregister("littlefs"); s_ready = false; }
}

bool storage_ready(void) { return s_ready; }

void storage_ensure_dirs(void)
{
    if (!s_ready) return;
    mkdir_p(ZONES_DIR);
    mkdir_p(CAL_DIR);
    mkdir_p(WATER_DIR);
    mkdir_p(LOGS_DIR);
}

/* ── Zones ──────────────────────────────────────────────────────────── */

esp_err_t storage_zone_list(uint16_t *ids_out, int max, int *count_out)
{
    *count_out = 0;
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    DIR *d = opendir(ZONES_DIR);
    if (!d) return ESP_ERR_NOT_FOUND;
    struct dirent *ent;
    while ((ent = readdir(d)) && *count_out < max) {
        unsigned id;
        if (sscanf(ent->d_name, "zone_%u", &id) == 1 && strstr(ent->d_name, ".json"))
            ids_out[(*count_out)++] = (uint16_t)id;
    }
    closedir(d);
    return ESP_OK;
}

esp_err_t storage_zone_load(uint16_t id, char *name_out, size_t name_len,
                             zone_perimeter_t *zone_out)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    char path[64]; zone_path_find(id, path, sizeof(path));
    char *json = read_file(path);
    if (!json) return ESP_ERR_NOT_FOUND;

    if (name_out && name_len > 0) {
        if (!json_get_str(json, "name", name_out, name_len))
            strncpy(name_out, "Zone", name_len-1);
    }

    if (zone_out) {
        memset(zone_out, 0, sizeof(*zone_out));
        int np = 0;
        json_get_int(json, "num_points", &np);
        zone_out->num_points = (uint8_t)np;

        /* Points array: "points":[{...},{...},...] */
        const char *pp = strstr(json, "\"points\":");
        if (pp) {
            pp = strchr(pp, '[');
            if (pp) pp++;
            for (int i = 0; i < np && pp && *pp; i++) {
                /* Find next { */
                pp = strchr(pp, '{');
                if (!pp) break;
                /* Extract fields from this object */
                char obj[256]; int depth=0, oj=0;
                const char *op = pp;
                while (*op && oj < (int)sizeof(obj)-1) {
                    obj[oj++] = *op;
                    if (*op=='{') depth++;
                    else if (*op=='}') { depth--; if(depth==0){op++;break;} }
                    op++;
                }
                obj[oj] = 0;
                float v;
                if (json_get_float(obj, "nozzle_deg", &v)) zone_out->points[i].nozzle_deg = v;
                if (json_get_float(obj, "valve_deg",  &v)) zone_out->points[i].valve_deg  = v;
                if (json_get_float(obj, "pressure",   &v)) zone_out->points[i].pressure_psi = v;
                if (json_get_float(obj, "throw_mm",   &v)) zone_out->points[i].throw_mm   = v;
                int wi = 0;
                if (json_get_int(obj, "walk_idx", &wi)) zone_out->points[i].walk_idx = (uint8_t)wi;
                pp = op;
            }
        }
    }
    free(json);
    return ESP_OK;
}

// b398: parse a zone JSON (same shape storage_zone_save writes / GET
// /zone/export emits) into a zone_perimeter_t, WITHOUT saving. Lives here so it
// can reuse the static json_get_* helpers and the exact point-scan used by
// storage_zone_load. out_id = the JSON "id" if numeric, else -1 (e.g. "new"/
// absent) so the caller can pick the next id. num_points is the COUNT ACTUALLY
// PARSED (capped at ZONE_MAX_PERIM_POINTS), not the header value.
esp_err_t storage_zone_parse_json(const char *json, int *out_id,
                                  char *name_out, size_t name_len,
                                  zone_perimeter_t *zone_out, int *out_np)
{
    if (!json || !zone_out) return ESP_ERR_INVALID_ARG;

    if (out_id) {
        // NUMERIC GUARD: json_get_int would atoi("new") -> 0, silently making an
        // "import as new" overwrite zone 0. Only accept a genuinely numeric id;
        // anything else (a quoted string like "new", or absent) -> -1 ("new").
        int idv = -1;
        const char *p = strstr(json, "\"id\"");
        if (p && (p = strchr(p, ':'))) {
            p++;
            while (*p == ' ' || *p == '\t') p++;
            if ((*p >= '0' && *p <= '9') || *p == '-') idv = atoi(p);
        }
        *out_id = idv;
    }
    if (name_out && name_len > 0) {
        if (!json_get_str(json, "name", name_out, name_len))
            strncpy(name_out, "Zone", name_len - 1);
    }

    memset(zone_out, 0, sizeof(*zone_out));
    int np = 0;
    json_get_int(json, "num_points", &np);
    if (np < 0) np = 0;
    if (np > ZONE_MAX_PERIM_POINTS) np = ZONE_MAX_PERIM_POINTS;

    const char *pp = strstr(json, "\"points\":");
    int parsed = 0;
    if (pp) {
        pp = strchr(pp, '[');
        if (pp) pp++;
        for (int i = 0; i < np && pp && *pp; i++) {
            pp = strchr(pp, '{');
            if (!pp) break;
            char obj[256]; int depth = 0, oj = 0;
            const char *op = pp;
            while (*op && oj < (int)sizeof(obj) - 1) {
                obj[oj++] = *op;
                if (*op == '{') depth++;
                else if (*op == '}') { depth--; if (depth == 0) { op++; break; } }
                op++;
            }
            obj[oj] = 0;
            float v;
            if (json_get_float(obj, "nozzle_deg", &v)) zone_out->points[i].nozzle_deg   = v;
            if (json_get_float(obj, "valve_deg",  &v)) zone_out->points[i].valve_deg    = v;
            if (json_get_float(obj, "pressure",   &v)) zone_out->points[i].pressure_psi = v;
            if (json_get_float(obj, "throw_mm",   &v)) zone_out->points[i].throw_mm     = v;
            int wi = 0;
            if (json_get_int(obj, "walk_idx", &wi)) zone_out->points[i].walk_idx = (uint8_t)wi;
            parsed++;
            pp = op;
        }
    }
    zone_out->num_points = (uint8_t)parsed;
    if (out_np) *out_np = parsed;
    return ESP_OK;
}

esp_err_t storage_zone_save(uint16_t id, const char *name, const zone_perimeter_t *zone)
{
    if (!s_ready) {
        ESP_LOGE(TAG, "storage_zone_save: storage not ready");
        return ESP_ERR_INVALID_STATE;
    }
    mkdir_p(ZONES_DIR);  // ensure parent exists (storage_init's mkdir may have been lost)
    char old_path[64]; zone_path_find(id, old_path, sizeof(old_path));
    char new_path[64]; zone_path_named(id, name, new_path, sizeof(new_path));
    FILE *f = fopen(new_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "storage_zone_save: fopen('%s') failed: errno=%d (%s)",
                 new_path, errno, strerror(errno));
        return ESP_FAIL;
    }

    fprintf(f, "{\n");
    fprintf(f, "  \"id\": %u,\n", id);
    fprintf(f, "  \"name\": \"%s\",\n", name ? name : "Zone");
    fprintf(f, "  \"num_points\": %u,\n", zone->num_points);
    fprintf(f, "  \"points\": [\n");
    for (int i = 0; i < zone->num_points; i++) {
        const perimeter_point_t *p = &zone->points[i];
        fprintf(f, "    {\"nozzle_deg\": %.2f, \"valve_deg\": %.2f, "
                   "\"pressure\": %.4f, \"throw_mm\": %.1f, \"walk_idx\": %u}%s\n",
                p->nozzle_deg, p->valve_deg, p->pressure_psi, p->throw_mm,
                p->walk_idx, (i < zone->num_points-1) ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
    /* If the file was renamed (name change), remove the old file. */
    if (strcmp(old_path, new_path) != 0) {
        FILE *chk = fopen(old_path, "rb");
        if (chk) { fclose(chk); remove(old_path); }
    }
    // (Caller logs the user-facing "Zone X saved" line.)
    return ESP_OK;
}

esp_err_t storage_zone_delete(uint16_t id)
{
    if (!s_ready) {
        ESP_LOGE(TAG, "storage_zone_delete: storage not ready");
        return ESP_ERR_INVALID_STATE;
    }
    char path[64]; zone_path_find(id, path, sizeof(path));
    if (remove(path) == 0) {
        ESP_LOGI(TAG, "Zone %u deleted (%s)", id, path);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "storage_zone_delete: remove('%s') failed: errno=%d (%s)",
             path, errno, strerror(errno));
    return ESP_FAIL;
}

esp_err_t storage_zone_rename(uint16_t id, const char *name)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    /* Load existing zone, re-save with new name */
    char old_name[32] = {0};
    zone_perimeter_t zone = {0};
    esp_err_t r = storage_zone_load(id, old_name, sizeof(old_name), &zone);
    if (r != ESP_OK) return r;
    return storage_zone_save(id, name, &zone);
}

/* ── Calibration ────────────────────────────────────────────────────── */

esp_err_t storage_cal_save(const pressure_map_t *map)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    mkdir_p(CAL_DIR);
    FILE *f = fopen(PRES_JSON, "wb");
    if (!f) {
        ESP_LOGE(TAG, "storage_cal_save: fopen('%s') failed: errno=%d (%s)",
                 PRES_JSON, errno, strerror(errno));
        return ESP_FAIL;
    }
    fprintf(f, "{\n  \"num_points\": %u,\n", map->num_points);
    fprintf(f, "  \"valve_deg\": [");
    for (int i = 0; i < map->num_points; i++)
        fprintf(f, "%s%.2f", i?",":"", map->valve_deg[i]);
    fprintf(f, "],\n  \"pressure_psi\": [");
    for (int i = 0; i < map->num_points; i++)
        fprintf(f, "%s%.4f", i?",":"", map->pressure_psi[i]);
    fprintf(f, "],\n  \"throw_mm\": [");
    for (int i = 0; i < map->num_points; i++)
        fprintf(f, "%s%.1f", i?",":"", map->throw_mm[i]);
    fprintf(f, "]\n}\n");
    fclose(f);
    ESP_LOGI(TAG, "Pressure cal saved (%u pts)", map->num_points);
    return ESP_OK;
}

esp_err_t storage_cal_load(pressure_map_t *map)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    char *json = read_file(PRES_JSON);
    if (!json) return ESP_ERR_NOT_FOUND;
    memset(map, 0, sizeof(*map));
    int np = 0;
    json_get_int(json, "num_points", &np);
    map->num_points = (uint8_t)np;
    json_get_float_array(json, "valve_deg",    map->valve_deg,    np);
    json_get_float_array(json, "pressure_psi", map->pressure_psi, np);
    json_get_float_array(json, "throw_mm",     map->throw_mm,     np);
    free(json);
    return ESP_OK;
}

/* ── Speed map ──────────────────────────────────────────────────────── */

esp_err_t storage_spd_save(const speed_map_t *map)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    mkdir_p(CAL_DIR);
    FILE *f = fopen(SPD_JSON, "wb");
    if (!f) {
        ESP_LOGE(TAG, "storage_spd_save: fopen('%s') failed: errno=%d (%s)",
                 SPD_JSON, errno, strerror(errno));
        return ESP_FAIL;
    }
    fprintf(f, "{\n");
    fprintf(f, "  \"num_points\": %u,\n", map->num_points);
    fprintf(f, "  \"num_points_ccw\": %u,\n", map->num_points_ccw);
    fprintf(f, "  \"min_continuous_dps\": %.2f,\n", map->min_continuous_dps);
    fprintf(f, "  \"jog_pulse_duty\": %u,\n", map->jog_pulse_duty);
    fprintf(f, "  \"jog_pulse_ms\": %u,\n", map->jog_pulse_ms);
    fprintf(f, "  \"jog_deg_per_pulse\": %.4f,\n", map->jog_deg_per_pulse);
    fprintf(f, "  \"use_bump_stop\": %u,\n", map->use_bump_stop);
    fprintf(f, "  \"duty\": [");
    for (int i = 0; i < map->num_points; i++)
        fprintf(f, "%s%u", i?",":"", map->duty[i]);
    fprintf(f, "],\n  \"deg_per_sec\": [");
    for (int i = 0; i < map->num_points; i++)
        fprintf(f, "%s%.2f", i?",":"", map->deg_per_sec[i]);
    fprintf(f, "],\n  \"duty_ccw\": [");
    for (int i = 0; i < map->num_points_ccw; i++)
        fprintf(f, "%s%u", i?",":"", map->duty_ccw[i]);
    fprintf(f, "],\n  \"deg_per_sec_ccw\": [");
    for (int i = 0; i < map->num_points_ccw; i++)
        fprintf(f, "%s%.2f", i?",":"", map->deg_per_sec_ccw[i]);
    fprintf(f, "]\n}\n");
    fclose(f);
    ESP_LOGI(TAG, "Speed map saved (CW:%u CCW:%u pts)", map->num_points, map->num_points_ccw);
    return ESP_OK;
}

esp_err_t storage_spd_load(speed_map_t *map)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    char *json = read_file(SPD_JSON);
    if (!json) return ESP_ERR_NOT_FOUND;
    memset(map, 0, sizeof(*map));
    int v = 0;
    json_get_int(json, "num_points",     &v); map->num_points = (uint8_t)v;
    json_get_int(json, "num_points_ccw", &v); map->num_points_ccw = (uint8_t)v;
    json_get_int(json, "jog_pulse_duty", &v); map->jog_pulse_duty = (uint16_t)v;
    json_get_int(json, "jog_pulse_ms",   &v); map->jog_pulse_ms = (uint16_t)v;
    json_get_int(json, "use_bump_stop",  &v); map->use_bump_stop = (uint8_t)v;
    float fv = 0;
    json_get_float(json, "min_continuous_dps", &fv); map->min_continuous_dps = fv;
    json_get_float(json, "jog_deg_per_pulse",  &fv); map->jog_deg_per_pulse = fv;
    /* Arrays */
    int duty_cw[16]={0}, duty_ccw[16]={0};
    int ncw  = json_get_int_array(json, "duty",     duty_cw,  map->num_points);
    int nccw = json_get_int_array(json, "duty_ccw", duty_ccw, map->num_points_ccw);
    for (int i=0;i<ncw;i++)  map->duty[i]     = (uint16_t)duty_cw[i];
    for (int i=0;i<nccw;i++) map->duty_ccw[i] = (uint16_t)duty_ccw[i];
    json_get_float_array(json, "deg_per_sec",     map->deg_per_sec,     map->num_points);
    json_get_float_array(json, "deg_per_sec_ccw", map->deg_per_sec_ccw, map->num_points_ccw);
    free(json);
    return ESP_OK;
}

/* ── Water run ──────────────────────────────────────────────────────── */

esp_err_t storage_water_save(uint16_t zone_id, const water_run_t *run)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    /* Look up zone name so water file matches its zone file. */
    char zone_name[32] = {0};
    {
        char zp[64]; zone_path_find(zone_id, zp, sizeof(zp));
        char *zjson = read_file(zp);
        if (zjson) { json_get_str(zjson, "name", zone_name, sizeof(zone_name)); free(zjson); }
    }
    mkdir_p(WATER_DIR);
    char old_path[64]; water_path_find(zone_id, old_path, sizeof(old_path));
    char path[64]; water_path_named(zone_id, zone_name, path, sizeof(path));
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "storage_water_save: fopen('%s') failed: errno=%d (%s)",
                 path, errno, strerror(errno));
        return ESP_FAIL;
    }
    fprintf(f, "{\n  \"zone_id\": %u,\n  \"fw_build\": %u,\n  \"num_rings\": %u,\n"
               "  \"arc_start\": %.1f,\n  \"arc_span\": %.1f,\n"
               "  \"total_time_s\": %.1f,\n"
               // b281: run-level supply pressure summary (back-computed from
               // per-ring avg_psi via cal's f(valve_deg)). Diagnoses well-pump
               // cycling impact on watering uniformity.
               "  \"supply_psi_min\": %.3f,\n  \"supply_psi_max\": %.3f,\n"
               "  \"supply_psi_avg\": %.3f,\n"
               // b282: gap analysis count -- rings flagged as supply-limited.
               "  \"rings_supply_limited\": %u,\n"
               // b388: run target depth (N x 1/8") for heatmap normalization.
               "  \"target_depth_mm\": %.3f,\n",
            zone_id, run->fw_build, run->num_rings,
            run->arc_start_deg, run->arc_span_deg, run->total_time_s,
            run->supply_psi_min, run->supply_psi_max, run->supply_psi_avg,
            (unsigned)run->rings_supply_limited, run->target_depth_mm);
    fprintf(f, "  \"rings\": [\n");
    for (int i = 0; i < run->num_rings; i++) {
        const water_ring_data_t *r = &run->rings[i];
        fprintf(f, "    {\"throw_mm\": %.1f, \"avg_psi\": %.4f, "
                   "\"actual_throw_mm\": %.1f, \"dps\": %.1f, \"active_deg\": %.1f,"
                   " \"arc_s\": %.1f, \"arc_e\": %.1f, \"depth_mm\": %.3f, "
                   // b281: per-ring valve angle (for downstream gap analysis)
                   // and back-computed supply pressure during this ring.
                   "\"valve_deg\": %.2f, \"supply_psi_est\": %.3f}%s\n",
                r->throw_mm, r->avg_psi, r->actual_throw_mm, r->dps, r->active_deg,
                r->arc_start_deg, r->arc_end_deg, r->depth_mm,
                r->valve_deg, r->supply_psi_est,
                (i < (int)run->num_rings-1) ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
    if (strcmp(old_path, path) != 0) {
        FILE *chk = fopen(old_path, "rb");
        if (chk) { fclose(chk); remove(old_path); }
    }
    return ESP_OK;
}

esp_err_t storage_water_load(uint16_t zone_id, water_run_t *run)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    char path[64]; water_path_find(zone_id, path, sizeof(path));
    char *json = read_file(path);
    if (!json) return ESP_ERR_NOT_FOUND;
    memset(run, 0, sizeof(*run));
    int v = 0;
    json_get_int(json, "fw_build",  &v); run->fw_build  = (uint16_t)v;
    json_get_int(json, "num_rings", &v); run->num_rings = (uint16_t)v;
    { float fv2=0;
      if (json_get_float(json, "arc_start",     &fv2)) run->arc_start_deg = fv2;
      if (json_get_float(json, "arc_span",      &fv2)) run->arc_span_deg  = fv2;
      if (json_get_float(json, "total_time_s",  &fv2)) run->total_time_s  = fv2;
      // b281: run-level supply summary (zeroes if loading a pre-b281 file)
      if (json_get_float(json, "supply_psi_min", &fv2)) run->supply_psi_min = fv2;
      if (json_get_float(json, "supply_psi_max", &fv2)) run->supply_psi_max = fv2;
      if (json_get_float(json, "supply_psi_avg", &fv2)) run->supply_psi_avg = fv2; }
    // b282: gap analysis count (zero if loading a pre-b282 file)
    { int vi = 0;
      if (json_get_int(json, "rings_supply_limited", &vi))
          run->rings_supply_limited = (uint8_t)vi; }
    // b388: run target depth (0 if loading a pre-b388 file -> card falls back)
    { float fv3 = 0;
      if (json_get_float(json, "target_depth_mm", &fv3)) run->target_depth_mm = fv3; }
    const char *pp = strstr(json, "\"rings\":");
    if (pp) {
        pp = strchr(pp, '['); if (pp) pp++;
        for (int i = 0; i < run->num_rings && pp && *pp; i++) {
            pp = strchr(pp, '{'); if (!pp) break;
            char obj[256]; int depth=0, oj=0;
            const char *op = pp;
            while (*op && oj < (int)sizeof(obj)-1) {
                obj[oj++] = *op;
                if (*op=='{') depth++;
                else if (*op=='}') { depth--; if(depth==0){op++;break;} }
                op++;
            }
            obj[oj] = 0;
            float fv = 0;
            if (json_get_float(obj, "throw_mm",        &fv)) run->rings[i].throw_mm        = fv;
            if (json_get_float(obj, "avg_psi",          &fv)) run->rings[i].avg_psi          = fv;
            if (json_get_float(obj, "actual_throw_mm",  &fv)) run->rings[i].actual_throw_mm  = fv;
            if (json_get_float(obj, "dps",              &fv)) run->rings[i].dps              = fv;
            if (json_get_float(obj, "active_deg",       &fv)) run->rings[i].active_deg       = fv;
            if (json_get_float(obj, "arc_s",            &fv)) run->rings[i].arc_start_deg    = fv;
            if (json_get_float(obj, "arc_e",            &fv)) run->rings[i].arc_end_deg      = fv;
            if (json_get_float(obj, "depth_mm",         &fv)) run->rings[i].depth_mm         = fv;
            // b281: per-ring valve angle + back-computed supply pressure
            //       (zeroes if loading a pre-b281 file)
            if (json_get_float(obj, "valve_deg",        &fv)) run->rings[i].valve_deg        = fv;
            if (json_get_float(obj, "supply_psi_est",   &fv)) run->rings[i].supply_psi_est   = fv;
            pp = op;
        }
    }
    free(json);
    return ESP_OK;
}

esp_err_t storage_water_delete(uint16_t zone_id)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t r = ESP_OK;
    char path[64];
    // Delete water run JSON
    water_path_find(zone_id, path, sizeof(path));
    if (remove(path) != 0) r = ESP_FAIL;
    // Delete CSV: scan for water_NNN*.csv (name-suffixed format) rather than
    // the old zone_NNN.csv pattern which never matched the actual filenames.
    char prefix[13]; snprintf(prefix, sizeof(prefix), "water_%03u", zone_id);
    DIR *d = opendir(WATER_DIR);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d))) {
            size_t plen = strlen(prefix);
            if (strncmp(ent->d_name, prefix, plen) == 0 &&
                    (ent->d_name[plen] == '.' || ent->d_name[plen] == '_') &&
                    (strstr(ent->d_name, ".csv") || strstr(ent->d_name, ".wbin"))) {
                char csv_path[300];
                snprintf(csv_path, sizeof(csv_path), WATER_DIR "/%s", ent->d_name);
                remove(csv_path);
            }
        }
        closedir(d);
    }
    return r;
}

esp_err_t storage_make_room(size_t bytes_needed, uint16_t skip_zone_id)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    size_t used = 0, total = 0;
    if (storage_usage(&used, &total) != ESP_OK) return ESP_FAIL;
    if (total > used && (total - used) >= bytes_needed) return ESP_OK;

    ESP_LOGW(TAG, "make_room: need %uKB, %uKB free -- clearing old water logs",
             (unsigned)(bytes_needed / 1024),
             (unsigned)((total > used ? total - used : 0) / 1024));

    // Collect all water log files for zones other than skip_zone_id.
    // We'll delete binary/csv data files first (bulk), then JSON summaries.
    typedef struct { uint16_t zone_id; uint8_t is_data; char name[60]; } mr_ent_t;
    static mr_ent_t mr_buf[200];
    int mr_n = 0;

    DIR *d = opendir(WATER_DIR);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d)) && mr_n < 200) {
            if (strncmp(ent->d_name, "water_", 6) != 0) continue;
            uint16_t zid = (uint16_t)atoi(ent->d_name + 6);
            if (zid == skip_zone_id) continue;
            bool is_data = (strstr(ent->d_name, ".wbin") || strstr(ent->d_name, ".csv")) ? true : false;
            bool is_json = strstr(ent->d_name, ".json") ? true : false;
            if (!is_data && !is_json) continue;
            mr_buf[mr_n].zone_id = zid;
            mr_buf[mr_n].is_data = is_data ? 1u : 0u;
            strncpy(mr_buf[mr_n].name, ent->d_name, sizeof(mr_buf[mr_n].name) - 1);
            mr_buf[mr_n].name[sizeof(mr_buf[mr_n].name) - 1] = '\0';
            mr_n++;
        }
        closedir(d);
    }

    // Sort: data files (is_data=1) before JSON (is_data=0); within each group,
    // lower zone_id first (oldest zone deleted first).
    for (int i = 1; i < mr_n; i++) {
        mr_ent_t key = mr_buf[i];
        int j = i - 1;
        while (j >= 0) {
            bool shift = (mr_buf[j].is_data < key.is_data) ||
                         (mr_buf[j].is_data == key.is_data && mr_buf[j].zone_id > key.zone_id);
            if (!shift) break;
            mr_buf[j + 1] = mr_buf[j];
            j--;
        }
        mr_buf[j + 1] = key;
    }

    // Delete in sorted order until enough space is free.
    for (int i = 0; i < mr_n; i++) {
        char path[300];
        snprintf(path, sizeof(path), WATER_DIR "/%s", mr_buf[i].name);
        if (remove(path) == 0) {
            ESP_LOGI(TAG, "make_room: removed %s", mr_buf[i].name);
            if (storage_usage(&used, &total) == ESP_OK
                    && total > used && (total - used) >= bytes_needed)
                return ESP_OK;
        }
    }

    // Report final state even if target wasn't reached.
    if (storage_usage(&used, &total) == ESP_OK)
        ESP_LOGW(TAG, "make_room: done, %uKB free (target %uKB)",
                 (unsigned)((total > used ? total - used : 0) / 1024),
                 (unsigned)(bytes_needed / 1024));
    return ESP_FAIL;
}

/* ── Logging ────────────────────────────────────────────────────────── */

esp_err_t storage_log(const char *line)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    time_t now = time(NULL);
    struct tm t; localtime_r(&now, &t);
    char path[64];
    snprintf(path, sizeof(path), LOGS_DIR "/%04d%02d%02d.log",
             t.tm_year+1900, t.tm_mon+1, t.tm_mday);
    FILE *f = fopen(path, "ab");
    if (!f) return ESP_FAIL;
    fprintf(f, "%s\n", line);
    fclose(f);
    /* Rotate old logs */
    DIR *d = opendir(LOGS_DIR);
    if (d) {
        char names[MAX_LOG_FILES+4][16]; int count = 0;
        struct dirent *ent;
        while ((ent = readdir(d)) && count < (int)(sizeof(names)/sizeof(names[0])))
            if (ent->d_name[0] != '.') strncpy(names[count++], ent->d_name, 15);
        closedir(d);
        if (count > MAX_LOG_FILES) {
            /* Simple sort to find oldest */
            for (int i=0; i<count-1; i++)
                for (int j=i+1; j<count; j++)
                    if (strcmp(names[i], names[j]) > 0) {
                        char tmp[16]; strcpy(tmp, names[i]);
                        strcpy(names[i], names[j]); strcpy(names[j], tmp);
                    }
            char del[64]; snprintf(del, sizeof(del), LOGS_DIR "/%s", names[0]);
            remove(del);
        }
    }
    return ESP_OK;
}

/* ── Usage ──────────────────────────────────────────────────────────── */

esp_err_t storage_usage(size_t *used_out, size_t *total_out)
{
    *used_out = *total_out = 0;
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    return esp_littlefs_info("littlefs", total_out, used_out);
}

/* ── Schedule (binary, kept for future use) ─────────────────────────── */

esp_err_t storage_schedule_load(storage_schedule_t *out)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    FILE *f = fopen(SCHED_JSON, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;
    size_t rd = fread(out, 1, sizeof(*out), f); fclose(f);
    return (rd == sizeof(*out)) ? ESP_OK : ESP_FAIL;
}

esp_err_t storage_schedule_save(const storage_schedule_t *sched)
{
    if (!s_ready) {
        ESP_LOGE(TAG, "storage_schedule_save: storage not ready");
        return ESP_ERR_INVALID_STATE;
    }
    FILE *f = fopen(SCHED_JSON, "wb");
    if (!f) {
        ESP_LOGE(TAG, "storage_schedule_save: fopen('%s') failed: errno=%d (%s)",
                 SCHED_JSON, errno, strerror(errno));
        return ESP_FAIL;
    }
    size_t wr = fwrite(sched, 1, sizeof(*sched), f); fclose(f);
    if (wr != sizeof(*sched)) {
        ESP_LOGE(TAG, "storage_schedule_save: short write %u/%u",
                 (unsigned)wr, (unsigned)sizeof(*sched));
        return ESP_FAIL;
    }
    return ESP_OK;
}
