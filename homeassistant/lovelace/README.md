# irrigoto — Lovelace custom card

`irrigoto-heatmap-card.js` is a self-contained Lovelace card that renders
the per-zone watering depth heatmap from an irrigoto device, matching the
on-device "Actual" mode rendering in `zone_setup.html`. It uses HA theme
CSS variables (`--card-background-color`, `--primary-text-color`,
`--secondary-text-color`, `--divider-color`, `--primary-color`,
`--error-color`) so it picks up your dashboard styling.

The card fetches three endpoints from the device on render:

| Endpoint | Purpose |
| --- | --- |
| `GET /api/all` | Zone polygon vertices (perimeter) |
| `GET /zone/last_water?id=N` | Per-ring summary (throw, dps, supply PSI) |
| `GET /zone/water_csv?id=N` | Per-(ring,sector) aggregate records |

**No HA REST sensors required** — the card fetches directly, same way the
on-device web UI does. The only requirement is that the HA browser
(whichever device you view the dashboard on) can reach the device's URL.

## Install

1. Copy `irrigoto-heatmap-card.js` to your HA config:
   ```
   config/www/community/irrigoto/irrigoto-heatmap-card.js
   ```
   (Any path under `www/` works; the `community/irrigoto/` convention
   mirrors HACS.)

2. Add it as a Lovelace resource. **Settings → Dashboards → Resources →
   Add Resource**:
   - URL: `/local/community/irrigoto/irrigoto-heatmap-card.js`
   - Type: **JavaScript Module**

   (Or via YAML in `lovelace:` mode:
   ```yaml
   resources:
     - url: /local/community/irrigoto/irrigoto-heatmap-card.js
       type: module
   ```)

3. Hard-reload your browser (Ctrl+F5) so HA picks up the new resource.

## Use

Add a card to any dashboard via **Add Card → Custom: Irrigoto Heatmap**,
or directly in YAML.

**Preferred form — HA-managed device discovery (no IP in YAML):**

```yaml
type: custom:irrigoto-heatmap-card
device: irrigoto-xxxxxx        # ESPHome device slug (HA Settings → Devices)
zone_id: 0
```

The card looks the device up in HA's device registry and resolves
its URL via, in order:

1. **`configuration_url`** on the device record. Auto-set when the
   device is adopted via HA's ESPHome integration. If your device
   was adopted via the ESPHome dashboard add-on, this field stays
   empty unless you set it manually — see one-time setup below.
2. **`config_entries/get` data.host** — fallback for some HA
   versions. Recent HA versions redact this field from the
   frontend API for security; if so, falls through.
3. **mDNS hostname** `http://<device-slug>.local` — last-ditch
   resolution. Works when the browser handles .local addresses;
   blocked on HTTPS HA + HTTP .local (mixed-content).

**One-time HA setup for fully deployable cards**: Settings →
Devices & Services → Devices → click your device → click the
edit (pencil) icon → set the **Configuration URL** field to
`http://<device-ip>`. The card then resolves via path 1 with no
IP in the dashboard YAML. Static IP reservation for the device
on your router is recommended so the value stays valid.

**Explicit-URL form — override / fallback:**

```yaml
type: custom:irrigoto-heatmap-card
device_url: http://192.168.1.x
zone_id: 0
```

Use this if the device isn't adopted into the ESPHome integration, or
if you need to point at a specific URL (proxy, custom port, etc.).

**Common options for either form:**

```yaml
title: "Zone heatmap"        # optional, defaults to "Zone N heatmap"
refresh_interval: 0          # optional, seconds; 0 = manual refresh only
max_throw_mm: 8534           # optional, display scale
target_depth_mm: 3.175       # optional, 1/8 in per pass = 3.175 mm
```

Add one card per zone you want to view. The card has a ↻ button in the
header for manual refresh; the same data the device itself uses, so
freshness is end-of-last-run.

## Config reference

| Key | Required | Default | Notes |
| --- | --- | --- | --- |
| `device` | yes¹ | — | ESPHome device slug or HA device name (matched against `name`, `name_by_user`, or identifier values). Auto-resolves the device's IP via HA's device registry. |
| `device_url` | yes¹ | — | Explicit device base URL, e.g. `http://192.168.1.x`. Overrides `device`-based discovery when both are set. |
| `mirror_slug` | no | `device` | Per-device HA mirror sub-folder. The fallback cache lives at `/local/community/irrigoto/<mirror_slug>/csv/`, matching where the `irrigoto_mirror_zone` shell_command writes for that device. Defaults to the `device` slug; set it explicitly only if they differ. Keeps multiple devices' caches separate. |
| `zone_id` | yes | — | Integer zone id (`0` = first zone, etc.). |
| `title` | no | `Zone N heatmap` | Card header text. |
| `refresh_interval` | no | `0` | Seconds between auto-refreshes. `0` disables (manual ↻ only). |
| `max_throw_mm` | no | `8534` | Radial display scale. The on-device UI uses 8534 (~28 ft) so the card matches by default. |
| `target_depth_mm` | no | `3.175` | Target depth = 1/8 inch per pass. Used to normalize the color ramp. |

¹ Exactly one of `device` or `device_url` is required. Prefer `device`
for deployability — no IPs hardcoded in dashboard YAML.

## Color ramp

Same as the device UI:

- **blue** &nbsp; < 0.6 × target depth
- **teal** &nbsp;&nbsp; 0.6 – 0.88
- **green** &nbsp;0.88 – 1.12
- **amber** &nbsp;1.12 – 1.5
- **red** &nbsp;&nbsp;&nbsp;&nbsp; > 1.5

Green = on target. Amber/red = over-watered. Blue/teal = under-watered.

## Limitations

- **Mixed-content blocking**: if your HA is served over HTTPS but the
  device URL is `http://`, browsers may block the fetch. Use the
  device's mDNS hostname over `http://` only in mixed-content allowed
  contexts, or front the device with a reverse proxy.
- **Per-browser fetch**: each viewer's browser fetches directly from the
  device. The device's web server is single-threaded but the data is
  small (~few KB per endpoint) so multi-viewer load is negligible.
- **End-of-last-run snapshot only**: the data the device exposes is the
  result of the last completed watering. The card does not poll mid-run
  (the firmware only finalizes the CSV at run end).
