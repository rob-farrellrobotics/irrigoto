# irrigoto — Home Assistant configuration examples

Drop-in YAML for running one **or several** irrigoto devices from Home
Assistant once they've been adopted by HA's ESPHome integration. Nothing
here requires HACS — every entry uses HA's built-in domains (template,
automation, notify, recorder). The one optional custom card (the watering
heatmap) is plain JavaScript you copy into `www/`.

## TL;DR — adding devices is one file + one command

1. Copy `devices.example.yaml` → `devices.yaml` and list your devices
   (slug, friendly name, IP).
2. Run `python tools/ha-regen.py`.
3. Copy `generated/packages/*.yaml` into your HA `config/packages/`
   and paste `generated/dashboards/irrigoto.yaml` into your dashboard.

Adding a second/third device is **just another entry in `devices.yaml`
and a re-run** — no per-device find/replace, no hand-edited copies, no
colliding entity IDs. The dashboard grows a device picker automatically.

## Layout

```
homeassistant/
├── devices.example.yaml        ← copy to devices.yaml; your fleet manifest
├── packages/
│   ├── irrigoto_common.yaml    ← SHARED template: device picker, HTTP
│   │                             helpers, fleet-wide activity log
│   ├── irrigoto_device.yaml    ← PER-DEVICE template: one device's
│   │                             automations + sensors (slug-prefixed)
│   └── irrigoto_schedule.yaml  ← SHARED template: HA-side desired-schedule
│                                 store, compose form, overlap validator
├── dashboards/
│   └── irrigoto.yaml           ← single-device dashboard template; regen
│                                 turns it into a multi-device dashboard
├── cards/                      ← individual card snippets for splicing
├── lovelace/
│   └── irrigoto-heatmap-card.js ← optional custom card: per-zone heatmap
└── extras/
    └── csv_watering_log.yaml    ← optional flat-file watering audit trail
```

The files above are **templates**. `tools/ha-regen.py` reads
`devices.yaml` and writes ready-to-copy files into `generated/` (the whole
folder is gitignored). You install the generated files, not the templates —
the filenames are exactly what HA wants, so it's a 1:1 copy with no renaming.

## How multi-device works

There are two halves, and the split is the whole trick to scaling cleanly:

- **Shared** entities are defined **once** and named with the bare
  `irrigoto_` prefix (e.g. `sensor.irrigoto_desired_schedule`,
  `input_select.irrigoto_active_device`). These come from
  `irrigoto_common.yaml` and `irrigoto_schedule.yaml`.
- **Per-device** entities are defined **once per device** and named with
  that device's slug (e.g. `sensor.irrigoto_ab12cd_cumulative_volume`).
  These come from `irrigoto_device.yaml`, which regen stamps out once for
  each device in the manifest. Because every name carries the device slug,
  N devices coexist in one HA with **zero entity-ID collisions**.

The rule, if you ever read the generated YAML:

| Name shape | Meaning |
| --- | --- |
| `irrigoto_<x>` | shared across the whole fleet |
| `<slug>_<x>` (e.g. `irrigoto_ab12cd_cumulative_volume`) | belongs to one device, defined here in HA |
| `<slug>_irrigoto_<x>` (e.g. `irrigoto_ab12cd_irrigoto_pressure`) | published *by* the device's ESPHome integration |

> **Backend vs. dashboard.** Every device's automations and sensors
> always run, for all devices, all the time. The dashboard's device
> picker is **display-only** — it changes what you *see*, never what
> *runs*.

## The manifest (`devices.yaml`)

```yaml
devices:
  - slug: irrigoto-ab12cd     # dash-form slug from Settings → Devices &
    name: Front Yard          #   Services → ESPHome (under the title)
    url:  http://192.168.1.50 # device IP (or http://<slug>.local if it
                              #   resolves from your HA host)
  - slug: irrigoto-ef34gh
    name: Back Yard
    url:  http://192.168.1.51
```

- **slug** — the dash-form device slug ESPHome shows. Lowercase letters,
  digits, dashes. It's both the entity-ID prefix and the per-device cache
  folder name.
- **name** — friendly label. This is what the dashboard's device picker
  shows; it must be unique across devices (it's the picker's match key).
- **url** — base URL the heatmap card and the HTTP-poll schedule sensor
  fetch from. Use the IP if `<slug>.local` doesn't resolve from HA.

`devices.yaml` is gitignored (it carries your MACs / LAN addresses);
`devices.example.yaml` is the committed template.

## Install

1. One-time, in `configuration.yaml` (if not already present):

   ```yaml
   homeassistant:
     packages: !include_dir_named packages
   ```

2. `cp homeassistant/devices.example.yaml homeassistant/devices.yaml`
   and edit it for your fleet.

3. `python tools/ha-regen.py` — writes into `homeassistant/generated/`:

   ```
   generated/packages/irrigoto_common.yaml          (one)
   generated/packages/irrigoto_schedule.yaml         (one)
   generated/packages/irrigoto_device_<slug_usc>.yaml (one per device; the
                                                       slug's dashes become
                                                       underscores so HA
                                                       accepts the filename
                                                       as a package name)
   generated/dashboards/irrigoto.yaml                (one)
   ```

   (`tools/ha-regen.sh` still works — it just forwards to the Python tool.)

4. Copy `generated/packages/*.yaml` into your HA `config/packages/`
   directory — a straight 1:1 copy, the filenames are already correct.

5. Restart HA, or **Developer Tools → YAML → All YAML configuration** for
   a soft reload. Verify in **Developer Tools → States** by searching
   `irrigoto`.

6. (Dashboard) **Settings → Dashboards → + Add Dashboard → "Irrigoto" →
   New dashboard from scratch →** then **three-dot menu → Edit dashboard →
   three-dot menu → Raw configuration editor →** paste the contents of
   `generated/dashboards/irrigoto.yaml`.

### Adding another device later

1. Add its entry to `devices.yaml`.
2. `python tools/ha-regen.py`.
3. Copy the new `generated/packages/irrigoto_device_<slug_usc>.yaml` into
   `config/packages/` and re-paste `generated/dashboards/irrigoto.yaml` into
   the dashboard's raw config (it now has a device picker at the top of
   every tab).
4. Reload YAML.

The compose form's device dropdown and the dashboard picker both pick up
the new device automatically — they're filled from the manifest.

## The dashboard

`dashboards/irrigoto.yaml` is a full Lovelace dashboard (Overview, Trends,
Heatmaps, Chase, Schedule, Device, Calibration). Regen turns it into a
multi-device dashboard:

- **One device** → a plain single-device dashboard, no picker.
- **Two or more** → a **Device** picker
  (`input_select.irrigoto_active_device`) is added to the top of every tab,
  and each tab's cards are repeated once per device, each copy wrapped in a
  `conditional` so only the **selected** device's cards render. Pick a
  device once and every tab follows it. No combined tab, no per-device
  dashboards to maintain.

This uses only built-in card types — the per-device duplication is
generated, never hand-maintained.

## What's in the per-device package

| Section | Purpose |
| ------- | ------- |
| Auto-sleep off on connect | Keeps the device awake while you're working on it; fires once when it reconnects after a sleep cycle. |
| Log watering completion | Appends a one-line summary (zone, duration, volume, score, status) to the Logbook every time a watering finishes. |
| Last-watering sensors | Surface the rich `watering_complete` event data as regular sensors for dashboards/automations. |
| Cumulative volume statistic | Running water-use total with `state_class: total_increasing` for HA long-term statistics. |
| Schedule mirror (display) | Stores the device's live `schedule_text` in `<slug>_schedule_mirror`. |
| Chase scripts + valve switch | Quick "chase" watering buttons and a valve open/close toggle (both via the shared HTTP helpers). |
| Heatmap mirror | Pulls per-zone heatmap data into HA's `www/` so the card renders while the device sleeps. **Namespaced per device** (see Cached files below). |
| Schedule reconciler (bidirectional) | Per-device sensors + `<slug>_push_schedule` script + reconciler automation. Edit anywhere — HA dashboard or device web UI — and both sides stay in lockstep via per-entry IDs + `last_modified`, conflicts resolving last-writer-wins (device wins on tie). See [the design doc](../docs/schedule_sync_design.md). |

## Cached files are per-device

The heatmap mirror writes each device's cached data to its **own folder**:

```
/config/www/community/irrigoto/<slug>/csv/{api_all.json, zone_<id>.csv, zone_<id>_lastwater.json}
   → served by HA at /local/community/irrigoto/<slug>/csv/...
```

So two devices never clobber each other's heatmap cache. The shared
`shell_command.irrigoto_mirror_zone` takes the device `slug` as a
parameter; the heatmap card reads its `mirror_slug` config to fetch from
the matching folder. The card's browser `localStorage` fallback is also
keyed per device. (Legacy single-device installs that used the flat
`/local/community/irrigoto/csv/` path keep working if no slug is set.)

## HA-side scheduling (`irrigoto_schedule.yaml`)

The device runs its schedule autonomously and survives HA outages — this is
non-negotiable for an outdoor sprinkler controller. But HA can hold a richer
"desired schedule" (potentially driven by weather forecasts, calendars, etc.)
and reconcile it into each device whenever that device is awake.

Storage is **pure HA helpers** — no files, no `command_line:`, no File Editor
add-on. 10 per-slot `input_text` helpers (`input_text.irrigoto_sched_1`
through `_10`) each hold one CSV-formatted entry; a template sensor
aggregates non-empty slots into the per-device shape the reconciler
consumes. A compose form on the Schedule tab (dropdowns + time picker) plus
four scripts (Save / Load / Clear slot / Clear all) let you edit schedules
entirely from the HA UI. The compose form's **Device** dropdown is filled
from the manifest, so an entry is always tied to a specific device's slug.

The compose form's **Days** dropdown offers `Daily`, `Weekdays`, `Weekends`,
`MWF`, `TTh`, each individual day, plus `Custom` (raw 0..127 mask, bit0=Sun
… bit6=Sat — same convention as the firmware).

### Pending states

Two per-device badges, one per direction, so it's always clear which side
holds the unsynced change:

**`sensor.<slug>_schedule_pending_outbound`** — HA → device

| State | Meaning |
| ----- | ------- |
| `disabled` | No HA slots reference this device — reconciler is a no-op. |
| `synced` | HA's desired set matches the device. Steady state. |
| `pending - device asleep` | HA has changes; device offline. Pushes on next wake. |
| `pending - pushing` | HA has changes; device online; reconciler in flight. |
| `blocked: N conflict(s)` | Cross-device overlap validator flagged a conflict; push refused unless override is on. |
| `push failed: <reason>` | Last sync rejected by the device. |

**`sensor.<slug>_schedule_pending_inbound`** — device → HA

| State | Meaning |
| ----- | ------- |
| `synced` | Device version ≤ HA's last-seen version. Steady state. |
| `pending - pulling` | Device reports a newer version; reconciler is pulling its entries into HA slots. |
| `pending - HA offline` | Device moved ahead while HA was off; surfaces until the reconciler runs. |

Safe defaults: empty HA slots are a no-op (the reconciler never wipes the
device blind). Editing on the device web UI is supported — the device
increments its `schedule_version` and HA pulls those entries into the next
free slot within a few seconds while online.

### How conflicts resolve

Each entry has a `last_modified` (unix epoch) and a device-assigned `id`.
When the same entry is edited on both surfaces in one sync window, the
strictly-newer `last_modified` wins; **device wins on exact tie**. Entries
the device has whose `id` HA doesn't mention (e.g. added via the web UI
before HA learned of them) are **preserved unconditionally** — web-UI adds
survive HA pushes automatically. Clearing an HA slot whose `id` is non-zero
emits a tombstone telling the device to delete that entry.

### Cross-device overlap validator

`sensor.irrigoto_schedule_overlap_status` (shared) pairwise-checks every
enabled entry across **all** devices for time-window overlap on shared
days, using a mode-based duration heuristic (+10% margin). State is `ok` or
`N conflict(s)` (first conflict in the `first` attribute). The reconciler
**blocks the push** when status is non-ok unless
`input_boolean.irrigoto_schedule_overlap_override` is on. This is the only
cross-device coordination shipped here, and it's schedule-time only —
there's deliberately no live "don't run two devices at once" mutex (add one
yourself if your devices share a water supply).

## Dashboards / cards

If you'd rather splice into an existing dashboard than use the full one,
the snippets in `cards/` show individual `type: ...` blocks. Note they use
`<<DEV_USC>>`/`<<DEV_DASH>>` placeholders — substitute your device's slug by
hand, or copy the equivalent card out of a generated
`generated/dashboards/irrigoto.yaml`.

The custom heatmap card under `lovelace/` is **optional**. Install +
config instructions are in `lovelace/README.md`; the generated dashboard
already wires it up per device (with `device` and `mirror_slug` set).

## Optional extras

`extras/csv_watering_log.yaml` adds a `notify.file_irrigoto_log` service plus
an automation that appends every watering to `/config/irrigoto_waterings.csv`
— a flat-file audit trail for Excel. Not enabled by default.

## Native history without any of this

HA's recorder already logs every entity state change. With nothing beyond
the ESPHome integration you can open **History**, pick
`binary_sensor.<slug>_irrigoto_watering`, and see every ON period; add a
Statistics card on `sensor.<slug>_irrigoto_pressure`; or watch
`esphome.<slug>_watering_complete` live in **Developer Tools → Events**. The
package adds richer, longer-lived views on top of that.
