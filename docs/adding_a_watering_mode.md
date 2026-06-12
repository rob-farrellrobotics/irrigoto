# Adding a New Watering Mode

A checklist for wiring a new watering mode through every surface of the
project. Written after adding **Serpentine** (mode 8, builds 423–436), which
shipped in stages and kept surfacing one more place a mode list was
hardcoded — use this so your mode lands everywhere in one pass.

A "mode" here is a watering *style* (like Pulse / Gentle / Smooth /
Serpentine), selectable from the device web UI, Home Assistant, and the
on-device schedule.

## The three encodings (don't mix them up)

| Encoding | Values | Where it lives |
|---|---|---|
| **Web mode digit** | `1`-`4` pulse, `5`/`6` gentle, `7` smooth, `8` serpentine, `c` chase, `d` demo | `POST /zone/water` `mode=` param; `s_web_water_mode` |
| **HA select index** | `0` Pulse, `1` Gentle, `2` Smooth, `3` Serpentine | `irrigoto_get_mode()` / `IrrigotoModeSelect`; HA package templates |
| **Schedule mode number** | same as HA select index | schedule text format `zone,mode,depth,...`; `schedule_web_mode()` translates to the web digit |

Depth is carried separately everywhere as eighths of an inch (`depth=1..8`).
Zone ids: `/zone/water` is 0-based, schedule entries are 1-based — never
touch that boundary.

## 1. Firmware core (`components/irrigoto/irrigoto.c`)

1. **Pick the next web mode digit** and add it to the parser in
   `zone_water_handler()` (search `b423: '8' = snake`-style comments for the
   mode ladder). Add any mode-specific body params there too (serpentine
   added `speed=` and `dry=`), stored in `static` "consumed-once" variables
   like `s_web_water_depth_eighths`.
2. **`phase_water_zone()` dispatch**: extend the `sel` character mapping,
   add a `bool <mode>_mode` flag, and set `passes`.
3. **Feature gates** — grep for `smooth_mode || gentle_mode` and decide for
   each whether your mode joins it:
   - pressure-trace reset and save (`water_trace_reset` / `water_trace_save`)
   - Watering-Quiet + task priority 15
   - watering log setup: deferred wbin open, `storage_make_room` size,
     aggregate accumulator arm + flush (`s_smooth_accum_mode`)
   - cumulative-depth branch of the per-ring depth compute
   - the completion-message chain (add an explicit branch — do **not** let
     your mode fall into the pulse `else`, which runs cleanup passes)
4. **The mode itself**: either a branch inside `phase_water_zone` that
   `goto abort`s into the shared closeout (serpentine does this — the
   preamble gives you cal/zone load, ring plan, supply check, logging, and
   the closeout gives you valve close, flush, score, summary for free), or
   a fully separate path (chase does this). Strongly prefer the former for
   coverage modes.
5. **Motion/task rules** (hard-won, do not relax): watering motion runs in
   the existing water task on **PRO_CPU**; no file I/O while motors run
   (RAM accumulator only, flush at run end); never stream logs during a
   wet run — use the `/zone/last_log` RAM capture.

## 2. Status & labels (`irrigoto.c` + `irrigoto.cpp`)

- `irrigoto_get_status()` — the `mname` shown in "Watering <zone> (<mode>)".
- `irrigoto_last_water_mode_label()` — the completion-event label.
- `irrigoto_get_mode()` — web digit → HA select index.
- `irrigoto.cpp`: the `labels[]` array in the select publish **and**
  `IrrigotoModeSelect::control()` string→index mapping.

## 3. Device web UI (`components/irrigoto/html/`)

Edit the **`.html` sources**, then run `python components/irrigoto/html/regen.py`
(verify with `--check`). Never hand-edit the generated `*_html.h`.

- `landing.html` — the water-modal `mode-btn` grid (`data-mode="<digit>"`).
- `schedule.html` — `MODE_LABELS` array **and** the per-entry mode
  `<select>` options.

## 4. Schedule (`irrigoto.c`)

- `schedule_web_mode()` — schedule mode number → web digit.
- **Both** schedule text parsers' range checks (`m < 0 || m > N` appears
  twice). This is the one that bites: an out-of-range mode **rejects the
  entire schedule batch atomically**, so a stale bound silently kills every
  schedule push that contains your mode.
- `schedule_estimate_duration_min()` — a per-⅛″ base estimate for your mode
  (measured history overrides it after the first completed run).

## 5. Home Assistant packages (`homeassistant/packages/`)

Three files, and the mode tables are *not* shared between them:

- `irrigoto_device.yaml` — quick-water `input_select` options + the
  `mode_digit` Jinja mapping in the `water_now` script.
- `irrigoto_common.yaml` — the `shell_command` doc comment.
- `irrigoto_schedule.yaml` — **six** sites: compose `input_select` options,
  one label→number map in the compose-save script, two number→label
  `MODES` arrays (device→HA mirror decode), and two label→number `MODES`
  maps (push encode). Grep `Pulse` in the file and update every hit.

Then `python tools/ha-regen.py`, copy `homeassistant/generated/packages/*.yaml`
into the HA `config/packages/`, and reload.

## 6. Build & validation discipline

- One concern per build: bump `FW_BUILD` in `components/irrigoto/fw_version.h`,
  `python -m esphome compile esphome/irrigoto.yaml` must succeed **before**
  committing, subject line `Build <N> <summary>`.
- For motion modes, add a **dry-rehearsal path** (run the full motion with
  the valve held closed, no logs written) and validate geometry against the
  zone polygon before the first wet run. Serpentine's `dry=1` caught a
  wet-turn polygon excursion and a log-truncation bug before any water flew.
- Validate the full matrix: web UI button, HA quick-water, a schedule entry
  that fires, cancel/stop-all mid-run, and the completion event label.

## Quick grep checklist

Before calling it done, search the repo for an existing mode name and make
sure your mode appears alongside it at every hit:

```
grep -rn "Smooth" components/irrigoto/ homeassistant/packages/ \
  --include="*.c" --include="*.cpp" --include="*.h" \
  --include="*.html" --include="*.yaml"
```

Roughly ten distinct touchpoints should show up. If one of them doesn't
mention your mode, you've found the next bug report.
