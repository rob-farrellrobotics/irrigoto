# Bidirectional HA ↔ device schedule sync — design

Status: design doc, not yet implemented (2026-05-26).

## Goals

- One canonical schedule, consistent in HA and on the device.
- Both surfaces can edit (HA dashboard + device web UI).
- Device runs autonomously when HA is offline; the weekly schedule it
  has continues firing indefinitely.
- HA can hold richer logic (weather, soil, calendar) and adjust the
  schedule; updates flow to the device on next reconnect.
- Both sides know when there's a pending update in either direction.

## Conflict resolution rule

**Last-writer-wins by per-entry `last_modified` timestamp. Device wins
on tie.**

In practice ties are vanishingly rare — they'd require two edits in the
same wall-clock second from two surfaces. The tiebreak rule exists for
determinism, not to express a value judgement.

## Entry schema

Each schedule entry grows three new fields:

| Field | Type | Notes |
| --- | --- | --- |
| `id` | uint32 | Sequential, stable across edits, assigned at creation. Persisted in NVS with the entry. |
| `last_modified` | uint32 | Unix epoch when this entry was last edited. Updated on any field change. |
| `source` | uint8 | Reserved: who set this entry. `0`=unknown, `1`=user_device, `2`=user_ha, `3`=algorithm. Persisted + sync'd but no behavior wired up yet. |
| _(existing)_ zone, mode, depth, hour, minute, days_mask, enabled | uint8 ×7 | Unchanged. |

Total entry size: 7 + 4 + 4 + 1 = 16 bytes. With current `SCHEDULE_MAX_ENTRIES`
the NVS blob grows by ~9 bytes × N entries — negligible.

A second uint32 ID counter (`schedule_id_next`) is persisted alongside
the schedule blob so freshly-created entries always get a unique ID
even across reboots.

## Schedule version counter

A device-wide `schedule_version` (uint32, persisted in NVS) increments
on every commit to the schedule, regardless of the source. Surfaced
as `sensor.<<DEV_USC>>_irrigoto_schedule_version` (a regular ESPHome
sensor — auto-synced on HA reconnect, no event-loss problem).

HA stores `last_seen_schedule_version` per device. Comparing the two
gives the device→HA pending indicator with no event-replay logic.

## Firmware additions

### NVS

- New key `sched_ver` (uint32) — schedule_version
- New key `sched_idnxt` (uint32) — next ID to assign
- Existing key `schedule` blob — entry struct grows to 16 bytes/entry

### ESPHome services

| Service | Purpose | Behavior |
| --- | --- | --- |
| `set_schedule(entries)` | Existing, kept for back-compat. | Legacy REPLACE-ALL (old CSV format, no IDs). Deprecated; only used by tools that haven't been updated. |
| `sync_schedule(entries)` | New. Merge HA-proposed entries into device's schedule using LWW + device-wins-tiebreak. | See protocol below. |
| `clear_schedule()` | Existing. | Wipe all. Increments schedule_version. |
| `delay_schedule_hours(h)` | Existing. | Rain delay; doesn't touch entries, doesn't bump version. |

The new `sync_schedule` payload format (one entry per `;`, fields by `,`):

```
<id>,<last_modified>,<zone>,<mode>,<depth>,<hour>,<minute>,<days_mask>,<enabled>,<source>,<tombstone>
```

`tombstone=1` means "delete this ID if present, else no-op." Lets HA
express deletions without requiring "absence implies delete" — important
because the new sync_schedule must NOT delete device-only entries that
HA doesn't know about.

`id=0` means "no ID assigned yet, allocate one." The device assigns the
next sequential ID and includes it in the next schedule_changed event so
HA can learn it.

### Sensors / events

- `schedule_version` text_sensor (new) — uint32 as string. Auto-synced
  to HA on API reconnect.
- `schedule_text` text_sensor (existing) — kept; format extended to
  include id + last_modified per entry so HA can parse without a
  separate fetch.
- `schedule_changed` event (existing) — payload extended to include
  `schedule_version` (so HA can update last_seen_version inline) and
  the structured entries-with-IDs blob.

## HA-side additions

### Storage

Keep the per-slot `input_text` model from Stage 6. Each slot's text
format grows to include id + last_modified:

```
<id>,<last_modified>,<dev-slug>|<zone>,<mode>,<depth>,<hour>,<minute>,<days_mask>,<enabled>
```

Empty slot = unused. The template sensor that aggregates slots also
strips/parses the new fields.

A new `input_text.irrigoto_<<DEV_USC>>_last_seen_version` (per device,
small int) holds HA's last-known schedule_version for that device.

### Compose form

When the user saves to a slot from the form:
- If slot was empty → assign a placeholder `id=0` (device allocates real ID on first sync)
- If slot had an entry → keep the existing id
- `last_modified` set to `now() | int` (unix epoch)

### Reconciler

Triggered on:
- Slot edit (any `input_text.irrigoto_sched_*` state change)
- Device online edge
- Device schedule_version sensor change

Logic:
1. If HA's view of `last_seen_version` differs from device's reported
   `schedule_version`: HA is behind. Fetch device's state via the
   existing `schedule_text` sensor (or `schedule_changed` payload),
   reconcile entries by ID, update HA's slots for any device-newer
   entries. Update `last_seen_version`.
2. Compute the delta: HA's current entries vs the now-up-to-date
   `last_seen` snapshot. Any HA entry with newer `last_modified` than
   device's view is a push candidate. Any device entry no longer in
   HA (user cleared the slot AND it was a HA-tracked ID) is a
   tombstone candidate.
3. Build the sync_schedule payload with push candidates + tombstones
   + any new (id=0) entries.
4. If payload is non-empty: call `esphome.<<DEV_USC>>_sync_schedule`.
5. Device responds via schedule_changed event with new version.

### Pending indicators (dashboard)

Two badges, surfaced separately so the user can tell which direction
is unsync'd:

| `sensor.<<DEV_USC>>_schedule_pending_outbound` | HA → device |
| --- | --- |
| `synced` | No HA-side edits pending |
| `pending - device asleep` | HA has changes, device offline |
| `pending - pushing` | HA pushed, awaiting confirmation |
| `push failed: <reason>` | Last sync_schedule call errored |

| `sensor.<<DEV_USC>>_schedule_pending_inbound` | device → HA |
| --- | --- |
| `synced` | `last_seen_version == schedule_version` |
| `pending - pulling` | `schedule_version > last_seen_version`, HA fetching |
| `pending - HA offline` | (computed externally) HA was off when device fired event; same root condition surfaces on HA boot |

## Migration

### Device firmware

On first boot of the new firmware:
- For each existing entry in the NVS blob, assign sequential IDs
  starting from 1.
- Set `last_modified = time(NULL)` (or 0 if time not synced yet) for
  all entries.
- Set `source = 0` (unknown) for all.
- Initialize `sched_idnxt = N+1` where N is the highest assigned ID.
- Initialize `schedule_version = 1`.

After this, the new schema is in place. Old blobs auto-upgrade on
first boot.

### HA package

On first run:
- `last_seen_version` per device starts at 0.
- The first reconciliation pulls device's current schedule (since
  device's version is >= 1) and populates HA's slots with the IDs
  device assigned.
- No data loss — device's pre-existing schedule is preserved, HA's
  view comes into alignment.

If the user already has HA-side slots populated under the old
schema, those entries have `id=0` and current `last_modified`, so
they'll push as new on first sync.

## Out of scope for this design

- Multi-device automation that depends on cross-device awareness
  (overlap checking already exists; not changed by this work).
- Algorithmic schedule generation (weather, soil) — design accommodates
  it via the `source` field, but no implementation here.
- Schedule history / undo. Single-revision; no audit trail of who
  changed what when (beyond `last_modified` + `source`).
- Compression of the wire format for very large schedules. Current
  CSV with the new fields stays under 100 chars per entry; that's
  fine for the SCHEDULE_MAX_ENTRIES we have.

## Open questions for implementation

1. Should `time(NULL)` returning < epoch threshold (pre-NTP-sync)
   block sync? Or accept stale timestamps and let LWW be wrong for
   a few minutes after wake? Probably accept — bias toward "always
   sync something" over "wait for clock."

2. If the device's NVS schedule blob fails to parse on boot (corruption),
   today the code resets `s_schedule.count = 0`. With the new schema,
   should we attempt to recover individual entries that look valid?
   Probably no — pre-existing behavior is fine; corrupt blob = empty
   schedule; user re-adds.

3. The HA-side parse of `schedule_text` to learn device's current state
   only works if `schedule_text` includes the IDs and timestamps. If
   the text grows beyond the existing 240-char limit on text_sensor
   state, we lose data. Mitigations: bump the limit (ESPHome allows
   up to ~64KB), or add a separate "schedule_export" service that
   dumps the full blob to a `schedule_full_export` text_sensor on
   demand. Decide during implementation.
