# Contributing to irrigoto

Thanks for your interest! A note on how this repository is maintained, because
it isn't a conventional layout.

## How this repo is published

`main` is a **snapshot release branch**. Each commit on `main` is a complete,
flattened snapshot of one firmware build — it does **not** carry the granular,
per-build development history. That history lives on a private development
branch and is intentionally not published (early commits contained build
artifacts with embedded Wi‑Fi credentials; rather than rewriting that history,
we publish clean snapshots instead).

Practically, that means:

- Every release adds **one** commit to `main` (parented on the previous
  release) and a matching `v<build>` tag. `main` only ever moves forward — it
  is **never force-pushed**, so your clones and forks always fast-forward
  cleanly.
- A single release commit will look like a large squashed diff. That's
  expected — it's the sum of all changes since the previous build.
- Pin to a specific build with the tag (e.g. `ref: v402`), not to a commit
  SHA you scraped from `main`.

## Submitting changes

1. Open an issue or PR against `main` describing the change.
2. Because development happens on the private branch, an accepted change is
   **re-applied there by a maintainer** and then appears in the next published
   snapshot — your PR may be closed with a reference to the release that
   includes it rather than merged directly. You'll still be credited.
3. Keep changes focused; firmware changes should compile
   (`python -m esphome compile esphome/irrigoto.yaml`).

## Embedded web UI pages

The device's web pages live in `components/irrigoto/html/`. Each page is a
pair: the `.html` file (source of truth — edit this one) and a generated
`*_html.h` (the C++ raw-string literal the firmware embeds). After editing a
`.html`, run:

```
python components/irrigoto/html/regen.py
```

and commit both files. `regen.py --check` verifies every pair is in sync.
Never edit the `*_html.h` files directly — past hand-edits let the pairs
drift until regenerating a header silently reverted shipped features.

## Secrets

Never commit real credentials. Per-user secrets are gitignored and provided via
examples:

- `esphome/secrets.yaml` — Wi‑Fi / API / OTA secrets (see ESPHome docs). Not
  tracked.
- `homeassistant/devices.yaml` — your device fleet (copy from
  `devices.example.yaml`). Not tracked.

Maintainer release tooling scans every release for tracked secret files and
for any value from a local `secrets.yaml` before pushing, and aborts if it
finds one.
