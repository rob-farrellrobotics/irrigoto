#!/usr/bin/env bash
# ha-regen.sh — compatibility shim.
#
# The HA config generator is now tools/ha-regen.py and is driven by the
# fleet manifest homeassistant/devices.yaml (not a slug argument). This
# shim just forwards to it so old muscle memory keeps working.
#
#   python tools/ha-regen.py              ← the real command
#
# Add/rename/remove devices by editing homeassistant/devices.yaml, then
# re-run. See homeassistant/README.md.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

if [[ $# -gt 0 && "$1" != --* ]]; then
  echo "note: ha-regen no longer takes a device slug argument." >&2
  echo "      Devices live in homeassistant/devices.yaml now; '$1' is ignored." >&2
  shift || true
fi

exec python "$here/ha-regen.py" "$@"
