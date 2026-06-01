# Flashing a new OtO unit with irrigoto

First-time flash procedure for a stock OtO Smart Sprinkler — backs up the
factory firmware first so you can revert if anything goes wrong, then erases
the chip and writes the irrigoto build via USB-UART.

Subsequent updates use the device's OTA path (`esphome run` finds it on
mDNS); this doc is only for the very first flash.

## Prerequisites

**Hardware**
- USB-UART adapter, 3.3V logic (FTDI, CP2102, CH340, any of those work)
- J2 jumper cable for the OtO (5-pin, accessible from the underside of the
  unit without full disassembly — see [`oto_pin_summary.md`](oto_pin_summary.md#j2-uart-flash-connector)
  for pinout)
- The OtO unit with battery connected (external supply alone causes
  brownout on the TCA6408A during flashing)

**Software** (on the PC doing the flash)
- ESPHome installed in a Python venv. The repo uses
  `C:\esphome-env\Scripts\esphome` per the README; adjust paths if yours
  differs.
- `esptool.py` ships with ESPHome's pip install. Reachable via
  `python -m esptool` if `esptool` isn't on PATH.
- A clone of this repo with `esphome/secrets.yaml` populated (WiFi creds
  + ESPHome API key). The new device will join your WiFi using these.

## Step 1: Wire up J2

Per the [pin summary](oto_pin_summary.md#j2-uart-flash-connector):

| J2 pin | Signal | Adapter |
| ------ | ------ | ------- |
| 1 | GND | GND |
| 3 | IO0 | GND (only at power-up, see Step 3) |
| 4 | TX | RX |
| 5 | RX | TX (red wire on the OtO cable) |

EN (pin 2) is not normally needed — power-cycle works as reset.

## Step 2: Identify the COM port

Plug in the adapter (before connecting to J2) and check **Device Manager →
Ports (COM & LPT)**. Note the new port — typically `COM3` through `COM8`
on Windows. Subsequent commands assume `COM4`; substitute yours.

You can also run:

```powershell
python -m esptool --port COM4 chip_id
```

If the adapter is alive and connected to a powered-down OtO, this errors
with "Failed to connect" — that's normal (no chip on the wire yet). It
confirms the port is the adapter.

## Step 3: Enter bootloader mode

Per the [README bootloader section](../README.md#entering-bootloader-mode-usb-flash-only):

1. Connect J2 pin 3 (IO0) to GND
2. Apply power to the OtO (battery in / button press)
3. Release IO0 from GND

The device is now sitting in the ROM bootloader waiting for esptool. You
have to do this once per `esptool`/`esphome` invocation — power-cycling
the device exits bootloader mode.

## Step 4: Back up the factory firmware

Read the entire 8 MB flash to a local file before erasing anything. If you
ever want to revert this unit to the original OtO firmware, this image is
your only path back.

```powershell
python -m esptool --port COM4 --baud 460800 read_flash 0x000000 0x800000 oto_factory_<MAC>.bin
```

Replace `<MAC>` with the MAC suffix of this unit (you'll see it scroll past
in the connect handshake — last 6 hex chars). Naming the backup by MAC
keeps multi-unit collections sane.

Expected output ends with `Hard resetting via RTS pin...` and the file
should be exactly **8388608 bytes** (8 MB). If it's smaller, the read was
incomplete — re-enter bootloader mode (Step 3) and retry.

Verify and stash the backup somewhere you'll find it later:

```powershell
Get-FileHash oto_factory_<MAC>.bin -Algorithm MD5
mkdir $env:USERPROFILE\backups\oto -Force
move oto_factory_<MAC>.bin $env:USERPROFILE\backups\oto\
```

## Step 5: Erase the flash

Wipes nvs/otadata/partitions/littlefs so the new partition layout from
`irrigoto-partitions.csv` takes effect cleanly.

Re-enter bootloader mode (Step 3 — every esptool invocation needs this on
this adapter), then:

```powershell
python -m esptool --port COM4 --baud 460800 erase_flash
```

Takes ~15 seconds. Ends with `Chip erase completed successfully`.

## Step 6: First-time write of irrigoto

Re-enter bootloader mode, then from the repo root:

```powershell
C:\esphome-env\Scripts\esphome run esphome\irrigoto.yaml --device COM4
```

`esphome run` compiles (if needed), then writes the bootloader, partition
table, OTA-data, and the app image at their respective offsets via
esptool, then opens a serial monitor automatically.

The first boot should show:

```
[I][irrigoto.tz] Set libc TZ to EST5EDT,M3.2.0/2,M11.1.0/2
[C][wifi]: Connecting to '<ssid>'...
[I][wifi]: WiFi Connected!
```

…followed by the API key handshake. Watch for the device's name in the
logs: it'll be `irrigoto-<MAC suffix>` (auto-derived from this unit's MAC
because `name_add_mac_suffix: true` in irrigoto-core.yaml). Note this
suffix — you'll need it for HA setup and for `ha-regen.sh`.

If WiFi join fails the device falls back to an AP named `irrigoto-<suffix>
Fallback Hotspot` — join that, browse to `192.168.4.1`, fix creds, save.

`Ctrl+C` to exit the serial monitor. Power-cycle to confirm the device
boots into the new firmware standalone.

## Step 7: Bring up in HA

The new device announces itself via mDNS. In HA: **Settings → Devices &
Services**, the ESPHome integration should show a new "Discovered"
banner — click **Configure**, paste the API key from `esphome/secrets.yaml`,
done.

For the per-device package + dashboard, run `tools/ha-regen.sh
irrigoto-<new-suffix>` to generate the second device's `.local.yaml`
sibling, then drop into `config/packages/` alongside the first device's.

(See the multi-device notes in [`homeassistant/README.md`](../homeassistant/README.md)
for the dashboard side.)

## Restoring the factory firmware

If you ever want to put this unit back to stock:

```powershell
# Re-enter bootloader mode (Step 3)
python -m esptool --port COM4 --baud 460800 erase_flash
# Re-enter bootloader mode again
python -m esptool --port COM4 --baud 460800 write_flash 0x000000 oto_factory_<MAC>.bin
```

The factory backup is for THIS unit specifically (it contains MAC-tied
calibration data and the original WiFi creds). Don't cross-flash one
unit's backup onto another.

## Troubleshooting

**"Failed to connect" during esptool**
- Bootloader mode dropped. Re-do Step 3 between every `esptool` /
  `esphome` invocation.
- Adapter TX/RX swapped — the red wire is RX (J2 pin 5), connect to
  adapter TX.
- Battery low — external supply alone causes brownout. Plug in a
  charged battery.

**`esphome run` hangs at "Connecting"**
- Same as above. The serial-port portion of `esphome run` uses esptool
  internally; same bootloader entry rules apply.

**Boots but no WiFi**
- Check `esphome/secrets.yaml` is populated and the device is in range
  of the SSID listed there.
- The fallback AP comes up after ~1 minute of failing to join the
  primary network.
