# irrigoto

ESP-IDF + ESPHome firmware for the OtO sprinkler. Provides zone-based
watering with multiple spray modes (smooth / pulse / gentle / chase),
pressure and throw calibration, scheduling, OTA updates, a built-in web
UI, and integrates with Home Assistant via the native ESPHome API. A
Lovelace heatmap card visualizes per-zone watering depth.

## ⚠ Use at your own risk

This is hobbyist firmware that controls water-handling hardware. Before you
flash anything onto a device or follow any procedure here, understand that:

- **No warranty, no fitness for any purpose.** See `LICENSE`. The code, the
  schematics implied by the code, and the documentation are provided AS IS.
- **Water damage is real.** Misconfigured zones, stuck valves, bad calibration,
  or a bug in this firmware can leave water running. Do not deploy without a
  manual shutoff you can reach quickly, and do not run unattended until you
  have observed the device through several complete cycles.
- **Electrical safety is on you.** If you wire anything to mains, GFCI-protect
  it and have a licensed electrician confirm your work. Low-voltage parts of
  this project are not a substitute for that.
- **Plant and property loss.** Over- or under-watering can damage landscaping;
  pressurized leaks can damage structures. The author has no liability for
  any of this — by using the code you accept that risk.
- **Test in a safe configuration first.** Run the device on a bench with the
  outlet pointed somewhere harmless before connecting it to your plumbing.

If any of the above is not OK with you, do not use this code.

## Install via Home Assistant ESPHome Builder

The fastest path for most users. The ESPHome Builder add-on
([install instructions](https://esphome.io/guides/installing_esphome.html#installation-from-home-assistant-add-on-store))
will compile and flash the firmware entirely from the HA UI.

1. **Copy two files into `/config/esphome/`** on your HA host:
   - [`example.yaml`](example.yaml) — the device config (rename to anything
     you like, e.g. `irrigoto.yaml`)
   - [`irrigoto-partitions.csv`](esphome/irrigoto-partitions.csv) — the 8 MB
     OTA partition table; must sit next to the device yaml

2. **Add the secrets to `/config/esphome/secrets.yaml`** (create the file
   if it doesn't exist):
   ```yaml
   api_key:        "<openssl rand -base64 32>"  # ESPHome <-> HA encryption
   ota_password:   "<any string>"               # OTA upload password
   wifi_ssid:      "<your WiFi SSID>"
   wifi_password:  "<your WiFi password>"
   ap_password:    "<8+ chars>"                  # Fallback hotspot pwd
   ```

3. **Open the device yaml in ESPHome Builder and click Install.** The
   first flash needs a USB-UART adapter (Builder prompts for the port);
   subsequent flashes go over WiFi automatically via OTA.

`example.yaml` tracks the `main` branch by default. To pin to a specific
firmware build, edit both `ref: main` lines and point them at a release tag
like `ref: v392`. Each published release is tagged `v<build>`; tags are
immutable, while `main` is a flattened snapshot replaced on each release.

After the device comes online, set up the Home Assistant side — see
[Home Assistant integration](#home-assistant-integration) below for the
template sensors, automations, schedule sync, and per-zone heatmap.

## Local development (compile from a clone)

For modifying the firmware itself rather than just deploying a release.

### Prerequisites

- Windows 10/11 build host (Linux/macOS work too with the obvious shell
  substitutions)
- ESPHome installed in its own Python venv. Install from the pinned
  `requirements.txt`, not a bare `pip install esphome` — the latter
  re-resolves to whatever is newest, which has shipped regressions that
  only surface at runtime (e.g. a 2026.5.x post-OTA HA-reconnect crash):
  ```powershell
  python -m venv C:\esphome-env
  C:\esphome-env\Scripts\pip install -r requirements.txt
  ```
- For the initial flash, a USB-UART adapter connected to the device at
  J2 (GPIO16=RxD, GPIO17=TxD, GND). Subsequent flashes use ESPHome OTA
  over WiFi automatically.
- GPIO0 accessible to pull low during reset for bootloader entry.

### Project structure

```
irrigoto/
├── example.yaml                       <- end-user device wrapper (type: git)
├── esphome/
│   ├── irrigoto.yaml                  <- dev wrapper (type: local)
│   ├── irrigoto-core.yaml             <- canonical config, shared by both
│   └── irrigoto-partitions.csv        <- custom 8 MB OTA partition table
├── components/
│   └── irrigoto/                      <- ESPHome external component
│       ├── irrigoto.c                 <- main firmware
│       ├── irrigoto.cpp               <- C++ ESPHome wrapper
│       ├── irrigoto.h
│       ├── irrigoto_api.h             <- C ↔ C++ interface
│       ├── i2c_bus.c/.h               <- I2C driver
│       ├── storage.c/.h               <- LittleFS persistence
│       ├── html/                      <- web UI HTML fragments
│       │   ├── *.html                 <- editable sources
│       │   └── *_html.h               <- R"(...)" #include payloads
│       └── *.py                       <- Python ESPHome entity definitions
├── homeassistant/
│   ├── packages/                      <- HA package YAML (template sensors,
│   │                                     automations, shell_commands)
│   ├── dashboards/                    <- HA dashboard YAML
│   ├── cards/                         <- HA Lovelace card YAML
│   └── lovelace/
│       └── irrigoto-heatmap-card.js   <- custom heatmap card
└── LICENSE
```

### Build & flash

Create `esphome/secrets.yaml` with the same keys listed in the Builder
section above (this file is gitignored — never commit it).

Then compile and flash from plain PowerShell (not the ESP-IDF prompt):

```powershell
C:\esphome-env\Scripts\esphome compile esphome\irrigoto.yaml
C:\esphome-env\Scripts\esphome run     esphome\irrigoto.yaml
```

`esphome run` compiles if needed and then OTA-pushes to the device
(resolved by mDNS). For the very first flash to a brand-new device,
ESPHome falls back to USB and prompts for a serial port.

The dev wrapper uses `external_components: type: local` pointing at
the sibling `components/` directory, so local edits compile without a
git push. End-user `example.yaml` uses `type: git` against this repo.
Both share `irrigoto-core.yaml` via the `packages:` mechanism, so adding
a new HA service or sensor only needs to be edited in one place.

Once flashed, the device announces itself to Home Assistant via the
native ESPHome API. See [Home Assistant integration](#home-assistant-integration)
to load the template sensors, automations, and heatmap dashboard.

## Home Assistant integration

The HA-side config — template sensors, automations, schedule sync, and the
per-zone heatmap dashboard — is generated per-fleet from a manifest. The
files under `homeassistant/packages/` and `homeassistant/dashboards/` are
**templates** with `<<DEV_*>>` placeholders and will NOT load as-is; run the
generator first.

1. Describe your devices. Copy the example manifest and edit it:
   ```bash
   cp homeassistant/devices.example.yaml homeassistant/devices.yaml
   ```
   One entry per device — `slug` (the ESPHome node name, e.g.
   `irrigoto-ab12cd`), a friendly `name`, and the device `url`.

2. Generate the personalized config:
   ```bash
   python tools/ha-regen.py
   ```
   This writes `homeassistant/generated/packages/*.yaml` and
   `homeassistant/generated/dashboards/irrigoto.yaml`.

3. Copy the generated files into Home Assistant:
   - `homeassistant/generated/packages/*.yaml` → your HA `config/packages/`
     (with `homeassistant: packages: !include_dir_named packages` enabled)
   - `homeassistant/generated/dashboards/irrigoto.yaml` → a dashboard
   - register `homeassistant/lovelace/irrigoto-heatmap-card.js` as a
     Lovelace resource for the per-zone watering heatmap

Re-run `python tools/ha-regen.py` whenever you add or rename a device.

## Entering bootloader mode (USB flash only)

To flash the ESP32 over USB:
1. Hold GPIO0 LOW (connect to GND)
2. Press and release RESET (EN pin)
3. Release GPIO0
4. Run the flash command

OTA flashes don't need this — the device handles them in firmware.

## Hardware reference

A full pin-by-pin summary of the OtO control board (ESP32 GPIO
assignments, I2C bus, motor drivers, ADC channels, RGB LED register
values, J2 flash header, WROOM module pinout) lives at
[`docs/oto_pin_summary.md`](docs/oto_pin_summary.md). Refer to it
when modifying the firmware's GPIO usage or wiring new sensors.

## Expected I2C devices

| Address | Device | Function |
|---------|--------|---------|
| 0x18 | MPRLS | Water pressure |
| 0x20 | TCA6408A | GPIO expander / LEDs |
| 0x36 | AS5600 | Nozzle position |
| 0x40 | AS5600L | Valve position |

## Battery voltage calibration

`VBATT_DIVIDER_RATIO` in `components/irrigoto/irrigoto.c` defaults to 2.0.
Measure the two resistors of the VBattRaw divider near J5 and set the
correct ratio before trusting voltage readings.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull
request with your improvements.

## License

Apache License 2.0. See [`LICENSE`](LICENSE) for the full text, including
the warranty disclaimer and limitation of liability.
