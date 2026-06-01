# OtO Smart Sprinkler

_ESP32 Pin & Hardware Summary_

_MCU: ESP32-WROOM-32UE (8MB flash)_

## Digital Outputs — Power Control

| GPIO | WROOM Pin | Signal | Function | Active |
|---|---|---|---|---|
| GPIO4 | 26 | 3V3Sen | Sensor rail enable (AS5600s, MPRLS, TCA6408A, I2C pullups) | HIGH |
| GPIO18 | 35 | 9V_EN | Motor rail enable (9V H-bridge supply) | HIGH |
| GPIO13 | 16 | K (Kill) | Motor brake / kill signal | LOW |

## Digital Outputs — Motor Drive (H-Bridge)

| GPIO | WROOM Pin | Signal | Function | Notes |
|---|---|---|---|---|
| GPIO22 | 36 | vFwd | Valve motor forward | PWM drive — decreases angle |
| GPIO25 | 10 | vRev | Valve motor reverse | PWM drive — increases angle |
| GPIO26 | 11 | nFwd | Nozzle motor forward | PWM drive |
| GPIO27 | 12 | nRev | Nozzle motor reverse | PWM drive |

PWM: 20kHz (10MHz clock, 500 tick period). Direction: one pin PWM, opposite pin held LOW.

## I2C Bus

| GPIO | WROOM Pin | Signal | Function |
|---|---|---|---|
| GPIO17 | 28 | SDA | I2C data |
| GPIO23 | 37 | SCL | I2C clock (shared with Hall wake pin) |

Bus: I2C_NUM_0, 400kHz, powered by 3V3Sen rail (GPIO4).

### I2C Devices

| Address | Device | Function | Notes |
|---|---|---|---|
| 0x40 | AS5600L (U8) | Valve angular position sensor | 12-bit, 0-360° |
| 0x36 | AS5600 (U1) | Nozzle angular position sensor | 12-bit, 0-360° |
| 0x08 | MPRLS | Water pressure sensor | 0-25 PSI gauge |
| 0x20 | TCA6408A (U2) | GPIO expander — RGB LEDs |  |

## ADC Inputs (ADC1)

| GPIO | WROOM | ADC Ch | Signal | Function | Notes |
|---|---|---|---|---|---|
| GPIO36 | 4 | CH0 | VBattRaw | Battery voltage (÷2 divider) | ~4.1V full charge |
| GPIO39 | 5 | CH3 | Charge | Solar/charge current (placeholder) | Reads idle baseline — BQ25504 uses internal GPIO38 |
| GPIO32 | 8 | CH4 | VCur | Valve motor current sense | INA4180A3, 50mΩ, gain 100 |
| GPIO34 | 6 | CH6 | PCur | Pump current sense (unused) | Pump not populated |
| GPIO35 | 7 | CH7 | NCur | Nozzle motor current sense | INA4180A3, 50mΩ, gain 100 |

Current conversion: I(mA) = V(mV) × 0.2
Note: Idle baseline ~142mV = ~28mA (INA4180A3 zero-current offset). Battery must be connected for stable operation — external supply alone causes brownout on TCA6408A switching.

## Wake / Interrupt

| GPIO | WROOM Pin | Signal | Notes |
|---|---|---|---|
| GPIO23 | 37 | SCL / Hall wake | Shared with I2C SCL. Hall sensor physically interrupts power causing a full reboot rather than GPIO wake. Timer wakeup only — Hall = hard reset. |

## TCA6408A RGB LED Mapping

I2C address: 0x20   |   Off state: output register = 0xF9

### Bit Assignments

| Bit | Pin | LED | Logic | Notes |
|---|---|---|---|---|
| 0 | P0 | Red | Active LOW | Dual-drive: also driven by BQ25504 charger status pin during solar charging, even during deep sleep. CONFIRMED: red LED lights during ESP32 deep sleep when solar connected. TCA releases control via LED_OFF (0xF9) before sleep. |
| 1 | P1 | Blue | Active HIGH | Firmware only |
| 2 | P2 | Green | Active HIGH | Firmware only |
| 3-7 | P3-P7 | — | — | Auxiliary pump/valve controls, unpopulated in this unit |

### LED Color Register Values

| Colour | Register Value | Bit State |
|---|---|---|
| Off | 0xF9 | R=HIGH G=LOW B=LOW |
| Red | 0xF8 | R=LOW G=LOW B=LOW |
| Blue | 0xFB | R=HIGH G=LOW B=HIGH |
| Green | 0xFD | R=HIGH G=HIGH B=LOW |
| Yellow | 0xFC | R=LOW G=HIGH B=LOW |
| Cyan | 0xFF | R=HIGH G=HIGH B=HIGH |
| Purple | 0xFA | R=LOW G=LOW B=HIGH |
| White | 0xFE | R=LOW G=HIGH B=HIGH |

## Power System

| Component | Details |
|---|---|
| Battery | LiPo, ~4.1V full, connected via ÷2 voltage divider to GPIO36 |
| Solar panel | 6.5V → BQ25504 boost charger input |
| Charger | BQ25504 — accepts input from ~0.1V, charges LiPo autonomously. Status pin drives red LED independently during charging, including during deep sleep. CONFIRMED: red LED active during ESP32 deep sleep with solar connected. |
| Motor rail | 9V, enabled via GPIO18 (9V_EN HIGH) |
| Sensor rail | 3.3V, enabled via GPIO4 (3V3Sen HIGH) |
| GPIO38 (internal) | Not broken out on WROOM module. Used by original firmware for charge_mA telemetry (BQ25504 monitoring). Inaccessible. |

## J2 UART / Flash Connector

5-pin connector, sensor side of main board, bottom centre. Used for initial UART flashing and serial monitoring. Once custom firmware with OTA is installed, J2 is only needed to recover a bricked device.

| Pin | Signal | Connect to | Notes |
|---|---|---|---|
| 1 | GND | Adapter GND | Ground reference |
| 2 | EN | Not required | Reset — not needed for normal flashing once OTA is running |
| 3 | IO0 | GND (at power-up only) | Pull LOW during power-up to enter bootloader mode. Release after boot. |
| 4 | TX | Adapter RX | ESP32 transmit → PC receive |
| 5 | RX | Adapter TX | PC transmit → ESP32 receive. Red wire on known cable. |

Minimum wiring for flashing: GND (pin 1), adapter RX→TX (pin 4), adapter TX→RX (pin 5). Bridge pin 3 to GND before applying power, release after boot to enter bootloader mode.
Access note: the J2 cable is accessible from the underside of the sprinkler unit without full disassembly. The red wire identifies pin 5 (RX) — this is the key reference point since pin 1 (GND) would normally be expected to carry the red wire by convention.

## WROOM-32 Module Pin Reference

```
          ┌─────────────────────┐
GND       │  1              38  │ GND
3V3       │  2              37  │ GPIO23  SCL / Hall wake
EN        │  3              36  │ GPIO22  vFwd (valve fwd)
GPIO36 VP │  4              35  │ GPIO21  (unused)
GPIO39 VN │  5              34  │ GPIO20  (unused)
GPIO34    │  6              33  │ GPIO19  (unused)
GPIO35    │  7              32  │ GPIO18  9V_EN
GPIO32    │  8              31  │ GPIO5   (unused)
GPIO33    │  9              30  │ GPIO17  SDA
GPIO25    │ 10              29  │ GPIO16  (unused)
GPIO26    │ 11              28  │ GPIO4   3V3Sen
GPIO27    │ 12              27  │ GPIO0   (boot)
GPIO14    │ 13              26  │ GPIO2   (unused)
GPIO12    │ 14              25  │ GPIO15  (unused)
GND       │ 15              24  │ GND
GPIO13    │ 16              23  │ NC
GPIO9     │ 17              22  │ NC
GPIO10    │ 18              21  │ NC
GPIO11    │ 19              20  │ NC
          └─────────────────────┘
```
