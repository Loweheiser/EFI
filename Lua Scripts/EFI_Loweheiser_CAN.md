# EFI_Megasquirt_CAN — MegaSquirt EFI Driver (CAN bus)

Lua scripting driver for ArduPilot that receives real-time engine telemetry from
**MegaSquirt 2 / MicroSquirt / MS2Extra** ECUs over the CAN bus using the
**MS-Extra Advanced Realtime Broadcast** protocol.

## Supported hardware

| ECU | Firmware | CAN mode |
|-----|----------|----------|
| MegaSquirt 2 | MS2Extra 3.x | Advanced Realtime Broadcast |
| MicroSquirt | MS2Extra 3.x | Advanced Realtime Broadcast |
| MS3 | MS3 firmware | Advanced Realtime Broadcast |

Standard CAN, 11-bit IDs, 500 kbit/s, big-endian encoding.

## What it does

The script registers as an ArduPilot EFI scripting backend (`EFI_TYPE = 7`) and
decodes the following OutPC groups broadcast by the ECU:

| CAN ID (hex) | Group | Variables decoded |
|---|---|---|
| `BASE_ID + 0` | G0 | RPM, injector pulse-width 1 & 2 |
| `BASE_ID + 2` | G2 | Barometric pressure, MAP, intake air temp (MAT), coolant temp (CLT) |
| `BASE_ID + 3` | G3 | Battery voltage, AFR1, AFR2 |

Decoded values are published via `EFI_State` and `Cylinder_Status` and appear in:

- MAVLink `EFI_STATUS` message (#225)
- ArduPilot `.bin` flight logs (`EFI` message)
- Any GCS that supports EFI telemetry (Mission Planner, QGroundControl, etc.)

## Parameters

The script registers a parameter table with the prefix **`EFI_MS_`**:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EFI_MS_ENABLE` | 0 | Set to **1** to enable the driver |
| `EFI_MS_CANDRV` | 1 | CAN driver instance (1 or 2) |
| `EFI_MS_MODE` | 1 | Protocol mode — **1** = Advanced Realtime Broadcast (recommended) |
| `EFI_MS_BASE_ID` | 1520 | Base CAN message ID (decimal). Must match TunerStudio setting |
| `EFI_MS_RATE_HZ` | 50 | Script update rate (Hz) |
| `EFI_MS_BARO_KPA` | 101.3 | Reference baro pressure used in mode 0 |
| `EFI_MS_OPTIONS` | 0 | Bit 0: log raw CAN frames in .bin log |

## ArduPilot parameters required

```
SCR_ENABLE      = 1
SCR_HEAP_SIZE   = 90000    # minimum; increase if other scripts are loaded
EFI_TYPE        = 7
CAN_P1_DRIVER   = 1
CAN_D1_PROTOCOL = 10       # Scripting
CAN_P1_BITRATE  = 500000
```

## Installation

1. Copy `EFI_Megasquirt_CAN.lua` to the `APM/scripts/` directory on the autopilot SD card.
2. Set the parameters above and reboot.
3. After the first boot the `EFI_MS_*` parameters will appear — set `EFI_MS_ENABLE = 1` and reboot again.

## TunerStudio configuration

In TunerStudio go to **CAN Bus / Testmodes → CAN Realtime Data Broadcasting**:

- Enable: **On**
- Base message identifier: **1520** (decimal)
- Broadcasting rate: **50 Hz** (minimum 20 Hz)
- Enable groups: **00**, **02**, **03**
- Click **Burn**

> **Do not** use the classic CAN Broadcasting screen (0x280 / 0x316 etc.) — it uses
> a different ID scheme that is not decoded by this driver.

## CAN bus wiring

```
ECU                              Autopilot
CAN_H ──────────────────────── CAN_H
CAN_L ──────────────────────── CAN_L
GND  ──────────────────────── GND
[120 Ω]                        [120 Ω internal on Pixhawk 4]
```

Total bus resistance between CAN_H and CAN_L (power off) must be **~60 Ω**.

> Pixhawk 4 has an **internal** 120 Ω terminator. Do **not** add a second resistor
> at the autopilot end.

## Verification

With the script loaded and the ECU broadcasting, the Messages tab in Mission Planner
should show:

```
EFI: MegaSquirt CAN loaded
```

Live data is visible in **MAVLink Inspector → EFI_STATUS (#225)**. Temperature fields
are in Kelvin (subtract 273.15 for °C).

## Tested with

- ArduCopter 4.6.3 on Pixhawk 4 (STM32F765)
- MS2Extra 3.4.x, Advanced Realtime Broadcast, base ID 1520
- CAN bus at 500 kbit/s

## Notes

- The driver is **receive-only**; it does not write to the ECU.
- Groups G1 and G22 (ignition timing, gear) are parsed but not yet mapped to
  ArduPilot EFI fields; contributions welcome.
- CLT and MAT sensors must be connected to the ECU — open-circuit sensors report
  0 °F / -17.8 °C which is passed through as-is.
