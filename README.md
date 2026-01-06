# NoNag

An EGS/NAG52 emulator for Mercedes W211 manual transmission swaps.

## Purpose

When converting a W211 (and similar Mercedes models) from automatic to manual transmission, you face several problems:

1. **Reversing lights and sensors don't work** - These are CAN-driven and expect signals from the EGS (Electronic Gearbox Control)

2. **Leaving the EGS connected causes torque limiting** - The EGS reports gearbox faults to the ECU, which responds by limiting engine power

3. **Unplugging the EGS causes other issues** - ABS faults and loss of power steering

**NoNag solves all of these.** It emulates the essential EGS CAN messages, allowing the car to operate normally with a manual gearbox - full power, working reverse lights/sensors, no ABS faults, and functioning power steering.

## Features

- Prevents ECU torque limiting
- Enables CAN-driven reversing lights and parking sensors
- No ABS faults
- Power steering works normally
- Gear position display on cluster (P/R/D)
- Driving program indicator (S)
- Automatic handbrake detection from CAN
- Status LED indicates current gear state

## Hardware

### Board
- **[Seeed Studio CANBed](https://www.seeedstudio.com/CANBed-Arduino-CAN-BUS-Development-Kit-Atmega32U4-with-MCP2515-and-MCP2551-p-4365.html)** (ATmega32U4 + MCP2515 + MCP2551)
- Arduino Leonardo compatible
- Native USB for serial debugging
- Integrated CAN transceiver - connects directly to vehicle CAN bus
- 5V logic level

### CAN Interface
- **Seeed Studio CANBed** - Arduino CAN-BUS Development Kit
- ATmega32U4 with MCP2515 (CAN controller) and MCP2551 (CAN transceiver)
- All-in-one board - no separate Arduino needed
- 500kbps CAN bus speed
- CS Pin: GPIO 17

### Pin Configuration

| Pin | Function | Type |
|-----|----------|------|
| 17 | SPI CS (CAN) | Output |
| 8 | Reverse Switch | Input (Pull-up) |
| 13 | Status LED | Output |

The reverse switch connects between pin 8 and ground. The internal pull-up resistor is enabled, so the switch reads LOW when pressed (reverse engaged).

## Dependencies

### PlatformIO Libraries
```ini
lib_deps = https://github.com/Longan-Labs/Arduino_CAN_BUS_MCP2515
```

### Arduino Libraries
- `SPI.h` (built-in)
- `mcp_canbus.h` (Longan Labs MCP2515 library - compatible with Seeed Studio CANBed)

## How It Works

### Gear Selection Logic

The system determines the current gear based on two inputs:

1. **Reverse Switch** (GPIO pin 8): Physical switch activated when reverse gear is engaged
2. **Handbrake Status** (CAN 0x240): Monitors handbrake position via CAN bus

```
Priority Logic:
1. If reverse switch pressed → REVERSE (0x07)
2. Else if handbrake OFF    → DRIVE (0x09)  
3. Else                     → PARK (0x08)
```

### CAN Bus Setup

The emulator connects to **CAN-C** (Powertrain CAN) at 500kbps. It uses hardware filtering to only receive CAN ID 0x240 (EZS message containing handbrake state).

```cpp
CAN.init_Mask(0, 0, 0x7FF);  // Match all 11 bits
CAN.init_Filt(0, 0, 0x240);  // Only accept 0x240
```

### Status LED Behavior

| Gear | LED Behavior |
|------|--------------|
| Park | Solid ON |
| Drive | OFF |
| Reverse | Pulsing (500ms interval) |

## CAN Message Details

### 0x230 - EWM Shifter Position
**Interval**: 10ms  
**Length**: 1 byte

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | 0x07 | Reverse |
| 0 | 0x08 | Park |
| 0 | 0x09 | Drive |

### 0x218 - GS_218h Main EGS Status
**Interval**: 20ms  
**Length**: 8 bytes  
**Critical for preventing torque limiting!**

| Byte | Bits | Signal | Description |
|------|------|--------|-------------|
| 0 | 0 | MTGL_EGS | Toggle bit (40ms period) |
| 0 | 1 | MMIN_EGS | Torque request min |
| 0 | 2 | MMAX_EGS | Torque request max |
| 0 | 3-7 | M_EGS MSB | Engine torque request |
| 1 | 0-7 | M_EGS LSB | Engine torque request |
| 2 | 0-3 | GZC | Target gear |
| 2 | 4-7 | GIC | Actual gear |
| 3 | - | Flags | Status flags (0x48) |
| 4 | 0 | GET_OK | Gearbox OK (CRITICAL - must be 1) |
| 4 | 1 | KS | Ball start |
| 4 | 2 | ALF | Start enable |
| 4 | 3 | GS_NOTL | Emergency mode (must be 0) |
| 4 | 6 | GSP_OK | Profile OK (CRITICAL - must be 1) |
| 5 | 3 | MPAR_EGS | Torque parity |
| 5 | 7 | MOT_NAUS | Emergency shutdown (must be 0) |
| 6 | - | MKRIECH | Creep torque (0xFF) |
| 7 | 0-1 | FEHLPRF_ST | Error check status (1=OK) |
| 7 | 2 | CALID_CVN_AKT | CVN active |
| 7 | 3-7 | FEHLER | CVN counter (5-bit) |

**Gear codes for GIC/GZC:**
- 0x5 = Drive (5th gear indication)
- 0xB = Reverse
- 0xD = Park
- 0xF = Signal not available

### 0x338 - GS_338h Transmission Speeds
**Interval**: 20ms  
**Length**: 8 bytes

| Byte | Signal | Value | Description |
|------|--------|-------|-------------|
| 0-1 | NAB | 0xFFFF | Output shaft speed (not available) |
| 2-5 | - | 0x00 | Reserved |
| 6-7 | NTURBINE | 0xFFFF | Turbine speed (not available) |

### 0x418 - GS_418h Gear Display
**Interval**: 20ms  
**Length**: 8 bytes

| Byte | Signal | Description |
|------|--------|-------------|
| 0 | FSC | Gear display character (ASCII) |
| 1 | FPC | Driving program ('S' = 0x53) |
| 2 | T_GET | Oil temp (0xFF = not available) |
| 3 | Flags | Various flags |
| 4 | GIC/GZC | Gear codes |
| 5 | M_VERL | Loss torque (0xFF) |
| 6 | FMRAD+Toggle | Wheel torque factor + toggle |
| 7 | - | Reserved (0xFF) |

**FSC display characters:**
- 'P' = 0x50
- 'R' = 0x52
- 'D' = 0x44

### Toggle Bit Timing

The toggle bit (MTGL_EGS) must change state every 40ms. Since messages are sent every 20ms, the toggle flips every other frame:

```cpp
if (timeToToggle) {
    toggle = !toggle;
}
timeToToggle = !timeToToggle;
```

## Known Issues - Help Wanted!

The following ECU DTCs remain and **we are actively seeking help to resolve them**:

| DTC | Description | Status |
|-----|-------------|--------|
| 2218-1 | Transmission Control Electronic Transmission Control (ETC/EGS) Fault 1 | Stored |
| 2226-1 | Transmission Control Electronic Transmission Control (ETC/EGS) Fault 9 | Stored |
| 2204-1 | External quantity control by N15/3 (ETC/EGS Module) Toggle Fault | Stored/Current |

**These codes do not affect driveability** - the car drives perfectly with full power. However, we would like to eliminate them if possible.

### What We've Tried
- Adding NAK_TGL toggle to 0x338 - no change
- Setting CALID_CVN_AKT = 1 - caused additional DTCs
- Various 0x418 byte layouts from NAG52 structure - caused ABS faults
- Adding MPAR_EGS parity bit - no change

If you have experience with Mercedes EGS CAN protocols or ideas on what the ECU might be checking for, please open an issue or PR!

## Building & Uploading

This is a PlatformIO project:

1. Open the project in VS Code with PlatformIO extension
2. Connect Arduino Leonardo via USB
3. Click "Upload" or run `pio run --target upload`

### Serial Monitor
Connect at 115200 baud to see gear changes and handbrake status:
```
CAN BUS OK!
Handbrake is ON
Sent: Park
Handbrake is OFF
Sent: Drive
```

## Compatibility

Developed and tested on a **2005 W211 E-Class**.

Should work on other Mercedes models from 2000-2008 that use the same CAN-C protocol:
- W203 (C-Class)
- W209 (CLK)
- W211 (E-Class)
- W219 (CLS)
- W215 (CL)
- W220 (S-Class)
- W163 (ML)
- W463 (G-Class)
- R170 (SLK)
- R230 (SL)

## Acknowledgments

CAN message structure referenced from the [Ultimate NAG52](https://github.com/rnd-ash/ultimate-nag52-fw) project by rnd-ash.

## License

MIT License - Use at your own risk. Modifying vehicle control systems can be dangerous. Always test in a safe environment.