# Pinc'Open with a Dynamixel XM430-W350-T

This project is a fork of the amazing [Pinc'Open Project](https://github.com/pollen-robotics/PincOpen/tree/main). The design files have been modified to use a **Dynamixel XM430-W350-T** for driving the gripper, with a few mechanical tweaks (e.g. 25 mm dowel instead of 24 mm due to availability). A custom 2-part mount is included for attachment to a **UR10** (also compatible with UR5 / UR3), and extra attachments are provided for a wrist-mounted **Intel RealSense D435i** camera with an adjustable angle bracket.

## Table of Contents
- [Pinc'Open with a Dynamixel XM430-W350-T](#pincopen-with-a-dynamixel-xm430-w350-t)
  - [Table of Contents](#table-of-contents)
- [About the Project](#about-the-project)
- [Build Resources](#build-resources)
  - [BOM (Bill Of Materials)](#bom-bill-of-materials)
  - [STL and SolidWorks Files](#stl-and-solidworks-files)
  - [Assembly Guide](#assembly-guide)
- [Software — Python Gripper Control](#software--python-gripper-control)
  - [Installation](#installation)
  - [Hardware Configuration](#hardware-configuration)
  - [Step 0 — Verify Connection with `test.py`](#step-0--verify-connection-with-testpy)
  - [Option A — One-Shot Function: `control_gripper()`](#option-a--one-shot-function-control_gripper)
    - [Full Signature](#full-signature)
  - [Option B — Persistent Class: `GripperController`](#option-b--persistent-class-grippercontroller)
    - [Basic Usage](#basic-usage)
    - [Constructor Parameters](#constructor-parameters)
    - [Methods](#methods)
  - [Torque / Compliant Grasping](#torque--compliant-grasping)
  - [Return Values and Error Handling](#return-values-and-error-handling)
  - [Control Table Reference](#control-table-reference)

---

# About the Project

Made out of necessity and availability of resources. The gripper uses a Dynamixel XM430-W350-T in **Current-Based Position Mode (Mode 5)**, which allows simultaneous position trajectory control and current (torque) limiting — useful for compliant grasps on delicate objects.

A wrist-mounted Intel RealSense D435i camera attachment is included with an adjustable angle bracket. A custom 2-part UR10 mount is also included.

---

# Build Resources

## BOM (Bill Of Materials)

| # | Component |
|---|-----------|
| 1 | Dynamixel XM430-W350-T |
| 2 | U2D2 (USB-to-TTL interface) |
| 3 | U2D2 Power Hub |
| 4 | 3D printed parts (see CAD section) |
| 5 | M3 × 16 mm Dowels × 4 |
| 6 | M3 × 25 mm Dowels × 4 |
| 7 | Threaded Inserts M3 × 4.2 mm (OD) × 5 mm (L) × 4 |
| 8 | M3 Bolts (assorted) |
| 9 | M2.5 Bolts (assorted) |
| 10 | Bushings: 3 (ID) × 6 (OD) × 5 mm (L) |

## STL and SolidWorks Files

SLDPRT, SLDASM, STL, and STEP files can be found [here](https://github.com/rudra-8000/PincOpen_Dynamixel_XM430-W350-T/tree/main/cad).
Files were edited from the original PincOpen STEP files using SolidWorks.

## Assembly Guide

Assembly follows the same process as the original [PincOpen](https://github.com/pollen-robotics/PincOpen/tree/main). Refer to their guide and substitute the modified CAD parts.

---

# Software — Python Gripper Control

The gripper is controlled via `dynamixel_control.py`, which provides two interfaces:
- `control_gripper()` — a simple one-shot function, good for scripting
- `GripperController` — a persistent class that holds the serial connection open, recommended for robot pipelines

Both use **Current-Based Position Mode (Mode 5)** on the XM430-W350-T, enabling torque-limited compliant grasps.

## Installation

```bash
pip install dynamixel-sdk
```

## Hardware Configuration

At the top of `dynamixel_control.py`, edit these constants to match your setup:

```python
DEVICE_PORT = '/dev/ttyUSB0'   # Linux default. Windows: 'COM3', macOS: '/dev/tty.usbserial-*'
BAUD_RATE   = 1_000_000        # Must match what is set on the motor via Dynamixel Wizard
DXL_ID      = 0                # Must match the ID set on the motor
```

> **Tip:** Use [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to set the motor's baud rate and ID before first use. The default factory baud rate is `57600` and default ID is `1`.

---

## Step 0 — Verify Connection with `test.py`

Before running any motion code, use `test.py` to confirm the U2D2 can communicate with the motor. It scans all common baud rates and IDs 0–10:

```bash
python test.py
```

Expected output when the motor is found:

```
Scanning /dev/ttyUSB0 ...
─────────────────────────────────────────────
✓ FOUND  ID=0  Baud=1000000  Model=1020
```

The model number `1020` corresponds to the XM430-W350-T.

**If nothing is found**, check:
- USB cable is connected and U2D2 is powered (LED should be on)
- Power supply is connected to the U2D2 Power Hub
- The correct port is set in `test.py` (`PORT = '/dev/ttyUSB0'`)
- On Linux, you may need: `sudo chmod 666 /dev/ttyUSB0`

To scan a wider ID range, change `SCAN_IDS = range(0, 253)` in `test.py`.

---

## Option A — One-Shot Function: `control_gripper()`

Opens the serial port, executes the move, waits for completion, verifies position, then closes the port. Suitable for scripts or infrequent calls.

```python
from dynamixel_control import control_gripper

# Move to 220° at 80% speed, 50% torque
result = control_gripper(220.0, speed=0.8, torque=0.5)
print(result)  # 1 = success, 0 = failure
```

### Full Signature

```python
control_gripper(
    angle:       float,           # Target angle in degrees (clamped to valid range)
    speed:       float = 1.0,     # Speed fraction [0.01, 1.0]. 1.0 ≈ 46 rpm
    torque:      float = 1.0,     # Current limit fraction [0.01, 1.0]. 1.0 = ~3.2 A (rated peak)
    open_angle:  float = 285.0,   # Angle (°) for fully open
    close_angle: float = 166.0,   # Angle (°) for fully closed
    port:        str   = '/dev/ttyUSB0',
    baud:        int   = 1_000_000,
    dxl_id:      int   = 0,
) -> int                          # Returns 1 on success, 0 on failure
```

> **Note:** This function writes the Operating Mode to EEPROM on every call. Avoid calling it in a fast loop — use `GripperController` instead to protect EEPROM lifespan.

---

## Option B — Persistent Class: `GripperController`

Holds the serial port open across multiple calls. The operating mode is written **once at `__init__`**, not on every move. This is the recommended interface for robot manipulation pipelines.

### Basic Usage

```python
from dynamixel_control import GripperController
import time

# Using a context manager (auto-disconnects on exit)
with GripperController(open_angle=285.0, close_angle=166.0) as gripper:
    gripper.open(speed=0.8)
    time.sleep(1)
    gripper.move(240.0, speed=0.5, torque=0.4)   # mid-grasp, 40% torque
    time.sleep(1)
    gripper.close(torque=0.6)                     # gentle close

# Or manage manually:
gripper = GripperController(
    open_angle=285.0,
    close_angle=166.0,
    default_speed=0.8,    # used by open()/close() if not overridden
    default_torque=1.0,
)
gripper.open()
gripper.close()
gripper.close_connection()
```

### Constructor Parameters

```python
GripperController(
    port:           str   = '/dev/ttyUSB0',
    baud:           int   = 1_000_000,
    dxl_id:         int   = 0,
    open_angle:     float = 285.0,   # Fully open position (°)
    close_angle:    float = 166.0,   # Fully closed position (°)
    default_speed:  float = 1.0,     # Default speed for open()/close()
    default_torque: float = 1.0,     # Default torque for open()/close()
)
```

### Methods

| Method | Description |
|--------|-------------|
| `move(angle, speed=None, torque=None) → int` | Move to arbitrary angle in degrees. `None` falls back to instance defaults. Returns `1`/`0`. |
| `open(speed=None, torque=None) → int` | Move to `open_angle`. |
| `close(speed=None, torque=None) → int` | Move to `close_angle`. |
| `set_defaults(speed=None, torque=None)` | Update instance-level default speed/torque. |
| `close_connection()` | Disable torque and close the serial port. |

---

## Torque / Compliant Grasping

The `torque` parameter maps to **Goal Current** (address 102) in Current-Based Position Mode. This limits the maximum motor current, which directly limits force output.

| `torque` value | Approx. current | Use case |
|---|---|---|
| `1.0` | ~3.2 A (1193 units) | Maximum grip force |
| `0.5` | ~1.6 A (597 units) | General objects |
| `0.3` | ~960 mA (358 units) | Soft / deformable objects |
| `0.15` | ~480 mA (179 units) | Very delicate objects |

When the gripper contacts an object before reaching the goal angle, the motor stalls at the contact point and `MOVING` goes `0`. The code treats this as a **valid grasp event** (returns `1` with a `[WARN]` log), not a failure.

---

## Return Values and Error Handling

Both `control_gripper()` and `GripperController.move()` return:
- **`1`** — Motion completed; final position verified within tolerance (15 ticks ≈ 1.3°), or motor stalled at a torque-limited contact.
- **`0`** — Hard failure: port error, communication error, or motion timeout (10 s).

All failures print a descriptive `[ERROR]` or `[WARN]` message to stdout.

---

## Control Table Reference

| Address | Size | Name | Description |
|---------|------|------|-------------|
| 11 | 1B | Operating Mode | `5` = Current-Based Position (used here) |
| 38 | 2B | Current Limit | Hard cap on current; default = 1193 units |
| 64 | 1B | Torque Enable | `1` = on, `0` = off |
| 102 | 2B | Goal Current | Soft current limit per move; unit ≈ 2.69 mA |
| 108 | 4B | Profile Acceleration | unit: 214.577 rev/min² |
| 112 | 4B | Profile Velocity | unit: 0.229 rpm; `200` ≈ 45.8 rpm |
| 116 | 4B | Goal Position | unit: 1 tick; 4096 ticks/rev (0.088°/tick) |
| 122 | 1B | Moving | `1` = in motion, `0` = stopped |
| 132 | 4B | Present Position | Current position in ticks |