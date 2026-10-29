# mini-dummy

A clean starting point for a small **4-DOF desktop robot arm** powered by the Raspberry Pi Pico. The repository pairs header-only inverse kinematics in C++ with a USB-CDC command listener and a Python host controller, leaving the electrical and servo calibration layer explicit for the final mechanism.

> **Project state:** firmware and control template. Cartesian targets are solved into four joint angles and returned over USB, but `apply_joint_targets()` is intentionally a no-op until the real servos, pulse limits, directions, and mechanical zero offsets are calibrated.

## System architecture

```text
Python controller
  └─ G0 X Y Z over USB-CDC
       └─ Raspberry Pi Pico / RP2040
            ├─ command parser
            ├─ 4-DOF inverse kinematics
            └─ calibrated servo output hook
                 └─ base / shoulder / elbow / wrist
```

The coordinate solver models a serial arm with base yaw and a three-joint planar chain:

1. **Q0 — base yaw:** rotates the arm around the vertical axis.
2. **Q1 — shoulder pitch:** raises and lowers the upper arm.
3. **Q2 — elbow pitch:** controls reach through the forearm.
4. **Q3 — wrist pitch:** maintains the configured tool angle.

`G0 X Y Z` coordinates are expressed in millimetres relative to the base rotation axis. Positive `Z` points upward; `X` and `Y` form the horizontal plane. The solver removes the tool offset, solves the two-link shoulder/elbow geometry, then calculates wrist compensation for a fixed end-effector pitch.

## Repository layout

```text
mini-dummy/
├── CMakeLists.txt
├── include/
│   └── kinematics.h       Header-only 4-DOF inverse kinematics
├── src/
│   └── main.cpp           USB-CDC listener and command parser
├── software/
│   ├── controller.py      Pico discovery, one-shot moves, and interactive REPL
│   ├── requirements.txt   Python serial-controller dependency
│   └── vision/
│       ├── tracker.py     HSV color tracker with preview and JSON output
│       └── requirements.txt
└── README.md
```

## Bill of materials

The exact actuator and power choices depend on arm mass and target payload. These specifications are a practical baseline for a printed desktop prototype.

| Qty. | Component | Recommended specification | Purpose |
| ---: | --- | --- | --- |
| 1 | Raspberry Pi Pico | Original Pico or pin-compatible RP2040 board | Kinematics and USB-CDC control |
| 1 | Base servo | Metal gear, 180°, 20–25 kg·cm at 6 V | Q0 base yaw |
| 2 | Shoulder/elbow servos | Metal gear, 180°, 20–25 kg·cm at 6 V | Q1 and Q2 under primary load |
| 1 | Wrist servo | Metal gear, 180°, 9–12 kg·cm | Q3 tool-pitch compensation |
| 1 | Regulated servo supply | 5–6 V, 10 A minimum; 15 A preferred | Isolated high-current actuator rail |
| 1 | Pico logic supply | USB power or regulated 5 V to VSYS | RP2040 power |
| 1 | Bulk capacitor | 1,000–2,200 µF, at least 10 V | Reduces servo-rail voltage sag |
| 4 | Local bypass capacitors | 100 µF electrolytic + 100 nF ceramic | Servo connector decoupling |
| 4 | Servo connectors | 3-pin, keyed where possible | Signal, power, and ground |
| 2 | Radial bearings | Sized for the selected base/shoulder shafts | Reduces side loading on servo horns |
| 1 set | Mechanical links | PETG/ABS/nylon prints or 2–3 mm aluminium | Base, upper arm, forearm, and wrist |
| 1 set | Fasteners | M3 screws, locknuts, washers, heat-set inserts | Structural assembly |
| 4 | Servo horns/hubs | Metal preferred for loaded joints | Transfers actuator torque |
| 1 | End effector | Lightweight gripper, pen mount, or test pointer | Tooling |
| 1 | USB data cable | Micro-USB for the original Pico | Firmware and command transport |
| Optional | Limit/home switches | Lever microswitches or Hall sensors | Repeatable homing and fault detection |
| Optional | Emergency-stop switch | Latching, rated for servo-rail current | Cuts actuator power independently |

**Never power the servos from the Pico's `3V3`, `VBUS`, or GPIO pins.** Use a dedicated regulated actuator supply and connect its ground to Pico ground so the PWM signals have a common reference.

## Mechanical design specification

The default dimensions in [`include/kinematics.h`](include/kinematics.h) describe this reference geometry:

| Parameter | Default | Design note |
| --- | ---: | --- |
| Degrees of freedom | 4 | Base yaw, shoulder pitch, elbow pitch, wrist pitch |
| Base-axis height | 60 mm | Ground plane to shoulder pivot |
| Upper-arm length | 110 mm | Shoulder axis to elbow axis |
| Forearm length | 105 mm | Elbow axis to wrist axis |
| Tool offset | 45 mm | Wrist axis to tool tip |
| Tool pitch | −20° | Fixed Cartesian tool angle maintained by Q3 |
| Nominal maximum reach | 260 mm | Link sum; real safe reach is lower |
| Suggested payload | 100–150 g | Conservative printed-arm starting target |
| Joint construction | Double-supported where practical | Bearings should carry radial loads, not servo shafts |
| Printed material | PETG, ABS, or nylon | Prefer four or more perimeters and 35–45% infill |
| Structural fasteners | M3 | Use heat-set inserts in repeatedly serviced joints |

### Joint ranges and calibration

The C++ joint limits are mathematical ranges, not servo pulse limits. Before enabling motion:

1. Assemble each joint at its mechanical midpoint.
2. Record the mathematical zero angle for each servo.
3. Determine whether each joint direction must be inverted.
4. Measure safe minimum and maximum pulse widths without the links attached.
5. Tighten the software limits to stop before every mechanical hard point.
6. Add acceleration and velocity limits rather than stepping directly between targets.

The template reserves no GPIO assignments because actuator hardware varies. A conventional first mapping is GP2–GP5 for Q0–Q3. If a PCA9685 or other external PWM driver is used, document its I²C address, update rate, and power-domain boundaries before implementing `apply_joint_targets()`.

## Firmware protocol

Commands and responses are newline-delimited ASCII.

| Command | Example | Response |
| --- | --- | --- |
| Cartesian target | `G0 150 0 130` | `OK G0 ... Q0=... Q1=... Q2=... Q3=...` |
| Read last target | `M114` | `OK Q0=... Q1=... Q2=... Q3=...` |
| Link check | `PING` | `OK PONG` |
| Command help | `HELP` | Supported command summary |

Unreachable coordinates or coordinates that violate a joint limit return `ERR UNREACHABLE`. Malformed commands return `ERR BAD_COMMAND`.

## Build the Pico firmware

### Prerequisites

- CMake 3.13 or newer
- Ninja or Make
- Arm GNU Toolchain (`arm-none-eabi-gcc`)
- [Raspberry Pi Pico C/C++ SDK](https://github.com/raspberrypi/pico-sdk) with its submodules

Clone and initialize the Pico SDK once:

```bash
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git "$HOME/pico/pico-sdk"
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
```

Configure and build `mini-dummy`:

```bash
cmake -S . -B build -DPICO_BOARD=pico
cmake --build build
```

The flashable image is generated at:

```text
build/mini_dummy.uf2
```

### Flash the Pico

Hold **BOOTSEL** while connecting the Pico, then copy `build/mini_dummy.uf2` to the `RPI-RP2` volume. With `picotool` installed, the same image can be loaded from the terminal:

```bash
picotool load -f build/mini_dummy.uf2
picotool reboot
```

After reboot, the firmware enumerates as a USB serial device and prints `READY MINI-DUMMY 1.0`.

## Use the Python controller

Create an isolated environment and install PySerial:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r software/requirements.txt
```

List available serial devices:

```bash
python software/controller.py --list
```

Send a single Cartesian target. The controller automatically selects the Pico when exactly one Raspberry Pi USB device is present:

```bash
python software/controller.py 150 0 130
```

Select a port explicitly when necessary:

```bash
python software/controller.py --port /dev/cu.usbmodem1101 150 0 130
```

Run without coordinates for an interactive session:

```bash
python software/controller.py

mini-dummy> 150 0 130
mini-dummy> status
mini-dummy> ping
mini-dummy> quit
```

On Linux, serial access may require membership in the `dialout` group. On Windows, pass a port such as `COM5` with `--port`.

## Use the vision tracker

The vision module tracks the largest object inside a configurable HSV color range. It reports the target centroid, normalized horizontal/vertical offsets from the image center, contour area, radius, and a size-based confidence value. The default range targets saturated green objects.

Install its dependencies separately so the serial controller stays lightweight:

```bash
python -m pip install -r software/vision/requirements.txt
```

Start the live camera preview:

```bash
python software/vision/tracker.py --source 0 --mirror
```

Tune the HSV range for a different target color. OpenCV hue values use a `0..179` scale:

```bash
python software/vision/tracker.py \
  --lower 20 120 100 \
  --upper 35 255 255 \
  --min-area 800
```

For a perception or control pipeline, disable the GUI and emit newline-delimited JSON:

```bash
python software/vision/tracker.py \
  --headless \
  --json \
  --print-every 1
```

Example detection:

```json
{"frame":42,"timestamp_s":1793232000.0,"detected":true,"center_x_px":701,"center_y_px":338,"offset_x_norm":0.09531,"offset_y_norm":0.06111,"area_px":4280.5,"radius_px":39.2,"confidence":0.214}
```

The tracker intentionally does **not** send motion commands. Camera pixels must be calibrated into the robot coordinate frame, and any closed-loop controller should add workspace, velocity, acceleration, and collision limits before forwarding targets to the arm.

## Adapting the kinematics

Update `ArmGeometry` in [`include/kinematics.h`](include/kinematics.h) with **axis-to-axis measurements**, not outside dimensions. The tool offset is measured from the wrist axis to the intended Cartesian tool point. Rebuild after changing geometry.

The provided solver assumes:

- the shoulder, elbow, and wrist axes remain parallel;
- the base axis is vertical;
- the requested tool pitch is fixed;
- links are rigid and backlash-free;
- only one analytic elbow configuration is selected by the firmware.

A real mechanism should add homing, persistent calibration, motion profiling, current-aware fault handling, and collision constraints before carrying payloads.

## Bring-up checklist

- [ ] Build and flash with no servos powered.
- [ ] Confirm `PING` and a reachable `G0` command over USB-CDC.
- [ ] Measure the finished arm's axis-to-axis geometry.
- [ ] Establish servo midpoint, direction, and pulse limits one joint at a time.
- [ ] Verify the external servo supply and common ground.
- [ ] Test at reduced current with links unloaded.
- [ ] Add velocity and acceleration limiting before coordinated movement.
- [ ] Validate the reachable workspace with a lightweight pointer before attaching a gripper.

## Safety

Even a small servo arm can pinch fingers, throw a loose part, or pull damaging current when stalled. Keep a hardware power disconnect within reach, secure the base, test with low current limits, and never depend on inverse kinematics alone for collision prevention.
