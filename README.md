# ArduPilot Precision Landing with Yaw Alignment

Custom ArduCopter firmware for AprilTag-based precision landing with automatic yaw alignment to the target.

**Tested hardware:** Pixhawk 6X, OpenMV Cam RT1062 (OV5640), rangefinder (LiDAR).

---

## Overview

The system lets the drone:

1. Detect an AprilTag and receive position + orientation via MAVLink
2. Center over the target (XY), then align heading (yaw) to the tag
3. Descend with combined XY and yaw control and land precisely

**Main components:**

- **OpenMV script** — AprilTag detection, pose (yaw quaternion), `LANDING_TARGET` MAVLink messages
- **ArduCopter changes** — extended `AC_PrecLand` with a yaw alignment state machine, descent logic, and `PL2` logging

---

## Architecture

```
OpenMV (AprilTag + Yaw)     UART/MAVLink      Pixhawk (ArduCopter)
┌─────────────────────┐  LANDING_TARGET   ┌──────────────────────────┐
│ mavlink_apriltags_   │ ──────────────────│ AC_PrecLand_MAVLink       │
│ landing_target.py     │  angle_x/y,       │   → AC_PrecLand (EKF)     │
│ (quaternion, time)   │  quaternion       │   → Yaw align state machine│
└─────────────────────┘                   │   → mode.cpp (descent/XY)  │
                                           │   → AutoYaw PRECLAND_TARGET│
     Rangefinder ─────────────────────────│   (altitude)              │
                                           └──────────────────────────┘
```

---

## Features

- **Yaw alignment state machine** — SEARCHING → XY_CENTERING → COARSE_ALIGNING → DESCENDING → FINE_ALIGNING → FINAL_DESCENT
- **XY-first centering** — Center over tag before rotating to avoid losing the target
- **Configurable tolerances** — Coarse and fine yaw phases with separate time and altitude gates
- **Position hold during yaw** — During coarse/fine align, position held via EKF when target is lost
- **Optional yaw** — Set `PLND_YAW_TGT = -1` for position-only precision landing (no yaw alignment)

---

## State Machine

```
┌─────────────┐
│  SEARCHING  │  Descent allowed, no target yet
└──────┬──────┘
       │ Target visible
       ▼
┌───────────────┐
│ XY_CENTERING  │  Center over tag, slow descent, no yaw yet (≈1s stable)
└───────┬───────┘
        │ XY error < PLND_ACC_ERR for PLND_YAW_STABLE
        ▼
┌─────────────────┐
│ COARSE_ALIGNING │  Rotate to tag yaw, descent PAUSED, position hold
└────────┬────────┘
         │ Yaw within PLND_YAW_COARSE for PLND_YAW_TIME
         ▼
┌────────────────┐
│ COARSE_HOLDING │  Short transition
└────────┬───────┘
         ▼
┌─────────────┐
│ DESCENDING  │  Descent with XY + yaw correction
└──────┬──────┘
       │ Altitude < PLND_YAW_FALT
       ▼
┌───────────────┐
│ FINE_ALIGNING │  Fine yaw trim, descent PAUSED (timeout 20s)
└───────┬───────┘
        │ Yaw within PLND_YAW_FINE for PLND_YAW_TIME
        ▼
┌──────────────┐
│ FINAL_DESCENT│  Land with yaw locked
└──────────────┘
```

---

## Installation

### Files to Copy Into Your ArduPilot Tree

Replace the same paths in your clone with these (from your precision-landing branch or repo):

**ArduCopter**

- `ArduCopter/mode.cpp`
- `ArduCopter/mode.h`
- `ArduCopter/autoyaw.cpp`
- `ArduCopter/mode_land.cpp`
- `ArduCopter/mode_rtl.cpp`
- `ArduCopter/mode_auto.cpp`

**Libraries**

- `libraries/AC_PrecLand/AC_PrecLand.cpp`
- `libraries/AC_PrecLand/AC_PrecLand.h`
- `libraries/AC_PrecLand/LogStructure.h`

**OpenMV (optional, for camera/sim)**

- `OpenMV Code/mavlink_apriltags_landing_target.py` — camera script
- `OpenMV Code/apriltag_mavlink_sim.py` — Gazebo SITL sim

### Build

```bash
cd ardupilot
./waf configure --board <your-board>   # e.g. sitl, Pixhawk6X-bdshot
./waf copter
```

---

## Parameters

### Core

| Parameter       | Default | Description                          |
|----------------|---------|--------------------------------------|
| `PLND_ENABLED` | 0       | 1 = precision landing enabled       |
| `PLND_TYPE`    | 0       | 1 = MAVLink backend (OpenMV)         |
| `PLND_YAW_TGT` | -1      | Target yaw offset (°), -1 = no yaw   |

### Yaw Alignment

| Parameter         | Default | Range   | Description                    |
|------------------|---------|--------|--------------------------------|
| `PLND_YAW_MAXALT`| 3.0     | 0–50 m | Max altitude to start yaw      |
| `PLND_YAW_COARSE`| 1000    | 100–9000 | Coarse tolerance (centidegrees) |
| `PLND_YAW_FINE`  | 300     | 50–1000 | Fine tolerance (centidegrees)  |
| `PLND_YAW_FALT`  | 0.5     | 0–10 m | Altitude for fine phase        |
| `PLND_YAW_TIME`  | 2.0     | 0.5–10 s | Hold time in tolerance       |
| `PLND_YAW_RATE`  | 30      | 5–90 °/s | Max yaw rate                 |
| `PLND_YAW_FILT`  | 0.3     | 0–0.9  | Yaw low-pass (0 = off)        |
| `PLND_YAW_STABLE`| 0.5     | 0–5 s  | Required stable hover time    |

### Position

| Parameter        | Default | Description              |
|-----------------|---------|--------------------------|
| `PLND_ACC_ERR`  | 0.5     | Acceptable XY error (m)  |
| `PLND_FINE_CORR`| 0.3     | Max fine-phase XY (m)    |
| `PLND_TIMEOUT`  | 4.0     | Target-lost timeout (s)  |

---

## Recommended Presets

**Small tags (&lt; 50 mm)**  
`PLND_YAW_MAXALT=2.0` `PLND_YAW_COARSE=500` `PLND_YAW_FINE=200` `PLND_YAW_FALT=0.3` `PLND_ACC_ERR=0.3`

**Medium tags (50–150 mm)**  
`PLND_YAW_MAXALT=3.0` `PLND_YAW_COARSE=1000` `PLND_YAW_FINE=300` `PLND_YAW_FALT=0.5` `PLND_ACC_ERR=0.5`

**Large tags (&gt; 150 mm)**  
`PLND_YAW_MAXALT=5.0` `PLND_YAW_COARSE=1500` `PLND_YAW_FINE=500` `PLND_YAW_FALT=1.0` `PLND_ACC_ERR=0.8`

---

## OpenMV Camera Setup

- **Hardware:** OpenMV Cam H7 Plus or RT1062, UART to autopilot.
- **Script:** Flash `mavlink_apriltags_landing_target.py` to the OpenMV; set tag ID/size and serial port in the script.
- **ArduPilot:**  
  `SERIALx_PROTOCOL = 1` (MAVLink), `SERIALx_BAUD = 115200`, `PLND_ENABLED = 1`, `PLND_TYPE = 1`.

More detail: see `OpenMV Code/README_Precision_Landing.md`.

---

## Simulation (Gazebo)

```bash
pip install apriltag pupil-apriltags pymavlink opencv-python numpy
```

**Terminal 1:** `sim_vehicle.py -v ArduCopter --gazebo`  
**Terminal 2:** `python OpenMV\ Code/apriltag_mavlink_sim.py --tag-size 800 --decimate 2.0`  
**Terminal 3 (MAVProxy):** `mode GUIDED` → `arm throttle` → `takeoff 5` → `mode LAND`

Options: `--port`, `--sitl-port`, `--tag-size`, `--decimate`, `--invert-yaw`, `--yaw-offset`.

---

## Quick Test

1. GUIDED, arm, takeoff (e.g. 5 m).
2. Move drone off-center from tag.
3. `mode LAND` — expect: XY centering → yaw alignment → descent → fine align → final descent.

**Typical GCS messages:**  
`PrecLand: Yaw align active, searching` → `Target found, centering XY` → `XY stable` → `Coarse alignment complete!` → `Descending with yaw control` → `Fine align at …` → `Fine alignment complete!` → `Final descent with yaw locked`.

---

## Troubleshooting

| Symptom              | Likely cause           | Action                    |
|----------------------|------------------------|---------------------------|
| No yaw alignment     | `PLND_YAW_TGT = -1`    | Set 0–360                 |
| Loses target in turn | Tag too small          | Lower `PLND_YAW_MAXALT`   |
| Yaw oscillation      | Filter too low         | Increase `PLND_YAW_FILT`  |
| XY center timeout    | Wind / accuracy        | Increase `PLND_ACC_ERR`   |
| Fine align timeout   | Tolerance too tight    | Increase `PLND_YAW_FINE`  |

---

## Files Overview

| Path / file | Role |
|-------------|------|
| `libraries/AC_PrecLand/AC_PrecLand.cpp` | State machine, EKF, PL2 log, parameters |
| `libraries/AC_PrecLand/AC_PrecLand.h`   | State enums, parameter declarations, API |
| `libraries/AC_PrecLand/LogStructure.h`  | `PL2` log message for yaw diagnostics   |
| `ArduCopter/mode.cpp`                   | Descent/XY and precision landing logic  |
| `ArduCopter/mode.h`                     | Precision landing state, `precland_reset_state()`, AutoYaw mode |
| `ArduCopter/autoyaw.cpp`                | Yaw rate control for `PRECLAND_TARGET`  |
| `ArduCopter/mode_land.cpp`              | LAND init: `yaw_align_init()`, `precland_reset_state()` |
| `ArduCopter/mode_rtl.cpp`               | RTL init: same resets                   |
| `ArduCopter/mode_auto.cpp`              | Auto init: same resets                  |
| `OpenMV Code/mavlink_apriltags_landing_target.py` | OpenMV AprilTag + MAVLink script |
| `OpenMV Code/apriltag_mavlink_sim.py`   | Gazebo SITL AprilTag injector           |

---

## More Detail

- **Camera, parameters, logging:** `OpenMV Code/README_Precision_Landing.md`
- **Log analysis:** `OpenMV Code/Precision_Landing_Log_Debugging_Guide.txt` (if present)
- **Gazebo workflow:** `OpenMV Code/How To Gazebo.txt` (if present)

---

## License

This modification follows the ArduPilot project license (GPLv3).
