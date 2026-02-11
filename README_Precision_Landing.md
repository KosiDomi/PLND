# Precision Landing System with Yaw Alignment

ArduCopter custom firmware for AprilTag-based precision landing with automatic yaw alignment.

**Hardware**: Pixhawk 6X + OpenMV RT1062 (OV5640) + Rangefinder (LiDAR)

## Overview

This system enables a drone to autonomously detect an AprilTag, center itself over it, align its heading (yaw) to the tag's orientation, and perform a controlled precision landing. It consists of two main components:

1. **OpenMV Camera Script** (`mavlink_apriltags_landing_target.py`) — detects AprilTags and sends `LANDING_TARGET` MAVLink messages with position angles and yaw quaternion to the flight controller.
2. **ArduCopter Firmware Modifications** — extended precision landing library with a yaw alignment state machine, descent control, and diagnostic logging.

## Architecture

```
OpenMV RT1062                    Pixhawk 6X (ArduCopter)
┌─────────────┐    UART/MAVLink   ┌─────────────────────────┐
│ AprilTag     │ ──LANDING_TARGET──│ AC_PrecLand_MAVLink     │
│ Detection    │   (angle_x,      │   ↓                     │
│ + Yaw Quat   │    angle_y,      │ AC_PrecLand (EKF)       │
│ + Timestamp  │    quaternion)    │   ↓                     │
└─────────────┘                   │ Yaw Align State Machine │
                                  │   ↓                     │
     Rangefinder ─────────────────│ mode.cpp (Descent/XY)   │
                                  │   ↓                     │
                                  │ AutoYaw (PRECLAND_TARGET)│
                                  └─────────────────────────┘
```

## Modified Files

### ArduCopter

| File | Changes |
|------|---------|
| `ArduCopter/mode.cpp` | Precision landing descent control, XY positioning with state-specific behavior, yaw alignment cache, mid-flight parameter change support |
| `ArduCopter/mode.h` | Member variables for precision landing state, `precland_reset_state()`, AutoYaw `PRECLAND_TARGET` mode |
| `ArduCopter/autoyaw.cpp` | Rate-limited yaw control for `PRECLAND_TARGET` mode |
| `ArduCopter/mode_land.cpp` | Init: calls `yaw_align_init()` + `precland_reset_state()` |
| `ArduCopter/mode_rtl.cpp` | Init: calls `yaw_align_init()` + `precland_reset_state()` |
| `ArduCopter/mode_auto.cpp` | Init: calls `yaw_align_init()` + `precland_reset_state()` |

### Libraries

| File | Changes |
|------|---------|
| `libraries/AC_PrecLand/AC_PrecLand.cpp` | Yaw alignment state machine, yaw extraction from quaternion (body→NED conversion), EKF robustness improvements, `PL2` log message, 13 new parameters |
| `libraries/AC_PrecLand/AC_PrecLand.h` | State machine enums, new parameter declarations, yaw alignment API |
| `libraries/AC_PrecLand/LogStructure.h` | New `PL2` log message definition for yaw diagnostics |

### OpenMV Camera

| File | Description |
|------|-------------|
| `OpenMV Code/mavlink_apriltags_landing_target.py` | Production camera script (v9.0) — single-tag AprilTag detection, MAVLink encoding, yaw sign correction |
| `OpenMV Code/apriltag_mavlink_sim.py` | Gazebo SITL simulation script — GStreamer video + pupil_apriltags + pymavlink |

## Yaw Alignment State Machine

```
  ┌──────────┐
  │ SEARCHING │ ← Target not yet detected. Descent allowed.
  └────┬─────┘
       │ Target visible
       ▼
  ┌──────────────┐
  │ XY_CENTERING │ ← Center drone over tag. Descent allowed (slow).
  │  (1s stable) │   No yaw rotation yet.
  └──────┬───────┘
         │ XY error < PLND_ACC_ERR for 1 second
         ▼
  ┌─────────────────┐
  │ COARSE_ALIGNING │ ← Rotate to align yaw. Descent PAUSED.
  │ (hold position) │   Position held via GPS/EKF (no target tracking).
  └───────┬─────────┘
          │ Yaw error < PLND_YAW_COARSE for PLND_YAW_TIME
          ▼
  ┌────────────────┐
  │ COARSE_HOLDING │ → Brief transition state
  └───────┬────────┘
          ▼
  ┌────────────┐
  │ DESCENDING │ ← Controlled descent with active XY + yaw correction.
  └──────┬─────┘
         │ Altitude < PLND_YAW_FALT
         ▼
  ┌───────────────┐
  │ FINE_ALIGNING │ ← Fine yaw correction. Descent PAUSED.
  │ (20s timeout) │   Limited XY corrections (PLND_FINE_CORR).
  └───────┬───────┘
          │ Yaw error < PLND_YAW_FINE for PLND_YAW_TIME
          ▼
  ┌──────────────┐
  │ FINAL_DESCENT│ ← Landing with yaw locked.
  └──────────────┘
```

**Disable yaw alignment**: Set `PLND_YAW_TGT = -1` — system performs position-only precision landing.

## OpenMV Camera Script (v9.0)

### Key Features

- **Single-tag mode**: Detects only tag ID 2 (70mm). Landing point = tag center.
- **NEGATE_YAW**: Inverts `z_rotation` sign to match ArduPilot convention (fixes wrong rotation direction observed in flight tests).
- **TAG36H11 family filter**: Faster detection, fewer false positives.
- **Corrected FOV**: Uses official 70.8° x 55.6° instead of incorrect datasheet values.
- **QVGA with decimate=2**: 320x240 resolution, detection on 160x120, pose on full resolution.
- **Monotonic timestamps**: Accumulated deltas for ArduPilot jitter correction.
- **MAVLink 2 encoding**: Pre-allocated buffer, correct CRC-16/MCRF4XX with extra CRC byte last.
- **Watchdog timer**: 10-second WDT resets board on script hang.
- **Error resilience**: `try/except` in main loop, `MemoryError` recovery via `gc.collect()`.

### Configuration

```python
NEGATE_YAW = True     # Invert yaw sign (True = match simulation convention)
TAG_ID     = 2        # AprilTag ID to detect
TAG_SIZE_MM = 70      # Physical tag size in mm
USE_QVGA   = True     # True=320x240 (better accuracy), False=160x120 (faster)
```

### Camera Mounting

- Lens pointing **down**
- Connector pointing to **rear** of drone
- Mounted **in front of** the flight controller
- Set `PLND_CAM_POS_X/Y/Z` to the camera offset from CG (in meters)

## New Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `PLND_YAW_TGT` | 0 | cdeg | Target yaw offset from tag orientation (-1 = disable) |
| `PLND_YAW_COARSE` | 2000 | cdeg | Coarse alignment tolerance (20°) |
| `PLND_YAW_FINE` | 500 | cdeg | Fine alignment tolerance (5°) |
| `PLND_YAW_TIME` | 5.0 | s | Hold time before state transition |
| `PLND_YAW_FALT` | 2.0 | m | Fine alignment altitude (0 = disable) |
| `PLND_YAW_RATE` | 20 | deg/s | Maximum yaw rotation rate |
| `PLND_YAW_STABLE` | 2.0 | s | Required stable time after alignment |
| `PLND_YAW_XY_GATE` | 1 | - | XY gate: align only when centered (0/1) |
| `PLND_YAW_MAXALT` | 5.0 | m | Max altitude for yaw alignment start |
| `PLND_YAW_FILT` | 0.5 | - | Yaw low-pass filter (0=none, 0.9=heavy) |
| `PLND_ACC_ERR` | 0.15 | m | Acceptable XY error for slow descent |
| `PLND_MIN_DSPD` | 0.1 | m/s | Minimum descent speed during precision landing |
| `PLND_FINE_CORR` | 0.15 | m | Max XY correction in fine/final phase |

## Recommended Parameter Settings

### For single 70mm tag (current setup)

```
PLND_ENABLED    = 1
PLND_TYPE       = 1          # MAVLink backend
PLND_LAG        = 0.08       # 80ms sensor lag (QVGA)

# Yaw Alignment
PLND_YAW_TGT    = 0          # Align to tag X-axis (or -1 to disable)
PLND_YAW_COARSE = 1500       # 15° coarse tolerance
PLND_YAW_FINE   = 500        # 5° fine tolerance
PLND_YAW_TIME   = 2.0        # 2s hold time
PLND_YAW_RATE   = 15         # 15°/s max rotation
PLND_YAW_STABLE = 1.5        # 1.5s stable time
PLND_YAW_MAXALT = 5.0        # Start alignment at ≤5m
PLND_YAW_FALT   = 1.5        # Fine align at 1.5m
PLND_YAW_FILT   = 0.5        # Moderate filtering
PLND_YAW_XY_GATE = 1         # XY gate enabled

# Position
PLND_ACC_ERR    = 0.15       # 15cm XY tolerance
PLND_MIN_DSPD   = 0.10       # 10cm/s min descent
PLND_FINE_CORR  = 0.15       # 15cm max fine correction
PLND_XY_DIST_MAX = 2.0       # 2m max XY error before pausing descent
PLND_TIMEOUT    = 4          # 4s retry timeout
PLND_STRICT     = 1          # Normal strictness

# EKF / Altitude
PLND_EST_TYPE   = 1          # Kalman filter
PLND_ACC_P_NSE  = 2.5        # EKF process noise
EK3_SRC1_POSZ   = 1          # Baro for EKF altitude (NOT rangefinder)
EK3_RNG_USE_HGT = 70         # Use rangefinder below 70% of max range
```

## Log Messages

### PL (Precision Landing)

| Field | Unit | Description |
|-------|------|-------------|
| Heal | 0/1 | Backend healthy |
| TAcq | 0/1 | Target acquired (EKF initialized) |
| pX, pY | m | Target position relative to vehicle (N, E) |
| vX, vY | m/s | Target velocity relative to vehicle |
| mX, mY, mZ | m | Raw measurement position (N, E, D) |
| LastMeasMS | ms | Time of last measurement |
| EKFOutl | - | EKF outlier count |

### PL2 (Yaw Alignment Diagnostics) — NEW

| Field | Unit | Description |
|-------|------|-------------|
| YSt | - | Yaw state (0=Disabled, 1=Searching, 2=XYCenter, 3=CoarseAlign, 4=CoarseHold, 5=Descend, 6=FineAlign, 7=FineHold, 8=FinalDesc) |
| TVis | 0/1 | Target visible (backend has recent measurement) |
| TAcq | 0/1 | Target acquired (EKF initialized) |
| TYaw | deg | Target yaw in NED frame |
| YErr | deg | Current yaw alignment error |
| MDt | ms | Time since last measurement |
| RFAl | m | Rangefinder altitude |
| XYEr | m | Horizontal distance to target |

## Test Procedure

### Simple Auto Mission

```
WP 1: TAKEOFF  Alt=5m
WP 2: LAND
```

### Flight Mode Setup

```
FLTMODE1 = 5    (Loiter)      ← Safe default
FLTMODE4 = 3    (Auto)        ← Mission start
FLTMODE6 = 0    (Stabilize)   ← Emergency override
```

### Procedure

1. Place AprilTag (ID 2, 70mm) on flat ground
2. Position drone with camera over the tag
3. Wait for GPS lock (>10 sats, HDOP < 1.5)
4. Arm in Loiter mode
5. Switch to Auto → drone takes off to 5m, then lands with precision landing
6. Monitor: OpenMV green LED = tag detected
7. Emergency: switch to Loiter (position hold) or Stabilize (manual, hold throttle!)

## Build

```bash
./waf configure --board Pixhawk6X-bdshot
./waf copter
```

Firmware output: `build/Pixhawk6X-bdshot/bin/arducopter.apj`

## Version History

| Version | Date | Changes |
|---------|------|---------|
| v9.0 | Feb 2026 | Single-tag mode, NEGATE_YAW fix, mid-flight PLND_YAW_TGT disable fix |
| v8.0 | Feb 2026 | Watchdog timer, try/except, FOV correction, TAG36H11 filter |
| v7.x | Jan 2026 | Performance optimization (decimate, pre-alloc buffers), CRC fix |
| v6.x | Jan 2026 | Multi-marker support, monotonic timestamps, 3D angle geometry |

## Known Issues / Notes

- **NEGATE_YAW**: OpenMV's `z_rotation` has opposite sign from the `pupil_apriltags` library used in Gazebo simulation. `NEGATE_YAW = True` corrects this. If yaw direction is wrong in flight, toggle this flag.
- **Precision Loiter**: Requires AUX switch (`RC_OPTIONx = 39`). Only does XY positioning, no yaw alignment.
- **Camera offset**: Set `PLND_CAM_POS_X/Y/Z` for accurate CG-over-tag landing. Without it, the camera lens lands over the tag center.
- **EK3_SRC1_POSZ**: Should be `1` (Baro), NOT `2` (Rangefinder). The rangefinder is used via `EK3_RNG_USE_HGT` for terrain following, but the primary altitude source must be barometer for EKF stability.
