# MAVLink 2 AprilTags Precision Landing - PRODUCTION VERSION
# OpenMV RT1062 + Pixhawk 6X + ArduPilot
# Version 9.0 - Single-tag mode + yaw sign fix
#
# Key features:
#   - Corrected FOV (70.8x55.6 deg) from official OpenMV specs
#   - Dynamic scale factor calibrated to actual f_x
#   - TAG36H11 family filter (faster, fewer false positives)
#   - Watchdog timer for crash recovery
#   - Error-resilient main loop (try/except)
#   - Monotonic timestamps for ArduPilot jitter correction
#   - SINGLE TAG mode: uses tag ID 2 (70mm), landing point = tag center
#   - NEGATE_YAW: fixes yaw sign to match Gazebo simulation convention
#   - Pre-computed MAVLink header + bytearray reuse
#   - decimate=2 at QVGA (quad detect on 160x120, pose on 320x240)
#
# LED: Green = Tag detected, Red = No tag

import image, math, sensor, struct, time, machine

# LEDs (OpenMV RT1062: physical green is on "LED_RED" pin, physical red on "LED_GREEN")
led_green = machine.Pin("LED_RED", machine.Pin.OUT)
led_red = machine.Pin("LED_GREEN", machine.Pin.OUT)
led_green.off()
led_red.off()
time.sleep(0.1)

# Watchdog timer: resets the board if main loop hangs for >10 seconds
# (e.g., sensor.snapshot() freeze, I2C lockup, infinite loop)
try:
    _wdt = machine.WDT(timeout=10000)
except Exception:
    _wdt = None  # WDT not available on all boards/firmwares

# =============================================================================
# CONFIGURATION
# =============================================================================

uart_baudrate = 115200
MAV_system_id = 1
MAV_component_id = 0x54

lens_to_camera_mm = 22

# Official FOV for OpenMV RT1062 + standard 2.8mm M12 lens + OV5640
# Source: OpenMV product specs (openmv.io / sparkfun.com)
# NOTE: Datasheet sensor_w/h_mm values (4.592/3.423) are WRONG for this
# lens+sensor combo and produce ~12% angle error causing oscillation!
H_FOV_DEG = 70.8
V_FOV_DEG = 55.6

# =============================================================================
# YAW SIGN CORRECTION
# =============================================================================
# OpenMV's tag.z_rotation uses a different sign convention than the
# pupil_apriltags/dt_apriltags libraries used in the Gazebo simulation.
# The Gazebo sim extracts yaw as atan2(R[1,0], R[0,0]) from the rotation
# matrix, while OpenMV returns z_rotation directly.
# When the drone rotates in the WRONG direction during yaw alignment,
# set NEGATE_YAW = True to flip the sign.
# This is equivalent to the --invert-yaw flag in apriltag_mavlink_sim.py.
NEGATE_YAW = True

# =============================================================================
# SINGLE TAG MODE
# =============================================================================
# Use ONLY tag ID 2 (70mm, the 3rd smallest marker).
# Landing point = center of this tag (no offset correction needed).
# This eliminates offset errors from inaccurate marker placement/scaling
# and simplifies the detection pipeline.
TAG_ID = 2
TAG_SIZE_MM = 70

# =============================================================================
# CAMERA SETUP
# =============================================================================
# Resolution choice:
#   QQVGA (160x120): ~10-12 FPS, good detection, lower pose accuracy
#   QVGA  (320x240): ~8-9 FPS, best detection + accuracy, higher latency
USE_QVGA = True

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)

if USE_QVGA:
    sensor.set_framesize(sensor.QVGA)
    x_res = 320
    y_res = 240
    _DECIMATE = 2   # Detect on 160x120, pose on 320x240
else:
    sensor.set_framesize(sensor.QQVGA)
    x_res = 160
    y_res = 120
    _DECIMATE = 1   # Full resolution, already small enough

sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)       # Disabled: reduces motion blur from vibrations
sensor.set_auto_whitebal(False)   # Disabled: irrelevant for grayscale
sensor.set_auto_exposure(True)    # MUST be enabled: adapts to changing light conditions

# Pre-compute camera intrinsics from official FOV values
h_fov = math.radians(H_FOV_DEG)
v_fov = math.radians(V_FOV_DEG)
f_x = (x_res * 0.5) / math.tan(h_fov * 0.5)   # QQVGA: ~112.6, QVGA: ~225.1
f_y = (y_res * 0.5) / math.tan(v_fov * 0.5)    # QQVGA: ~113.8, QVGA: ~227.6
c_x = x_res * 0.5
c_y = y_res * 0.5
_inv_x_res = 1.0 / x_res
_inv_y_res = 1.0 / y_res

# Pre-compute scale factor (converts AprilTag translations to mm).
# The factor 165 was empirically calibrated with old (wrong) f_x=97.56 at QQVGA.
# AprilTag translations scale proportionally with f_x, so we must adjust:
#   scale_factor = 165.0 * (f_x_actual / f_x_reference)
# This ensures z_t * scale gives correct mm values.
_F_X_REF = 97.56  # Old QQVGA f_x (lens_mm=2.8, sensor_w=4.592)
_SCALE_FACTOR = 165.0 * (f_x / _F_X_REF)
TAG_SCALE = (100.0 * TAG_SIZE_MM) / _SCALE_FACTOR

# =============================================================================
# PRE-COMPUTED MAVLINK CONSTANTS
# =============================================================================

uart = machine.UART(1, uart_baudrate, timeout_char=100)  # 100ms RX timeout (TX is non-blocking)

# MAVLink constants
_MSG_ID = 149
_FRAME = 12
_EXTRA_CRC = 200
_PAYLOAD_FMT = "<QfffffBBfffffffBB"
_PAYLOAD_LEN = struct.calcsize(_PAYLOAD_FMT)  # 60 bytes

# Pre-compute the static part of the MAVLink header
# Only packet_sequence changes per frame
_msgid_bytes = struct.pack("<I", _MSG_ID)[:3]

# Pre-allocate output buffer (1 + 9 + 60 + 2 = 72 bytes)
_packet_buf = bytearray(72)
_packet_buf[0] = 0xFD  # MAVLink 2 start byte
# Bytes 1-9: header (partially static)
_packet_buf[1] = _PAYLOAD_LEN  # msg_len
_packet_buf[2] = 0  # incompat_flags
_packet_buf[3] = 0  # compat_flags
# [4] = packet_sequence (set per frame)
_packet_buf[5] = MAV_system_id
_packet_buf[6] = MAV_component_id
_packet_buf[7] = _msgid_bytes[0]
_packet_buf[8] = _msgid_bytes[1]
_packet_buf[9] = _msgid_bytes[2]

# =============================================================================
# OPTIMIZED FUNCTIONS
# =============================================================================

# Monotonic timestamp tracking
_last_ticks = time.ticks_us()
_accumulated_us = 0

def get_timestamp_us():
    global _last_ticks, _accumulated_us
    now = time.ticks_us()
    delta = time.ticks_diff(now, _last_ticks)
    _last_ticks = now
    if delta > 0:
        _accumulated_us += delta
    return _accumulated_us


def _crc_accumulate(data, crc):
    """Accumulate MAVLink CRC-16/MCRF4XX over data bytes."""
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


# Packet sequence counter
_pkt_seq = 0

def send_landing_target(tag, capture_us):
    """Send LANDING_TARGET MAVLink2 message. Single-tag mode: no offset correction.
    Optimized: pre-allocated buffer, minimal allocations."""
    global _pkt_seq

    # --- 3D position (for body_x/y/z fields — not used by ArduPilot) ---
    _k = TAG_SCALE
    x_t = tag.x_translation
    y_t = tag.y_translation
    z_t = tag.z_translation
    # No offset correction needed: landing point = tag center
    x_mm = x_t * _k
    y_mm = y_t * _k
    z_mm = z_t * _k - lens_to_camera_mm
    body_x = z_mm * 0.001
    body_y = x_mm * 0.001
    body_z = y_mm * 0.001

    # --- Angles from 3D geometry (CRITICAL: this is what ArduPilot uses) ---
    # No offset correction in single-tag mode: landing point = tag center.
    # Angles are computed directly from tag translations.
    # Cache math functions locally (MicroPython optimization)
    _atan2 = math.atan2

    if z_t > 0.01:
        angle_x = _atan2(x_t, z_t)
        angle_y = -_atan2(y_t, z_t)
    else:
        angle_x = 0.0
        angle_y = 0.0

    # --- Distance and size ---
    dist = math.sqrt(body_x * body_x + body_y * body_y + body_z * body_z)
    size_x = tag.w * _inv_x_res * h_fov
    size_y = tag.h * _inv_y_res * v_fov

    # --- Yaw quaternion ---
    # NEGATE_YAW: OpenMV's z_rotation has opposite sign convention compared
    # to the pupil_apriltags library used in the Gazebo simulation.
    # The simulation code extracts yaw as atan2(R[1,0], R[0,0]) which
    # produces the opposite sign from OpenMV's z_rotation.
    # Negating the yaw ensures the drone rotates in the CORRECT direction.
    yaw = tag.z_rotation
    if NEGATE_YAW:
        yaw = -yaw
    if yaw > 3.14159265:
        yaw -= 6.28318530
    elif yaw < -3.14159265:
        yaw += 6.28318530
    hy = yaw * 0.5
    qw = math.cos(hy)
    qz = math.sin(hy)

    # --- Pack payload directly into buffer ---
    payload = struct.pack(
        _PAYLOAD_FMT,
        capture_us, angle_x, angle_y, dist, size_x, size_y,
        tag.id, _FRAME,
        body_x, body_y, body_z, qw, 0.0, 0.0, qz, 0, 1,
    )

    # --- Set sequence byte and compute CRC ---
    _packet_buf[4] = _pkt_seq & 0xFF

    # MAVLink CRC order: header[1:10] → payload → extra_crc_byte
    crc = _crc_accumulate(memoryview(_packet_buf)[1:10], 0xFFFF)
    crc = _crc_accumulate(payload, crc)
    # Extra CRC byte (msg-specific seed) goes LAST
    crc = _crc_accumulate(bytes([_EXTRA_CRC]), crc)

    # Copy payload and CRC into buffer
    _packet_buf[10:70] = payload
    _packet_buf[70] = crc & 0xFF
    _packet_buf[71] = (crc >> 8) & 0xFF

    uart.write(_packet_buf)
    _pkt_seq += 1

    return z_mm, yaw


# Single-tag detection: find the specific TAG_ID in the detected tags
def find_target_tag(tags):
    """Find TAG_ID in the list of detected tags. Returns tag or None."""
    for t in tags:
        if t.id == TAG_ID:
            return t
    return None


# =============================================================================
# MAIN LOOP
# =============================================================================

clock = time.clock()
print("=" * 50)
print("MAVLink 2 Precision Landing v9.0")
print("SINGLE TAG MODE: ID=%d  size=%dmm" % (TAG_ID, TAG_SIZE_MM))
print("Res: %dx%d  decimate:%d" % (x_res, y_res, _DECIMATE))
print("FOV: %.1f x %.1f deg" % (math.degrees(h_fov), math.degrees(v_fov)))
print("f_x=%.1f f_y=%.1f  scale=%.3f" % (f_x, f_y, TAG_SCALE))
print("NEGATE_YAW: %s" % NEGATE_YAW)
print("WDT: %s" % ("active" if _wdt else "disabled"))
print("=" * 50)

# Self-test: capture one frame to verify sensor is working
try:
    _test_img = sensor.snapshot()
    print("SELF-TEST: Sensor OK (%dx%d)" % (_test_img.width(), _test_img.height()))
    del _test_img
except Exception as e:
    print("SELF-TEST: SENSOR FAILED - %s" % str(e))

msg_count = 0
frame_count = 0
miss_count = 0
error_count = 0

# Latency tracking (EMA)
avg_det_ms = 0.0
avg_send_ms = 0.0

while True:
    try:
        # Feed watchdog at start of each iteration
        if _wdt:
            _wdt.feed()

        clock.tick()
        frame_count += 1

        # --- Capture ---
        # Timestamp AFTER snapshot: that's when the frame data is actually available.
        # The real exposure happened slightly before, but this is the most consistent
        # reference point. PLND_LAG compensates for the remaining pipeline delay.
        img = sensor.snapshot()
        t0 = get_timestamp_us()
        t1 = t0

        # --- Detect ---
        tags = img.find_apriltags(families=image.TAG36H11, fx=f_x, fy=f_y, cx=c_x, cy=c_y, decimate=_DECIMATE)
        t2 = get_timestamp_us()

        # --- Find target tag ---
        tag = find_target_tag(tags)

        if tag is not None:
            # --- Send ---
            z_mm, yaw = send_landing_target(tag, t0)
            t3 = get_timestamp_us()
            msg_count += 1

            # Update timing averages (EMA alpha=0.2)
            det_ms = (t2 - t1) * 0.001
            send_ms = (t3 - t2) * 0.001
            avg_det_ms = avg_det_ms * 0.8 + det_ms * 0.2
            avg_send_ms = avg_send_ms * 0.8 + send_ms * 0.2

            # Status every 20 detections
            if msg_count % 20 == 0:
                det_pct = msg_count * 100 // frame_count if frame_count > 0 else 0
                tot = avg_det_ms + avg_send_ms
                print(
                    "[%04d] ID:%d z:%.1fm yaw:%+.0f° %.1ffps det:%d%% | det:%.0f send:%.0f tot:%.0fms"
                    % (msg_count, tag.id, z_mm * 0.001, math.degrees(yaw),
                       clock.fps(), det_pct, avg_det_ms, avg_send_ms, tot)
                )

            led_green.on()
            led_red.off()
        else:
            miss_count += 1
            led_green.off()
            led_red.on()

            if miss_count % 100 == 0:
                det_pct = msg_count * 100 // frame_count if frame_count > 0 else 0
                print("[MISS] %d/%d det:%d%% %.1ffps" % (miss_count, frame_count, det_pct, clock.fps()))

    except MemoryError:
        # Critical: out of memory. Force garbage collection and continue.
        import gc
        gc.collect()
        error_count += 1
        print("[ERR] MemoryError #%d, gc done" % error_count)
    except Exception as e:
        # Non-critical: log and continue. The main loop must never stop.
        error_count += 1
        if error_count <= 10 or error_count % 100 == 0:
            print("[ERR] #%d: %s" % (error_count, str(e)))
