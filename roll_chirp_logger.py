#!/usr/bin/env python3
"""
Roll-axis chirp injection for system identification, with CSV logging.

Sends a chirp on roll rate setpoint (p_sp) via SET_ATTITUDE_TARGET
(type_mask = IGNORE_ATTITUDE, same as your working offboard_rate_setpoints.py),
while pitch_rate/yaw_rate stay at zero. Logs (time, p_sp, phi) to CSV for
offline MATLAB system ID (roll_sysid.m).

phi is read from PX4's own ATTITUDE message by default. If you're using the
MPU6050 as independent ground truth instead, swap attitude_listener() to read
from your RPi4 IMU pipeline and feed phi_current the same way.

Structure mirrors your working script:
  - PX4-only heartbeat check, verify_flying() pre-flight check (no self-arm)
  - Position-hold pre-stream before switching to offboard
  - Background thread streams rate setpoints at SETPOINT_RATE_HZ
  - Background thread listens for ATTITUDE messages, updates latest phi
  - Main loop computes chirp p_sp, logs (t, p_sp, phi) each cycle
  - Ctrl+C or chirp completion -> zero rates, return to position hold, save CSV
"""

import time
import math
import threading
import csv
from pymavlink import mavutil

# ---------------- CONFIG ----------------
CONNECTION_STRING = 'udp:127.0.0.1:14540'   # SITL. For RPi4 hardware bridge, use '/dev/serial0'
SETPOINT_RATE_HZ = 50                # streaming/control rate (>=2Hz required, 50Hz recommended)
HOVER_THRUST = 0.7286                # normalized 0..1 -- adjust for your rig
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

# Chirp parameters (roll rate setpoint, rad/s)
# NOTE: this is a RATE chirp, and rate integrates to angle. At low frequencies
# a near-constant rate is held for a long stretch, so peak angle excursion is
# roughly amplitude / (2*pi*f). Keep f0 high enough and amplitude low enough
# that even the lowest-frequency portion stays within a safe angle range.
CHIRP_AMPLITUDE = 0.15   # rad/s (~8.6 deg/s)
CHIRP_F0 = 0.3           # Hz, start frequency (raised from 0.1 -- bounds low-freq angle buildup)
CHIRP_F1 = 2.0           # Hz, end frequency
CHIRP_DURATION = 20.0    # seconds

# Safety cutoff: abort chirp immediately if measured roll exceeds this, and
# return to position hold. Critical once running on the real 3-DOF gimbal to
# avoid hitting mechanical stops.
ROLL_LIMIT_DEG = 15.0
ROLL_LIMIT_RAD = math.radians(ROLL_LIMIT_DEG)

OUTPUT_CSV = 'roll_chirp_log.csv'

# ---------------- SHARED STATE ----------------
setpoint_lock = threading.Lock()
current_setpoint = {'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0, 'thrust': 0.0}
streaming = True

attitude_lock = threading.Lock()
latest_attitude = {'phi': 0.0, 't': None}   # updated by attitude_listener

log_rows = []   # (t, p_sp, phi) appended by main loop


def send_position_hold(conn, home_x, home_y, home_z):
    conn.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        home_x, home_y, home_z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def send_attitude_target(conn, roll_rate, pitch_rate, yaw_rate, thrust):
    conn.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        conn.target_system, conn.target_component,
        0b10000000,                # ignore attitude quaternion, use body rates + thrust
        [1.0, 0.0, 0.0, 0.0],
        roll_rate, pitch_rate, yaw_rate, thrust
    )


def setpoint_streamer(conn):
    period = 1.0 / SETPOINT_RATE_HZ
    while streaming:
        with setpoint_lock:
            sp = dict(current_setpoint)
        send_attitude_target(conn, sp['roll_rate'], sp['pitch_rate'], sp['yaw_rate'], sp['thrust'])
        time.sleep(period)


def update_setpoint(roll_rate, pitch_rate, yaw_rate, thrust):
    with setpoint_lock:
        current_setpoint['roll_rate'] = roll_rate
        current_setpoint['pitch_rate'] = pitch_rate
        current_setpoint['yaw_rate'] = yaw_rate
        current_setpoint['thrust'] = thrust


def attitude_listener(conn):
    """Runs in background thread. Continuously updates latest_attitude['phi']
    from PX4's ATTITUDE message. Swap this for your MPU6050 read if using
    the IMU as independent ground truth instead."""
    while streaming:
        msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg is not None:
            with attitude_lock:
                latest_attitude['phi'] = msg.roll   # rad
                latest_attitude['t'] = time.time()


def wait_heartbeat(conn):
    print("Waiting for PX4 heartbeat...")
    while True:
        msg = conn.wait_heartbeat()
        if msg.autopilot == 12:
            print(f"PX4 connected! System: {conn.target_system}, Component: {conn.target_component}")
            return
        else:
            print(f"Ignoring non-PX4 heartbeat (autopilot={msg.autopilot})...")


def verify_flying(conn, min_alt=3.0):
    print("Checking altitude...")
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("No position data received -- is the vehicle connected/flying?")
    alt = -msg.z
    print(f"Current altitude: {alt:.2f}m")
    if alt < min_alt:
        raise RuntimeError("Drone not flying! Take off in QGroundControl first.")
    print(f"Home position saved: x={msg.x:.2f} y={msg.y:.2f} alt={alt:.2f}m")
    return msg.x, msg.y, msg.z


def switch_to_offboard(conn):
    print("Switching to Offboard mode...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0, 0, 0, 0, 0
    )
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Offboard mode switch failed or not confirmed: {ack}")
    print("Offboard mode active.")


def chirp_rate(t):
    """Linear-sweep chirp: instantaneous frequency goes from CHIRP_F0 to CHIRP_F1
    over CHIRP_DURATION seconds. Returns roll rate setpoint in rad/s."""
    if t > CHIRP_DURATION:
        return 0.0
    k = (CHIRP_F1 - CHIRP_F0) / CHIRP_DURATION
    phase = 2 * math.pi * (CHIRP_F0 * t + 0.5 * k * t * t)
    return CHIRP_AMPLITUDE * math.sin(phase)


def save_csv():
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'p_sp', 'phi'])
        writer.writerows(log_rows)
    print(f"Saved {len(log_rows)} rows to {OUTPUT_CSV}")


def main():
    global streaming

    conn = mavutil.mavlink_connection(CONNECTION_STRING)
    wait_heartbeat(conn)

    home_x, home_y, home_z = verify_flying(conn)

    print("Pre-streaming position hold for 3 seconds...")
    for _ in range(int(20 * 3)):
        send_position_hold(conn, home_x, home_y, home_z)
        time.sleep(0.05)

    switch_to_offboard(conn)

    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg is None or -msg.z < 3.0:
        raise RuntimeError("Vehicle lost altitude after mode switch -- aborting.")
    print(f"Altitude after mode switch: {-msg.z:.2f}m -- still flying.")

    stream_thread = threading.Thread(target=setpoint_streamer, args=(conn,), daemon=True)
    stream_thread.start()

    attitude_thread = threading.Thread(target=attitude_listener, args=(conn,), daemon=True)
    attitude_thread.start()

    print(f"Starting roll chirp: {CHIRP_F0}-{CHIRP_F1} Hz over {CHIRP_DURATION}s, "
          f"amplitude {CHIRP_AMPLITUDE} rad/s. Press Ctrl+C to abort early.")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            p_sp = chirp_rate(t)
            update_setpoint(p_sp, 0.0, 0.0, HOVER_THRUST)

            with attitude_lock:
                phi = latest_attitude['phi']
            log_rows.append((t, p_sp, phi))

            if t > CHIRP_DURATION:
                print("Chirp complete.")
                break

            time.sleep(1.0 / SETPOINT_RATE_HZ)

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        print("Zeroing rates, returning to position hold...")
        streaming = False
        stream_thread.join(timeout=2)
        attitude_thread.join(timeout=2)

        for _ in range(int(20 * 2)):
            send_position_hold(conn, home_x, home_y, home_z)
            time.sleep(0.05)

        save_csv()
        print("Holding position. Land manually via QGroundControl when ready.")


if __name__ == '__main__':
    main()
