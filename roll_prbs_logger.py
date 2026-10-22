#!/usr/bin/env python3
"""
Roll-axis PRBS (pseudo-random binary sequence) injection for system
identification cross-validation, with CSV logging.

Same architecture as roll_chirp_logger.py: PX4-only heartbeat check,
verify_flying() pre-flight check, position-hold pre-stream, offboard switch,
background setpoint streamer + attitude listener threads, safe exit.

Only the excitation signal differs: instead of a swept-frequency chirp, this
sends a maximal-length PRBS on roll rate setpoint (p_sp), switching between
+/-amplitude at a fixed bit period. PRBS excites a broad frequency band
simultaneously (not sequentially like a chirp), so comparing a model fit
against this data is a good cross-check that identified dynamics aren't an
artifact of the chirp's sweep structure.

phi is read from PX4's own ATTITUDE message by default -- same caveat as
roll_chirp_logger.py: swap attitude_listener() for your MPU6050 pipeline
when running on the real gimbal instead of SITL.
"""

import time
import math
import threading
import csv
from pymavlink import mavutil

# ---------------- CONFIG ----------------
CONNECTION_STRING = 'udp:127.0.0.1:14540'   # SITL. For RPi4 hardware bridge, use '/dev/serial0'
SETPOINT_RATE_HZ = 50                # streaming/control rate
HOVER_THRUST = 0.7286                # normalized 0..1 -- adjust for your rig
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

# PRBS parameters (roll rate setpoint, rad/s)
PRBS_AMPLITUDE = 0.15        # rad/s -- matches roll_chirp_logger.py amplitude
PRBS_ORDER = 8                # sequence length = 2^order - 1 (255 for order 8)
PRBS_BIT_HOLD_SAMPLES = 5     # control ticks per PRBS bit (5 * 0.02s = 0.1s -> ~10Hz max content)
PRBS_PERIODS = 1              # number of full sequence repeats to run

# Safety cutoff: abort immediately if measured roll exceeds this, return to
# position hold. Same rationale as roll_chirp_logger.py -- critical on the
# real 3-DOF gimbal to avoid hitting mechanical stops.
ROLL_LIMIT_DEG = 15.0
ROLL_LIMIT_RAD = math.radians(ROLL_LIMIT_DEG)

OUTPUT_CSV = 'roll_prbs_log.csv'

# ---------------- SHARED STATE ----------------
setpoint_lock = threading.Lock()
current_setpoint = {'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0, 'thrust': 0.0}
streaming = True

attitude_lock = threading.Lock()
latest_attitude = {'phi': 0.0, 't': None}

log_rows = []   # (t, p_sp, phi) appended by main loop


# ---------------- PRBS GENERATION ----------------
# Maximal-length LFSR taps (Fibonacci form) for common orders.
_PRBS_TAPS = {
    2: [2, 1], 3: [3, 2], 4: [4, 3], 5: [5, 3],
    6: [6, 5], 7: [7, 6], 8: [8, 6, 5, 4], 9: [9, 5], 10: [10, 7],
}


def generate_prbs(order):
    """Returns a maximal-length +/-1 sequence of length 2^order - 1."""
    if order not in _PRBS_TAPS:
        raise ValueError(f"No tap set defined for order {order}")
    taps = _PRBS_TAPS[order]
    length = (1 << order) - 1
    reg = [1] * order   # initial state, all ones (must be nonzero)
    seq = []
    for _ in range(length):
        seq.append(1 if reg[-1] == 1 else -1)
        fb = 0
        for tap in taps:
            fb ^= reg[tap - 1]
        reg = [fb] + reg[:-1]
    return seq


_prbs_sequence = generate_prbs(PRBS_ORDER)
_prbs_bit_period_s = PRBS_BIT_HOLD_SAMPLES / SETPOINT_RATE_HZ
_prbs_total_duration = PRBS_PERIODS * len(_prbs_sequence) * _prbs_bit_period_s


def prbs_rate(t):
    """Returns roll rate setpoint in rad/s at time t, indexing into the
    precomputed PRBS sequence. Returns 0.0 once PRBS_PERIODS have completed."""
    if t > _prbs_total_duration:
        return 0.0
    bit_index = int(t / _prbs_bit_period_s) % len(_prbs_sequence)
    return PRBS_AMPLITUDE * _prbs_sequence[bit_index]


# ---------------- MAVLINK HELPERS (same as roll_chirp_logger.py) ----------------
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
        0b10000000,
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
    while streaming:
        msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg is not None:
            with attitude_lock:
                latest_attitude['phi'] = msg.roll
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


def save_csv():
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'p_sp', 'phi'])
        writer.writerows(log_rows)
    print(f"Saved {len(log_rows)} rows to {OUTPUT_CSV}")


def main():
    global streaming

    print(f"PRBS sequence length: {len(_prbs_sequence)} bits, "
          f"bit period: {_prbs_bit_period_s:.3f}s, "
          f"total duration: {_prbs_total_duration:.1f}s")

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

    print(f"Starting roll PRBS: amplitude {PRBS_AMPLITUDE} rad/s, "
          f"{_prbs_total_duration:.1f}s total. Press Ctrl+C to abort early.")
    t0 = time.time()
    aborted_for_safety = False
    try:
        while True:
            t = time.time() - t0
            p_sp = prbs_rate(t)
            update_setpoint(p_sp, 0.0, 0.0, HOVER_THRUST)

            with attitude_lock:
                phi = latest_attitude['phi']
            log_rows.append((t, p_sp, phi))

            if abs(phi) > ROLL_LIMIT_RAD:
                print(f"\nSAFETY ABORT: roll {math.degrees(phi):.1f}deg exceeds "
                      f"{ROLL_LIMIT_DEG}deg limit.")
                aborted_for_safety = True
                break

            if t > _prbs_total_duration:
                print("PRBS complete.")
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
        if aborted_for_safety:
            print("Run was safety-aborted -- inspect data before trusting it for ID.")
        print("Holding position. Land manually via QGroundControl when ready.")


if __name__ == '__main__':
    main()
