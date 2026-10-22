#!/usr/bin/env python3
"""
Send body-rate setpoints to PX4 via Offboard mode.

Uses SET_ATTITUDE_TARGET with type_mask = IGNORE_ATTITUDE (0b10000000),
so PX4's rate controller (inner loop) tracks your commanded p, q, r
while your outer-loop controller (e.g. LQR) supplies those rates.

Structure:
  - Background thread streams setpoints continuously at SETPOINT_RATE_HZ
    (PX4 drops out of offboard if it doesn't see a setpoint at least ~2Hz)
  - Main thread handles connection, pre-flight check, mode switch, and the
    actual control loop that computes rate commands (currently a placeholder —
    replace `compute_rate_setpoint()` with your LQR/PID output)
  - Vehicle must already be armed/flying (take off via QGroundControl first)
  - Ctrl+C triggers a safe shutdown: rates go to zero, returns to position hold
    (does NOT disarm mid-air — land manually via QGroundControl afterward)
"""

import time
import threading
from pymavlink import mavutil

# ---------------- CONFIG ----------------
CONNECTION_STRING = 'udp:127.0.0.1:14540'   # SITL. For RPi4 hardware bridge, use '/dev/serial0'
BAUD = 921600  # only used for serial connections (ignored for udp)
SETPOINT_RATE_HZ = 50                # streaming rate for setpoints (>=2Hz required, 50-100Hz recommended)
HOVER_THRUST = 0.5                   # normalized 0..1 — adjust for your rig; fixed if bench-testing on 3-DOF gimbal
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6    # PX4 custom_mode value for Offboard

# Shared state for the setpoint the streaming thread sends.
# Main loop updates this; streaming thread just sends whatever is current.
setpoint_lock = threading.Lock()
current_setpoint = {
    'roll_rate': 0.0,   # rad/s
    'pitch_rate': 0.0,  # rad/s
    'yaw_rate': 0.0,    # rad/s
    'thrust': 0.0,      # 0..1
}
streaming = True  # controls the background thread loop


def send_position_hold(conn, home_x, home_y, home_z):
    """Hold current position — used for pre-streaming and safe entry/exit around the rate-setpoint test."""
    conn.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,   # use position x,y,z only
        home_x, home_y, home_z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def send_attitude_target(conn, roll_rate, pitch_rate, yaw_rate, thrust):
    """Send one SET_ATTITUDE_TARGET message with rates only (attitude ignored)."""
    conn.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms
        conn.target_system,
        conn.target_component,
        0b10000000,                # type_mask: ignore attitude quaternion, use body rates + thrust
        [1.0, 0.0, 0.0, 0.0],      # dummy quaternion (ignored due to mask)
        roll_rate,
        pitch_rate,
        yaw_rate,
        thrust
    )


def setpoint_streamer(conn):
    """Runs in background thread. Sends whatever is currently in current_setpoint."""
    period = 1.0 / SETPOINT_RATE_HZ
    while streaming:
        with setpoint_lock:
            sp = dict(current_setpoint)
        send_attitude_target(conn, sp['roll_rate'], sp['pitch_rate'], sp['yaw_rate'], sp['thrust'])
        time.sleep(period)


def update_setpoint(roll_rate, pitch_rate, yaw_rate, thrust):
    """Call this from your control loop to update what's being streamed."""
    with setpoint_lock:
        current_setpoint['roll_rate'] = roll_rate
        current_setpoint['pitch_rate'] = pitch_rate
        current_setpoint['yaw_rate'] = yaw_rate
        current_setpoint['thrust'] = thrust


def wait_heartbeat(conn):
    """Wait for a heartbeat and confirm it's actually PX4 (autopilot==12), not some other component."""
    print("Waiting for PX4 heartbeat...")
    while True:
        msg = conn.wait_heartbeat()
        if msg.autopilot == 12:
            print(f"PX4 connected! System: {conn.target_system}, Component: {conn.target_component}")
            return
        else:
            print(f"Ignoring non-PX4 heartbeat (autopilot={msg.autopilot})...")


def verify_flying(conn, min_alt=3.0):
    """
    Confirm the vehicle is already airborne (armed/hovering via QGroundControl)
    before we touch offboard control. Mirrors the safety check in the working
    chirp script — refuses to proceed if not already flying.
    Returns the current NED position as the "home" reference for position hold.
    """
    print("Checking altitude...")
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("No position data received — is the vehicle connected/flying?")
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


def compute_rate_setpoint(t):
    """
    PLACEHOLDER — replace this with your actual outer-loop controller output
    (e.g. LQR: rates = K @ attitude_error).

    For now: returns zero rates + hover thrust, so the vehicle just holds
    still under offboard control once armed. Useful as a first connectivity/
    safety test before injecting real LQR/chirp commands.
    """
    roll_rate = 0.0
    pitch_rate = 0.0
    yaw_rate = 0.0
    thrust = HOVER_THRUST
    return roll_rate, pitch_rate, yaw_rate, thrust


def main():
    global streaming

    conn = mavutil.mavlink_connection(CONNECTION_STRING)
    wait_heartbeat(conn)

    # Vehicle must already be armed/flying (take off via QGroundControl first)
    home_x, home_y, home_z = verify_flying(conn)

    # PX4 requires a stream of setpoints BEFORE it will accept the offboard
    # mode switch — pre-stream position hold (safer than rates for entry).
    print("Pre-streaming position hold for 3 seconds...")
    for _ in range(int(20 * 3)):  # 20Hz for 3 seconds
        send_position_hold(conn, home_x, home_y, home_z)
        time.sleep(0.05)

    switch_to_offboard(conn)

    # Re-verify still flying right after the mode switch (mirrors working script's check)
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg is None or -msg.z < 3.0:
        raise RuntimeError("Vehicle lost altitude after mode switch — aborting.")
    print(f"Altitude after mode switch: {-msg.z:.2f}m — still flying.")

    # Start continuous rate-setpoint streaming now that offboard is confirmed active.
    stream_thread = threading.Thread(target=setpoint_streamer, args=(conn,), daemon=True)
    stream_thread.start()

    print("Entering control loop. Press Ctrl+C to stop safely.")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            roll_rate, pitch_rate, yaw_rate, thrust = compute_rate_setpoint(t)
            update_setpoint(roll_rate, pitch_rate, yaw_rate, thrust)
            time.sleep(0.02)  # 50Hz control loop — match or exceed SETPOINT_RATE_HZ

    except KeyboardInterrupt:
        print("\nShutdown requested.")

    finally:
        # Zero rates and switch back to position hold — do NOT disarm mid-air.
        print("Zeroing rates, returning to position hold...")
        streaming = False
        stream_thread.join(timeout=2)

        for _ in range(int(20 * 2)):  # 20Hz for 2 seconds
            send_position_hold(conn, home_x, home_y, home_z)
            time.sleep(0.05)

        print("Holding position. Land manually via QGroundControl when ready.")


if __name__ == '__main__':
    main()
