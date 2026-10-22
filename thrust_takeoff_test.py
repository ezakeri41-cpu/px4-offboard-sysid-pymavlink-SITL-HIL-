#!/usr/bin/env python3
"""
Thrust-only takeoff test.

Sends zero roll/pitch/yaw RATE setpoints (so PX4's rate controller stays
active and stabilizes attitude) while ramping thrust up, holding, then
ramping back down. Logs altitude so you can see if/when it leaves the
ground. Same offboard mechanism (SET_ATTITUDE_TARGET, ignore-attitude
bit) as your working roll_chirp_logger.py.

Purpose: find/verify a working hover thrust value on this AirSim HIL
setup before trusting HOVER_THRUST in the chirp script.
"""

import time
import threading
from pymavlink import mavutil

# ---------------- CONFIG ----------------
CONNECTION_STRING = 'udp:127.0.0.1:14550'   # AirSim HIL relay
SETPOINT_RATE_HZ = 50
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

RAMP_UP_TIME_S   = 5.0     # time to ramp thrust from 0 to TARGET_THRUST
HOLD_TIME_S      = 10.0    # time to hold at TARGET_THRUST
RAMP_DOWN_TIME_S = 5.0     # time to ramp back to 0

TARGET_THRUST = 0.5        # normalized [0,1] -- START LOW, increase gradually
                             # across runs until you see it lift off. Don't
                             # jump straight to a guessed hover value.
# -----------------------------------------

setpoint_lock = threading.Lock()
current_thrust = 0.0
streaming = True


def send_attitude_target(conn, thrust):
    conn.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        conn.target_system, conn.target_component,
        0b10000000,                 # ignore attitude quaternion, use body rates + thrust
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, 0.0,              # roll_rate, pitch_rate, yaw_rate = 0
        thrust
    )


def setpoint_streamer(conn):
    period = 1.0 / SETPOINT_RATE_HZ
    while streaming:
        with setpoint_lock:
            t = current_thrust
        send_attitude_target(conn, t)
        time.sleep(period)


def set_thrust(value):
    global current_thrust
    with setpoint_lock:
        current_thrust = value


def altitude_listener(conn, alt_holder):
    while streaming:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg is not None:
            alt_holder['alt'] = -msg.z


def wait_heartbeat(conn):
    print("Waiting for PX4 heartbeat...")
    while True:
        msg = conn.wait_heartbeat()
        if msg.autopilot == 12:
            print(f"PX4 connected! System: {conn.target_system}, Component: {conn.target_component}")
            return


def switch_to_offboard(conn):
    print("Switching to Offboard mode...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0
    )
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Offboard mode switch failed or not confirmed: {ack}")
    print("Offboard mode active.")


def arm(conn, value):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if value else 0, 0, 0, 0, 0, 0, 0
    )


def main():
    global streaming

    conn = mavutil.mavlink_connection(CONNECTION_STRING)
    wait_heartbeat(conn)

    print("Pre-streaming zero-thrust setpoints (required before OFFBOARD switch)...")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        send_attitude_target(conn, 0.0)
        time.sleep(1.0 / SETPOINT_RATE_HZ)

    switch_to_offboard(conn)

    print("Arming...")
    arm(conn, True)
    time.sleep(1.0)

    alt_holder = {'alt': 0.0}
    stream_thread = threading.Thread(target=setpoint_streamer, args=(conn,), daemon=True)
    stream_thread.start()
    alt_thread = threading.Thread(target=altitude_listener, args=(conn, alt_holder), daemon=True)
    alt_thread.start()

    try:
        print(f"Ramping thrust 0 -> {TARGET_THRUST} over {RAMP_UP_TIME_S}s...")
        steps = int(RAMP_UP_TIME_S * SETPOINT_RATE_HZ)
        for i in range(steps):
            set_thrust(TARGET_THRUST * (i + 1) / steps)
            print(f"  thrust={current_thrust:.3f}  alt={alt_holder['alt']:.2f}m")
            time.sleep(1.0 / SETPOINT_RATE_HZ)

        print(f"Holding thrust={TARGET_THRUST} for {HOLD_TIME_S}s...")
        t_hold = time.time()
        while time.time() - t_hold < HOLD_TIME_S:
            print(f"  thrust={TARGET_THRUST:.3f}  alt={alt_holder['alt']:.2f}m")
            time.sleep(0.5)

        print(f"Ramping thrust down over {RAMP_DOWN_TIME_S}s...")
        steps = int(RAMP_DOWN_TIME_S * SETPOINT_RATE_HZ)
        for i in range(steps):
            set_thrust(TARGET_THRUST * (1 - (i + 1) / steps))
            time.sleep(1.0 / SETPOINT_RATE_HZ)

    except KeyboardInterrupt:
        print("\nAborted by user -- cutting thrust.")
        set_thrust(0.0)
        time.sleep(0.3)

    finally:
        print("Disarming...")
        streaming = False
        stream_thread.join(timeout=2)
        alt_thread.join(timeout=2)
        arm(conn, False)
        print("Done.")


if __name__ == '__main__':
    main()
