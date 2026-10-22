#!/usr/bin/env python3
"""
Bypass validation test: sustained asymmetric torque at the ACTUATOR level.

Sends a constant roll bias via SET_ACTUATOR_CONTROL_TARGET (mixer-level,
bypassing position/velocity/attitude/rate control entirely) plus base
thrust, and logs attitude + body rates.

Expected result if the bypass is real: roll angle/rate should NOT settle
or self-correct -- it should keep diverging for as long as the bias is
held, because nothing is closing an attitude/rate loop around it.
If you instead see it stabilize at some tilted-but-bounded angle, or
the rate settle to a fixed value, a controller is still active somewhere.

This is a SIM-ONLY test (AirSim HIL) -- letting it diverge/tumble is the
whole point and costs nothing. A generous attitude safety limit just
stops logging and cuts thrust once divergence is clearly confirmed, so
you don't have to watch it indefinitely or manually reset the sim state
every time.
"""

import time
import math
import threading
import csv
from pymavlink import mavutil

# ---------------- CONFIG ----------------
CONNECTION_STRING = 'udp:127.0.0.1:14550'   # AirSim HIL relay
SETPOINT_RATE_HZ = 50
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

BASE_THRUST = 0.5      # normalized [0,1] -- use whatever value you confirmed lifts off
ROLL_BIAS   = 0.15     # normalized [-1,1] -- sustained, asymmetric actuator-level roll command

CLIMB_TIME_S     = 4.0    # hold base thrust (rate-level, stabilized) to get airborne
                            # before switching to actuator-level bias -- avoids the
                            # ground-contact confound (tipping over ON the ground is
                            # not the same thing as diverging IN THE AIR)
MAX_DURATION_S   = 15.0   # hard stop regardless of outcome
ROLL_SAFETY_DEG  = 70.0   # stop early once divergence is unambiguous (sim-only, generous limit)
ROLL_SAFETY_RAD  = math.radians(ROLL_SAFETY_DEG)

OUTPUT_CSV = 'bypass_validation_log.csv'
# -----------------------------------------

setpoint_lock = threading.Lock()
current_controls = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'thrust': 0.0}
streaming = True

attitude_lock = threading.Lock()
latest_attitude = {'roll': 0.0, 'rollspeed': 0.0, 'pitch': 0.0, 't': None}

log_rows = []   # (t, roll_cmd, roll_meas, rollspeed_meas)


def send_actuator_control(conn, roll, pitch, yaw, thrust):
    controls = [roll, pitch, yaw, thrust, 0.0, 0.0, 0.0, 0.0]
    conn.mav.set_actuator_control_target_send(
        int(time.time() * 1e6) & 0xFFFFFFFF,
        0,                          # group_mlx: 0 = main outputs
        conn.target_system, conn.target_component,
        controls,
    )


def setpoint_streamer(conn):
    period = 1.0 / SETPOINT_RATE_HZ
    while streaming:
        with setpoint_lock:
            c = dict(current_controls)
        send_actuator_control(conn, c['roll'], c['pitch'], c['yaw'], c['thrust'])
        time.sleep(period)


def set_controls(roll, pitch, yaw, thrust):
    with setpoint_lock:
        current_controls['roll'] = roll
        current_controls['pitch'] = pitch
        current_controls['yaw'] = yaw
        current_controls['thrust'] = thrust


def attitude_listener(conn):
    while streaming:
        msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg is not None:
            with attitude_lock:
                latest_attitude['roll'] = msg.roll
                latest_attitude['rollspeed'] = msg.rollspeed
                latest_attitude['pitch'] = msg.pitch
                latest_attitude['t'] = time.time()


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
        PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0,
    )
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"Offboard mode switch failed or not confirmed: {ack}")
    print("Offboard mode active.")


def arm(conn, value):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if value else 0, 0, 0, 0, 0, 0, 0,
    )


def save_csv():
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'roll_cmd_norm', 'roll_meas_rad', 'rollspeed_meas_rad_s'])
        writer.writerows(log_rows)
    print(f"Saved {len(log_rows)} rows to {OUTPUT_CSV}")


def main():
    global streaming

    conn = mavutil.mavlink_connection(CONNECTION_STRING)
    wait_heartbeat(conn)

    print(f"Pre-streaming actuator setpoints at BASE_THRUST={BASE_THRUST} "
          f"(NOT zero -- vehicle is assumed already airborne/hovering via QGC; "
          f"streaming zero here would cause a thrust drop the instant offboard "
          f"is accepted)...")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        send_actuator_control(conn, 0, 0, 0, BASE_THRUST)
        time.sleep(1.0 / SETPOINT_RATE_HZ)

    switch_to_offboard(conn)
    # NOTE: no arm() call here -- vehicle is assumed already armed and flying
    # (took off manually via QGC). Sending an extra arm command is harmless/
    # idempotent if you do want to keep it as a safety net, but it's not
    # needed for this flow.

    stream_thread = threading.Thread(target=setpoint_streamer, args=(conn,), daemon=True)
    stream_thread.start()
    attitude_thread = threading.Thread(target=attitude_listener, args=(conn,), daemon=True)
    attitude_thread.start()

    try:
        print(f"Already airborne at BASE_THRUST={BASE_THRUST} (symmetric, no bias yet). "
              f"Confirming stable hold for {CLIMB_TIME_S}s before introducing any bias...")
        t_climb = time.time()
        while time.time() - t_climb < CLIMB_TIME_S:
            set_controls(0, 0, 0, BASE_THRUST)
            with attitude_lock:
                roll = latest_attitude['roll']
            print(f"  climbing...  roll={math.degrees(roll):6.2f}deg")
            time.sleep(1.0 / SETPOINT_RATE_HZ)

        print(f"Now applying sustained roll bias={ROLL_BIAS} at thrust={BASE_THRUST}. "
              f"Watch roll -- it should NOT settle if the bypass is real. "
              f"Auto-stop at {ROLL_SAFETY_DEG} deg or {MAX_DURATION_S}s.")
        t_start = time.time()
        while True:
            t = time.time() - t_start
            if t > MAX_DURATION_S:
                print("Max duration reached.")
                break

            set_controls(ROLL_BIAS, 0, 0, BASE_THRUST)

            with attitude_lock:
                roll = latest_attitude['roll']
                rollspeed = latest_attitude['rollspeed']

            log_rows.append((t, ROLL_BIAS, roll, rollspeed))
            print(f"  t={t:5.2f}s  roll={math.degrees(roll):7.2f}deg  "
                  f"rollspeed={math.degrees(rollspeed):7.2f}deg/s")

            if abs(roll) > ROLL_SAFETY_RAD:
                print(f"Roll exceeded {ROLL_SAFETY_DEG} deg -- divergence confirmed, stopping.")
                break

            time.sleep(1.0 / SETPOINT_RATE_HZ)

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        print("Cutting thrust, disarming...")
        set_controls(0, 0, 0, 0)
        time.sleep(0.3)
        streaming = False
        stream_thread.join(timeout=2)
        attitude_thread.join(timeout=2)
        arm(conn, False)
        save_csv()
        print("Done. Reset the sim vehicle in AirSim/QGC before your next run.")


if __name__ == '__main__':
    main()
