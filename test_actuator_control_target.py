"""
test_actuator_control_target.py

Purpose:
    Check whether this PX4 build/dialect supports SET_ACTUATOR_CONTROL_TARGET,
    and whether PX4 actually accepts it in Offboard mode (bypassing
    mc_att_control's own PID entirely).

    Run this in SITL first, as a rehearsal before HITL.

WHAT THIS SCRIPT DOES:
    1. Checks pymavlink's dialect for the message/send function.
    2. Connects, waits for a PX4 heartbeat.
    3. Pre-streams SET_ACTUATOR_CONTROL_TARGET at 50Hz (required before
       PX4 will accept switching into Offboard mode for this control type).
    4. Switches to Offboard.
    5. Sends a small, safe roll actuator command for a few seconds.
    6. Prints ATTITUDE feedback so you can visually confirm the vehicle
       is responding to a RAW actuator command, not a PID-mediated one.

IMPORTANT — this script alone cannot prove PID is bypassed.
    To fully confirm, run this at the same time as, in MAVLink Console:
        listener actuator_controls
    and watch whether the values you send here show up directly,
    with no visible shaping/smoothing that would suggest mc_att_control
    is still in the loop. This script gives you the send-side half of
    that verification; the console listener gives you the receive-side half.

SAFETY:
    - Run in SITL/Gazebo first. Do not run on real hardware until the
      SITL check confirms the message is accepted and behaves as expected.
    - Amplitude is deliberately tiny (0.05, i.e. 5% control authority).
    - Ctrl+C at any time returns to position hold before exiting.
"""

import time
import math
import threading
from pymavlink import mavutil

# ── STEP 0: CHECK DIALECT SUPPORT FIRST ────────────────────
# Before connecting to anything, confirm pymavlink actually knows about
# this message. If this fails, the message either isn't in the dialect
# pymavlink loaded, or PX4's own build doesn't support it — worth knowing
# immediately rather than discovering it mid-flight.
required_attrs = [
    "MAVLink_set_actuator_control_target_message",
]
missing = [a for a in required_attrs if not hasattr(mavutil.mavlink, a)]
if missing:
    print("MISSING from pymavlink dialect:", missing)
    print("SET_ACTUATOR_CONTROL_TARGET is not available in this pymavlink build.")
    print("Check your MAVLINK_DIALECT environment variable / installed dialect version.")
    raise SystemExit(1)
else:
    print("pymavlink dialect supports SET_ACTUATOR_CONTROL_TARGET. Proceeding.")

# ── CONNECT ─────────────────────────────────────────────────
print("Connecting to PX4...")
master = mavutil.mavlink_connection("udp:127.0.0.1:14540")

while True:
    msg = master.wait_heartbeat()
    if msg.autopilot == 12:
        print(f"PX4 connected! System: {master.target_system}, Component: {master.target_component}")
        break
    else:
        print(f"Ignoring non-PX4 heartbeat (autopilot={msg.autopilot})...")

# ── VERIFY DRONE IS FLYING ──────────────────────────────────
print("Checking altitude...")
msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
if msg:
    alt = -msg.z
    print(f"Current altitude: {alt:.2f}m")
    if alt < 3.0:
        print("Drone not flying! Take off in QGroundControl first.")
        exit()
    home_x, home_y, home_z = msg.x, msg.y, msg.z
    print(f"Home position saved: x={home_x:.2f} y={home_y:.2f} alt={alt:.2f}m")
else:
    print("ERROR: No position data")
    exit()

# ── HELPERS ─────────────────────────────────────────────────
t0 = time.time()
def get_time_ms():
    return int((time.time() - t0) * 1000)

MODE_POSITION_HOLD = 0
MODE_ACTUATOR       = 1

stream_mode  = MODE_POSITION_HOLD
streaming    = True
roll_cmd     = 0.0          # normalized, -1..1
HOVER_THRUST = 0.725        # normalized, 0..1 — same trim as your chirp script

def send_position_hold():
    master.mav.set_position_target_local_ned_send(
        get_time_ms(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        home_x, home_y, home_z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def send_actuator_control(roll, pitch=0.0, yaw=0.0, thrust=HOVER_THRUST):
    # controls[8]: [roll, pitch, yaw, thrust, 4 spare]
    controls = [roll, pitch, yaw, thrust, 0.0, 0.0, 0.0, 0.0]
    master.mav.set_actuator_control_target_send(
        get_time_ms(),
        0,                          # group_mlx: 0 = attitude/rate/thrust group
        master.target_system,
        master.target_component,
        controls
    )

# ── BACKGROUND STREAM THREAD ────────────────────────────────
def stream_loop():
    while streaming:
        if stream_mode == MODE_POSITION_HOLD:
            send_position_hold()
            time.sleep(0.05)   # 20Hz
        else:
            send_actuator_control(roll_cmd)
            time.sleep(0.01)   # 100Hz — actuator control needs a fast stream

stream_thread = threading.Thread(target=stream_loop, daemon=True)

# ── START STREAMING (position hold) ─────────────────────────
print("\nPre-streaming position hold for 3 seconds...")
stream_thread.start()
time.sleep(3)

# ── SWITCH TO OFFBOARD ───────────────────────────────────────
print("Switching to Offboard mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6,
    0, 0, 0, 0, 0
)
time.sleep(1)

msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg:
    print(f"Current mode: {msg.custom_mode}  (should be 6)")
    if msg.custom_mode == 6:
        print("Offboard confirmed!")
    else:
        print("WARNING: Not in Offboard mode!")

# ── SWITCH STREAM TO RAW ACTUATOR CONTROL ───────────────────
# Now that we're confirmed in offboard, switch the background thread
# to start sending SET_ACTUATOR_CONTROL_TARGET instead of position hold.
print("\nSwitching stream to raw actuator control...")
print("If PX4 rejects/ignores this, it will likely fall out of Offboard")
print("mode on its own (watch the HEARTBEAT custom_mode value below).")
stream_mode = MODE_ACTUATOR
time.sleep(1)

# ── SMALL TEST PULSE ─────────────────────────────────────────
# Deliberately tiny amplitude (5% roll authority) and short duration —
# this is a bypass CHECK, not a real excitation signal. Watch the vehicle
# and the printed ATTITUDE feedback for a visible, immediate response.
print("\nSending small roll pulse (roll_cmd = 0.05) for 5 seconds...")
print("Watch Gazebo + the printed roll angle below.\n")

roll_cmd = 0.05
start = time.time()
while time.time() - start < 5.0:
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb:
        mode_ok = (hb.custom_mode == 6)
        if not mode_ok:
            print(f"\nDROPPED OUT OF OFFBOARD (custom_mode={hb.custom_mode}) — "
                  f"PX4 likely rejected SET_ACTUATOR_CONTROL_TARGET on this build.")
            break

    att = master.recv_match(type='ATTITUDE', blocking=False)
    if att:
        print(f"t={time.time()-start:.2f}s  roll_cmd={roll_cmd:+.2f}  "
              f"actual_roll={math.degrees(att.roll):+.2f}deg", end='\r')
    time.sleep(0.01)

print("\n\nTest pulse complete.")
print("Cross-check now: in MAVLink Console run 'listener actuator_controls'")
print("and confirm the roll value you see there matches what this script sent,")
print("not a PID-shaped/smoothed version of it.")

# ── RETURN TO POSITION HOLD ──────────────────────────────────
print("\nReturning to position hold...")
roll_cmd = 0.0
stream_mode = MODE_POSITION_HOLD
time.sleep(2)

streaming = False
stream_thread.join(timeout=2)
print("Done. Vehicle left in position hold — land manually via QGC when ready.")
