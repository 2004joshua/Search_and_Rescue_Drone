#!/usr/bin/env python3
import socket
from pymavlink import mavutil

# 1) Open UDP listener for RSSI (and print any packets immediately)
RSSI_PORT = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", RSSI_PORT))
print(f"[companion] Listening for RSSI on UDP port {RSSI_PORT}")

# 2) Connect to MAVProxy (SITL) on default port
#    (replace with your actual connection string if different)
mav = mavutil.mavlink_connection("udpout:127.0.0.1:14550")

# Wait for the first heartbeat so we know we're talking to the vehicle
mav.wait_heartbeat()
print("MAVLink heartbeat received!")

# Your RSSI threshold
THRESHOLD = -70.0  # dBm, for example

# 3) Main loop: read RSSI and act
while True:
    # Block until we receive an RSSI packet
    data, addr = sock.recvfrom(1024)
    # Dump every raw packet so we know it's arriving
    print(f"[companion] Raw UDP ‹{data!r}› from {addr}")
    try:
        rssi = float(data.decode("utf-8"))
    except ValueError:
        print("[companion] malformed packet, ignoring")
        continue  # ignore malformed

    print(f"[companion] RSSI = {rssi:.1f} dBm")

    if rssi > THRESHOLD:
        # a) switch to GUIDED mode
        mav.set_mode_guided()
        print("[companion] Switched to GUIDED")

        # b) command the vehicle to loiter (hold) in place
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,      # confirmation
            0, 0, 0, 0,  # no extra params
            0, 0, 0     # current lat/lon/alt (use actual if you compute them)
        )
        print("[companion] Holding until further notice")
        break  # or remove this break to keep monitoring
