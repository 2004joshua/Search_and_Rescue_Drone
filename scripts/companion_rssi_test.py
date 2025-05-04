#!/usr/bin/env python3
import socket
from pymavlink import mavutil

# 1) Open UDP listener for RSSI (and print any packets immediately)
RSSI_PORT = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", RSSI_PORT))
print(f"[companion] Listening for RSSI on UDP port {RSSI_PORT}")

# 2) MAVLink connection to SITL/MAVProxy
#    Use 'udpin' to listen for incoming from SITL
mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
print("[companion] Waiting for heartbeat from vehicle…")
mav.wait_heartbeat()
print(f"[companion] Heartbeat from system {mav.target_system}, component {mav.target_component}")

# 2) Main loop: read RSSI and print it
while True:
    data, addr = sock.recvfrom(1024)
    try:
        rssi = float(data.decode("utf-8"))
        print(f"[companion] RSSI = {rssi:.9f}  (from {addr})")
    except Exception:
        print(f"[companion] received malformed packet: {data!r}")
