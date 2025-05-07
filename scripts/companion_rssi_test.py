#!/usr/bin/env python3
import socket, time, math, collections, collections.abc
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Python 3.10+ hack
collections.MutableMapping = collections.abc.MutableMapping

# — CONFIGURATION —
RSSI_PORT        = 9999      # UDP port to listen on
CALIB_SAMPLES    = 10        # how many first‐contact readings to average
HOP_DISTANCE     = 5.0       # meters per gradient hop
YAW_STEPS        = 8         # 360°/8 = 45° per scan step
GUIDED_THRESHOLD = None      # set after calibration
LAND_THRESHOLD   = 0.0034    # absolute RSSI at which you decide "found" and land

# Earth radius (WGS‐84)
R_EARTH = 6378137.0

def dest_latlon(lat, lon, bearing_deg, dist_m):
    b = math.radians(bearing_deg)
    φ1 = math.radians(lat)
    λ1 = math.radians(lon)
    dR = dist_m / R_EARTH
    φ2 = math.asin(math.sin(φ1)*math.cos(dR) +
                   math.cos(φ1)*math.sin(dR)*math.cos(b))
    λ2 = λ1 + math.atan2(math.sin(b)*math.sin(dR)*math.cos(φ1),
                        math.cos(dR)-math.sin(φ1)*math.sin(φ2))
    return math.degrees(φ2), math.degrees(λ2)

def condition_yaw(vehicle, heading, yaw_rate=0, direction=1, relative=False):
    is_rel = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, yaw_rate, direction, is_rel,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def gradient_homing(vehicle, sock):
    curr_rssi = 0.0
    # pull one fresh reading to seed
    data, _ = sock.recvfrom(100)
    curr_rssi = float(data.decode())
    print(f"[homing] starting RSSI = {curr_rssi:.6f}")

    while True:
        # if we already exceed landing threshold, just land
        if curr_rssi >= LAND_THRESHOLD:
            print(f"[homing] RSSI {curr_rssi:.6f} ≥ {LAND_THRESHOLD:.6f}, landing…")
            vehicle.mode = VehicleMode("LAND")
            return

        best_bearing = None
        best_rssi = curr_rssi

        # 1) scan around
        for i in range(YAW_STEPS):
            step_angle = (360.0 / YAW_STEPS)
            # do *relative* yaw by step_angle each time
            condition_yaw(vehicle, step_angle, relative=True)
            time.sleep(2)

            # collect a few samples
            samples = []
            for _ in range(3):
                d, _ = sock.recvfrom(100)
                samples.append(float(d.decode()))
            avg = sum(samples) / len(samples)
            heading_total = (i + 1) * step_angle  # approximate cumulated rotation
            print(f"  scan {heading_total:.0f}° → {avg:.6f}")
            if avg > best_rssi:
                best_rssi = avg
                best_bearing = heading_total

        # 2) if we didn’t improve by at least 1e-5, assume convergence
        if best_bearing is None or (best_rssi - curr_rssi) < 1e-5:
            print("[homing] no more gain—landing…")
            vehicle.mode = VehicleMode("LAND")
            return

        # 3) hop
        lat0 = vehicle.location.global_frame.lat
        lon0 = vehicle.location.global_frame.lon
        alt0 = vehicle.location.global_relative_frame.alt
        lat1, lon1 = dest_latlon(lat0, lon0, best_bearing, HOP_DISTANCE)
        print(f"[homing] step {HOP_DISTANCE:.1f} m @ {best_bearing:.0f}° (+{best_rssi-curr_rssi:.6f})")
        vehicle.simple_goto(LocationGlobalRelative(lat1, lon1, alt0))

        # 4) wait to arrive
        while True:
            dlat = (vehicle.location.global_frame.lat - lat1)*1.11e5
            dlon = (vehicle.location.global_frame.lon - lon1)*1.11e5 * \
                   math.cos(math.radians(lat1))
            if math.hypot(dlat, dlon) < 1.0:
                break
            time.sleep(0.5)

        curr_rssi = best_rssi
        print(f"[homing] arrived, RSSI now {curr_rssi:.6f}")

def main():
    # 1) bind RSSI socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", RSSI_PORT))
    print("[companion] Listening for RSSI on UDP port", RSSI_PORT)

    # 2) connect to SITL
    print("[companion] Connecting to vehicle…")
    vehicle = connect("udp:127.0.0.1:14550", wait_ready=True)
    print("[companion] Vehicle mode:", vehicle.mode.name)

    # 3) wait for first‐contact beacon and calibrate
    readings = []
    print("[search] Waiting for first beacon…")
    while len(readings) < CALIB_SAMPLES:
        data, _ = sock.recvfrom(100)
        val = float(data.decode())
        if val > 0.0:
            readings.append(val)
    baseline = sum(readings) / len(readings)
    global GUIDED_THRESHOLD
    GUIDED_THRESHOLD = baseline  # you could *lower* this if you like
    print(f"[search] baseline RSSI₀={baseline:.6f} → GUIDED when ≥ {GUIDED_THRESHOLD:.6f}")

    # 4) wait until we exceed GUIDED_THRESHOLD
    print(f"[search] Chasing until RSSI ≥ {GUIDED_THRESHOLD:.6f}…")
    while True:
        data, _ = sock.recvfrom(100)
        r = float(data.decode())
        if r >= GUIDED_THRESHOLD:
            print(f"[search] first good RSSI={r:.6f} → switching to GUIDED")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)
            break

    # 5) now gradient‐ascent chase until LAND_THRESHOLD
    print(f"[homing] Beginning gradient‑ascent until RSSI ≥ {LAND_THRESHOLD:.6f}")
    gradient_homing(vehicle, sock)

if __name__ == "__main__":
    main()
