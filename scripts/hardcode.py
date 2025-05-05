#!/usr/bin/env python3
import math

# --- Constants for GPS conversion ---
START_LAT = -35.363696
START_LON = 149.163653
# meters → degrees (accounting for latitude)
M_TO_LAT = 1/111200.0
M_TO_LON = 1/(111200.0 * math.cos(math.radians(START_LAT)))

# --- Sweep parameters (meters) ---
X_MIN, X_MAX = -145.0, 145.0
Y_MIN, Y_MAX = -100.0, 100.0
X_STEP        =   10.0
ALTITUDE      =   10.0  # flight altitude in meters

def generate_waypoints():
    """
    1) Take off at the home point (–145, –100)
    2) Fly north to (–145, +100)
    3) Zig‑zag eastward in 10 m columns until (+145, +100)
    """
    # start at the home corner in meters
    cur_x, cur_y = X_MIN, Y_MIN
    # start GPS at your given lat/lon
    cur_lat, cur_lon = START_LAT, START_LON

    wps = [(cur_lat, cur_lon)]  # WP0 = takeoff here

    # 1) straight north to (X_MIN, Y_MAX)
    dy = Y_MAX - cur_y
    cur_y = Y_MAX
    cur_lat += dy * M_TO_LAT
    wps.append((cur_lat, cur_lon))

    # 2) now zig‑zag east
    while cur_x < X_MAX - 1e-6:
        # a) step east
        step = min(X_STEP, X_MAX - cur_x)
        cur_x += step
        cur_lon += step * M_TO_LON
        wps.append((cur_lat, cur_lon))

        # if we’re at the east edge, done
        if abs(cur_x - X_MAX) < 1e-6:
            break

        # b) flip north/south
        target_y = Y_MIN if cur_y == Y_MAX else Y_MAX
        dy = target_y - cur_y
        cur_y = target_y
        cur_lat += dy * M_TO_LAT
        wps.append((cur_lat, cur_lon))

    return wps

def write_qgc_wpl(waypoints):
    """
    Emit QGC WPL 110 with:
     - WP0: TAKEOFF (22)
     - WP1…: NAV_WAYPOINT (16)
    """
    print("QGC WPL 110")
    for i, (lat, lon) in enumerate(waypoints):
        cmd = 22 if i == 0 else 16
        print(f"{i}\t0\t3\t{cmd}\t0\t0\t0\t0\t"
              f"{lat:.6f}\t{lon:.6f}\t{ALTITUDE:.1f}\t1")

if __name__ == "__main__":
    wps = generate_waypoints()
    write_qgc_wpl(wps)
