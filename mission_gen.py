#!/usr/bin/env python3
import argparse
import math
import os

"""
Generate a lawn-mower mission file for Webots ArduCopter AUTO mode,
inspecting an inset rectangle of a larger floor.
"""

def world_to_gps(x_m, z_m, lat_ref, lon_ref):
    """
    Convert world coords (east, north) to GPS degrees.
    x_m: meters east of world origin
    z_m: meters north of world origin
    lat_ref, lon_ref: GPS at world origin (0,0)
    """
    deg_per_m_lat = 1.0 / 111320.0
    deg_per_m_lon = 1.0 / (111320.0 * math.cos(math.radians(lat_ref)))
    lat = lat_ref + z_m * deg_per_m_lat
    lon = lon_ref + x_m * deg_per_m_lon
    return lat, lon


def generate_waypoints(width, length, spacing, start_x, start_z, lat_ref, lon_ref, alt):
    """
    Generate back-and-forth lawn-mower waypoints over a rectangle:
      width: east-west span (m)
      length: north-south span (m)
      spacing: distance between passes (m)
      start_x, start_z: SW corner in world meters
      lat_ref, lon_ref: GPS at world origin
      alt: flight altitude (m)
    """
    n_strips = math.ceil(width / spacing)
    dx = width / n_strips
    wps = []
    for i in range(n_strips + 1):
        x = start_x + i * dx
        # define north endpoints
        north_list = [start_z, start_z + length]
        # reverse on odd strips for back-and-forth
        if i % 2 == 1:
            north_list.reverse()
        for z in north_list:
            lat, lon = world_to_gps(x, z, lat_ref, lon_ref)
            wps.append((lat, lon, alt))
    return wps


def write_mission(fname, wps):
    os.makedirs(os.path.dirname(fname), exist_ok=True)
    with open(fname, 'w') as f:
        f.write("QGC WPL 110\n")
        # write takeoff command
        lat0, lon0, alt0 = wps[0]
        f.write(f"0\t0\t3\t22\t0\t0\t0\t0\t{lat0:.6f}\t{lon0:.6f}\t{alt0:.1f}\t1\n")
        # NAV_WAYPOINTs
        for idx, (lat, lon, alt) in enumerate(wps, start=1):
            f.write(f"{idx}\t0\t3\t16\t0\t0\t0\t0\t{lat:.6f}\t{lon:.6f}\t{alt:.1f}\t1\n")
    print(f"Mission written to {fname} with {len(wps)+1} commands.")


def main():
    p = argparse.ArgumentParser(description="Generate inset-lawn-mower mission.")
    p.add_argument('--floor-width',  type=float, required=True, help='Full floor width (m)')
    p.add_argument('--floor-length', type=float, required=True, help='Full floor length (m)')
    p.add_argument('--inset',        type=float, default=1.0,     help='Margin from edges (m)')
    p.add_argument('--spacing',      type=float, required=True,  help='Pass spacing (m)')
    p.add_argument('--lat-ref',      type=float, required=True,  help='GPS lat at world origin')
    p.add_argument('--lon-ref',      type=float, required=True,  help='GPS lon at world origin')
    p.add_argument('--alt',          type=float, default=5.0,    help='Flight altitude (m)')
    p.add_argument('--output',       type=str,   default='missions/waypoints.txt', help='Output mission file')
    args = p.parse_args()

    # compute inset rectangle
    start_x = -args.floor_width/2 + args.inset
    start_z = -args.floor_length/2 + args.inset
    width   =  args.floor_width  - 2*args.inset
    length  =  args.floor_length - 2*args.inset

    wps = generate_waypoints(width, length, args.spacing,
                             start_x, start_z,
                             args.lat_ref, args.lon_ref,
                             args.alt)
    write_mission(args.output, wps)

if __name__ == '__main__':
    main()