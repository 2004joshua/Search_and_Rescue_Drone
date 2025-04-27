#!/usr/bin/env python3
print("quad_controller.py launching…", flush=True)
from controller import Robot

robot = Robot()
ts = int(robot.getBasicTimeStep())

# use the Crazyflie’s actual rotor names
motor_names = ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]
motors = []
for name in motor_names:
    m = robot.getDevice(name)
    if m is None:
        raise RuntimeError(f"Device {name!r} not found on Crazyflie")
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    motors.append(m)

# spin-up test at safe speeds
for speed in [100.0, 200.0, 300.0]:
    for m in motors:
        m.setVelocity(speed)
    robot.step(ts * 50)

while robot.step(ts) != -1:
    pass
