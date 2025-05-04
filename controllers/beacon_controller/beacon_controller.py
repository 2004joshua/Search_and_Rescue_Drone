from controller import Robot, Emitter

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice("beacon_emitter")
emitter.setChannel(1)

while robot.step(timestep) != -1:
    emitter.send(b"\x00" * 8)  # ping every time-step
    #print("ping")  # uncomment to see the pings in the console
