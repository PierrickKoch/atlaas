import pymorse

with pymorse.Morse() as sim:
    sim.robot.odom.reset_noise()
