from morse.builder import *

robot = ATRV()

kb = Keyboard()
kb.properties(Speed=2)
robot.append(kb)

pose = Pose()
robot.append(pose)

camera = DepthCamera()
camera.translate(z = 1)
camera.frequency(3)
robot.append(camera)

robot.add_default_interface('socket')

env = Environment('indoors-1/boxes')
