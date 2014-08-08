from morse.builder import *

robot = ATRV()

kb = Keyboard()
kb.properties(Speed=5)
robot.append(kb)

camera = DepthCamera()
camera.translate(z = 1)
camera.frequency(3)
robot.append(camera)
camera.add_stream('socket')
robot.add_default_interface('socket')

env = Environment('indoors-1/boxes')
