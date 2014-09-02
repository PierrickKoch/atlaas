from morse.builder import *

robot = ATRV()

kb = Keyboard()
kb.properties(Speed = 3)
robot.append(kb)

pose = Pose()
robot.append(pose)
pose.add_stream('ros', method='morse.middleware.ros.pose.TFPublisher')

camera = Velodyne()
camera.translate(z = 1)
robot.append(camera)
camera.add_stream('ros')

env = Environment('outdoors')
