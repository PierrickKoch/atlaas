from morse.builder import *

robot = ATRV()

kb = Keyboard()
kb.properties(Speed = 3)
robot.append(kb)

odom = Odometry()
odom.level('integrated')
robot.append(odom)
odom.add_stream('ros', method='morse.middleware.ros.pose.TFPublisher')
odom.add_stream('ros', method='morse.middleware.ros.pose.PoseWithCovarianceStampedPublisher')
odom.alter(classpath='morse.modifiers.odometry_noise.OdometryNoiseModifier',
    factor = 1, factor_sigma = 0.5,
    gyro_drift = 0.00001, gyro_drift_sigma = 0.0001)
odom.add_service('socket')

camera = Velodyne()
camera.translate(z = 1)
robot.append(camera)
camera.add_stream('ros')

ghost = ATRV()
ghost.make_ghost()
teleport = Teleport()
ghost.append(teleport)
teleport.add_stream('ros', method='morse.middleware.ros.read_pose.PoseReader')

env = Environment('outdoors')
