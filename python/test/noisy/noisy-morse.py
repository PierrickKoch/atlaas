from math import pi
from morse.builder import *

class AGV(ATRV):
    def __init__(self, base_name):
        ATRV.__init__(self)
        self.base_name = base_name
        self.add_velodyne()
    def add_velodyne(self):
        self.velodyne = Velodyne()
        self.velodyne.properties(cam_width=128, cam_height=128)
        self.velodyne.translate(z=1)
        self.append(self.velodyne)
        self.velodyne.add_stream('ros',
            parent_frame_id=self.base_name+'/base_link')
    def add_pose(self):
        self.pose = Pose()
        self.append(self.pose)
        self.pose.add_stream('ros',
            method='morse.middleware.ros.pose.TFPublisher',
            child_frame_id=self.base_name+'/base_link')
    def add_odom(self):
        self.odom = Odometry()
        self.odom.level('integrated')
        self.append(self.odom)
        self.odom.add_stream('ros', method =
            'morse.middleware.ros.pose.TFPublisher',
            child_frame_id=self.base_name+'/base_link')
        self.odom.add_stream('ros', method =
            'morse.middleware.ros.pose.PoseWithCovarianceStampedPublisher')
        self.odom.alter(classpath =
            'morse.modifiers.odometry_noise.OdometryNoiseModifier',
            factor = 1, factor_sigma = 0.5, gyro_drift = 0.00001,
            gyro_drift_sigma = 0.0001)
        self.odom.add_service('socket')
    def add_third_person_camera(self):
        # for 3rd person view
        self.third_person_camera = Camera('third_person_camera')
        self.third_person_camera.translate(x=-6, z=4)
        self.third_person_camera.rotation_euler = [1.3, 0, -pi/2]
        # camera Z-far to 400m (instead of 100)
        self.third_person_camera._bpy_object.data.clip_end = 400.0
        self.append(self.third_person_camera)
    def add_keyboard(self):
        # for demo
        self.keyboard = Keyboard()
        self.keyboard.properties(Speed=5.0, ControlType='Position')
        self.append(self.keyboard)

mana = AGV('mana')
mana.add_third_person_camera()
mana.add_keyboard()
mana.add_odom()
# ground truth
pose = Pose()
mana.append(pose)
pose.add_stream('ros', method='morse.middleware.ros.pose.PoseStampedPublisher')

ghost = ATRV()
ghost.make_ghost()
teleport = Teleport()
ghost.append(teleport)
teleport.add_stream('ros', method='morse.middleware.ros.read_pose.PoseReader')

env = Environment('outdoors')
env.set_camera_clip(clip_end=1000)
env.show_framerate(True)
env.set_camera_speed(15)
env.create()
