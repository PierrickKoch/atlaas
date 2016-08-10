'''
morse run -g 800x600 test-atlaas.py
rostopic pub -1 /momo/waypoint geometry_msgs/Pose2D "{x: 1,y: 1,theta: 1}"
rostopic pub -1 /momo/waypoint geometry_msgs/Pose2D "{x: 242,y: -12,theta: 1}"

'''
from math import pi
from morse.builder import *

class AGV(ATRV):
    def __init__(self, base_name):
        ATRV.__init__(self)
        self.velodyne = Velodyne()
        self.velodyne.properties(cam_width=128, cam_height=128, cam_far=70)
        self.velodyne.translate(z=1)
        self.append(self.velodyne)
        #self.set_mass(20)
        self.pose = Pose()
        self.append(self.pose)
        self.velodyne.add_stream('ros', parent_frame_id=base_name+'/base_link')
        self.pose.add_stream('ros',
            method='morse.middleware.ros.pose.TFPublisher',
            child_frame_id=base_name+'/base_link')
        self.pose.add_stream('ros',
            method='morse.middleware.ros.pose.PoseStampedPublisher',
            child_frame_id=base_name+'/base_link')
        self.waypoint = Waypoint()
        self.waypoint.properties(Speed=5.0)
        self.waypoint.add_stream('ros',
            method='morse.middleware.ros.waypoint2D.Pose2DReader')
        self.append(self.waypoint)
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
# mana.add_keyboard()
mana.translate(x=240, y=-10)
mana.rotate(z=2.2)

momo = AGV('momo')
momo.add_third_person_camera()
momo.translate(x=242, y=-12)
momo.rotate(z=-2.4)

# Scene
env = Environment("caylus50.blend")
env.set_camera_clip(clip_end=1000)
env.set_camera_rotation([1, 0, 1.2])
env.set_camera_location([280, -40, 60])
env.show_framerate(True)
env.set_camera_speed(15)
env.create()
