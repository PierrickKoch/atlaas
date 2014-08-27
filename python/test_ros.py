import time
import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener, transformations
import numpy as np
import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
profile = []

def transformation(frame, header):
    tfl.waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3.0) )
    position, quaternion = tfl.lookupTransform(frame, header.frame_id, header.stamp)
    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def cloud(data):
    buff = np.frombuffer(data, dtype=np.float32)
    return buff.reshape(buff.size/3, 3)

def callback(msg):
    start = time.time()
    test.merge(cloud(msg.data), transformation("/map", msg.header))
    profile.append(time.time() - start)

rospy.init_node("atlaas_ros")
tfl = TransformListener()
rospy.Subscriber("/robot/camera", PointCloud2, callback)
try:
    rospy.spin() # this will block untill you hit Ctrl+C
finally:
    test.save_currents()
    print("merge min/max/avg = %.3f / %.3f / %.3f ms" % \
        ( 1000*min(profile), 1000*max(profile),
          1000*sum(profile) / len(profile) ) )
