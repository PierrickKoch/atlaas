import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener, transformations
import numpy as np
import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

def transformation(frame, header):
    tfl.waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3.0) )
    position, quaternion = tfl.lookupTransform(frame, header.frame_id, header.stamp)
    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def callback(msg):
    cloud = np.fromstring(msg.data, dtype=np.float32)
    cloud = cloud.reshape(cloud.size/3, 3)
    test.merge(cloud, transformation( "/map", msg.header ))

rospy.init_node("atlaas_ros")
tfl = TransformListener()
rospy.Subscriber("/robot/camera", PointCloud2, callback)
rospy.spin() # this will block untill you hit Ctrl+C
