import sys
import time
import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener, transformations
import numpy as np
import atlaas

robot_name = sys.argv[1]

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
test.set_atlaas_path(robot_name)
profile = []

def transformation(frame, header):
    tfl.waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3.0) )
    position, quaternion = tfl.lookupTransform(frame, header.frame_id, header.stamp)
    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def cloud(msg):
    assert(msg.height == 1)
    return np.ndarray(shape=(msg.width, 3), dtype=np.float32, buffer=msg.data)

def callback(msg):
    start = time.time()
    test.merge(cloud(msg), transformation("/map", msg.header), False)
    profile.append(time.time() - start)
    test.export8u('%s/atlaas.jpg'%robot_name)

rospy.init_node("atlaas_%s"%robot_name)
tfl = TransformListener()
rospy.Subscriber("/%s/velodyne"%robot_name, PointCloud2, callback)
try:
    rospy.spin() # this will block untill you hit Ctrl+C
finally:
    start = time.time()
    test.region('%s/region.png'%robot_name)
    print("region = %.3f ms" % (1000 * (time.time() - start)))
    print("merge min/max/avg = %.3f / %.3f / %.3f ms" % \
        ( 1000*min(profile), 1000*max(profile),
          1000*sum(profile) / len(profile) ) )
    with open('%s/profile.json'%robot_name, 'w') as f:
        f.write(str(profile))
