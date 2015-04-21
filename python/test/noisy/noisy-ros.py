import time
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import TransformListener, transformations
import numpy as np
import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
profile = []
covariance = [0]*36

# Fix broken TransformListener.waitForTransform
# Exception: Lookup would require extrapolation into the future.
# waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3))
def wait(frame, header, duration=3):
    start_time = time.time()
    while not tfl.canTransform(frame, header.frame_id, header.stamp) \
            and (time.time() - start_time) < duration \
            and not rospy.is_shutdown():
        rospy.sleep(0.0001) # 100 us
    print("Ive been waiting %f"%(time.time() - start_time))

def transformation(frame, header):
    wait(frame, header)
    position, quaternion = tfl.lookupTransform(frame, header.frame_id, header.stamp)
    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def cloud(msg):
    assert(msg.height == 1)
    return np.ndarray(shape=(msg.width, 3), dtype=np.float32, buffer=msg.data)

def callback(msg):
    start = time.time()
    cov = np.array(covariance, dtype=np.float64).reshape(6, 6)
    test.merge(cloud(msg), transformation("/map", msg.header), cov, False)
    profile.append(time.time() - start)
    test.export8u('atlaas.jpg')

def callback_pose(msg):
    covariance = msg.pose.covariance

rospy.init_node("atlaas_ros")
tfl = TransformListener()
rospy.Subscriber("/robot/camera", PointCloud2, callback)
rospy.Subscriber("/robot/odom", PoseWithCovarianceStamped, callback_pose)
try:
    rospy.spin() # this will block untill you hit Ctrl+C
finally:
    test.save_currents()
    print("merge min/max/avg = %.3f / %.3f / %.3f ms" % \
        ( 1000*min(profile), 1000*max(profile),
          1000*sum(profile) / len(profile) ) )
