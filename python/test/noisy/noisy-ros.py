import sys
import time
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import TransformListener, transformations
import numpy as np
import atlaas

robot_name = sys.argv[1]

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
test.set_atlaas_path(robot_name)
profile = []
covariance = [0]*36
pose_stamped = None
poses = []

# Fix broken TransformListener.waitForTransform
# Exception: Lookup would require extrapolation into the future.
# waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3))
def wait(frame, header, duration=3):
    start_time = time.time()
    while not tfl.canTransform(frame, header.frame_id, header.stamp) \
            and (time.time() - start_time) < duration \
            and not rospy.is_shutdown():
        rospy.sleep(0.0001) # 100 us
    # print("Ive been waiting %f"%(time.time() - start_time))

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
    test.merge(cloud(msg), transformation("/map", msg.header), cov)
    poses.append([pose_stamped.header.stamp.to_time(),
                  pose_stamped.pose.position.x,
                  pose_stamped.pose.position.y])
    profile.append(time.time() - start)
    test.export8u('%s/atlaas.jpg'%robot_name)

def callback_cov(msg):
    global covariance
    covariance = msg.pose.covariance

def callback_pose(msg):
    global pose_stamped
    pose_stamped = msg

rospy.init_node("atlaas_%s"%robot_name)
tfl = TransformListener()
rospy.Subscriber("/%s/velodyne"%robot_name, PointCloud2, callback)
rospy.Subscriber("/%s/odom"%robot_name, PoseWithCovarianceStamped, callback_cov)
rospy.Subscriber("/%s/pose"%robot_name, PoseStamped, callback_pose)
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
    with open('%s/poses.json'%robot_name, 'w') as f:
        f.write(str(poses))
    print("repro...")
    np = test.reprocess(int(poses[0][0]*1000), int(poses[-1][0]*1000), poses[-1][1], poses[-1][2])
    print("updated %i point clouds"%np)
    test.export8u('%s/atlaas_repro.jpg'%robot_name)
