import sys
import time
import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener
import atlaas
from atlaas.helpers.ros import wait, cloud, transformation

robot_name = sys.argv[1]

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
test.set_atlaas_path(robot_name)
profile = []

d_noise = 0.0
poses = []

def callback(msg):
    global d_noise, poses
    start = time.time()
    T = transformation(tfl, "/map", msg.header)
    poses.append([time.time(), T[0:2,3][0], T[0:2,3][1]])
    d_noise += 0.02
    T[0:2,3] += d_noise
    test.merge(cloud(msg), T, None)
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
    with open('%s/poses.json'%robot_name, 'w') as f:
        f.write(str(poses))
    print("repro...")
    np = test.reprocess(int(poses[0][0]*1000), int(poses[-1][0]*1000), poses[-1][1], poses[-1][2])
    print("updated %i point clouds"%np)
    test.export8u('%s/atlaas_repro.jpg'%robot_name)
    test.region('%s/region_repro.png'%robot_name)
