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

def callback(msg):
    start = time.time()
    test.merge(cloud(msg), transformation(tfl, "/map", msg.header), None, False)
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
