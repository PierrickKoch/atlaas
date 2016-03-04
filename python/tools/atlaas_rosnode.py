#! /usr/bin/env python
"""
Atlaas ROS node
"""
import os
import time
import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener
import atlaas
from atlaas.helpers.ros import wait, cloud, transformation

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
profile = []

def callback(msg):
    start = time.time()
    test.merge(cloud(msg), transformation(tfl, "/map", msg.header), None, True)
    profile.append(time.time() - start)
    test.export8u('%s/atlaas.jpg'%atlaas_path)

rospy.init_node("atlaas")
tfl = TransformListener()

atlaas_path = rospy.get_param("atlaas_path", os.environ.get("ATLAAS_PATH", "."))
test.set_atlaas_path(atlaas_path)

rospy.Subscriber("velodyne", PointCloud2, callback)

try:
    rospy.spin() # this will block untill you hit Ctrl+C
finally:
    start = time.time()
    test.region('%s/region.png'%atlaas_path)
    print("region = %.3f ms" % (1000 * (time.time() - start)))
    print("merge min/max/avg = %.3f / %.3f / %.3f ms" % \
        ( 1000*min(profile), 1000*max(profile),
          1000*sum(profile) / len(profile) ) )
    with open('%s/profile.json'%atlaas_path, 'w') as f:
        f.write(str(profile))
