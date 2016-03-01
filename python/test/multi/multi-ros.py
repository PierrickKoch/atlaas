import sys
import time
from math import pi
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose2D, PoseStamped
from tf import TransformListener
import atlaas
from atlaas.helpers.ros import wait, cloud, transformation

robot_name = sys.argv[1]

goals = {'mana': [
    [175, 65,  -3],
    [162, 60,  -2.5],
    [120, 25,  -3],
    [80,  14,  -1],
    [84,  0,   -1],
    [115, -40, -1],
    [125, -50, -0.8],
    [140, -72, -1],
    [160, -105,0.8],
    [190, -70, 0.8],
    [236, -16, 0.5],
]}
goals['momo'] = [[x,y,theta-pi] for x,y,theta in reversed(goals['mana'][:-1])] + [[237, -5, -1]]

trajectory = goals[robot_name]

def distance_sq(p1, p2):
    x = p1.x - p2.x
    y = p1.y - p2.y
    return x*x + y*y

goal = None
def next_goal(msg):
    global goal, pub
    if not goal or distance_sq(goal, msg.pose.position) < 4:
        if trajectory:
            goal = Pose2D(*trajectory.pop(0))
            pub.publish(goal)
        else:
            rospy.signal_shutdown('done')

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
pub = rospy.Publisher("/%s/waypoint"%robot_name, Pose2D, queue_size=5, latch=True)
rospy.Subscriber("/%s/pose"%robot_name, PoseStamped, next_goal)
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
