import sys
from math import pi
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped

robot_name = sys.argv[1] # 'mana' or 'momo'
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
def callback(msg):
    global goal, pub
    if not goal or distance_sq(goal, msg.pose.position) < 4:
        if trajectory:
            goal = Pose2D(*trajectory.pop(0))
            pub.publish(goal)
        else:
            rospy.signal_shutdown('done')

rospy.init_node("control_%s"%robot_name)
pub = rospy.Publisher("/%s/waypoint"%robot_name, Pose2D, queue_size=5, latch=True)
rospy.Subscriber("/%s/pose"%robot_name, PoseStamped, callback)
rospy.spin() # this will block untill you hit Ctrl+C
