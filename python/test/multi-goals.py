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
    [242, -12, 0],
]}
traj2 = goals['mana'][:-1]
traj2.reverse()
goals['momo'] = [[x,y,z-pi] for x,y,z in traj2] + [[240, -10, 0]]

trajectory = goals[robot_name]

def distance_sq(p1, p2):
    x = p1.x - p2.x
    y = p1.y - p2.y
    return x*x + y*y

goal = None
def callback(msg):
    global goal, pub
    if trajectory and (not goal or distance_sq(goal, msg.pose.position) < 4):
        goal = Pose2D(*trajectory.pop(0))
        pub.publish(goal)

rospy.init_node("control_%s"%robot_name)
pub = rospy.Publisher("/%s/waypoint"%robot_name, Pose2D, queue_size=5, latch=True)
rospy.Subscriber("/%s/pose"%robot_name, PoseStamped, callback)
rospy.spin() # this will block untill you hit Ctrl+C
