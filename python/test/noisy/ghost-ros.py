import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

def callback(msg):
    pub.publish(msg.pose.pose)

rospy.init_node("ghost_ros")
pub = rospy.Publisher("/ghost/teleport", Pose)
rospy.Subscriber("/mana/odom", PoseWithCovarianceStamped, callback)
rospy.spin() # this will block untill you hit Ctrl+C
