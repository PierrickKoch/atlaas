"""
Usage example:

    import rospy
    import atlaas
    from atlaas.helpers.ros import wait, cloud, transformation
    from sensor_msgs.msg import PointCloud2
    from tf import TransformListener
    test = atlaas.Atlaas()
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
    rospy.init_node("atlaas")
    tfl = TransformListener()
    def callback(msg):
        test.merge(cloud(msg), transformation(tfl, "/map", msg.header))
        test.export8u('atlaas.jpg')

    rospy.Subscriber("/velodyne", PointCloud2, callback)
"""
import rospy
import numpy
from tf import transformations
from tf2_ros import Buffer, LookupException, ConnectivityException, ExtrapolationException

# Fix broken TransformListener.waitForTransform
# Exception: Lookup would require extrapolation into the future.
# waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3))
def wait(tfl, frame, header, duration=3):
    start_time = rospy.Time.now().to_time()
    while not tfl.canTransform(frame, header.frame_id, header.stamp) \
          and (rospy.Time.now().to_time() - start_time) < duration \
          and not rospy.is_shutdown():
        rospy.sleep(0.0001) # 100 us
    # print("Ive been waiting %f"%(time.time() - start_time))

def wait_tf2(tf2buffer, frame, header, duration=3):
    trans = tf2buffer.lookup_transform(frame, header.frame_id, header.stamp, rospy.Duration(duration))
    t, r = trans.transform.translation, trans.transform.rotation
    position = (t.x, t.y, t.z)
    quaternion = (r.x, r.y, r.z, r.w)
    return position, quaternion

def transformation(tfl_or_tf2buffer, frame, header):
    if isinstance(tfl_or_tf2buffer, Buffer):
        position, quaternion = wait_tf2(tfl_or_tf2buffer, frame, header)
    else:
        wait(tfl, frame, header)
        position, quaternion = tfl_or_tf2buffer.lookupTransform(frame, header.frame_id, header.stamp)

    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def cloud(msg):
    assert(msg.height == 1)
    assert(msg.point_step % 4 == 0) # make sure we wont truncate data
    return numpy.ndarray(shape=(msg.width, msg.point_step / 4),
                         dtype='float32', buffer=msg.data)
