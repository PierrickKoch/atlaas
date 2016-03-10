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

def transformation(tflistener_or_tf2buffer, frame, header):
    if 'lookup_transform' in dir(tflistener_or_tf2buffer):
        # tf2 use lower_case_with_underscores (PEP8)
        # tf2 lookup_transform returns a geometry_msgs/TransformStamped Message
        tf = tflistener_or_tf2buffer.lookup_transform(frame,
            header.frame_id, header.stamp, rospy.Duration(3))
        t, r = tf.transform.translation, tf.transform.rotation
        translation = (t.x, t.y, t.z)
        quaternion = (r.x, r.y, r.z, r.w)
    else:
        wait(tfl, frame, header)
        # we could use return tfl.asMatrix(frame, header)
        # tf lookupTransform returns a geometry_msgs/Transform Message
        translation, quaternion = tflistener_or_tf2buffer.lookupTransform(frame,
            header.frame_id, header.stamp)

    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = translation[:3]
    return M

def cloud(msg):
    assert(msg.height == 1)
    assert(msg.point_step % 4 == 0) # make sure we wont truncate data
    # PCL stores XYZI on 32 bytes (XYZ on 16 bytes, for SSE alignment)
    # while atlaas does it on 16 (using Intensity as padding)
    # following is a workaround TODO proper point-cloud conversion
    #    see sensor_msgs.point_cloud2.read_points(msg, 'XYZI') ?
    if msg.point_step > 16:
        return numpy.ndarray(shape=(msg.width, msg.point_step / 4),
                             dtype='float32', buffer=msg.data)[:,:4]
    return numpy.ndarray(shape=(msg.width, msg.point_step / 4),
                         dtype='float32', buffer=msg.data)
