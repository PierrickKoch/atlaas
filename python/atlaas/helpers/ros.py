"""
Usage example:

    def callback(msg):
        test.merge(cloud(msg), transformation(tfl, "/map", msg.header))
        test.export8u('atlaas.jpg')
"""
import rospy

# Fix broken TransformListener.waitForTransform
# Exception: Lookup would require extrapolation into the future.
# waitForTransform(frame, header.frame_id, header.stamp, rospy.Duration(3))
def wait(tfl, frame, header, duration=3):
    start_time = time.time()
    while not tfl.canTransform(frame, header.frame_id, header.stamp) \
          and (time.time() - start_time) < duration \
          and not rospy.is_shutdown():
        rospy.sleep(0.0001) # 100 us
    # print("Ive been waiting %f"%(time.time() - start_time))

def transformation(tfl, frame, header):
    wait(tfl, frame, header)
    position, quaternion = tfl.lookupTransform(frame, header.frame_id,
                                               header.stamp)
    M = transformations.quaternion_matrix(quaternion)
    M[:3, 3] = position[:3]
    return M

def cloud(msg):
    assert(msg.height == 1)
    assert(msg.point_step % 4 == 0) # make sure we wont truncate data
    return np.ndarray(shape=(msg.width, msg.point_step / 4),
                      dtype=np.float32, buffer=msg.data)
