#! /usr/bin/env python
""" Convert ASCII point cloud to ROS bag

see http://wiki.ros.org/rosbag/Commandline
see http://wiki.ros.org/rosbag/Code%20API

Use atlaas/tools/convert_ascii to convert legacy velodyne-genom format.

for f in velodyneShot.[0-9]*; do
    i=${f##*.} # get the %04i indice
    convert_ascii $f > cloud.$(printf %05i $((10#$i))).txt
done

python velodyne_bag.py 0 9999 minnie

"""
import sys
import time
import struct
from math import sin, cos

import rospy
import rosbag

from tf.msg import tfMessage
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField

begin   = int(sys.argv[1])
end     = int(sys.argv[2])
name    = str(sys.argv[3])

fields  = [PointField('x', 0, PointField.FLOAT32, 1),
           PointField('y', 4, PointField.FLOAT32, 1),
           PointField('z', 8, PointField.FLOAT32, 1),
           PointField('intensity', 12, PointField.FLOAT32, 1)]

def matrix(yaw, pitch, roll, x, y, z):
    # from pom-genom/libeuler/pomEuler.c:287 (pomWriteSensorPos)
    # euler.{yaw,pitch,roll,x,y,z}
    ca, sa = cos(yaw),   sin(yaw)
    cb, sb = cos(pitch), sin(pitch)
    cg, sg = cos(roll),  sin(roll)
    return [[ ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg, x],
            [ sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg, y],
            [ -sb, cb*sg, cb*cg, z],
            [ 0.0, 0.0, 0.0, 1.0]]

def tfm(matrix, parent, child, stamp):
    rotation = quaternion_from_matrix(matrix)
    translation = translation_from_matrix(matrix)
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp
    t.child_frame_id = child
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return tfMessage([t])

f0 = '/map'
f1 = '/%s/base'%name
f2 = '/%s/velodyne'%name
path1 = 'cloud.%05i.txt'
path2 = 'velodyneShot.pos.%04i'
base = time.time()

with rosbag.Bag('%s.bag'%name, 'w') as bag:
    for seq in range(begin, end+1):
        with open(path1%seq) as f:
            lines = f.readlines()

        points  = [map(float, line.split()) for line in lines]
        stamp   = rospy.Time.from_sec(base + 0.1 * seq) # FIXME
        header  = Header(seq=seq, stamp=stamp, frame_id=f2)
        pc2     = point_cloud2.create_cloud(header, fields, points)
        bag.write(f2, pc2, stamp)

        with open(path2%seq) as f:
            lines = f.readlines()

        parser = lambda label: matrix(*map(float, [line for line in lines \
            if line.startswith(label)][0].strip().split()[2:]))

        sensor_to_main = parser('sensorToMain = ')
        main_to_origin = parser('mainToOrigin = ')

        bag.write('/tf', tfm(sensor_to_main, f1, f2, stamp), stamp)
        bag.write('/tf', tfm(main_to_origin, f0, f1, stamp), stamp)
