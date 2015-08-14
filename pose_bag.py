#! /usr/bin/env python
""" Convert ASCII point cloud to ROS bag

see http://wiki.ros.org/rosbag/Commandline
see http://wiki.ros.org/rosbag/Code%20API
"""
import sys
import time
from math import sin, cos

import rospy
import rosbag
import geometry_msgs.msg
import tf
import tf.msg
from tf.transformations import quaternion_from_matrix, translation_from_matrix

begin   = int(sys.argv[1])
end     = int(sys.argv[2])
name    = str(sys.argv[3])

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
    t = geometry_msgs.msg.TransformStamped()
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
    return tf.msg.tfMessage([t])

f0 = '/%s/map'%name
f1 = '/%s/base'%name
f2 = '/%s/velodyne'%name
path = 'velodyneShot.pos.%04i'

with rosbag.Bag('%s.bag'%name, 'w') as bag:
    for seq in range(begin, end+1):
        with open(path%seq) as f:
            lines = f.readlines()

        parser = lambda label: matrix(*map(float, [line for line in lines \
            if line.startswith(label)][0].strip().split()[2:]))

        sensor_to_main = parser('sensorToMain = ')
        main_to_origin = parser('mainToOrigin = ')

        stamp = rospy.Time.from_sec(time.time() + seq) # FIXME
        bag.write('/tf', tfm(sensor_to_main, f1, f2, stamp), stamp)
        bag.write('/tf', tfm(main_to_origin, f0, f1, stamp), stamp)
