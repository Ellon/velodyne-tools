#! /usr/bin/env python
""" Convert ASCII point cloud to ROS bag

see http://wiki.ros.org/rosbag/Commandline
see http://wiki.ros.org/rosbag/Code%20API
"""
import sys
import time

import rospy
import rosbag
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField

begin   = int(sys.argv[1])
end     = int(sys.argv[2])
frame   = str(sys.argv[3])
name    = str(sys.argv[4])

fields  = [PointField('x', 0, PointField.FLOAT32, 1),
           PointField('y', 4, PointField.FLOAT32, 1),
           PointField('z', 8, PointField.FLOAT32, 1),
           PointField('intensity', 12, PointField.FLOAT32, 1)]
path    = "cloud.%05i.txt"
bag     = rosbag.Bag(name, 'w')

try:
    for seq in range(begin, end+1):
        with open(path%seq) as f:
            lines = f.readlines()

        points  = [map(float, line.split()) for line in lines]
        stamp   = rospy.Time.from_sec(time.time() + seq) # FIXME
        header  = Header(seq=seq, stamp=stamp, frame_id=frame)
        pc2     = point_cloud2.create_cloud(header, fields, points)
        bag.write(frame, pc2, stamp)

finally:
    bag.close()
