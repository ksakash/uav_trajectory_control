#!/usr/bin/env python3

'''
To redirect the odometry msg with different frame id
'''

import rospy
import numpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg

import tf.transformations as trans

odo_pub = rospy.Publisher('mavros/odometry/in/re', Odometry, queue_size=10, latch=True)

def pose_callback (msg):
    msg_ = Odometry ()
    msg_.pose.pose = msg.pose
    msg_.header.frame_id = "world"
    odo_pub.publish (msg_)

if __name__ == '__main__':
    rospy.init_node('odom_redirect')
    sub_odometry = rospy.Subscriber('mavros/local_position/pose',
                                    PoseStamped, pose_callback)
    rospy.spin ()
