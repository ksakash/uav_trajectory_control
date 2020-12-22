#!/usr/bin/env python

'''
To redirect the odometry msg with different frame id
'''

import rospy
import numpy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

import tf.transformations as trans

odo_pub = rospy.Publisher('mavros/odometry/in/re', Odometry, queue_size=10, latch=True)

def odometry_callback (msg):
    msg_ = msg
    msg_.header.frame_id = "world"
    odo_pub.publish (msg_)

if __name__ == '__main__':
    rospy.init_node('odom_redirect')
    sub_odometry = rospy.Subscriber('mavros/odometry/in',
                                    numpy_msg(Odometry), odometry_callback)
    rospy.spin ()
