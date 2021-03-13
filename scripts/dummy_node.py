#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy

rospy.init_node ('dummy_node')
pub = rospy.Publisher ('/mavros/local_position/pose', PoseStamped, queue_size=10, latch=True)

rate = rospy.Rate (40)
pose = PoseStamped ()
while not rospy.is_shutdown ():
    pub.publish (pose)
    rate.sleep ()

