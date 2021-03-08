#!/usr/bin/env python3
import rospy
import numpy as np
from uav_control_interfaces import DPControllerBase
from uav_trajectory_control.srv import *
import math
import tf.transformations as trans

import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose, PoseStamped


class ROV_CascadedController(DPControllerBase):
    _LABEL = 'Cascaded PID dynamic position controller'

    def __init__(self):
        DPControllerBase.__init__(self, is_model_based=False, planner_full_dof=False)
        self._logger.info('Initializing: ' + self._LABEL)

        self._force_torque = np.zeros(6)
        self._logger.info(self._LABEL + ' ready')

        self.cmd_pose_pub = rospy.Publisher('mavros/setpoint_position/local',
                                            PoseStamped, queue_size=10, latch=True)

        self._logger.info('Cascaded PID controller ready!')
        self._last_t = None
        self._is_init = True

    def _reset_controller(self):
        super(ROV_CascadedController, self).reset_controller()
        self._force_torque = np.zeros(6)

    def update_controller(self):
        if not self._is_init:
            return False

        t = rospy.get_time()
        if self._last_t is None:
            self._last_t = t
            return False

        dt = t - self._last_t
        if dt <= 0:
            self._last_t = t
            return False

        cmd_pose = Pose()
        cmd_pose.position.x = self._reference['pos'][0]
        cmd_pose.position.y = self._reference['pos'][1]
        cmd_pose.position.z = self._reference['pos'][2]
        cmd_pose.orientation.x = 0 # self._reference['rot'][0]
        cmd_pose.orientation.y = 0 # self._reference['rot'][1]
        cmd_pose.orientation.z = 0 # self._reference['rot'][2]
        cmd_pose.orientation.w = 1 # self._reference['rot'][3]

        msg = PoseStamped ()
        msg.pose = cmd_pose

        self.cmd_pose_pub.publish (msg)
        return True

if __name__ == '__main__':
    print('Starting Cascaded PID Controller')
    rospy.init_node('rov_cascaded_pid_controller')

    try:
        node = ROV_CascadedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
