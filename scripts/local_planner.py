#!/usr/bin/env python3

import rospy
import time
import threading
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from uav_trajectory_control.srv import InitWaypointSet, TrajectoryComplete

curr_pose = Pose ()
targectory_complete = False

def odom_cb (data):
    curr_pose = data.pose.pose

def in_range (target_wp, dist_thres):
    c_x = curr_pose.position.x
    c_y = curr_pose.position.y
    c_z = curr_pose.position.z

    t_x = target_wp.x
    t_y = target_wp.y
    t_z = target_wp.z

    dist = abs (c_x - t_x) + abs (c_y - t_y) + abs (c_z - t_z)
    if dist <= dist_thres:
        return True
    return False

def is_trajectory_complete (request):
    if trajectory_complete:
        return TrajectoryCompleteResponse (True)
    return TrajectoryCompleteResponse (False)

def achieved (target_wp):
    freq_thres = 5
    dist_thres = 0.2

    count = 20
    rate = rospy.Rate (count)
    hit = 0

    for i in range (count):
        if in_range (target_wp, dist_thres):
            hit += 1
        if hit >= freq_thres:
            return True
        rate.sleep ()

    if hit >= freq_thres:
        return True
    return False

def target_handler (waypoint_set):
    target_pub = rospy.Publisher ('mavros/setpoint_position/local', PoseStamped, queue_size=10, latch=True)

    for waypoint in waypoint_set:
        rate = rospy.Rate (10)
        target_pose = PoseStamped ()
        target_pose.pose.position.x = waypoint.x
        target_pose.pose.position.y = waypoint.y
        target_pose.pose.position.z = waypoint.z

        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0.707
        target_pose.pose.orientation.w = 0.707

        while not achieved (waypoint):
            target_pub.publish (target_pose)
            rate.sleep ()

    global trajectory_complete
    trajectory_complete = True

def init_waypoint_set_cb (request):
    if len (request.waypoints) == 0:
        print ('Waypoint list is empty')
        return InitWaypointSetResponse (False)
    th = threading.Thread (target=target_handler, args=(request.waypoint_set,))
    th.start ()
    return InitWaypointSetResponse (True)

rospy.init_node ('local_planner')
odom_sub = rospy.Subscriber ('mavros/global_position/local', Odometry, odom_cb)

init_waypoint_set_service = rospy.Service ('start_waypoint_list', \
                                        InitWaypointSet, init_waypoint_set_cb)
trajectory_complete_service = rospy.Service ('trajectory_complete', \
                                        TrajectoryComplete, is_trajectory_complete)

rospy.spin ()
