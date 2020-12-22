import rospy
import numpy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String, Float32, Time

from uuv_control_msgs.srv import InitWaypointsFromFile
from uuv_control_msgs.srv import InitWaypointSet
from uuv_control_msgs.msg import Waypoint

import tf.transformations as trans

def read_file (filename):
    f = open (filename, 'r')
    lines = f.readlines ()
    waypoints = []
    for line in lines:
        x = line.split (' ')
        p = Point ()
        p.x = float (x[0])
        p.y = float (x[1])
        p.z = float (x[2])
        w = Waypoint (point=p, max_forward_speed=0.5)
        waypoints.append (w)
    return waypoints

if __name__ == '__main__':
    rospy.init_node('temp')
    try:
        init_from_file = \
            rospy.ServiceProxy ('/anahita/init_waypoints_from_file', InitWaypointsFromFile)
        start_from_list = \
            rospy.ServiceProxy ('/anahita/start_waypoint_list', InitWaypointSet)
    except:
        print ("unable to call the service")

    filename = "/home/ksakash/misc/catkin_ws/src/beginner_tutorials/cfg/waypoints"
    waypoints = read_file (filename)
    t = Time ()
    t.data = rospy.Time.now()
    print (t.data)
    ret = start_from_list (start_time=t, start_now=True, waypoints=waypoints,\
                     max_forward_speed=0.2, heading_offset=0.5,
                     interpolator=String ("cubic"))
    print ("what is happening")
    if ret:
        print ("Successful!")
    else:
        print ("Failed!!")
