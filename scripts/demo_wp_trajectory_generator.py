import roslib
import numpy as np
import matplotlib.pyplot as plt
import uav_trajectory_generator
import uav_waypoints
import time
from geometry_msgs.msg import Point
from mpl_toolkits.mplot3d import Axes3D

"""
Demo file to demonstrate the waypoint interpolation method with generation of
velocity and acceleration profile using a constant rate.
"""

def run_generator(waypoint_set, interp_method):
    # Initialize the trajectory generator
    gen = uav_trajectory_generator.WPTrajectoryGenerator(full_dof=False)
    gen.set_interpolation_method(interp_method)
    gen.init_waypoints(waypoint_set)

    dt = 0.05
    idx = 0
    pnts = list()
    avg_time = 0.0

    gen.set_start_time(0)
    gen.set_duration(100)

    for ti in np.arange(-2, gen.get_max_time(), dt):
        tic = time.time()
        pnts.append(gen.interpolate(ti))
        toc = time.time()
        avg_time += toc - tic
        idx += 1
    avg_time /= idx
    print ('Average processing time [s] =', avg_time)
    fig = plt.figure()
    # Trajectory and heading 3D plot
    ax = fig.add_subplot(111, projection='3d')

    # Plot the generated path
    ax.plot([p.x for p in pnts], [p.y for p in pnts], [p.z for p in pnts], 'b')

    # Plot original waypoints
    ax.plot(waypoint_set.x, waypoint_set.y, waypoint_set.z, 'r.')

    # Plot the raw path along the waypoints
    ax.plot(waypoint_set.x, waypoint_set.y, waypoint_set.z, 'g--')

    for i in range(1, len(pnts), 100):
        p0 = pnts[i - 1]
        p1 = p0.pos + np.dot(p0.rot_matrix, [2, 0, 0])
        ax.plot([p0.pos[0], p1[0]], [p0.pos[1], p1[1]], [p0.pos[2], p1[2]], 'c', linewidth=2)
    ax.grid(True)
    ax.set_title(interp_method)

    # Position and orientation plot
    '''
    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.pos[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Position - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.rot[i] * 180 / np.pi for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Rotation - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear velocity - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular velocity - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear accelerations - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular accelerations - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)
    '''

if __name__ == '__main__':
    wp_set = uav_waypoints.WaypointSet()
    speed = 0.1
    filename = '/home/ksakash/projects/control_ws/src/uav_trajectory_control/cfg/plan_0'
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    height = 20

    for line in lines:
        (x, y, z) = (int (line.split(' ')[0]), int (line.split(' ')[1]), height)
        wp_set.add_waypoint (uav_waypoints.Waypoint (x, y, z, speed))

    print (wp_set)
    run_generator(wp_set, 'cubic_interpolator')
    run_generator(wp_set, 'linear_interpolator')

    plt.show()

