cmake_minimum_required(VERSION 2.8.3)
project(uav_trajectory_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED geometry_msgs message_generation)

add_message_files(
    FILES
    Trajectory.msg
    TrajectoryPoint.msg
    Waypoint.msg
    WaypointSet.msg
)

add_service_files(
    FILES
    AddWaypoint.srv
    ClearWaypoints.srv
    InitCircularTrajectory.srv
    InitHelicalTrajectory.srv
    InitWaypointsFromFile.srv
    GetWaypoints.srv
    GoTo.srv
    GoToPose.srv
    GoToIncremental.srv
    Hold.srv
    IsRunningTrajectory.srv
    InitWaypointSet.srv
    InitRectTrajectory.srv
    StartTrajectory.srv
    SwitchToAutomatic.srv
    SwitchToManual.srv
    SetPIDParams.srv
    GetPIDParams.srv
    SetSMControllerParams.srv
    GetSMControllerParams.srv
    SetMBSMControllerParams.srv
    GetMBSMControllerParams.srv
    ResetController.srv
    TrajectoryComplete.srv
    PoseReach.srv
    CancelRequest.srv
    BottomSearch.srv
    FrontSearch.srv
    StallVehicle.srv
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime
)

catkin_install_python(PROGRAMS scripts/cascaded_pid_dp_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  foreach(T
      test/test_all.launch
      )
     add_rostest(${T})
  endforeach()

  catkin_add_nosetests(test/test_waypoint.py)
  catkin_add_nosetests(test/test_trajectory_point.py)
  catkin_add_nosetests(test/test_waypoint_set.py)
endif()
