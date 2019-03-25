cmake_minimum_required(VERSION 2.8.3)
project(robot_skills)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    amigo_msgs
    cb_planner_msgs_srvs
    control_msgs
    dragonfly_speech_recognition
    ed_msgs
    ed_gui_server
    ed_navigation
    ed_perception
    geometry_msgs
    head_ref
    rgbd_msgs
    rospy
    smach
    std_srvs
    text_to_speech
    tf
    trajectory_msgs
    tue_manipulation_msgs
    tue_msgs
    visualization_msgs
)

catkin_python_setup()

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    ${catkin_INCLUDE_DIRS}
)
