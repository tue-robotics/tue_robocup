cmake_minimum_required(VERSION 2.8.3)
project(robot_skills)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    smach
    smach_ros
    geometry_msgs
    std_srvs
    actionlib
    tf
    tf_server
    amigo_msgs
    tue_msgs
    visualization_msgs
    head_ref
    control_msgs
    trajectory_msgs
    cb_planner_msgs_srvs
    tue_manipulation_msgs
    text_to_speech
    ed_msgs
    ed_navigation
    dragonfly_speech_recognition
)

catkin_python_setup()

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

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
    include
    ${catkin_INCLUDE_DIRS}
)

# add_library(library_name
#     src/lib_source_file1.cpp
#     ...
# )
# target_link_libraries(library_name ${catkin_LIBRARIES})

# add_executable(exec_name
#     src/source_file1.cpp
#     ...
# )
# target_link_libraries(exec_name ${catkin_LIBRARIES})


