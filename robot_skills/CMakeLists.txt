cmake_minimum_required(VERSION 2.8.3)
project(robot_skills)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    amigo_msgs
    cb_planner_msgs_srvs
    control_msgs
    cv_bridge
    ed_msgs
    ed_gui_server_msgs
    ed_navigation_msgs
    ed_people_recognition_msgs
    ed_perception_msgs
    ed_sensor_integration_msgs
    geometry_msgs
    head_ref_msgs
    hmi
    hmi_msgs
    people_recognition_msgs
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

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
