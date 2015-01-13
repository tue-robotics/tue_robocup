cmake_minimum_required(VERSION 2.8.3)
project(robot_smach_states)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_skills
    smach
    smach_ros
    geometry_msgs
    ed
)

catkin_python_setup()

catkin_package()
