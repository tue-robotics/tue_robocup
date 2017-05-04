cmake_minimum_required(VERSION 2.8.3)
project(challenge_restaurant)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_smach_states
    robot_skills)

catkin_python_setup()

catkin_package()
