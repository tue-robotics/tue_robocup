cmake_minimum_required(VERSION 2.8.3)
project(challenge_following_and_guiding)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_skills
    robot_smach_states
)

catkin_package(
)
