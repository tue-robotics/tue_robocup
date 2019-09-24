cmake_minimum_required(VERSION 2.8.3)
project(challenge_rips)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_skills
    robot_smach_states
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()


