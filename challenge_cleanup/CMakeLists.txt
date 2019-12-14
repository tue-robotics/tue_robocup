cmake_minimum_required(VERSION 2.8.3)
project(challenge_cleanup)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_skills
    robot_smach_states
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
