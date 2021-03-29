cmake_minimum_required(VERSION 3.0.2)
project(robot_smach_states)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
  rospy
)

# add robot-console
catkin_add_env_hooks(60.robot_smach_states SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
