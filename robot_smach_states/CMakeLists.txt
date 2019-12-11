cmake_minimum_required(VERSION 2.8.3)
project(robot_smach_states)

find_package(catkin REQUIRED)

# add robot-console
catkin_add_env_hooks(60.robot_smach_states SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
