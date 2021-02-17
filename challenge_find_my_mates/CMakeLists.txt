cmake_minimum_required(VERSION 3.0.2)
project(challenge_find_my_mates)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
