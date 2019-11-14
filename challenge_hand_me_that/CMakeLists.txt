cmake_minimum_required(VERSION 2.8.3)
project(challenge_hand_me_that)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
