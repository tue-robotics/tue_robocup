cmake_minimum_required(VERSION 3.5)
project(sergio_skills)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(sergio_description REQUIRED)
  add_rostest(test/test_sergio.test)
endif()
