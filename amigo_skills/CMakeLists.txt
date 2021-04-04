cmake_minimum_required(VERSION 3.0.2)
project(amigo_skills)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(amigo_description REQUIRED)
  add_rostest(test/test_amigo.test)
endif()
