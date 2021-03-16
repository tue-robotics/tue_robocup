cmake_minimum_required(VERSION 3.0.2)
project(hero_skills)

find_package(catkin REQUIRED COMPONENTS
  hero_description
  robot_skills
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_hero.test)
endif()

