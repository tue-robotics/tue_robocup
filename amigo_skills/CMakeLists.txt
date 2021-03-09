cmake_minimum_required(VERSION 3.0.2)
project(amigo_skills)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
)

catkin_python_setup()

catkin_package()
