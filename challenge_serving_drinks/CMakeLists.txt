cmake_minimum_required(VERSION 2.8.3)
project(challenge_serving_drinks)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
  robot_smach_states
)

catkin_python_setup()

catkin_package()
