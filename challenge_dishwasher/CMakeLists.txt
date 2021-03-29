cmake_minimum_required(VERSION 3.0.2)
project(challenge_dishwasher)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
  robot_smach_states
  rospy
)

catkin_python_setup()

catkin_package()
