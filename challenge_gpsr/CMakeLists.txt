cmake_minimum_required(VERSION 3.0.2)
project(challenge_gpsr)

find_package(catkin REQUIRED COMPONENTS
  conversation_engine
  hmi
  robot_skills
  robot_smach_states
  rospy
)

catkin_package()
