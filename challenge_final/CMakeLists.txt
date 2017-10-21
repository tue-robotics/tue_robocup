cmake_minimum_required(VERSION 2.8.3)
project(challenge_final)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  ed
  head_ref_ed_client
  robot_skills
  robot_smach_states
  rospy
  tue_serialization
  message_generation
)

catkin_python_setup()

catkin_package()