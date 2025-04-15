cmake_minimum_required(VERSION 3.5)
project(challenge_set_a_table)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
  robot_smach_states
  rospy
)

catkin_python_setup()

catkin_package()
