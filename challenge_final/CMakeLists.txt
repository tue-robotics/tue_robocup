cmake_minimum_required(VERSION 3.0.2)
project(challenge_final)

find_package(catkin REQUIRED COMPONENTS
  challenge_find_my_mates
  robot_skills
  robot_smach_states
  rospy
  telegram_ros
)

catkin_python_setup()

catkin_package()
