cmake_minimum_required(VERSION 3.5)
project(challenge_take_out_the_garbage)

find_package(catkin REQUIRED COMPONENTS
  robocup_knowledge
  robot_skills
  robot_smach_states
  rospy
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
