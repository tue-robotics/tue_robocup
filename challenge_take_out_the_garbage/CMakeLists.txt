cmake_minimum_required(VERSION 2.8.3)
project(challenge_take_out_the_garbage)

find_package(catkin REQUIRED COMPONENTS
  robot_skills
  robot_smach_states
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
