cmake_minimum_required(VERSION 2.8.3)
project(challenge_open)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  ed
  head_ref_ed_client
  robot_skills
  robot_smach_states
  rospy
  tue_recorder
  tue_serialization
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EmptyString.srv
  EmptyBool.srv
  StartCountdown.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(challenge_open
#   src/${PROJECT_NAME}/challenge_open.cpp
# )

## Declare a cpp executable
add_executable(send_image src/send_image.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(challenge_open_node challenge_open_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(send_image
   ${catkin_LIBRARIES}
 )

