cmake_minimum_required(VERSION 2.8.3)
project(challenge_follow_me)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  wire_interface
  actionlib
  pein_msgs
  pein_srvs
  std_srvs
  amigo_msgs
  sensor_msgs
  geometry_msgs
  perception_srvs
  tue_carrot_planner
  tue_pocketsphinx
  text_to_speech
  amigo_head_ref
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# to show include files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

# Follower as a library
add_library(tue_follower src/Follower.cpp ${HEADER_FILES})
target_link_libraries(tue_follower ${catkin_LIBRARIES})

add_executable(follow_me include/challenge_follow_me/Follower.h src/Follower.cpp src/follow_me.cpp ${HEADER_FILES})
target_link_libraries(follow_me tue_follower ${catkin_LIBRARIES})
