cmake_minimum_required(VERSION 2.8.3)
project(challenge_restaurant)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  robot_smach_states
  robot_skills
  sensor_msgs
  image_transport
  image_geometry
  pcl_ros
  depth_image_proc
  message_generation
)

add_service_files(
  FILES
  GetNormalScore.srv
)

catkin_python_setup()

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(get_normal_score_node src/get_normal_score_node.cpp)
target_link_libraries(get_normal_score_node ${catkin_LIBRARIES})
add_dependencies(get_normal_score_node ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
