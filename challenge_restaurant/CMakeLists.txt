cmake_minimum_required(VERSION 3.0.2)
project(challenge_restaurant)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  depth_image_proc
  image_geometry
  image_transport
  message_generation
  pcl_ros
  robot_skills
  robot_smach_states
  roscpp
  rospy
  sensor_msgs
  tf2_ros
)

add_service_files(
  FILES
  GetNormalScore.srv
)

catkin_python_setup()

generate_messages()

catkin_package()

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_normal_score_node src/get_normal_score_node.cpp)
target_link_libraries(get_normal_score_node ${catkin_LIBRARIES})
add_dependencies(get_normal_score_node ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
