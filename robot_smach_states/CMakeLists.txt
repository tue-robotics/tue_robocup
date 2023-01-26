cmake_minimum_required(VERSION 3.0.2)
project(robot_smach_states)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  robot_skills
  rospy
  rosapi
  message_generation
  std_msgs
  cb_base_navigation_msgs
)

# add robot-console
catkin_add_env_hooks(60.robot_smach_states SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES door_info.srv
)
generate_messages( DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros message_runtime std_msgs cb_base_navigation_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_the_door_service
  my_service_server.cpp
)

add_dependencies(open_the_door_service ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_link_libraries(open_the_door_service
  ${catkin_LIBRARIES}
)

install(TARGETS open_the_door_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
  add_rostest(test/rostest/test_grab.test)
  add_rostest(test/rostest/test_place.test)
endif()
