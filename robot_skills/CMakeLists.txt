cmake_minimum_required(VERSION 3.5)
project(robot_skills)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cb_base_navigation_msgs
  control_msgs
  cv_bridge
  ed_msgs
  ed_gui_server_msgs
  ed_navigation_msgs
  ed_people_recognition_msgs
  ed_perception_msgs
  ed_sensor_integration_msgs
  geometry_msgs
  head_ref_msgs
  hmi
  hmi_msgs
  people_recognition_msgs
  picovoice_msgs
  pykdl_ros
  rgbd_msgs
  rospy
  smach
  std_srvs
  text_to_speech
  tf_conversions
  tf2_geometry_msgs
  tf2_kdl
  tf2_pykdl_ros
  tf2_ros
  trajectory_msgs
  tue_manipulation_msgs
  tue_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/nosetest)
  add_rostest(test/rostest/test_compose_robot.test)
endif()
