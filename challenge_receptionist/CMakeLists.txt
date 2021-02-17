cmake_minimum_required(VERSION 3.0.2)
project(challenge_receptionist)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_skills
    robot_smach_states
)

catkin_python_setup()

catkin_package()

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    ${catkin_INCLUDE_DIRS}
)

