# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot_from_argv

# Robot Smach States
from robot_smach_states.navigation.topological_navigation import TopologicalNavigateTo

if __name__ == "__main__":

    # Create node and robot
    rospy.init_node("test_topological_navigation")

    robot = get_robot_from_argv(index=1)

    sm = TopologicalNavigateTo(robot, lambda x: None)
    sm.execute()
