#!/usr/bin/python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy

from robot_skills.amigo import Amigo

import robot_skills
from robot_smach_states import navigation

if __name__ == "__main__":
    rospy.init_node('simple_navigate')

    robot = Amigo()

    rospy.sleep(2) # wait for tf cache to be filled

    nav_state = navigation.NavigateGeneric(robot, goal_pose_2d=(6, -1, 0), look_at_path_distance=2.7)
    nav_state.execute()

    nav_state1 = navigation.NavigateGeneric(robot, goal_pose_2d=(7, -7, 0), look_at_path_distance=2.7)
    nav_state1.execute()
    
    nav_state2 = navigation.NavigateGeneric(robot, goal_pose_2d=(1.5, -7.5, -1.57), look_at_path_distance=2.7)
    nav_state2.execute()
    
    nav_state3 = navigation.NavigateGeneric(robot, goal_pose_2d=(2, 0, 3.14), look_at_path_distance=2.7)
    nav_state3.execute()