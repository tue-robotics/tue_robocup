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

    nav_state = navigation.NavigateGeneric(robot, goal_pose_2d=(6, 6, 0), look_at_path_distance=1.5)
    nav_state.execute()

    reset_state = navigation.ResetCostmap(robot)
    reset_state.execute()
    
    #nav_state2 = navigation.NavigateGeneric(robot, goal_pose_2d=(7, -5, -1.57), look_at_path_distance=1.5)
    #nav_state2.execute()
    
    #nav_state3 = navigation.NavigateGeneric(robot, goal_pose_2d=(2, -5, 3.14), look_at_path_distance=1.5)
    #nav_state3.execute()
    
    #nav_state4 = navigation.NavigateGeneric(robot, goal_pose_2d=(0, 0, 0), look_at_path_distance=1.5)
    #nav_state4.execute()
