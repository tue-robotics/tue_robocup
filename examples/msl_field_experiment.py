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
    
    nav_state_begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(4.2, 4.2, -1.57), look_at_path_distance=2.7)
    nav_state_begin_corridor.execute()
    
    nav_state_door = navigation.NavigateGeneric(robot, goal_pose_2d=(1.8, 0.8, 3.14), look_at_path_distance=2.7)
    nav_state_door.execute()    
    
    #nav_state_end_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(-3, 5, -1.57), look_at_path_distance=2.7)
    #nav_state_end_corridor.execute()
    
    #nav_state_table = navigation.NavigateGeneric(robot, goal_pose_2d=(2.2, 1.7, 1.57), look_at_path_distance=2.7)
    #nav_state_table.execute() 
        
    #nav_state_begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(4, -4.4, 1.57), look_at_path_distance=2.7)
    #nav_state_begin_corridor.execute()
