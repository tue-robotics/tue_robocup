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
    
    nav_state_end_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(0, 0.25, 0), look_at_path_distance=2.4)
    nav_state_end_corridor.execute()
    
    #nav_state_door = navigation.NavigateGeneric(robot, goal_pose_2d=(1, 0, 0), look_at_path_distance=2.7)
    #nav_state_door.execute()    
    
    #nav_state_living_room = navigation.NavigateGeneric(robot, goal_pose_2d=(7.8, 0.7, 0), look_at_path_distance=2.7)
    #nav_state_living_room.execute()
    
    #nav_state_bedroom = navigation.NavigateGeneric(robot, goal_pose_2d=(7, -6.5, 1.57), look_at_path_distance=2.7)
    #nav_state_bedroom.execute() 
        
    nav_state_begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(4.0, 0, 1.57), look_at_path_distance=2.4)
    nav_state_begin_corridor.execute()
