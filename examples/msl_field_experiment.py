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
    
    nav_state_begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(4.8, 4.5, -1.57), look_at_path_distance=2.4)
    nav_state_begin_corridor.execute()
    
    nav_state_end_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(4.8, -3.4, -3.14), look_at_path_distance=2.4)
    nav_state_end_corridor.execute()
    
    nav_state_living_room = navigation.NavigateGeneric(robot, goal_pose_2d=(-2.0, -3.4, 1.57), look_at_path_distance=2.4)
    nav_state_living_room.execute()    
       
    nav_state_kitchen = navigation.NavigateGeneric(robot, goal_pose_2d=(-2.0, 3.0, 0), look_at_path_distance=2.4)
    nav_state_kitchen.execute() 
        
    nav_state_kitchen_2 = navigation.NavigateGeneric(robot, goal_pose_2d=(3.0, 3.0, -1.57), look_at_path_distance=2.4)
    nav_state_kitchen_2.execute()
    
    nav_state_living_room_2 = navigation.NavigateGeneric(robot, goal_pose_2d=(-2.0, -1.5, 0), look_at_path_distance=2.4)
    nav_state_living_room_2.execute()    
       
