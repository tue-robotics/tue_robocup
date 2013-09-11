#!/usr/bin/python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import random

from robot_skills.amigo import Amigo
import robot_skills

from robot_smach_states import navigation
from robot_skills import base

if __name__ == "__main__":
    rospy.init_node('3dnav_sim_experiment')

    robot = Amigo()

    rospy.sleep(2) # wait for tf cache to be filled
    
    end_corridor   = navigation.NavigateGeneric(robot, goal_pose_2d=(-0.5, -11.0, 1.57 ), look_at_path_distance=2.7) 
    hallway        = navigation.NavigateGeneric(robot, goal_pose_2d=(2   , 0    , 0    ), look_at_path_distance=2.7) 
    living_room    = navigation.NavigateGeneric(robot, goal_pose_2d=(7.8 , 0.7  , 0    ), look_at_path_distance=2.7)  
    bedroom        = navigation.NavigateGeneric(robot, goal_pose_2d=(7   , -6.5 , 1.57 ), look_at_path_distance=2.7) 
    kitchen        = navigation.NavigateGeneric(robot, goal_pose_2d=(3.2 , -5.5 ,-1.57 ), look_at_path_distance=2.7)           
    begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(-0.5, 2.5  , 1.57 ), look_at_path_distance=2.7)
    cabinet1       = navigation.NavigateGeneric(robot, goal_pose_2d=(4.4 , -1.45, 3.14 ), look_at_path_distance=2.7)
    cabinet2       = navigation.NavigateGeneric(robot, goal_pose_2d=(7.8 , -1.25, 0.0  ), look_at_path_distance=2.7)
    
    goal_states = [begin_corridor,end_corridor,hallway,bedroom,kitchen,cabinet2]

    while not rospy.is_shutdown():    
        #base.Base(robot).reset_costmap()
        goal_states[random.randint(0,len(goal_states)-1)].execute()
    
