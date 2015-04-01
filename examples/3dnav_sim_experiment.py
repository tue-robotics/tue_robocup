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
    
    end_corridor   = navigation.NavigateGeneric(robot, goal_pose_2d=(-0.5, -12.0, 1.57 ), look_at_path_distance=2.45) 
    hallway        = navigation.NavigateGeneric(robot, goal_pose_2d=( 1.5,  0.0 , 0.0  ), look_at_path_distance=2.45) 
    living_room    = navigation.NavigateGeneric(robot, goal_pose_2d=( 7.8,  0.7 , 0.0  ), look_at_path_distance=2.45)  
    bedroom        = navigation.NavigateGeneric(robot, goal_pose_2d=( 7.0, -6.5 , 1.57 ), look_at_path_distance=2.45) 
    kitchen        = navigation.NavigateGeneric(robot, goal_pose_2d=( 1.7, -6.3 , 0.0  ), look_at_path_distance=2.45)           
    begin_corridor = navigation.NavigateGeneric(robot, goal_pose_2d=(-0.5,  5.0 , 1.57 ), look_at_path_distance=2.45)
    cabinet1       = navigation.NavigateGeneric(robot, goal_pose_2d=( 4.4, -1.45, 3.14 ), look_at_path_distance=2.45)
    cabinet2       = navigation.NavigateGeneric(robot, goal_pose_2d=( 7.8, -1.25, 0.0  ), look_at_path_distance=2.45)
    
    goal_states = [begin_corridor,end_corridor,hallway,bedroom,kitchen,cabinet2]
    
    nr_of_goals = 0;

    while not rospy.is_shutdown():    
        #base.Base(robot).reset_costmap()
        goal_states[random.randint(0,len(goal_states)-1)].execute()
        if goal_states[random.randint(0,len(goal_states)-1)].succeeded()
			nr_of_goals = nr_of_goals + 1;
        rospy.logwarn("nr of goals achieved = {0}".format(nr_of_goals))
    
