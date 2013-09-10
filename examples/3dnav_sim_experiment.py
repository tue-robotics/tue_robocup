#!/usr/bin/python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import random

from robot_skills.amigo import Amigo

import robot_skills
from robot_smach_states import navigation

if __name__ == "__main__":
    rospy.init_node('simple_navigate')

    robot = Amigo()

    rospy.sleep(2) # wait for tf cache to be filled

    while not rospy.is_shutdown():    
        random_goal = navigation.NavigateGeneric(robot, goal_pose_2d=(random.uniform(-0.9,10.0),random.uniform(1.0,-8.0),random.uniform(-3.14,3.14)), look_at_path_distance=2.7)
        random_goal.execute()
    
