#!/usr/bin/python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy

from robot_skills.amigo import Amigo

import robot_skills
import robot_smach_states

if __name__ == "__main__":
    rospy.init_node('simple_navigate')

    robot = Amigo()

    rospy.sleep(2) # wait for tf cache to be filled

    nav_state = navigation.NavigateGeneric(robot, goal_pose_2d=(4, 0, 0), look_at_path_distance=1.5)
    nav_state.execute()
