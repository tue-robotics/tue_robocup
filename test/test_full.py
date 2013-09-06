#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import robot_skills
import robot_skills.amigo

import geometry_msgs

if __name__ == "__main__":
    rospy.init_node('amigo_skills_test_full')

    amigo = robot_skills.amigo.Amigo(wait_services=True)

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #
    #                                           HEAD
    #
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

    p = geometry_msgs.msg.PointStamped()
    p.header.frame_id = '/amigo/base_link'

    # straight
    p.point.x = 10
    p.point.y = 0
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    # up
    p.point.x = 10
    p.point.y = 0
    p.point.z = 3
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    # down
    p.point.x = 1
    p.point.y = 0
    p.point.z = 0
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    # left
    p.point.x = 1
    p.point.y = -10
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    # right
    p.point.x = 1
    p.point.y = 10
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    # straight
    p.point.x = 10
    p.point.y = 0
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)


