#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import robot_skills
import robot_skills.amigo

import geometry_msgs
import sensor_msgs

global joint_positions

def joint_state_callback(msg):
    global joint_positions
    for i in range(len(msg.name)):
        joint_positions[msg.name[i]] = msg.position[i]

def in_bounds(v, test, bound):
    return abs(v - test) < bound

def show_test(message, test):
    if test:
        print '{0:30s} {1:3s}'.format("\033[1m" + message + "\033[0m:", "\033[92mOK\033[0m")
    else:
        print '{0:30s} {1:3s}'.format("\033[1m" + message + "\033[0m:", "\033[0;31mERROR\033[0m")

if __name__ == "__main__":
    rospy.init_node('amigo_skills_test_full')

    global joint_positions
    joint_positions = {}

    rospy.Subscriber("/amigo/neck/measurements", sensor_msgs.msg.JointState, joint_state_callback)

    amigo = robot_skills.amigo.Amigo(wait_services=True)    

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #
    #                                           HEAD
    #
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

    max_err = 0.02

    p = geometry_msgs.msg.PointStamped()
    p.header.frame_id = '/amigo/base_link'

    # straight
    p.point.x = 10
    p.point.y = 0
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
   
    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # up
    p.point.x = 5
    p.point.y = 0
    p.point.z = 3
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    show_test("head up",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                          and in_bounds(joint_positions['neck_tilt_joint'], -0.303218606856, max_err))

    # down
    p.point.x = 1
    p.point.y = 0
    p.point.z = 0
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    show_test("head down",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0.928546372105, max_err))

    # right
    p.point.x = 1
    p.point.y = -10
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    show_test("head right",      in_bounds(joint_positions['neck_pan_joint'], -1.456987231, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # left
    p.point.x = 1
    p.point.y = 10
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    show_test("head left",      in_bounds(joint_positions['neck_pan_joint'], 1.463121363, max_err)
                             and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # straight
    p.point.x = 10
    p.point.y = 0
    p.point.z = 1.5
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)

    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))


