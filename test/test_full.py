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
    p.header.frame_id = '/amigo/torso'

    # straight
    p.point.x = 0.4
    p.point.y = 0
    p.point.z = 10
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])   
    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # up
    p.point.x = 0.5
    p.point.y = 0
    p.point.z = 1
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head up",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                          and in_bounds(joint_positions['neck_tilt_joint'], -0.26, max_err))

    # down
    p.point.x = -0.4
    p.point.y = 0
    p.point.z = 1
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head down",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0.54, max_err))

    # right
    p.point.x = 0.4
    p.point.y = 100
    p.point.z = 1
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head right",      in_bounds(joint_positions['neck_pan_joint'], -1.547, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # left
    p.point.x = 0.4
    p.point.y = -100
    p.point.z = 1
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head left",      in_bounds(joint_positions['neck_pan_joint'], 1.547, max_err)
                             and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # straight
    p.point.x = 0.4
    p.point.y = 0
    p.point.z = 10
    amigo.head.send_goal(p, timeout=10.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))


