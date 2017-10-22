#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import robot_skills
import robot_skills.amigo
from robot_skills.util.kdl_conversions import VectorStamped

import geometry_msgs
import sensor_msgs
import amigo_msgs

global joint_positions
global left_gripper_meas, right_gripper_meas

def joint_state_callback(msg):
    global joint_positions
    for i in range(len(msg.name)):
        joint_positions[msg.name[i]] = msg.position[i]

def left_gripper_callback(msg):
    global left_gripper_meas
    left_gripper_meas = msg

def right_gripper_callback(msg):
    global right_gripper_meas
    right_gripper_meas = msg

def in_bounds(v, test, bound):
    return abs(v - test) < bound

def show_test(message, test):
    if test:
        print '{0:30s} {1:3s}'.format("\033[1m" + message + "\033[0m:", "\033[92mOK\033[0m")
    else:
        print '{0:30s} {1:3s}'.format("\033[1m" + message + "\033[0m:", "\033[0;31mERROR\033[0m")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
#                                           HEAD
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def test_head(amigo):

    global joint_positions

    max_err = 0.02

    # straight
    vs = VectorStamped(x=0.4, z=10, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # up
    vs = VectorStamped(x=0.5, z=1, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head up",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                          and in_bounds(joint_positions['neck_tilt_joint'], -0.26, max_err))

    # down
    vs = VectorStamped(x=-0.4, z=1, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head down",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0.54, max_err))

    # right
    vs = VectorStamped(x=0.4, y=100, z=1, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head right",      in_bounds(joint_positions['neck_pan_joint'], -1.547, max_err)
                            and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # left
    vs = VectorStamped(x=0.4, y=-100, z=1, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head left",      in_bounds(joint_positions['neck_pan_joint'], 1.547, max_err)
                             and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))

    # straight
    vs = VectorStamped(x=0.4, z=10, frame_id='/amigo/torso')
    amigo.head.look_at_point(vs)
    # print str(joint_positions['neck_pan_joint']) + " " + str(joint_positions['neck_tilt_joint'])
    show_test("head straight",      in_bounds(joint_positions['neck_pan_joint'], 0, max_err)
                                and in_bounds(joint_positions['neck_tilt_joint'], 0, max_err))


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
#                                         GRIPPERS
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def test_grippers(amigo):
    global left_gripper_meas, right_gripper_meas

    amigo.leftArm.send_gripper_goal_open(timeout=10)
    show_test("left gripper open", left_gripper_meas.direction == amigo_msgs.msg.AmigoGripperMeasurement.OPEN)

    amigo.leftArm.send_gripper_goal_close(timeout=10)
    show_test("left gripper close", left_gripper_meas.direction == amigo_msgs.msg.AmigoGripperMeasurement.CLOSE)

    amigo.rightArm.send_gripper_goal_open(timeout=10)
    show_test("right gripper open", right_gripper_meas.direction == amigo_msgs.msg.AmigoGripperMeasurement.OPEN)

    amigo.rightArm.send_gripper_goal_close(timeout=10)
    show_test("right gripper close", right_gripper_meas.direction == amigo_msgs.msg.AmigoGripperMeasurement.CLOSE)


def test_torso(amigo):
    global joint_positions

    amigo.torso.send_goal(0.4, timeout=10)
    show_test("torso up", in_bounds(joint_positions['torso_joint'], 0.4, 0.01))

    amigo.torso.send_goal(0.1, timeout=10)
    show_test("torso down", in_bounds(joint_positions['torso_joint'], 0.1, 0.01))

if __name__ == "__main__":
    rospy.init_node('amigo_skills_test_full')

    global joint_positions
    joint_positions = {}

    rospy.Subscriber("/amigo/neck/measurements", sensor_msgs.msg.JointState, joint_state_callback)
    rospy.Subscriber("/amigo/torso/measurements", sensor_msgs.msg.JointState, joint_state_callback)
    rospy.Subscriber("/amigo/left_gripper/measurements", amigo_msgs.msg.AmigoGripperMeasurement, left_gripper_callback)
    rospy.Subscriber("/amigo/right_gripper/measurements", amigo_msgs.msg.AmigoGripperMeasurement, right_gripper_callback)

    amigo = robot_skills.amigo.Amigo(wait_services=True)

    test_torso(amigo)
    test_grippers(amigo)
    test_head(amigo)

