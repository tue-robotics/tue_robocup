#! /usr/bin/env python

from __future__ import print_function

import sys

import rospy

from pykdl_ros import FrameStamped

from robot_skills.get_robot import get_robot

if len(sys.argv) < 2:
    print("Please specify a robot name")
    sys.exit()

robot_name = sys.argv[1]
rospy.init_node("arm_test")

robot = get_robot(robot_name)

# Keep track of which errors occur per arm, so we can report them at the end
failed_actions_per_arm = {}

# First make sure the arms are in a known state
for side, arm in robot._arms.items():
    robot.speech.speak("I will first reset my {} arm".format(side))
    arm.reset()

for side, arm in robot._arms.items():
    failed_actions = []
    robot.speech.speak("I will test my {} arm".format(side))

    if not arm.operational:
        rospy.logerr("{} arm is not operational".format(side))
        sys.exit(-1)

    goal1 = FrameStamped.from_xyz_rpy(0.342,  0, 0.748, 0, 0, 0, rospy.Time.now(), robot.base_link_frame)

    robot.speech.speak("Moving {} arm to dummy goal pose".format(side))
    if not arm.send_goal(goal1):
        robot.speech.speak("{} arm could not reach dummy goal pose".format(side))
        failed_actions += [goal1]
    arm.wait_for_motion_done()

    if arm.gripper.send_goal("open"):
        robot.speech.speak("My {} hand is now open".format(side))
    else:
        robot.speech.speak("Could not open {} hand".format(side))
        failed_actions += ["open gripper"]
    arm.wait_for_motion_done()

    if arm.gripper.send_goal("close"):
        robot.speech.speak("Now my {} hand is closed".format(side))
    else:
        robot.speech.speak("Could not close {} hand".format(side))
        failed_actions += ["close gripper"]
    arm.wait_for_motion_done()

    # Iterate over the default configs of the arms. They are specified in <robot_name>_description/custom/skills.yaml
    for config in arm.default_configurations.keys():
        # Some poses have ..._pose as name, this sounds nicer
        postfix = "pose" if "pose" not in config else ""

        robot.speech.speak("{} arm going to {p} {postfix}".format(side, p=config, postfix=postfix))
        if not arm.send_joint_goal(config, timeout=10):
            # Some poses have ..._pose as name, this sounds nicer
            robot.speech.speak("{} arm could not reach {p} {postfix}".format(side, p=config, postfix=postfix))
            failed_actions += [config]
        arm.wait_for_motion_done()
        arm.reset()
        arm.wait_for_motion_done()

    failed_actions_per_arm[side] = failed_actions

    arm.reset()

for side, failed_configs in failed_actions_per_arm.items():
    rospy.logerr("{side} fails {count} actions: {configs}".format(side=side, count=len(failed_configs), configs=failed_configs))
