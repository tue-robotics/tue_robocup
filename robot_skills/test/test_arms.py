#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("audio_test")
if robot_name == "amigo":
    from robot_skills.amigo import Amigo
    robot = Amigo()
elif robot_name == "sergio":
    from robot_skills.sergio import Sergio
    robot = Sergio()
else:
    print "Unknown robot '%s'"%robot_name
    sys.exit()

for side, arm in robot.arms.items():
    robot.speech.speak("I will test my {} arm".format(side))

    if not arm.operational:
        rospy.logerr("{} arm is not operational".format(side))
        sys.exit(-1)

    goal1 = kdl_conversions.kdlFrameStampedFromXYZRPY(0.192, 0.125, 0.748, 0, 0, 0, "/"+robot.robot_name+"/base_link")

    if not arm.send_goal(goal1):
        robot.speech.speak("Could not reach arm {} dummy goal pose".format(side))
    arm.wait_for_motion_done()

    # TODO: Now check that the hand frame is within some tolerance to the desired goal.
    # Kwin's AR comparison stuff may also come in handy here.

    if arm.send_gripper_goal("open"):
        robot.speech.speak("My {} hand is now open".format(side))
    else:
        robot.speech.speak("Could not open {} hand".format(side))
    arm.wait_for_motion_done()

    if arm.send_gripper_goal("close"):
        robot.speech.speak("Now my {} hand is closed".format(side))
    else:
        robot.speech.speak("Could not close {} hand".format(side))
    arm.wait_for_motion_done()


    for config in arm.default_configurations.keys():
        robot.speech.speak("{} arm going to {p} {postfix}".format(side, p=config, postfix="pose" if "pose" not in config else ""))
        if not arm.send_joint_goal(config):
            robot.speech.speak("{} arm could not reach {p} {postfix}".format(side, p=config, postfix="pose" if "pose" not in config else ""))
        arm.wait_for_motion_done()
        arm.reset()
        arm.wait_for_motion_done()

    arm.reset()
