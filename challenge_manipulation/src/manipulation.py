#!/usr/bin/python
import rospy
import smach
import sys

from robot_smach_states.designators.designator import Designator
import robot_smach_states as states


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               states.StartChallengeRobust(
                            robot, "initial_pose", use_entry_points=True),
                               transitions={"Done": "AT_END",
                                            "Aborted": "AT_END",
                                            "Failed": "AT_END"})

        smach.StateMachine.add('AT_END',
                               states.Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})
    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('manipulation_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    states.util.startup(setup_statemachine, robot_name=robot_name)
