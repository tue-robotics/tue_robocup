#!/usr/bin/python
import roslib;
import rospy
import smach
import sys

from robot_smach_states.designators.designator import Designator
import robot_smach_states as states

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                states.StartChallengeRobust(robot, "initial_pose", use_entry_points = True),
                                transitions={   "Done"              :   "SAY_GOTO_WAYPOINT_A",
                                                "Aborted"           :   "SAY_GOTO_WAYPOINT_A",
                                                "Failed"            :   "SAY_GOTO_WAYPOINT_A"})  

        smach.StateMachine.add( 'SAY_GOTO_WAYPOINT_A',
                                states.Say(robot, ["I will go to waypoint a now",
                                                    "I will now go to waypoint a",
                                                    "Lets go to waypoint a",
                                                    "Going to waypoint a"], block=False),
                                transitions={   'spoken'            :   'GOTO_WAYPOINT_A'})

        smach.StateMachine.add('GOTO_WAYPOINT_A',
                                states.NavigateToObserve(robot, Designator("dinner_table"), radius=0.7),
                                transitions={   'arrived'           :   'SAY_WAYPOINT_A_REACHED',
                                                'unreachable'       :   'SAY_WAYPOINT_A_FAILED',
                                                'goal_not_defined'  :   'SAY_WAYPOINT_A_FAILED'})

        smach.StateMachine.add( 'SAY_WAYPOINT_A_REACHED',
                                states.Say(robot, ["Reached waypoint a",
                                                    "I have arrived at waypoint a",
                                                    "I am now at waypoint a"], block=False),
                                transitions={   'spoken'            :   'SAY_GOTO_WAYPOINT_B'})

        # Should we mention that we failed???
        smach.StateMachine.add( 'SAY_WAYPOINT_A_FAILED',
                                states.Say(robot, ["I am not able to reach waypoint a",
                                                    "I cannot reach waypoint a",
                                                    "Waypoint a is unreachable"], block=False),
                                transitions={   'spoken'            :   'SAY_GOTO_WAYPOINT_B'})

        smach.StateMachine.add( 'SAY_GOTO_WAYPOINT_B',
                                states.Say(robot, ["I will go to waypoint B now",
                                                    "I will now go to waypoint B",
                                                    "Lets go to waypoint B",
                                                    "Going to waypoint B"], block=False),
                                transitions={   'spoken'            :   'GOTO_WAYPOINT_B'})

        smach.StateMachine.add('GOTO_WAYPOINT_B',
                                states.NavigateToObserve(robot, Designator("dinner_table"), radius=0.7),
                                transitions={   'arrived'           :   'SAY_WAYPOINT_B_REACHED',
                                                'unreachable'       :   'SAY_WAYPOINT_B_FAILED',
                                                'goal_not_defined'  :   'SAY_WAYPOINT_B_FAILED'})

        smach.StateMachine.add( 'SAY_WAYPOINT_B_REACHED',
                                states.Say(robot, ["Reached waypoint B",
                                                    "I have arrived at waypoint B",
                                                    "I am now at waypoint B"], block=False),
                                transitions={   'spoken'            :   'SAY_GOTO_EXIT'})

        # Should we mention that we failed???
        smach.StateMachine.add( 'SAY_WAYPOINT_B_FAILED',
                                states.Say(robot, ["I am not able to reach waypoint B",
                                                    "I cannot reach waypoint B",
                                                    "Waypoint B is unreachable"], block=False),
                                transitions={   'spoken'            :   'SAY_GOTO_EXIT'})

        smach.StateMachine.add( 'SAY_GOTO_EXIT',
                                states.Say(robot, ["I heard continue, so I will move to the exit now. See you guys later!"], block=False),
                                transitions={   'spoken'            :   'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT',
                                states.NavigateToWaypoint(robot, Designator("exit_1_rips"), radius = 0.5),
                                transitions={   'arrived'           :   'AT_END',
                                                'unreachable'       :   'AT_END',
                                                'goal_not_defined'  :   'AT_END'})

        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={   'spoken'            :   'Done'})
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('navigation_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE NAVIGATION] Please provide robot name as argument."
        exit(1)

    states.util.startup(setup_statemachine, robot_name=robot_name)
