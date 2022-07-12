import robot_smach_states.util.designators as ds
import smach

from .locate_people import LocatePeople
from .report_people import ReportPeople
from robocup_knowledge import load_knowledge
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.utility import WaitTime

challenge_knowledge = load_knowledge('challenge_find_my_mates')

STARTING_POINT = challenge_knowledge.starting_point
ROOM_ID = challenge_knowledge.room
SEARCH_POINT = challenge_knowledge.search_point
OPERATOR_POINT = challenge_knowledge.operator_point


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['done', 'failed', 'aborted'])

    with sm:
        smach.StateMachine.add('START_CHALLENGE_ROBUST', StartChallengeRobust(robot, STARTING_POINT),
                               transitions={'Done': 'GO_TO_SEARCH_POSE',
                                            'Aborted': 'aborted',
                                            'Failed': 'GO_TO_SEARCH_POSE'})

        smach.StateMachine.add('SAY_START',
                               Say(robot, "Finding your mates, here we go!", block=False),
                               transitions={'spoken': 'GO_TO_SEARCH_POSE'})

        smach.StateMachine.add('GO_TO_SEARCH_POSE',
                               NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, uuid=SEARCH_POINT),
                                                  radius=0.375),
                               transitions={'arrived': 'RISE_FOR_THE_PEOPLE',
                                            'goal_not_defined': 'failed',
                                            'unreachable': 'WAIT_SEARCH_POSE'})

        smach.StateMachine.add('WAIT_SEARCH_POSE',
                               WaitTime(robot, 5),
                               transitions={'preempted': 'aborted',
                                            'waited': 'GO_TO_SEARCH_POSE'})

        @smach.cb_interface(outcomes=["done"])
        def _rise_for_the_people(userdata=None):
            """ Resets the location hmi attempt so that each operator gets three attempts """
            list(robot.arms.values())[0]._send_joint_trajectory([[0.70, -1.9, 0.0, -1.57, 0.0]])
            robot.speech.speak("Hi there. My Name is Hero. I'm looking for the mates of my operator", block=False)
            return "done"

        smach.StateMachine.add("RISE_FOR_THE_PEOPLE",
                               smach.CBState(_rise_for_the_people),
                               transitions={"done": "LOCATE_PEOPLE"})

        # locate all four people
        smach.StateMachine.add('LOCATE_PEOPLE',
                               LocatePeople(robot,
                                            room_id=ROOM_ID
                                            ),
                               transitions={'done': 'RESET_FOR_DRIVING',
                                            }
                               )

        @smach.cb_interface(outcomes=["done"])
        def _reset_for_driving(userdata=None):
            """ Resets the location hmi attempt so that each operator gets three attempts """
            robot.speech.speak("Thank you for your attention", block=False)
            list(robot.arms.values())[0]._send_joint_trajectory([[0.01, -1.9, 0.0, -1.57, 0.0],  # Inspect with q0 low
                                                                 [0.01, 0.0, -1.57, -1.57, 0.0]])  # Reset
            return "done"

        smach.StateMachine.add("RESET_FOR_DRIVING",
                               smach.CBState(_reset_for_driving),
                               transitions={"done": "GO_BACK_TO_OPERATOR"})

        # drive back to the operator to describe the mates
        smach.StateMachine.add('GO_BACK_TO_OPERATOR',
                               NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, uuid=OPERATOR_POINT),
                                                  radius=0.7,
                                                  look_at_designator=ds.EntityByIdDesignator(robot, uuid=OPERATOR_POINT)),
                               transitions={'arrived': 'REPORT_PEOPLE',
                                            'goal_not_defined': 'REPORT_PEOPLE',
                                            'unreachable': 'WAIT_GO_BACK'})

        smach.StateMachine.add('WAIT_GO_BACK',
                               WaitTime(robot, 5),
                               transitions={'preempted': 'aborted',
                                            'waited': 'GO_BACK_TO_OPERATOR'})

        # check how to uniquely define them  # ToDo: make this more interesting
        smach.StateMachine.add('REPORT_PEOPLE', ReportPeople(robot),
                               transitions={'done': 'done'}
                               )

    return sm
