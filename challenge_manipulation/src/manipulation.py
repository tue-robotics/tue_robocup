#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Manipulation.tex

In short, the robot starts at 1-1.5m from a bookcase and must wait until started by an operator (by voice or a start button)

This bookcase has a couple of shelves on which some items are placed. 
**The middle shelve starts empty**, this is where the objects need to be placed. 

The robot must take objects form the shelves and place them on the middle shelve and indicate the class of each grasped object. 

After the robot is started by voice or a button, 
    the ManipRecogSingleItem state machine is repeated at least 5 times (for 5 objects). 
Afterwards, a PDF report has to be made:
'After the test is completed or the time has run out, 
    the robot may upload a single PDF report file including the list of recognized objects with a picture showing:
    - the object, 
    - the object name,
    - the bounding box of the object.'
"""

import rospy
import smach
import sys

from robot_smach_states.designators.designator import Designator
import robot_smach_states as states

class ManipRecogSingleItem(smach.StateMachine):
    """The ManipRecogSingleItem state machine (for one object) is:
    - Stand of front of the bookcase
    - Look at the bookcase
    - Select an item, which is: 
        - inside the bookcase
        - not yet grasped/not on the middle shelve
    - Grab that item
    - Say the class of the grabbed item
    - Place the item in an open spot on the middle shelve. """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        bookcase = Designator("bookcase")  #TODO: Get the entityID of the bookcase

        item_to_grasp = Designator()  # Some item to grasp from the bookcase that is _not_ on the empty placement-shelve
        place_position = Designator()  # Designates an empty spot on the empty placement-shelve. 

        with self:
            smach.StateMachine.add( "NAV_TO_OBSERVE_BOOKCASE",
                                    states.NavigateToObserve(robot, bookcase),
                                    transitions={   'arrived'           :'LOOKAT_BOOKCASE',
                                                    'unreachable'       :'LOOKAT_BOOKCASE',
                                                    'goal_not_defined'  :'LOOKAT_BOOKCASE'})

            smach.StateMachine.add( "LOOKAT_BOOKCASE",
                                    states.Say(robot, ["I'm looking at the bookcase to see what items I can find"]),
                                    transitions={   'spoken'            :'GRAB_ITEM'})

            smach.StateMachine.add( "GRAB_ITEM",
                                    states.Grab(robot, item_to_grasp),
                                    transitions={   'done'              :'ANNOUNCE_CLASS',
                                                    'failed'            :'NAV_TO_OBSERVE_BOOKCASE'})

            smach.StateMachine.add( "ANNOUNCE_CLASS",
                                    states.SayFormatted(robot, ["This is a {0.type}."], [item_to_grasp], blocking=False),
                                    transitions={   'spoken'            :'PLACE_ITEM'})

            smach.StateMachine.add( "PLACE_ITEM",
                                    states.Place(robot, item_to_grasp, place_position),
                                    transitions={   'done'              :'succeeded',
                                                    'failed'            :'SAY_HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( "SAY_HANDOVER_TO_HUMAN",
                                    states.Say(robot, ["I'm can't get rid of this item  myself, can somebody help me maybe?"]),
                                    transitions={   'spoken'            :'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( "HANDOVER_TO_HUMAN",
                                    states.HandoverToHuman(robot),
                                    transitions={   'succeeded'         :'succeeded',
                                                    'failed'            :'failed'})






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
