#! /usr/bin/env python

################################################
# Creator: Luis Ferreira (luisfferreira@outlook.com)
# Date: October 2015
################################################

import roslib;
import rospy
import sys
import smach
import smach_ros
import robot_smach_states as states
import geometry_msgs.msg as gm
import robot_skills.util.msg_constructors as msgs
import robot_smach_states.human_interaction.human_interaction as states_interaction

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
from robot_skills.classification_result import ClassificationResult
from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup

# import designators
from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, VariableDesignator, DeferToRuntime, analyse_designators

# import states from another file
import test_states as test_states


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             INITIALIZATIONS
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


# load knowledge
common_knowledge = load_knowledge("common")
challenge_knowledge = load_knowledge("challenge_test")

# define print shortcuts from common knowledge
printOk, printError, printWarning = common_knowledge.make_prints("[Challenge Test] ")


personNameDes = VariableDesignator("", resolve_type=str)
objectsIDsDes = VariableDesignator([], resolve_type=[ClassificationResult])
containerResultDes = VariableDesignator(0, resolve_type=int)



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             NAVIGATION_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class EnterRoomContainer(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:
            smach.StateMachine.add( 'SAY_STARTING_TEST',
                                    states.Say(robot,"Starting navigation test", block=False),
                                    transitions={'spoken':'SAY_TEST_SYMBOLIC'})

            smach.StateMachine.add( 'SAY_TEST_SYMBOLIC',
                                    states.Say(robot,"Testing Navigate To Symbolic", block=False),
                                    transitions={'spoken':'NAV_TO_SYMB'})

            smach.StateMachine.add( 'NAV_TO_SYMB',
                                    states.NavigateToSymbolic(robot,
                                        {EntityByIdDesignator(robot, id="living_room") : "in" },
                                        EntityByIdDesignator(robot, id="dinnertable")),
                                    transitions={   'arrived'           :   'SAY_TEST_WAYPOINT',
                                                    'unreachable'       :   'SAY_FAILED_SYMBOLIC',
                                                    'goal_not_defined'  :   'SAY_FAILED_SYMBOLIC'})

            smach.StateMachine.add( 'SAY_FAILED_SYMBOLIC',
                                    states.Say(robot,"Failed Navigate To Symbolic.", block=True),
                                    transitions={'spoken':'SAY_TEST_WAYPOINT'})

            smach.StateMachine.add( 'SAY_TEST_WAYPOINT',
                                    states.Say(robot,"Testing Navigate To Waypoint.", block=False),
                                    transitions={'spoken':'NAV_TO_WAYPOINT'})

            smach.StateMachine.add( 'NAV_TO_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.wp_test_nav)),
                                    transitions={   'arrived' : 'container_success',
                                                    'unreachable' : 'SAY_FAILED_WAYPOINT',
                                                    'goal_not_defined' : 'SAY_FAILED_WAYPOINT'})

            smach.StateMachine.add( 'SAY_FAILED_WAYPOINT',
                                    states.Say(robot,"Failed reaching the waypoint.", block=True),
                                    transitions={'spoken':'container_success'})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             WAIT_PERSON_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class WaitPersonContainer(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add( "SAY_WAITING_OPERATOR",
                                    states.Say(robot,[  "I'm waiting for the operator to stand in front of me.",
                                                        "Would the operator please come forward.",
                                                        "I need an operator, please stand in front of me."], block=False),
                                    transitions={   'spoken':'LOOK_AT_OPERATOR'})

            smach.StateMachine.add('LOOK_AT_OPERATOR',
                                    states_interaction.LookAtPersonInFront(robot, lookDown=False),
                                    transitions={   'succeeded':'WAIT_FOR_OPERATOR',
                                                    'failed':'WAIT_FOR_OPERATOR'})


            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                    states_interaction.WaitForPersonDetection(robot, attempts=5, sleep_interval=1),
                                    transitions={   'succeeded':'container_success',
                                                    'failed':'SAY_FAILED_WAITING'})

            smach.StateMachine.add( 'SAY_FAILED_WAITING',
                                    states.Say(robot,"I don't see anyone.", block=True),
                                    transitions={'spoken':'WAIT_FOR_OPERATOR_2'})

            smach.StateMachine.add("WAIT_FOR_OPERATOR_2",
                                    states_interaction.WaitForPersonDetection(robot, attempts=5, sleep_interval=1),
                                    transitions={   'succeeded':'container_success',
                                                    'failed':'container_success'})

            smach.StateMachine.add( 'SAY_FAILED_WAITING_AGAIN',
                                    states.Say(robot,"I still couldn't find anyone in front of me. I will continue my task.", block=True),
                                    transitions={'spoken':'WAIT_FOR_OPERATOR_2'})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             LEARN_NAME_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class LearnNameContainer(smach.StateMachine):
    def __init__(self, robot, personNameDesLocal):

        @smach.cb_interface(outcomes=['spoken'])
        def sayIsYourName(userdata):
            printOk("sayIsYourName")
            robot.speech.speak( "I heard " + personNameDes.resolve() + ". Is this correct?", block=True)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def sayCouldNotLearnNameCB(userdata):
            printOk("sayCouldNotLearnNameCB")
            robot.speech.speak( "Sorry but I could not understand your name. I will just call you " + personNameDesLocal.resolve(), block=False)
            return 'spoken'

        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            learnNameIterator = smach.Iterator( outcomes=['iterator_success', 'iterator_failed'],
                                                        it = lambda:range(0, 3),
                                                        it_label='counter',
                                                        input_keys=[],
                                                        output_keys=[],
                                                        exhausted_outcome = 'iterator_failed')

            with learnNameIterator:

                # for some reason the container needs to have the same outcomes as the iterator, plus a "continue" state
                learnNameContainer = smach.StateMachine(outcomes = ['iterator_success', 'iterator_failed', 'inner_container_failed'])
                with learnNameContainer:

                    smach.StateMachine.add( 'ASK_PERSON_NAME',
                                            test_states.AskPersonName(robot, personNameDesLocal.writeable),
                                            transitions={   'succeeded':'SAY_IS_YOUR_NAME',
                                                            'failed':'SAY_HEAR_FAILED'})

                    smach.StateMachine.add( 'SAY_IS_YOUR_NAME',
                                            smach.CBState(sayIsYourName),
                                            transitions={    'spoken':'HEAR_YES_NO_1'})

                    smach.StateMachine.add( 'HEAR_YES_NO_1',
                                            states_interaction.HearYesNo(robot),
                                            transitions={   'heard_yes' : 'iterator_success',
                                                            'heard_no' : 'SAY_NAME_WRONG',
                                                            'heard_failed' : 'SAY_HEAR_FAILED'})

                    smach.StateMachine.add( 'SAY_NAME_WRONG',
                                            states.Say(robot, ["Sorry, I understood wrong.", "Oh I'm sorry."], block=False),
                                            transitions={    'spoken':'inner_container_failed'})

                    smach.StateMachine.add( 'SAY_HEAR_FAILED',
                                            states.Say(robot, ["I did not understand your name.", "I didn't listen correctly."], block=False),
                                            transitions={    'spoken':'inner_container_failed'})

                smach.Iterator.set_contained_state( 'LEARN_NAME_CONTAINER',
                                                    learnNameContainer,
                                                    loop_outcomes=['inner_container_failed'])
                                                    # break_outcomes=['iterator_success'])

            smach.StateMachine.add( 'LEARN_NAME_ITERATOR',
                                    learnNameIterator,
                                    transitions={   'iterator_success':'container_success',
                                                    'iterator_failed':'SAY_COULD_NOT_LEARN_NAME'})

            smach.StateMachine.add( 'SAY_COULD_NOT_LEARN_NAME',
                                    smach.CBState(sayCouldNotLearnNameCB),
                                    transitions={   'spoken':'container_failed'})



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                                 LEARN_FACE_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class LearnFaceContainer(smach.StateMachine):
    def __init__(self, robot, personNameDesLocal):

        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                    states.Say(robot,"Please stand in front of me and look at my camera while I learn your face.", block=False),
                                    transitions={    'spoken':'LOOK_AT_OPERATOR'})

            smach.StateMachine.add( 'LOOK_AT_OPERATOR',
                                    states_interaction.LookAtPersonInFront(robot, lookDown=True),
                                    transitions={   'succeeded':'LEARN_PERSON',
                                                    # 'failed':'SAY_LEARN_FACE_FAILED'})
                                                    'failed':'LEARN_PERSON'})

            smach.StateMachine.add( 'LEARN_PERSON',
                                    states.LearnPerson(robot, name_designator = personNameDesLocal),
                                    transitions={   'succeeded_learning':'container_success',
                                                    'failed_learning':'SAY_LEARN_FACE_FAILED',
                                                    'timeout_learning':'SAY_LEARN_FACE_FAILED'})

            smach.StateMachine.add( 'SAY_LEARN_FACE_FAILED',
                                    states.Say(robot,"I could not learn your face.", block=False),
                                    transitions={    'spoken':'container_failed'})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                                 PICK_UP_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class PickUpContainer(smach.StateMachine):
    def __init__(self, robot, objectsIDsDesLocal):

        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add( 'SAY_STARTING_TEST',
                                    states.Say(robot,"Starting object pickup test test", block=False),
                                    transitions={'spoken':'SAY_SEARCHING_OBJECTS'})

            smach.StateMachine.add( 'SAY_SEARCHING_OBJECTS',
                                    states.Say(robot,"I'm going to the dinning table to search for objects", block=False),
                                    transitions={'spoken':'NAV_TO_TABLE'})

            smach.StateMachine.add( 'NAV_TO_TABLE',
                                    states.NavigateToSymbolic(robot,
                                        {EdEntityDesignator(robot, id=challenge_knowledge.INSPECT_ROOM_ID) : "in" },
                                        EdEntityDesignator(robot, id=challenge_knowledge.INSPECT_ENTITY_ID)),
                                    transitions={   'arrived'           :   'SEGMENT_OBJECTS',
                                                    'unreachable'       :   'SAY_FAILED_NAV_TO_TABLE',
                                                    'goal_not_defined'  :   'SAY_FAILED_NAV_TO_TABLE'})


            smach.StateMachine.add( 'SAY_FAILED_NAV_TO_TABLE',
                                    states.Say(robot,"I could not reach the table, but i will try to continue.", block=True),
                                    transitions={'spoken':'SEGMENT_OBJECTS'})

            smach.StateMachine.add( "SEGMENT_OBJECTS",
                                    states.SegmentObjects(robot, objectsIDsDesLocal.writeable, EdEntityDesignator(robot, id=challenge_knowledge.INSPECT_ENTITY_ID), "on_top_of"),
                                    transitions={   'done':'PICKUP_OBJECT'})

            smach.StateMachine.add( 'PICKUP_OBJECT',
                                    test_states.PickUpRandomObj(robot, objectsIDsDesLocal),
                                    transitions={   'succeeded':'SAY_I_HAVE_OBJ',
                                                    'failed':'SAY_PICKUP_FAILED',
                                                    'no_objects' : 'SAY_NO_OBJECTS'})

            smach.StateMachine.add( 'SAY_I_HAVE_OBJ',
                                    states.Say(robot,"I have the object!", block=False),
                                    transitions={'spoken':'container_success'})

            smach.StateMachine.add( 'SAY_PICKUP_FAILED',
                                    states.Say(robot,"Something went wrong and i could not pick up the object!", block=False),
                                    transitions={'spoken':'container_failed'})

            smach.StateMachine.add( 'SAY_NO_OBJECTS',
                                    states.Say(robot,"I don't see any objects to pick up!", block=False),
                                    transitions={'spoken':'container_failed'})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             SEARCH_PEOPLE_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SearchPeopleContainer(smach.StateMachine):
    def __init__(self, robot, personNameDesLocal):

        @smach.cb_interface(outcomes=['spoken'])
        def saySearchingOperatorCB(userdata = None):
            printOk("saySearchingOperatorCB")
            robot.speech.speak( "I am searching for " + personNameDesLocal.resolve() + "!", block=False)
            return 'spoken'

        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add( 'NAV_TO_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.wp_test_nav)),
                                    transitions={   'arrived' : 'SAY_SEARCHING_OPERATOR',
                                                    'unreachable' : 'SAY_FAILED_WAYPOINT',
                                                    'goal_not_defined' : 'SAY_FAILED_WAYPOINT'})

            smach.StateMachine.add( 'SAY_FAILED_WAYPOINT',
                                    states.Say(robot,"Failed reaching the waypoint.", block=True),
                                    transitions={'spoken':'SAY_SEARCHING_OPERATOR'})

            smach.StateMachine.add( 'SAY_SEARCHING_OPERATOR',
                                    smach.CBState(saySearchingOperatorCB),
                                    transitions={   'spoken':'SAY_UNFINSHED'})

            smach.StateMachine.add( 'SAY_UNFINSHED',
                                    states.Say(robot,"This part is not finished yet.", block=False),
                                    transitions={'spoken':'container_success'})



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                                 RECOGNIZE_PEOPLE_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


class RecognizePeopleContainer(smach.StateMachine):
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add( 'SAY_STARTING_TEST',
                                    states.Say(robot,"Starting people recognition test", block=False),
                                    transitions={'spoken':'NAV_TO_WAYPOINT'})

            smach.StateMachine.add( 'NAV_TO_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.wp_test_nav)),
                                    transitions={   'arrived' : 'LOOK_AT_OPERATOR',
                                                    'unreachable' : 'SAY_FAILED_WAYPOINT',
                                                    'goal_not_defined' : 'SAY_FAILED_WAYPOINT'})

            smach.StateMachine.add( 'SAY_FAILED_WAYPOINT',
                                    states.Say(robot,"Failed reaching the waypoint.", block=True),
                                    transitions={'spoken':'LOOK_AT_OPERATOR'})

            smach.StateMachine.add('LOOK_AT_OPERATOR',
                                    states_interaction.LookAtPersonInFront(robot, lookDown=False),
                                    transitions={   'succeeded':'RECOGNIZE_PEOPLE',
                                                    'failed':'RECOGNIZE_PEOPLE'})

            smach.StateMachine.add('RECOGNIZE_PEOPLE',
                                    test_states.RecognizePeople(robot),
                                    transitions={   'succeeded':'container_success',
                                                    'failed':'SAY_NO_PEOPLE',
                                                    'no_people':'SAY_NO_PEOPLE'})

            smach.StateMachine.add( 'SAY_NO_PEOPLE',
                                    states.Say(robot,"I don see anyone in front of me.", block=False),
                                    transitions={'spoken':'LOOK_AT_OPERATOR'})




# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             MAIN STATE MACHINE
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class ChallengeTest(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])


        # - - - - - - - - - - - - - - - - - - - Callback States  - - - - - - - - - - - - - - - - - - -

        @smach.cb_interface(outcomes=['spoken'])
        def sayHelloCB(userdata = None):
            printOk("sayHelloCB")
            robot.speech.speak( "Hello " + personNameDes.resolve() + "!", block=False)
            return 'spoken'

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        with self:

            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={   'initialized':'INIT_WM',
                                                    'abort':'Aborted'})

            smach.StateMachine.add( "INIT_WM",
                                    states.InitializeWorldModel(robot),
                                    transitions={   'done':'ENTER_ROOM_CONTAINER'})

            # smach.StateMachine.add( 'SELECT_NEXT_CONTAINER',
            #                         test_states.SelectNextContainer(robot, containerResultDes),
            #                         transitions={   'go_to_enter_room':'ENTER_ROOM_CONTAINER',
            #                                         'go_to_wait_person':'WAIT_PERSON_CONTAINER',
            #                                         'go_to_pick_up':'PICK_UP_CONTAINER',
            #                                         'go_to_recognize_people':'RECOGNIZE_PEOPLE_CONTAINER',
            #                                         'go_to_search_people':'SEARCH_PEOPLE_CONTAINER',
            #                                         'go_to_end_challenge':'END_CHALLENGE'})

            # Navigation test
            smach.StateMachine.add( 'ENTER_ROOM_CONTAINER',
                                    EnterRoomContainer(robot),
                                    transitions={   'container_success':'WAIT_PERSON_CONTAINER',
                                                    'container_failed': 'WAIT_PERSON_CONTAINER'})

            # Human Interaction Test
            smach.StateMachine.add( 'WAIT_PERSON_CONTAINER',
                                    WaitPersonContainer(robot),
                                    transitions={   'container_success':'LEARN_NAME_CONTAINER',
                                                    'container_failed': 'WAIT_PERSON_CONTAINER'})

            # Speech Test
            smach.StateMachine.add( 'LEARN_NAME_CONTAINER',
                                    LearnNameContainer(robot, personNameDes),
                                    transitions={   'container_failed':'SAY_HELLO',
                                                    'container_success':'SAY_HELLO'})

            smach.StateMachine.add( 'SAY_HELLO',
                                    smach.CBState(sayHelloCB),
                                    transitions={   'spoken':'LEARN_FACE_CONTAINER'})

            # Face Learning Test
            smach.StateMachine.add( 'LEARN_FACE_CONTAINER',
                                    LearnFaceContainer(robot, personNameDes),
                                    transitions={   'container_success':'RECOGNIZE_PEOPLE_CONTAINER',
                                                    'container_failed': 'RECOGNIZE_PEOPLE_CONTAINER'})

            # Face Recognition Test
            smach.StateMachine.add( 'RECOGNIZE_PEOPLE_CONTAINER',
                                    RecognizePeopleContainer(robot),
                                    transitions={   'container_success':'PICK_UP_CONTAINER',
                                                    'container_failed': 'PICK_UP_CONTAINER'})

            # Manipulation Test
            smach.StateMachine.add( 'PICK_UP_CONTAINER',
                                    PickUpContainer(robot, objectsIDsDes),
                                    transitions={   'container_success':'SEARCH_PEOPLE_CONTAINER',
                                                    'container_failed': 'SEARCH_PEOPLE_CONTAINER'})

            # Face Recogniton Test
            smach.StateMachine.add( 'SEARCH_PEOPLE_CONTAINER',
                                    SearchPeopleContainer(robot, personNameDes),
                                    transitions={   'container_success':'END_CHALLENGE',
                                                    'container_failed': 'END_CHALLENGE'})

            smach.StateMachine.add( 'END_CHALLENGE',
                                    states.Say(robot,"My work here is done, goodbye!"),
                                    transitions={    'spoken':'Done'})



############################## MAIN ##############################

if __name__ == "__main__":
    rospy.init_node('test_executive')

    startup(ChallengeTest, challenge_name="test")


'''
Setup:
    - insert 2 persons in "computer room"
    - insert person in living room, in sight
    - insert 3 drinks in dinning table
    - insert object behind the kitchen table


Behaviours to test:
    - navigate to waypoint
    - navigate to object
    - avoid obstacle
    - detect/recognize objects
    - detect/recognize people
    - speach
    - listenning
    - grasping object
    - placing object
    - test designators


Executive:
    - drive to living room (waypoint)
    - test speech: say waiting for people
    - facing forward, learn the person in front
    - drive to dinning room (object)
    - test speech: say looking for objects on top of the table
    - detect and recognize the three drinks
    - grab one of the drinks
    - navigate to behind the "kitchen counter", and test navigation fail by placing an object in the way
    - navigate to second door
    - recognize the person in front
    - finish challenge
'''
