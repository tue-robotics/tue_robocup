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
from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup
from robot_smach_states.utility import CancelHeadGoals
import robot_smach_states.human_interaction.human_interaction as states_interaction

# import designators
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, DeferToRuntime, analyse_designators

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


personNameDes = VariableDesignator("")


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             ENTER_ROOM_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class EnterContainer(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add('SAY_TEST_SYMBOLIC',
                                   states.Say(robot,"Testing Navigate To Symbolic, going to the couch table.", block=False),
                                   transitions={'spoken':'NAV_TO_SYMB'})

            smach.StateMachine.add('NAV_TO_SYMB',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id="living_room") : "in" }, 
                                        EdEntityDesignator(robot, id="couchtable")),
                                    transitions={   'arrived'           :   'SAY_TEST_WAYPOINT',
                                                    'unreachable'       :   'SAY_FAILED_SYMBOLIC',
                                                    'goal_not_defined'  :   'SAY_FAILED_SYMBOLIC'})

            smach.StateMachine.add('SAY_FAILED_SYMBOLIC',
                                   states.Say(robot,"Failed Navigate To Symbolic.", block=True),
                                   transitions={'spoken':'SAY_TEST_WAYPOINT'})

            smach.StateMachine.add('SAY_TEST_WAYPOINT',
                                   states.Say(robot,"Testing Navigate To Waypoint.", block=False),
                                   transitions={'spoken':'NAV_TO_WAYPOINT'})

            smach.StateMachine.add( 'NAV_TO_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.wp_test_nav)),
                                    transitions={   'arrived' : 'container_success',
                                                    'unreachable' : 'SAY_FAILED_WAYPOINT',
                                                    'goal_not_defined' : 'SAY_FAILED_WAYPOINT'})

            smach.StateMachine.add('SAY_FAILED_WAYPOINT',
                                   states.Say(robot,"Failed Navigate To Waypoint.", block=True),
                                   transitions={'spoken':'container_success'})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             WAIT_PERSON_CONTAINER
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class WaitPersonContainer(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['container_success','container_failed'])

        with self:

            smach.StateMachine.add("SAY_WAITING_OPERATOR",
                                    states.Say(robot,[  "I'm waiting for the operator to stand in front of me.",
                                                        "Would the operator please come forward.",
                                                        "I need an operator, please stand in front of me."], block=False),
                                    transitions={   'spoken':'LOOK_AT_OPERATOR'})

            smach.StateMachine.add('LOOK_AT_OPERATOR',
                                    states_interaction.LookAtPersonInFront(robot, lookDown=False),
                                    transitions={   'succeded':'WAIT_FOR_OPERATOR',
                                                    'failed':'WAIT_FOR_OPERATOR'})


            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                    states_interaction.WaitForPerson(robot, attempts=5, sleep_interval=1),
                                    transitions={   'succeded':'container_success',
                                                    'failed':'container_success'})
                                                    # 'failed':'SAY_WAITING_OPERATOR'})




# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             LEARN_PERSON_NAME
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class LearnNameContainer(smach.StateMachine):
    def __init__(self, robot, personNameDes):
        
        @smach.cb_interface(outcomes=['spoken'])
        def sayIsYourName(userdata):
            printOk("sayIsYourName")
            robot.speech.speak( "I heard " + personNameDes.resolve() + ". Is this correct?", block=True)
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

                # for some reason the container needs to have the same outcomes as the iterator, plus equivalent to "continue"
                learnNameContainer = smach.StateMachine(outcomes = ['iterator_success', 'iterator_failed', 'inner_container_failed'])
                with learnNameContainer:

                    smach.StateMachine.add( 'ASK_PERSON_NAME',
                                            test_states.AskPersonName(robot, personNameDes),
                                            transitions={   'succeded':'SAY_IS_YOUR_NAME',
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
                                                    'iterator_failed':'container_failed'})



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                             MAIN STATE MACHINE
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class ChallengeTest(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])


        # - - - - - - - - - - - - - - - - - - - Callback States  - - - - - - - - - - - - - - - - - - -

        @smach.cb_interface(outcomes=['spoken'])
        def sayCouldNotLearnNameCB(userdata):
            printOk("sayCouldNotLearnNameCB")
            robot.speech.speak( "Sorry but I could not understand your name. I will just call you " + personNameDes.resolve(), block=False)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def sayHelloCB(userdata):
            printOk("sayHelloCB")
            robot.speech.speak( "Hello " + personNameDes.resolve() + "!", block=False)
            return 'spoken'

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        with self:

            # Initializations

            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={   'initialized':'INIT_WM',
                                                    'abort':'Aborted'})

            smach.StateMachine.add( "INIT_WM",
                                    states.InitializeWorldModel(robot), 
                                    # transitions={   'done':'LEARN_NAME_CONTAINER'})
                                    transitions={   'done':'ENTER_ROOM_CONTAINER'})
            

            # test navigation by going to an object and a waypoint
            smach.StateMachine.add( 'ENTER_ROOM_CONTAINER',
                                    EnterContainer(robot),
                                    transitions={   'container_success':'WAIT_PERSON_CONTAINER',
                                                    'container_failed': 'WAIT_PERSON_CONTAINER'})

            # wait for a person to be seen in front of the robot
            smach.StateMachine.add( 'WAIT_PERSON_CONTAINER',
                                    WaitPersonContainer(robot),
                                    transitions={   'container_success':'LEARN_NAME_CONTAINER',
                                                    'container_failed': 'WAIT_PERSON_CONTAINER'})

            # ask the person name until it is confirmed
            smach.StateMachine.add( 'LEARN_NAME_CONTAINER',
                                    LearnNameContainer(robot, personNameDes),
                                    transitions={   'container_failed':'SAY_COULD_NOT_LEARN_NAME',
                                                    'container_success':'SAY_HELLO'})

            smach.StateMachine.add( 'SAY_COULD_NOT_LEARN_NAME',
                                    smach.CBState(sayCouldNotLearnNameCB),
                                    transitions={    'spoken':'END_CHALLENGE'})

            smach.StateMachine.add( 'SAY_HELLO',
                                    smach.CBState(sayHelloCB),
                                    transitions={    'spoken':'END_CHALLENGE'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 LEARN_PERSON_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            learnPerson_container = smach.StateMachine(outcomes = ['container_success', 'container_failed'],
                                                        output_keys = ['personName_userData'])

            with learnPerson_container:



                # - - - - - - - - - - - - - - - - - - - Learn Person Face - - - - - - - - - - - - - - - - - - -

                smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                        states.Say(robot,"Please stand one meter in front of me and look at me while I learn your face.", block=False),
                                        transitions={    'spoken':'LOOK_AT_OPERATOR_2'})

                smach.StateMachine.add( 'LOOK_AT_OPERATOR_2',
                                        states_interaction.LookAtPersonInFront(robot, lookDown=True),
                                        transitions={   'succeded':'LEARN_PERSON',
                                                        'failed':'LEARN_PERSON'})

                # smach.StateMachine.add( 'TOGGLE_PERCEPTION_ON',
                #                         PersonRecStates.TogglePerceptionMode(robot, toggle_mode=True),
                #                         transitions={   'done':'LEARN_PERSON'})

                smach.StateMachine.add('LEARN_PERSON',
                                        states.LearnPerson(robot, name_designator = personNameDes),
                                        transitions={   'succeded_learning':'SAY_OPERATOR_LEARNED',
                                                        'failed_learning':'SAY_LEARN_FACE_FAILED'})

                # smach.StateMachine.add( 'TOGGLE_PERCEPTION_OFF_SUCCESS',
                #                         PersonRecStates.TogglePerceptionMode(robot, toggle_mode=False),
                #                         transitions={   'done':'SAY_OPERATOR_LEARNED'})

                # smach.StateMachine.add( 'TOGGLE_PERCEPTION_OFF_FAILED',
                #                         PersonRecStates.TogglePerceptionMode(robot, toggle_mode=False),
                #                         transitions={   'done':'SAY_LEARN_FACE_FAILED'})

                smach.StateMachine.add('SAY_LEARN_FACE_FAILED',
                                       states.Say(robot,"I could not learn your face for some reason. Let's try again.", block=False),
                                       transitions={    'spoken':'LOOK_AT_OPERATOR_2'})

                smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                       states.Say(robot,"Now i know what you look like. Please go mix with the crowd."),
                                       transitions={'spoken':   'container_success'})

            smach.StateMachine.add( 'LEARN_PERSON_CONTAINER',
                                    learnPerson_container,
                                    transitions={   'container_success':'END_CHALLENGE',
                                                    'container_failed': 'CANCEL_HEAD_GOALS_1'})

            smach.StateMachine.add( 'CANCEL_HEAD_GOALS_1',
                                    CancelHeadGoals(robot),
                                    transitions={    'done':'SAY_FAILED_LEARNING'})

            smach.StateMachine.add( 'SAY_FAILED_LEARNING',
                                    states.Say(robot,"I could not learn my operator's face. Let me try again.", block=True),
                                    transitions={    'spoken':'LEARN_PERSON_CONTAINER'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})


############################## PYTHON ENTRY POINT ##############################

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