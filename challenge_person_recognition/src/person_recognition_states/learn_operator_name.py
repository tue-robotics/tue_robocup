#! /usr/bin/env python

import rospy
import smach
import robot_smach_states as states

from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup

import robot_smach_states.util.designators as ds
from dragonfly_speech_recognition.srv import GetSpeechResponse

common_knowledge = load_knowledge("common")


class AskPersonName(smach.State):
    def __init__(self, robot, operatorNameDes):
        smach.State.__init__(   self,
                                outcomes=['succeeded', 'failed'])

        self.robot = robot

        ds.is_writeable(operatorNameDes)
        self.operatorNameDes = operatorNameDes

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What is your name?", block=True)

        spec = ds.Designator("((<prefix> <name>)|<name>)")

        choices = ds.Designator({"name": common_knowledge.names, "prefix": ["My name is", "I'm called", "I am"]})

        answer = ds.VariableDesignator(resolve_type=GetSpeechResponse)

        state = states.HearOptionsExtra(self.robot, spec, choices, answer.writeable)
        outcome = state.execute()

        if not outcome == "heard":
            name = "Mister Operator"
            self.operatorNameDes.write(name)

            rospy.logwarn("Speech recognition outcome was not successful (outcome: '{0}'). Using default name '{1}'".
                          format(str(outcome), self.operatorNameDes.resolve()))
            return 'failed'
        else:
            try:
                name = answer.resolve().choices["name"]
                self.operatorNameDes.write(name)

                rospy.loginfo("Result received from speech recognition is '" + name + "'")
            except KeyError, ke:
                rospy.loginfo("KeyError resolving the name heard: " + str(ke))
                pass

        return 'succeeded'


class LearnOperatorName(smach.StateMachine):
    def __init__(self, robot, operatorNameDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        ds.check_resolve_type(operatorNameDes, str)
        ds.is_writeable(operatorNameDes)
        self.operatorNameDes = operatorNameDes

        @smach.cb_interface(outcomes=['spoken'])
        def say_could_not_learn_name(userdata):
            robot.head.look_at_standing_person()
            robot.speech.speak( "Sorry but I could not understand your name. I will just call you " + operatorNameDes.resolve(), block=False)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def say_is_your_name(userdata):
            robot.speech.speak( "I heard " + operatorNameDes.resolve() + ". Is this correct?", block=True)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def say_hello(userdata):
            robot.speech.speak( "Hello " + operatorNameDes.resolve() + "!", block=False)
            return 'spoken'

        @smach.cb_interface(outcomes=['done'])
        def lookat_standing_person(userdata):
            robot.head.look_at_standing_person()
            return 'done'

        with self:
                smach.StateMachine.add("SAY_WAITING_OPERATOR", states.Say(robot, ["I'm waiting for the operator to stand in front of me.",
                                                                                  "I need an operator, please stand in front of me."],
                                                                          block=False), transitions={   'spoken': 'LOOK_AT_OPERATOR'})

                smach.StateMachine.add( 'LOOK_AT_OPERATOR',
                                        smach.CBState(lookat_standing_person),
                                        transitions={    'done':'LEARN_NAME_ITERATOR'})

                learnNameIterator = smach.Iterator( outcomes=['container_success', 'container_failed'],
                                                    it = lambda:range(0, 3),
                                                    it_label='counter',
                                                    input_keys=[],
                                                    output_keys=['personName_userData'],
                                                    exhausted_outcome = 'container_failed')

                with learnNameIterator:

                    learnNameContainer = smach.StateMachine(output_keys=['personName_userData'],
                                                            outcomes = ['container_success', 'container_failed'])
                    with learnNameContainer:

                        # initialize personName_userData
                        learnNameContainer.userdata.personName_userData = ""

                        smach.StateMachine.add( 'ASK_PERSON_NAME',
                                                AskPersonName(robot, operatorNameDes),
                                                remapping={     'personName_out':'personName_userData'},
                                                transitions={   'succeeded':'SAY_IS_YOUR_NAME',
                                                                'failed':'SAY_HEAR_FAILED'})

                        smach.StateMachine.add( 'SAY_IS_YOUR_NAME',
                                                smach.CBState(say_is_your_name),
                                                transitions={    'spoken':'HEAR_YES_NO_1'})

                        smach.StateMachine.add( 'HEAR_YES_NO_1',
                                                states.HearYesNo(robot),
                                                transitions={   'heard_yes' : 'container_success',
                                                                'heard_no' : 'SAY_NAME_WRONG',
                                                                'heard_failed' : 'SAY_HEAR_FAILED'})

                        smach.StateMachine.add( 'SAY_NAME_WRONG',
                                                states.Say(robot, [  "Sorry, I understood wrong.",
                                                                    "Oh I'm sorry."], block=False),
                                                transitions={    'spoken':'container_failed'})

                        smach.StateMachine.add( 'SAY_HEAR_FAILED',
                                                states.Say(robot, [  "I did not understand your name.",
                                                                    "I didn't listen correctly."], block=False),
                                                transitions={    'spoken':'container_failed'})

                    smach.Iterator.set_contained_state( 'LEARN_NAME_CONTAINER',
                                                        learnNameContainer,
                                                        # loop_outcomes=['container_failed'],
                                                        break_outcomes=['container_success'])

                smach.StateMachine.add( 'LEARN_NAME_ITERATOR',
                                        learnNameIterator,
                                        transitions = { 'container_failed':'SAY_COULD_NOT_LEARN_NAME',
                                                        'container_success':'SAY_HELLO'})

                # ----------------------------------------

                smach.StateMachine.add( 'SAY_COULD_NOT_LEARN_NAME',
                                        smach.CBState(say_could_not_learn_name),
                                        transitions={    'spoken':'failed'})

                smach.StateMachine.add( 'SAY_HELLO',
                                        smach.CBState(say_hello),
                                        transitions={    'spoken':'succeeded'})
