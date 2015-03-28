#! /usr/bin/env python
import roslib;
import rospy
import smach
import random

from robot_smach_states.state import State

from robot_smach_states.util.designators import Designator, DesignatorResolvementError, EdEntityDesignator, check_type
from robot_smach_states.utility import WaitForDesignator
import robot_skills.util.msg_constructors as gm


# Say: Immediate say
# Hear: Immediate hear
# Ask: Interaction, say + hear

##########################################################################################################################################

class Say(State):
    """Say a sentence or pick a random one from a list.

    >>> from mock import MagicMock
    >>> robot = MagicMock()
    >>> robot.speech = MagicMock()
    >>> robot.speech.speak = MagicMock()
    >>>
    >>> sf = Say(robot, ["a", "b", "c"])
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes = [sf.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes)
    >>>
    >>> #After many calls, all options in the list will very likely have been called at least one.
    >>> robot.speech.speak.assert_any_call('a', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('b', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('c', 'us', 'kyle', 'default', 'excited', True)"""
    def __init__(self, robot, sentence=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        check_type(sentence, str, list)
        check_type(sentence, str, list)
        check_type(language, str)
        check_type(personality, str)
        check_type(voice, str)
        check_type(mood, str)
        check_type(block, bool)

        State.__init__(self, locals(), outcomes=["spoken"])

    def run(self, robot, sentence, language, personality, voice, mood, block):
        robot.head.look_at_standing_person()

        if not isinstance(sentence, str) and isinstance(sentence, list):
            sentence = random.choice(sentence)

        robot.speech.speak(sentence, language, personality, voice, mood, block)

        robot.head.cancel_goal()

        return "spoken"

##########################################################################################################################################

class Hear(State):
    def __init__(self, robot, spec, time_out = rospy.Duration(10)):
        State.__init__(self, locals(), outcomes=["heard", "not_heard"])

    def run(self, robot, spec, time_out):
        robot.head.look_at_standing_person()

        answer = robot.ears.recognize(spec, {}, time_out)

        robot.head.cancel_goal()

        if answer:
            if answer.result:
                return "heard"
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "not_heard"

class HearOptions(State):
    def __init__(self, robot, options, time_out = rospy.Duration(10)):
        State.__init__(self, locals(), outcomes=options.append("no_result"))

    def run(self, robot, options, time_out):
        robot.head.look_at_standing_person()

        answer = robot.ears.recognize("<option>", {"option":options}, time_out)

        robot.head.cancel_goal()

        if answer:
            if answer.result:
                return answer.choices["option"]
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"

class HearOptionsExtra(smach.State):
    """
    >>> from robot_skills.mockbot import Mockbot
    >>> mockbot = Mockbot()
    >>> from robot_smach_states.util.designators import Designator, VariableDesignator
    >>> spec = Designator("I will go to the <table> in the <room>")
    >>> choices = Designator({  "room"  : ["livingroom", "bedroom", "kitchen" ], "table" : ["dinner table", "couch table", "desk"]})
    >>> answer = VariableDesignator()
    >>> state = HearOptionsExtra(mockbot, spec, choices, answer)
    >>> outcome = state.execute()
    """
    def __init__(self, robot, spec_designator,
                        choices_designator,
                        speech_result_designator,
                        time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot
        self.spec_designator = spec_designator
        self.choices_designator = choices_designator
        self.speech_result_designator = speech_result_designator
        self.time_out = time_out

    def execute(self, userdata=None):
        spec = self.spec_designator.resolve()
        choices = self.choices_designator.resolve()

        self.robot.head.look_at_standing_person()

        answer = self.robot.ears.recognize(spec, choices, self.time_out)

        self.robot.head.cancel_goal()

        if answer:
            if answer.result:
                self.speech_result_designator.current = answer
                return "heard"
        else:
            self.robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"


##########################################################################################################################################

class AskContinue(smach.StateMachine):
    def __init__(self, robot, timeout=rospy.Duration(10)):
        smach.StateMachine.__init__(self, outcomes=['continue','no_response'])
        self.robot = robot
        self.timeout = timeout

        with self:
            smach.StateMachine.add('SAY',
                                    Say(self.robot,
                                        random.choice(["I will continue my task if you say continue.","Please say continue so that I can continue my task.","I will wait until you say continue."])),
                                    transitions={'spoken':'HEAR'})

            smach.StateMachine.add('HEAR',
                                    Hear(self.robot, 'continue', self.timeout),
                                    transitions={'heard':'continue','not_heard':'no_response'})

##########################################################################################################################################

class WaitForPersonInFront(WaitForDesignator):
    """
    Waits for a person to be found in fron of the robot. Attempts to wait a number of times with a sleep interval
    """

    def __init__(self, robot, attempts = 1, sleep_interval = 1):
        # TODO: add center_point in front of the robot and radius of the search on EdEntityDesignator
        # human_entity = EdEntityDesignator(robot, center_point=gm.PointStamped(x=1.0, frame_id="base_link"), radius=1, id="human")
        human_entity = EdEntityDesignator(robot, type="human")
        WaitForDesignator.__init__(self, robot, human_entity, attempts, sleep_interval)



##########################################################################################################################################

if __name__ == "__main__":
    import doctest
    doctest.testmod()
