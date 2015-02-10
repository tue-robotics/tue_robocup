#! /usr/bin/env python
import roslib; 
import rospy
import smach
import random
import util

from state import State

from util.designators import Designator, DesignatorResolvementError

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
        smach.State.__init__(self, locals(), outcomes=["spoken"])
        
    def run(self, robot, sentence, language, personality, voice, mood, block):
        if not isinstance(sentence, str) and isinstance(sentence, list):
            sentence = random.choice(self.sentence)

        robot.speech.speak(sentence, language, personality, voice, mood, block)

        return "spoken"

##########################################################################################################################################

class Hear(State):
    def __init__(self, robot, spec, time_out = rospy.Duration(10)):
        smach.State.__init__(self, locals(), outcomes=["heard", "not_heard"])

    def run(self, robot, spec, time_out):
        answer = robot.ears.recognize(spec, {}, time_out)

        if answer:
            if answer.result:
                return "heard"
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "not_heard"

class HearOptions(State):
    def __init__(self, robot, options, time_out = rospy.Duration(10)):
        smach.State.__init__(self, locals(), outcomes=options.append("no_result"))

    def run(self, robot, options, time_out):
        answer = robot.ears.recognize("<option>", {"option":options}, time_out)

        if answer:
            if answer.result:
                return answer.choices["option"]
        else:
            robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "no_result"

##########################################################################################################################################

class AskContinue(smach.StateMachine):
    def __init__(self, robot, timeout=rospy.Duration(10)):
        smach.StateMachine.__init__(self, outcomes=['continue','no_response'])
        self.robot = robot
        self.timeout = timeout

        with self:
            smach.StateMachine.add('SAY',  Say(self.robot, random.choice(["I will continue my task if you say continue.","Please say continue so that I can continue my task.","I will wait until you say continue."])), transitions={'spoken':'HEAR'})
            smach.StateMachine.add('HEAR', Hear(self.robot, 'continue', self.timeout), transitions={'heard':'continue','not_heard':'no_response'})

##########################################################################################################################################

if __name__ == "__main__":
    import doctest
    doctest.testmod()
