#! /usr/bin/env python
import roslib; 
import rospy
import smach
import random
import util

from designators.designator import Designator, DesignatorResolvementError

# Say: Immediate say
# Hear: Immediate hear
# Ask: Interaction, say + hear

##########################################################################################################################################

class Say(smach.State):
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
    >>> robot.speech.speak.assert_any_call('c', 'us', 'kyle', 'default', 'excited', True)

    Most uses from tue_robocup:
    >>> s = Say(robot, 'Please put the gripper in the hand that I close', block=False)
    >>> s.execute()
    'spoken'
    >>> robot.speech.speak.assert_any_call('Please put the gripper in the hand that I close', 'us', 'kyle', 'default', 'excited', False)
    """
    def __init__(self, robot, sentence=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot
        self.sentence = sentence
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        
    def execute(self, userdata=None):
        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            sentence = random.choice(self.sentence)
        else:
            sentence = self.sentence
        self.robot.speech.speak(sentence, self.language, self.personality, self.voice, self.mood, self.block)

        return "spoken"

class SayFormatted(smach.State):
    """Say a sentence (or pick a random one from a list) that has a python format string.
    The format is filled in with the fmt_args and fmt_kwargs parameters that can also be designators.
    These are resolved every time this state is executed

    >>> from mock import MagicMock
    >>> robot = MagicMock()
    >>> robot.speech = MagicMock()
    >>> robot.speech.speak = MagicMock()
    >>> 
    >>> sf = SayFormatted(robot, ['This is a {adj} {0} that works with{1} designators too'], [Designator('test'), 'out'], {'adj':Designator("silly")})
    >>> sf.execute()
    'spoken'
    >>> 
    >>> robot.speech.speak.assert_called_with('This is a silly test that works without designators too', 'us', 'kyle', 'default', 'excited', True)

    Test 2 to check whether this is a replacement for Say
    >>> from mock import MagicMock
    >>> robot = MagicMock()
    >>> robot.speech = MagicMock()
    >>> robot.speech.speak = MagicMock()
    >>> 
    >>> sf = SayFormatted(robot, ["a", "b", "c"])
    >>> #Repeat command 50 times, every time it should succeed and return "spoken"
    >>> outcomes = [sf.execute() for i in range(50)]
    >>> assert all(outcome == "spoken" for outcome in outcomes)
    >>>
    >>> #After many calls, all options in the list will very likely have been called at least one. 
    >>> robot.speech.speak.assert_any_call('a', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('b', 'us', 'kyle', 'default', 'excited', True)
    >>> robot.speech.speak.assert_any_call('c', 'us', 'kyle', 'default', 'excited', True)"""
    def __init__(self, robot, sentence=None, fmt_args=None, fmt_kwargs=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot
        self.sentence = sentence
        self.format_args = fmt_args if fmt_args else list()
        self.format_kwargs = fmt_kwargs if fmt_kwargs else dict()
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        
    def execute(self, userdata=None):
        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            sentence = random.choice(self.sentence)
        else:
            sentence = self.sentence

        try:
            #Resolve designators if there are any in the format
            def resolve_if_desig(fmt):
                if isinstance(fmt, Designator):
                    return fmt.resolve()
                else:
                    return fmt
            concrete_format_args = map(resolve_if_desig, self.format_args)
            concrete_format_kwargs = {key:resolve_if_desig(value) for key, value in self.format_kwargs.iteritems()}
            sentence = sentence.format(*concrete_format_args, **concrete_format_kwargs)
        except DesignatorResolvementError, e:
            rospy.logerr(e)
            pass

        self.robot.speech.speak(sentence, self.language, self.personality, self.voice, self.mood, self.block)

        return "spoken"

##########################################################################################################################################

class Hear(smach.State):
    def __init__(self, robot, spec, time_out = rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["heard", "not_heard"])
        self.robot = robot
        self.time_out = time_out
        self.spec = spec

    def execute(self, userdata):
        answer = self.robot.ears.recognize(self.spec, {}, self.time_out)

        if answer:
            if answer.result:
                return "heard"
        else:
            self.robot.speech.speak("Something is wrong with my ears, please take a look!")

        return "not_heard"

class HearOptions(smach.State):
    def __init__(self, robot, options, time_out = rospy.Duration(10)):
        smach.State.__init__(self, outcomes=options.append("no_result"))
        self.robot = robot
        self.time_out = time_out
        self.options = options

    def execute(self, userdata):
        answer = self.robot.ears.recognize("<option>", {"option":self.options}, self.time_out)

        if answer:
            if answer.result:
                return answer.choices["option"]
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
            smach.StateMachine.add('SAY',  Say(self.robot, random.choice(["I will continue my task if you say continue.","Please say continue so that I can continue my task.","I will wait until you say continue."])), transitions={'spoken':'HEAR'})
            smach.StateMachine.add('HEAR', Hear(self.robot, 'continue', self.timeout), transitions={'heard':'continue','not_heard':'no_response'})

##########################################################################################################################################

if __name__ == "__main__":
    import doctest
    doctest.testmod()
