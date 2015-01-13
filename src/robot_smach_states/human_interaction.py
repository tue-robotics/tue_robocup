#! /usr/bin/env python
import roslib; 
import rospy
import smach
import random
import util

# Say: Immediate say
# Hear: Immediate hear
# Ask: Interaction, say + hear

##########################################################################################################################################

class Say(smach.State):
    def __init__(self, robot, sentence=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot
        self.sentence = sentence
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        
    def execute(self, userdata):
        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            sentence = random.choice(self.sentence)
        else:
            sentence = self.sentence
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