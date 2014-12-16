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
        
    def execute(self):
        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            sentence = random.choice(self.sentence)
        else:
            sentence = self.sentence
        self.robot.speech.speak(sentence, self.language, self.personality, self.voice, self.mood, self.block)

        return "spoken"

##########################################################################################################################################






##########################################################################################################################################
    
class AskContinue(smach.State):
    def __init__(self, robot, timeout=10):
        smach.State.__init__(self, outcomes=["continue", "no_response"])
        self.robot = robot

    def execute(self):
        return "yes"

class AskYesNo(smach.State):
    def __init__(self, robot, timeout=10):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self.robot = robot

    def execute(self):
        return "yes"

##########################################################################################################################################