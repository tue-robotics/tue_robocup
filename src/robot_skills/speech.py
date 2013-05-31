#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
from text_to_speech_philips.srv import Speak
from std_msgs.msg import String
from math import sqrt

import thread

from speech_interpreter.srv import GetAction, GetInfo # for speech_to_text only

class Speech(object):
    """Interface to Amigo's tts-module"""

    def __init__(self, wait_service=True):
        self.amigo_speak_up = rospy.Publisher("/text_to_speech/input", String)
        self.amigo_speak_up_info = rospy.Publisher("/amigo_speak_up_info", String)
        self.pub_amigo_speech_sim = rospy.Publisher("/amigo_speech_sim", String)    # For using amigo's speech in simulation

        if wait_service:
            rospy.loginfo("Waiting for service amigo_speakup_advanced")
            rospy.wait_for_service('/text_to_speech/speak', timeout=2)
        self.speech_service = rospy.ServiceProxy('/text_to_speech/speak', Speak)

        """Interface to Amigo's stt-module
           To do: better integration with other Speech code"""
        if wait_service:
            rospy.loginfo("Waiting for service /speech_interpreter/get_info_user and /speech_interpreter/get_action_user.")

            try:
                rospy.wait_for_service('interpreter/get_action_user', timeout = 2)
                rospy.wait_for_service('interpreter/get_info_user', timeout = 2)
            except rospy.ROSException:
                rospy.logwarn("Please roslaunch create_speech_files speech.launch \& rosrun speech_interpreter interpreter to start the speech recognizer and speech interpretr.")

        self.get_info_service = rospy.ServiceProxy('speech_interpreter/get_info_user', GetInfo)
        self.get_action_service = rospy.ServiceProxy('speech_interpreter/get_action_user', GetAction)

    def close(self):
        pass

    def speak(self, sentence, language="us", personality="kyle", character="default", mood="excited", block=True):
        """
        Send a sentence to amigo's text to speech module
        """
        try:
            if language == 'nl' and not (personality in ['david', 'marjolein']):
                personality = 'david' #kyle doesn't work for NL
            rospy.loginfo("\x1b[1;32m'"+ sentence + "'\x1b[0m") #The funny stuff around sentence is for coloring the output text in the console
            
            # Also send the sentence over a topic (for simulation purposes)
            self.pub_amigo_speech_sim.publish(sentence)

            resp1 = self.speech_service(language, personality, character, mood, sentence, block)
            return resp1.error_msg == ""

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            return False

    def _amigo_speak(self, sentence, wait_time=1.0):
        self.amigo_speak_up.publish(sentence)
        rospy.loginfo("AMIGO says: '{0}'".format(sentence))
        # wait for a certain amount of time, based on the length of the spoken sentence
        if wait_time == "auto" or wait_time == None:
            delay = sqrt(len(sentence))
            rospy.sleep(delay)
        else:
            rospy.sleep(wait_time)
        return True

    def speak_info(self, sentence):

        self.amigo_speak_up_info.publish(sentence)
        rospy.loginfo("AMIGO says: '{0}'".format(sentence))
        return True

    def get_info(self, info_type = 'name', n_tries = 1, time_out = 10.0):
        """ function to get info from the user, info_type could be name or object categories or location categories """

        if info_type not in ['name', 'drink', 'snack', 'food', 'bathroomstuff', 'seat', 'table', 'shelf', 'appliance', 'bin']:
            rospy.logerr("Wrong info_type!")
            return False
        else:
            resp = self.get_info_service(info_type, n_tries, time_out)

        if resp.answer == 'no_answer':
            rospy.logwarn("Time out before the answer is given.")

        return resp.answer

    def get_action(self, time_out = 100.0):
        """ function to get the goal of the task - does not work yet
            To do: make it work """
        resp = self.get_action_service(time_out)

        # can find the actions from speech_interpreter/src/Interpreter.cpp
        if resp.action == 'transport':
            rospy.loginfo("The goal is to tranport the '{0}' to '{1}'.".format(resp.object, resp.end_location))
        elif resp.action == 'get':
            rospy.loginfo("The goal is to get the '{0}' from the '{1}'".format(resp.object, resp.start_location))
        elif resp.action == 'point':
            rospy.loginfo("The goal is to point at '{0}'".format(resp.object))
        elif resp.action == 'find':
            rospy.loginfo("The goal is to find the '{0}'".format(resp.object))
        elif resp.action == 'leave':
            rospy.loginfo("The goal is to go to '{0}'".format(resp.end_location))
        else:
            rospy.logwarn("Action is not defined")

        goal = resp
        return goal

    @staticmethod
    def buildList(items, joinLast='and'): #other options are "or"
        items = list(items)
        a = items[:-1]
        b = items[-1]

        conj = ", ".join(a) + " " + joinLast + " " + b
        return conj

if __name__ == "__main__":
    rospy.init_node("amigo_speech_executioner", anonymous=True)
    speech = Speech()
