#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
from text_to_speech.srv import Speak
from std_msgs.msg import String
from math import sqrt

import thread

from speech_interpreter.srv import AskUser # for speech_to_text only


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
            rospy.loginfo("Waiting for service /speech_interpreter/ask_user and /speech_interpreter/get_action_user.")

            try:
                rospy.wait_for_service('interpreter/ask_user', timeout = 2)
                rospy.wait_for_service('interpreter/get_info_user', timeout = 2)
            except rospy.ROSException:
                rospy.logwarn("Speech interpreter services are not running! Start interpreter!")

        self.ask_user_service_get_info = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        self.ask_user_service_get_action = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def close(self):
        pass

    def speak(self, sentence, language="us", personality="kyle", voice="default", mood="excited", block=True, replace={"_":" "}):
        """
        Send a sentence to amigo's text to speech module. 
        You can set language, personality, voice and mood for the Phiips TTS.
        When block=False, this method returns immediatly.
        With the replace-dictionary, you can specify which characters to replace with what. By default, it replace underscores with spaces.
        """

        for orig, replacement in replace.iteritems():
            sentence = sentence.replace(orig, replacement)

        try:
            if language == 'nl' and not (personality in ['david', 'marjolein']):
                personality = 'david' #kyle doesn't work for NL
            rospy.loginfo("\x1b[1;32m'"+ sentence + "'\x1b[0m") #The funny stuff around sentence is for coloring the output text in the console
            
            # Also send the sentence over a topic (for simulation purposes)
            self.pub_amigo_speech_sim.publish(sentence)

            resp1 = self.speech_service(language, personality, voice, mood, sentence, block)
            return resp1.error_msg == ""

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            return False

    def _amigo_speak(self, sentence, timeout=1.0):
        self.amigo_speak_up.publish(sentence)
        rospy.loginfo("AMIGO says: '{0}'".format(sentence))
        # wait for a certain amount of time, based on the length of the spoken sentence
        if timeout == "auto" or timeout == None:
            delay = sqrt(len(sentence))
            rospy.sleep(delay)
        else:
            rospy.sleep(timeout)
        return True

    def speak_info(self, sentence):

        self.amigo_speak_up_info.publish(sentence)
        rospy.loginfo("AMIGO says: '{0}'".format(sentence))
        return True

    def get_info(self, info_type = 'name', n_tries = 1, timeout = rospy.Duration(10.0)):
        """ function to get info from the user, info_type could be name or object categories or location categories """

        if info_type not in ['name', 'drink', 'snack', 'food', 'bathroomstuff', 'seat', 'table', 'shelf', 'appliance', 'bin']:
            rospy.logerr("Wrong info_type!")
            return False
        else:
            self.response = self.ask_user_service_get_info(info_type, n_tries, timeout)

            for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "answer":
                    response_answer = self.response.values[x]

        if response_answer == 'no_answer':
            rospy.logwarn("Time out before the answer is given.")

        return response_answer

    def get_action(self, timeout = 100.0):
        """ function to get the goal of the task - does not work yet
            To do: make it work """
        self.response = self.ask_user_service_get_action("action", 1 , rospy.Duration(timeout))  

        for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "action":
                    response_action = self.response.values[x]
                elif self.response.keys[x] == "start_location":
                    response_start_location = self.response.values[x]
                elif self.response.keys[x] == "end_location":
                    response_end_location = self.response.values[x]
                elif self.response.keys[x] == "object":
                    response_object = self.response.values[x]
                elif self.response.keys[x] == "object_room":
                    response_object_room = self.response.values[x]
                elif self.response.keys[x] == "object_location":
                    response_object_location = self.response.values[x]

        # can find the actions from speech_interpreter/src/Interpreter.cpp
        if response_action == 'transport':
            rospy.loginfo("The goal is to tranport the '{0}' to '{1}'.".format(response_object, response_end_location))
        elif response_action == 'get':
            rospy.loginfo("The goal is to get the '{0}' from the '{1}'".format(response_object, response_start_location))
        elif response_action == 'point':
            rospy.loginfo("The goal is to point at '{0}'".format(response_object))
        elif response_action == 'find':
            rospy.loginfo("The goal is to find the '{0}'".format(response_object))
        elif response_action == 'leave':
            rospy.loginfo("The goal is to go to '{0}'".format(response_end_location))
        else:
            rospy.logwarn("Action is not defined")

        return self.response

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
