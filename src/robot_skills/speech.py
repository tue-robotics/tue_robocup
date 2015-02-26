#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
from text_to_speech.srv import Speak
from std_msgs.msg import String
from math import sqrt

import thread

class Speech(object):
    """Interface to TTS-module"""

    def __init__(self, robot_name, wait_service=True, pre_hook=None, post_hook=None):
        if wait_service:
            rospy.loginfo("Waiting for service /%s/text_to_speech/speak"%robot_name)
            rospy.wait_for_service('/%s/text_to_speech/speak'%robot_name, timeout=2)
        self._speech_service = rospy.ServiceProxy('/%s/text_to_speech/speak'%robot_name, Speak)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

    def close(self):
        pass

    def speak(self, sentence, language="us", personality="kyle", voice="default", mood="excited", block=True, replace={"_":" "}):
        """
        Send a sentence to the text to speech module.
        You can set language, personality, voice and mood for the Phiips TTS.
        When block=False, this method returns immediatly.
        With the replace-dictionary, you can specify which characters to replace with what. By default, it replace underscores with spaces.
        """

        if hasattr(self._pre_hook, '__call__'):
            self._pre_hook()

        for orig, replacement in replace.iteritems():
            sentence = sentence.replace(orig, replacement)

        result = False
        try:
            if language == 'nl' and not (personality in ['david', 'marjolein']):
                personality = 'david' #kyle doesn't work for NL
            rospy.loginfo("\x1b[1;32m'"+ sentence + "'\x1b[0m") #The funny stuff around sentence is for coloring the output text in the console

            resp1 = self._speech_service(language, personality, voice, mood, sentence, block)
            result = resp1.error_msg == ""

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            result = False

        if hasattr(self._post_hook, '__call__'):
            self._post_hook()

        return result

if __name__ == "__main__":
    rospy.init_node("amigo_speech_executioner", anonymous=True)
    speech = Speech("amigo")
