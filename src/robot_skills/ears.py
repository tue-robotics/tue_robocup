#! /usr/bin/env python
import rospy
from dragonfly_speech_recognition.msg import Choice
from dragonfly_speech_recognition.srv import GetSpeech, GetSpeechRequest


class Ears:
    """
    Interface to amigo Ears.
    Works with dragonfly speech recognition (as of december 2014)

    Function listen explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    """

    def __init__(self, robot_name, pre_hook=None, post_hook=None):
        self._get_speech_client_service = rospy.ServiceProxy("/%s/speech_client/get_speech"%robot_name, GetSpeech)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

    #Function listens explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    def recognize(self, spec, choices={}, time_out = rospy.Duration(10)):
        req = GetSpeechRequest()
        req.spec = spec
        req.choices = [ Choice(id=k, values=v) for k, v in choices.iteritems() ]
        req.time_out = time_out

        if hasattr(self._pre_hook, '__call__'):
            self._pre_hook()

        answer = None

        try:
            answer = self._get_speech_client_service(req)
            rospy.loginfo("Robot heard \x1b[1;42m'{}'\x1b[0m".format(answer.result)) #The funny characters color the background

            if answer:
                answer.choices = dict((x.id, x.values[0]) for x in answer.choices)
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: %s"%e)

        if hasattr(self._post_hook, '__call__'):
            self._post_hook()

        return answer

if __name__ == "__main__":
    rospy.init_node('robot_ears', anonymous=True)
    ears = Ears("amigo")
