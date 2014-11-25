#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

from dragonfly_speech_recognition.srv import *


## TODO: some words are appended like 'applejuice', this should be handled gracefully in some way
## by the executive
class Ears:
    """
    Interface to amigo Ears. 
    Works with dragonfly speech recognition (as of december 2014)

    Function listen explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    """

    def __init__(self,entries=30):
        
        self.speech_recognition_service = rospy.ServiceProxy('dragonfly_speech_recognition/recognize_speech', GetSpeech)

    #Function listens explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    def listen(self, spec, choices = [], timeout = rospy.Duration(10)):
        req = GetSpeechRequest()
        req.spec = spec
        req.choices = choices
        req.time_out = timeout

        answer = None

        try:
            resp = self.speech_recognition_service(req)
            if resp:
                answer.result = resp.result
                for i in range(len(resp.choices)):
                    answer.choices[resp.choices[i].id] = resp.choices[i].values[0]
        except rospy.ServiceException as e:
            rospy.logerr("Could not connect to dragonfly_speech_recognition: " + str(e))

        return answer

if __name__ == "__main__":
    rospy.init_node('amigo_ears_executioner', anonymous=True)
    ears = Ears()