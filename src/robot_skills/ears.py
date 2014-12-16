#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

from dragonfly_speech_recognition.srv import GetSpeech, GetSpeechRequest
from dragonfly_speech_recognition.msg import Choice

class Ears:
    """
    Interface to amigo Ears. 
    Works with dragonfly speech recognition (as of december 2014)

    Function listen explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    """

    def __init__(self, robot_name):
        self.get_speech_client_service = rospy.ServiceProxy("/%s/speech_client/get_speech"%robot_name, GetSpeech)

    #Function listens explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    def recognize(self, spec, choices={}, time_out = rospy.Duration(10)):
        req = GetSpeechRequest()
        req.spec = spec
        req.choices = [ Choice(id=k, values=v) for k, v in choices.iteritems() ]
        req.time_out = time_out

        answer = None

        try:
            answer = self.get_speech_client_service(req)
            if answer:
                answer.choices = dict((x.id, x.values[0]) for x in answer.choices)
        except rospy.ServiceException as e:
            rospy.logerr("Service exeption: %s"%e)
        except:
            rospy.logerr("Something else went wrong, please notify Rein")

        return answer

if __name__ == "__main__":
    rospy.init_node('robot_ears', anonymous=True)
    ears = Ears("amigo")

