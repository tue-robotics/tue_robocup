#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

from dragonfly_speech_recognition.srv import GetSpeech, GetSpeechRequest
from dragonfly_speech_recognition.msg import Choice
from amigo_msgs.msg import RGBLightCommand
from std_msgs.msg import ColorRGBA


class Ears:
    """
    Interface to amigo Ears. 
    Works with dragonfly speech recognition (as of december 2014)

    Function listen explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    """

    def __init__(self, robot_name):
        self.get_speech_client_service = rospy.ServiceProxy("/%s/speech_client/get_speech"%robot_name, GetSpeech)
        self._topic = rospy.Publisher('user_set_rgb_lights', RGBLightCommand)

    # I know that the lights are not ears, but these lights should be as close to the source as possible. 
    # Amigo must turn green when listening to user.
    def set_color(self, r,g,b,a=1.0):
        rgb_msg = RGBLightCommand(color=ColorRGBA(r,g,b,a))
        rgb_msg.show_color.data = True
        self._topic.publish(rgb_msg)

    #Function listens explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    def recognize(self, spec, choices={}, time_out = rospy.Duration(10)):
        req = GetSpeechRequest()
        req.spec = spec
        req.choices = [ Choice(id=k, values=v) for k, v in choices.iteritems() ]
        req.time_out = time_out

        answer = None

        try:
            self.set_color(0, 255, 0) #green
            answer = self.get_speech_client_service(req)
            self.set_color(0, 0, 255) #blue
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

