#! /usr/bin/env python
import rospy
from std_msgs.msg import Bool


class EButton:
    """
    Interface to amigo emergency switch. Listens to /emergency_switch topic
    """

    def __init__(self):

        rospy.loginfo("Initializing ebutton listener")
        self._ebuttonstatus = True
        self._topic = rospy.Subscriber("/amigo/emergency_switch",
                                         Bool,
                                         self._listen)

    def close(self):
        pass

    def _listen(self, s):
        """
        Callback methods that listens to /emergency_switch
        """
        #rospy.loginfo("Received ebutton status")
        self._ebuttonstatus = s.data

    def read_ebutton(self):
        #rospy.loginfo("Returning ebuttonstatus")
        return self._ebuttonstatus


if __name__ == "__main__":
    rospy.init_node('amigo_ebutton_executioner', anonymous=True)
    ebutton = EButton()
