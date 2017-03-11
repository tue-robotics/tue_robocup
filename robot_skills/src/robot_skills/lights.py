#! /usr/bin/env python
import rospy
from std_msgs.msg import ColorRGBA

from amigo_msgs.msg import RGBLightCommand
from robot_part import RobotPart


class Lights(RobotPart):
    """
    Interface to amigo's lights.
    """

    def __init__(self, robot_name, tf_listener):
        super(Lights, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._topic = rospy.Publisher('/'+robot_name+'/rgb_lights_manager/user_set_rgb_lights', RGBLightCommand, queue_size=10)

    def close(self):
        pass

    def set_color(self, r,g,b,a=1.0):
        rgb_msg = RGBLightCommand(color=ColorRGBA(r,g,b,a))
        rgb_msg.show_color.data = True
        self._topic.publish(rgb_msg)

    def reset(self):
        rgb_msg = RGBLightCommand(color=ColorRGBA(0.0,0.0,1.0,1.0))
        rgb_msg.show_color.data = True
        self._topic.publish(rgb_msg)

    def on(self):
        rgb_msg = RGBLightCommand(show_color=True)
        self._topic.publish(rgb_msg)

    def off(self):
        rgb_msg = RGBLightCommand(show_color=False)
        self._topic.publish(rgb_msg)

    def start_sinus(self): pass
