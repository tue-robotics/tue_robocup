# ROS
import rospy
from std_msgs.msg import ColorRGBA

# TU/e Robotics
from amigo_msgs.msg import RGBLightCommand
from robot_part import RobotPart

LISTENING = ColorRGBA(0, 1, 0, 1)
SPEAKING = ColorRGBA(1, 0, 0, 1)
RESET = ColorRGBA(0, 0, 1, 1)


class Lights(RobotPart):
    """
    Interface to amigo's lights.
    """
    def __init__(self, robot_name, tf_listener):
        """
        constructor
        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        """
        super(Lights, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._topic = rospy.Publisher('/'+robot_name+'/rgb_lights_manager/user_set_rgb_lights', RGBLightCommand,
                                      queue_size=10)

    def close(self):
        pass

    def set_color(self, r, g, b, a=1.0):
        # type: (object, object, object, object) -> object
        """
        Set the color of the lights of the robot in RGBA values
        :param r: red value 0.0-1.0
        :param g: green value 0.0-1.0
        :param b: blue value 0.0-1.0
        :param a: alpha value 0.0-1.0
        :return: no return
        """
        self.set_color_colorRGBA(ColorRGBA(r, g, b, a))

    def set_color_colorRGBA(self, rgba):
        """
        Set the color of the robot by a std_msgs.msg.ColorRGBA
        :param rgba: std_msgs.msg.ColorRGBA
        :return: no return
        """
        rgb_msg = RGBLightCommand(color=rgba)
        rgb_msg.show_color.data = True
        self._topic.publish(rgb_msg)

    def reset(self):
        """
        Set the lights to blue
        :return: no return
        """
        self.set_color_colorRGBA(RESET)

    def on(self):
        """
        Set the lights of the robot ON
        :return: no return
        """
        rgb_msg = RGBLightCommand(show_color=True)
        self._topic.publish(rgb_msg)

    def off(self):
        """
        Set the lights of the robot OFF
        :return: no return
        """
        rgb_msg = RGBLightCommand(show_color=False)
        self._topic.publish(rgb_msg)

    def taste_the_rainbow(self):
        """ Show awesome rainbow on the real amigo robot
        """

        # rood: \_
        # groen: /\
        # blauw: _/

        def red(time_after_start):
            if time_after_start < total_time/2:
                rainbowr = 1.0 - (time_after_start/(total_time/2))
            else:
                rainbowr = 0.0
            return rainbowr

        def green(time_after_start):
            if time_after_start < total_time/2:
                rainbowg = (time_after_start/(total_time/2))
            else:
                rainbowg = 2 - (time_after_start/(total_time/2))
            return rainbowg

        def blue(time_after_start):
            if time_after_start < total_time / 2:
                rainbowb = 0.0
            else:
                rainbowb = -1 + (time_after_start / (total_time/2))
            return rainbowb

        import time
        total_time = 5.0
        t_start = time.time()
        while time.time() - t_start < total_time:
            time_after_start = time.time() - t_start
            r, g, b = (red(time_after_start), green(time_after_start), blue(time_after_start))
            self.set_color(r, g, b)
            print r, g, b
            time.sleep(0.02)
