# ROS
import rospy
from std_msgs.msg import ColorRGBA
# TU/e Robotics
from tue_msgs.msg import RGBLightCommand

from robot_skills.robot_part import RobotPart

LISTENING = ColorRGBA(0, 1, 0, 1)
SPEAKING = ColorRGBA(1, 0, 0, 1)
RESET = ColorRGBA(0, 0, 1, 1)


class LightsInterface(RobotPart):
    def __init__(self, robot_name, tf_buffer):
        """
        Interface to the robot's lights. To use this, a deriving class needs to be defined that implements _send_color_msg.

        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        """
        super(LightsInterface, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)

    def close(self):
        pass

    def set_color(self, r, g, b, a=1.0):
        """
        Set the color of the lights of the robot in RGBA values

        :param r: red value 0.0-1.0
        :param g: green value 0.0-1.0
        :param b: blue value 0.0-1.0
        :param a: alpha value 0.0-1.0
        :return: no return
        """
        self.set_color_rgba_msg(ColorRGBA(r, g, b, a))

    def set_color_rgba_msg(self, rgba):
        """
        Set the color of the robot by a std_msgs.msg.ColorRGBA

        :param rgba: std_msgs.msg.ColorRGBA
        :return: no return
        """
        self._send_color_msg(rgba_msg=rgba)

    def _send_color_msg(self, rgba_msg):
        """
        Sends the color message to the robot hardware. This function needs to be implemented by deriving classes.

        :param rgba_msg: message to send
        """
        # ToDo: replace by fstring (after going to Python3)
        raise NotImplementedError("_send_color_msg is not implemented for {}".format(self.__class__.__name__))

    def selfreset(self):
        """
        Set the lights to blue

        :return: no return
        """
        self.set_color_rgba_msg(RESET)
        return True


class TueLights(LightsInterface):
    def __init__(self, robot_name, tf_buffer):
        """
        Interface to the robot's lights. This uses the TU/e-specific RGBLightCommand message type.

        :param robot_name: robot_name
        :param tf_buffer: tf_server.TFClient()
        """
        super(TueLights, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._publisher = rospy.Publisher(
            '/{}/rgb_lights_manager/user_set_rgb_lights'.format(robot_name), RGBLightCommand, queue_size=10
        )

    def _send_color_msg(self, rgba_msg):
        """
        Sends the color message to the robot hardware. This uses the RGBLightCommand message

        :param rgba_msg: message to send
        """
        rgb_msg = RGBLightCommand(color=rgba)
        rgb_msg.show_color.data = True
        self._publisher.publish(rgb_msg)

    def taste_the_rainbow(self, duration=5.0):
        """
        Show awesome rainbow on the real amigo robot

        :param duration: (float) Indicates the total duration of the rainbow
        """

        # red: \_
        # green: /\
        # blue: _/

        def red(t):
            if t < duration / 2.0:
                rainbowr = 1.0 - (t / (duration / 2.0))
            else:
                rainbowr = 0.0
            return rainbowr

        def green(t):
            if t < duration / 2.0:
                rainbowg = (t / (duration / 2.0))
            else:
                rainbowg = 2 - (t / (duration / 2.0))
            return rainbowg

        def blue(t):
            if t < duration / 2.0:
                rainbowb = 0.0
            else:
                rainbowb = -1.0 + (t / (duration / 2.0))
            return rainbowb

        t_start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20.0)
        while (rospy.Time.now().to_sec() - t_start) < duration:
            time_after_start = rospy.Time.now().to_sec() - t_start
            r, g, b = (red(time_after_start), green(time_after_start), blue(time_after_start))
            self.set_color(r, g, b)
            rate.sleep()


class Lights(LightsInterface):
    def __init__(self, robot_name, tf_buffer, topic):
        """
        Interface to the robot's lights. This uses the TU/e-specific RGBLightCommand message type.

        :param robot_name: robot_name
        :param tf_buffer: tf_server.TFClient()
        :param topic: topic where to publish the messages
        """
        super(Lights, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._publisher = rospy.Publisher(topic, ColorRGBA, queue_size=1)

    def _send_color_msg(self, rgba_msg):
        """
        Sends the color message to the robot hardware. This uses the RGBLightCommand message

        :param rgba_msg: message to send
        """
        self._publisher.publish(rgba_msg)
