# ROS
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty

# TU/e Robotics
from hmi_msgs.msg import QueryAction
from hmi import Client, TimeoutException
from robot_skills.robot_part import RobotPart


class Api(RobotPart):
    def __init__(self, robot_name, tf_listener, pre_hook=None, post_hook=None):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param pre_hook: callable function to execute before a query call, i.e. to set light color
        :param post_hook: callable function to execute after a query call
        """
        super(Api, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

        self.cv_bridge = CvBridge()

        client = self.create_simple_action_client('/' + robot_name + '/hmi', QueryAction)
        self._client = Client(simple_action_client=client)

        self.restart_srv = self.create_service_client('/' + robot_name + '/hmi/dragonfly_speech_recognition/restart_node', Empty)
        self._image_from_ros_publisher = rospy.Publisher('/' + robot_name + '/hmi/image', CompressedImage, queue_size=1)

    def query(self, description, grammar, target, timeout=10):
        """
        Perform a HMI query, returns a HMIResult

        :param description: text describing the query
        :param grammar: string with the grammar to load
        :param target: string identifying the target of the grammar to recognize
        :param timeout: timeout in seconds (float)
        """
        if callable(self._pre_hook):
            self._pre_hook()

        try:
            answer = self._client.query(description, grammar, target, timeout)
            self.restart_dragonfly()
        except TimeoutException as e:
            if callable(self._post_hook):
                self._post_hook()
            self.restart_dragonfly()
            raise e

        if callable(self._post_hook):
            self._post_hook()

        return answer

    def _show_image(self, msg, seconds=5.0):
        """
        Show an image on the HMI display interface

        :param msg: CompressedImage to display
        :param seconds: How many seconds you would like to display the image on the screen
        """
        msg.header.stamp = rospy.Time.from_sec(seconds)
        self._image_from_ros_publisher.publish(msg)

    def show_image(self, path_to_image, seconds=5.0):
        """
        Show an image on the HMI display interface

        :param path_to_image: Absolute path to image file
        :param seconds: How many seconds you would like to display the image on the screen
        """
        compressed_image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(cv2.imread(path_to_image))
        self._show_image(compressed_image_msg, seconds)

    def show_image_from_msg(self, msg, seconds=5.0):
        """
        Show an image on the HMI display interface

        :param msg: rgb msg
        :param seconds: How many seconds you would like to display the image on the screen
        """
        compressed_image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(self.cv_bridge.imgmsg_to_cv2(msg, "bgr8"))
        self._show_image(compressed_image_msg, seconds)

    @property
    def last_talker_id(self):
        return self._client.last_talker_id

    def old_query(self, spec, choices, timeout=10):
        msg = 'robot.ears.recognize IS REMOVED. Use `robot.hmi.query`'
        rospy.logerr(msg)
        raise Exception(msg)

    def reset(self):
        self.restart_dragonfly()
        return True

    def restart_dragonfly(self):
        try:
            self.restart_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))


