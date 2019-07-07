import sys
import cv2
from threading import Event
from collections import OrderedDict

# ROS
import rospy
import smach
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

from telegram_ros.msg import Options

# Robot skills
from robot_smach_states import WaitTime
from robot_smach_states.util.designators import EdEntityDesignator
import robot_smach_states as states


class SelectOptionForImage(smach.State):
    def __init__(self, robot, options,
                 question='Which do you want?',
                 instruction='Please let me know your selection',
                 timeout=60, strict=True):
        """

        :param robot: Robot object, because all our Smach states have this and we need consistency
        :param options: What options are available?
        :type options: List[str]
        :param question: What is the question to select something for?
        :type question: str
        :param instruction: Instructions for the user on what to do
        :type instruction: str
        :param timeout: How long to wait until the state returns failed
        :type timeout: int
        :param strict: Only allow selection from the options list
        :type strict: bool
        """
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['person_dict'],
                             output_keys=['selection'])

        self._timeout = timeout
        self._received = Event()

        self._options = options
        self._option_str = ', '.join(self._options[:-1])
        if len(self._options) > 1:
            self._option_str += ' or ' + self._options[-1]

        self._strict = strict

        self._cv_bridge = cv_bridge.CvBridge()

        self._selection = ''

        self._question = question
        self._instruction = instruction

        self._image_pub = rospy.Publisher('/hero/image_from_ros', Image, queue_size=1)
        self._text_pub = rospy.Publisher('/hero/message_from_ros', String, queue_size=1)
        self._options_pub = rospy.Publisher('/hero/options_from_ros', Options, queue_size=1)
        self._text_sub = rospy.Subscriber('/hero/message_to_ros', String, self._handle_reply)

        self._text_pub.publish("Hello, I'm HERO")

    def execute(self, user_data):
        # Get a dict {'rgb':..., 'person_detection':..., 'map_ps':...}
        # Publish this image to telegram
        # Then ask what their selection is from a list of options

        self._received = Event()
        self._selection = None

        self._options_pub.publish(Options('Test 1 2 3', ['OK', 'Nope']))

        try:
            ros_image = user_data['person_dict']['rgb']  # Image

            cv_image = self._cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
            roi = user_data['person_dict']['person_detection'].face.roi
            cropped_cv_image = cv_image[roi.y_offset:roi.y_offset + roi.height, roi.x_offset:roi.x_offset + roi.width]
            # import ipdb; ipdb.set_trace()
            cropped_ros_image = self._cv_bridge.cv2_to_imgmsg(cropped_cv_image, "bgr8")
            cropped_ros_image.header.frame_id = self._question + '\n' + self._instruction + '\n' + self._option_str
            rospy.loginfo("Sending image with header {}".format(cropped_ros_image.header.frame_id))
            self._image_pub.publish(cropped_ros_image)
            options_msg = Options()
            options_msg.question = self._question
            options_msg.options = self._options
            self._options_pub.publish(options_msg)

            start = rospy.Time.now()
            while not rospy.is_shutdown() and rospy.Time.now() < start + rospy.Duration(self._timeout):
                if self._received.wait(1):
                    user_data['selection'] = self._selection
                    self._text_pub.publish("OK, gotcha")
                    return 'succeeded'
                else:
                    rospy.logwarn_throttle(10, "No reply received yet")
        except Exception as ex:
            rospy.logerr('Exception while getting selection: {}'.format(ex))

        self._text_pub.publish("I didn't get a reply in time, sorry")
        user_data['selection'] = None
        return 'failed'

    def _handle_reply(self, msg):
        # type: (String) -> None
        rospy.loginfo('Got answer from user: {}'.format(msg.data))

        if self._strict:
            if msg.data.lower().strip() not in self._options:
                self._text_pub.publish("{} is not on my list, please select something else, like {}"
                                       .format(msg.data, '\n'.join(self._options)))
            else:
                self._selection = msg.data
                self._received.set()
        else:
            self._selection = msg.data
            self._received.set()


if __name__ == '__main__':
    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        image_path = sys.argv[2]

        rospy.init_node('test_select_option_for_image')
        import mock

        # Robot
        # _robot = get_robot_from_argv(index=1)
        _robot = mock.MagicMock()
        _robot.speech = mock.MagicMock()
        def speak(*args, **kwargs):
            print(args, kwargs)
        _robot.speech.speak = speak

        bridge = cv_bridge.CvBridge()

        sm = SelectOptionForImage(_robot, options=['banana', 'widget', 'gadget'])

        ud = {}

        cv_image = cv2.imread(image_path)

        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  # cv2.imread returns in bgr order
        ud['person_dict'] = {'rgb': ros_image}  # Other keys are not required for this state

        print(sm.execute(ud))

        rospy.loginfo(ud['selection'])
    else:
        print "Please provide robot name as argument."
        exit(1)


