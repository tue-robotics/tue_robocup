import sys
import cv2
from threading import Event

# ROS
import rospy
import smach
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Robot skills
from robot_smach_states import WaitTime
from robot_smach_states.util.designators import EdEntityDesignator
import robot_smach_states as states


class SelectImageByLetter(smach.State):
    def __init__(self, robot, question='Which do you want?', timeout=60):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['label2image'],
                             output_keys=['selected_label'])

        self._timeout = timeout
        self._received = Event()
        self._selection = ''

        self._question = question

        self._image_pub = rospy.Publisher('image_from_ros', Image, queue_size=10)
        self._text_pub = rospy.Publisher('message_from_ros', String, queue_size=10)
        self._text_sub = rospy.Subscriber('message_to_ros', String, self._handle_reply)

    def execute(self, user_data):
        # Get a dict {str: Image}
        # Publish these to telegram:
        #   image 1
        #   text 1
        #   image 2
        #   text 2
        # Please select a picture by it's label
        #  Receive the desired label
        # Return the label to the next state

        self._received = Event()
        self._selection = None

        rate = rospy.Rate(2)

        self._text_pub.publish(self._question)
        for label, image_msg in user_data['label2image'].items():
            # The order in which these images are received matters! text should be below the
            self._image_pub.publish(image_msg)
            rate.sleep()
            self._text_pub.publish(label)
            rate.sleep()

        self._text_pub.publish("Please pick something from one of the pictures and send me the label below")

        if self._received.wait(self._timeout):
            user_data['selected_label'] = self._selection
            return 'succeeded'

        return 'failed'

    def _handle_reply(self, msg):
        # type: (String) -> None
        self._selection = msg.data

        self._received.set()

if __name__ == '__main__':
    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

        rospy.init_node('test_find_person_in_room')
        _robot = None # get_robot(robot_name)

        bridge = cv_bridge.CvBridge()

        sm = SelectImageByLetter(_robot)

        image_paths = sys.argv[2:]
        labels = ['a', 'b', 'c', 'd', 'e']

        ud = {'label2image': {}}

        for label, image_path in zip(labels, image_paths):
            rospy.loginfo("Loading image for {}: {}".format(label, image_path))
            try:
                cv_image = cv2.imread(image_path)

                ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                ud['label2image'][label] = ros_image
            except TypeError as type_err:
                rospy.logerr("Could not load {}".format(image_path))

        sm.execute(ud)

        _robot.speech.speak(ud['selected_label'])
    else:
        print "Please provide robot name as argument."
        exit(1)


