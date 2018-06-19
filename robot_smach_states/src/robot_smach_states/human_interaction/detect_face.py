# ROS
import rospy
import smach
from std_msgs.msg import String
from sensor_msgs.msg import Image


class DetectFace(smach.State):
    """
    Smach state to detect a face. It tries to match these with 'known' faces (either loaded from file or learned
    earlier). If one or more faces have been recognized in the image, the image is published over the /photo_to_telegram
    topic and the corresponding best matched label is published over the message to telegram topic.
    """

    def __init__(self, robot):
        """ Initialization

        :param robot: Robot API object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._pub_image = rospy.Publisher(robot.robot_name + '/image_from_ros', Image, queue_size=1)
        self._pub_label = rospy.Publisher(robot.robot_name + '/message_from_ros', String, queue_size=1)

    def execute(self, userdata):

        # Acquire the image and detect the faces in it
        image = self._robot.perception.get_image()
        faces = self._robot.perception.detect_faces(image=image)

        # Check result
        if not faces:
            rospy.logerr("DetectFaces: did not detect any faces")
            return 'failed'

        # Find the best match
        best_match = {}  # Contains index, face, label and probability
        for index, face in enumerate(faces):
            for probability in face.categorical_distribution.probabilities:
                if not best_match or probability.probability > best_match['probability']:
                    best_match = {'index': index, 'face': face, 'label': probability.label,
                                  'probability': probability.probability}

        self._pub_image.publish(image)
        self._pub_label.publish(best_match['label'])

        # Return
        rospy.loginfo("DetectFace, best match: {}".format(best_match['label']))
        return 'succeeded'
