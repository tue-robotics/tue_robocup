from __future__ import absolute_import

# ROS
import rospy
import smach


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

    def execute(self, userdata):
        # Acquire the image and detect the faces in it
        self._robot.head.look_at_standing_person()
        rospy.sleep(1.0)

        image = self._robot.perception.get_image()
        faces, _ = self._robot.perception.detect_faces(image=image)

        # Check result
        if not faces:
            self._robot.image_pub.publish(image)
            rospy.logerr("DetectFaces: did not detect any faces")
            self._robot.speech.speak("I don't see anyone here")
            return 'failed'

        # Find the best match
        best_match = {}  # Contains index, face, label and probability
        for index, face in enumerate(faces):
            for probability in face.categorical_distribution.probabilities:
                if not best_match or probability.probability > best_match['probability']:
                    best_match = {'index': index, 'face': face, 'label': probability.label,
                                  'probability': probability.probability}

        self._robot.image_pub.publish(image)

        if "label" in best_match:
            self._robot.message_pub.publish(best_match['label'])

            # Return
            rospy.loginfo("DetectFace, best match: {}".format(best_match['label']))
            self._robot.speech.speak("Hi there {}".format(best_match['label']))
            self._robot.speech.speak("I will take a picture and send it to my operator now.")
        else:
            self._robot.speech.speak("I don't see anyone here")

        return 'succeeded'
