# ROS
import rospy
import smach


class DetectFace(smach.State):
    """
    Detect face ToDo: expand docstring
    """
    def __init__(self, robot):
        """ Initialization

        :param robot: Robot API object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot

    def execute(self, userdata):

        # Acquire the image and detect the faces in it
        image = self.robot.perception.get_image()
        faces = self.robot.perception.detect_faces(image=image)

        # Check result
        if not faces:
            rospy.logerr("DetectFaces: did not detect any faces")
            return 'failed'

        # Find the best match
        best_match = {}  # Contains index, face, label and probability
        for index, face in enumerate(faces):
            for probability in face.probabilities:
                if not best_match or probability.probability > best_match['probability']:
                    best_match = {'index': index, 'face': face, 'label': probability.label,
                                  'probability': probability.probability}

        # Return
        rospy.loginfo("DetectFace, best match: {}".format(best_match['label']))
        # ToDo: what do we want to do with our best match???
        return 'succeeded'
