# ROS
import rospy
import smach


class TrackFace(smach.State):
    """ State to track the face of an operator. If the face is lost (or cannot be found), "lost" is returned. Otherwise,
     "aborted" is returned
     """
    def __init__(self, robot, name="operator"):
        """ Constructor
        :param robot: robot object
        :param name: name of the person whose face should be tracked
        """
        smach.StateMachine.__init__(self, outcomes=["aborted", "lost"])

        self._robot = robot
        self._name = name

    def execute(self, userdata):

        rate = rospy.Rate(0.5)

        # For now, set a timeout for 60 seconds
        # This should be replaced by a decent break out condition
        starttime = rospy.Time.now()
        while not rospy.is_shutdown():

            # Check the breakout condition
            if (rospy.Time.now() - starttime).to_sec() > 60.0:
                return "aborted"

            # Look for the face. Do a maximum of 5 tries
            for i in range(5):
                face = self._detect_face(self._name)
                if face is not None:
                    break
                rate.sleep()

            # If we still haven't found the operator, he's lost
            if face is None:
                return "lost"

            # Otherwise, send a head goal
            self._robot.head.look_at_point(vector_stamped=face, end_time=0)

            rate.sleep()

    def _detect_face(self, name, threshold=None):
        """ Tries to detect the face that is associated with the provided name
        :param name: name of the operator
        :param threshold: threshold used for discarding (not yet implemented)
        :return: VectorStamped with the position (in map) and None if not detected
        """
        # raw_detections is a list of Recognitions
        # a recognition constains a CategoricalDistribution
        # a CategoricalDistribution is a list of CategoryProbabilities
        # a CategoryProbability has a label and a float
        raw_detections = self._robot.head.detect_faces()

        # Only take detections with operator
        detections = []
        for d in raw_detections:
            for cp in d.categorical_distribution.probabilities:
                if cp.label == name:
                    detections.append((d, cp.probability))

        # Sort based on probability
        if detections:
            detections = sorted(detections, key=lambda det: det[1])
            best_detection = detections[0][0]
        else:
            best_detection = None

        # ToDo: threshold

        if best_detection:

            # print "Best detection: {}".format(best_detection)
            roi = best_detection.roi

            try:
                operator_pos_kdl = self._robot.head.project_roi(roi=roi, frame_id="map")
            except Exception as e:
                rospy.logerr("head.project_roi failed: %s", e)
                return None

        return operator_pos_kdl

