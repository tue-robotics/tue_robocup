import math
import threading

import rospy
import smach
from geometry_msgs.msg import Twist


class TrackFace(smach.State):
    """ State to track the face of an operator. If the face is lost (or cannot be found), "lost" is returned. Otherwise,
     "aborted" is returned
     """

    def __init__(self, robot, name="operator"):
        """ Constructor
        :param robot: robot object
        :param name: name of the person whose face should be tracked
        """
        smach.State.__init__(self, outcomes=["aborted", "lost"])

        self._robot = robot
        self._name = name
        self._cmd_vel_pub = rospy.Publisher("/{}/base/references".format(self._robot.robot_name),
                                            Twist, queue_size=1)
        self._stop_requested = False
        self._angle = 0.0

    def execute(self, userdata):
        return self._execute()
        result = None
        try:
            result = self._execute()
        except Exception as e:
            rospy.logerr("{}".format(e))
            result = "lost"
        finally:
            self.stop()
            if abs(self._angle) > 0.25:
                rospy.sleep(rospy.Duration(0.5))  # Give the base controller half a second
            return result

    def _execute(self):
        """ Does the execute but can be wrapped in a try-except for safety reasons """

        self._stop_requested = False
        self._angle = 0.0

        # Start by looking at a standing person
        self._robot.head.look_at_standing_person()

        # Start the controller thread
        # controller_thread = threading.Thread(target=self._control_base)
        # controller_thread.start()

        # Start the breakout thread
        breakout_thread = threading.Thread(self._breakout_checker)
        breakout_thread.start()

        rate = rospy.Rate(15.0)

        # For now, set a timeout for 60 seconds
        # This should be replaced by a decent break out condition
        starttime = rospy.Time.now()
        while not rospy.is_shutdown():

            # Check the breakout condition
            if (rospy.Time.now() - starttime).to_sec() > 60.0:
                self.stop()
                return "aborted"

            # Look for the face. Do a maximum of 5 tries
            for i in range(5):
                face = self._detect_face(self._name)
                if face is not None:
                    break
                rate.sleep()

            # If we still haven't found the operator, he's lost
            if face is None:
                self.stop()
                return "lost"

            # Otherwise, send a head goal
            self._robot.head.look_at_point(vector_stamped=face, end_time=0, pan_vel=0.2, tilt_vel=0.2)

            # Also: add the base controller
            face_bl = face.projectToFrame(frame_id="/{}/base_link".format(self._robot.robot_name),
                                          tf_listener=self._robot.tf_listener)
            self._angle = math.atan2(face_bl.vector.y(), face_bl.vector.x())
            rospy.loginfo("Angle: {}".format(self._angle))
            rate.sleep()

        self.stop()

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
        raw_detections = self._robot.perception.detect_faces()

        # Only take detections with operator
        detections = []
        for d in raw_detections:
            for cp in d.categorical_distribution.probabilities:
                # if cp.label == name:
                if cp.label == "operator":
                    detections.append((d, cp.probability))

        # Sort based on probability
        if not detections:
            return None

        detections = sorted(detections, key=lambda det: det[1])
        best_detection = detections[0][0]
        rospy.loginfo("Detection probability/l2distance: {}".format(detections[0][1]))

        # ToDo: threshold

        # print "Best detection: {}".format(best_detection)
        roi = best_detection.roi
        try:
            operator_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
        except Exception as e:
            rospy.logerr("perception.project_roi failed: %s", e)
            return None

        return operator_pos_kdl

    def stop(self):
        """ Sets stop requested to True
        """
        self._stop_requested = True

    def _control_base(self):
        """ Base controller function that should be run in a separate thread
        """
        # Hardcoded controller gain
        gain = 0.5

        # Loop until stop requested
        rate = rospy.Rate(20.0)
        while not self._stop_requested and not rospy.is_shutdown():
            msg = Twist()
            # Threshold (approximately 30 degrees)
            if abs(self._angle) > 0.5:
                msg.angular.z = gain * self._angle

            self._cmd_vel_pub.publish(msg)
            rate.sleep()

        # Finally: twist a zero message to make the robot stop
        msg = Twist()
        self._cmd_vel_pub.publish(msg)

    def _breakout_checker(self):
        """ Continuously queries hmi so that the operator can ask the robot to stop
        """
        # Loop forever until
        while not self._stop_requested and not rospy.is_shutdown():
            try:
                speech_result = self._robot.hmi.query(description="Do you want me to stop stalking you?",
                                                      grammar="C -> amigo stop", target="C")
                self.stop()
            except TimeoutException:
                pass
