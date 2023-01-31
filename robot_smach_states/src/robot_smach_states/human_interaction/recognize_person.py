# System
import typing

# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
from image_recognition_msgs.msg import CategoricalDistribution, CategoryProbability, Recognition
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.util.designators.core import VariableDesignator


class NoDetections(Exception):
    """
    Exception that is thrown if there is no detection, i.e., list of probabilities is empty
    """
    pass


class NoRecognitionMatch(Exception):
    """
    Exception that is thrown if detections don't match any learned faces, i.e., all detections are below the given
    threshold
    """
    pass


def get_operator_name(recognition: Recognition, threshold: typing.Optional[float] = 3.0) -> str:
    """
    Gets the operator name from a recognition

    :param recognition: recognition of the face
    :param threshold: if the best match is below this threshold, it is designated as unknown and an exception is raised
    :return: operator name
    :raises: NoDetections (if recognition is empty) or NoRecognitionMatch if all detections below threshold
    """
    distribution = recognition.categorical_distribution  # type: CategoricalDistribution
    if not distribution.probabilities:
        raise NoDetections("Recognition does not contain probabilities")
    best_match = max(distribution.probabilities, key=lambda cp: cp.probability)  # type: CategoryProbability
    if best_match.probability < threshold:
        raise NoRecognitionMatch(
            f"Best match has a probability of {best_match.probability} which is lower than the threshold {threshold}"
        )
    return best_match.label


class RecognizePerson(smach.State):
    def __init__(
        self,
        robot: Robot,
        name_designator: VariableDesignator,
        expected_operator_position: typing.Optional[kdl.Vector] = None,
        operator_distance_threshold: typing.Optional[float] = 0.5,
        recognition_threshold: typing.Optional[float] = 3.0,
    ):
        """
        State to recognize a person. Detects faces, stores name with highest probability in a designator

        :param robot: robot API object
        :param name_designator: VariableDesignator to write the name to
        :param expected_operator_position: expected operator position w.r.t. robot base
        :param operator_distance_threshold: people outside this radius from the expected position are discarded
        :param recognition_threshold: if the 'probability' of the recognition is below this threshold, it is designated
        as 'not recognized'. N.B.: this is not really a probability.
        """
        smach.State.__init__(self, outcomes=["succeeded", "no_operator_detected", "operator_not_recognized"])
        self._robot = robot
        self._name_designator = name_designator
        self._expected_operator_position = kdl.Vector(1.0, 0.0, 0.0) if expected_operator_position is None \
            else expected_operator_position
        self._operator_distance_threshold = operator_distance_threshold
        self._recognition_threshold = recognition_threshold

    # noinspection PyUnusedLocal
    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person(timeout=5.0)
        rospy.sleep(rospy.Duration(1.0))  # Timeout to accomodate delays in head motions and image streams
        try:
            operator_recognition = self._robot.perception.detect_operator_face(
                expected_operator_position=self._expected_operator_position,
                operator_distance_threshold=self._operator_distance_threshold,
            )
            operator_name = get_operator_name(recognition=operator_recognition)
            rospy.loginfo(f"Operator name: {operator_name}")
        except (RuntimeError, NoDetections) as e:
            rospy.loginfo(f"Did not detect the operator: {e}")
            return "no_operator_detected"
        except NoRecognitionMatch as e:
            rospy.loginfo(f"Did not recognize the operator: {e}")
            return "operator_not_recognized"

        self._name_designator.write(operator_name)
        return "succeeded"
