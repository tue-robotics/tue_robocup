# ROS
import rospy
import smach

# TU/e Robotics
from image_recognition_msgs.msg import CategoricalDistribution, CategoryProbability, Recognition
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.util.designators.core import VariableDesignator


def get_operator_name(recognition: Recognition) -> str:
    """
    Gets the operator name from a recognition

    :param recognition:
    :return: operator name
    :raises: ...
    """
    # ToDo: what can it raise? --> add to docstring
    # ToDo: add tests
    distribution = recognition.categorical_distribution  # type: CategoricalDistribution
    best_match = max(distribution.probabilities, key=lambda cp: cp.probability)  # type: CategoryProbability
    return best_match.label


class RecognizePerson(smach.State):
    def __init__(self, robot: Robot, name_designator: VariableDesignator):
        """
        State to recognize a person. Detects faces, stores name with highest probability in a designator

        :param robot: robot API object
        :param name_designator: VariableDesignator to write the name to
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        # ToDo: add threshold
        self._robot = robot
        self._name_designator = name_designator

    # noinspection PyUnusedLocal
    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person(timeout=3.0)
        try:
            operator_recognition = self._robot.perception.detect_operator_face()
            operator_name = get_operator_name(recognition=operator_recognition)
            rospy.loginfo(f"Operator name: {operator_name}")
        except Exception as e:  # ToDo: narrow down exceptions
            rospy.loginfo(f"Did not recognize the operator: {e}")
            return "failed"

        self._name_designator.write(operator_name)
        return "succeeded"
