# System
import argparse
import functools
import math
import typing

# ROS
import rospy
import smach
import PyKDL as kdl

# TU/e Robotics
from image_recognition_msgs.msg import CategoricalDistribution, CategoryProbability, Recognition
from pykdl_ros import VectorStamped
from robot_skills.get_robot import get_robot
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.human_interaction import LearnPerson, Say


def get_operator_name(recognition: Recognition) -> str:
    """
    Gets the operator name from a recognition

    :param recognition:
    :return: operator name
    :raises: ...
    """
    # ToDo: move to a better location
    # ToDo: what can it raise? --> add to docstring
    # ToDo: add tests
    distribution = recognition.categorical_distribution  # type: CategoricalDistribution
    best_match = max(distribution.probabilities, key=lambda cp: cp.probability)  # type: CategoryProbability
    return best_match.label


class RecognizePerson(smach.State):
    def __init__(self, robot: Robot):
        """
        State to recognize a person. Detects faces, stores name with highest probability in a designator

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        # ToDo: add threshold
        # ToDo: provide designator to store the person name
        # ToDo: move this state to the src folder
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person(timeout=3.0)
        # recognition = self._robot.perception.detect_face()
        recognitions = self._robot.perception.detect_faces()  # For now, detect all faces here
        try:
            operator_recognition = self.filter_operator_recognition(recognitions)
            operator_name = get_operator_name(recognition=operator_recognition)
            rospy.loginfo(f"Operator name: {operator_name}")
        except:  # ToDo: narrow down exceptions
            rospy.loginfo("Did not recognize the operator")
            return "failed"

        # ToDo: write this in the designator
        return "succeeded"

    def filter_operator_recognition(
          self,
          recognitions: typing.List[Recognition],
          expected_operator_pos: typing.Optional[kdl.Vector] = None,
          threshold: float = 0.5,
    ) -> Recognition:
        """
        For all provided recognitions, project the ROI and determine the one closest to the expected operator position,
        computed on the floor plane.

        N.B.: it might make sense to move this method to 'Perception.py' as 'detect_operator_face'

        :param recognitions: recognitions
        :param expected_operator_pos: expected position of the operator w.r.t. the robot
        :param threshold: if the distance between the recognition and the expected pos exceed the threshold, an error
        is raised
        :return: Recognition closed to the expected position
        :raises: RuntimeError
        """
        # ToDo: move this method to Perception.py?
        frame_id = self._robot.base_link_frame
        projected_recognitions = [(rcg, self._robot.perception.project_roi(r.roi, frame_id)) for rcg in recognitions]
        expected_operator_pos = kdl.Vector(0.8, 0.0, 0.0) if expected_operator_pos is None else expected_operator_pos

        def _distance_from_expected(
              expected_pos: kdl.Vector,
              projected_tup: typing.Tuple[Recognition, VectorStamped]
        ) -> float:
            """
            Helper method to compute the distance between the projected tuple and the expected position

            :param expected_pos:
            :param projected_tup:
            :return:
            """
            dx = projected_tup[1].vector.x() - expected_pos.x()
            dy = projected_tup[1].vector.y() - expected_pos.y()
            return math.hypot(dx, dy)

        projected_recognitions.sort(functools.partial(_distance_from_expected, expected_operator_pos))
        best_distance = _distance_from_expected(expected_operator_pos, projected_recognitions[0])
        if best_distance > threshold:
            err_msg = f"Distance between face detection and expected pos {best_distance} exceeds threshold {threshold}"
            rospy.logwarn(err_msg)
            raise RuntimeError(err_msg)
        else:
            return projected_recognitions[0]


class ExamplePeopleRecognition(smach.StateMachine):
    def __init__(self, robot: Robot):
        """
        Example statemachine to test people learning and recognition

        :param robot: robot API object
        """
        smach.StateMachine.__init__(self, outcomes=["Done"])

        with self:
            smach.StateMachine.add(
                "LEARN_PERSON",
                LearnPerson(robot=robot, person_name="Yoda"),
                transitions={
                    "succeeded": "RECOGNIZE_PERSON",
                    "failed": "SAY_LEARNING_FAILED",
                },
            )

            smach.StateMachine.add(
                "RECOGNIZE_PERSON",
                RecognizePerson(robot=robot),
                transitions={
                    "succeeded": "SAY_RECOGNIZED",
                    "failed": "SAY_RECOGNITION_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_LEARNING_FAILED",
                Say(robot=robot, sentence="Something went terribly wrong while trying to learn your face"),
                transitions={"spoken": "Done"},
            )

            # ToDo: add the designator here as well to pronounce the operator name
            smach.StateMachine.add(
                "SAY_RECOGNIZED",
                Say(robot=robot, sentence="Hey, I have seen you before"),
                transitions={"spoken": "Done"},
            )

            smach.StateMachine.add(
                "SAY_RECOGNITION_FAILED",
                Say(robot=robot, sentence="I do not think I have seen you before"),
                transitions={"spoken": "Done"},
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the say state with a sentence")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_people_recognition')
    # r = get_robot(args.robot)
    from robot_skills.mockbot import Mockbot
    r = Mockbot()

    state = ExamplePeopleRecognition(robot=r)
    state.execute()

    rospy.loginfo("Example done")
