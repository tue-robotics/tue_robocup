#ROS
import rospy
import smach
import argparse

# TU/e Robotics
from image_recognition_msgs.msg import CategoricalDistribution, CategoryProbability, Recognition
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
    distribution = Recognition.categorical_distribution  # type: CategoricalDistribution
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
        # ToDo: the following two methods won't work: they'll return whether a learned person is in the image,
        # not if the person in front of the robot is learned (and his name)
        recognition = self._robot.perception.detect_face()
        try:
            operator_name = get_operator_name(recognition=recognition)
        except:  # ToDo: narrow down exceptions
            rospy.loginfo("Did not recognize the operator")
            return "failed"

        # ToDo: write this in the designator
        return "succeeded"


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
