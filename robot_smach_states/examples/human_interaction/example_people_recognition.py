# System
import argparse

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.get_robot import get_robot
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.util.designators.core import VariableDesignator
from robot_smach_states.human_interaction import LearnPerson, RecognizePerson, Say


class ExamplePeopleRecognition(smach.StateMachine):
    def __init__(self, robot: Robot):
        """
        Example statemachine to test people learning and recognition

        :param robot: robot API object
        """
        smach.StateMachine.__init__(self, outcomes=["Done"])

        name_designator = VariableDesignator(resolve_type=str)

        with self:
            smach.StateMachine.add(
                "LEARN_PERSON",
                LearnPerson(robot=robot, person_name="Yoda"),
                transitions={
                    "succeeded": "SAY_WILL_RECOGNIZE",
                    "failed": "SAY_LEARNING_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_WILL_RECOGNIZE",
                Say(robot=robot, sentence="Let's see if I recognize you"),
                transitions={"spoken": "RECOGNIZE_PERSON"},
            )

            smach.StateMachine.add(
                "RECOGNIZE_PERSON",
                RecognizePerson(robot=robot, name_designator=name_designator.writeable),
                transitions={
                    "succeeded": "SAY_RECOGNIZED",
                    "no_operator_detected": "SAY_RECOGNITION_FAILED",
                    "operator_not_recognized": "SAY_RECOGNITION_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_LEARNING_FAILED",
                Say(robot=robot, sentence="Something went terribly wrong while trying to learn your face"),
                transitions={"spoken": "Done"},
            )

            smach.StateMachine.add(
                "SAY_RECOGNIZED",
                Say(robot=robot, sentence="Hey {name}", name=name_designator),
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
    r = get_robot(args.robot)
    # from robot_skills.mockbot import Mockbot
    # r = Mockbot()

    state = ExamplePeopleRecognition(robot=r)
    state.execute()

    rospy.loginfo("Example done")
