# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot


class SelectOperator(smach.State):
    def __init__(self, robot: Robot):
        """
        Selects the operator to follow and starts the tracking action

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I am selecting the operator")
        return "succeeded"


class FollowBreadcrumb(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state to follow a breadcrumb provided by the input queue

        :param robot:
        """
        smach.State.__init__(self, outcomes=["within_reach", "lost"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I am trying to follow the breadcrumb")
        return "within_reach"


class WithinReach(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state that is executed when the operator is within reach of the robot.
        Here, it is decided whether we keep tracking or are done.

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["continue", "done"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I am done here")
        return "done"


class Recover(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state that is executed when the operator is lost. Here, it is decided whether
        to start again or to go to failed

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["found", "lost"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I have lost my buddy")
        return "lost"


@smach.cb_interface(
    outcomes=["done", "lost"]
)
def cleanup(_: smach.UserData, robot: Robot, result: str) -> str:
    """
    Preempts the action

    :param _: Smach userdata
    :param robot: robot API object
    :param result: result to return. Passing this in eases reuse
    :return: result
    """
    robot.speech.speak("Now I should cancel the tracking")
    return result


class FollowOperator(smach.StateMachine):
    def __init__(self, robot: Robot):
        """
        Class to follow an operator

        :param robot: robot API object
        """
        smach.StateMachine.__init__(self, outcomes=["done", "lost"])

        with self:
            smach.StateMachine.add(
                "SELECT_OPERATOR", SelectOperator(robot), transitions={
                    "succeeded": "FOLLOW",
                    "failed": "lost",
                }
            )

            smach.StateMachine.add(
                "FOLLOW", FollowBreadcrumb(robot), transitions={
                    "within_reach": "WITHIN_REACH",
                    "lost": "RECOVER",
                }
            )

            smach.StateMachine.add(
                "WITHIN_REACH", WithinReach(robot), transitions={
                    "continue": "FOLLOW",
                    "done": "CLEANUP_DONE",
                }
            )

            smach.StateMachine.add(
                "RECOVER", Recover(robot), transitions={
                    "found": "FOLLOW",
                    "lost": "CLEANUP_LOST",
                }
            )

            smach.StateMachine.add(
                "CLEANUP_DONE", smach.CBState(cleanup, cb_kwargs={"robot": robot, "result": "done"}), transitions={
                    "done": "done",
                    "lost": "lost",
                }
            )

            smach.StateMachine.add(
                "CLEANUP_LOST", smach.CBState(cleanup, cb_kwargs={"robot": robot, "result": "lost"}), transitions={
                    "done": "done",
                    "lost": "lost",
                }
            )
