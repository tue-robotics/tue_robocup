# ROS
import actionlib
import geometry_msgs.msg
import PyKDL as kdl
import rospy
import smach
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
import tf2_ros

# TU/e Robotics
import pykdl_ros
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
from people_recognition_msgs.msg import TrackOperatorAction, TrackOperatorGoal
from robot_skills.robot import Robot


class SelectOperator(smach.State):
    def __init__(self, robot: Robot):
        """
        Selects the operator to follow and starts the tracking action

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        # ToDo: move to robot_skills
        self._ac = actionlib.ActionClient(f"/{robot.robot_name}/track_operator", TrackOperatorAction)

    def execute(self, _):
        self._robot.speech.speak("I am selecting the operator, still assuming he or she is right in front of me")
        robot_pose_stamped = self._robot.base.get_location()
        operator_pose = robot_pose_stamped.frame * kdl.Frame(kdl.Vector(1.0, 0.0, 0.0))
        operator_pose_stamped = pykdl_ros.FrameStamped(operator_pose, rospy.Time.now(), "map")
        goal = TrackOperatorGoal()
        goal.start_pose = tf2_ros.convert(operator_pose_stamped, geometry_msgs.msg.PoseStamped)
        self._ac.send_goal(goal=goal)  # ToDo: how do we register the feedback CB?
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
