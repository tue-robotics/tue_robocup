import PyKDL as kdl
import rospy
from actionlib import GoalStatus
from tue_manipulation_msgs.msg import GripperCommandAction, GripperCommandGoal
from tue_msgs.msg import GripperCommand

from robot_skills.robot_part import RobotPart
from ed.entity import Entity


class GripperState(object):
    """
    Specifies a State either OPEN or CLOSE
    """
    OPEN = "open"
    CLOSE = "close"


class Gripper(RobotPart):
    """
    A gripper used for manipulating objects in the environment.
    """
    def __init__(self, robot_name: str, tf_buffer: str) -> None:
        """
        constructor

        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        """
        super(Gripper, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.offset = None
        # TODO: remove occupied by, an interface should be stateless. Store this information elsewhere.
        self._occupied_by = None

    def send_goal(self, state: str):
        rospy.logdebug("send_goal() not implemented for {} gripper: {}".format(self.robot_name, self))
        return True

    @property
    def occupied_by(self) -> Entity:
        """
        The 'occupied_by' property will return the current entity that is in the gripper of this arm.

        :return: robot_skills.util.entity, ED entity
        """

        return self._occupied_by

    @occupied_by.setter
    def occupied_by(self, value: Entity) -> None:
        """
        Set the entity which occupies the arm.

        :param value: robot_skills.util.entity, ED entity
        :return: no return
        """
        self._occupied_by = value


class ParrallelGripper(Gripper):
    """
    A gripper with two fingers which closes by pressing the fingers together
    """
    def __init__(self, robot_name: str, tf_buffer: str, gripper_name: str) -> None:
        """
        constructor

        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        :param gripper_name: string to identify the gripper
        """
        super(ParrallelGripper, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.gripper_name = gripper_name
        offset = self.load_param('skills/' + self.gripper_name + '/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),
                                        kdl.Vector(offset["x"], offset["y"], offset["z"]))

        # Init gripper actionlib
        ac_name = self.load_param('skills/' + self.gripper_name + '/ac_gripper')
        self._ac_gripper = self.create_simple_action_client(ac_name, GripperCommandAction)

    def send_goal(self, state: str, timeout: float = 5.0, max_torque: float = 0.1) -> bool: #Todo: should send goal be universal for all grippers?
        """
        Send a GripperCommand to the gripper of this arm and wait for finishing

        :param state: open or close (GripperState)
        :type state: str (GripperState)
        :param timeout: timeout in seconds; timeout of 0.0 is not allowed
        :param max_torque: How much torque [Nm] to apply, only applied when closing the gripper
        :return: True of False
        """
        goal = GripperCommandGoal()

        if state == "open":
            goal.command.direction = GripperCommand.OPEN
        elif state == "close":
            goal.command.direction = GripperCommand.CLOSE
            goal.command.max_torque = max_torque
        else:
            rospy.logerr('State shoulde be open or close, now it is {0}'.format(state))
            return False

        self._ac_gripper.send_goal(goal)

        if goal.command.direction == GripperState.OPEN:
            if self.occupied_by is not None:
                rospy.logerr("ParrallelGripper.send_goal open is called but there is still an entity with id '%s' \
                occupying the gripper, removing this entity from the robot interface." % self.occupied_by.uuid)
            self.occupied_by = None

        goal_status = GoalStatus.SUCCEEDED
        if timeout != 0.0:
            self._ac_gripper.wait_for_result(rospy.Duration(timeout))
            goal_status = self._ac_gripper.get_state()

        return goal_status == GoalStatus.SUCCEEDED

    def selfreset(self):
        return self.send_goal('open', 0.0)
