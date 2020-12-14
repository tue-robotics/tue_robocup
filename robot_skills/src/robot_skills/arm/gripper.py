import rospy
import PyKDL as kdl

from robot_skills.robot_part import RobotPart
from tue_manipulation_msgs.msg import GripperCommandGoal, GripperCommandAction
from tue_msgs.msg import GripperCommand

class Gripper(RobotPart):
    """
    TODO: add description
    """
    def __init__(self, robot_name, tf_listener):
        """
        Todo: add docstring
        Args:
            robot_name:
            tf_listener:
        """
        super(Gripper, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.offset = None
        # TODO: remove occupied by, an interface should be stateless. Store this information elsewhere.
        self._occupied_by = None

    def send_goal(self, state):
        rospy.logdebug("send_goal() not implemented for {} gripper: {}".format(self.robot_name, self))
        return True

    @property
    def occupied_by(self):
        """
        The 'occupied_by' property will return the current entity that is in the gripper of this arm.

        :return: robot_skills.util.entity, ED entity
        """

        return self._occupied_by

    @occupied_by.setter
    def occupied_by(self, value):
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
    def __init__(self, robot_name, tf_listener, gripper_name):
        """
        Todo: add docstring
        Args:
            robot_name:
            tf_listener:
        """
        super(ParrallelGripper, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.gripper_name = gripper_name
        offset = self.load_param('skills/arm/' + self.gripper_name + '/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),
                                        kdl.Vector(offset["x"], offset["y"], offset["z"]))

        # Init gripper actionlib
        self._ac_gripper = self.create_simple_action_client(
            "/" + robot_name + "/" + self.gripper_name + "_arm/gripper/action", GripperCommandAction)

    def send_goal(self, state, timeout=5.0, max_torque=0.1): #Todo: should send goal be universal for all grippers?
        """
        Send a GripperCommand to the gripper of this arm and wait for finishing

        :param state: open or close
        :type state: str (GripperState)
        :param timeout: timeout in seconds; timeout of 0.0 is not allowed
        :type timeout: float
        :param max_torque: How much torque [Nm] to apply, only applied when closing the gripper
        :return: True of False
        :rtype: bool
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

        if state == GripperState.OPEN:
            if self.occupied_by is not None:
                rospy.logerr("ParrallelGripper.send_goal open is called but there is still an entity with id '%s' \
                occupying the gripper, please update the world model and remove this entity" % self.occupied_by.id)
            self.occupied_by = None

        goal_status = GoalStatus.SUCCEEDED
        if timeout != 0.0:
            self._ac_gripper.wait_for_result(rospy.Duration(timeout))
            goal_status = self._ac_gripper.get_state()

        return goal_status == GoalStatus.SUCCEEDED

    def selfreset(self):
        return self.send_goal('open', 0.0)
