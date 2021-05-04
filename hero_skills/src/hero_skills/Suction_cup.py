#ROS
import rospy

import PyKDL as kdl
from actionlib import GoalStatus
from tue_manipulation_msgs.msg import GripperCommandAction, GripperCommandGoal
from tue_msgs.msg import GripperCommand


from robot_skills.arms.gripper import Gripper
# tmc_suction
from tmc_suction.msg import SuctionControlAction, SuctionControlGoal


class SuctionGripper(Gripper):
    """
    A Suction gripper that has a vacuum suction cup to grasp objects
    """
    def __init__(self, robot_name, tf_listener, gripper_name):
        """
        constructor
        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param gripper_name: string to identify the gripper
        """
        super(SuctionGripper, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.gripper_name = gripper_name
        offset = self.load_param('skills/' + self.gripper_name + '/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),kdl.Vector(offset["x"], offset["y"], offset["z"]))

        # Init gripper actionlib for SuctionGripper
        self._ac_suction = self.create_simple_action_client("/" + robot_name + "/suction_control", SuctionControlAction)

        # Waits until the action server has started up and started
        # listening for goals
        self._ac_suction.wait_for_server()

    def send_gripper_goal(self, sucking, timeout=5.0):
        """
        Send a GripperCommand to the gripper and wait for finishing
        :param sucking: turn suction on or off
        :type sucking: bool
        :param timeout: timeout in seconds; timeout of 0.0 is not allowed
        :type timeout: float
        :return: True of False
        :rtype: bool
        """

        if not isinstance(sucking, bool):
            rospy.logerr('State should be true or false, now it is {0}'.format(sucking))
            return False

        goal = SuctionControlGoal()

        goal.suction_on.data = sucking

        self._ac_suction.send_goal(goal)

        goal_status = GoalStatus.SUCCEEDED

        if timeout != 0.0:
            self._ac_suction.wait_for_result(rospy.Duration(timeout))
            goal_status = self._ac_suction.get_state()

        return goal_status == GoalStatus.SUCCEEDED

