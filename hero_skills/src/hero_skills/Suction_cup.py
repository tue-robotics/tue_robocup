#ROS
import rospy
from actionlib import GoalStatus

#General
import PyKDL as kdl

# TUe Robotics
from robot_skills.arm.gripper import Gripper
# Toyota
from tmc_suction.msg import SuctionControlAction, SuctionControlGoal


class SuctionGripper(Gripper):
    """
    A Suction gripper that has a vacuum suction cup to grasp objects
    """
    def __init__(self, robot_name, tf_buffer, gripper_name):
        """
        constructor
        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        :param gripper_name: string to identify the gripper
        """
        super(SuctionGripper, self).__init__(robot_name=robot_name, tf_listener=tf_buffer)
        self.gripper_name = gripper_name
        offset = self.load_param('skills/' + self.gripper_name + '/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),kdl.Vector(offset["x"], offset["y"], offset["z"]))

        # Init gripper actionlib for SuctionGripper
        self._ac_suction = self.create_simple_action_client("/" + robot_name + "/suction_control", SuctionControlAction)

        # Waits until the action server has started up and started
        # listening for goals
        self._ac_suction.wait_for_server()

    def send_gripper_goal(self, sucking, timeout=0.0):
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

        if timeout != 0.0:
            self._ac_suction.wait_for_result(rospy.Duration(timeout))
            goal_status = self._ac_suction.get_state()
            goal_status_bool = goal_status == GoalStatus.SUCCEEDED
        else:
            rospy.logerr("A timeout value of 0.0 is given, which is not allowed!")
            goal_status = self._ac_suction.get_state()
            while goal_status == GoalStatus.PENDING:
                goal_status = self._ac_suction.get_state()
            goal_status_bool = goal_status == GoalStatus.SUCCEEDED

        return goal_status_bool
