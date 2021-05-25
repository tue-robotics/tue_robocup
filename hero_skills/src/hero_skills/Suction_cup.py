#ROS
import rospy
import time
import PyKDL as kdl
from actionlib import GoalStatus
from tue_manipulation_msgs.msg import GripperCommandAction, GripperCommandGoal
from tue_manipulation_msgs.msg import GraspPrecomputeGoal, GraspPrecomputeAction
from tue_msgs.msg import GripperCommand


from robot_skills.arm.gripper import Gripper
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

    @property
    def occupied_by_suction(self):
        """
        The 'occupied_by_suction' property will return the current entity that is in the suction cup of this arm.
        :return: robot_skills.util.entity, ED entity
        """
        return self._occupied_by_suction

    @occupied_by_suction.setter
    def occupied_by_suction(self, value):
        """
        Set the entity which occupies the suction cup.
        :param value: robot_skills.util.entity, ED entity
        :return: no return
        """
        self._occupied_by_suction = value

    def send_goal_suction(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                          allowed_touch_objects=None):
        """
        Send a arm to a goal:
        Using a combination of position and orientation: a kdl.Frame. A time
        out time_out. pre_grasp means go to an offset that is normally needed
        for things such as grasping. You can also specify the frame_id which
        defaults to base_link
        :param frameStamped: A FrameStamped to move the arm's end effector to
        :param timeout: timeout in seconds; In case of 0.0, goal is executed without feedback and waiting
        :param pre_grasp: Bool to use pre_grasp or not
        :param first_joint_pos_only: Bool to only execute first joint position of whole trajectory
        :param allowed_touch_objects: List of object names in the worldmodel, which are allowed to be touched
        :return: True of False
        """
        if allowed_touch_objects is None:
            allowed_touch_objects = list()

        # save the arguments for debugging later
        myargs = locals()

        # If necessary, prefix frame_id
        if frameStamped.frame_id.find(self.robot_name) < 0:
            frameStamped.frame_id = "/" + self.robot_name + "/" + frameStamped.frame_id
            rospy.loginfo("Grasp precompute frame id = {0}".format(frameStamped.frame_id))

        # Convert to baselink, which is needed because the offset is defined in the base_link frame
        frame_in_baselink = frameStamped.projectToFrame("/" + self.robot_name + "/base_link", self.tf_listener)

        # TODO: Get rid of this custom message type
        # Create goal:
        grasp_precompute_goal = GraspPrecomputeGoal()
        grasp_precompute_goal.goal.header.frame_id = frame_in_baselink.frame_id
        grasp_precompute_goal.goal.header.stamp = rospy.Time.now()

        grasp_precompute_goal.PERFORM_PRE_GRASP = pre_grasp
        grasp_precompute_goal.FIRST_JOINT_POS_ONLY = first_joint_pos_only

        grasp_precompute_goal.allowed_touch_objects = allowed_touch_objects

        grasp_precompute_goal.goal.x = frame_in_baselink.frame.p.x()
        grasp_precompute_goal.goal.y = frame_in_baselink.frame.p.y()
        grasp_precompute_goal.goal.z = frame_in_baselink.frame.p.z()

        roll, pitch, yaw = frame_in_baselink.frame.M.GetRPY()
        grasp_precompute_goal.goal.roll = roll
        grasp_precompute_goal.goal.pitch = pitch
        grasp_precompute_goal.goal.yaw = yaw

        self._publish_marker(grasp_precompute_goal, [1, 0, 0], "grasp_point")

        # Add tunable parameters
        # todo: generalise send_goal in base arm so it can work with multiple gripper types
        offset_frame = frame_in_baselink.frame * self.offset_suction

        grasp_precompute_goal.goal.x = offset_frame.p.x()
        grasp_precompute_goal.goal.y = offset_frame.p.y()
        grasp_precompute_goal.goal.z = offset_frame.p.z()

        roll, pitch, yaw = offset_frame.M.GetRPY()
        grasp_precompute_goal.goal.roll = roll
        grasp_precompute_goal.goal.pitch = pitch
        grasp_precompute_goal.goal.yaw = yaw

        # rospy.loginfo("Arm goal: {0}".format(grasp_precompute_goal))

        self._publish_marker(grasp_precompute_goal, [0, 1, 0], "grasp_point_corrected")

        time.sleep(0.001)  # This is necessary: the rtt_actionlib in the hardware seems
        # to only have a queue size of 1 and runs at 1000 hz. This
        # means that if two goals are send approximately at the same
        # time (e.g. an arm goal and a torso goal), one of the two
        # goals probably won't make it. This sleep makes sure the
        # goals will always arrive in different update hooks in the
        # hardware TrajectoryActionLib server.

        # Send goal:

        if timeout == 0.0:
            self._ac_grasp_precompute.send_goal(grasp_precompute_goal)
            return True
        else:
            result = self._ac_grasp_precompute.send_goal_and_wait(
                grasp_precompute_goal,
                execute_timeout=rospy.Duration(timeout))
            if result == GoalStatus.SUCCEEDED:

                result_pose = self.tf_listener.lookupTransform(self.robot_name + "/base_link",
                                                               self.robot_name + "/grippoint_{}".format(self.side),
                                                               rospy.Time(0))
                dx = grasp_precompute_goal.goal.x - result_pose[0][0]
                dy = grasp_precompute_goal.goal.y - result_pose[0][1]
                dz = grasp_precompute_goal.goal.z - result_pose[0][2]

                if abs(dx) > 0.005 or abs(dy) > 0.005 or abs(dz) > 0.005:
                    rospy.logwarn("Grasp-precompute error too large: [{}, {}, {}]".format(
                        dx, dy, dz))
                return True
            else:
                # failure
                rospy.logerr('grasp precompute goal failed: \n%s', repr(myargs))
                return False


