# ROS
import copy

import rospy

# TU/e Robotics
import hero_msgs.srv

# Robot skills
import tf

from ..arms import Arm
from ..util.kdl_conversions import kdl_frame_stamped_to_pose_stamped_msg, pose_msg_to_kdl_frame


class HeroArm(Arm):
    def __init__(self, robot_name, tf_listener, get_joint_states):
        """
        Specific Hero arm object

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param get_joint_states: get_joint_states function for getting the last joint states
        """
        super(HeroArm, self).__init__(robot_name=robot_name, tf_listener=tf_listener,
                                      get_joint_states=get_joint_states, side="left")

        self._base_offset_client = self.create_service_client("/{}/compute_base_pose".format(robot_name),
                                                              hero_msgs.srv.GetBasePose)

    def get_base_offset(self, end_effector_goal):
        # type: (FrameStamped) -> kdl.Vector
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        By default, this method returns the (static) base offset. This should/might be overwritten by robot-specific
        implementations.

        :param end_effector_goal: goal where the end-effector should go
        :return: Position of an object w.r.t. the base link of a robot.
        """
        try:
            return self._get_base_offset(end_effector_goal)
        except Exception as e:
            rospy.logwarn("Computing base offset failed: {}, using default {}".format(
                e.message, self._base_offset
            ))
            return self._base_offset

    def _get_base_offset(self, end_effector_goal):
        # type: (FrameStamped) -> kdl.Vector
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        :param end_effector_goal: goal where the end-effector should go
        :return: Position of an object w.r.t. the base link of a robot.
        """
        assert end_effector_goal.frame_id.endswith("map"), "It is assumed that the provided goal is defined w.r.t." \
                                                           " map, not {}".format(end_effector_goal.frame_id)

        request = hero_msgs.srv.GetBasePoseRequest()
        request.end_effector_pose = kdl_frame_stamped_to_pose_stamped_msg(end_effector_goal)
        response = self._base_offset_client(request)  # type: hero_msgs.srv.GetBasePoseResponse

        # Convert this frame to map frame (assumed in end-effector-goal)
        # base_pose_map
        listener = self.tf_listener  # type: tf.TransformListener
        base_pose_response_frame = copy.deepcopy(response.base_pose)
        base_pose_response_frame.header.stamp = rospy.Time(0)
        base_pose_map = listener.transformPose(end_effector_goal.frame_id, base_pose_response_frame)

        # Compute the offset
        # base_pose * offset = end_effector_pose
        # offset = base_pose.Inverse() * end_effector_pose
        base_pose_map_kdl = pose_msg_to_kdl_frame(base_pose_map.pose)
        return base_pose_map_kdl.Inverse() * end_effector_goal.frame.p


