# ROS
import rospy

# TU/e Robotics
import hero_msgs.srv

# Robot skills
from ..arms import Arm
from ..util.kdl_conversions import kdl_frame_stamped_to_pose_stamped_msg


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
        request = hero_msgs.srv.GetBasePoseRequest()
        request.end_effector_pose = kdl_frame_stamped_to_pose_stamped_msg(end_effector_goal)
        response = self._base_offset_client(request)
        rospy.logwarn("Base offset response: {}".format(response))


