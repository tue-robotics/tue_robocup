from __future__ import absolute_import

import smach
import math
import rospy

from pykdl_ros import VectorStamped


# TODO: In order to let this state work smoothly, we should take timestamps into account
# TODO: We could access the pose stamped by self._robot.ssl._msg

class SSLLookatAndRotate(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        yaw = self._robot.ssl.get_last_yaw(1.0)
        look_distance = 2.0
        height = 1.7

        if yaw:
            rospy.loginfo("SSL Yaw: %.2f", yaw)

            lookat_vector = VectorStamped.from_xyz(
                x=look_distance * math.cos(yaw),
                y=look_distance * math.sin(yaw),
                z=height,
                stamp=rospy.Time.now(),
                frame_id=self._robot.base_link_frame
            )
            lookat_vector = self._robot.tf_buffer.Transform(lookat_vector, "map")

            self._robot.head.look_at_point(lookat_vector, pan_vel=2.0)

            if yaw > math.pi:
                yaw -= 2 * math.pi
            if yaw < -math.pi:
                yaw += 2 * math.pi

            vyaw = 1.0
            self._robot.base.force_drive(0, 0, (yaw / abs(yaw)) * vyaw, abs(yaw) / vyaw)
        else:
            rospy.loginfo("No SSL Yaw found ...")

        return "done"
