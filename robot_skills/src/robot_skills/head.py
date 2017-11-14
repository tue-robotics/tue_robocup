#! /usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PointStamped
from head_ref.msg import HeadReferenceAction, HeadReferenceGoal

from robot_part import RobotPart

from .util import msg_constructors as msgs
from .util.kdl_conversions import kdlVectorStampedToPointStamped, VectorStamped


class Head(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(Head, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._ac_head_ref_action = self.create_simple_action_client("/"+robot_name+"/head_ref/action_server",
                                                                    HeadReferenceAction)
        self._goal = None
        self._at_setpoint = False

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    # -- Helpers --

    def reset(self, timeout=0):
        """
        Reset head position
        """
        reset_goal = VectorStamped(x=10, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(reset_goal, timeout=timeout)

    def look_at_hand(self, side):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        if side == "left":
            return self.look_at_point(VectorStamped(0, 0, 0, frame_id="/"+self.robot_name+"/grippoint_left"))
        elif side == "right":
            return self.look_at_point(VectorStamped(0, 0, 0, frame_id="/"+self.robot_name+"/grippoint_right"))
        else:
            rospy.logerr("No side specified for look_at_hand. Give me 'left' or 'right'")
            return False

    def look_at_ground_in_front_of_robot(self, distance=2):
        goal = VectorStamped(x=distance, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal)

    def look_down(self, timeout=0):
        """
        Gives a target at z = 1.0 at 1 m in front of the robot
        """
        goal = VectorStamped(1, 0, 0.5, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal, timeout=timeout)

    def look_up(self, timeout=0):
        """
        Gives a target at z = 1.0 at 1 m in front of the robot
        """
        goal = VectorStamped(0.2, 0.0, 4.5, frame_id="/"+self.robot_name+"/base_link")

        return self.look_at_point(goal, timeout=timeout)

    def look_at_standing_person(self, timeout=0):
        """
        Gives a target at z = 1.75 at 1 m in front of the robot
        """
        goal = VectorStamped(1.0, 0.0, 1.6, frame_id="/" + self.robot_name + "/base_link")

        return self.look_at_point(goal, timeout=timeout)

    # -- Functionality --

    def look_at_point(self, vector_stamped, end_time=0, pan_vel=1.0, tilt_vel=0.8, timeout=0):
        assert isinstance(vector_stamped, VectorStamped)
        point_stamped = kdlVectorStampedToPointStamped(vector_stamped)
        self._setHeadReferenceGoal(0, pan_vel, tilt_vel, end_time, point_stamped, timeout=timeout)

    def cancel_goal(self):
        self._ac_head_ref_action.cancel_goal()
        self._goal = None
        self._at_setpoint = False

    def wait_for_motion_done(self, timeout=5.0):
        self._at_setpoint = False
        starttime = rospy.Time.now()
        if self._goal:
            while (rospy.Time.now() - starttime).to_sec() < timeout:
                if self._at_setpoint:
                    rospy.sleep(0.3)
                    return True
                else:
                    rospy.sleep(0.1)
        return False

    # ---- INTERFACING THE NODE ---

    def _setHeadReferenceGoal(self, goal_type, pan_vel, tilt_vel, end_time, point_stamped=PointStamped(), pan=0, tilt=0,
                              timeout=0):
        self.cancel_goal()

        self._goal = HeadReferenceGoal()
        self._goal.goal_type = goal_type
        self._goal.priority = 0  # Executives get prio 1
        self._goal.pan_vel = pan_vel
        self._goal.tilt_vel = tilt_vel
        self._goal.target_point = point_stamped
        self._goal.pan = pan
        self._goal.tilt = tilt
        self._goal.end_time = end_time
        self._ac_head_ref_action.send_goal(self._goal, done_cb=self.__doneCallback, feedback_cb=self.__feedbackCallback)

        start = rospy.Time.now()
        if timeout != 0:
            rospy.logdebug("Waiting for %d seconds to reach target ...".format(timeout))
            while not (not ((rospy.Time.now() - start) < rospy.Duration(timeout)) or self._at_setpoint):
                rospy.sleep(0.1)

    def __feedbackCallback(self, feedback):
        self._at_setpoint = feedback.at_setpoint

    def __doneCallback(self, terminal_state, result):
        self._goal = None
        self._at_setpoint = False

#######################################


if __name__ == "__main__":
    import tf_server
    tf_listener = tf_server.TFClient()
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head("amigo", tf_listener)
