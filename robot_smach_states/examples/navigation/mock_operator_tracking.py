# System
import argparse

# ROS
import actionlib
import rospy
import tf2_ros
import PyKDL as kdl

# TU/e Robotics
from people_recognition_msgs.msg import TrackOperatorAction, TrackOperatorGoal, TrackOperatorFeedback


class TrackingMocker:
    def __init__(self, robot_name):
        """
        Class to mock operator tracking to allow easy testing

        :param robot_name: name of the robot
        """
        self._robot_name = robot_name
        self._as = actionlib.SimpleActionServer(
            f"/{robot_name}/track_operator", TrackOperatorAction, execute_cb=self._execute, auto_start=False
        )
        self._as.start()

        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _execute(self, goal: TrackOperatorGoal):
        rospy.loginfo("Received tracking goal, start tracking")
        # robot_pose = self._get_robot_pose()
        # ToDo: get rid of hardcoding
        start_pos = kdl.Vector(1.0, 0.4, 0.0)
        current_pos = kdl.Vector(start_pos)
        distance = (current_pos - start_pos).Norm()
        dt = 0.5
        r = rospy.Rate(1./dt)  # Assumed detection rate
        velocity = 1.0  # Assumed operator velocity
        while distance < 5.0:
            current_pos.x(current_pos.x() + velocity * dt)
            rospy.loginfo(f"Current operator pos: {current_pos}")
            distance = (current_pos - start_pos).Norm()
            self._publish_feedback(current_pos)
            r.sleep()

        rospy.loginfo(f"Stopped walking after {distance} meters")
        self._as.set_succeeded()

    # def _get_robot_pose(self) -> kdl.Pose:
    #     # ToDo: implement using tf buffer
    #     return kdl.Frame()

    def _publish_feedback(self, position):
        feedback_msg = TrackOperatorFeedback()
        feedback_msg.current_pose.header.frame_id = "map"
        feedback_msg.current_pose.header.stamp = rospy.Time.now()
        feedback_msg.current_pose.pose.position.x = position.x()
        feedback_msg.current_pose.pose.position.y = position.y()
        feedback_msg.current_pose.pose.position.z = position.z()
        feedback_msg.current_pose.pose.orientation.w = 1.0
        self._as.publish_feedback(feedback_msg)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the FollowOperator state")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("mock_operator_tracking")
    tracker = TrackingMocker(args.robot)
    rospy.spin()
