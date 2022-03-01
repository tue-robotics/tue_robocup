# ROS
import actionlib
import rospy
import tf2_ros
import PyKDL as kdl

# TU/e Robotics
from people_recognition_msgs.msg import TrackOperatorAction, TrackOperatorGoal


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
        robot_pose = self._get_robot_pose()
        

    def _get_robot_pose(self) -> kdl.Pose:
        # ToDo: implement using tf buffer
        return kdl.Frame()


if __name__ == "__main__":

    rospy.init_node("mock_operator_tracking")
    tracker = TrackingMocker()
    rospy.spin()
