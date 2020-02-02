from numpy import array as np_array
from numpy.linalg import norm as np_norm
import rospy
from geometry_msgs.msg import WrenchStamped
from util.exceptions import TimeOutException

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class ForceSensor(RobotPart):
    def __init__(self, robot_name, tf_listener, topic, force_norm_threshold=2.5):
        """
        Class for conveniently using a force sensor

        :param robot_name: Which robot is this part of?
        :param tf_listener: tf_server.TFClient for  use in RobotPart
        :param topic: Topic to use for WrenchStamped measurement
        :param force_norm_threshold: Edge up if norm exceeds this value [N] (defaults to 2.5N)
        """
        super(ForceSensor, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._topic = topic

        self._calibrated_msg = None
        self._edge_up = False
        self._force_norm_threshold = force_norm_threshold

    def _wrench_callback(self, msg):
        # type: (WrenchStamped) -> None
        """
        Process a WrenchStamped-message to determine if the norm(force) goes over the limit

        :param msg: WrenchStamped to see if it goes over self.force_norm_threshold
        :return: None but sets self.edge_up
        """
        def _detect_edge_up(calibrated_msg, msg):
            """
            Determine if the difference between calibrated_msg and msg is larger than self._force_norm_threshold

            :param calibrated_msg: Reference message
            :param msg: Comparision message, to be compared with the reference message
            :return: bool if the difference is larger than the threshold.
            """
            def _norm(v):
                return np_norm(np_array([v.x, v.y, v.z]))

            calibrated_force_norm = _norm(calibrated_msg.wrench.force)
            force_norm = _norm(msg.wrench.force)

            rospy.logdebug("Force norm: %.2f", force_norm - calibrated_force_norm)

            return abs(force_norm - calibrated_force_norm) > self._force_norm_threshold

        if self._calibrated_msg is None:
            self._calibrated_msg = msg
        else:
            self._edge_up = _detect_edge_up(self._calibrated_msg, msg)

    def wait_for_edge_up(self, timeout=10.0):
        """
        Returns if an edge up event is detected. Will raise a TimeOutException if no edge up is detected within timeout.

        :param timeout: Edge up wait patience
        """
        self._edge_up = False
        self._calibrated_msg = None
        subscriber = rospy.Subscriber(self._topic, WrenchStamped, self._wrench_callback, queue_size=1)

        end_time = rospy.Time.now() + rospy.Duration.from_sec(timeout)
        while not rospy.is_shutdown() and not self._edge_up:
            if rospy.Time.now() > end_time:
                raise TimeOutException("Force sensor edge up detection timeout of {} exceeded".format(timeout))
            rospy.Rate(10).sleep()
        subscriber.unregister()
        rospy.loginfo("Edge up detected!")
