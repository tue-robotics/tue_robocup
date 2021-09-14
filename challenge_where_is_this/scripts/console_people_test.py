from pykdl_ros import VectorStamped
import rospy

from robot_skills import get_robot_from_argv
from robot_smach_states.navigation.guidance import _detect_operator_behind_robot


if __name__ == "__main__":
    rospy.init_node("console_people_test")
    robot = get_robot_from_argv(1)

    a = VectorStamped(-1, 0, 1, rospy.Time.now(), robot.base_link_frame)
    robot.head.look_at_point(a)
    _detect_operator_behind_robot(robot)

