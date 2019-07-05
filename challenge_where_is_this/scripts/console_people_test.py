import sys
from robot_skills.util import kdl_conversions
from robot_skills import get_robot
from robot_smach_states.navigation.guidance import _detect_operator_behind_robot


robot = get_robot(sys.argv[1])

a = kdl_conversions.VectorStamped(-1, 0, 1, '/{}/base_link'.format(sys.argv[1]))
hero.head.look_at_point(a)
_detect_operator_behind_robot(robot)

