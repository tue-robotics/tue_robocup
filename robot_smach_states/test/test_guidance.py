import os
import rospy
import sys
import std_srvs.srv
import robot_smach_states.util.designators as ds

from robot_skills.get_robot import get_robot_from_argv
from robot_smach_states.navigation import guidance

OPERATOR_AVAILABLE = True


def toggle_operator(_):
    """
    Toggles if the operator is following
    :param _:
    :return:
    """
    global OPERATOR_AVAILABLE
    OPERATOR_AVAILABLE = not OPERATOR_AVAILABLE
    rospy.loginfo("Operator available: {}".format(OPERATOR_AVAILABLE))
    return std_srvs.srv.EmptyResponse()


if __name__ == "__main__":

    assert len(sys.argv) == 3, "Please provide the robot name and the entity id of the object to guide to," \
                               "e.g., 'python guidance.py amigo bed'"

    # Create node, robot and toggle interface
    rospy.init_node("test_guidance")
    r = get_robot_from_argv(1)
    e_id = sys.argv[2]

    # Instantiate GuideToSymbolic machine
    s = guidance.GuideToSymbolic(r,
                                 {ds.EntityByIdDesignator(r, id=e_id): "in_front_of"},
                                 ds.EntityByIdDesignator(r, id=e_id)
                                 )

    # In simulation, mock the _check_operator method
    if os.getenv("ROBOT_REAL", "false").lower() != "true":
        # noinspection PyUnusedLocal
        def _mock_detect_operator(robot, distance=1.0, radius=0.5):
            return OPERATOR_AVAILABLE
        guidance._detect_operator_behind_robot = _mock_detect_operator
        rospy.Service("toggle_operator", std_srvs.srv.Empty, toggle_operator)
        rospy.loginfo('\n\nCall:\nrosservice call /toggle_operator "{}"\n\nto toggle operator availability\n'.format('{}'))

    # Execute the state
    s.execute()
