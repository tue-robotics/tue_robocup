import rospy
import sys
import std_srvs.srv
import robot_smach_states.util.designators as ds

from robot_skills.get_robot import get_robot_from_argv
from robot_smach_states.navigation.guidance import GuideToSymbolic

OPERATOR_AVAILABLE = True


def toggle_operator(_):
    """
    Toggles if the operator is following
    :param _:
    :return:
    """
    global OPERATOR_AVAILABLE
    OPERATOR_AVAILABLE = not OPERATOR_AVAILABLE
    return std_srvs.srv.EmptyResponse()


if __name__ == "__main__":

    assert len(sys.argv) == 3, "Please provide the robot name and the entity id of the object to guide to," \
                               "e.g., 'python guidance.py amigo bed'"

    # Create node, robot and toggle interface
    rospy.init_node("test_guidance")
    r = get_robot_from_argv(1)
    e_id = sys.argv[2]
    rospy.Service("toggle_operator", std_srvs.srv.Empty, toggle_operator)

    # Instantiate GuideToSymbolic machine
    s = GuideToSymbolic(r,
                        {ds.EntityByIdDesignator(r, id=e_id): "in_front_of"},
                        ds.EntityByIdDesignator(r, id=e_id)
                        )

    # Mock the _check_operator method
    execute_state = s.get_children()["EXECUTE_PLAN"]
    execute_state._check_operator = lambda: OPERATOR_AVAILABLE

    rospy.loginfo('\nCall:\nrosservice call /toggle_operator "{}"\n\nto toggle operator availability\n'.format('{}'))

    # Execute the state
    s.execute()
