#ROS
import rospy
import argparse

# TU/e Robotics
from hmi import HMIResult

# Robot Smach States
from robot_smach_states.util.designators.string_manipulation import FieldOfHMIResult
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the string manipulation")
    args = parser.parse_args()

    rospy.init_node("Set_gripper_state")

    # Test 1 from docstrings
    query_result = HMIResult(sentence='ignored', semantics={u'widget': {u'gadget': {u'bla': u'foo', u'bar': u'buzz'}}})
    query_des = Designator(query_result, name="d1")
    field_des = FieldOfHMIResult(query_des, semantics_path=['widget', 'gadget', 'bar'])
    rospy.loginfo(field_des.resolve())

    # Test 2 from docstrings
    query_result2 = HMIResult(sentence='ignored', semantics=u'dinges')
    query_des2 = Designator(query_result2, name="d2")
    field_des2 = FieldOfHMIResult(query_des2, semantics_path=[])
    rospy.loginfo(field_des2.resolve())
