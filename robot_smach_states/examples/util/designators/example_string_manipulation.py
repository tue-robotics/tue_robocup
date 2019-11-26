#ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot
from hmi import HMIResult

# Robot Smach States
from robot_smach_states.util.designators.string_manipulation import FieldOfHMIResult
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the string manipulation")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("Set_gripper_state")

    robot = get_robot(args.robot)

    query_result = HMIResult(sentence='ignored', semantics={u'widget': {u'gadget': {u'bla': u'foo', u'bar': u'buzz'}}})
    query_des = Designator(query_result, name="d1")
    field_des = FieldOfHMIResult(query_des, semantics_path=['widget', 'gadget', 'bar'])
    rospy.loginfo(field_des.resolve())
