#ROS
import rospy
import argparse

# Robot Smach States
from robot_smach_states.util.designators.utility import ValueByKeyDesignator
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the value by key designator")
    args = parser.parse_args()
    rospy.init_node("value_by_key_designator")

    random_dict = {'foo': 'bar', ' hello': 'bye'}
    container = Designator(random_dict, name='des1')
    des = ValueByKeyDesignator(container=container, key='foo', resolve_type=str)
    rospy.loginfo(des.resolve())
