#ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.util.designators.utility import AttrDesignator
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the atttrDesignator")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("Set_gripper_state")

    robot = get_robot(args.robot)

    d = Designator(object(), resolve_type=object)
    #Get the __doc__ attribute of the object that d resolves to. d is an object and d.__doc__ is 'The most base type'
    wrapped = AttrDesignator(d, '__doc__', resolve_type=str)
    wrapped.resolve() == 'The most base type'

    rospy.loginfo(issubclass(wrapped.resolve_type, str))


