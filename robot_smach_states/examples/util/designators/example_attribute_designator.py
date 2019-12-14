import rospy
import argparse

# Robot Smach States
from robot_smach_states.util.designators.utility import AttrDesignator
from robot_smach_states.util.designators.core import Designator

from collections import namedtuple


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the atttrDesignator")
    args = parser.parse_args()
    rospy.init_node("test_attrDesignator")

    # Example 1 from docstrings
    d = Designator(object(), resolve_type=object)
    #Get the __doc__ attribute of the object that d resolves to. d is an object and d.__doc__ is 'The most base type'
    wrapped = AttrDesignator(d, '__doc__', resolve_type=str)
    wrapped.resolve() == 'The most base type'
    rospy.loginfo(issubclass(wrapped.resolve_type, str))

    # Example 2 from docstrings
    d2 = Designator("banana", resolve_type=str)
    wrapped2 = AttrDesignator(d2, '__class__.__class__', resolve_type=type)
    wrapped2.resolve()
    rospy.loginfo(wrapped2.resolve())

    # Example 3 from docstrings

    A = namedtuple("A", ['foo'])
    B = namedtuple("B", ['bar'])
    a = A(foo=B(bar='banana'))
    d3 = Designator(a, resolve_type=A)
    wrapped3 = AttrDesignator(d3, 'foo.bar', resolve_type=str)
    rospy.loginfo(wrapped3.resolve())
