from __future__ import absolute_import

# ROS
import rospy

# TU/e Robotics
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint


def combine_constraints(func_list):
    """
    Combine multiple navigation constraints into a single position and orientation constraint

    :param func_list: list of constraint functions each one to be resolved without parameters
     and resolving to a tuple(PositionConstraint, OrientationConstraint)
    :return: navigation constraints
    :rtype: tuple(PositionConstraint, OrientationConstraint)
    """
    pc = None
    oc = None
    for func in func_list:
        constraint = func()
        if not constraint:
            rospy.logerr("function: {} in combine_constraints resulted in {}".format(func, constraint))
            continue

        pci = constraint[0]
        oci = constraint[1]

        if pci:
            if not pc:
                pc = pci
            else:
                pc = combine_position_constraints(pc, pci)
        if oci:
            if not oc:
                oc =oci
            else:
                oc = combine_orientation_constraints(oc, oci)

    if pc or oc:
        # if one is missing fill it in.
        if not pc:
            pc = PositionConstraint()
        if not oc:
            oc = OrientationConstraint()
        return pc, oc
    else:
        rospy.logerr("combine_constraints function did not resolve to any constraints! functions combined were {}".format(func_list))
        return None


def combine_position_constraints(pos1, pos2):
    """
    Combine two navigation constraints into one

    :param pos1: PositionConstraint
    :param pos2: PositionConstraint
    :return: PositionConstraint
    """
    assert (pos1.frame == pos2.frame), "frames of different position constraints must be the same"
    return PositionConstraint(constraint=pos1.constraint + " and " + pos2.constraint, frame=pos1.frame)


def combine_orientation_constraints(oc1, oc2):
    """
    Combine two orientation constraints into one

    :param oc1: OrientationConstraint
    :param oc2: OrientationConstraint
    :return: OrientationConstraint
    """
    rospy.logerr("Tried to combine {} and {}. Orientation constraints cannot be combined at this time".format(oc1, oc2))
    return oc1
