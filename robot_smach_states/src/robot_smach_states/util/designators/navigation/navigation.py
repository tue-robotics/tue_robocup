from __future__ import absolute_import

# System
import abc
from collections import namedtuple

# ROS
import rospy

# TU/e Robotics
from ..core import Designator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint
from .. import check_resolve_type

PoseConstraint = namedtuple('PoseConstraint', 'pc oc')


class NavigationConstraintsDesignator(Designator):
    """
    Designator to provide constraints for navigation.
    This is an abstract class. you cannot create an instance of this

    :param name: name of the designator
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None):
        super(NavigationConstraintsDesignator, self).__init__(resolve_type=PoseConstraint, name=name)

    @abc.abstractmethod
    def _resolve(self):
        pass


class CompoundConstraintsDesignator(NavigationConstraintsDesignator):
    """
    Combine multiple navigation constraints.

    :param name: name of the compound designator
    usage:
        compdes = CompoundConstraintDesignator()
        compdes.add(SomeConstraintDesignator, 'designator_name')
    """
    def __init__(self, name=None):
        super(CompoundConstraintsDesignator, self).__init__(name)
        self.designators = {}

    def _resolve(self):
        if not self.designators:
            rospy.logwarn("No designators included in CompoundConstraintsDesignator {}".format(self._name))
            return None

        pc_string = ""
        pc_frame_id = None
        pc = None
        oc = None
        for key in self.designators:
            constraint = self.designators[key].resolve()
            if not constraint:
                rospy.logerr("Designator: {} in CompoundConstraintdesignator {} did not resolve".format(self.designators[key].name, self.name))
                continue

            pci = constraint.pc
            oci = constraint.oc

            if pci:
                if pc_frame_id:
                    assert (pc_frame_id == pci.frame), "frames of different position constraints must be the same"
                    pc_string += ' and '
                else:
                    pc_frame_id = pci.frame
                pc_string += pci.constraint
            if oci:
                if oc:
                    rospy.logerr("only one orientation constraint allowed! This must be fixed in cb_base_navigation")
                else:
                    oc = oci

        if pc_frame_id:
            pc = PositionConstraint(constraint=pc_string, frame=pc_frame_id)

        if pc_frame_id or oc:
            constraint = PoseConstraint(pc, oc)
            return constraint
        else:
            rospy.logerr("CompoundDesignator {} did not resolve to any constraints".format(self.name))
            return None

    def add(self, constraint_designator, name):
        """
        Add a constraint designator to the compound designator

        :param constraint_designator: Designator to add to the compoound designator
        :param name: name of the designator that is added
        :return: None
        """
        check_resolve_type(constraint_designator, PoseConstraint)
        self.designators[name] = constraint_designator
