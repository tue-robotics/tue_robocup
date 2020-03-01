from __future__ import absolute_import

# System
import abc

# ROS
import rospy

# TU/e Robotics
from ..core import Designator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint
from .. import check_resolve_type


class NavigationConstraintsDesignator(Designator):
    """ Designator to provide constraints for navigation.
        This is an abstract method. you cannot create an instance of this
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None):
        super(NavigationConstraintsDesignator, self).__init__(resolve_type=tuple, name=name)

    @abc.abstractmethod
    def _resolve(self):
        pass


class CompoundConstraintsDesignator(NavigationConstraintsDesignator):
    """ Combine multiple navigation constraints
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
        oc = None
        for key in self.designators:
            pci, oci = self.designators[key].resolve()
            if pci:
                if pc_frame_id:
                    assert pc_frame_id == pci.frame
                    pc_string += ' and '
                else:
                    pc_frame_id = pci.frame
                pc_string += pci.constraint
            if oci:
                if oc:
                    rospy.logerr("only one orientation constraint allowed! Tell Peter to fix this")
                else:
                    oc = oci

        pc = PositionConstraint(constraint=pc_string, frame=pc_frame_id)
        return pc, oc

    def add(self, constraint_designator, name):
        check_resolve_type(constraint_designator, tuple)
        self.designators[name] = constraint_designator

