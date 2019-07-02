#! /usr/bin/env python
import rospy
from robot_smach_states.util.designators.core import Designator
from hmi import HMIResult


class FieldOfHMIResult(Designator):
    """
    Extract a field of a QueryResult
    """
    def __init__(self, query_result_des, semantics_field, name=None):
        """
        Construct a designator that picks a field out of the semantics dict of a QueryResult
        (such as resulting from a HearOptionsExtra-state)
        :param query_result_des: A designator resolving to a QueryResult
        :param semantics_field: str (or string designator) used in query_result.semantics[semantics_field]
        :param name: Name for this designator for debugging purposes
        """
        super(FieldOfHMIResult, self).__init__(resolve_type=str, name=name)

        ds.check_type(query_result_des, HMIResult)
        ds.check_type(semantics_field, str)

        self.query_result_des = query_result_des
        self.semantics_field = semantics_field

    def _resolve(self):
        try:
            field = self.semantics_field.resolve() if hasattr(self.semantics_field, 'resolve') else self.semantics_field
            return str(self.query_result_des.resolve().semantics[field])
        except Exception as e:
            rospy.logerr(e)
            return None
