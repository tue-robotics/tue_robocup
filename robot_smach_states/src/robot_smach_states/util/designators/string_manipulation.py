import rospy
import copy
from robot_smach_states.util.designators.checks import check_type
from robot_smach_states.util.designators.core import Designator
from hmi import HMIResult


class FieldOfHMIResult(Designator):
    """
    Extract a field of a QueryResult

    >>> query_result = HMIResult(sentence='ignored', semantics={u'widget': {u'gadget': {u'bla': u'foo', u'bar': u'buzz'}}})
    >>> query_des = Designator(query_result, name="d1")
    >>> field_des = FieldOfHMIResult(query_des, semantics_path=['widget', 'gadget', 'bar'])
    >>> field_des.resolve()
    'buzz'

    >>> query_result2 = HMIResult(sentence='ignored', semantics=u'dinges')
    >>> query_des2 = Designator(query_result2, name="d2")
    >>> field_des2 = FieldOfHMIResult(query_des2, semantics_path=[])
    >>> field_des2.resolve()
    'dinges'
    """
    def __init__(self, query_result_des, semantics_path=None, name=None):
        """
        Construct a designator that picks a field out of the semantics dict of a QueryResult
        (such as resulting from a HearOptionsExtra-state)

        :param query_result_des: A QueryResult or a designator resolving to a QueryResult
        :param semantics_path: str or [str] (or designator) used in query_result.semantics[semantics_path]
        :param name: str naming for this designator for debugging purposes
        """
        super(FieldOfHMIResult, self).__init__(resolve_type=str, name=name)

        check_type(query_result_des, HMIResult)

        if semantics_path is not None:
            if semantics_path == []:
                # In this case, the path is just empty and apparently we do not want to traverse a
                # path into a dict but have the thing itself?
                pass
            else:
                if isinstance(semantics_path, str):
                    semantics_path = [semantics_path]
                check_type(semantics_path, [str])
        else:
            rospy.logerr("No semantics path given")

        self.query_result_des = query_result_des
        self.semantics_path = semantics_path

    def _resolve(self):
        try:
            path = self.semantics_path.resolve() if hasattr(self.semantics_path, 'resolve') else self.semantics_path
            semantics = self.query_result_des.resolve().semantics # type: dict, nested
            rospy.loginfo("Going to traverse path: {} into semantics: {}".format(path, semantics))
            level = copy.deepcopy(semantics)  # Make a copy
            if path is not None:
                for step in path:
                    level = level[step]
            return str(level)
        except Exception as e:
            rospy.logerr(e)
            return None


if __name__ == "__main__":
    import doctest
    doctest.testmod()
