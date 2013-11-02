#!/usr/bin/python
"""Designators are intended to excapsulate the process of selecting goals and 
of marking them as finished or inachievable.

TODO:
Designators can be refined by a user of it. 
This means that the query for it is extended, for example. 
Some variables in the query can also be made constants, as to make the query more specific."""
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
from psi  import Conjunction, Compound

import robot_smach_states.util.reasoning_helpers as urh
import robot_skills.util.msg_constructors as msgs

def iter_children(term):
    if term.is_variable() or term.is_constant():
        yield term
    if term.is_compound() or term.is_sequence():
        for child in term.arguments:
            for grandchild in iter_children(child):
                yield grandchild

class NoAnswerException(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)

class Designator(object):
    """A Designator defines a goal, which can be defined at runtime or at write-time.
    A Designator can hold a set of goals it can choose from and finally selects one, by calling self.next.
    After that goal is finished or cannot be finished, a next option can be selected again.
    """
    def __init__(self):
        super(Designator, self).__init__()

        self._current = None

    def resolve(self):
        """Selects a new goal and sets it as the current goal"""
        self._current = None
        raise NotImplementedError("No method for goal selection implemented")
        return self.current

    @property
    def current(self):
        """The currently selected goal"""
        return self._current

class ReasonedDesignator(Designator):
    """docstring for ReasonedPose2DDesignator"""
    def __init__(self, reasoner, query, identifier="Object", sort=min, _filter=lambda x: True):
        """A ReasonedDesignator can be used to refer to an object in the world model,
            whose identity, ID, is not yet known. 
        Each world-model object has an ID: the identifier-variable will be unified with an object's ID
        
        Use the reasoner to find answers to the given query.
        These answers are then filtered and finally sorted by the respective functions.
        """
        
        super(ReasonedDesignator, self).__init__()
        
        self.query = query
        self.reasoner = reasoner
        self.sort = sort
        self.filter = _filter

        self.identifier = identifier

        self._decorated_query = query #This will be modified by the designator itself

        self.processed_identifiers = []

    def resolve(self):
        """Pose the query to the reasoner and filter, sort the answers, returning the best answer"""
        answers = self.reasoner.query(self._decorated_query)
        if not answers:
            raise NoAnswerException("No answer for query {0}".format(self._decorated_query))
        filtered = filter(self.filter, answers)
        if not filtered:
            raise NoAnswerException("No answers left for query {0} after filtering".format(self._decorated_query))
        self._current = self.sort(filtered)
        return self.current

    def extend(self, conjunct):
        """Extend the current query with another conjunct"""
        self._decorated_query = Conjunction(self._decorated_query, conjunct)

    def refine(self, variable, constant):
        """Set a variable in the query to a constant.
        >>> query = Compound("some_predicate", "Object", "B")
        >>> desig = ReasonedDesignator(None, query, "Object")
        >>> desig.refine("Object", "specific_object")
        Compound(',', Compound('some_predicate', Variable('Object'), Variable('B')), Compound('=', Variable('Object'), 'specific_object'))
        """
        
        # for child in iter_children(self._decorated_query):
        #     if child.is_variable():
        #         if child.var_name == variable:
        #             child = constant
        # return self._decorated_query
        # should return Compound('some_predicate', 'specific_object', Variable('B'))
        #But, replacing items in a Term is not that easy.
        self.extend(Compound("=", variable, constant))
        return self._decorated_query

    # def loosen(self, constant, variable):
    #     """Make some constant in the query a variable"""
    #     pass


class ReasonedPose2DDesignator(ReasonedDesignator):
    """docstring for ReasonedPose2DDesignator"""
    def __init__(self, reasoner, query,
        X="X", Y="Y", Phi="Phi", identifier="Object",
        finished_marker='visited', blocked_marker='unreachable',
        **kwargs):
        
        super(ReasonedPose2DDesignator, self).__init__(reasoner, query, 
            identifier=identifier,
            **kwargs)

        self.X = X
        self.Y = Y
        self.Phi = Phi

    @property
    def current_as_pose(self):
        """The current answer interpreted as a PoseStamped"""
        pose = msgs.PoseStamped(
                x   = float(self.current[self.X]), 
                y   = float(self.current[self.Y]), 
                phi = float(self.current[self.Phi]))
        return pose


class ReasonedPointDesignator(ReasonedDesignator):
    """docstring for ReasonedPointDesignator"""
    def __init__(self, reasoner, query,
        X="X", Y="Y", Z="Z", identifier="Object",
        finished_marker='visited', blocked_marker='unreachable',
        **kwargs):
        
        super(ReasonedPointDesignator, self).__init__(reasoner, query, 
            identifier=identifier,
            **kwargs)

        self.X = X
        self.Y = Y
        self.Z = Z

    @property
    def current_as_pose(self):
        """The current answer interpreted as a PointStamped"""
        point = msgs.PointStamped(
                x = float(self.current[self.X]), 
                y = float(self.current[self.Y]), 
                z = float(self.current[self.Z]))
        return point

        
if __name__ == "__main__":
    import doctest
    doctest.testmod()

    rospy.init_node('Designator_test', log_level=rospy.INFO)

    import robot_skills.amigo
    robot = robot_skills.amigo.Amigo(dontInclude=["arms", "head", "perception", "spindle","speech","leftArm","rightArm","ears","ebutton","lights"])

    r = robot.reasoner

    #r.query(r.load_database("tue_knowledge", 'prolog/locations.pl'))
    #r.assertz(r.challenge("clean_up"))
    import math

    r.assertz(r.waypoint("a", r.pose_2d(1, 2, 1.57)))
    r.assertz(r.waypoint("b", r.pose_2d(2, 3, 2.25)))
    r.assertz(r.waypoint("c", r.pose_2d(3, 4, math.pi)))

    r.assertz(r.point_of_interest("a", r.point_3d(1, 2, 4)))
    r.assertz(r.point_of_interest("b", r.point_3d(2, 3, 5)))
    r.assertz(r.point_of_interest("c", r.point_3d(3, 4, 6)))

    pose_query = r.waypoint("Name", r.pose_2d("X", "Y", "Phi"))
    roi_query = r.point_of_interest("Name", r.point_3d("X", "Y", "Z"))

    rpd = ReasonedPose2DDesignator(robot.reasoner, pose_query, identifier="Name")
    rrd = ReasonedPointDesignator(robot.reasoner, roi_query, identifier="Name")

    #import ipdb; ipdb.set_trace()
    answer1 = rpd.resolve()
    rpd.refine("Name", 'c')
    answer2 = rpd.resolve()
    print answer1
    print answer2
    assert answer1 != answer2 and len(answer1) == len(answer2)
    assert str(answer2["Name"]) == 'c'

    print rpd.current_as_pose

    #print "cleaning up..."
    r.query(r.retractall(r.visited("X")))
    r.query(r.retractall(r.unreachable("X")))

    r.query(r.retractall(r.waypoint("X", "Y")))
    r.query(r.retractall(r.point_of_interest("X", "Y")))
    #print "Done"
