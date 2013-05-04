#!/usr/bin/python
"""Designators are intended to excapsulate the process of selecting goals and 
of marking them as finished or inachievable"""
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
from psi  import Conjunction, Compound

import robot_smach_states.util.reasoning_helpers as urh
import robot_smach_states.util
import math

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
        raise StopIteration("No method for goal selection implemented")
        return self.current

    def update_current(self):
        """Keep the same _thing_ as a goal, but update it coordinates or other derivative information"""
        raise NotImplementedError("No method for updating a goal implemented")

    #TODO: Implement current_for_base, current_for_head, current_for_arms?
    @property
    def current(self):
        """The currently selected goal"""
        return self._current

    def finish_current(self):
        """Mark the current goal as finished. Sets self.current to None"""
        self._current = None
        raise NotImplementedError("No method for goal finishing implemented")

    def block_current(self, reason=None):
        """Marks the current goal as blocked or otherwise inachievable"""
        raise NotImplementedError("No method for goal blocking implemented")

    def unmark_all(self):
        """Remove finished and blocked marks from all processed goals"""
        raise NotImplementedError("No method for unmarking potential goals implemented")

class ReasonedPoseDesignator(Designator):
    """docstring for ReasonedPoseDesignator"""
    def __init__(self, robot, query, sorter=None,
        X="X", Y="Y", Phi="Phi", identifier="Object",
        finished_marker='visited', blocked_marker='unreachable'):
        
        super(ReasonedPoseDesignator, self).__init__()
        
        self.query = query
        self.robot = robot

        self.X, self.Y, self.Phi = X, Y, Phi
        self.identifier = identifier

        self.decorated_query = Conjunction(self.query, 
                        Compound("not", Compound("visited",     identifier)),
                        Compound("not", Compound("unreachable", identifier)))

        self.current_answer = {self.X:0.0,self.Y:0.0,self.Phi:0.0,self.identifier:"none"}
        self.processed_identifiers = []

        if not sorter:
            self.sorter = self._calc_dist
        else:
            self.sorter = sorter

        self.finished_marker = finished_marker
        self.blocked_marker = blocked_marker

    def _calc_dist(self, (xg,yg,phig)):
        #TODO: Take rotation into account as well
        loc = self.robot.base.location[0]
        xr,yr,zr = loc.x, loc.y, loc.z
        dist = math.sqrt(abs(xr-xg)**2 + abs(yr-yg)**2)
        return dist

    def resolve(self):
        """Gets the answer to the query and selects the best one, depending on self.sorter"""
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.decorated_query)

        # if not answers:
        #     if len(self.robot.reasoner.query(self.query)) == 0:
        #         #Even the plain query has no answers
        #         rospy.logerr("No answers found for query {query} at all".format(query=self.query))
        #         raise NoAnswerException(self.query)
        #     else:
        #         rospy.logerr("No answers found for query {query} that are not finished and are not blocked.".format(query=self.decorated_query))
        #         raise StopIteration("No answers found for query {query} that are not finished and are not blocked.".format(query=self.decorated_query))
        # else:
        #     possible_locations = [( float(answer[self.X]), 
        #                             float(answer[self.Y]), 
        #                             float(answer[self.Phi])) for answer in answers]

        #     x,y,phi = min(possible_locations, key=self.sorter)
        #     location_index = possible_locations.index((x,y,phi))
        #     self.current_answer = answers[location_index]

        #     rospy.logdebug("Found a pose for '{0}': {1}".format(self.query, (x,y,phi)))
        #     self._current = self.robot.base.point(x,y), self.robot.base.orient(phi)

        #     return self.current

        basepos = self.robot.base.location[0]
        basepos = (basepos.x, basepos.y, 0) #Just set phi to 0 for a sec

        self.current_answer = urh.select_answer(answers, 
                                            lambda answer: urh.xyphi_dist(answer, basepos), 
                                            minmax=min)
        x,y,phi = urh.answer_to_tuple(self.current_answer, variable_ordering=[self.X, self.Y, self.Phi])

        self._current = self.robot.base.point(x,y), self.robot.base.orient(phi)

        return self.current

    def update_current(self):
        x   = float(self.current_answer[self.X]), 
        y   = float(self.current_answer[self.Y]), 
        phi = float(self.current_answer[self.Phi])
        
        rospy.logdebug("Found a pose for '{0}': {1}".format(self.query, (x,y,phi)))
        self._current = self.robot.base.point(x,y), self.robot.base.orient(phi)

        return self.current

    def mark_current(self, mark_with):
        self.robot.reasoner.assertz(Compound(mark_with, self.current_answer[self.identifier]))
        self.processed_identifiers += [self.current_answer[self.identifier]]
        self.current_answer = {self.X:0.0,self.Y:0.0,self.Phi:0.0,self.identifier:"none"}
        self._current = None

    def finish_current(self):
        self.mark_current(self.finished_marker)
    
    def block_current(self, reason=None):
        self.mark_current(self.blocked_marker)

    def unmark_all(self):
        """Retract from the knowledgebase that all poses were tried"""
        asserts = []
        for identifier in self.processed_identifiers:
            finished = Compound("retractall", Compound(self.finished_marker, identifier))
            blocked = Compound("retractall", Compound(self.blocked_marker, identifier))
            asserts += [finished, blocked]
        self.robot.reasoner.query(Conjunction(*asserts))
        
if __name__ == "__main__":
    rospy.init_node('Designator_test', log_level=rospy.DEBUG)

    import robot_skills.amigo
    robot = robot_skills.amigo.Amigo()

    r = robot.reasoner

    r.query(r.load_database("tue_knowledge", 'prolog/locations.pl'))
    r.assertz(r.challenge("clean_up"))

    r.assertz(r.base_pose("a", r.pose_2d(1, 2, 1.57)))
    r.assertz(r.base_pose("b", r.pose_2d(2, 3, 2.25)))
    r.assertz(r.base_pose("c", r.pose_2d(3, 4, 3.14)))

    pose_query = r.base_pose("Name", r.pose_2d("X", "Y", "Phi"))

    rpd = ReasonedPoseDesignator(robot, pose_query, identifier="Name")

    r.query(r.retractall(r.visited("X")))
    r.query(r.retractall(r.unreachable("X")))

    while len(r.query(r._not(r.visited("Name")))):
        rpd.resolve()
        rpd.finish_current()
    rpd.unmark_all()
    print "OK"



