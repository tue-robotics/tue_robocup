#!/usr/bin/python
"""Designators are intended to excapsulate the process of selecting goals and 
of marking them as finished or inachievable"""
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
from psi  import Conjunction, Compound

import robot_smach_states.util.reasoning_helpers as urh
import robot_smach_states.util.msg_constructors as msgs
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

class ReasonedDesignator(Designator):
    """docstring for ReasonedPoseDesignator"""
    def __init__(self, robot, query, identifier="Object",
        finished_marker='visited', blocked_marker='unreachable'):
        
        super(ReasonedDesignator, self).__init__()
        
        self.query = query
        self.robot = robot

        self.identifier = identifier

        self.decorated_query = Conjunction(self.query, 
                        Compound("not", Compound("visited",     identifier)),
                        Compound("not", Compound("unreachable", identifier)))

        self.processed_identifiers = []

        self.finished_marker = finished_marker
        self.blocked_marker = blocked_marker

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

class ReasonedPoseDesignator(ReasonedDesignator):
    """docstring for ReasonedPoseDesignator"""
    def __init__(self, robot, query,
        X="X", Y="Y", Phi="Phi", identifier="Object",
        finished_marker='visited', blocked_marker='unreachable'):
        
        super(ReasonedPoseDesignator, self).__init__(robot, query, 
            identifier=identifier,
            finished_marker='visited', 
            blocked_marker='unreachable')

        self.X, self.Y, self.Phi = X, Y, Phi

        self.current_answer = {self.X:0.0,self.Y:0.0,self.Phi:0.0,self.identifier:"none"}

    def resolve(self):
        """Gets the answer to the query and selects the best one, depending on self.sorter"""
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.decorated_query)

        basepos = self.robot.base.location.pose.position
        basepos = (basepos.x, basepos.y, 0) #Just set phi to 0 for a sec

        self.current_answer = urh.select_answer(answers, 
                                            lambda answer: urh.xyphi_dist(answer, basepos), 
                                            minmax=min)
        x,y,phi = urh.answer_to_tuple(self.current_answer, variable_ordering=[self.X, self.Y, self.Phi])

        self._current = self.robot.base.point(x,y), self.robot.base.orient(phi)

        return self.current


class ReasonedRoiDesignator(ReasonedDesignator):
    """Designator for an ROI. instance.current is of type PointStamped"""
    def __init__(self, robot, query,
        X="X", Y="Y", Z="Z", identifier="Object",
        finished_marker='visited', blocked_marker='unreachable'):
        
        super(ReasonedRoiDesignator, self).__init__(robot, query, 
            identifier=identifier,
            finished_marker='visited', 
            blocked_marker='unreachable')

        self.X, self.Y, self.Z = X, Y, Z

        self.current_answer = {self.X:0.0,self.Y:0.0,self.Z:0.0,self.identifier:"none"}

    def resolve(self):
        """Gets the answer to the query and selects the best one, depending on self.sorter"""
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.decorated_query)

        basepos = self.robot.base.location.pose.position
        basepos = (basepos.x, basepos.y, 0) #Base is always at Z=0

        self.current_answer = urh.select_answer(answers, 
                                            lambda answer: urh.xyz_dist(answer, basepos), 
                                            minmax=min)
        x,y,z = urh.answer_to_tuple(self.current_answer, variable_ordering=[self.X, self.Y, self.Z])

        self._current = msgs.PointStamped(x, y, z)

        return self.current

        
if __name__ == "__main__":
    rospy.init_node('Designator_test', log_level=rospy.DEBUG)

    import robot_skills.amigo
    robot = robot_skills.amigo.Amigo(dontInclude=["arms", "head", "perception", "spindle","speech","leftArm","rightArm","ears","ebutton","lights"])

    r = robot.reasoner

    r.query(r.load_database("tue_knowledge", 'prolog/locations.pl'))
    r.assertz(r.challenge("clean_up"))

    r.assertz(r.base_pose("a", r.pose_2d(1, 2, 1.57)))
    r.assertz(r.base_pose("b", r.pose_2d(2, 3, 2.25)))
    r.assertz(r.base_pose("c", r.pose_2d(3, 4, 3.14)))

    pose_query = r.base_pose("Name", r.pose_2d("X", "Y", "Phi"))
    roi_query = r.point_of_interest("Name", r.point_3d("X", "Y", "Z"))

    rpd = ReasonedPoseDesignator(robot, pose_query, identifier="Name")
    rrd = ReasonedRoiDesignator(robot, pose_query, identifier="Name")

    r.query(r.retractall(r.visited("X")))
    r.query(r.retractall(r.unreachable("X")))

    import ipdb; ipdb.set_trace()
    while True: #Die variable word niet goed gelezen door de variable oid.
        try:
            rpd.resolve()
            rpd.finish_current()
        except ValueError:
            break
    rpd.unmark_all()
    print "OK"



