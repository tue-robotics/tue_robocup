#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import threading
import std_srvs.srv
import swi_prolog.srv
import rospkg

class Reasoner(object):
    """Interface to the robot's reasoner.
    Converts __getattr__-calls into terms"""
    def __init__(self, robot_name, wait_service=False):
        self._lock = threading.RLock()
        if wait_service:
            rospy.loginfo("Waiting for /reasoner/query service")
            rospy.wait_for_service("/"+robot_name+"/reasoner/query", timeout=2.0)
            rospy.loginfo("service connected.")

        self.sv_reasoner = rospy.ServiceProxy("/"+robot_name+"/reasoner/query", swi_prolog.srv.Query)


    def close(self):
        pass

    def query(self, term):
        res = []
        res_msg = self.sv_reasoner(term)

        for bindings_msg in res_msg.bindings:
            bindings = {}
            for i in range(0, len(bindings_msg.variables)):
                bindings[bindings_msg.variables[i]] = bindings_msg.values[i]
            res += [bindings]

        return res

    def assertz(self, *facts):
        for fact in facts:
            self.query("assertz(" + str(fact) + ")")

    def exists_predicate(self, predicatename):
        raise NotImplementedError()

    # filename is relative to the provided rospkg
    def load_database(self, pkg, filename):
        full_filename = str(rospkg.RosPack().get_path(pkg)) + "/" + filename
        self.query("consult('" + str(full_filename) + "')")

if __name__ == "__main__":
    rospy.init_node("reasoner_executioner", log_level=rospy.DEBUG)
    reasoner = Reasoner()

    reasoner.query("retractall(foo(_, _))")

    reasoner.assertz("foo(bla, 1)")
    reasoner.assertz("foo(blop, 2)")

    reasoner.load_database("tue_knowledge", "prolog/locations.pl")

    print reasoner.query("foo(X, Y)")
