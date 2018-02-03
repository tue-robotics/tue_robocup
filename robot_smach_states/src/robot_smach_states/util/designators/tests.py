#! /usr/bin/env python

__author__ = 'loy'

# System
import unittest

# TU/e Robotics
from robot_smach_states.util.designators.core import Designator, VariableDesignator


class MyTestCase(unittest.TestCase):
    def test_visited_and_unreachable(self):
        """In our RoboCup executives, we keep track of which items are 'processed' or visited,
        and which cannot be processed because they are unreachable.

        One way to do this, is to mark objectIDs (targets) as visited and unreachable in the reasoner,
        e.g. a global blackboard that can be manipulated an queried through Prolog.

        This gets messy with complex queries. With designators, it should be a lot simpler"""
        success = lambda goal: goal % 2  # Even targets will fail

        target_list = [i for i in range(10)] + [i for i in range(10)]  # all are in twice!
        targets = Designator(target_list)

        unreachable_set = set()
        unreachable = VariableDesignator(unreachable_set, resolve_type=set)

        visited_set = set()
        visited = VariableDesignator(visited_set, resolve_type=set)

        for target in target_list:
            if target not in unreachable.resolve() and target not in visited.resolve():
                print "Processing target {0}".format(target)
                if success(target):
                    visited.resolve().add(target)  # This is a set so no += [...]
                    print "Target {0} was successfull, its visited".format(target)
                else:
                    unreachable.resolve().add(target)  # This is a set so no += [...]
                    print "Target {0} failed, its unreachable".format(target)
            else:
                print "NOT processing target {0}: its unreachable or already visited".format(target)

        print "##### Done #####"
        print "These are unreachable: {0}".format(unreachable.resolve())
        print "These are visited: {0}".format(visited.resolve())

        self.assertEqual(len(unreachable.resolve()), 5)
        self.assertEqual(len(visited.resolve()), 5)


if __name__ == '__main__':
    unittest.main()
