#! /usr/bin/env python

from test_tools.doctest_unittest import TestDocTests
import unittest


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestDocTests("robot_smach_states"))
    unittest.TextTestRunner(verbosity=2).run(suite)
