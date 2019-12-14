from test_tools.doctest_unittest import _TestDocTests
import unittest


class DocTestsRobotSmachStates(_TestDocTests):
    def __init__(self, method_name="test_doctests"):
        super(DocTestsRobotSmachStates, self).__init__(module_name="robot_smach_states", method_name=method_name)


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(DocTestsRobotSmachStates())
    unittest.TextTestRunner(verbosity=2).run(suite)
