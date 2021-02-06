from test_tools.doctest_unittest import _TestDocTests
import unittest


class DocTestsRobotSkills(_TestDocTests):
    def __init__(self, method_name="test_doctests"):
        super(DocTestsRobotSkills, self).__init__(module_name="robot_skills", method_name=method_name)


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(DocTestsRobotSkills())
    unittest.TextTestRunner(verbosity=2).run(suite)
