#! /usr/bin/env python

import doctest
import os
import rospkg
import unittest


class TestDocTests(unittest.TestCase):

    def test_doctests(self):
        """
        Iterates over all Python files in robot_smach_states/src/robot_smach_states and runs doctest.testfile
        """

        base_module_name = "robot_smach_states"
        path = rospkg.RosPack().get_path(base_module_name)
        path = os.path.join(path, "src", base_module_name)

        for root, dirs, files in os.walk(path):
            for filename in files:
                if filename.endswith(".py") and "__init__" not in filename:
                    filepath = os.path.join(root, filename)
                    doctest.testfile(filepath, raise_on_error=False, module_relative=False, report=1)

        failed_count, attempted_count = doctest.master.summarize(True)
        print("Attempted count: {}".format(attempted_count))
        self.assertEqual(failed_count, 0, "{} out of {} tests failed".format(failed_count, attempted_count))


if __name__ == '__main__':
    unittest.main()
