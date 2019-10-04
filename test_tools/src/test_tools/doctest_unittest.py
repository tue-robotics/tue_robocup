#! /usr/bin/env python

import doctest
import importlib
import os
import rospkg
import unittest


class TestDocTests(unittest.TestCase):

    def __init__(self, module_name):
        super(TestDocTests, self).__init__("test_doctests")
        self.module_name = module_name

    def test_doctests(self):
        """
        Iterates over all Python files in module_name/src/module_name and runs doctest.testmod
        """

        path = rospkg.RosPack().get_path(self.module_name)
        path = os.path.join(path, "src", self.module_name)

        for root, dirs, files in os.walk(path):
            for filename in files:
                if filename.endswith(".py") and "__init__" not in filename:
                    filepath = os.path.join(root, filename)
                    module_name = self.module_name + filepath.split(self.module_name)[-1]
                    module_name = module_name[:-3]
                    module_name = module_name.replace("/", ".")
                    mod = importlib.import_module(module_name)
                    doctest.testmod(mod, report=1)

        failed_count, attempted_count = doctest.master.summarize(True)
        print("Attempted count: {}".format(attempted_count))
        self.assertEqual(failed_count, 0, "{} out of {} tests failed".format(failed_count, attempted_count))
