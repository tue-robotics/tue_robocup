#! /usr/bin/env python

import doctest
import importlib
import os
import rospkg
import unittest


# def _find_modules_recursive(path):
#     """
#     Recursive generator
#
#     :param path:
#     """
#     for importer, modname, ispkg in pkgutil.iter_modules(path):
#         if ispkg:
#             print("This should do the recursion")
#         else:
#             print("Found submodule %s (is a package: %s)" % (modname, ispkg))
#             yield importer, modname, ispkg


class TestDocTests(unittest.TestCase):

    def test_doctests(self):

        base_module_name = "robot_smach_states"
        path = rospkg.RosPack().get_path(base_module_name)
        path = os.path.join(path, "src", base_module_name)

        failed = False
        for root, dirs, files in os.walk(path):
            for filename in files:
                if filename.endswith(".py") and "__init__" not in filename:
                    filepath = os.path.join(root, filename)
                    # # Remove root
                    # module_name = base_module_name + filepath.split(base_module_name)[-1]
                    # # Remove .py extension
                    # module_name = module_name[:-3]
                    # # Replace slashes
                    # module_name = module_name.replace("/", ".")
                    # # print("Module name: {}".format(module_name))
                    #
                    # # print("Should check {}".format(filepath))
                    # module = importlib.import_module(module_name)

                    doctest.testfile(filepath, raise_on_error=True, module_relative=False)
                    try:
                        # doctest.testmod(module, raise_on_error=True)
                        doctest.testfile(filepath, raise_on_error=True, module_relative=False)
                    except Exception as e:
                        print("\n{}\n".format(e))
                        failed = True

        self.assertFalse(failed)


if __name__ == '__main__':
    unittest.main()
