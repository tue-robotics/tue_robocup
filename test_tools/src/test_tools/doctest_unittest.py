import doctest
import importlib
import os
import rospkg
import unittest


class _TestDocTests(unittest.TestCase):
    """
    Parent class, which runs all doctests in a python module. As rospkg is used to locate the module, this test only
    can be applied to catkin packages.
    To create your own test:
    from test_tools.doctest_unittest import _TestDocTests

    class DocTestsModuleA(_TestDocTests):
        def __init__(self, method_name="test_doctests"):
        super(DocTestsModuleA, self).__init__(module_name="ModuleA", method_name=method_name)

    if __name__ == '__main__':
        suite = unittest.TestSuite()
        suite.addTest(DocTestsModuleA())
        unittest.TextTestRunner(verbosity=2).run(suite)

    """

    def __init__(self, module_name, method_name="test_doctests"):
        """
        Constructor

        :param module_name: (str) Name of the python module
        :param method_name: (str) Name of the member variable to run, this should be "test_doctests" and shouldn't
            be changed.
        """
        assert method_name == "test_doctests", "The method_name should be 'test_doctests'. This is the function which" \
                                               "implements the functionality of this TestCase."
        super(_TestDocTests, self).__init__(method_name)
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
