import unittest

from robot_smach_states.designator_iterator import IterateDesignator
import robot_smach_states.util.designators as ds


class IterateDesignatorTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.collection_des = ds.Designator(['a', 'b', 'c'])
        cls.element_des = ds.VariableDesignator(resolve_type=str).writeable
        cls.iterator = IterateDesignator(cls.collection_des, cls.element_des)

    def test_first_run(self):
        self.iterator._current_elements = None
        assert self.iterator.execute() == 'next'
        assert self.element_des.resolve() == 'a'

    def test_stopiteration_and_second_run(self):
        collection = self.collection_des.resolve()
        self.iterator._current_elements = iter(collection)
        for i in range(len(collection)):
            next(self.iterator._current_elements)
        assert self.iterator.execute() == 'stop_iteration'
        assert self.iterator.execute() == 'next'
        assert self.element_des.resolve() == 'a'

    def test_collection_non_designator(self):
        self.assertRaises(AssertionError, IterateDesignator, ['a', 'b', 'c'],
                          ds.VariableDesignator(resolve_type=str).writeable)

    def test_collection_non_iterable(self):
        self.assertRaises(AssertionError, IterateDesignator, ds.Designator('a'),
                          ds.VariableDesignator(resolve_type=str).writeable)

    def test_element_non_writeable(self):
        self.assertRaises(TypeError, IterateDesignator, self.collection_des,
                          ds.VariableDesignator(resolve_type=str))

    def test_mismatching_resolve_type(self):
        self.assertRaises(AssertionError, IterateDesignator, self.collection_des,
                          ds.VariableDesignator(resolve_type=unicode).writeable)


if __name__ == "__main__":
    unittest.main()
