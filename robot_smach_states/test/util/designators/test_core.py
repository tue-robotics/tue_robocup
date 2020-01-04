#! /usr/bin/env python
import unittest

from robot_smach_states.util.designators.core import Designator, VariableDesignator


class TestDesignator(unittest.TestCase):
    def test_resolve_type(self):
        d1 = Designator("some string", name="tester1", resolve_type=str)
        result = d1.resolve()
        self.assertEqual(result, "some string")

    def test_wrong_resolve_type(self):
        d2 = Designator("not an integer", name="tester2", resolve_type=int)
        with self.assertRaises(TypeError):
            d2.resolve()
        self.assertTrue(issubclass(d2.resolve_type, int))

    def test_list(self):
        v = Designator(['a', 'b', 'c'])
        self.assertEqual(v.resolve_type, [str])
        self.assertListEqual(v.resolve(), ['a', 'b', 'c'])


class TestVariableDesignator(unittest.TestCase):
    def test_basics(self):
        v = VariableDesignator('Hello')
        self.assertEqual(v.resolve_type, str)
        self.assertEqual(v.resolve(),  "Hello")

    def test_current_deprecated(self):
        v = VariableDesignator('Hello')
        with self.assertRaises(DeprecationWarning):
            result = v.current

    def test_write_not_possible(self):
        v = VariableDesignator('Hello')

        with self.assertRaises(DeprecationWarning):
            v.current = "Goodbye"

        self.assertEqual(v.resolve(), "Hello")  # No change

    def test_list(self):
        v = VariableDesignator(['a', 'b', 'c'])
        self.assertEqual(v.resolve_type, [str])
        self.assertListEqual(v.resolve(), ['a', 'b', 'c'])

    def test_none_value(self):
        # Resolve a [object] to a None value
        class MyVariableDesignator(VariableDesignator):
            def __init__(self):
                super(MyVariableDesignator, self).__init__(resolve_type=[object])
            def _resolve(self):
                return None

        v = MyVariableDesignator()
        self.assertEquals(v.resolve(), None)

    def test_empty_list_value(self):
        # Resolve a [object] to an empty list value.
        class MyVariableDesignator(VariableDesignator):
            def __init__(self):
                super(MyVariableDesignator, self).__init__(resolve_type=[str])
            def _resolve(self):
                return []

        v = MyVariableDesignator()
        self.assertEquals(v.resolve(), [])

    def test_none_list_value(self):
        # Resolve a [object] to a [None] list value.
        class MyVariableDesignator(VariableDesignator):
            def __init__(self):
                super(MyVariableDesignator, self).__init__(resolve_type=[str])
            def _resolve(self):
                return [None]

        v = MyVariableDesignator()
        with self.assertRaises(TypeError):
            self.assertEquals(v.resolve(), [None])

    def test_empty_list_type(self):
        with self.assertRaises(TypeError):
            v = VariableDesignator(resolve_type=[])

    def test_none_type_in_list(self):
        with self.assertRaises(TypeError):
            v = VariableDesignator(resolve_type=[None])

    def test_multi_element_type(self):
        with self.assertRaises(TypeError):
            v = VariableDesignator(resolve_type=[str, bool])

    def test_none_type_singular(self):
        with self.assertRaises(TypeError):
            v = VariableDesignator(resolve_type=None)

class TestVariableWriter(unittest.TestCase):
    def test_basics(self):
        v = VariableDesignator('Hello')
        self.assertEqual(v.resolve(),  "Hello")

        w = v.writeable
        w.write("Goodbye")
        self.assertEqual(v.resolve(),  "Goodbye")

    def test_list_write(self):
        v = VariableDesignator(['a', 'b', 'c'])
        vw = v.writeable
        self.assertEqual(vw.resolve_type, [str])
        self.assertListEqual(v.resolve(), ['a', 'b', 'c'])

        vw.write(v.resolve() + ['d'])
        self.assertListEqual(v.resolve(), ['a', 'b', 'c', 'd'])

    def test_list_write_with_wrong_type(self):
        v = VariableDesignator(['a', 'b', 'c'])
        vw = v.writeable
        self.assertEqual(vw.resolve_type, [str])
        self.assertListEqual(v.resolve(), ['a', 'b', 'c'])

        with self.assertRaises(TypeError):
            vw.write([1, 2, 3, 4])


if __name__ == '__main__':
    unittest.main()
