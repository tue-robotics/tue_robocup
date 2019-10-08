import unittest

from robot_smach_states.designator_iterator import IterateDesignator
import robot_smach_states.util.designators as ds



collection_des = ds.Designator(['a', 'b', 'c'])
element_des = ds.VariableDesignator(resolve_type=str)

iterator = IterateDesignator(collection_des, element_des.writeable)

assert iterator.execute() == 'next'
assert element_des.resolve() == 'a'

assert iterator.execute() == 'next'
assert element_des.resolve() == 'b'

assert iterator.execute() == 'next'
assert element_des.resolve() == 'c'

assert iterator.execute() == 'stop_iteration'
assert element_des.resolve() == 'c'

assert iterator.execute() == 'next'
assert element_des.resolve() == 'a'

assert iterator.execute() == 'next'
assert element_des.resolve() == 'b'

assert iterator.execute() == 'next'
assert element_des.resolve() == 'c'

assert iterator.execute() == 'stop_iteration'
