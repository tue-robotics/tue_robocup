import smach
import rospy

import robot_smach_states.util.designators.core as ds
from robot_smach_states.util.designators import is_writeable


class IterateDesignator(smach.State):
    def __init__(self, collection_des, element_des):
        """Iterate over a designator that resolves to a collection.

        The collection is resolved on the fist time the state is called AND after is was exhausted.

        Once the collection is exhausted, the outcome will be 'stop_iteration'.
        The next time the state is executed, the collection will be resolved again

        :param collection_des: Designator with a iterable resolve_type
        :param element_des: Writeable designator with a resolve_type that should match the element type of the collection

        >>> collection_des = ds.Designator(['a', 'b', 'c'])
        >>> element_des = ds.VariableDesignator(resolve_type=str)
        >>>
        >>> iterator = IterateDesignator(collection_des, element_des.writeable)
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'a'
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'b'
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'c'
        >>>
        >>> assert iterator.execute() == 'stop_iteration'
        >>> assert element_des.resolve() == 'c'
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'a'
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'b'
        >>>
        >>> assert iterator.execute() == 'next'
        >>> assert element_des.resolve() == 'c'
        >>>
        >>> assert iterator.execute() == 'stop_iteration'
        """
        smach.State.__init__(self, outcomes=['next', 'stop_iteration'])

        is_writeable(element_des)

        assert hasattr(collection_des, 'resolve'), "collection_des should have attribute 'resolve'"
        assert hasattr(collection_des.resolve_type, '__iter__'), "collection_des should resolve to an interable type"
        assert collection_des.resolve_type[0] == element_des.resolve_type, "Resolve type of collection and element" \
                                                                           "don't match"

        self.collection_des = collection_des
        self.element_des = element_des

        self._current_elements = None

    def execute(self, userdata=None):
        if self._current_elements is None:
            collection = self.collection_des.resolve()
            if collection is None:
                rospy.logwarn("Collection is None")
                return 'stop_iteration'

            rospy.loginfo("Current elements: {}".format(collection))
            self._current_elements = iter(collection)

        try:
            next_elem = next(self._current_elements)
            rospy.logdebug("Iterate to next element {}".format(next_elem))
            self.element_des.write(next_elem)
            return 'next'
        except StopIteration:
            self._current_elements = None
            return 'stop_iteration'


if __name__ == "__main__":
    import doctest
    doctest.testmod()
