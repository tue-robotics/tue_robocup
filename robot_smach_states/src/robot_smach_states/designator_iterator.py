from __future__ import absolute_import

import smach
import rospy

import robot_smach_states.util.designators.core as ds
from robot_smach_states.util.designators import is_writeable


class IterateDesignator(smach.State):
    def __init__(self, collection_des, element_des):
        """
        Iterate over a designator that resolves to a collection.

        The collection is resolved on each iteration. When the resolved collection is different from the collection in use,
        it will start from the beginning of the new collection. It keeps iterating the new collection in futher calls.

        Once the collection is exhausted, the outcome will be 'stop_iteration'.
        The next time the state is executed, the collection will be resolved again

        :param collection_des: Designator with a iterable resolve_type
        :param element_des: Writeable designator with a resolve_type that should match the element type of the collection

        >>> collection_des = ds.VariableDesignator(['a', 'b', 'c']).writeable
        >>> element_des = ds.VariableDesignator(resolve_type=str)
        >>> iterator = IterateDesignator(collection_des, element_des.writeable)

        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'a'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'b'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'c'
        >>> iterator.execute()
        'stop_iteration'
        >>> element_des.resolve()
        'c'

        The iterator will start from the beginning of the collection again
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'a'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'b'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'c'
        >>> iterator.execute()
        'stop_iteration'

        >>> collection_des.write(['a', 'b', 'c'])
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'a'

        >>> collection_des.write(['d', 'e', 'f', 'g'])
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'd'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'e'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'f'
        >>> iterator.execute()
        'next'
        >>> element_des.resolve()
        'g'
        >>> iterator.execute()
        'stop_iteration'

        >>> collection_des2 = ds.VariableDesignator(resolve_type=[str]).writeable
        >>> collection_des2.resolve()  # Should resolve to None
        >>> element_des2 = ds.VariableDesignator(resolve_type=str)
        >>> iterator2 = IterateDesignator(collection_des2, element_des2.writeable)
        >>> iterator2.execute()
        'stop_iteration'
        """
        smach.State.__init__(self, outcomes=['next', 'stop_iteration'])

        is_writeable(element_des)

        assert hasattr(collection_des, 'resolve'), "collection_des should have attribute 'resolve'"
        assert hasattr(collection_des.resolve_type, '__iter__') and \
               hasattr(collection_des.resolve_type, '__getitem__') and \
               not isinstance(collection_des.resolve_type, type), f"collection_des of type " \
                                                                  f"{collection_des.resolve_type} should resolve to " \
                                                                  f"an iterable type"
        assert collection_des.resolve_type[0] == element_des.resolve_type, "Resolve type of collection and element" \
                                                                           "don't match"

        self.collection_des = collection_des
        self.element_des = element_des

        self._current_collection = None
        self._current_elements = None

    def execute(self, userdata=None):
        collection = self.collection_des.resolve()
        if collection != self._current_collection:
            rospy.loginfo(f"Collection has changed, current elements: {collection}")
            self._current_collection = collection
            self._current_elements = None

        if self._current_elements is None:
            if self._current_collection is None:
                rospy.logwarn("Collection is None")
                return 'stop_iteration'

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
