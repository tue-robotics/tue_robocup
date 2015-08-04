#! /usr/bin/env python

import inspect
import pprint
import rospy

from robot_smach_states.util.designators.core import Designator

__author__ = 'loy'


class ListElementDesignator(Designator):
    """Wraps a designator(resolve_type=[X]) (i.e.) a list of objects with type X and reduces that to a single item of type X.
    The reduction takes some criteria and sorting into account
    >>> l = Designator([1,2,3,4,5], resolve_type=[int])
    >>> reduce = ListElementDesignator(l, sortkey=lambda elem: elem)
    >>> reduce.resolve()
    1

    >>> l = Designator(["noot", "broer", "aap"], resolve_type=[str]) #
    >>> reduc = ListElementDesignator(l, sortkey=lambda elem: len(elem), minmax=max) #Get the element with the max length
    >>> reduc.resolve()
    'broer'

    >>> l = Designator(["noot", "broer", "aap", "mies"], resolve_type=[str])
    >>> #The criterium removes all elements with an e
    >>> #Get the element with the min length
    >>> reduc = ListElementDesignator(l, sortkey=lambda elem: len(elem), minmax=min, criteriafuncs=[lambda elem: not 'e' in elem])
    >>> reduc.resolve()
    'aap'

    >>> l = Designator(["noot", "broer", "aap", "mies"], resolve_type=[str])
    >>> #The criterium removes all elements with an e
    >>> #Get the element with the min length
    >>> reduc = ListElementDesignator(l) #Not sorting or selecting will just return the 'smallest' element (because min is the default for minmax)
    >>> reduc.resolve()
    'aap'
    """

    def __init__(self, list_designator, sortkey=None, criteriafuncs=None, minmax=min):
        if not len(list_designator.resolve_type) == 1:
            raise TypeError("The list_designator should indicate the type of the list elements, e.g. [str] for a list of str's")
        element_type = list_designator.resolve_type[0]
        super(ListElementDesignator, self).__init__(self, resolve_type=element_type)

        self.list_designator = list_designator
        self.sortkey = sortkey
        self.minmax = minmax
        self.criteriafuncs = criteriafuncs or []

    def resolve(self):
        elements = self.list_designator.resolve()
        if elements:
            for criterium in self.criteriafuncs:
                elements = filter(criterium, elements)
                criterium_code = inspect.getsource(criterium)
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(elements)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([elem for elem in elements])))

            if elements:
                if self.sortkey:
                    self._current = self.minmax(elements, key=self.sortkey)
                else:
                    self._current = self.minmax(elements)
                return self.current

        rospy.logerr("No elements found in {0}".format(self))
        return None


if __name__ == "__main__":
    import doctest
    doctest.testmod()