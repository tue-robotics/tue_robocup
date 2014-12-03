#!/usr/bin/python
"""Designators are intended to encapsulate the process of resolving values needed at runtime based on write-time definitions. 
This sound very vague, but imagine a navigation task that needs to navigate to some object ID. 


"""
import roslib; roslib.load_manifest('robot_smach_states')
import rospy

class Designator(object):
    """A Designator defines a goal, which can be defined at runtime or at write-time.
    Its value cannot be set, it can only be get. 
    This allows to later define Designators that take a goal specification, like a query to a world model. 

    current is therefore a property with only a getter.

    >>> d = Designator("Initial value")
    >>> d.current
    'Initial value'
    >>> d.current = 'Error'
    Traceback (most recent call last):
     ...
    AttributeError: can't set attribute"""
    def __init__(self, initial_value=None):
        super(Designator, self).__init__()

        self._current = initial_value

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        return self.current

    def _get_current(self):
        """The currently selected goal"""
        return self._current

    current = property(_get_current)


class VariableDesignator(Designator):
    """A VariableDesignator simply contains a variable that can be set by anyone. 
    This variable is encapsulated by a property called current. 
    You can also set current = ... 

    >>> v = VariableDesignator() 
    >>> v.current = 'Works'
    >>> v.current
    'Works'
    """
    def __init__(self, initial_value=None):
        super(VariableDesignator, self).__init__(initial_value)

    def _set_current(self, value):
        self._current = value

    current = property(Designator._get_current, _set_current)

        
if __name__ == "__main__":
    rospy.init_node('Designator_test', log_level=rospy.INFO)

    import doctest
    doctest.testmod()
