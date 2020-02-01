#! /usr/bin/env python
# ROS
import rospy

# System
import operator

# TU/e Robotics
from robot_smach_states.util.designators.core import Designator, VariableDesignator

__author__ = 'loy'


class LockingDesignator(Designator):
    """A designator's resolve() method may return a different object everytime.
    For some cases, this may be unwanted because a process has to be done with the same object.
    In that case, a designator resolving to a different object every time is not usable.
    A LockingDesignator will resolve to the same object after a call to .lock() and
    will only resolve to a different value after an unlock() call.

    >>> varying = VariableDesignator(0, int).writeable #To be able to write to a designator, it must be writeable!
    >>> locking = LockingDesignator(varying)
    >>> assert(varying.resolve() == 0)
    >>> assert(locking.resolve() == 0)
    >>>
    >>> locking.lock()
    >>>
    >>> varying.write(1)
    >>> assert(varying.resolve() == 1)  # The value changed
    >>> assert(locking.resolve() == 0)  # This one sticks to the value it had when locked
    >>>
    >>> varying.write(2)
    >>> assert(varying.resolve() == 2)  # The value changed
    >>> assert(locking.resolve() == 0)  # This one sticks to the value it had when locked
    >>>
    >>> locking.unlock()
    >>>
    >>> varying.write(3)
    >>> assert(varying.resolve() == 3)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> locking.lock()
    >>>
    >>> varying.write(4)
    >>> assert(varying.resolve() == 4)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> varying.write(5)
    >>> assert(varying.resolve() == 5)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> locking.unlock()
    >>>
    >>> varying.write(6)
    >>> assert(varying.resolve() == 6)  # The value changed
    >>> assert(locking.resolve() == 6)  # This one sticks to the value it had when locked

    >>> assert(varying.resolve_type == int)
    >>> assert(locking.resolve_type == int)
    """

    def __init__(self, to_be_locked, name=None):
        super(LockingDesignator, self).__init__(resolve_type=to_be_locked.resolve_type, name=name)
        self.to_be_locked = to_be_locked
        self._locked = False
        self._current = None

    def lock(self):
        self._locked = True

    def unlock(self):
        self._current = None
        self._locked = False

    def _resolve(self):
        if self._locked:
            if self._current == None:
                self._current = self.to_be_locked.resolve()
                # rospy.loginfo("{0} locked to {1}".format(self, str(self._current)[:10]))
            return self._current
        else:
            self._current = self.to_be_locked.resolve()
            rospy.loginfo("LockingDesignator '{0}' resolved to {1}, but is *not locked* to it".format(self.name, str(self._current)[:30]))
            return self._current

    def __repr__(self):
        return "LockingDesignator({})".format(str(self.to_be_locked.name))

    def _get_name(self):
        return "lockable({})".format(self.to_be_locked.name)


class AttrDesignator(Designator):
    """Get some attribute of the object a wrapped designator resolves to.
    This can be recursive, so you can specify a chain of nested attributes like <resolved_object>.foo.bar

    For example:
    >>> d = Designator(object(), resolve_type=object)
    >>> #Get the __doc__ attribute of the object that d resolves to. d is an object and d.__doc__ is 'The most base type'
    >>> wrapped = AttrDesignator(d, '__doc__', resolve_type=str)
    >>> wrapped.resolve() == 'The most base type'
    True

    >>> assert(issubclass(wrapped.resolve_type, str))

    >>> d2 = Designator("banana", resolve_type=str)
    >>> wrapped2 = AttrDesignator(d2, '__class__.__class__', resolve_type=type)
    >>> wrapped2.resolve()
    <type 'type'>

    >>> from collections import namedtuple
    >>> A = namedtuple("A", ['foo'])
    >>> B = namedtuple("B", ['bar'])
    >>> a = A(foo=B(bar='banana'))
    >>> d3 = Designator(a, resolve_type=A)
    >>> wrapped3 = AttrDesignator(d3, 'foo.bar', resolve_type=str)
    >>> wrapped3.resolve()
    'banana'

    >>> wrapped3 = AttrDesignator(d3, 'foo.not_bar', resolve_type=str)
    >>> wrapped3.resolve() is None  # Will not resolve and return None
    True
    """

    def __init__(self, orig, attribute, resolve_type=None, name=None):
        super(AttrDesignator, self).__init__(resolve_type=resolve_type, name=name)
        self.orig = orig

        self._get_attr = operator.attrgetter(attribute)  # Generate a function that gives you '.your.attr.path' etc.

    def _resolve(self):
        orig = self.orig.resolve()
        if orig:
            try:
                return self._get_attr(orig)
            except AttributeError as attr_err:
                rospy.logerr(attr_err)
                return None
        else:
            return None


class ValueByKeyDesignator(Designator):
    def __init__(self, container, key, resolve_type, name=None):
        """
        Get a value from a dictionary by it's key

        :param container: any object with a __getitem__ method or a designator that resolves to it
        :param name: Name of the designator for introspection purposes
        """
        super(ValueByKeyDesignator, self).__init__(resolve_type=resolve_type, name=name)
        # TODO: Add type checks to make sure that we can do container[key]
        # OR container.resolve[key]
        self._container = container
        self._key = key

    def _resolve(self):
        # ToDo: possible cases: container=None, Missing key
        container = self._container.resolve()
        if container:
            return container[self._key]
        else:
            return None


class FuncDesignator(Designator):

    """Apply a function to the object a wrapped designator resolves to
    For example:
    >>> d = Designator("Hello")
    >>> wrapped = FuncDesignator(d, len, resolve_type=int) #Determine the len of whatever d resolves to
    >>> wrapped.resolve()
    5

    >>> assert(issubclass(wrapped.resolve_type, int))
    """

    def __init__(self, orig, func, resolve_type=None, name=None):
        super(FuncDesignator, self).__init__(resolve_type=resolve_type, name=name)
        self.orig = orig
        self.func = func

    def _resolve(self):
        orig = self.orig.resolve()
        if orig:
            try:
                return self.func(orig)
            except Exception as e:
                rospy.logerr("Cannot apply function {0} on {1}: {2}".format(self.func, orig, e))
                return None
        else:
            return None


class DeferToRuntime(Designator):
    """Run the given function at runtime. Using Python closures, you can use any variable in scope in this function.
    For example:
    >>> d1 = Designator("world") #Create a designator and assign it to a variable
    >>> prefix = "Hello" #Create a string variable
    >>> def prepend(): return prefix + " " + d1.resolve() #This function takes prefix and d1 as variables from the outer scope!
    >>> d = DeferToRuntime(prepend, resolve_type=str) #A designator to execute the prepend-function
    >>> d.resolve()
    'Hello world'
    """

    def __init__(self, func, resolve_type, name=None):
        super(DeferToRuntime, self).__init__(resolve_type=resolve_type, name=name)
        self.func = func

    def _resolve(self):
        return self.func()


if __name__ == "__main__":
    import doctest
    doctest.testmod()
