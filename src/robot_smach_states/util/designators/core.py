#! /usr/bin/env python
__author__ = 'loy'

class Designator(object):

    """
    A Designator defines a goal, which can be defined at runtime or at write-
    time. Its value cannot be set, it can only be get.  This allows to later
    define Designators that take a goal specification, like a query to a world
    model.

    current is therefore a property with only a getter.

    >>> d = Designator("Initial value")
    >>> d.current
    'Initial value'
    >>> d.current = 'Error'
    Traceback (most recent call last):
     ...
    AttributeError: can't set attribute

    >>> assert(issubclass(d.resolve_type, str))"""

    def __init__(self, initial_value=None, resolve_type=None):
        super(Designator, self).__init__()

        self._current = initial_value
        if not resolve_type:
            self._resolve_type = type(initial_value)
        else:
            self._resolve_type = resolve_type

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        return self.current

    def _get_current(self):
        """The currently selected goal"""
        return self._current

    def _get_resolve_type(self):
        """The currently selected goal"""
        return self._resolve_type

    current = property(_get_current)
    resolve_type = property(_get_resolve_type)


class VariableDesignator(Designator):

    """
    A VariableDesignator simply contains a variable that can be set by anyone.
    This variable is encapsulated by a property called current.

    You can also set current = ...

    >>> v = VariableDesignator()  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> v = VariableDesignator(resolve_type=str)
    >>> v.current = 'Works'
    >>> v.current
    'Works'

    >>> v.current = 666  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...
    >>> assert(v.current == 'Works') #Unchanged
    """

    def __init__(self, initial_value=None, resolve_type=None):
        if not resolve_type:
            resolve_type = type(initial_value)
            if resolve_type == type(None): #If the initial value is None, then the resolve_type cannot be interred and thus we raise an exception
                raise TypeError("VariableDesignator requires to set resolve_type to ensure user can use it")
        super(VariableDesignator, self).__init__(initial_value, resolve_type)

    def _set_current(self, value):
        resolve_type = self.resolve_type
        try:
            resolve_type = tuple(self.resolve_type)
        except TypeError:
            pass

        if not issubclass(type(value), resolve_type):
            raise TypeError("Assigned value does not match resolve_type for {0}. Expected a (subclass of) {1} but got a {2}".format(self, self.resolve_type, type(value)))
        self._current = value

    current = property(Designator._get_current, _set_current)


if __name__ == "__main__":
    import doctest
    doctest.testmod()