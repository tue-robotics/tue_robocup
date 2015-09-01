#! /usr/bin/env python
__author__ = 'loy'

class Designator(object):

    """
    A Designator defines a goal, which can be defined at runtime or at write-
    time. Its value cannot be set, it can only be get.  This allows to later
    define Designators that take a goal specification, like a query to a world
    model.

    current is therefore a property with only a getter.

    >>> d = Designator("Initial value", name="tester")
    >>> d.current
    'Initial value'
    >>> d.current = 'Error'
    Traceback (most recent call last):
     ...
    AttributeError: can't set attribute

    >>> assert(issubclass(d.resolve_type, str))"""

    instances = []

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        super(Designator, self).__init__()

        self._name = name

        self._current = initial_value
        if not resolve_type:
            self._resolve_type = type(initial_value)
        else:
            self._resolve_type = resolve_type

        Designator.instances += [self]

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        return self.current

    def _get_current(self):
        """The currently selected goal"""
        return self._current

    def _get_resolve_type(self):
        """The currently selected goal"""
        return self._resolve_type

    def _get_name(self):
        """The currently selected goal"""
        return self._name

    current = property(_get_current)
    resolve_type = property(_get_resolve_type)
    name = property(_get_name)


class VariableDesignator(Designator):

    """
    A VariableDesignator simply contains a variable that can be set if you have a writer for it.
    """

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        if not resolve_type:
            resolve_type = type(initial_value)
            if resolve_type == type(None): #If the initial value is None, then the resolve_type cannot be interred and thus we raise an exception
                raise TypeError("VariableDesignator requires to set resolve_type to ensure user can use it")
        super(VariableDesignator, self).__init__(initial_value, resolve_type, name=name)

    def _set_current_protected(self, value):
        resolve_type = self.resolve_type
        try:
            resolve_type = tuple(self.resolve_type)
        except TypeError:
            pass

        if not issubclass(type(value), resolve_type):
            raise TypeError("Assigned value does not match resolve_type for {0}. Expected a (subclass of) {1} but got a {2}".format(self, self.resolve_type, type(value)))
        self._current = value

    def _set_current(self, value):
        raise DeprecationWarning("Cannot directly write to a VariableDesignator, use a VariableWriter (aliased as writeable(designator) instead")

    current = property(Designator._get_current, _set_current) #TODO: Once all state machines use the VariableWriter, the _set_current can be removed


class VariableWriter(object):
    """When writing to a VariableDesignator you must use a writer, 
        to make it explicit who changes designators so you can directly spot the changer.
    This way, the dataflow can be more accurately visualized and understood.

    >>> #Create a VariableDesignator with a string
    >>> v = VariableDesignator('Hello')
    >>> v.resolve() #No surprise, it resolves to the given string!
    'Hello'
    >>> #Writing to the designator directly will fail.
    >>> v.current = 'World'   # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    DeprecationWarning: ...

    >>> v.resolve() #Still unchanged
    'Hello'
    >>> #Instead, use a writer to assign to a VariableDesignator
    >>> w = writeable(v)
    >>> w.current = 'World'
    >>> v.resolve() #Now it works!
    'World'
    """

    instances = []

    def __init__(self, variable_designator):
        self.variable_designator = variable_designator
        self.name = "writeable({})".format(self.variable_designator.name)
        VariableWriter.instances += [self]

    def write(self, value):
        if isinstance(value, self.variable_designator.resolve_type):
            self.variable_designator._set_current_protected(value)
        else:
            raise TypeError("Cannot assign {} to {} which has resolve_type {}".format(type(value), 
                self.variable_designator, 
                self.variable_designator.resolve_type))

    def _set_current(self, value):
        self.variable_designator._set_current_protected(value)

    current = property(Designator._get_current, _set_current)

    def _get_resolve_type(self):
        return self.variable_designator.resolve_type

    resolve_type = property(_get_resolve_type)

    def resolve(self, *args, **kwargs):
        return self.variable_designator.resolve(*args, **kwargs)

    def _get_resolve_type(self):
        return self.variable_designator.resolve_type

    resolve_type = property(_get_resolve_type)


writeable = VariableWriter


if __name__ == "__main__":
    import doctest
    doctest.testmod()