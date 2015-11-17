#! /usr/bin/env python
__author__ = 'loy'
from deprecation_warnings import deprecated, get_caller_info

class Designator(object):

    """
    A Designator defines a goal, which can be defined at runtime or at write-
    time. Its value cannot be set, it can only be get.  This allows to later
    define Designators that take a goal specification, like a query to a world
    model.

    current is therefore a property with only a getter.

    >>> d = Designator("Initial value", name="tester")
    >>> d.resolve()
    'Initial value'

    >>> assert(issubclass(d.resolve_type, str))

    >>> d2 = Designator("not an integer", name="tester2", resolve_type=int)
    >>> d2.resolve()
    Traceback (most recent call last):
    ...
    TypeError: Designator(resolve_type=<type 'int'>, name=tester2) resolved to a '<type 'str'>' instead of expected '<type 'int'>'
    >>> assert(issubclass(d.resolve_type, str))
    """

    instances = []

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        super(Designator, self).__init__()

        self._name = name

        self.__initial_value = initial_value
        if not resolve_type:
            self._resolve_type = type(self.__initial_value)
        else:
            self._resolve_type = resolve_type

        Designator.instances += [self]

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        result = self._resolve()
        if not isinstance(result, self.resolve_type):
            raise TypeError("{} resolved to a '{}' instead of expected '{}'".format(self, type(result), self.resolve_type))
        return result

    def _resolve(self):
        return self.__initial_value

    def _get_resolve_type(self):
        """The currently selected goal"""
        return self._resolve_type

    def _get_name(self):
        """The currently selected goal"""
        return self._name

    resolve_type = property(_get_resolve_type)
    name = property(_get_name)

    def __repr__(self):
        return "Designator(" \
                           "resolve_type={self.resolve_type}, " \
                           "name={self._name}" \
               ")".format(self=self)

    def lockable(self):
        """Designators can be lockable. This means their value does not change between calls to .lock and .unlock().
        What this means exactly is different for different subclasses.

        For example, an EdEntityDesignator must lock only the Entity's ID and not the Python object itself.
        In the latter case, ED can update the Entity's info but the designator will not update that because its
        locked to a Python-object.
        For Arms, it makes sense to lock to the actual Python-object with the current implementation.

        For a lot of Designators, .lockable() will yield the Error below, but in those cases it will make sense,
        because locking them has no meaning.
        Then, calling .lock will fail as well. States that want a .lock method should check for it."""
        raise NotImplementedError("Core Designator is not lockable. Override .lockable() in subclasses and "
                                  "make it so that there's only *one* locker instead of a new one every call."
                                  " For locking to the exact same Python-object, use the LockingDesignator")

    def _get_current(self):
        caller_info = get_caller_info()
        raise DeprecationWarning("Using designator.current (as in {}:{} is deprecated. Use designator.resolve() instead".format(self, caller_info["filename"],
                                                                                             caller_info["line_number"]))

    current = property(_get_current)


class VariableDesignator(Designator):

    """
    A VariableDesignator simply contains a variable that can be set if you have a writer for it.

    >>> #Create a VariableDesignator with a string
    >>> v = VariableDesignator('Hello')
    >>> v.resolve() #No surprise, it resolves to the given string!
    'Hello'

    >>> #Getting from .current will fail because it is deprecated
    >>> v.current  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    DeprecationWarning: ...

    >>> #Writing to the designator directly will fail.
    >>> v.current = 'World'   # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    DeprecationWarning: ...

    >>> v.resolve() #Still unchanged
    'Hello'
    """

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        if not resolve_type:
            resolve_type = type(initial_value)
            if resolve_type == type(None): #If the initial value is None, then the resolve_type cannot be interred and thus we raise an exception
                raise TypeError("VariableDesignator requires to set resolve_type to ensure user can use it")

        super(VariableDesignator, self).__init__(initial_value, resolve_type, name=name)

        self._current = initial_value

        self.writeable = VariableWriter(self)

    def _set_current_protected(self, value):
        resolve_type = self.resolve_type
        try:
            resolve_type = tuple(self.resolve_type)
        except TypeError:
            pass

        if not issubclass(type(value), resolve_type):
            raise TypeError("Assigned value does not match resolve_type for {0}. Expected a (subclass of) {1} but got a {2}".format(self, self.resolve_type, type(value)))
        self._current = value

    def _resolve(self):
        return self._current

    def _set_current(self, value):
        raise DeprecationWarning("Cannot directly write to a VariableDesignator, use a VariableWriter instead. E.g. XxxState(robot, someVariableDes.writable)")

    def _get_current(self):
        caller_info = get_caller_info()
        raise DeprecationWarning("Using designator.current (as in {}:{} is deprecated. Use designator.resolve() instead".format(self, caller_info["filename"],
                                                                                             caller_info["line_number"]))

    current = property(_get_current, _set_current) #TODO: Once all state machines use the VariableWriter, the _set_current can be removed


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
    >>> w.write('World')
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
        print "writable(VariableDesignator).current = ...  is deprecated, use writable(VariableDesignator).write(...) instead"
        self.variable_designator._set_current_protected(value)

    current = property(VariableDesignator._get_current, _set_current)

    def _get_resolve_type(self):
        return self.variable_designator.resolve_type

    resolve_type = property(_get_resolve_type)

    def _resolve(self, *args, **kwargs):
        return self.variable_designator.resolve(*args, **kwargs)

    def _get_resolve_type(self):
        return self.variable_designator.resolve_type

    resolve_type = property(_get_resolve_type)


writeable = VariableWriter


if __name__ == "__main__":
    import doctest
    doctest.testmod()