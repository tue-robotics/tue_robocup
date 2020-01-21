__author__ = 'loy'
import weakref
from deprecation_warnings import get_caller_info
import rospy


class Designator(object):

    """
    A Designator defines a goal, which can be defined at runtime or at write-
    time. Its value cannot be set, it can only be get.  This allows to later
    define Designators that take a goal specification, like a query to a world
    model.

    >>> d = Designator("Initial value", name="tester")
    >>> d.resolve()
    'Initial value'

    >>> assert(issubclass(d.resolve_type, str))
    """

    instances = weakref.WeakSet()

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        """
        Initialization method

        :param initial_value: Initial value, should match the given resolve_type.
                              None is allowed if a resolve_type is provided.
        :param resolve_type: Type to which this designator should resolve.
                             If None, use the type of the initial value. Value
                             must be not None in that case.
        :vartype resolve_type: Type or a list with a single type (the element type).
                               The latter represents list of values where each
                               value has the element type.

        :param name: name used for debugging purposes
        :vartype name: str
        """
        super(Designator, self).__init__()

        self._name = name
        self.__initial_value = initial_value
        if resolve_type:
            if isinstance(resolve_type, list):
                # Value is a list of items with each item of type resolve_type[0] .
                if len(resolve_type) != 1:
                    msg = "The resolve_type must be a single-item list, found {}."
                    raise TypeError(msg.format(resolve_type))

                if resolve_type[0] is None:
                    raise TypeError("The resolve_type may not be [None]")
            self._resolve_type = resolve_type
        else:
            # No resolve type provided, use the inital value to derive a type.
            if self.__initial_value is None:
                raise TypeError("No resolve_type provided and type could not be inferred "
                                "from the initial_value (found 'None').")

            if isinstance(self.__initial_value, list):
                if len(self.__initial_value) > 0:
                    self._resolve_type = [type(self.__initial_value[0])]
                else:
                    raise TypeError("No resolve_type provided and type could not be inferred "
                                    "from the initial_value (found empty list).")
            else:
                self._resolve_type = type(self.__initial_value)

        Designator.instances.add(self)

    def resolve(self):
        """
        Selects a new goal and sets it as the current value.

        Don't override this method, override self._resolve() instead.
        """
        result = self._resolve()
        result_type = type(result)
        resolve_type = self.resolve_type

        if result is None: # No data available for checking the type.
            return result

        if isinstance(self._resolve_type, list):
            # Not None, list expected instead.
            if not isinstance(result, list):
                self.fail_with_type_error(result_type, resolve_type)

            if len(result) == 0: # Empty list, cannot check element type.
                return result

            if not isinstance(result[0], resolve_type[0]): # Element type check.
                self.fail_with_type_error(result_type, resolve_type)

            return result

        # Normal type.
        if not issubclass(result_type, resolve_type):
            self.fail_with_type_error(result_type, resolve_type)

        return result

    def fail_with_type_error(self, result_type, resolve_type):
            msg = "{} resolved to a '{}' instead of expected '{}'."
            raise TypeError(msg.format(self, result_type, resolve_type))

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
        return "Designator(resolve_type={}, name={})".format(self.resolve_type, self.name)

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
        raise DeprecationWarning(
            "Using designator.current (as in {}:{}) is deprecated."
            "Use designator.resolve() instead".format(self, caller_info["filename"],
                                                      caller_info["line_number"]))

    current = property(_get_current)


class VariableDesignator(Designator):
    """
    A VariableDesignator simply contains a variable that can be set if you have a writer for it.

    >>> #Create a VariableDesignator with a string
    >>> v = VariableDesignator('Hello')
    >>> v.resolve() #No surprise, it resolves to the given string!
    'Hello'

    You cannot directly set a VariableDesignator though, you must obtain a VariableWriter through the
    .writeable-attribute
    """

    def __init__(self, initial_value=None, resolve_type=None, name=None):
        if not resolve_type and not initial_value:
            raise TypeError("VariableDesignator requires to set either initial_value or resolve_type to ensure user "
                            "can use it")

        super(VariableDesignator, self).__init__(initial_value, resolve_type, name=name)
        self._current = initial_value
        self.writeable = VariableWriter(self)

    def _set_current_protected(self, value):
        resolve_type = self.resolve_type

        if isinstance(value, list) and isinstance(self.resolve_type, list):
            if value and not issubclass(type(value[0]), resolve_type[0]):
                raise TypeError(
                    "Assigned value does not match resolve_type for {0}. "
                    "Expected a (subclass of) {1} but got a {2}".format(self, self.resolve_type, type(value)))
        else:
            if not issubclass(type(value), resolve_type):
                raise TypeError(
                    "Assigned value does not match resolve_type for {0}. "
                    "Expected a (subclass of) {1} but got a {2}".format(self, self.resolve_type, type(value)))
        self._current = value

    def _resolve(self):
        return self._current

    def _set_current(self, value):
        raise DeprecationWarning(
            "Cannot directly write to a VariableDesignator, use a VariableWriter instead. "
            "E.g. XxxState(robot, someVariableDes.writable)")

    def _get_current(self):
        caller_info = get_caller_info()
        raise DeprecationWarning(
            "Using designator.current (as in {}:{} is deprecated. "
            "Use designator.resolve() instead".format(self, caller_info["filename"],
                                                      caller_info["line_number"]))

    # ToDo: Once all state machines use the VariableWriter, the _set_current can be removed
    current = property(_get_current, _set_current)


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

    >>> v2 = VariableDesignator(['a', 'b', 'c'], resolve_type=[str])
    >>> v2.resolve()
    ['a', 'b', 'c']
    >>> v2w = v2.writeable
    >>> v2w.write(v2.resolve() + ['d'])
    >>> v2.resolve()
    ['a', 'b', 'c', 'd']
    """

    instances = weakref.WeakSet()

    def __init__(self, variable_designator):
        self.variable_designator = variable_designator
        self.name = "writeable({})".format(self.variable_designator.name)
        VariableWriter.instances.add(self)

    def write(self, value):
        """
        Write a value to the designator this writer is associated with.

        :param value: the value to be written
        :return: None

        Value can be either a singular object or a list, which may be empty or not.
        The singular object's type has to fit the resolve_type.
        If the value is a list, then resolve_type must also be a list,
            which the only element being the type each value-element must have.
        In case the value-list has an element,
            this element must be of the type specified by the only element in the resolve_type-list
        In case the value-list is empty, we simply assign that empty list without any type checking.
        """
        if isinstance(value, list) and isinstance(self.variable_designator.resolve_type, list):
            if not value:
                self.variable_designator._set_current_protected(value)
            elif isinstance(value[0], self.variable_designator.resolve_type[0]):
                self.variable_designator._set_current_protected(value)
            else:
                raise TypeError("Cannot assign {} to {} which has resolve_type {}".format(
                    type(value),
                    self.variable_designator,
                    self.variable_designator.resolve_type))
        elif isinstance(value, self.variable_designator.resolve_type):
            self.variable_designator._set_current_protected(value)
        else:
            raise TypeError("Cannot assign {} to {} which has resolve_type {}".format(
                type(value),
                self.variable_designator,
                self.variable_designator.resolve_type))

    def _set_current(self, value):
        rospy.loginfo("writable(VariableDesignator).current = ...  is deprecated, " \
              "use writable(VariableDesignator).write(...) instead")
        self.variable_designator._set_current_protected(value)

    current = property(VariableDesignator._get_current, _set_current)

    def _get_resolve_type(self):
        return self.variable_designator.resolve_type

    resolve_type = property(_get_resolve_type)

    def resolve(self, *args, **kwargs):
        return self.variable_designator.resolve(*args, **kwargs)

    def __repr__(self):
        return "VariableWriter({}(..., name={}))".format(type(self.variable_designator), self.variable_designator.name)


writeable = VariableWriter


if __name__ == "__main__":
    import doctest
    doctest.testmod()
