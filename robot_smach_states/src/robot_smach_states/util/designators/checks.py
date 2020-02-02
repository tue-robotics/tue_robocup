__author__ = 'loy'
import core


def check_resolve_type(designator, *allowed_types):
    """
    Check if the resolve type of a designator is one of the allowed types. If the resolve type is a list, the internal
    type is also checked.

        Incorrect: []; [str, int]
        Correct: [str]; [str], [int]

    This check allows for type checking on construction of a statemachine, so runtime error can be prevented.

    :param designator: Designator to check the resolve type of. In case of a list resolve type, one type in the list
        needs to be defined.
    :param allowed_types: Allowed resolve type of the designator. Allowed list types, also need to have an internal type
        defined.

    >>> from robot_smach_states.util.designators.core import Designator
    >>> d1 = Designator("a", resolve_type=str, name='d1')
    >>> check_resolve_type(d1, str)
    >>> d2 = Designator("a", resolve_type=str, name='d2')
    >>> check_resolve_type(d2, int, str)

    >>> d3 = Designator("a", resolve_type=str, name='d3')
    >>> #The resolve_type is actually str but we check for int, thus get an exception
    >>> check_resolve_type(d3, int)  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d4 = Designator("a", resolve_type=str, name='d4')
    >>> #The resolve_type is actually str but we check for int, thus get an exception
    >>> check_resolve_type(d4, float, int)  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d5 = Designator(["a", "b", "c"], resolve_type=[str], name='d5')

    >>> d6 = Designator(["a", "b", "c"], resolve_type=[str], name='d6')
    >>> #The resolve_type is actually [str] but we check for [int], thus get an exception
    >>> check_resolve_type(d6, [int])  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d7 = Designator("a", resolve_type=str, name='d7')
    >>> d8 = Designator(["a"], resolve_type=[str], name='d8')
    >>> check_resolve_type(d7, str, [str])
    >>> check_resolve_type(d8, str, [str])
    >>> check_resolve_type(d7, [str], str)
    >>> check_resolve_type(d8, [str], str)
    """
    if isinstance(designator.resolve_type, list):
        real_resolve_type = designator.resolve_type[0]

        # allowed_types is a list (because of the *).
        real_allowed_types = [allowed_type[0] if isinstance(allowed_type, list) else allowed_type for allowed_type in allowed_types]

        if real_resolve_type not in real_allowed_types:
            raise TypeError("{0} resolves to {1} but should resolve to one of {2}".format(designator, designator.resolve_type, allowed_types))

    if designator.resolve_type not in allowed_types:
        raise TypeError("{0} resolves to {1} but should resolve to one of {2}".format(designator, designator.resolve_type, allowed_types))


def check_type(designator_or_value, *allowed_types):
    """
    Check if the resolve type of a designator or a variable is one of the allowed types. If the type is a list, the internal
    type is also checked.
    Incorrect: []; [str, int]
    Correct: [str]; [str], [int]
    This check allows for type checking on construction of a statemachine, so runtime error can be prevented.

    :param designator_or_value: Designator or variable to check the type of.
    :param allowed_types: Allowed type of the designator or variable.

    >>> from robot_smach_states.util.designators.core import Designator
    >>> d = Designator("a", resolve_type=str)
    >>> check_type(d, str)
    >>> c = "a"
    >>> check_type(c, int, str)

    >>> c2 = "string"
    >>> #The type is str but we check for int, thus get an exception
    >>> check_type(c2, int)  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> c3 = Designator(["a"], resolve_type=[str], name='c3')
    >>> check_type(c3, [str])
    >>> check_type(c3, str, [str])
    >>> check_type(c3, [str], str)
    >>> c4 = ["a"]
    >>> check_type(c4, [str])
    >>> check_type(c4, str, [str])
    >>> check_type(c4, [str], str)
    >>> check_type(c4, [int])  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...
    >>> check_type(c4, [str, int])  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    AssertionError: ...
    """
    for allowed_type in allowed_types:
        if isinstance(allowed_type, list):
            assert len(allowed_type) == 1, "Allowed list type should contain one internal type, incorrect type: {}".format(allowed_type)

    if hasattr(designator_or_value, "resolve_type"):  # If its a designator: ...
        check_resolve_type(designator_or_value, *allowed_types)
    else:
        if isinstance(designator_or_value, list):
            allowed_list_types = [allowed_type[0] for allowed_type in allowed_types if isinstance(allowed_type, list)]

            if type(designator_or_value[0]) not in allowed_list_types:
                allowed_list_types_string = ", ".join(map("[{}]".format, allowed_list_types))
                raise TypeError("{0} is of type [{1}] but should be {2}".format(designator_or_value,
                                                                                type(designator_or_value[0]),
                                                                                allowed_list_types_string))
        else:
            if type(designator_or_value) not in allowed_types:
                raise TypeError("{0} is of type {1} but should be {2}".format(designator_or_value, type(designator_or_value), allowed_types))


def is_writeable(variable_writer):
    if isinstance(variable_writer, core.VariableWriter):
        return True
    else:
        raise TypeError("{0} is not writable".format(variable_writer))


if __name__ == "__main__":
    import doctest
    doctest.testmod()
