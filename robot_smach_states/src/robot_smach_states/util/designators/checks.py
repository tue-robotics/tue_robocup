#! /usr/bin/env python
__author__ = 'loy'
import core


def check_resolve_type(designator, *allowed_types):
    """
    >>> from robot_smach_states.util.designators.core import Designator
    >>> d1 = Designator("a", resolve_type=str)
    >>> check_resolve_type(d1, str)
    >>> d2 = Designator("a", resolve_type=str)
    >>> check_resolve_type(d2, int, str)

    >>> d3 = Designator("a", resolve_type=str)
    >>> #The resolve_type is actually str but we check for int, thus get an exception
    >>> check_resolve_type(d3, int)  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d4 = Designator("a", resolve_type=str)
    >>> #The resolve_type is actually str but we check for int, thus get an exception
    >>> check_resolve_type(d4, float, int)  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d5 = Designator(["a", "b", "c"], resolve_type=[str])

    >>> d6 = Designator(["a", "b", "c"], resolve_type=[str])
    >>> #The resolve_type is actually [str] but we check for [int], thus get an exception
    >>> check_resolve_type(d6, [int])  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> d7 = Designator(["a", "b", "c"], resolve_type=[str])
    >>> check_resolve_type(d7, [str])

    >>> d8 = Designator(["a", "b", "c"], resolve_type=[str])
    >>> #The resolve_type is actually [str] but we check for int, thus get an exception
    >>> check_resolve_type(d8, str)
    Traceback (most recent call last):
      ...
    TypeError: Designator(resolve_type=[<type 'str'>], name=None) resolves to [<type 'str'>] but should resolve to one of (<type 'str'>,)
    """

    if isinstance(designator.resolve_type, list):
        real_resolve_type = designator.resolve_type[0]  # Designator should only resolve to one type.
        correct_resolve_type = False
        for allowed_type in allowed_types:  # allowed_types is a tuple (because of the *).
            real_allowed_type = allowed_type[0] if isinstance(allowed_type, list) else allowed_type

            if real_resolve_type == real_allowed_type:
                correct_resolve_type = True
                break

        if not correct_resolve_type:
            raise TypeError("{0} resolves to {1} but should resolve to one of {2}".format(designator,
                                                                                          designator.resolve_type,
                                                                                          allowed_types))

    if designator.resolve_type not in allowed_types:
        raise TypeError("{0} resolves to {1} but should resolve to one of {2}".format(designator,
                                                                                      designator.resolve_type,
                                                                                      allowed_types))


def check_type(designator_or_value, *allowed_types):
    """
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
    """
    if hasattr(designator_or_value, "resolve_type"):  # If its a designator: ...
        check_resolve_type(designator_or_value, *allowed_types)
    else:
        if not type(designator_or_value) in allowed_types:
            raise TypeError("{0} is of type {1} but should be {2}".format(designator_or_value, type(designator_or_value), allowed_types))


def is_writeable(variable_writer):
    if isinstance(variable_writer, core.VariableWriter):
        return True
    else:
        raise TypeError("{0} is not writable".format(variable_writer))


if __name__ == "__main__":
    import doctest
    doctest.testmod()
