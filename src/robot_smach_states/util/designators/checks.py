#! /usr/bin/env python
__author__ = 'loy'


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
    """
    if not designator.resolve_type in allowed_types:
        raise TypeError("{0} resolves to {1} but should resolve to one of {2}".format(designator, designator.resolve_type, allowed_types))


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
    if hasattr(designator_or_value, "resolve_type"): #If its a designator: ...
        check_resolve_type(designator_or_value, *allowed_types)
    else:
        if not type(designator_or_value) in allowed_types:
            raise TypeError("{0} is of type {1} but should be {2}".format(designator_or_value, type(designator_or_value), allowed_types))

if __name__ == "__main__":
    import doctest
    doctest.testmod()