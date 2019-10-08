"""Deprecation warnings.
From:
- https://wiki.python.org/moin/PythonDecoratorLibrary#Generating_Deprecation_Warnings
- http://stackoverflow.com/questions/2536307/decorators-in-the-python-standard-lib-deprecated-specifically
"""

from __future__ import print_function

__author__ = 'Loy'

import warnings
import functools
import inspect

# def deprecated(func):
#     '''This is a decorator which can be used to mark functions
#     as deprecated. It will result in a warning being emitted
#     when the function is used.'''
#     def new_func(*args, **kwargs):
#         warnings.warn("Call to deprecated function {}.".format(func.__name__),
#                       category=DeprecationWarning)
#         return func(*args, **kwargs)
#     new_func.__name__ = func.__name__
#     new_func.__doc__ = func.__doc__
#     new_func.__dict__.update(func.__dict__)
#     return new_func


def deprecated(func):
    """
    This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used.
    """

    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  # turn off filter

        warnings.warn_explicit(
            "Call to deprecated function {}.".format(func.__name__),
            category=DeprecationWarning,
            filename=func.func_code.co_filename,
            lineno=func.func_code.co_firstlineno + 1
        )

        warnings.simplefilter('default', DeprecationWarning)  # reset filter

        return func(*args, **kwargs)
    return new_func


def get_caller_info():
    # Index 1 would get caller of get_caller_info, index 2 gets the caller of of the method that calls get_caller_info
    frame, filename, line_number, function_name, lines, index = inspect.getouterframes(inspect.currentframe())[2]
    return {"frame": frame, "filename": filename, "line_number": line_number, "function_name": function_name,
            "lines": lines, "index": index}


if __name__ == "__main__":
    # Usage examples
    @deprecated
    def my_func():
        print("I'm a function")

    my_func()
