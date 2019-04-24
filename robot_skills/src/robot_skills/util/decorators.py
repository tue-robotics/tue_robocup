# ROS
import rospy


def deprecated(func_or_cls):
    """Print a deprecation warning once on first use of the function.

    >>> @deprecated                      # doctest: +SKIP
    ... def f():
    ...     pass
    >>> f()                              # doctest: +SKIP
    f is deprecated
    """
    count = [0]

    def wrapper(*args, **kwargs):
        count[0] += 1
        if count[0] == 1:
            item_name = func_or_cls.__name__
            if item_name == "__init__":
                item_name = args[0]
            try:
                rospy.logwarn("function or class {0} is deprecated".format(item_name))
            except Exception:
                print("function or class {0} is deprecated".format(item_name))
        return func_or_cls(*args, **kwargs)
    return wrapper


def deprecated_replace_with(replacement):
    def deprecated(func_or_cls):
        """Print a deprecation warning once on first use of the function.

        >>> @deprecated                      # doctest: +SKIP
        ... def f():
        ...     pass
        >>> f()                              # doctest: +SKIP
        f is deprecated
        """
        count = [0]

        def wrapper(*args, **kwargs):
            count[0] += 1
            if count[0] == 1:
                item_name = func_or_cls.__name__
                if item_name == "__init__":
                    item_name = args[0]
                try:
                    rospy.logwarn("Function or class {0} is deprecated. You should use {1} instead".format(item_name,
                                                                                                           replacement))
                except Exception:
                    print("Function or class {0} is deprecated. You should use {1} instead".format(item_name,
                                                                                                   replacement))
            return func_or_cls(*args, **kwargs)
        return wrapper
    return deprecated
