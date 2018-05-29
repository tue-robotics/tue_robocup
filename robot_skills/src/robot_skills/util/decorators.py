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
            try:
                rospy.logwarn("function or class {0} is deprecated".format(func_or_cls.__name__))
            except:
                print func_or_cls.__name__, 'is deprecated'
        return func_or_cls(*args, **kwargs)
    return wrapper


def deprecated_replace_with(replacement):
    #import ipdb; ipdb.set_trace()
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
            RED = '\033[91m'
            END = '\033[0m'

            itemname = func_or_cls.__name__
            if itemname == "__init__":
                itemname = args[0]
            rospy.logerr(RED+"Function or class {0} is deprecated. You should use {1} instead".format(itemname, replacement)+END)
            return func_or_cls(*args, **kwargs)
        return wrapper
    return deprecated
