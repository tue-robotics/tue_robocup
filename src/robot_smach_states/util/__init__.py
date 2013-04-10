"""Several ROS, Python, Smach, TF etc related utility functions"""
import roslib; roslib.load_manifest('robot_smach_states')
import rospy

from startup import startup

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