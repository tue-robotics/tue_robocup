#! /usr/bin/env python
import roslib; 
import rospy
import math

import pprint

def xy_dist(answer, (origX, origY), mapping={"X":"X", "Y":"Y"}):
    """
    >>> xyz_dist({'X': 0.5, 'Y': 0.5}, (0.0,0.0)) == 0.70710678118654757
    True
    """
    x = float(answer[mapping["X"]])
    y = float(answer[mapping["Y"]])

    origX = float(origX)
    origY = float(origY)
    dist = math.sqrt(abs(origX-x)**2 + abs(origY-y)**2)
    return dist

def xyz_dist(answer, (origX, origY, origZ), mapping={"X":"X", "Y":"Y", "Z":"Z"}):
    """
    >>> xyz_dist({'X': 0.5, 'Y': 0.5, 'Z': 0.0}, (0.0,0.0,0.0)) == 0.70710678118654757
    True
    """
    x = float(answer[mapping["X"]])
    y = float(answer[mapping["Y"]])
    z = float(answer[mapping["Z"]])

    origX = float(origX)
    origY = float(origY)
    origZ = float(origZ)
    dist = math.sqrt(abs(origX-x)**2 + abs(origY-y)**2 + abs(origZ-z)**2)
    return dist

def xyphi_dist(answer, (origX, origY, origPhi), mapping={"X":"X", "Y":"Y", "Phi":"Phi"}):
    """
    >>> xyphi_dist({'X': 0.5, 'Y': 0.5, 'Z': 0.0}, (0.0,0.0,0.0)) == 0.70710678118654757
    True
    """
    x = float(answer[mapping["X"]])
    y = float(answer[mapping["Y"]])
    #phi = answer[mapping["Phi"]]

    origX = float(origX)
    origY = float(origY)
    origPhi = float(origPhi)

    dist = math.sqrt(abs(origX-x)**2 + abs(origY-y)**2)
    return dist

def select_answer(answers, keyfunc, minmax=min, criteria=None):
    """ @param answers a list of dictionaries returned by the reasoner.
        @param keyfunc a function that returns a score for each answerdict
        @param minmax select the min or max scoring answer? valid answers are min or max, as those are python builtins
        @param criteria a list of functions that return True when the passed answerdict meets a criterium

        >>> bindings = [{'X': 0.0, 'Y': 0.0, 'Z': 0.0}, \
                        {'X': 1.0, 'Y': 0.0, 'Z': 0.0}, \
                        {'X': 0.0, 'Y': 1.0, 'Z': 0.0}, \
                        {'X': 2.0, 'Y': 2.0, 'Z': 0.0}, \
                        {'X': 0.5, 'Y': 0.5, 'Z': 0.0}]
        >>> key = lambda answer: xyz_dist(answer, (0,0,0))
        >>> crit = lambda answer: 0.1 < xyz_dist(answer, (0,0,0)) < 0.8
        >>> select_answer(bindings, keyfunc=key) == \
            {'X': 0.0, 'Y': 0.0, 'Z': 0.0} # doctest:+ELLIPSIS
        [INFO] ...
        True
        
        >>> select_answer(bindings, keyfunc=key, minmax=max) == \
            {'X': 2.0, 'Y': 2.0, 'Z': 0.0} # doctest:+ELLIPSIS
        [INFO] ...
        True
        
        >>> select_answer(bindings, keyfunc=key, minmax=min, criteria=[crit]) == \
            {'X': 0.5, 'Y': 0.5, 'Z': 0.0} # doctest:+ELLIPSIS
        [INFO] ...
        [INFO] ...
        True
        """

    criteria = criteria or []
    rospy.loginfo("{0} answers before filtering: {1}".format(len(answers), pprint.pformat(answers)))
    for criterium in criteria:
        answers = filter(criterium, answers)
        import inspect
        criterium_code = inspect.getsource(criterium)
        rospy.loginfo("Criterium {0} leaves {1} answers: {2}".format(criterium_code, len(answers), pprint.pformat(answers)))

    if not answers:
        raise ValueError("No answers matched the critera.")

    return minmax(answers, key=keyfunc)

def answer_to_tuple(answer, variable_ordering=["X", "Y", "Z"], castTo=[float, float, float]):
    """ @param answer a dictionary as returned by the reasoner
        @param variable_ordering which fields of the dict should be listed in the tuple

        >>> ans = {"X":1, "Y":3, "Z":5, "A":47, "B":"foo", "C":"bar"}
        >>> answer_to_tuple(ans, variable_ordering=["B", "X", "Y", "Z"], castTo=[str, int, int, int])
        ('foo', 1, 3, 5)
        >>> answer_to_tuple({"FloatAsStr":"9.5"}, variable_ordering=["FloatAsStr"], castTo=[float])
        (9.5,)
    """
    if castTo:
        return tuple(_type(answer[variable]) for variable, _type in zip(variable_ordering, castTo)) #Loop over both the labels and the types in one go using zip. 
    else:
        return tuple(answer[variable] for variable in variable_ordering)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
