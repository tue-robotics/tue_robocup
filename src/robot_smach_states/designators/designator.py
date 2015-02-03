#!/usr/bin/python
"""
Designators are intended to encapsulate the process of resolving values needed
at runtime based on write-time definitions.  This sound very vague, but
imagine a navigation task that needs to navigate to some object.

The objects you want to navigate to is not yet known beforehand, because its not yet present.
Instead, you can define some function that finds an object to navigate to.

A designator is an object that encapsulates such a function.
The only interface method is that they have a resolve function that gives some value.

How .resolve() does its work is not important here and
    may be a custom implementation for all instances.

The library here defines a couple of standard designators:
- Designator:           simply returns a predefined value that defaults to None
- VariableDesignator:   any user of this designator can set the designators value and
                        can be used to pass around data.
- AttrDesignator:       Some designator types wrap other designators, like AttrDesignator.
                        It returns some attribute of of whatever the wrapped designator resolved to
- FuncDesignator:       Apply a function to the resolution of another, wrapped, designator
"""
import rospy

from ed.srv import SimpleQuery, SimpleQueryRequest
import geometry_msgs.msg as gm
import std_msgs.msg as std
import inspect
import pprint

class DesignatorResolvementError(Exception):
    """This error is raised when a designator cannot be resolved."""


class Designator(object):

    """
    A Designator defines a goal, which can be defined at runtime or at write-
    time. Its value cannot be set, it can only be get.  This allows to later
    define Designators that take a goal specification, like a query to a world
    model.

    current is therefore a property with only a getter.

    >>> d = Designator("Initial value")
    >>> d.current
    'Initial value'
    >>> d.current = 'Error'
    Traceback (most recent call last):
     ...
    AttributeError: can't set attribute"""

    def __init__(self, initial_value=None):
        super(Designator, self).__init__()

        self._current = initial_value

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        return self.current

    def _get_current(self):
        """The currently selected goal"""
        return self._current

    current = property(_get_current)


class VariableDesignator(Designator):

    """
    A VariableDesignator simply contains a variable that can be set by anyone.
    This variable is encapsulated by a property called current.

    You can also set current = ...

    >>> v = VariableDesignator()
    >>> v.current = 'Works'
    >>> v.current
    'Works'
    """

    def __init__(self, initial_value=None):
        super(VariableDesignator, self).__init__(initial_value)

    def _set_current(self, value):
        self._current = value

    current = property(Designator._get_current, _set_current)


class PointStampedOfEntityDesignator(Designator):

    def __init__(self, entity_designator):
        super(VariableDesignator, self).__init__()
        self.entity_designator
        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

    def resolve(self):
        # type is a reserved keyword. Maybe unpacking a dict as kwargs is
        # cleaner
        query = SimpleQueryRequest(id=self.entity_designator.resolve())
        entities = self.ed(query).entities
        if entities:
            entity = entities[0]
            pointstamped = gm.PointStamped(point=entity.center_point,
                                           header=std.Header(
                                               entity.id, rospy.get_rostime())
                                           )  # ID is also the frame ID. Ed just works that way
            self._current = pointstamped
            return self.current
        else:
            # TODO: Make cutom exception here
            raise Exception("No such entity")


class PsiDesignator(Designator):

    """A PsiDesignator encapsulates Psi queries to a reasoner.
    A reasoner may return multiple valid answers to a query, but """

    def __init__(self, query, reasoner, sortkey=None, sortorder=min, criteriafuncs=None):
        """Define a new designator around a Psi query, to be posed to a reasoner
        @param query the query to be posed to the given reasoner
        @param reasoner the reasoner that should answer the query"""
        self.query = query
        self.reasoner = reasoner
        self.sortkey = sortkey
        self.sortorder = sortorder
        self.criteriafuncs = criteriafuncs or []

    def resolve(self):
        """
        Returns an answer from the reasoner that satisfies some criteria and
        is the best according to some function and sorting      """
        answers = self.reasoner.query(self.query)

        rospy.loginfo("{0} answers before filtering: {1}".format(
                      len(answers), pprint.pformat(answers))
                      )
        for criterium in self.criteriafuncs:
            answers = filter(criterium, answers)
            criterium_code = inspect.getsource(criterium)
            rospy.loginfo("Criterium {0} leaves {1} answers: {2}".format(
                          criterium_code, len(answers), pprint.pformat(answers))
                          )

        if not answers:
            raise ValueError("No answers matched the critera.")

        return self.sortorder(answers, key=self.sortkey)[0]


class EdEntityByQueryDesignator(Designator):

    """
    Resolves to an entity from an Ed query, (TODO: selected by some filter and
    criteria functions)
    """

    def __init__(self, ed_query, criteriafuncs=None):
        super(EdEntityByQueryDesignator, self).__init__()
        self.query = ed_query
        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.criteriafuncs = criteriafuncs or []

    def resolve(self):
        entities = self.ed(self.query)
        if entities:
            for criterium in self.criteriafuncs:
                entities = filter(criterium, entities)
                criterium_code = inspect.getsource(criterium)
                rospy.loginfo("Criterium {0} leaves {1} entities: {2}".format(
                              criterium_code, len(entities), pprint.pformat(entities))
                              )

            import ipdb; ipdb.set_trace()
            self._current = entities[0]  # TODO: add sortkey
            return self.current
        else:
            raise Exception(
                "No entities found matching query {0}".format(self.query))


class AttrDesignator(Designator):

    """Get some attribute of the object a wrapped designator resolves to.
    For example:
    >>> d = Designator(object())
    >>> #Get the __class__ attribute of the object that d resolves to
    >>> wrapped = AttrDesignator(d, '__class__')
    >>> wrapped.resolve() == object
    True
    """

    def __init__(self, orig, attribute):
        super(AttrDesignator, self).__init__()
        self.orig = orig
        self.attribute = attribute

    def resolve(self):
        return self.orig.resolve().__getattribute__(self.attribute)


class FuncDesignator(Designator):

    """Apply a function to the object a wrapped designator resolves to
    For example:
    >>> d = Designator("Hello")
    >>> wrapped = FuncDesignator(d, len) #Determine the len of whatever d resolves to
    >>> wrapped.resolve()
    5
    """

    def __init__(self, orig, func):
        super(FuncDesignator, self).__init__()
        self.orig = orig
        self.func = func

    def resolve(self):
        return self.func(self.orig.resolve())


class ArmDesignator(Designator):
    """Resolves to an instance of the Arm-class in robot_skills.
    >>> left, right = "left", "right" #Just strings in this test, but must be robot_skills.Arm-instances
    >>> a = ArmDesignator({"left":left, "right":right}, left)
    >>> a.resolve()
    'left'
    """

    def __init__(self, all_arms, preferred_arm=None):
        """Initialize a new ArmDesignator with a collection of arms available on the robot and an arm that is preferred for the given operations.
        @param all_arms a dictionary of arms available on the robot
        @param preferred_arm the arm that is preferred for the operations that use this designator"""

        self.all_arms = all_arms
        self.preferred_arm = preferred_arm

        if not self.preferred_arm:
            self.preferred_arm = self.all_arms.values()[0]

        if not self.preferred_arm in self.all_arms.values():
            raise ValueError("The preferred arm is not in the list of arms. Preferred_arm should be one of the arms in the system")

    def resolve(self):
        # import ipdb; ipdb.set_trace()
        if self.available(self.preferred_arm):
            return self.preferred_arm
        else:
            available_arms = filter(self.available, self.all_arms.values())
            if any(available_arms):
                return available_arms[0]
            else:
                raise DesignatorResolvementError("ArmDesignator {0} could not resolve to an arm".format(self))

    def available(self, arm):
        """Check whether the given arm is available for some function."""
        return True


class UnoccupiedArmDesignator(ArmDesignator):
    """An UnoccupiedArmDesignator resolves to an arm that is not occupied by an entity.
    .resolve() raises an DesignatorResolvementError when no such arm can be found
    >>> from mock import MagicMock
    >>> leftArm = MagicMock()
    >>> leftArm.occupied_by = None
    >>> rightArm = MagicMock()
    >>> rightArm.occupied_by = None
    >>> arms = {"left":leftArm, "right":rightArm}
    >>> empty_arm_designator = UnoccupiedArmDesignator(arms, rightArm)
    >>> arm_to_use_for_first_grab = empty_arm_designator.resolve()
    >>> assert(arm_to_use_for_first_grab == rightArm)
    >>> 
    >>> #Grab the 1st item with the rightArm
    >>> rightArm.occupied_by = "entity1"
    >>> arm_to_use_for_second_grab = empty_arm_designator.resolve()
    >>> assert(arm_to_use_for_second_grab == leftArm)
    >>> 
    >>> #Grab the 2nd item with the rightArm
    >>> leftArm.occupied_by = "entity2"
    >>> #You can't do 3 grabs with a 2 arms robot without placing an entity first, so this will fail to resolve for a 3rd time
    >>> arm_to_use_for_third_grab = empty_arm_designator.resolve()  # doctest: +IGNORE_EXCEPTION_DETAIL 
    Traceback (most recent call last):
      ...
    DesignatorResolvementError: ...
    """
    def __init__(self, all_arms, preferred_arm):
        super(UnoccupiedArmDesignator, self).__init__(all_arms, preferred_arm)
    
    def available(self, arm):
        """Check that there is no entity occupying the arm"""
        return arm.occupied_by == None


class ArmHoldingEntityDesignator(ArmDesignator):
    """An UnoccupiedArmDesignator resolves to an arm that is not occupied by an entity.
    .resolve() raises an DesignatorResolvementError when no such arm can be found
    >>> from mock import MagicMock
    >>> leftArm = MagicMock()
    >>> leftArm.occupied_by = None
    >>> rightArm = MagicMock()
    >>> rightArm.occupied_by = "entity3"
    >>> arms = {"left":leftArm, "right":rightArm}
    >>> entity_designator = Designator("entity3")
    >>> holding_arm_designator = ArmHoldingEntityDesignator(arms, entity_designator)
    >>> arm_to_use_for_placing_entity3 = holding_arm_designator.resolve()
    >>> assert(arm_to_use_for_placing_entity3 == rightArm)
    >>>
    >>> #place the object
    >>> rightArm.occupied_by = None
    >>> 
    >>> #After placing the item, there is no arm holding the item anymore
    >>> arm_to_use_for_second_place = holding_arm_designator.resolve()  # doctest: +IGNORE_EXCEPTION_DETAIL 
    Traceback (most recent call last):
      ...
    DesignatorResolvementError: ..."""
    def __init__(self, all_arms, entity_designator):
        super(ArmHoldingEntityDesignator, self).__init__(all_arms)

        self.entity_designator = entity_designator
    
    def available(self, arm):
        """Check that the arm is occupied by the entity refered to by the entity_designator"""
        return arm.occupied_by == self.entity_designator.resolve()


def test_visited_and_unreachable():
    """In our RoboCup executives, we keep track of which items are 'processed' or visited,
    and which cannot be processed because they are unreachable.

    One way to do this, is to mark objectIDs (targets) as visited and unreachable in the reasoner,
    e.g. a global blackboard that can be manipulated an queried through Prolog.

    This gets messy with complex queries. With designators, it should be a lot simpler"""
    success = lambda goal: goal % 2  # Even targets will fail

    target_list = [i for i in range(10)] + [i for i in range(10)]  # all are in twice!
    targets = Designator(target_list)

    unreachable_set = set()
    unreachable = VariableDesignator(unreachable_set)

    visited_set = set()
    visited = VariableDesignator(visited_set)

    for target in target_list:
        if target not in unreachable.resolve() and target not in visited.resolve():
            print "Processing target {0}".format(target)
            if success(target):
                visited.current.add(target)  # This is a set so no += [...]
                print "Target {0} was successfull, its visited".format(target)
            else:
                unreachable.current.add(target)  # This is a set so no += [...]
                print "Target {0} failed, its unreachable".format(target)
        else:
            print "NOT processing target {0}: its unreachable or already visited".format(target)

    print "##### Done #####"
    print "These are unreachable: {0}".format(unreachable.resolve())
    print "These are visited: {0}".format(visited.resolve())

    assert len(unreachable.resolve())   == 5
    assert len(visited.resolve())       == 5

if __name__ == "__main__":
    import doctest
    doctest.testmod()
