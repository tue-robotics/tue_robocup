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
from ed.msg import EntityInfo
import geometry_msgs.msg as gm
import std_msgs.msg as std
import inspect
import pprint
from robot_skills.arms import Arm

def check_resolve_type(designator, *allowed_types):
    """
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
    AttributeError: can't set attribute

    >>> assert(issubclass(d.resolve_type, str))"""

    def __init__(self, initial_value=None, resolve_type=None):
        super(Designator, self).__init__()

        self._current = initial_value
        if not resolve_type:
            self._resolve_type = type(initial_value)
        else:
            self._resolve_type = resolve_type

    def resolve(self):
        """Selects a new goal and sets it as the current value."""
        return self.current

    def _get_current(self):
        """The currently selected goal"""
        return self._current

    def _get_resolve_type(self):
        """The currently selected goal"""
        return self._resolve_type

    current = property(_get_current)
    resolve_type = property(_get_resolve_type)


class VariableDesignator(Designator):

    """
    A VariableDesignator simply contains a variable that can be set by anyone.
    This variable is encapsulated by a property called current.

    You can also set current = ...

    >>> v = VariableDesignator()  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...

    >>> v = VariableDesignator(resolve_type=str)
    >>> v.current = 'Works'
    >>> v.current
    'Works'

    >>> v.current = 666  # doctest: +IGNORE_EXCEPTION_DETAIL
    Traceback (most recent call last):
      ...
    TypeError: ...
    >>> assert(v.current == 'Works') #Unchanged
    """

    def __init__(self, initial_value=None, resolve_type=None):
        if not resolve_type:
            raise TypeError("VariableDesignator requires to set resolve_type to ensure user can use it")
        super(VariableDesignator, self).__init__(initial_value, resolve_type)

    def _set_current(self, value):
        if not issubclass(type(value), self.resolve_type):
            raise TypeError("Assigned value does not match resolve_type for {0}".format(self))
        self._current = value

    current = property(Designator._get_current, _set_current)


class LockingDesignator(Designator):
    """A designator's resolve() method may return a different object everytime.
    For some cases, this may be unwanted because a process has to be done with the same object.
    In that case, a designator resolving to a different object every time is not usable.
    A LockingDesignator will resolve to the same object after a call to .lock() and
    will only resolve to a different value after an unlock() call.

    >>> varying = VariableDesignator(0, int)
    >>> locking = LockingDesignator(varying)
    >>> assert(varying.resolve() == 0)
    >>> assert(locking.resolve() == 0)
    >>>
    >>> locking.lock()
    >>>
    >>> varying.current = 1
    >>> assert(varying.resolve() == 1)  # The value changed
    >>> assert(locking.resolve() == 0)  # This one sticks to the value it had when locked
    >>>
    >>> varying.current = 2
    >>> assert(varying.resolve() == 2)  # The value changed
    >>> assert(locking.resolve() == 0)  # This one sticks to the value it had when locked
    >>>
    >>> locking.unlock()
    >>>
    >>> varying.current = 3
    >>> assert(varying.resolve() == 3)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> locking.lock()
    >>>
    >>> varying.current = 4
    >>> assert(varying.resolve() == 4)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> varying.current = 5
    >>> assert(varying.resolve() == 5)  # The value changed
    >>> assert(locking.resolve() == 3)  # This one sticks to the value it had when locked
    >>>
    >>> locking.unlock()
    >>>
    >>> varying.current = 6
    >>> assert(varying.resolve() == 6)  # The value changed
    >>> assert(locking.resolve() == 6)  # This one sticks to the value it had when locked

    >>> assert(varying.resolve_type == int)
    >>> assert(locking.resolve_type == int)
    """

    def __init__(self, to_be_locked):
        super(LockingDesignator, self).__init__(resolve_type=to_be_locked.resolve_type)
        self.to_be_locked = to_be_locked
        self._locked = False

    def lock(self):
        self._locked = True

    def unlock(self):
        self._current = None
        self._locked = False

    def resolve(self):
        if self._locked:
            if self._current == None:
                self._current = self.to_be_locked.resolve()
                rospy.loginfo("{0} locked to {1}".format(self, str(self._current)[:10]))
            return self._current
        else:
            self._current = self.to_be_locked.resolve()
            rospy.loginfo("{0} not resolved to, but is not locked to {1}".format(self, str(self._current)[:10]))
            return self._current


class PointStampedOfEntityDesignator(Designator):

    def __init__(self, entity_designator):
        super(VariableDesignator, self).__init__(resolve_type=gm.PointStamped)
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


class EdEntityDesignator(Designator):

    """
    Resolves to an entity from an Ed query, (TODO: selected by some filter and
    criteria functions)
    """

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None,
        type_designator=None, center_point_designator=None, id_designator=None, debug=False):
        """Designates an entity of some type, within a radius of some center_point, with some id,
        that match some given criteria functions.
        @param robot the robot to use for Ed queries
        @param type the type of the entity to resolve to (default: any type)
        @param center_point combined with radius: a sphere to search an entity in
        @param radius combined with center_point: a sphere to search an entity in
        @param id the ID of the object to get info about
        @param parse whether to parse the data string associated with the object model or entity
        @param criteriafuncs a list of functions that take an entity and return a bool (True if criterium met)
        @param type_designator same as type but dynamically resolved trhough a designator. Mutually exclusive with type
        @param center_point_designator same as center_point but dynamically resolved trhough a designator. Mutually exclusive with center_point
        @param id_designator same as id but dynamically resolved trhough a designator. Mutually exclusive with id"""
        super(EdEntityDesignator, self).__init__(resolve_type=EntityInfo)
        self.ed = robot.ed
        if type != "" and type_designator != None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point != None and center_point_designator != None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point == None and center_point_designator == None:
            center_point = gm.PointStamped()
        if id != "" and id_designator != None:
            raise TypeError("Specify either id or id_designator, not both")

        self.type = type
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []

        if type_designator: check_resolve_type(type_designator, str, list) #the resolve type of type_designator can be either st or list
        self.type_designator = type_designator

        if center_point_designator: check_resolve_type(center_point_designator, gm.PointStamped) #the resolve type of type_designator can be either st or list
        self.center_point_designator = center_point_designator

        if id_designator: check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def resolve(self):
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        if isinstance(_type, list):
            _type = "" #Do the check not in Ed but in code here
            typechecker = lambda entity: entity.type in _type
            _criteria += [typechecker]

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if self.debug:
            import ipdb; ipdb.set_trace()
        if entities:
            for criterium in _criteria:
                entities = filter(criterium, entities)
                criterium_code = inspect.getsource(criterium)
                rospy.logdebug("Criterium {0} leaves {1} entities: {2}".format(
                              criterium_code, len(entities), pprint.pformat([ent.id for ent in entities]))
                              )

            if entities:
                self._current = entities[0]  # TODO: add sortkey
                return self.current

        raise DesignatorResolvementError(
                "No entities found in {0}".format(self))

    def __repr__(self):
        return "EdEntityDesignator(robot, type={0}, center_point={1}, radius={2}, id={3}, parse={4}, criteriafuncs={5})".format(
            self.type, str(self.center_point).replace("\n", " "), self.radius, self.id, self.parse, self.criteriafuncs)


class AttrDesignator(Designator):

    """Get some attribute of the object a wrapped designator resolves to.
    For example:
    >>> d = Designator(object(), resolve_type=object)
    >>> #Get the __doc__ attribute of the object that d resolves to. d is an object and d.__doc__ is 'The most base type'
    >>> wrapped = AttrDesignator(d, '__doc__', resolve_type=str)
    >>> wrapped.resolve() == 'The most base type'
    True

    >>> assert(issubclass(wrapped.resolve_type, str))
    """

    def __init__(self, orig, attribute, resolve_type=None):
        super(AttrDesignator, self).__init__(resolve_type=resolve_type)
        self.orig = orig
        self.attribute = attribute

    def resolve(self):
        return self.orig.resolve().__getattribute__(self.attribute)


class FuncDesignator(Designator):

    """Apply a function to the object a wrapped designator resolves to
    For example:
    >>> d = Designator("Hello")
    >>> wrapped = FuncDesignator(d, len, resolve_type=int) #Determine the len of whatever d resolves to
    >>> wrapped.resolve()
    5

    >>> assert(issubclass(wrapped.resolve_type, int))
    """

    def __init__(self, orig, func, resolve_type=None):
        super(FuncDesignator, self).__init__(resolve_type=resolve_type)
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

        super(ArmDesignator, self).__init__(resolve_type=Arm)
        self.all_arms = all_arms
        self.preferred_arm = preferred_arm

        if not self.preferred_arm:
            self.preferred_arm = self.all_arms.values()[0]

        if not self.preferred_arm in self.all_arms.values():
            raise ValueError("The preferred arm is not in the list of arms. Preferred_arm should be one of the arms in the system")

    def resolve(self):
        if self.available(self.preferred_arm):
            return self.preferred_arm
        else:
            # import ipdb; ipdb.set_trace()
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
