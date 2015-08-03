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
import inspect
import pprint

import rospy

from robot_skills.arms import Arm
from robot_smach_states.util.geometry_helpers import *
from robot_smach_states.util.designators.core import Designator, VariableDesignator


class ListElementDesignator(Designator):
    """Wraps a designator(resolve_type=[X]) (i.e.) a list of objects with type X and reduces that to a single item of type X.
    The reduction takes some criteria and sorting into account
    >>> l = Designator([1,2,3,4,5], resolve_type=[int])
    >>> reduce = ListElementDesignator(l, sortkey=lambda elem: elem)
    >>> reduce.resolve()
    1

    >>> l = Designator(["noot", "broer", "aap"], resolve_type=[str]) #
    >>> reduc = ListElementDesignator(l, sortkey=lambda elem: len(elem), minmax=max) #Get the element with the max length
    >>> reduc.resolve()
    'broer'

    >>> l = Designator(["noot", "broer", "aap", "mies"], resolve_type=[str])
    >>> #The criterium removes all elements with an e
    >>> #Get the element with the min length
    >>> reduc = ListElementDesignator(l, sortkey=lambda elem: len(elem), minmax=min, criteriafuncs=[lambda elem: not 'e' in elem])
    >>> reduc.resolve()
    'aap'

    >>> l = Designator(["noot", "broer", "aap", "mies"], resolve_type=[str])
    >>> #The criterium removes all elements with an e
    >>> #Get the element with the min length
    >>> reduc = ListElementDesignator(l) #Not sorting or selecting will just return the 'smallest' element (because min is the default for minmax)
    >>> reduc.resolve()
    'aap'
    """

    def __init__(self, list_designator, sortkey=None, criteriafuncs=None, minmax=min):
        if not len(list_designator.resolve_type) == 1:
            raise TypeError("The list_designator should indicate the type of the list elements, e.g. [str] for a list of str's")
        element_type = list_designator.resolve_type[0]
        super(ListElementDesignator, self).__init__(self, resolve_type=element_type)

        self.list_designator = list_designator
        self.sortkey = sortkey
        self.minmax = minmax
        self.criteriafuncs = criteriafuncs or []

    def resolve(self):
        elements = self.list_designator.resolve()
        if elements:
            for criterium in self.criteriafuncs:
                elements = filter(criterium, elements)
                criterium_code = inspect.getsource(criterium)
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(elements)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([elem for elem in elements])))

            if elements:
                if self.sortkey:
                    self._current = self.minmax(elements, key=self.sortkey)
                else:
                    self._current = self.minmax(elements)
                return self.current

        rospy.logerr("No elements found in {0}".format(self))
        return None


class ArmDesignator(Designator):
    """Resolves to an instance of the Arm-class in robot_skills.
    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot.arms, robot.arms['left'])
    >>> assert a.resolve() == robot.arms['left']
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
        if self.available(self.preferred_arm) and self.preferred_arm.operational:
            return self.preferred_arm
        else:
            # import ipdb; ipdb.set_trace()
            available_arms = filter(self.available, self.all_arms.values())
            rospy.loginfo("Found %d available arms" % len(available_arms))
            available_arms = filter(lambda arm: arm.operational, available_arms)
            rospy.loginfo("For those arms, there are %d arms operational" % len(available_arms))
            if any(available_arms):
                return available_arms[0]
            else:
                rospy.logerr("ArmDesignator {0} could not resolve to an arm".format(self))
                return None

    def available(self, arm):
        """Check whether the given arm is available for some function."""
        return True


class UnoccupiedArmDesignator(ArmDesignator):
    """An UnoccupiedArmDesignator resolves to an arm that is not occupied by an entity.
    .resolve() returns None when no such arm can be found
    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot.arms, robot.arms['left'])
    >>> assert a.resolve() == robot.arms['left']

    >>> robot.arms['left'].occupied_by = None
    >>> robot.arms['right'].occupied_by = None
    >>> empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.arms['right'])
    >>> arm_to_use_for_first_grab = empty_arm_designator.resolve()
    >>> assert(arm_to_use_for_first_grab == robot.arms['right'])
    >>>
    >>> #Grab the 1st item with the robot.arms['right']
    >>> robot.arms['right'].occupied_by = "entity1"
    >>> arm_to_use_for_second_grab = empty_arm_designator.resolve()
    >>> assert(arm_to_use_for_second_grab == robot.arms['left'])
    >>>
    >>> #Grab the 2nd item with the robot.arms['right']
    >>> robot.arms['left'].occupied_by = "entity2"
    >>> #You can't do 3 grabs with a 2 arms robot without placing an entity first, so this will fail to resolve for a 3rd time
    >>> arm_to_use_for_third_grab = empty_arm_designator.resolve()
    >>> assert arm_to_use_for_third_grab == None
    """
    def __init__(self, all_arms, preferred_arm):
        super(UnoccupiedArmDesignator, self).__init__(all_arms, preferred_arm)

    def available(self, arm):
        """Check that there is no entity occupying the arm"""
        return arm.occupied_by == None


class ArmHoldingEntityDesignator(ArmDesignator):
    """An UnoccupiedArmDesignator resolves to an arm that is not occupied by an entity.
    .resolve() returns None when no such arm can be found

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot.arms, robot.arms['left'])
    >>> assert a.resolve() == robot.arms['left']

    >>> leftArm = robot.arms['left']
    >>> leftArm.occupied_by = None

    >>> rightArm = robot.arms['right']
    >>> rightArm.occupied_by = "entity3"

    >>> entity_designator = Designator("entity3")
    >>> holding_arm_designator = ArmHoldingEntityDesignator(robot.arms, entity_designator)
    >>> arm_to_use_for_placing_entity3 = holding_arm_designator.resolve()
    >>> assert(arm_to_use_for_placing_entity3 == rightArm)
    >>>
    >>> #place the object
    >>> rightArm.occupied_by = None
    >>>
    >>> #After placing the item, there is no arm holding the item anymore
    >>> arm_to_use_for_second_place = holding_arm_designator.resolve()
    >>> assert arm_to_use_for_second_place == None
    """
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
