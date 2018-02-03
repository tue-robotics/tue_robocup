#! /usr/bin/env python

# ROS
import rospy

# TU/e Robotics
from robot_skills.arms import Arm
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.utility import LockingDesignator


__author__ = 'loy'


class ArmDesignator(Designator):
    """Resolves to an instance of the Arm-class in robot_skills.
    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot.arms, robot.arms['left'])
    >>> assert a.resolve() == robot.arms['left']
    """

    def __init__(self, all_arms, preferred_arm=None, name=None):
        """Initialize a new ArmDesignator with a collection of arms available on the robot and an arm that is preferred for the given operations.
        @param all_arms a dictionary of arms available on the robot
        @param preferred_arm the arm that is preferred for the operations that use this designator"""

        super(ArmDesignator, self).__init__(resolve_type=Arm , name=name)
        if not all_arms:
            raise AssertionError("all_arms cannot be None or empty list")

        self.all_arms = all_arms
        self.preferred_arm = preferred_arm

        if not self.preferred_arm:
            self.preferred_arm = self.all_arms.values()[0]

        if not self.preferred_arm in self.all_arms.values():
            raise ValueError("The preferred arm is not in the list of arms. Preferred_arm should be one of the arms in the system")

        self._locker = None

    def _resolve(self):
        if self.available(self.preferred_arm) and self.preferred_arm.operational:
            return self.preferred_arm
        else:
            # import ipdb; ipdb.set_trace()
            arm2name = {arm:name for name,arm in self.all_arms.items()}
            all_arms = self.all_arms.values()
            rospy.loginfo("Robot has %d arms" % len(all_arms))

            available_arms = filter(self.available, all_arms)
            rospy.loginfo("Found {} available arms: {}".format(len(available_arms), [arm2name[arm] for arm in available_arms]))

            operational_arms = filter(lambda arm: arm.operational, available_arms)
            rospy.loginfo("Found {} operational arms: {}".format(len(operational_arms), [arm2name[arm] for arm in operational_arms]))

            if any(operational_arms):
                selected_arm = operational_arms[0]
                rospy.logdebug("Selected arm: {arm}".format(arm=selected_arm))
                return selected_arm
            else:
                rospy.logerr("ArmDesignator {0} could not resolve to an arm".format(self))
                return None

    def available(self, arm):
        """Check whether the given arm is available for some function."""
        return True

    def lockable(self):
        if not self._locker:
            self._locker = LockingDesignator(self)
        return self._locker


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
    def __init__(self, all_arms, preferred_arm, name=None):
        super(UnoccupiedArmDesignator, self).__init__(all_arms, preferred_arm, name=name)

    def available(self, arm):
        """Check that there is no entity occupying the arm"""
        return arm.occupied_by == None


class OccupiedArmDesignator(ArmDesignator):
    """An OccupiedArmDesignator resolves to an arm that is occupied by an entity.
    .resolve() returns None when no such arm can be found
    """
    def __init__(self, all_arms, preferred_arm, name=None):
        super(OccupiedArmDesignator, self).__init__(all_arms, preferred_arm, name=name)

    def available(self, arm):
        """Check that there is an entity occupying the arm"""
        return arm.occupied_by is not None


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
    def __init__(self, all_arms, entity_designator, name=None):
        super(ArmHoldingEntityDesignator, self).__init__(all_arms, name=name)

        self.entity_designator = entity_designator

    def available(self, arm):
        """Check that the arm is occupied by the entity referred to by the entity_designator"""
        # Check that the designator actually resolved to anything.
        # Otherwise, the check could become None == None, which is also True
        entity = self.entity_designator.resolve()
        if entity:
            rospy.logdebug("{arm} is occupied by entity we're looking for: {ent}".format(arm=arm, ent=entity))
            return arm.occupied_by == entity
        else:
            rospy.logwarn("Entity is None and {arm} is {un}occupied".format(arm=arm, un="un" if arm.occupied_by == None else ""))
            return False

if __name__ == "__main__":
    import doctest
    doctest.testmod()
