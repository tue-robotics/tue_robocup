#! /usr/bin/env python

# ROS
import rospy

# TU/e Robotics
# import GripperTypes and PseudoObjects to make them available for the user of these designators.
from robot_skills.arms import PublicArm, GripperTypes, PseudoObjects
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.utility import LockingDesignator


__author__ = 'loy'


class ArmDesignator(Designator):
    """Resolves to an instance of the Arm-class in robot_skills.
    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot, {required_arm_name: 'left'})
    >>> assert a.resolve()._arm == robot._arms['left']
    """

    def __init__(self, robot, arm_properties, name=None):
        """
        Initialize a new ArmDesignator for a robot, with the desired properties of the arm.

        :param robot: Robot object.
        :type  robot: robot_skills.robot.Robot object.

        :param arm_properties: Required and desired properties of the arm.
        :type  arm_properties: Map with arm properties, parameters of Robot.get_arm().

        :param name: Optional name of the arm designator.
        """
        super(ArmDesignator, self).__init__(resolve_type=PublicArm, name=name)
        self.robot = robot
        self.arm_properties = arm_properties

        self._locker = None

    def _resolve(self):
        return self.robot.get_arm(**self.arm_properties)

    def lockable(self):
        if not self._locker:
            self._locker = LockingDesignator(self)
        return self._locker


class UnoccupiedArmDesignator(ArmDesignator):
    """An UnoccupiedArmDesignator resolves to an arm that is not occupied by an entity.
    .resolve() returns None when no such arm can be found
    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>>
    >>> robot.arms['left'].occupied_by = None
    >>> robot.arms['right'].occupied_by = None
    >>> empty_arm_designator = UnoccupiedArmDesignator(robot, {})
    >>> arm_to_use_for_first_grab = empty_arm_designator.resolve()
    >>> assert arm_to_use_for_first_grab._arm in robot.arms
    >>>
    >>> # Grab the 1st item.
    >>> arm_to_use_for_first_grab.occupied_by = "entity1"
    >>>
    >>> # Find arm to grab the second item.
    >>> arm_to_use_for_second_grab = empty_arm_designator.resolve()
    >>> assert arm_to_use_for_second_grab is not None
    >>> assert arm_to_use_for_second_grab._arm != arm_to_use_for_first_grab._arm
    >>>
    >>> # Grab the 2nd item.
    >>> arm_to_use_for_second_grab.occupied_by = "entity2"
    >>>
    >>> # You can't do 3 grabs with a 2 arms robot without placing an entity first, so this will fail to resolve for a 3rd time
    >>> arm_to_use_for_third_grab = empty_arm_designator.resolve()
    >>> assert arm_to_use_for_third_grab == None
    """
    def __init__(self, robot, arm_properties, name=None):
        arm_properties['required_objects'] = [PseudoObjects.EMPTY]
        super(UnoccupiedArmDesignator, self).__init__(robot, arm_properties, name=name)


class OccupiedArmDesignator(ArmDesignator):
    """An OccupiedArmDesignator resolves to an arm that is occupied by an entity.
    .resolve() returns None when no such arm can be found
    """
    def __init__(self, robot, arm_properties, name=None):
        arm_properties['required_objects'] = [PseudoObjects.ANY]
        super(OccupiedArmDesignator, self).__init__(robot, arm_properties, name=name)


class ArmHoldingEntityDesignator(ArmDesignator):
    """An ArmDesignator resolving to an arm that is holding a specific entity.
    .resolve() returns None when no such arm can be found

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> a = ArmDesignator(robot, {required_arm_name: 'left'})
    >>> assert a.resolve()._arm == robot.arms['left']

    >>> leftArm = robot.arms['left']
    >>> leftArm.occupied_by = None

    >>> rightArm = robot.arms['right']
    >>> rightArm.occupied_by = "entity3"

    >>> entity_designator = Designator("entity3")
    >>> holding_arm_designator = ArmHoldingEntityDesignator(robot, {required_objects: entity_designator})
    >>> arm_to_use_for_placing_entity3 = holding_arm_designator.resolve()
    >>> assert(arm_to_use_for_placing_entity3._arm == rightArm)
    >>>
    >>> #place the object
    >>> rightArm.occupied_by = None
    >>>
    >>> #After placing the item, there is no arm holding the item anymore
    >>> arm_to_use_for_second_place = holding_arm_designator.resolve()
    >>> assert arm_to_use_for_second_place == None
    """
    def __init__(self, robot, entity_designator, arm_properties, name=None):
        super(ArmHoldingEntityDesignator, self).__init__(robot, arm_properties, name=name)
        self.entity_designator = entity_designator

    def _resolve(self):
        entity = self.entity_designator.resolve()
        if not entity:
            rospy.logdebug('ArmHoldingEntityDesignator: Entity to find in the arm does not exist')
            return None
        self.arm_properties['required_objects'] = entity
        return super(ArmHoldingEntityDesignator, self).resolve()


if __name__ == "__main__":
    import doctest
    doctest.testmod()
