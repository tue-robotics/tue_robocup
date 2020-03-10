#! /usr/bin/env python

"""
This script has the robot navigate to the couch table. Next, on a number of different heights, a (virtual) entity is
 added to the world model, the robot tries to grasp it and subsequently it is removed again. At the end, the results
 (height, outcome, duration) at each height are printed.

Since the position of the entity (X_ENTITY, Y_ENTITY) and the id of the supporting entity (SUPPORTING_ENTITY_ID) are
heavily dependent, these are not parametrized. You can provide a different robot though.
"""

# System
from collections import namedtuple

# ROS
import PyKDL as kdl
import rospy

# TU/e Robotics
from robot_smach_states.manipulation import Grab
from robot_smach_states.navigation import NavigateToSymbolic
from robot_smach_states.util.designators import EntityByIdDesignator, UnoccupiedArmDesignator

# Robot Skills
from robot_skills import arms
from robot_skills.hero_parts.hero_arm import HeroArm
from robot_skills.get_robot import get_robot_from_argv
from robot_skills.util.kdl_conversions import FrameStamped

DZ = 0.1
Z_MAX = 1.2

X_ENTITY = 3.2
Y_ENTITY = 1.

SUPPORTING_ENTITY_ID = "couch_table"

Result = namedtuple("Result", ["height", "result", "duration"])

if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(1)

    # Navigate to the couch table
    nav_designator = EntityByIdDesignator(robot=robot, id=SUPPORTING_ENTITY_ID)
    nav_state = NavigateToSymbolic(robot, {nav_designator: "in_front_of"}, nav_designator)
    nav_state.execute()

    # Setup the test
    z_values = [DZ]
    while z_values[-1] < Z_MAX:
        z_values.append(z_values[-1] + DZ)

    results = []
    for test_nr, z in enumerate(z_values):

        print("\nTest nr {} of {} at height {}".format(test_nr + 1, len(z_values), z))

        # Add an entity to ed
        grasp_entity_id = "test_entity"
        entity_pose = FrameStamped(kdl.Frame(kdl.Vector(X_ENTITY, Y_ENTITY, z)), "map")
        robot.ed.update_entity(id=grasp_entity_id, frame_stamped=entity_pose)

        # Try to grasp the entity
        item_designator = EntityByIdDesignator(robot=robot, id=grasp_entity_id)
        t_start = rospy.Time.now()
        grasp_state = Grab(
            robot,
            item_designator,
            UnoccupiedArmDesignator(robot, {
                "required_goals": ["reset", "carrying_pose"],
                "required_trajectories": ["prepare_grasp"],
                "required_gripper_types": [arms.GripperTypes.GRASPING]
            }))
        grasp_result = grasp_state.execute()
        grasp_duration = (rospy.Time.now() - t_start).to_sec()
        results.append(Result(z, grasp_result, grasp_duration))

        # Lose the entity
        # noinspection PyProtectedMember
        arm = robot._arms.values()[0]  # type: HeroArm
        arm.send_gripper_goal(state=arms.GripperState.OPEN)

        if rospy.is_shutdown():
            break

    # Print the results
    print("\n ---------- \n")
    for result in results:
        print "Height: {} --> {} (duration: {})".format(result.height, result.result, result.duration)
    print("\n ---------- \n")
