#! /usr/bin/env python

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
Y_ENTITY = 1.55

Result = namedtuple("Result", ["height", "result", "duration"])

if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(1)

    # Navigate to the couch table
    nav_designator = EntityByIdDesignator(robot=robot, id="couch_table")
    nav_state = NavigateToSymbolic(robot, {nav_designator: "in_front_of"}, nav_designator)
    nav_state.execute()

    # Setup the test
    z_values = [DZ]
    while z_values[-1] < Z_MAX:
        z_values.append(z_values[-1] + DZ)

    results = []
    for z in z_values:

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

    # Print the results
    print("\n ---------- \n")
    for result in results:
        print "Height: {} --> {} (duration: {})".format(result.height, result.result, result.duration)
    print("\n ---------- \n")
