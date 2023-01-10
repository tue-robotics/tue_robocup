#!/usr/bin/python

# System
import argparse

import rospy
import PyKDL as kdl
from pykdl_ros import FrameStamped

from robot_smach_states.util.designators import EntityByIdDesignator, ArmDesignator, Designator
from robot_smach_states.navigation import NavigateTo
from robot_skills.get_robot import get_robot
from robot_smach_states.navigation.constraint_functions import arms_reach_constraint, radius_constraint, \
    combine_constraints

if __name__ == "__main__":
    """
    Navigates to an item on the dinner_table with a specified constraint
    """

    parser = argparse.ArgumentParser(description="Navigate to an item on a furniture with arms_reach_constraint")
    parser.add_argument("furniture_id", default="dinner_table", type=str, help="Furniture to use (Default: "
                                                                               "dinner_table)")
    parser.add_argument("x", default=4, type=float, help="x-coordinate (in map) of the imaginary object (Default: 4)")
    parser.add_argument("y", default=1, type=float, help="y-coordinate (in map) of the imaginary object (Default: 1)")
    parser.add_argument("z", default=0.75, type=float, help="z-coordinate (in map) of the imaginary object (Default: "
                                                            "0.75)")
    parser.add_argument("--robot", default="hero", type=str, help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_navigate_with_constraint")

    robot = get_robot(args.robot)
    arm_designator = ArmDesignator(robot)

    table = EntityByIdDesignator(robot, args.furniture_id)
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(args.x, args.y, args.z)),
                        stamp=rospy.Time(0),
                        frame_id="map")

    pose_designator = Designator(pose)

    reach_fun = lambda: arms_reach_constraint(pose_designator, arm_designator)
    table_fun = lambda: radius_constraint(table, 0.38 + 0.1, 0.1)

    navigate_to = NavigateTo(robot, lambda: combine_constraints([reach_fun, table_fun]))
    navigate_to.execute()
