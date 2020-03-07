#! /usr/bin/env python

# ROS
import PyKDL as kdl
import rospy

# Robot Skills
from robot_skills.hero_parts.hero_arm import HeroArm
from robot_skills.get_robot import get_robot_from_argv
from robot_skills.util.kdl_conversions import FrameStamped

if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(1)
    # noinspection PyProtectedMember
    arm = robot._arms.values()[0]  # type: HeroArm

    t_start = rospy.Time.now()
    dz = 0.05
    z_max = 1.2
    heights = [dz]
    while heights[-1] < z_max:
        heights.append(heights[-1] + dz)
    for z in heights:

        goal_pose = kdl.Frame(kdl.Vector(0, 0, z))
        goal = FrameStamped(goal_pose, "map")
        base_offset = arm.get_base_offset(goal)

        print("Height: {} --> {}".format(z, base_offset))
    t_end = rospy.Time.now()
    duration = (t_end - t_start).to_sec()
    print("Computation took {} seconds for {} heights, {} per height".format(
        duration, len(heights), duration/len(heights)
    ))
