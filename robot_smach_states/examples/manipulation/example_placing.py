#! /usr/bin/env python

# System
import argparse

# ROS
import PyKDL as kdl
from pykdl_ros import FrameStamped
import rospy

# TU/e Robotics
from ed.entity import Entity
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation import Place


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Place an imaginary object at provided x, y, z coordinates using the "
                                                 "'Place' smach state")
    parser.add_argument("x", type=float, help="x-coordinate (in map)")
    parser.add_argument("y", type=float, help="y-coordinate (in map)")
    parser.add_argument("z", type=float, help="z-coordinate (in map)")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_placing")

    robot = get_robot(args.robot)

    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(args.x, args.y, args.z)),
                        stamp=rospy.Time.now(),
                        frame_id="map")
    item = Entity("dummy_id", "dummy_type", None, None, None, None, None, None)

    arm = ds.ArmDesignator(robot).lockable()

    place_state = Place(robot, ds.Designator(item), ds.Designator(pose), arm)
    place_state.execute()
