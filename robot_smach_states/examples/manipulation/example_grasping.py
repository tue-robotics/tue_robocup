# System
import argparse

# ROS
import PyKDL as kdl
import rospy

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.get_robot import get_robot
from robot_skills import arms

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation import Grab
from robot_skills.util.shape import RightPrism
from robot_skills.util.entity import Entity
import PyKDL as kdl


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Put an imaginary object in the world model and grasp it using the "
                                                 "'Grab' smach state")
    parser.add_argument("x", type=float, help="x-coordinate (in map) of the imaginary object")
    parser.add_argument("y", type=float, help="y-coordinate (in map) of the imaginary object")
    parser.add_argument("z", type=float, help="z-coordinate (in map) of the imaginary object")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_grasping")

    robot = get_robot(args.robot)

    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(args.x, args.y, args.z)),
                        frame_id="/map")
    robot.ed.update_entity(id=entity_id, frame_stamped=pose)
    shape = RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)], -0.1, 0.1)
    item = Entity(entity_id, "test_type", pose.frame_id, pose.frame, shape, None, None, None)

    item = ds.Designator(item)

    arm = ds.UnoccupiedArmDesignator(robot, arm_properties={})

    grab_state = Grab(robot, item, arm)
    grab_state.execute()
