# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot

# Robot Smach States
from robot_smach_states.manipulation.open_door import PassDoor
from robot_smach_states.util.designators import Designator, EntityByIdDesignator


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the topological navigation state")
    parser.add_argument("-r", "--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("-d", "--door-entity", default="door", help="ID of the door to pass")
    parser.add_argument("-a", "--area", default="in_front_of", help="Area of the entity to navigate to")
    args = parser.parse_args()

    # Create node and robot
    rospy.init_node("test_pass_door")

    robot = get_robot(args.robot)
    print(args)
    door_designator = EntityByIdDesignator(robot=robot, id=args.door_entity)
