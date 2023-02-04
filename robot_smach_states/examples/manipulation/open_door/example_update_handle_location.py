# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot

# Robot Smach States
from robot_smach_states.manipulation.open_door import UpdateHandleLocation, Door
from robot_smach_states.util.designators import Designator, EntityByIdDesignator


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the UpdateHandleLocation state")
    parser.add_argument("-r", "--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("-d", "--door-entity", default="door_inside", help="ID of the door to pass")
    parser.add_argument("-a", "--area", default="in_front_of", help="Area of the entity to navigate to")
    args = parser.parse_args()

    # Create node and robot
    rospy.init_node("test_update_handle_location")

    robot = get_robot(args.robot)
    door = robot.ed.get_entity(uuid=args.door_entity)
    door_des = Designator(Door(door))

    sm = UpdateHandleLocation(robot, door_des)
    sm.execute()
