# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from challenge_find_my_mates.locate_people import LocatePeople


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Locate all people in a room")
    parser.add_argument("room_id", type=float, help="uuid of the room in which to locate people")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_locate_people")

    robot = get_robot(args.robot)
    room_id = args.room_id

    state = LocatePeople(robot, room_id)
    state.execute()
