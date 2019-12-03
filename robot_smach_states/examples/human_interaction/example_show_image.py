import argparse

#ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.human_interaction import ShowImageState


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Show an image on the robot display.")
    parser.add_argument(
        "file_path",
        type=str,
        help="String describing the path to the image (absolute path incl. file extension)"
    )
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('example_show_image')

    robot = get_robot(args.robot)
    file_path = args.file_path

    show_image_state = ShowImageState(robot, file_path)
    show_image_state.execute()
