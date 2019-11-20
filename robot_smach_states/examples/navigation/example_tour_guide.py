import rospy
import argparse

# TU/e Robotics
from robot_smach_states.navigation import guidance
from robot_skills.get_robot import get_robot


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="While the robot is driving to a nav goal, this state tells the rooms"
                                                 "and furniture the robot encounters")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("--x_threshold", type=float, default=0.75, help="threshold determining when a piece of "
                                                                        "furniture is close enough to be described [m]")
    parser.add_argument("--y_threshold", type=float, default=1.5, help="threshold determining when a piece of furniture"
                                                                       " is close enough to be described [m]")
    args = parser.parse_args()

    # Create node, robot and tour_guide instance
    rospy.init_node("example_tour_guide")
    r = get_robot(args.robot)
    s = guidance.TourGuide(r, args.x_threshold, args.y_threshold)

    # Initialize
    s.initialize()

    # Keep explaining the robot movement
    while not rospy.is_shutdown():
        rospy.loginfo(s.describe_near_objects())

