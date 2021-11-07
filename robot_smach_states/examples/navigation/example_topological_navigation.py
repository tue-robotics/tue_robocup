# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot

# Robot Smach States
from robot_smach_states.navigation.topological_navigation import TopologicalNavigateTo
from robot_smach_states.util.designators import Designator, EntityByIdDesignator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the topological navigation state")
    parser.add_argument("-r", "--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("-e", "--entity", default="door", help="ID of the entity to navigate to")
    parser.add_argument("-a", "--area", default="", help="Area of the entity to navigate to")
    args = parser.parse_args()

    # Create node and robot
    rospy.init_node("test_topological_navigation")

    robot = get_robot(args.robot)

    entity = robot.ed.get_entity(args.entity)
    if entity.is_a("furniture") and not args.area:
        area = "in_front_of"
    else:
        area = args.area

    sm = TopologicalNavigateTo(
        robot,
        EntityByIdDesignator(robot, entity.id),
        Designator(area, str),
    )
    sm.execute()
