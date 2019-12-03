import rospy
import argparse

# TU/e Robotics
from robot_smach_states.human_interaction import give_directions
import robot_smach_states.util.designators as ds
from robot_skills.get_robot import get_robot


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Starting from the current robot position and orientation give "
                                                 "directions to the desired target object")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("target_entity_id", type=str, help="Entity id of the target object")
    args = parser.parse_args()

    # Create node
    rospy.init_node("test_give_directions")
    robot = get_robot(args.robot)
    e_id = args.target_entity_id

    # Instantiate GuideToSymbolic machine
    state = give_directions.GiveDirections(r, ds.EntityByIdDesignator(r, id=e_id))
    # Execute the state
    state.execute()
