import rospy
import sys
import robot_smach_states.util.designators as ds

from robot_skills.get_robot import get_robot_from_argv
from robot_smach_states.human_interaction import give_directions


if __name__ == "__main__":

    assert len(sys.argv) == 3, "Please provide the robot name and the entity id of the object to give directions to," \
                               "e.g., 'python guidance.py amigo bed'"

    # Create node, robot and toggle interface
    rospy.init_node("test_give_directions")
    r = get_robot_from_argv(1)
    e_id = sys.argv[2]

    # Instantiate GuideToSymbolic machine
    s = give_directions.GiveDirections(r,
                                    ds.EntityByIdDesignator(r, id=e_id)
                                    )
    # Execute the state
    s.execute()
