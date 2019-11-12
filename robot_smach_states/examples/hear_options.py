#ROS
import rospy
import sys

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states.human_interaction import HearOptions

if __name__ == "__main__":

    assert len(sys.argv) == 2, "Please provide the robot name" \
                               "e.g., 'python example_handover_to_human.py hero'"

    rospy.init_node('test_hear_options')

    robot = get_robot_from_argv(index=1)

    Hear_Option_State = HearOptions(robot, ['no', 'yes'])

    Hear_Option_State.execute()
