#ROS
import rospy
import sys

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states.human_interaction import Say


if __name__ == "__main__":

    assert len(sys.argv) == 2, "Please provide the robot name" \
                               "e.g., 'python example_say.py hero'"

    rospy.init_node('test_say')

    robot = get_robot_from_argv(index=1)

    sentence = 'I have said something useful'

    say_state = Say(robot, sentence)

    say_state.execute()
