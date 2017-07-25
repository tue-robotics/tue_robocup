# ROS
import rospy
import smach

# TU/e Robotics
from robocup_knowledge import load_knowledge
import robot_smach_states as states
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds


class SetTable(smach.StateMachine):
    """ Tries to get three objects from the fetch location and put them on the table """
    # def __init__(self, robot):
    #     smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
    #
    #     with self:
    #         # Part I: Set a table
    #         smach.StateMachine.add('ENTER_ROOM',  # Enter the room
    #                                states.StartChallengeRobust(robot, knowledge.initial_pose),
    #                                transitions={'Done': 'ANNOUNCEMENT',
    #                                             'Aborted': 'Aborted',
    #                                             'Failed': 'Aborted'})
