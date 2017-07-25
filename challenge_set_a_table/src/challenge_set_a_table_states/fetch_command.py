#! /usr/bin/env python

# ROS
import rospy
import smach

# TU/e Robotics
import hmi
import robot_smach_states.util.designators as ds

from robot_smach_states import Initialize, Say
from robot_smach_states.util.startup import startup


def fetch(robot, time_out=15.0):

    confirmed = False
    robot.speech.speak('What can I do for you, master?')

    # Listen for the new task
    i = 0
    while not confirmed and i < 10:
        try:
            sentence, semantics = robot.hmi.query('', 'T -> set the table | clean the table', 'T')

            # check if we have heard this correctly
            robot.speech.speak('I heard %s, is this correct?' % sentence)
            try:
                result = robot.hmi.query('', 'T -> yes | no', 'T').sentence
                if result == 'yes':
                    confirmed = True
                elif result == 'no':
                    robot.speech.speak('Sorry, please repeat')
                    pass

            except hmi.TimeoutException:
                confirmed = True
                # robot did not hear the confirmation, so lets assume its correct

        except hmi.TimeoutException:
            i += 1

    return 'heard'


class HearFetchCommand(smach.State):
        def __init__(self, robot, time_out=15.0):
                smach.State.__init__(self, outcomes=["heard"])
                self.robot = robot
                self.time_out = time_out

        def execute(self, userdata):

                self.robot.head.look_at_standing_person()

                return fetch(self.robot, time_out=self.time_out)


# Standalone testing -----------------------------------------------------------------~
class TestFetchCommand(smach.StateMachine):
        def __init__(self, robot):
                smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

                with self:
                        smach.StateMachine.add('INITIALIZE',
                                               Initialize(robot),
                                               transitions={'initialized': 'FETCH_COMMAND',
                                                            'abort': 'Aborted'})

                        smach.StateMachine.add('FETCH_COMMAND',  # Hear "set the table"
                                               HearFetchCommand(robot, 15.0),
                                               transitions={'heard': 'END_CHALLENGE'})

                        # End
                        smach.StateMachine.add('END_CHALLENGE',
                                               Say(robot, "I am finally free!"),
                                               transitions={'spoken': 'Done'})

                        ds.analyse_designators(self, "set_a_table")


if __name__ == "__main__":
        rospy.init_node('set_a_table_exec')

        startup(TestFetchCommand, challenge_name="challenge_set_a_table")
