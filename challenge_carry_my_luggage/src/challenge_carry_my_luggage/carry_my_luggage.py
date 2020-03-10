import smach

# ROS

# TUe
from robot_smach_states.human_interaction import Say


class CarryMyLuggage(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('Say_start', Say(robot, 'Test initial smach state'),
                                   transitions={'spoken': 'succeeded'})
