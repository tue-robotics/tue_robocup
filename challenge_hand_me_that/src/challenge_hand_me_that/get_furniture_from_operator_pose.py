#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
from robot_skills import Hero
from robot_smach_states import Entity, VariableDesignator
from robot_smach_states.util.designators import is_writeable, check_type
from smach import StateMachine, cb_interface, CBState


class GetFurnitureFromOperatorPose(StateMachine):
    def __init__(self, robot, furniture_designator):
        StateMachine.__init__(self, outcomes=['done'])

        is_writeable(furniture_designator)
        check_type(furniture_designator, Entity)

        @cb_interface(outcomes=['done'])
        def _prepare_operator(_):
            robot.speech.speak("Let's point")  # type: Hero
            return 'done'

        with self:
            self.add('PREPARE_OPERATOR', CBState(_prepare_operator), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    furniture_designator = VariableDesignator(resolve_type=Entity)
    hero = Hero()
    hero.reset()
    GetFurnitureFromOperatorPose(hero, furniture_designator.writeable).execute()

    print(furniture_designator.resolve())
