#! /usr/bin/python

from __future__ import print_function

import sys

import rospy
import smach


from robot_smach_states.util.startup import startup

from robot_smach_states.navigation.find import Find
from robot_smach_states.manipulation.grab import Grab
from robot_smach_states.manipulation.place import Place
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator
from robot_smach_states.util.designators import Designator, VariableDesignator, EdEntityDesignator, EntityByIdDesignator
from robot_smach_states.util.designators.arm import UnoccupiedArmDesignator, OccupiedArmDesignator

from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")


def setup_statemachine(robot, item_description, location_id):
    """
    Continuously pick and place an object on a piece of furniture
    """
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    unoccupied_arm_des = UnoccupiedArmDesignator(robot)
    occupied_arm_des = OccupiedArmDesignator(robot, {})

    item_description_designator = Designator(item_description)
    supporting_entity = EntityByIdDesignator(location_id)
    found_entity_designator = VariableDesignator()
    place_position = EmptySpotDesignator(robot, EdEntityDesignator(
                                         robot, id=supporting_entity.id),
                                         occupied_arm_des,
                                         area="on_top_of"
                                         )

    with sm:
        smach.StateMachine.add('FIND', Find(robot, common, supporting_entity, item_description_designator, found_entity_designator.writeable()),
                               transitions={'succeeded': 'GRAB',
                                            'inspect_failed': 'failed',
                                            'not_found': 'FIND'})

        smach.StateMachine.add('PICK', Grab(robot, found_entity_designator, unoccupied_arm_des),
                               transitions={'done': 'PLACE',
                                            'failed': 'failed'})

        smach.StateMachine.add('PLACE', Place(robot, found_entity_designator, place_position, occupied_arm_des),
                               transitions={'done': 'FIND',
                                            'failed': 'failed'})


if __name__ == "__main__":
    robot_name = rospy.get_param('~robot_name')
    item_description = rospy.get_param('~item_description', 'coke')
    location = rospy.get_param('~location', 'dinner_table')

    rospy.loginfo("[DEMO] Parameters:")
    rospy.loginfo("[DEMO] robot_name = {}".format(robot_name))
    rospy.loginfo("[DEMO] Will pick and place {} on the {}".format(item_description, location))

    startup(setup_statemachine, (item_description, location), robot_name=robot_name)
