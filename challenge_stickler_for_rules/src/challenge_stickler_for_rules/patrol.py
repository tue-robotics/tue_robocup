"""
Module contains states to patrol the environment while navigating to a goal.
"""
from __future__ import absolute_import

# ROS
import rospy
import smach

from ed.entity import Entity
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.human_interaction import Say
from robot_smach_states.designator_iterator import IterateDesignator
import robot_smach_states.util.designators as ds
from robot_smach_states.perception import LookAtEntity
from smach import cb_interface, CBState
from robocup_knowledge import load_knowledge
from robot_smach_states.human_interaction.find_people_in_room import FindPeople

challenge_knowledge = load_knowledge('challenge_stickler_for_the_rules')


class CheckPeopleInForbiddenRoom(smach.StateMachine):
    def __init__(self, robot_name, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])

        robot = robot_name
        room = room_des
        found_people = ds.VariableDesignator(resolve_type=[Entity], name='found_people')
        violating_person = ds.VariableDesignator(resolve_type=Entity, name='violating_person')
        forbidden_room_waypoint = ds.EntityByIdDesignator(robot,
                                                          uuid=challenge_knowledge.forbidden_room_waypoint)

        with self:
            @cb_interface(outcomes=["yes", "no"])
            def check_forbidden_room(userdata=None):
                return "yes" if room_des.uuid == challenge_knowledge.forbidden_room else "no"

            smach.StateMachine.add("CHECK_IN_FORBIDDEN_ROOM", CBState(check_forbidden_room),
                                   transitions={"yes": "NAVIGATE_TO_CHECK",
                                                "no": "done"})
            smach.StateMachine.add(
                "NAVIGATE_TO_CHECK",
                NavigateToWaypoint(robot, forbidden_room_waypoint),
                transitions={"arrived": "FIND_PEOPLE", "unreachable": "done",
                             "goal_not_defined": "done"},
            )
            smach.StateMachine.add(
                "FIND_PEOPLE",
                FindPeople(robot=robot,
                           query_entity_designator=room,
                           found_people_designator=found_people.writeable,
                           speak=True),
                transitions={"found": "ITERATE_PEOPLE", "failed": "done"})

            smach.StateMachine.add('ITERATE_PEOPLE',
                                   IterateDesignator(found_people,
                                                     violating_person.writeable),
                                   transitions={'next': 'LOOKAT_PERSON',
                                                'stop_iteration': 'done'})

            smach.StateMachine.add('LOOKAT_PERSON',
                                   LookAtEntity(robot, violating_person),
                                   transitions={'arrived': 'SAY_BEHAVE',
                                                'unreachable': 'SAY_BEHAVE',
                                                'goal_not_defined': 'SAY_BEHAVE'})
            smach.StateMachine.add("SAY_BEHAVE",
                                   Say(robot,
                                       "Unfortunately, the party is not in this room,"
                                       "please leave the room.", block=True),
                                   transitions={"spoken": "NAVIGATE_TO_CHECK"})


class Patrol(smach.StateMachine):
    def __init__(self, robot, room_des, speak=True):
        """
        Base Smach state to patrol to a designated position. monitoring a set of functions.

        :param robot: (Robot) robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        self.robot = robot
        self.room_des = room_des
        self.speak = speak

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_ROOM",
                NavigateToSymbolic(robot, {room_des: "in"}, room_des),
                transitions={"arrived": "CHECK_FORBIDDEN_ROOM", "unreachable": "CHECK_FORBIDDEN_ROOM",
                             "goal_not_defined": "CHECK_FORBIDDEN_ROOM"},
            )

            smach.StateMachine.add("CHECK_FORBIDDEN_ROOM",
                                   CheckPeopleInForbiddenRoom(robot_name=self.robot, room_des=self.room_des),
                                   transitions={"done": "done"})

