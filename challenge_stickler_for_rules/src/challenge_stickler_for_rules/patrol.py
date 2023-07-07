"""
Module contains states to patrol the environment while navigating to a goal.
"""
from __future__ import absolute_import

# ROS
import rospy
import smach

from robot_skills.robot import Robot
from ed.entity import Entity
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.human_interaction import Say
import robot_smach_states.util.designators as ds
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_stickler_for_the_rules')

from robot_smach_states.human_interaction.find_people_in_room import FindPeople

# Robot skills
from robot_smach_states.util.designators import EdEntityDesignator


class CheckPeopleInForbiddenRoom(smach.StateMachine):
    def __init__(self, robot_name, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])

        self.robot = robot_name
        self.room = room_des
        self.violating_person = ds.VariableDesignator(resolve_type=[Entity], name='violating_person')
        self.forbidden_room_waypoint = ds.EntityByIdDesignator(self.robot,
                                                               uuid=challenge_knowledge.forbidden_room_waypoint)

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_CHECK",
                NavigateToWaypoint(self.robot, self.forbidden_room_waypoint),
                transitions={"arrived": "FIND_PEOPLE", "unreachable": "done",
                             "goal_not_defined": "done"},
            )
            smach.StateMachine.add(
                "FIND_PEOPLE",
                FindPeople(robot=self.robot,
                           query_entity_designator=self.room,
                           found_people_designator=self.violating_person.writeable,
                           speak=True),
                transitions={"found": "NAVIGATE_TO_CHECK", "failed": "done"})

            smach.StateMachine.add('GOTO_PERSON',
                                   NavigateToObserve(self.robot, self.violating_person, radius=1.0,
                                                     margin=1.0,  # Makes the robot go within 1m of current_old_guest
                                                     speak=False),
                                   transitions={'arrived': 'SAY_BEHAVE',
                                                'unreachable': 'SAY_BEHAVE',
                                                'goal_not_defined': 'SAY_BEHAVE'})
            smach.StateMachine.add("SAY_BEHAVE",
                                   Say(self.robot,
                                       "Unfortunately, the party is not in this room,"
                                       "please leave the room.", block=False,),
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

