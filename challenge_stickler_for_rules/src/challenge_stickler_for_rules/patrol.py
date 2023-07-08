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
from robot_smach_states.human_interaction import Say, SetPoseFirstFoundPersonToEntity
import robot_smach_states.util.designators as ds
from robot_smach_states.perception import RotateToEntity
from smach import cb_interface, CBState
from robocup_knowledge import load_knowledge
from robot_smach_states.human_interaction.find_people_in_room import FindFirstPerson

challenge_knowledge = load_knowledge("challenge_stickler_for_the_rules")


class CheckPeopleInForbiddenRoom(smach.StateMachine):
    def __init__(self, robot_name, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])

        robot = robot_name
        room = room_des
        violating_person = ds.VariableDesignator(resolve_type=Entity, name="violating_person")
        forbidden_room_waypoint = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.forbidden_room_waypoint)

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_CHECK",
                NavigateToWaypoint(robot, forbidden_room_waypoint, speak=False),
                transitions={"arrived": "FIND_PEOPLE", "unreachable": "FIND_PEOPLE", "goal_not_defined": "FIND_PEOPLE"},
            )
            smach.StateMachine.add(
                "FIND_PEOPLE",
                FindFirstPerson(
                    robot=robot,
                    query_entity_designator=room,
                    found_person_designator=violating_person.writeable,
                    speak=True),
                transitions={"succeeded": "LOOKAT_PERSON", "failed": "LOOKAT_PERSON"}

            )
            smach.StateMachine.add(
                "LOOKAT_PERSON",
                RotateToEntity(robot, violating_person),
                transitions={"succeeded": "SAY_BEHAVE", "failed": "SAY_BEHAVE"}
            )

            smach.StateMachine.add(
                "SAY_BEHAVE",
                Say(robot, "Unfortunately, the party is not in this room, please leave the room.", block=True),
                transitions={"spoken": "NAVIGATE_TO_CHECK"},
            )


class CheckForDrinks(smach.StateMachine):
    """
    Base Smach state to check for people with drinks.
    :param robot: (Robot) robot api object
    """

    def __init__(self, robot_name, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])
        robot = robot_name
        room = room_des
        found_people = ds.VariableDesignator(resolve_type=Entity, name='found_people')
        caller_designator = ds.EdEntityDesignator(robot=robot, uuid=ds.value_or_resolve(found_people),
                                                  name="caller_des",
                                                  )
        smach.StateMachine.add("FIND_PERSON_WITHOUT_DRINK",
                               SetPoseFirstFoundPersonToEntity(robot=robot,
                                                               properties={'tags': ['LNotHolding', 'RNotHolding']},
                                                               strict=True,
                                                               dst_entity_designator=found_people,
                                                               query_entity_designator=room),
                               transitions={"done": "SAY_I_HAVE_SEEN",
                                            "failed": "SAY_PEOPLE_WITHOUT_DRINKS_FAILED"})
        # Detect fallback - detect waving people
        smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS_FAILED",
                               Say(robot=robot,
                                   sentence="Could not detect people without drinks",
                                   look_at_standing_person=True,
                                   block=True),
                               transitions={"spoken": "ASK_FOR_WAVING"})
        smach.StateMachine.add("ASK_FOR_WAVING",
                               Say(robot=robot,
                                   sentence="Please raise your arm completely and wave, if you want me to bring you something",
                                   look_at_standing_person=True,
                                   block=True),
                               transitions={"spoken": "done"}) #ToDO: Yet to implement Fallback of finding waving person
        smach.StateMachine.add("SAY_I_HAVE_SEEN",
                               Say(robot=robot,
                                   sentence="Found person who might want to place an order. I will be there shortly!",
                                   look_at_standing_person=True,
                                   block=True),
                               transitions={"spoken": "NAVIGATE_TO_PERSON"})
        smach.StateMachine.add("NAVIGATE_TO_PERSON",
                               NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                 radius=1),
                               transitions={"arrived": "done",
                                            "unreachable": "done",
                                            "goal_not_defined": "SAY_PEOPLE_WITHOUT_DRINKS"})


class Patrol(smach.StateMachine):
    def __init__(self, robot, room_des):
        """
        Base Smach state to patrol to a designated position. mCHECK_IN_FORBIDDEN_ROOMonitoring a set of functions.

        :param robot: (Robot) robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        self.robot = robot
        self.room_des = room_des

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_ROOM",
                NavigateToSymbolic(robot, {room_des: "in"}, room_des),
                transitions={"arrived": "CHECK_IN_FORBIDDEN_ROOM",
                             "unreachable": "CHECK_IN_FORBIDDEN_ROOM",
                             "goal_not_defined": "CHECK_IN_FORBIDDEN_ROOM"},
            )

            @cb_interface(outcomes=["yes", "no"])
            def check_forbidden_room(userdata=None):
                return "yes" if room_des.uuid == challenge_knowledge.forbidden_room else "no"

            smach.StateMachine.add("CHECK_IN_FORBIDDEN_ROOM", CBState(check_forbidden_room),
                                   transitions={"yes": "CHECK_PEOPLE_IN_FORBIDDEN_ROOM",
                                                "no": "SAY_PEOPLE_WITHOUT_DRINKS"})

            smach.StateMachine.add("CHECK_PEOPLE_IN_FORBIDDEN_ROOM",
                                   CheckPeopleInForbiddenRoom(robot_name=self.robot, room_des=self.room_des),
                                   transitions={"done": "done"})

            smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS",
                                   Say(robot, "I'm Trying to find people without a drink",
                                       block=True),
                                   transitions={"spoken": "CHECK_FOR_DRINKS"})
            smach.StateMachine.add("CHECK_FOR_DRINKS",
                                   CheckForDrinks(robot_name=self.robot,room_des=self.room_des),
                                   transitions={"done": "done"})
