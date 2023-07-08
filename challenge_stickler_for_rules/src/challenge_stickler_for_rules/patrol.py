"""
Module contains states to patrol the environment while navigating to a goal.
"""
from __future__ import absolute_import

# ROS
import rospy
import smach

from ed.entity import Entity
from robot_skills.simulation import is_sim_mode
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic, GuideToSymbolic
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.human_interaction import Say, SetPoseFirstFoundPersonToEntity, GiveDirections, AskYesNo, AskYesNoPicoVoice
import robot_smach_states.util.designators as ds
from robot_smach_states.utility import WaitTime
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
                NavigateToWaypoint(robot,
                                   forbidden_room_waypoint,
                                   speak=False),
                transitions={"arrived": "FIND_PEOPLE",
                             "unreachable": "FIND_PEOPLE",
                             "goal_not_defined": "FIND_PEOPLE"}
            )

            smach.StateMachine.add(
                "FIND_PEOPLE",
                FindFirstPerson(robot=robot,
                                query_entity_designator=room,
                                found_person_designator=violating_person.writeable,
                                speak=True,
                                search_timeout=30),
                transitions={"found": "GOTO_PERSON", "failed": "done"}
            )

            smach.StateMachine.add(
                "GOTO_PERSON",
                NavigateToObserve(robot,
                                  violating_person,
                                  room=room,
                                  radius=1.0,
                                  margin=0.2),
                transitions={"arrived": "SAY_BEHAVE",
                             "unreachable": "SAY_BEHAVE",
                             "goal_not_defined": "SAY_BEHAVE"}
            )

            smach.StateMachine.add(
                "SAY_BEHAVE",
                Say(robot,
                    "You are breaking the forbidden room house rule. I'll leave you 10 seconds to leave this room",
                    block=True),
                transitions={"spoken": "WAIT"},
            )

            smach.StateMachine.add(
                "WAIT",
                WaitTime(robot,
                         waittime=10),  # Wait 10s
                transitions={"waited": "NAVIGATE_TO_CHECK_VERIFICATION",
                             "preempted": "NAVIGATE_TO_CHECK_VERIFICATION"},
            )

            smach.StateMachine.add(
                "NAVIGATE_TO_CHECK_VERIFICATION",
                NavigateToWaypoint(robot,
                                   forbidden_room_waypoint,
                                   speak=False),
                transitions={"arrived": "VERIFY_PEOPLE",
                             "unreachable": "VERIFY_PEOPLE",
                             "goal_not_defined": "VERIFY_PEOPLE"},
            )
            smach.StateMachine.add(
                "VERIFY_PEOPLE",
                FindFirstPerson(robot=robot,
                                query_entity_designator=room,
                                found_person_designator=violating_person.writeable,
                                speak=True),
                transitions={"found": "GOTO_PERSON",
                             "failed": "SAY_VERIFY_DONE"}
            )

            smach.StateMachine.add(
                "SAY_VERIFY_DONE",
                Say(robot,
                    "There's no one left anymore in the forbidden room.",
                    block=False),
                transitions={"spoken": "done"},
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
        drinks_entity_id = challenge_knowledge.drinks_entity_id
        found_person = ds.VariableDesignator(resolve_type=Entity, name='found_people')
        drinks_loc_des = ds.EntityByIdDesignator(robot, uuid=drinks_entity_id)

        with self:
            smach.StateMachine.add("FIND_PERSON_WITHOUT_DRINK",
                                   SetPoseFirstFoundPersonToEntity(robot=robot,
                                                                   properties={'tags': ['LNotHolding', 'RNotHolding']},
                                                                   strict=True,
                                                                   dst_entity_designator=found_person.writeable,
                                                                   query_entity_designator=room,
                                                                   search_timeout=30),
                                   transitions={"done": "SAY_I_HAVE_SEEN",
                                                "failed": "SAY_PEOPLE_WITHOUT_DRINKS_FAILED"})
            # Detect fallback - detect waving people
            smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS_FAILED",
                                   Say(robot=robot,
                                       sentence="Could not detect people without a drink",
                                       look_at_standing_person=True,
                                       block=True),
                                   transitions={"spoken": "ASK_FOR_WAVING"})
            smach.StateMachine.add("ASK_FOR_WAVING",
                                   Say(robot=robot,
                                       sentence="Hi Guests, It is mandatory to have a drink. "
                                                "Please raise your arm completely and wave, "
                                                "if you do not have a drink yet.",
                                       look_at_standing_person=True,
                                       block=True),
                                   transitions={"spoken": "DETECT_WAVING_PERSON"})
            smach.StateMachine.add("DETECT_WAVING_PERSON",
                                   SetPoseFirstFoundPersonToEntity(robot=robot,
                                                                   properties={'tags': ['LWave', 'RWave']},
                                                                   strict=False,
                                                                   dst_entity_designator=found_person.writeable,
                                                                   query_entity_designator=room,
                                                                   search_timeout=15),
                                   transitions={"done": "SAY_I_HAVE_SEEN", "failed": "SAY_WAVING_FAILED"})
            smach.StateMachine.add("SAY_I_HAVE_SEEN",
                                   Say(robot=robot,
                                       sentence="Found person who does not have a drink. I will be there shortly!",
                                       look_at_standing_person=True,
                                       block=True),
                                   transitions={"spoken": "NAVIGATE_TO_PERSON"})
            smach.StateMachine.add("SAY_WAVING_FAILED",
                                   Say(robot=robot,
                                       sentence="Could not detect a waving person, I will continue my task",
                                       look_at_standing_person=False,
                                       block=False),
                                   transitions={"spoken": "done"})
            smach.StateMachine.add("NAVIGATE_TO_PERSON",
                                   NavigateToObserve(robot=robot, entity_designator=found_person,
                                                     radius=1),
                                   transitions={"arrived": "SAY_BREAKING_RULE",
                                                "unreachable": "SAY_NOT_REACHABLE",
                                                "goal_not_defined": "SAY_NOT_REACHABLE"})
            smach.StateMachine.add("SAY_NOT_REACHABLE",
                                   Say(robot=robot,
                                       sentence="I cannot reach the guest, I will continue my task",
                                       look_at_standing_person=False,
                                       block=False),
                                   transitions={"spoken": "done"})
            smach.StateMachine.add("SAY_BREAKING_RULE",
                                   Say(robot, "You are breaking the 'mandatory hydration' rule", look_at_standing_person=True, block=True),
                                   transitions={"spoken": "SAY_GET_DRINK"})
            smach.StateMachine.add("SAY_GET_DRINK",
                                   Say(robot, f"You need to get a drink from the {drinks_entity_id}, I will tell you how to get there", look_at_standing_person=False,
                                       block=True),
                                   transitions={"spoken": "GIVE_DIRECTIONS"})
            smach.StateMachine.add("GIVE_DIRECTIONS",
                                   GiveDirections(robot, drinks_loc_des),
                                   transitions={"succeeded": "NEED_GUIDANCE",
                                                "failed": "NEED_GUIDANCE"})
            smach.StateMachine.add("NEED_GUIDANCE",
                                   Say(robot, "Do you need guidance to get there? Yes or No",
                                       look_at_standing_person=True, block=True),
                                   transitions={"spoken": "done"})
            if is_sim_mode():
                smach.StateMachine.add("ASK_YES_NO",
                                       AskYesNo(robot),
                                       transitions={"yes": "GUIDE_TO_DRINKS",
                                                    "no": "done",
                                                    "no_result": "done"})
            else:
                smach.StateMachine.add("ASK_YES_NO",
                                       AskYesNoPicoVoice(robot),
                                       transitions={"yes": "GUIDE_TO_DRINKS",
                                                    "no": "done",
                                                    "no_result": "done"})

            smach.StateMachine.add("GUIDE_TO_DRINKS",
                                   GuideToSymbolic(robot, {drinks_loc_des: "in_front_of"}, drinks_loc_des),
                                   transitions={"arrived": "SAY_TAKE_DRINK",
                                                "unreachable": "SAY_NAV_FAILED",
                                                "goal_not_defined": "SAY_NAV_FAILED",
                                                "lost_operator": "SAY_LOST_OPERATOR",
                                                "preempted": "done"})

            smach.StateMachine.add("SAY_TAKE_DRINK",
                                   Say(robot, f"Dear guest, Take a drink from the {drinks_entity_id}", look_at_standing_person=True, block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("SAY_NAV_FAILED",
                                   Say(robot, "I cannot reach the drinks, I will continue my task", look_at_standing_person=False, block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("SAY_LOST_OPERATOR",
                                   Say(robot, "I lost the guest, I will continue my task", look_at_standing_person=False, block=True),
                                   transitions={"spoken": "done"})


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
