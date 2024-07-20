"""
Module contains states to patrol the environment while navigating to a goal.
"""
from __future__ import absolute_import

from math import pi
from typing import List, Optional, Dict
import os.path

import numpy as np
import rospy
import rospkg


# ROS
import smach

from ed.entity import Entity
from ed.volume import Volume
from robot_skills.classification_result import ClassificationResult
from robot_skills.simulation import is_sim_mode
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic, GuideToSymbolic
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.human_interaction import Say, GiveDirections, AskYesNo, AskYesNoPicoVoice, ShowImageArray,ShowImage
from robot_smach_states.utility import CheckTries, WriteDesignator
from robot_smach_states.world_model.world_model import Inspect
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
                                   look_at_designator=room_des,
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
                                look_range=(-pi*0.35, pi*0.35),
                                look_steps=5,
                                search_timeout=25),
                transitions={"found": "GOTO_PERSON", "failed": "done"}
            )

            smach.StateMachine.add(
                "GOTO_PERSON",
                NavigateToObserve(robot,
                                  violating_person,
                                  room=room,
                                  radius=0.8,
                                  margin=0.2),
                transitions={"arrived": "SAY_BEHAVE",
                             "unreachable": "SAY_BEHAVE",
                             "goal_not_defined": "SAY_BEHAVE"}
            )

            smach.StateMachine.add(
                "SAY_BEHAVE",
                Say(robot,
                    "You are breaking the forbidden room house rule. I'll leave you 10 seconds to leave this room",
                    block=False,
                    look_at_standing_person=True),
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
                                   look_at_designator=room_des,
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
                                speak=True,
                                look_range=(-pi * 0.35, pi * 0.35),
                                look_steps=5,
                                search_timeout=15),
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

    def __init__(self, robot, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])
        drinks_entity_id = challenge_knowledge.drinks_entity_id
        found_person = ds.VariableDesignator(resolve_type=Entity, name='found_people')
        drinks_loc_des = ds.EntityByIdDesignator(robot, uuid=drinks_entity_id)

        with self:
            smach.StateMachine.add("SAY_HOLD_DRINK",
                                   Say(robot, "Please make sure that you hold your drink in front of you chest."
                                              "Like shown on my screen",
                                       block=True),
                                   transitions={"spoken": "SHOW_IMAGE"})

            smach.StateMachine.add('SHOW_IMAGE', ShowImage(robot, os.path.join(
                                           rospkg.RosPack().get_path('challenge_stickler_for_rules'),
                                           "images",
                                           "hold_drink.jpeg"
                                       ),
                                       duration=25),
                                   transitions={'succeeded': 'FIND_PERSON_WITHOUT_DRINK',
                                                'failed': 'FIND_PERSON_WITHOUT_DRINK'})

            smach.StateMachine.add("FIND_PERSON_WITHOUT_DRINK",
                                   FindFirstPerson(robot=robot,
                                                   properties={'tags': ['LNotHolding', 'RNotHolding']},
                                                   strict=True,
                                                   reverse=False,
                                                   found_person_designator=found_person.writeable,
                                                   query_entity_designator=room_des,
                                                   search_timeout=25,
                                                   look_range=(-pi*0.5, pi*0.5),
                                                   look_steps=5),
                                   transitions={"found": "SAY_I_HAVE_SEEN",
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
                                   FindFirstPerson(robot=robot,
                                                   properties={'tags': ['LWave', 'RWave']},
                                                   strict=False,
                                                   found_person_designator=found_person.writeable,
                                                   query_entity_designator=room_des,
                                                   search_timeout=15,
                                                   look_range=(-pi * 0.5, pi * 0.5),
                                                   look_steps=5
                                                   ),
                                   transitions={"found": "SAY_I_HAVE_SEEN", "failed": "SAY_WAVING_FAILED"})
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
                                                     radius=0.8,
                                                     margin=0.2),
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
                                   transitions={"spoken": "ASK_YES_NO"})
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
                                   Say(robot, f"I cannot reach the drinks location, please look for the {drinks_entity_id} yourself, I will continue my task", look_at_standing_person=False, block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("SAY_LOST_OPERATOR",
                                   Say(robot, "I lost the guest, I will continue my task", look_at_standing_person=False, block=True),
                                   transitions={"spoken": "done"})


class CheckForLitter(smach.StateMachine):
    def __init__(self, robot, room_des):
        smach.StateMachine.__init__(self, outcomes=["done"])

        detected_litter: ds.Designator[List[ClassificationResult]] = ds.VariableDesignator([], resolve_type=[ClassificationResult], name="detected_litter")
        litter_item: ds.Designator[Entity] = ds.VariableDesignator(resolve_type=Entity, name="litter_item")

        def volume_filter(volumes: Dict[str, Volume]) -> List[str]:
            return [k for k in volumes if k.startswith("on_top_of") and not k.endswith("_nav")]

        def nav_volume(vol_des: ds.Designator[str]) -> str:
            vol = ds.value_or_resolve(vol_des)
            return vol + "_nav"

        on_top_of_volumes = ds.FuncDesignator(ds.AttrDesignator(room_des, "volumes", resolve_type=dict), volume_filter, resolve_type=[str])

        area_des = ds.VariableDesignator(resolve_type=str, name="area_des")
        nav_area_des = ds.FuncDesignator(area_des, nav_volume, resolve_type=str)
        image_des = ds.VariableDesignator(resolve_type=np.ndarray, name="image_des")

        found_person = ds.VariableDesignator(resolve_type=Entity, name="found_person")

        with self:
            smach.StateMachine.add("ITERATE_VOLUMES",
                                   IterateDesignator(on_top_of_volumes, area_des.writeable),
                                   transitions={"next": "FIND_LITTER",
                                                "stop_iteration": "SAY_NO_LITTER"}
                                   )

            smach.StateMachine.add(
                "FIND_LITTER",
                Inspect(robot, room_des, detected_litter.writeable, searchArea=area_des, navigation_area=nav_area_des, fit_supporting_entity=False, room=room_des),
                transitions={"done": "FILTER_LITTER",
                             "failed": "ITERATE_VOLUMES"}
            )

            @smach.cb_interface(outcomes=["done", "failed"])
            def filter_litter(userdata=None):
                detected_litter_items = detected_litter.resolve()
                item: ClassificationResult
                for item in detected_litter_items:
                    if hasattr(item, "uuid") and item.uuid:
                        e: Optional[Entity] = robot.ed.get_entity(item.uuid)
                        if e is None:
                            rospy.logerr(f"Could not find entity with uuid {item.uuid}")
                            continue

                        x_size = e.shape.x_max - e.shape.x_min
                        y_size = e.shape.y_max - e.shape.y_min
                        z_size = e.shape.z_max - e.shape.z_min

                        max_dim = max(x_size, y_size, z_size)
                        if max_dim > 0.17:
                            rospy.loginfo(f"Dropping litter item {item.uuid} because it is too big ({x_size}, {y_size}, {z_size})")
                            continue

                        rospy.loginfo(f"Found litter: {item.uuid}, {item.etype} ({x_size}, {y_size}, {z_size})")

                        litter_item.writeable.write(e)
                        image_des.writeable.write(item.image)
                        return "done"

                return "failed"

            smach.StateMachine.add("FILTER_LITTER",
                                   CBState(filter_litter),
                                   transitions={"done": "SHOW_LITTER_IMAGE",
                                                "failed": "ITERATE_VOLUMES"})


            smach.StateMachine.add("SHOW_LITTER_IMAGE",
                                   ShowImageArray(robot, image_des, duration=10),
                                   transitions={"succeeded": "SAY_LITTER_FOUND",
                                                "failed": "SAY_LITTER_FOUND"})

            smach.StateMachine.add("SAY_LITTER_FOUND",
                                   Say(robot, "I have found litter on the floor. Let's find someone to pick it up", block=True),
                                   transitions={"spoken": "FIND_PERSON_TO_CLEAN"})

            smach.StateMachine.add("SAY_NO_LITTER",
                                   Say(robot, "I have not found any litter on the floor", block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("FIND_PERSON_TO_CLEAN",
                                   FindFirstPerson(robot=robot,
                                                   strict=False,
                                                   found_person_designator=found_person.writeable,
                                                   query_entity_designator=room_des,
                                                   search_timeout=15,
                                                   look_range=(-pi * 0.5, pi * 0.5),
                                                   look_steps=5
                                                   ),
                                   transitions={"found": "NAVIGATE_TO_PERSON",
                                                "failed": "SAY_NO_PERSON"})

            smach.StateMachine.add("SAY_NO_PERSON",
                                   Say(robot, "I could not find anyone to pick up the litter. I will continue my task", block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("NAVIGATE_TO_PERSON",
                                   NavigateToObserve(robot=robot, entity_designator=found_person,
                                                     radius=0.8,
                                                     margin=0.2),
                                   transitions={"arrived": "SHOW_LITTER_IMAGE2",
                                                "unreachable": "SAY_NOT_REACHABLE",
                                                "goal_not_defined": "SAY_NOT_REACHABLE"})

            smach.StateMachine.add("SAY_NOT_REACHABLE",
                                   Say(robot, "I cannot reach the guest, I will continue my task", look_at_standing_person=False, block=False),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("SHOW_LITTER_IMAGE2",
                                   ShowImageArray(robot, image_des, duration=10),
                                   transitions={"succeeded": "SAY_CLEAN_LITTER",
                                                "failed": "SAY_CLEAN_LITTER"})

            smach.StateMachine.add("SAY_CLEAN_LITTER",
                                   Say(robot,
                                       "Please pick up the litter from the floor in the {room}, the item is shown on my screen",
                                       room=ds.AttrDesignator(room_des, "uuid", resolve_type=str),
                                       block=True),
                                   transitions={"spoken": "FIND_LITTER2"})

            smach.StateMachine.add(
                "FIND_LITTER2",
                Inspect(robot, room_des, detected_litter.writeable, searchArea=area_des, navigation_area=nav_area_des, fit_supporting_entity=False, room=room_des),
                transitions={"done": "FILTER_LITTER2",
                             "failed": "done"}
            )

            smach.StateMachine.add("FILTER_LITTER2",
                                   CBState(filter_litter),
                                   transitions={"done": "SHOW_LITTER_IMAGE3",
                                                "failed": "LITTER_GONE"})

            smach.StateMachine.add("LITTER_GONE",
                                   Say(robot, "The litter has been cleaned up. I will continue my task", block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("SHOW_LITTER_IMAGE3",
                                   ShowImageArray(robot, image_des, duration=10),
                                   transitions={"succeeded": "SAY_LITTER_FOUND2",
                                                "failed": "SAY_LITTER_FOUND2"})

            smach.StateMachine.add("SAY_LITTER_FOUND2",
                                      Say(robot, "The litter is still there, asking again", block=True),
                                      transitions={"spoken": "NAVIGATE_TO_PERSON2"})

            smach.StateMachine.add("NAVIGATE_TO_PERSON2",
                                   NavigateToObserve(robot=robot, entity_designator=found_person,
                                                     radius=0.8,
                                                     margin=0.2),
                                   transitions={"arrived": "SHOW_LITTER_IMAGE4",
                                                "unreachable": "SAY_NOT_REACHABLE",
                                                "goal_not_defined": "SAY_NOT_REACHABLE"})

            smach.StateMachine.add("SHOW_LITTER_IMAGE4",
                                   ShowImageArray(robot, image_des, duration=10),
                                   transitions={"succeeded": "SAY_LITTER_FOUND3",
                                                "failed": "SAY_LITTER_FOUND3"})

            smach.StateMachine.add("SAY_LITTER_FOUND3",
                                   Say(robot, "The litter is still there, please clean it up this time. I will continue my task", block=True),
                                   transitions={"spoken": "done"})


class Patrol(smach.StateMachine):
    def __init__(self, robot, room_des):
        """
        Base Smach state to patrol to a designated position. mCHECK_IN_FORBIDDEN_ROOMonitoring a set of functions.

        :param robot: (Robot) robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["done", "failed"])

        reset_des = ds.VariableDesignator(False, name="reset_des").writeable

        with self:
            smach.StateMachine.add("RESET_TRIES",
                                   WriteDesignator(reset_des, True),
                                   transitions={"written": "CHECK_TRIES"}
                                   )

            smach.StateMachine.add("CHECK_TRIES",
                                   CheckTries(max_tries=3, reset_des=reset_des),
                                   transitions={"not_yet": "NAVIGATE_TO_ROOM",
                                                "max_tries": "done"}
                                   )

            smach.StateMachine.add(
                "NAVIGATE_TO_ROOM",
                NavigateToSymbolic(robot, {room_des: "in"}, room_des),
                transitions={"arrived": "CHECK_IN_FORBIDDEN_ROOM",
                             "unreachable": "CHECK_TRIES",
                             "goal_not_defined": "failed"},
            )

            @cb_interface(outcomes=["yes", "no"])
            def check_forbidden_room(userdata=None):
                return "yes" if ds.value_or_resolve(room_des.uuid) == challenge_knowledge.forbidden_room else "no"

            smach.StateMachine.add("CHECK_IN_FORBIDDEN_ROOM", CBState(check_forbidden_room),
                                   transitions={"yes": "CHECK_PEOPLE_IN_FORBIDDEN_ROOM",
                                                "no": "CHECK_FOR_LITTER"})

            smach.StateMachine.add("CHECK_FOR_LITTER",
                                   CheckForLitter(robot, room_des),
                                   transitions={"done": "SAY_PEOPLE_WITHOUT_DRINKS"})

            smach.StateMachine.add("CHECK_PEOPLE_IN_FORBIDDEN_ROOM",
                                   CheckPeopleInForbiddenRoom(robot_name=robot, room_des=room_des),
                                   transitions={"done": "done"})

            smach.StateMachine.add("SAY_PEOPLE_WITHOUT_DRINKS",
                                   Say(robot, "I'm Trying to find people without a drink",
                                       block=True),
                                   transitions={"spoken": "CHECK_FOR_DRINKS"})

            smach.StateMachine.add("CHECK_FOR_DRINKS",
                                   CheckForDrinks(robot, room_des),
                                   transitions={"done": "done"})
