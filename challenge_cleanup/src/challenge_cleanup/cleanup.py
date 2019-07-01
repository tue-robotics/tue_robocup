#!/usr/bin/python

import rospy
import smach
import random

import hmi
import robot_smach_states
import robot_smach_states.util.designators as ds
from clean_inspect import CleanInspect

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')


class VerifyWorldModelInfo(smach.State):
    """
    Check consistency between world model and local knowledge
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["failed", "done"])
        self._robot = robot

    def execute(self, userdata=None):
        # Look for trash units; can be in living_room and kitchen.
        # THIS IS ARENA DEPENDANT!! (Should be handled in a different way?)
        # There should be an 'underscore_rule' : trash_bin or trashbin???
        # (Different between rgo2019 and robotics_testlab knowledge)

        ids = [e.id for e in self._robot.ed.get_entities()]
        for loc in challenge_knowledge.cleaning_locations:
            # if loc["room"] == "living_room":
            #     if "trash_bin" not in ids:
            #         return "failed"
            if loc["room"] == "kitchen":
                if "trashbin" not in ids:
                    return "failed"

    # Make sure the world model and local knowledge match
        for place in challenge_knowledge.cleaning_locations:
            if place["name"] not in ids:
                return "failed"
            if place["room"] not in ids:
                return "failed"

        return "done"


class RoomToCleanUpLocations(smach.State):
    def __init__(self, knowledge, room_des, cleanup_locations):
        """
        Determine cleaning locations on runtime
        :param knowledge: challenge knowledge
        :param room_des: Designator resolving to room
        :type room_des: Designator(str)
        :param cleanup_locations: Writable designator, which will be filled with a list of cleaning locations
        """
        smach.State.__init__(self, outcomes=["done"])
        ds.check_type(room_des, str)
        ds.is_writeable(cleanup_locations)
        assert(hasattr(knowledge, "cleaning_locations"))

        self._knowledge = knowledge
        self._room_des = room_des
        self._cleanup_locations = cleanup_locations

    def execute(self, userdata=None):
        cleaning_locations = []
        for loc in self._knowledge.cleaning_locations:
            if loc["room"] == self._room_des.resolve():
                cleaning_locations.append(loc)
        self._cleanup_locations.write(cleaning_locations)
        # Show the cleanup list on screen
        rospy.loginfo("Cleaning locations: {}".format(self._cleanup_locations.resolve()))

        return "done"


class AskWhichRoomToClean(smach.StateMachine):
    # Logic in this code is still flawed. No correct repetition.....
    # EXAMINE challenge_restaurant->take_orders.py for information about structure if interaction
    def __init__(self, robot, room_grammar, roomw, cleanup_locationsw):
        smach.StateMachine.__init__(self, outcomes=["done"])

        hmi_result_des = ds.VariableDesignator(resolve_type=hmi.HMIResult, name="hmi_result_des")
        room_name_des = ds.AttrDesignator(hmi_result_des, "sentence", resolve_type=str)

        @smach.cb_interface(outcomes=['done'])
        def write_room(ud, des_read, des_write):
            # type: (object, ds.Designator, ds.Designator) -> str
            assert(ds.is_writeable(des_write))
            assert(des_write.resolve_type == des_read.resolve_type)
            des_write.write(des_read.resolve())
            return 'done'

        with self:
            smach.StateMachine.add("ASK_WHICH_ROOM", robot_smach_states.Say(robot, "Which room should I clean for you?",
                                                                            block=True),
                                   transitions={"spoken": "HEAR_ROOM"})
            smach.StateMachine.add("HEAR_ROOM", robot_smach_states.HearOptionsExtra(robot, room_grammar,
                                                                                    ds.writeable(hmi_result_des)),
                                   transitions={"heard": "SAY_HEARD_CORRECT",
                                                "no_result": "ASK_WHICH_ROOM"})
            smach.StateMachine.add("SAY_HEARD_CORRECT", robot_smach_states.SayFormatted(
                robot, "I understood that the {room} should be cleaned, is this correct?", room=room_name_des,
                block=True),
                                   transitions={"spoken": "HEAR_CORRECT"})
            smach.StateMachine.add("HEAR_CORRECT", robot_smach_states.AskYesNo(robot),
                                   transitions={"yes": "FILL_LOCATIONS",
                                                "no": "ASK_WHICH_ROOM",
                                                "no_result": "ASK_WHICH_ROOM"})
            smach.StateMachine.add("FILL_LOCATIONS", RoomToCleanUpLocations(challenge_knowledge, roomw,
                                                                            cleanup_locationsw),
                                   transitions={"done": "WRITE_ROOM"})
            smach.StateMachine.add('WRITE_ROOM', smach.CBState(write_room, cb_args=[room_name_des, roomw]),
                                   transitions={'done': 'done'})


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    # The probability number must be determined experimentally
    # Designators
    # Room to search through
    roomr = ds.VariableDesignator(resolve_type=str)
    roomw = roomr.writeable

    # Cleanup location as defined in local knowledge
    cleanup_locationsr = ds.VariableDesignator([{'1': '2', '3': '4'}])
    cleanup_locationsw = cleanup_locationsr.writeable
    location_des = ds.VariableDesignator(resolve_type=dict)

    with sm:
        smach.StateMachine.add("INITIALIZE",
                               robot_smach_states.Initialize(robot),
                               transitions={"initialized": "SET_INITIAL_POSE",
                                            "abort": "Aborted"})
        smach.StateMachine.add("SET_INITIAL_POSE",
                               robot_smach_states.SetInitialPose(robot,challenge_knowledge.starting_point),
                               transitions={"done":    "INQUIRE_ROOM",
                                            "preempted": "Aborted",
                                            "error":  "INQUIRE_ROOM"})

        smach.StateMachine.add("INQUIRE_ROOM",
                                AskWhichRoomToClean(robot, ds.Designator(challenge_knowledge.grammar), roomw,
                                                    cleanup_locationsw),
                                transitions={"done":    "VERIFY"})

        smach.StateMachine.add('VERIFY', VerifyWorldModelInfo(robot),
                               transitions={"done": "SAY_START_CHALLENGE",
                                                                                  "failed": "SAY_KNOWLEDGE_NOT_COMPLETE"})

        smach.StateMachine.add('SAY_KNOWLEDGE_NOT_COMPLETE', robot_smach_states.Say(robot,
                                                                ["My knowledge of the world is not complete!",
                                                                "Please give me some more information!"], block=False),
                               transitions={"spoken": "Aborted"})

        smach.StateMachine.add('SAY_START_CHALLENGE',
                               robot_smach_states.Say(robot, ["Starting the cleanup challenge",
                                                              "What a mess here, let's clean this room!",
                                                              "Let's see if I can find some garbage here",
                                                              "All I want to do is clean this mess up!"], block=False),
                              transitions={"spoken": "ITERATE_NEXT_LOC"})

# Here the designator cleanup_locationsr has to be iterated over to visit all locations of the room (see designator_iterator.py)
# How is this to be done?
        smach.StateMachine.add('ITERATE_NEXT_LOC',
                               robot_smach_states.IterateDesignator(cleanup_locationsr, location_des.writeable),
                               transitions={"next": "INSPECT",
                                            "stop_iteration": "RETURN_TO_OPERATOR"})

        smach.StateMachine.add("INSPECT",
                                CleanInspect(robot, location_des), transitions={"done": "ITERATE_NEXT_LOC"})

        smach.StateMachine.add("RETURN_TO_OPERATOR",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=ds.EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.starting_point),
                                                                     radius=0.3),
                               transitions={"arrived": "SAY_CLEANED_ROOM",
                                            "unreachable": "SAY_CLEANED_ROOM",
                                            "goal_not_defined": "SAY_CLEANED_ROOM"})

        smach.StateMachine.add('SAY_CLEANED_ROOM',
                               robot_smach_states.SayFormatted(robot,
                                   ["I successfully cleaned the {room}!",
                                   "All done in the {room}. Am I a good robot now?",
                                   "There, I cleaned up your mess in the {room}, are you happy now!"],
                                                               room=roomr, block=False),
                               transitions={"spoken": "Done"})

    return sm
