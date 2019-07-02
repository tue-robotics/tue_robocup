import smach
import rospy
import robot_smach_states
import random

from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import UnoccupiedArmDesignator, OccupiedArmDesignator, Designator, EntityByIdDesignator

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')

#ToDo: Location for trash depends on the room chosen: trash_bin (living) or trash_can (kitchen). Not handled yet.


class DropPoseDesignator(Designator):
    def __init__(self, robot, drop_height, name):
        super(DropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)

        self._robot = robot
        self._drop_height = drop_height

    def _resolve(self):
        # Query ed
        #ToDo: Make this happen for the bin in the chosen room...

        try:
            frame = self._robot.ed.get_entity(id="trash_bin")._pose
        except Exception:
            return None

        frame.p.z(self._drop_height)

        return FrameStamped(frame, "/map")


class StorePlaceDesignator(Designator):
    def __init__(self, robot, name, selected_entity_designator):
        super(StorePlaceDesignator, self).__init__(resolve_type=Entity, name=name)

        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    def _resolve(self):
        e = self._selected_entity_designator.resolve()

        if e is None:
            rospy.logerr("Could not resolve the selected entity!")
            return None

        item_category = challenge_knowledge.common.get_object_category(e.type)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
            entities = self._robot.ed.get_entities(id=location)
            if entities:
                return entities[0]
            else:
                return None
        else:
            rospy.logerr("Could not resolve the object category of {}".format(e.type))
            return None


class StoreAreaDesignator(Designator):
    def __init__(self, robot, name, selected_entity_designator):
        super(StoreAreaDesignator, self).__init__(resolve_type=str, name=name)

        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    def _resolve(self):
        e = self._selected_entity_designator.resolve()

        if e is None:
            rospy.logerr("Could not resolve the selected entity!")
            return None

        item_category = challenge_knowledge.common.get_object_category(e.type)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
            return area_name
        else:
            rospy.logerr("Could not resolve the object category of {}".format(e.type))
            return None


class DetermineCleanupLocation(smach.State):
    def __init__(self, robot, selected_entity_designator):
        smach.State.__init__(self, outcomes=["trashbin", "other", "failed"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    def execute(self, userdata=None):
        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '%s'" % selected_entity.type)

        if challenge_knowledge.common.is_known_object(selected_entity.type):
            return "other"
        else:
            return "trashbin"


class ArmFree(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata=None):
        d = UnoccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"


class ArmOccupied(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata=None):
        d = OccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"


class SelfCleanup(smach.StateMachine):
    """
    Grab the current object and:
    - Bring it to the predefined destination, or
    - Drop it in the designated waste bin
    """
    def __init__(self, robot, selected_entity_designator, location_id, segment_area):

        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        trash_place_pose = DropPoseDesignator(robot, 0.6, "drop_pose")
        trash_designator = EntityByIdDesignator(robot, "trash_bin")
        item_store_entity = StorePlaceDesignator(robot,
                                                 "store_entity",
                                                 selected_entity_designator)
        # item_store_area = StorePlaceDesignator(robot,
        #                                        "store_area",
        #                                        selected_entity_designator)

        with self:

            smach.StateMachine.add("SPEAK", robot_smach_states.SayFormatted(robot, ["I will pick-up the {object}",
                                                                                    "Let's move the {object}"],
                                                                            object=selected_entity_designator,
                                                                            block=True),
                                   transitions={"spoken": "GRAB"})

            smach.StateMachine.add("GRAB",
                                   robot_smach_states.Grab(robot, selected_entity_designator,
                                                           UnoccupiedArmDesignator(robot, {},
                                                                                   name="empty_arm_designator")),
                                   transitions={"done": "SAY_GRAB_SUCCESS",
                                                "failed": "SAY_GRAB_FAILED"})

            smach.StateMachine.add('SAY_GRAB_SUCCESS',
                                   robot_smach_states.Say(robot, ["Now I am going to toss the item in the trashbin",
                                                                  "Let's clean up this object",
                                                                  "Away with this garbage",
                                                                  "Everything will be cleaned"], block=False),
                                   transitions={"spoken": "DETERMINE_PLACE_LOCATION"})

            smach.StateMachine.add('SAY_GRAB_FAILED',
                                   robot_smach_states.Say(robot, ["I could not grab the item.",
                                                                  "I failed to grasp the item",
                                                                  "I cannot reach the item",
                                                                  "Item grab failed"], block=False),
                                   transitions={"spoken": "failed"})

            smach.StateMachine.add('CHECK_ARM_FREE', ArmFree(robot),
                                   transitions={"yes": "done",
                                                "no": "CHECK_ARM_OCCUPIED"})

            smach.StateMachine.add('CHECK_ARM_OCCUPIED', ArmOccupied(robot),
                                   transitions={"yes": "DETERMINE_PLACE_LOCATION",
                                                "no": "done"})

            smach.StateMachine.add('DETERMINE_PLACE_LOCATION', DetermineCleanupLocation(robot,
                                                                                        selected_entity_designator),
                                   transitions={"trashbin": "INSPECT_TRASH",
                                                "other": "PLACE_TO_STORE",
                                                "failed": "NAVIGATE_TO_TRASH"})

            smach.StateMachine.add('NAVIGATE_TO_TRASH',
                                   robot_smach_states.NavigateToPlace(robot, trash_place_pose,
                                                                      OccupiedArmDesignator(robot, {},
                                                                            name="occupied_arm_designator")),
                                   transitions={"arrived": "PLACE_IN_TRASH",
                                                "unreachable": "SAY_PLACE_FAILED",
                                                "goal_not_defined": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('INSPECT_TRASH',
                                   robot_smach_states.Inspect(robot,
                                                              trash_designator),
                                   transitions={"done": "PLACE_IN_TRASH",
                                                "failed": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('PLACE_IN_TRASH',
                                   robot_smach_states.Place(robot, 
                                                            selected_entity_designator, 
                                                            trash_place_pose,
                                                            OccupiedArmDesignator(robot,
                                                                                  {},
                                                                                  name="occupied_arm_designator")),
                                   transitions={"done": "SAY_PLACE_SUCCESS",
                                                "failed": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('PLACE_TO_STORE', robot_smach_states.Place(robot, selected_entity_designator,
                                                                              item_store_entity,
                                                                              OccupiedArmDesignator(robot, {}, name=
                                                                              "occupied_arm_designator"),
                                                                              "on_top_of"),
                                   transitions={"done": "SAY_PLACE_SUCCESS",
                                                "failed": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('SAY_PLACE_SUCCESS',
                                   robot_smach_states.Say(robot, ["Bye bye!",
                                                                  "Yeah!",
                                                                  "Successfully disposed the item",
                                                                  "Another score for HERO"], block=False),
                                   transitions={"spoken": "CHECK_ARM_OCCUPIED"})

            smach.StateMachine.add('SAY_PLACE_FAILED',
                                   robot_smach_states.Say(robot, ["I could not cleanup the item.",
                                                                  "I cannot put the item in the trashbin",
                                                                  "Item cleanup failed"], block=False),
                                   transitions={"spoken": "CHECK_ARM_OCCUPIED"})
