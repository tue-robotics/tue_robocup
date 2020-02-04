import smach
import rospy
import robot_smach_states
import hmi

from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.util.entity import Entity
from robot_skills import arms
import robot_smach_states.util.designators as ds

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')

#ToDo: Location for trash depends on the room chosen: trash_bin (living) or trash_can (kitchen). Not handled yet.


class DropPoseDesignator(ds.Designator):
    def __init__(self, robot, entity_des, drop_height, name=None):
        super(DropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)

        self._robot = robot
        self._entity_des = entity_des
        self._drop_height = drop_height

    def _resolve(self):
        # ToDo: Make this happen for the bin in the chosen room...

        try:
            frame = self._entity_des.resolve()._pose
        except Exception:
            rospy.logerr(Exception.message)
            return None

        frame.p.z(self._drop_height)

        return FrameStamped(frame, "/map")


class StorePlaceDesignator(ds.Designator):
    def __init__(self, robot, selected_entity_type_category_designator, name=None):
        super(StorePlaceDesignator, self).__init__(resolve_type=Entity, name=name)

        self._robot = robot
        self._selected_entity_type_category_designator = selected_entity_type_category_designator

    def _resolve(self):
        type_category = self._selected_entity_type_category_designator.resolve()

        if type_category is None:
            rospy.logerr("Could not resolve the type/category!")
            return None

        item_category = challenge_knowledge.common.get_object_category(type_category)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
        elif type_category in challenge_knowledge.common.object_categories:
            location, area_name = challenge_knowledge.common.get_object_category_location(type_category)
        else:
            rospy.logerr("Could not resolve the object category of {}".format(e.type))
            return None

        entities = self._robot.ed.get_entities(id=location)
        if entities:
            return entities[0]
        else:
            return None


class StorePlaceLocation(smach.State):
    def __init__(self, robot, selected_entity_designator, store_loc_des):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self._robot = robot
        self._selected_entity_designator = selected_entity_designator
        self._store_loc_des = store_loc_des

    def execute(self, userdata=None):
        e = self._selected_entity_designator.resolve()

        if e is None:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        item_category = challenge_knowledge.common.get_object_category(e.type)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
            return area_name
        else:
            rospy.logerr("Could not resolve the object category of {}".format(e.type))
            return "failed"


class CategoryToLocation(smach.State):
    def __init__(self, category_des, entity_id_des, area_name_des):
        smach.State.__init__(self, outcomes=["trashbin", "other", "failed"])

        self._category_des = category_des
        self._entity_id_des = entity_id_des
        self._area_name_des = area_name_des

    def execute(self, ud=None):
        category = self._category_des.resolve()
        if not category:
            return "failed"

        if category == "trash":
            self._entity_id_des.write(challenge_knowledge.trashbin_id)
            return "trashbin"

        location, area_name = challenge_knowledge.common.get_object_category_location(category)
        if not location:
            return "failed"

        self._entity_id_des.write(location)
        if area_name:
            self._area_name_des.write(area_name)
        return "other"


class EntityToCategory(smach.State):
    def __init__(self, robot, selected_entity_designator, category_des):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator
        self._category_des = category_des

    def execute(self, userdata=None):
        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '{}'".format(selected_entity.type))

        category = challenge_knowledge.common.get_object_category(selected_entity.type)
        if category:
            self._category_des.write(category)
            return "done"
        else:
            return "failed"


class OperatorToCategory(smach.StateMachine):
    def __init__(self, robot, object_category_des, room_des):
        smach.StateMachine.__init__(self, outcomes=["done", "failed"])

        room_id_des = ds.AttrDesignator(room_des, "id", resolve_type=str)

        with self:
            smach.StateMachine.add("LOOK_INTO_ROOM", robot_smach_states.NavigateToRoom(robot, room_des, room_des),
                                   transitions={"arrived": "SAY_COME_TO_ME",
                                                "unreachable": "SAY_COME_TO_ME",
                                                "goal_not_defined": "SAY_COME_TO_ME"})

            smach.StateMachine.add("SAY_COME_TO_ME", robot_smach_states.Say(robot,
                                                                            "Operator, please come to me in the {room}",
                                                                            room=room_id_des, block=True),
                                   transitions={"spoken": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR", robot_smach_states.WaitTime(4),
                                   transitions={"waited": "ASK_WHICH_CATERGORY",
                                                "preempted": "ASK_WHICH_CATERGORY"})

            smach.StateMachine.add("ASK_WHICH_CATERGORY", AskWhichCategory(robot,
                ds.Designator(challenge_knowledge.category_grammar),
                                                                           object_category_des),
                                   transitions={"done": "done"})


class ArmFree(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata=None):
        d = ds.UnoccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"


class ArmOccupied(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata=None):
        d = ds.OccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"


class AskWhichCategory(smach.StateMachine):
    # Logic in this code is still flawed. No correct repetition.....
    # EXAMINE challenge_restaurant->take_orders.py for information about structure if interaction
    def __init__(self, robot, category_grammar, categoryw):
        smach.StateMachine.__init__(self, outcomes=["done"])

        hmi_result_des = ds.VariableDesignator(resolve_type=hmi.HMIResult, name="hmi_result_des")
        category_des = ds.FuncDesignator(ds.AttrDesignator(hmi_result_des, "semantics", resolve_type=unicode),
                                          str, resolve_type=str)

        @smach.cb_interface(outcomes=['done'])
        def write_category(ud, des_read, des_write):
            # type: (object, ds.Designator, ds.Designator) -> str
            assert(ds.is_writeable(des_write))
            assert(des_write.resolve_type == des_read.resolve_type)
            des_write.write(des_read.resolve())
            return 'done'

        with self:
            smach.StateMachine.add("ASK_WHERE_TO_DROP", robot_smach_states.Say(robot,
                "Please look at the object in my gripper and tell me which category it is. If it should be thrown away,"
                "call it trash", block=True),
                                   transitions={"spoken": "HEAR_LOCATION"})

            smach.StateMachine.add("HEAR_LOCATION", robot_smach_states.HearOptionsExtra(robot, category_grammar,
                                                                                        ds.writeable(hmi_result_des)),
                                   transitions={"heard": "SAY_HEARD_CORRECT",
                                                "no_result": "ASK_WHERE_TO_DROP"})
            smach.StateMachine.add("SAY_HEARD_CORRECT", robot_smach_states.Say(
                robot, "I understood that the object is of category {category}, is this correct?",
                category=category_des,
                block=True),
                                   transitions={"spoken": "HEAR_CORRECT"})
            smach.StateMachine.add("HEAR_CORRECT", robot_smach_states.AskYesNo(robot),
                                   transitions={"yes": "WRITE_CATEGORY",
                                                "no": "ASK_WHERE_TO_DROP",
                                                "no_result": "ASK_WHERE_TO_DROP"})
            smach.StateMachine.add('WRITE_CATEGORY', smach.CBState(write_category, cb_args=[category_des, categoryw]),
                                   transitions={'done': 'done'})


class SelfCleanup(smach.StateMachine):
    """
    Grab the current object and:
    - Bring it to the predefined destination, or
    - Drop it in the designated waste bin
    """
    def __init__(self, robot, selected_entity_designator, room_des):

        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        store_entity_id_des = ds.VariableDesignator(resolve_type=str, name="store_entity_id")
        store_entity_des = ds.EdEntityDesignator(robot, id_designator=store_entity_id_des)

        selected_entity_type_des = ds.AttrDesignator(selected_entity_designator, "type", resolve_type=str)

        store_area_name_des = ds.VariableDesignator(resolve_type=str, name="store_entity_id")

        trash_place_pose = DropPoseDesignator(robot, store_entity_des, 0.6, "drop_pose")

        category_des = ds.VariableDesignator(resolve_type=str, name="category_des")

        with self:

            smach.StateMachine.add("SPEAK", robot_smach_states.Say(robot, ["I will pick-up the {object}",
                                                                           "Let's move the {object}"],
                                                                   object=selected_entity_type_des,
                                                                   block=True),
                                   transitions={"spoken": "GRAB"})

            smach.StateMachine.add(
                "GRAB",
                robot_smach_states.Grab(
                    robot,
                    selected_entity_designator,
                    ds.UnoccupiedArmDesignator(robot,
                                               arm_properties={"required_trajectories": ["prepare_grasp"],
                                                               "required_goals": ["carrying_pose"],
                                                               "required_gripper_types": [arms.GripperTypes.GRASPING]},
                                               name="empty_arm_designator")),
                transitions={"done": "SAY_GRAB_SUCCESS", "failed": "ARM_RESET"})

            smach.StateMachine.add("ARM_RESET",robot_smach_states.ArmToJointConfig(
                    robot,
                    ds.UnoccupiedArmDesignator(robot,
                                               arm_properties={"required_goals": ["reset"]},
                                               name="empty_arm_designator"),
                    "reset"),
                transitions={"succeeded": "SAY_GRAB_FAILED", "failed": "SAY_GRAB_FAILED"})

            smach.StateMachine.add('SAY_GRAB_SUCCESS',
                                   robot_smach_states.Say(robot, ["Now I am going to move this item",
                                                                  "Let's clean up this object",
                                                                  "Away with this one",
                                                                  "Everything will be cleaned"], block=False),
                                   transitions={"spoken": "GET_CATEGORY"})

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
                                   transitions={"yes": "GET_CATEGORY",
                                                "no": "done"})

            # # ROBOT
            # smach.StateMachine.add('GET_CATEGORY',
            #                        EntityToCategory(robot, selected_entity_designator, category_des.writeable),
            #                        transitions={"done": "DETERMINE_PLACE_LOCATION",
            #                                     "failed": "NAVIGATE_TO_TRASH"})

            # OPERATOR
            smach.StateMachine.add('GET_CATEGORY', OperatorToCategory(robot, category_des.writeable, room_des),
                                   transitions={"done": "DETERMINE_PLACE_LOCATION",
                                                "failed": "NAVIGATE_TO_TRASH"})

            smach.StateMachine.add('DETERMINE_PLACE_LOCATION',
                                   CategoryToLocation(category_des, store_entity_id_des.writeable,
                                                      store_area_name_des.writeable),
                                   transitions={"trashbin": "INSPECT_TRASH",
                                                "other": "PLACE_TO_STORE",
                                                "failed": "NAVIGATE_TO_TRASH"})

            smach.StateMachine.add('NAVIGATE_TO_TRASH',
                                   robot_smach_states.NavigateToPlace(robot, trash_place_pose,
                                                                      ds.OccupiedArmDesignator(robot, {},
                                                                            name="occupied_arm_designator")),
                                   transitions={"arrived": "PLACE_IN_TRASH",
                                                "unreachable": "SAY_PLACE_FAILED",
                                                "goal_not_defined": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('INSPECT_TRASH',
                                   robot_smach_states.Inspect(robot,
                                                              store_entity_des),
                                   transitions={"done": "PLACE_IN_TRASH",
                                                "failed": "SAY_PLACE_FAILED"})

            arm_properties_place = {"required_trajectories": ["prepare_place"],
                                    "required_gripper_types": [arms.GripperTypes.GRASPING]}
            arm_designator_place = ds.OccupiedArmDesignator(robot, arm_properties_place, name="occupied_arm_designator")

            smach.StateMachine.add('PLACE_IN_TRASH',
                                   robot_smach_states.Place(robot,
                                                            selected_entity_designator,
                                                            trash_place_pose,
                                                            arm_designator_place),
                                   transitions={"done": "SAY_PLACE_SUCCESS",
                                                "failed": "SAY_PLACE_FAILED"})
            
            arm_designator_place_store = ds.OccupiedArmDesignator(robot, arm_properties_place,
                                                                  name="occupied_arm_designator")
            smach.StateMachine.add('PLACE_TO_STORE',
                                   robot_smach_states.Place(robot,
                                                            selected_entity_designator,
                                                            store_entity_des,
                                                            arm_designator_place_store,
                                                            "on_top_of"),
                                   transitions={"done": "SAY_PLACE_SUCCESS", "failed": "SAY_PLACE_FAILED"})

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


# class SelfCleanup2(smach.StateMachine):
#     """
#     Grab the current object and:
#     - Bring it to the predefined destination, or
#     - Drop it in the designated waste bin
#     """
#     def __init__(self, robot, selected_entity_designator, room_des):
#
#         smach.StateMachine.__init__(self, outcomes=['done', 'failed'])
#
#         trash_place_pose = DropPoseDesignator(robot, 0.6, "drop_pose")
#
#         room_id_des = ds.AttrDesignator(room_des, "id")
#         furniture_id_des = ds.VariableDesignator(resolve_type=str, name="drop_location")
#
#         with self:
#             smach.StateMachine.add("SPEAK", robot_smach_states.SayFormatted(robot, ["I will pick-up the {object}",
#                                                                                     "Let's move the {object}"],
#                                                                             object=selected_entity_designator,
#                                                                             block=True),
#                                    transitions={"spoken": "GRAB"})
#
#             smach.StateMachine.add("GRAB",
#                                    robot_smach_states.Grab(robot, selected_entity_designator,
#                                                            ds.UnoccupiedArmDesignator(robot, {},
#                                                                                       name="empty_arm_designator")),
#                                    transitions={"done": " planner couldn't plan a path to the specified constraints. Are the constraints you specified valid?
# SAY_GRAB_SUCCESS",
#                                                 "failed": "SAY_GRAB_FAILED"})
#
#             smach.StateMachine.add("LOOK_INTO_ROOM", robot_smach_states.NavigateToRoom(robot, room_des, room_des),
#                                    transitions={"arrived": "SAY_COME_TO_ME",
#                                                 "unreachable": "SAY_COME_TO_ME",
#                                                 "goal_not_defined": "SAY_COME_TO_ME"})
#
#             smach.StateMachine.add("SAY_COME_TO_ME", robot_smach_states.SayFormatted(robot,
#                                                         "Operator, please come to me in the {room}",
#                                                                                      room=room_id_des, block=True),
#                                    transitions={"spoken": "WAIT_FOR_OPERATOR"})
#
#             smach.StateMachine.add("WAIT_FOR_OPERATOR", robot_smach_states.FindPerson(robot),
#                                    transitions={"found": "ASK_WHERE_TO_PLACE",
#                                                 "failed": "ASK_WHERE_TO_PLACE"})
#
#             smach.StateMachine.add("ASK_WHERE_TO_PLACE", AskWhereToDrop(robot, challenge_knowledge.location_grammar,
#                                                                         furniture_id_des),
#                                    transitions={"done": "CHECK_ARM_FREE"})
#
#             # smach.StateMachine.add('SAY_GRAB_SUCCESS',
#             #                        robot_smach_states.Say(robot, ["Now I am going to toss the item in the trashbin",
#             #                                                       "Let's clean up this object",
#             #                                                       "Away with this garbage",
#             #                                                       "Everything will be cleaned"], block=False),
#             #                        transitions={"spoken": "DETERMINE_PLACE_LOCATION"})
#             #
#             # smach.StateMachine.add('SAY_GRAB_FAILED',
#             #                        robot_smach_states.Say(robot, ["I could not grab the item.",
#             #                                                       "I failed to grasp the item",
#             #                                                       "I cannot reach the item",
#             #                                                       "Item grab failed"], block=False),
#             #                        transitions={"spoken": "failed"})
#
#             smach.StateMachine.add('CHECK_ARM_FREE', ArmFree(robot),
#                                    transitions={"yes": "done",
#                                                 "no": "CHECK_ARM_OCCUPIED"})
#
#             smach.StateMachine.add('CHECK_ARM_OCCUPIED', ArmOccupied(robot),
#                                    transitions={"yes": "DETERMINE_PLACE_LOCATION",
#                                                 "no": "done"})
#
#             smach.StateMachine.add('DETERMINE_PLACE_LOCATION', DetermineCleanupLocation(robot,
#                                                                                         selected_entity_designator),
#                                    transitions={"trashbin": "INSPECT_TRASH",
#                                                 "other": "PLACE_TO_STORE",
#                                                 "failed": "NAVIGATE_TO_TRASH"})
#
#             smach.StateMachine.add('NAVIGATE_TO_TRASH',
#                                    robot_smach_states.NavigateToPlace(robot, trash_place_pose,
#                                                                       ds.OccupiedArmDesignator(robot, {},
#                                                                               name="occupied_arm_designator")),
#                                    transitions={"arrived": "PLACE_IN_TRASH",
#                                                 "unreachable": "SAY_PLACE_FAILED",
#                                                 "goal_not_defined": "SAY_PLACE_FAILED"})
#
#             smach.StateMachine.add('INSPECT_TRASH',
#                                    robot_smach_states.Inspect(robot,
#                                                               trash_designator),
#                                    transitions={"done": "PLACE_IN_TRASH",
#                                                 "failed": "SAY_PLACE_FAILED"})
#
#             smach.StateMachine.add('PLACE_IN_TRASH',
#                                    robot_smach_states.Place(robot,
#                                                             selected_entity_designator,
#                                                             trash_place_pose,
#                                                             ds.OccupiedArmDesignator(robot, {},
#                                                                                      name="occupied_arm_designator")),
#                                    transitions={"done": "SAY_PLACE_SUCCESS",
#                                                 "failed": "SAY_PLACE_FAILED"})
#
#             smach.StateMachine.add('PLACE_TO_STORE', robot_smach_states.Place(robot, selected_entity_designator,
#                                                                               item_store_entity,
#                                                                               ds.OccupiedArmDesignator(robot, {}, name=
#                                                                               "occupied_arm_designator"),
#                                                                               "on_top_of"),
#                                    transitions={"done": "SAY_PLACE_SUCCESS",
#                                                 "failed": "SAY_PLACE_FAILED"})
#
#             smach.StateMachine.add('SAY_PLACE_SUCCESS',
#                                    robot_smach_states.Say(robot, ["Bye bye!",
#                                                                   "Yeah!",
#                                                                   "Successfully disposed the item",
#                                                                   "Another score for HERO"], block=False),
#                                    transitions={"spoken": "CHECK_ARM_OCCUPIED"})
#
#             smach.StateMachine.add('SAY_PLACE_FAILED',
#                                    robot_smach_states.Say(robot, ["I could not cleanup the item.",
#                                                                   "I cannot put the item in the trashbin",
#                                                                   "Item cleanup failed"], block=False),
#                                    transitions={"spoken": "CHECK_ARM_OCCUPIED"})
