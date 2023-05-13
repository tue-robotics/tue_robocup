# ROS
import rospy
import smach

# TU/e
from ed.entity import Entity
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.arm.arms import GripperTypes
from robot_skills.classification_result import ClassificationResult
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator
from robot_smach_states.util.designators.core import Designator

# Challenge storing groceries
from challenge_storing_groceries.near_object_designator import NearObjectSpotDesignator
from challenge_storing_groceries.similarity import SimilarEntityDesignator

MIN_GRAB_OBJECT_HEIGHT = 0.0
MAX_GRAB_OBJECT_WIDTH = 1.8


class StoreSingleItem(smach.StateMachine):
    """
    Store a single item at another place
    """

    def __init__(self, robot, item_designator, place_pose_designator, arm=None, room: Designator = None):
        """
        Constructor

        :param robot: robot object
        :param item_designator:
        :param place_pose_designator: EdEntityDesignator designating the item to grab
        :param arm (optional): arm to use to store the item
        :param room (optional): room to stay in while storing the item
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        if not arm:
            arm = ds.UnoccupiedArmDesignator(robot, {"required_trajectories": ["prepare_grasp", "prepare_place"],
                                                     "required_goals": ["carrying_pose"],
                                                     "required_gripper_types": [GripperTypes.GRASPING]},
                                             name="empty_arm_designator").lockable()

        with self:
            smach.StateMachine.add("CHOOSE_ARM",
                                   states.utility.LockDesignator(arm),
                                   transitions={'locked': 'GRAB'}
                                   )

            smach.StateMachine.add("GRAB",
                                   states.manipulation.Grab(robot, item_designator, arm, room),
                                   transitions={'done': 'PLACE',
                                                'failed': 'failed'}
                                   )

            smach.StateMachine.add("PLACE",
                                   states.manipulation.Place(robot,
                                                             item_designator,
                                                             place_pose_designator,
                                                             arm),
                                   transitions={'done': 'succeeded',
                                                'failed': 'SAY_HANDOVER'}
                                   )

            smach.StateMachine.add("SAY_HANDOVER",
                                   states.human_interaction.Say(robot, ["I failed to place the object, "
                                                                        "I will hand it over to you"]),
                                   transitions={'spoken': 'HANDOVER'}
                                   )

            smach.StateMachine.add("HANDOVER",
                                   states.manipulation.HandoverToHuman(robot, arm),
                                   transitions={'succeeded': 'failed',
                                                'failed': 'failed'}
                                   )


class StoreItems(smach.StateMachine):
    """
    Store a number of items from one place to another
    """

    def __init__(self, robot, source_entity, target_entity, item_classifications, knowledge, room: Designator = None):
        """
        Constructor

        :param robot: robot object
        :param source_entity: EdEntityDesignator designating the source entity
        :param target_entity: EdEntityDesignator designating the target entity
        :param item_classifications: dict of item classifications
        :param knowledge: knowledge object
        :param room (optional): room to stay in while storing the item
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "preempted"])

        segmented_entities_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        closest_item_designator = ds.VariableDesignator(resolve_type=Entity)

        current_blacklist = []

        near_object_designator = SimilarEntityDesignator(robot, closest_item_designator, item_classifications,
                                                         knowledge,
                                                         name="similar_object_designator")
        place_near_designator = NearObjectSpotDesignator(robot, near_object_designator, target_entity,
                                                         name="place_near_designator")

        arm = ds.UnoccupiedArmDesignator(robot, {"required_trajectories": ["prepare_grasp", "prepare_place"],
                                                 "required_goals": ["carrying_pose", "handover_to_human"],
                                                 "required_gripper_types": [GripperTypes.GRASPING]},
                                         name="empty_arm_designator").lockable()
        place_anywhere_designator = EmptySpotDesignator(robot, target_entity, arm, area=knowledge.default_area,
                                                        name="empty_spot_designator")

        with self:
            smach.StateMachine.add('INSPECT',
                                   states.world_model.Inspect(robot, source_entity,
                                                              segmented_entities_designator,
                                                              navigation_area=knowledge.inspect_area,
                                                              fit_supporting_entity=False,
                                                              room=room),
                                   transitions={'done': 'SELECT_CLOSEST_ENTITY',
                                                'failed': 'failed'})

            @smach.cb_interface(outcomes=["selected", "no_entities"])
            def SelectClosestEntity(userdata=None):
                """ convert Classificationresult to Entity and select the closest for grabbing"""
                entities = []
                distances = []
                segmented_entities = segmented_entities_designator.resolve()
                hero_pose = robot.base.get_location()

                for seg_entity in segmented_entities:
                    if seg_entity.uuid in current_blacklist:
                        continue

                    e = robot.ed.get_entity(seg_entity.uuid)

                    distance = e.distance_to_2d(hero_pose.frame.p)
                    entities.append(e)
                    distances.append(distance)

                if not entities:
                    return "no_entities"

                closest_entity = entities[distances.index(min(distances))]
                writer = closest_item_designator.writeable
                writer.write(closest_entity)

                return "selected"

            smach.StateMachine.add("SELECT_CLOSEST_ENTITY",
                                   smach.CBState(SelectClosestEntity),
                                   transitions={'selected': 'CHECK_SIMILAR_ITEM',
                                                'no_entities': 'succeeded'})

            @smach.cb_interface(outcomes=["item_found", "no_similar_item"])
            def check_similar_item(userdata=None):
                rospy.loginfo("Going to store {}".format(closest_item_designator.resolve()))
                if near_object_designator.resolve():
                    return "item_found"
                return "no_similar_item"

            smach.StateMachine.add("CHECK_SIMILAR_ITEM",
                                   smach.CBState(check_similar_item),
                                   transitions={"item_found": "STORE_NEAR_ITEM",
                                                "no_similar_item": "STORE_ANYWHERE"})

            smach.StateMachine.add('STORE_NEAR_ITEM',
                                   StoreSingleItem(robot, closest_item_designator, place_near_designator, room=room),
                                   transitions={'succeeded': 'INSPECT',
                                                'failed': 'INSPECT'}
                                   )

            @smach.cb_interface(outcomes=["done"])
            def add_item_to_blacklist(userdata=None):
                current_blacklist.append(closest_item_designator.resolve().uuid)
                return "done"

            smach.StateMachine.add("ADD_ITEM_TO_BLACKLIST",
                                   smach.CBState(add_item_to_blacklist),
                                   transitions={"done": "INSPECT"})

            smach.StateMachine.add('STORE_ANYWHERE',
                                   StoreSingleItem(robot,
                                                   closest_item_designator,
                                                   place_anywhere_designator,
                                                   arm,
                                                   room=room),
                                   transitions={'succeeded': 'INSPECT',
                                                'failed': 'ADD_ITEM_TO_BLACKLIST'}
                                   )


if __name__ == '__main__':
    import sys
    from robot_skills.get_robot import get_robot
    from robot_smach_states.util.designators import EntityByIdDesignator, ArmDesignator

    if len(sys.argv) < 3:
        print(f"usage: python {sys.argv[0]} ROBOT ENTITY_ID [Optional: shelf ID]")
        sys.exit()

    place_area = "on_top_of"
    if len(sys.argv) > 3:
        place_area = sys.argv[3]

    rospy.init_node('test_inspect_shelves')

    robot = get_robot(sys.argv[1])
    robot.reset_all_arms()

    entityDes = EntityByIdDesignator(robot, uuid=sys.argv[2])

    shelfDes = EntityByIdDesignator(robot, uuid="cabinet")
    armDes = ArmDesignator(robot)
    placePoseDes = EmptySpotDesignator(robot, shelfDes, armDes, area=place_area, name="empty_spot_designator")

    sm = StoreSingleItem(robot, entityDes, placePoseDes)
    sm.execute()
