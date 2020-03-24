# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import arms
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import Entity
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator

# Challenge storing groceries
from near_object_designator import NearObjectSpotDesignator
from similarity import SimilarEntityDesignator

MIN_GRAB_OBJECT_HEIGHT = 0.0
MAX_GRAB_OBJECT_WIDTH = 1.8


class StoreSingleItem(smach.StateMachine):
    """
    Store a single item at another place
    """

    def __init__(self, robot, item_designator, place_pose_designator, arm=None):
        """
        Constructor

        :param robot: robot object
        :param item_designator:
        :param place_pose_designator: EdEntityDesignator designating the item to grab
        :param arm (optional): arm to use to store the item
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        if not arm:
            arm = ds.UnoccupiedArmDesignator(robot, {"required_trajectories": ["prepare_grasp", "prepare_place"],
                                                     "required_goals": ["carrying_pose"],
                                                     "required_gripper_types": [arms.GripperTypes.GRASPING]},
                                             name="empty_arm_designator").lockable()

        with self:
            smach.StateMachine.add("CHOOSE_ARM",
                                   states.utility.LockDesignator(arm),
                                   transitions={'locked': 'GRAB'}
                                   )

            smach.StateMachine.add("GRAB",
                                   states.manipulation.Grab(robot, item_designator, arm),
                                   transitions={'done': 'PLACE',
                                                'failed': 'failed'}
                                   )

            smach.StateMachine.add("PLACE",
                                   states.manipulation.Place(robot,
                                                             item_designator,
                                                             place_pose_designator,
                                                             arm),
                                   transitions={'done': 'succeeded',
                                                'failed': 'failed'}
                                   )


class StoreItems(smach.StateMachine):
    """
    Store a number of items from one place to another
    """

    def __init__(self, robot, source_entity, target_entity, item_classifications, knowledge):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "preempted"])

        segmented_entities_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        entities_designator = ds.VariableDesignator([], resolve_type=[Entity])
        item_designator = ds.VariableDesignator(resolve_type=Entity)

        near_object_designator = SimilarEntityDesignator(robot, item_designator, item_classifications, knowledge,
                                                         name="similar_object_designator")
        place_near_designator = NearObjectSpotDesignator(robot, near_object_designator, target_entity,
                                                         name="place_near_designator")

        arm = ds.UnoccupiedArmDesignator(robot, {"required_trajectories": ["prepare_grasp", "prepare_place"],
                                                 "required_goals": ["carrying_pose"],
                                                 "required_gripper_types": [arms.GripperTypes.GRASPING]},
                                         name="empty_arm_designator").lockable()
        place_anywhere_designator = EmptySpotDesignator(robot, target_entity, arm, area="on_top_of", name="empty_spot_designator")

        with self:
            smach.StateMachine.add('INSPECT',
                                   states.world_model.Inspect(robot, source_entity, segmented_entities_designator),
                                   transitions={'done': 'CONVERT_ENTITIES',
                                                'failed': 'failed'})

            @smach.cb_interface(outcomes=["converted"])
            def convert(userdata=None):
                """ convert Classificationresult to Entitiy"""
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                entities = []
                segmented_entities = segmented_entities_designator.resolve()
                for seg_entity in segmented_entities:
                    e = robot.ed.get_entity(seg_entity.id)
                    entities.append(e)
                writer = entities_designator.writeable
                writer.write(entities)
                return "converted"

            smach.StateMachine.add("CONVERT_ENTITIES",
                                   smach.CBState(convert),
                                   transitions={'converted': 'RANGE_ITERATOR'})

            # Begin setup iterator
            range_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(5),
                                            it_label='index',
                                            exhausted_outcome='succeeded')

            with range_iterator:
                contained_sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

                # process a single item
                with contained_sm:
                    smach.StateMachine.add('ITERATE_ENTITY',
                                           states.designator_iterator.IterateDesignator(entities_designator,
                                                                                        item_designator.writeable),
                                           transitions={'next': 'CHECK_SIMILAR_ITEM',
                                                        'stop_iteration': 'failed'}
                                           )

                    @smach.cb_interface(outcomes=["item_found", "no_similar_item"])
                    def check_similar_item(userdata=None):
                        rospy.loginfo("Going to store entity {}".format(item_designator.resolve()))
                        if near_object_designator.resolve():
                            return "item_found"
                        return "no_similar_item"

                    smach.StateMachine.add("CHECK_SIMILAR_ITEM",
                                           smach.CBState(check_similar_item),
                                           transitions={"item_found": "STORE_NEAR_ITEM",
                                                        "no_similar_item": "STORE_ANYWHERE"})

                    smach.StateMachine.add('STORE_NEAR_ITEM',
                                           StoreSingleItem(robot, item_designator, place_near_designator),
                                           transitions={'succeeded': 'succeeded',
                                                        'failed': 'failed'}
                                           )

                    smach.StateMachine.add('STORE_ANYWHERE',
                                           StoreSingleItem(robot, item_designator, place_anywhere_designator, arm),
                                           transitions={'succeeded': 'succeeded',
                                                        'failed': 'failed'}
                                           )

                smach.Iterator.set_contained_state('SINGLE_ITEM',
                                                   contained_sm,
                                                   loop_outcomes=['succeeded', 'failed'])

            smach.StateMachine.add('RANGE_ITERATOR', range_iterator,
                                   {'succeeded': 'succeeded',
                                    'failed': 'failed'})
