# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.classification_result import ClassificationResult
from robot_skills import arms
import robot_smach_states as states
from robot_smach_states.manipulation.place import EmptySpotDesignator
from robot_smach_states.util.designators import check_type
from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator, EntityByIdDesignator, UnoccupiedArmDesignator


class SelectEntity(smach.State):
    def __init__(self, robot, entitity_classifications_designator, selected_entity_designator):
        smach.State.__init__(self, outcomes=["entity_selected", "no_entities_left"])
        self._robot = robot
        self._entity_classifications_designator = entitity_classifications_designator
        self._selected_entity_designator = selected_entity_designator

    def execute(self, userdata):

        # Try to pop item from entities_ids_designator
        try:
            entity_classification = self._entity_classifications_designator.resolve().pop()
        except:
            self._robot.speech.speak("I cleaned everything up! Isn't that awesome?")
            return "no_entities_left"

        rospy.loginfo("We have selected the entity with id %s" % entity_classification.id)
        self._selected_entity_designator.id_ = entity_classification.id

        return "entity_selected"

class isitclear(smach.State):
    """
    Check if there are entities on the object in the world model
    """

    def __init__(self,
                 robot,
                 objectIDsDes):
        smach.State.__init__(self, outcomes=['clear', 'not_clear'])
        self._robot = robot
        self._object_designator = objectIDsDes

    def execute(self, userdata=None):
        rospy.loginfo("{}".format(self._object_designator.resolve()))
        if self._object_designator.resolve():
            return 'not_clear'
        else:
            return 'clear'


class Clear(smach.StateMachine):
    def __init__(self, robot, source_location, source_navArea, target_location, target_navArea, target_placeArea="on_top_of", source_searchArea="on_top_of"):
        """
        Let the given robot move to a location and remove all entities from that table one at a time

        :param robot: Robot to use
        :param source_location: Location which will be cleared
        :param target_location: Location where the objects will be placed
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        #check_type(source_location, Entity)
        #check_type(target_location, Entity)

        segmented_entities_designator = VariableDesignator([], resolve_type=[ClassificationResult])
        selected_entity_designator = EntityByIdDesignator(robot, "TBD", name='selected_entity_designator', )

        arm_des = UnoccupiedArmDesignator(
            robot,
            arm_properties={"required_trajectories": ["prepare_place", "prepare_grasp"],
                            "required_goals": ["carrying_pose"],
                            "required_gripper_types": [arms.GripperTypes.GRASPING]}).lockable()
        arm_des.lock()

        place_position = EmptySpotDesignator(robot, EdEntityDesignator(
                                             robot, id=target_location.id),
                                             arm_des,
                                             area="on_top_of"
                                             )

        with self:
            smach.StateMachine.add('INSPECT_SOURCE_ENTITY',
                                   states.Inspect(robot, source_location, objectIDsDes=segmented_entities_designator,
                                                  searchArea=source_searchArea, navigation_area=source_navArea),
                                   transitions={'done': 'DETERMINE_IF_CLEAR',
                                                'failed': 'failed'}
                                   )

            #smach.StateMachine.add('DETERMINE_IF_CLEAR',
            #                       isitclear(robot=robot,
            #                                 objectIDsDes=segmented_entities_designator),
            #                       transitions={'clear': 'done',
            #                                    'not_clear': 'failed'})

            smach.StateMachine.add('DETERMINE_IF_CLEAR',
                                   SelectEntity(robot, segmented_entities_designator, selected_entity_designator),
                                   transitions={'no_entities_left': 'done',
                                                'entity_selected': 'GRAB'}
                                   )

            smach.StateMachine.add('GRAB',
                                   states.Grab(robot, selected_entity_designator, arm_des),
                                   transitions={'done': 'INSPECT_TARGET',
                                                'failed': 'failed'}
                                   )

            smach.StateMachine.add('INSPECT_TARGET',
                                   states.Inspect(robot, target_location, searchArea=target_placeArea,
                                                  navigation_area=target_navArea),
                                   transitions={'done': 'PLACE',
                                                'failed': 'failed'}
                                   )

            smach.StateMachine.add('PLACE',
                                   states.Place(robot, selected_entity_designator, place_position, arm_des),
                                   transitions={'done': 'INSPECT_SOURCE_ENTITY',
                                                'failed': 'failed'}
                                   )

