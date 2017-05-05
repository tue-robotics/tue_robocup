import rospy
import smach
from robot_smach_states.util.designators import EntityByIdDesignator

from self_cleanup import SelfCleanup


def _loginfo_color(text):
    rospy.loginfo('\033[94m' + text + '\033[0m')


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
            return "no_entities_left"

        rospy.loginfo("We have selected the entity with id %s" % entity_classification.id)
        self._selected_entity_designator.id_ = entity_classification.id

        return "entity_selected"


class HandleDetectedEntities(smach.StateMachine):
    def __init__(self, robot, found_entity_classifications_designator, location_id, segment_area):
        smach.StateMachine.__init__(self, outcomes=['done'])

        selected_entity_designator = EntityByIdDesignator(robot, "TBD", name='selected_entity_designator', )

        with self:
            smach.StateMachine.add("SELECT_ENTITY",
                                   SelectEntity(robot, found_entity_classifications_designator,
                                                selected_entity_designator),
                                   transitions={"entity_selected": "SELF_CLEANUP",
                                                "no_entities_left": "done"})

            smach.StateMachine.add("SELF_CLEANUP",
                                   SelfCleanup(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"done": "SELECT_ENTITY", "failed": "SELECT_ENTITY"})
