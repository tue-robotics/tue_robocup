import smach
import rospy
import robot_smach_states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator
import robot_skills.util.msg_constructors as msgs

from operator_cleanup import OperatorCleanup
from self_cleanup import SelfCleanup
from other_robot_cleanup import OtherRobotCleanup


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


class DetermineAction(smach.State):
    def __init__(self, robot, selected_entity_designator, known_types):
        smach.State.__init__(self, outcomes=["self", "operator", "other_robot", "failed"])
        self._robot = robot
        self._known_types = known_types
        self._selected_entity_designator = selected_entity_designator

    def _get_action_outcome(self, e):
        _loginfo_color("== GET ACTION OUTCOME ==")
        _loginfo_color("Known types: '%s'" % self._known_types)
        _loginfo_color("Entity type: '%s'" % e.type)
        _loginfo_color("Entity pose position z: '%.3f'" % e.pose.frame.p.z())

        # Check if the object is on the ground
        if e.pose.frame.p.z() < 0.4:
            _loginfo_color("Object is on the ground, we cannot grasp it, call for help")
            action = "other_robot"
        else:
            # Check if we know the object
            if e.type in self._known_types:
                _loginfo_color("Object is not on the ground, we can grasp it")
                action = "self"
            else:
                _loginfo_color("Unknown object")
                action = "operator"

        return action

    def execute(self, userdata):

        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '%s'" % selected_entity.type)

        # If we don't know the entity type, try to classify again
        if selected_entity.type == "" or selected_entity.type == "unknown":
            # Make sure the head looks at the entity
            pos = selected_entity.pose.frame.p
            self._robot.head.look_at_point(msgs.PointStamped(pos.x(), pos.y(), 0.8, "/map"), timeout=10)

            # This is needed because the head is not entirely still when the look_at_point function finishes
            rospy.sleep(1)

            # Inspect the entity again
            self._robot.ed.update_kinect(selected_entity.id)

            # Classify the entity again
            try:
                selected_entity.type = self._robot.ed.classify(ids=[selected_entity.id])[0].type
                rospy.loginfo("We classified the entity again; type = %s" % selected_entity.type)
            except Exception as e:
                rospy.logerr(e)

        return self._get_action_outcome(selected_entity)


class HandleDetectedEntities(smach.StateMachine):
    def __init__(self, robot, found_entity_classifications_designator, known_types, location_id, segment_area):

        smach.StateMachine.__init__(self, outcomes=['done'])

        selected_entity_designator = EntityByIdDesignator(robot, "TBD", name='selected_entity_designator', )

        with self:

            smach.StateMachine.add("SELECT_ENTITY",
                                   SelectEntity(robot, found_entity_classifications_designator,
                                                selected_entity_designator),
                                   transitions={"entity_selected": "DETERMINE_ACTION",
                                                "no_entities_left": "done"})

            smach.StateMachine.add("DETERMINE_ACTION",
                                   DetermineAction(robot, selected_entity_designator, known_types),
                                   transitions={"self": "SELF_CLEANUP",
                                                "operator": "OPERATOR_CLEANUP",
                                                "failed": "SELECT_ENTITY",
                                                "other_robot": "OTHER_ROBOT_CLEANUP"})

            smach.StateMachine.add("SELF_CLEANUP", SelfCleanup(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"done": "SELECT_ENTITY", "failed": "SELECT_ENTITY"})

            smach.StateMachine.add("OPERATOR_CLEANUP", OperatorCleanup(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"cleanup": "SELF_CLEANUP", "no_cleanup": "SELECT_ENTITY"})

            smach.StateMachine.add("OTHER_ROBOT_CLEANUP", OtherRobotCleanup(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"done": "SELECT_ENTITY"})
