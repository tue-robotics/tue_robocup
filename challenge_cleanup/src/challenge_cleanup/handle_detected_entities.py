import smach
import rospy
from robot_smach_states.util.designators import EntityByIdDesignator
import robot_skills.util.kdl_conversions as kdl
from robot_skills.util.entity import Entity

from operator_cleanup import OperatorCleanup
from self_cleanup import SelfCleanup  # , SelfCleanup2


def _loginfo_color(text):
    rospy.loginfo('\033[94m' + text + '\033[0m')


class SelectEntity(smach.State):
    def __init__(self, robot, entitity_classifications_designator, selected_entity_designator, xy_limit=0.15,
                 xy_ratio=4):
        smach.State.__init__(self, outcomes=["entity_selected", "no_entities_left"])
        self._robot = robot
        self._entity_classifications_designator = entitity_classifications_designator
        self._selected_entity_designator = selected_entity_designator
        self._xy_limit = xy_limit
        self._xy_ratio = max(xy_ratio, 0.001)
        self._xy_ratio = max(self._xy_ratio, 1/self._xy_ratio)

    def execute(self, userdata=None):

        # Try to pop item from entities_ids_designator
        correct_entity = False
        while not correct_entity:
            try:
                entity_classification = self._entity_classifications_designator.resolve().pop()
            except Exception:
                return "no_entities_left"

            entity = EntityByIdDesignator(self._robot, entity_classification.id).resolve()  # type: Entity
            shape = entity.shape
            size_x = max(shape.x_max - shape.x_min, 0.001)
            size_y = max(shape.y_max - shape.y_min, 0.001)

            if size_x > self._xy_limit or size_y > self._xy_limit:
                continue

            if not 1/min(self._xy_ratio, 1000) <= size_x/size_y <= min(self._xy_ratio, 1000):
                continue

            correct_entity = True

        rospy.loginfo("We have selected the entity with id %s" % entity_classification.id)
        self._selected_entity_designator.id_ = entity_classification.id

        return "entity_selected"


class DetermineAction(smach.State):
    def __init__(self, robot, selected_entity_designator):
        smach.State.__init__(self, outcomes=["self", "operator", "failed"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    @staticmethod
    def _get_action_outcome(e):
        _loginfo_color("== GET ACTION OUTCOME ==")
        _loginfo_color("Entity type: '%s'" % e.type)

        #TODO: determine when to let a human solve our problems

        # Check if the object is on the ground
        if e.pose.frame.p.z() < 0.4:
            _loginfo_color("Object is on the ground, we cannot grasp it, call for help")
            return "operator"
        else:
            _loginfo_color("Object is not on the ground, we can grasp it")
            return "self"

    def execute(self, userdata=None):
        rospy.sleep(0.1)  # sleep because ed needs time to update
        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '%s'" % selected_entity.type)

        # If we don't know the entity type, try to classify again
        if selected_entity.type == "" or selected_entity.type == "unknown":
            # Make sure the head looks at the entity
            rospy.loginfo("entity: {}".format(selected_entity))
            pos = selected_entity.pose.frame.p
            self._robot.head.look_at_point(kdl.VectorStamped(pos.x(), pos.y(), 0.8, "/map"), timeout=10)

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
    """
    Handle all the found entities in the current location, and decide what to do
    - handle the entity yourself, or
    - have the operator handle the object
    """
    def __init__(self, robot, found_entity_classifications_designator, location_id, segment_area, room_des):

        smach.StateMachine.__init__(self, outcomes=['done'])

        selected_entity_designator = EntityByIdDesignator(robot, "TBD", name='selected_entity_designator', )

        with self:

            smach.StateMachine.add("SELECT_ENTITY",
                                   SelectEntity(robot, found_entity_classifications_designator,
                                                selected_entity_designator),
                                   transitions={"entity_selected": "DETERMINE_ACTION",
                                                "no_entities_left": "done"})

            smach.StateMachine.add("DETERMINE_ACTION",
                                   DetermineAction(robot, selected_entity_designator),
                                   transitions={"self": "SELF_CLEANUP",
                                                "operator": "OPERATOR_CLEANUP",
                                                "failed": "SELECT_ENTITY"})

            smach.StateMachine.add("SELF_CLEANUP", SelfCleanup(robot, selected_entity_designator, room_des),
                                   transitions={"done": "SELECT_ENTITY",
                                                "failed": "SELECT_ENTITY"})
            # smach.StateMachine.add("SELF_CLEANUP", SelfCleanup2(robot, selected_entity_designator, room_des),
            #                        transitions={"done": "SELECT_ENTITY",
            #                                     "failed": "SELECT_ENTITY"})

            smach.StateMachine.add("OPERATOR_CLEANUP", OperatorCleanup(robot, selected_entity_designator, location_id,
                                                                       segment_area),
                                   transitions={"cleanup": "SELF_CLEANUP",
                                                "no_cleanup": "SELECT_ENTITY"})
