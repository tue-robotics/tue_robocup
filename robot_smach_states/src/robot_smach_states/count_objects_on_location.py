import rospy
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.startup import startup
from robot_skills.classification_result import ClassificationResult


class CountObjectsOnLocation(smach.State):
    def __init__(self, robot, location, segmented_objects_designator, num_objects_designator,
                 segmentation_area='on_top_of', object_type='', threshold=0.0):
        """ Constructor

        :param robot: robot object
        :param location: Where to look for objects?
        :param segmented_objects_designator: a VariableDesignator (resolve_type=ClassificationResult) that stores the
            classification of an object
        :param num_objects_designator: a VariableDesignator(resolve_type=int).writeable() that will store the number
            of objects
        :param segmentation_area: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param threshold: float for classification score. Entities whose classification score is lower are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        """
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.robot = robot
        self.location = location
        self.segmentation_area = segmentation_area
        self.threshold = threshold
        self.object_type = object_type

        ds.checks.is_writeable(num_objects_designator)
        ds.checks.check_resolve_type(num_objects_designator, int)
        self.num_objects_designator = num_objects_designator
        self.segmented_objects_designator = segmented_objects_designator

    def execute(self, userdata=None):
        object_classifications = self.segmented_objects_designator.resolve()

        rospy.loginfo("Segmented %d objects" % len(object_classifications))
        if object_classifications:

            for idx, obj in enumerate(object_classifications):
                rospy.loginfo("   - Object {i} is a '{t}' (prob: {p}, ID: {id})".format(i=idx, t=obj.type,
                                                                                        id=obj.id, p=obj.probability))

            over_threshold = [obj for obj in object_classifications if obj.probability >= self.threshold]

            dropped = {obj.id: obj.probability for obj in object_classifications if obj.probability < self.threshold}
            rospy.debug("Dropping {l} entities due to low class. score (< {th}): {dropped}"
                        .format(th=self.threshold, dropped=dropped, l=len(dropped)))
            object_classifications = over_threshold

            list_objects = [obj for obj in object_classifications if obj.type == self.object_type.resolve()]
            num_objects = len(list_objects)
            rospy.loginfo("Counted {} objects matching the query".format(num_objects))
            self.num_objects_designator.write(num_objects)
            return 'done'
        else:
            return 'failed'


class InspectAndCount(smach.StateMachine):
    def __init__(self, robot, where_to_count_designator, type_to_count_designator, count_designator):
        """
        :param robot: robot object
        :param where_to_count_designator: Where to look for objects?
        :param type_to_count_designator: a VariableDesignator (resolve_type=string) that stores the type of the objects
            that should be counted
        :param count_designator:  VariableDesignator(resolve_type=int).writeable() that will store the number
            of objects
        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        entities = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=where_to_count_designator,
                                                                   objectIDsDes=entities,
                                                                   searchArea="on_top_of",
                                                                   navigation_area="in_front_of"),
                                   transitions={"done": "COUNT",
                                                "failed": "Aborted"})

            smach.StateMachine.add("COUNT",
                                   CountObjectsOnLocation(robot,
                                                          location=where_to_count_designator,
                                                          segmented_objects_designator=entities,
                                                          object_type=type_to_count_designator,
                                                          num_objects_designator=count_designator.writeable),
                                   transitions={'done': 'Done',
                                                'failed': 'Aborted'})
