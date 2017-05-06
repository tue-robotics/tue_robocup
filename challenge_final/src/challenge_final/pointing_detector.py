# ROS
import PyKDL as kdl
import rospy
import smach


class PointingDetector(smach.State):
    """ State to fill an EdEntityDesignator depending on the direction of an operator pointing at the entity
     """
    def __init__(self, robot, designator, super_type="furniture"):
        """ Constructor
        :param robot: robot object
        :param designator: Pointing designator to fill
        :param super_type: string indicating the required super type. If the RayTraceResult does not provide an entity
        of the desired super type, the closest entity with that desired supertype will be computed.
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        self._designator = designator
        self._super_type = super_type

    def execute(self, userdata):

        # Point head in the right direction

        # Get RayTraceResult from Reins function

        # result.intersection_point  (geometry_msgs/PointStamped)
        # result.entity_id (string)

        # Query the entity from ED
        entity = self._robot.ed.get_entity(id=result.entity_id)

        # If the result is of the desired super type, we can try to do something with it
        if entity.is_a(self._super_type):
            self._designator.set_id(identifier=entity.id)
            rospy.loginfo("Object pointed at: {}".format(entity.id))
            return "succeeded"

        # Otherwise, try to get the closest one
        entities = self.ed.get_entities()
        entities = [e for e in entities if e.is_a(self._super_type)]
        if not entities:
            rospy.logerr("ED doesn't contain any entities of super type {}".format(self._super_type))
            return "failed"

        # If we have entities, sort them according to distance
        # ToDO: this assumes the result is given in map frame
        raypos = kdl.Vector(result.intersection_point.point.x, result.intersection_point.point.y, 0.0)
        entities.sort(key=lambda e: e.distance_to_2d(raypos))
        self._designator.set_id(identifier=entities[0].id)
        rospy.loginfo("Object pointed at: {}".format(entities[0].id))
        return "succeeded"
