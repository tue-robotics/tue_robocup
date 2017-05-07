# ROS
import PyKDL as kdl
import rospy
import smach

from robot_skills.util import kdl_conversions


def get_frame_from_vector(x_vector, origin):
    unit_z = kdl.Vector(0, 0, 1)
    unit_z_cross_diff = (unit_z * x_vector) / (unit_z * x_vector).Norm()
    y_vector = x_vector * unit_z_cross_diff
    z_vector = x_vector * y_vector

    rotation = kdl.Rotation(x_vector, y_vector, z_vector)
    translation = origin.vector

    frame_stamped = kdl_conversions.FrameStamped(kdl.Frame(rotation, translation), origin.frame_id)
    return frame_stamped


def get_ray_trace_from_closest_person(robot, arm_norm_threshold=0.1, upper_arm_norm_threshold=0.7):
    persons = robot.head.detect_persons_3d()
    valid_persons = [person for person in persons if "right_shoulder" in person]

    if not valid_persons:
        return None

    person = sorted(valid_persons, key=lambda x: x["right_shoulder"].point.z)[0]

    # Check if arms are pointing
    left_arm_valid = "left_wrist" in person and "left_elbow" in person and "left_shoulder" in person
    right_arm_valid = "right_wrist" in person and "right_elbow" in person and "right_shoulder" in person

    if left_arm_valid:
        left_wrist = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_wrist"]).projectToFrame("/map",
                                                                                                          robot.tf_listeners)
        left_elbow = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_elbow"]).projectToFrame("/map",
                                                                                                              robot.tf_listeners)
        left_shoulder = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_shoulder"]).projectToFrame(
            "/map",
            robot.tf_listeners)
        left_lower_arm_vector = (left_wrist - left_elbow) / (left_wrist - left_elbow).Norm()
        left_upper_arm_vector = (left_elbow - left_shoulder) / (left_elbow - left_shoulder).Norm()
        left_frame = get_frame_from_vector(left_lower_arm_vector, left_wrist.vector)
        left_arm_norm = (left_lower_arm_vector * left_upper_arm_vector).Norm()
        left_upper_arm_norm = (left_upper_arm_vector * kdl.Vector(0, 0, 1)).Norm()

        rospy.loginfo("Left arm norm: %.2f", left_arm_norm)
        rospy.loginfo("Upper left arm norm: %.2f", left_upper_arm_norm)
    else:
        rospy.loginfo("Left arm not valid because it does not contain all required bodyparts")

    if right_arm_valid:
        right_wrist = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["right_wrist"]).projectToFrame("/map",
                                                                                                                robot.tf_listeners)

        right_elbow = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["right_elbow"]).projectToFrame("/map",
                                                                                                               robot.tf_listeners)

        right_shoulder = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["right_shoulder"]).projectToFrame("/map",
                                                                                                                     robot.tf_listeners)

        right_lower_arm_vector = (right_wrist - right_elbow) / (right_wrist - right_elbow).Norm()
        right_upper_arm_vector = (right_elbow - right_shoulder) / (right_elbow - right_shoulder).Norm()

        right_frame = get_frame_from_vector(right_lower_arm_vector, right_wrist.vector)
        right_arm_norm = (right_lower_arm_vector * right_upper_arm_vector).Norm()
        right_upper_arm_norm = (right_upper_arm_vector * kdl.Vector(0, 0, 1)).Norm()

        rospy.loginfo("Right arm norm: %.2f", right_arm_norm)
        rospy.loginfo("Upper right arm norm: %.2f", right_upper_arm_norm)
    else:
        rospy.loginfo("Right arm not valid because it does not contain all required bodyparts")

    rospy.loginfo("Arm norm threshold: %.2f", arm_norm_threshold)
    rospy.loginfo("Upper arm norm threshold: %.2f", upper_arm_norm_threshold)

    # Constraint based on pointing sideways
    if left_arm_valid and left_upper_arm_norm < upper_arm_norm_threshold:
        rospy.loginfo("Rejecting left arm because of not pointing sideways ..")
        left_arm_valid = False
    if right_arm_valid and right_upper_arm_norm < upper_arm_norm_threshold:
        rospy.loginfo("Rejecting right arm because of not pointing sideways ..")
        right_arm_valid = False

    # Constraint based on parralelliness
    if left_arm_valid and left_arm_norm > arm_norm_threshold:
        rospy.loginfo("Rejecting left arm because of norm threshold ...")
        left_arm_valid = False
    if right_arm_valid and right_arm_norm > arm_norm_threshold:
        rospy.loginfo("Rejecting right arm because of norm threshold ...")
        right_arm_valid = False

    # Optimize
    frame = None
    if left_arm_valid and right_arm_valid:
        if left_arm_norm > right_arm_norm:
            rospy.loginfo("Right arm is pointing the most, using this one")
            frame = right_frame
        else:
            rospy.loginfo("Left arm is pointing the most, using this one")
            frame = left_frame
    if left_arm_valid:
        frame = left_frame
    if right_arm_valid:
        frame = right_frame

    if frame is None:
        rospy.loginfo("No valid arms found ...")
        return None

    return robot.head.ray_trace(kdl_conversions.kdlFrameStampedToPoseStampedMsg(frame))


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
        self._robot.head.look_at_standing_person()

        # Wait until a face has been detected near the robot
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if self._face_within_range(threshold=2.0):
                break
            else:
                rospy.loginfo("PointingDetector: waiting for someone to come into view")

        # Get RayTraceResult
        result = get_ray_trace_from_closest_person(robot=self._robot)

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
        assert result.intersection_point.header.frame_id in ["/map", "map"]
        raypos = kdl.Vector(result.intersection_point.point.x, result.intersection_point.point.y, 0.0)
        entities.sort(key=lambda e: e.distance_to_2d(raypos))
        self._designator.set_id(identifier=entities[0].id)
        rospy.loginfo("Object pointed at: {}".format(entities[0].id))
        return "succeeded"

    def _face_within_range(self, threshold):
        """ Gets the closest face. If the distance from the camera is too large, None will be returned.

        :param threshold: threshold fro mthe camera in meters
        :return: bool indicating if the closest face is within the threshold
        """
        raw_detections = self._robot.head.detect_faces()
        if not raw_detections:
            return None

        # Only take detections with operator
        detections = []
        for d in raw_detections:
            vs = self._robot.head.project_roi(roi=d.roi)
            detections.append((d, vs.vector.Norm()))

        # Sort the detectiosn
        detections = sorted(detections, key=lambda det: det[1])

        return detections[0][1] < threshold
