import PyKDL as kdl
import rospy
import smach
from ed_sensor_integration.srv import RayTraceResponse
from robot_skills.util import kdl_conversions
from robot_smach_states.util.designators import Designator
from robot_smach_states.util.startup import startup


def get_frame_from_vector(x_vector, origin):
    unit_z = kdl.Vector(0, 0, 1)
    unit_z_cross_diff = (unit_z * x_vector) / (unit_z * x_vector).Norm()
    y_vector = x_vector * unit_z_cross_diff
    z_vector = x_vector * y_vector

    rotation = kdl.Rotation(x_vector, y_vector, z_vector)
    translation = origin.vector

    frame_stamped = kdl_conversions.FrameStamped(kdl.Frame(rotation, translation), origin.frame_id)
    return frame_stamped


def get_ray_trace_from_closest_person_dummy(robot,
                                            arm_norm_threshold=0.3,
                                            upper_arm_norm_threshold=0.7,
                                            entity_id=None,
                                            operator_pos=None,
                                            furniture_pos=None):
    try:
        operator_vec = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(operator_pos)
        furniture_vec = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(furniture_pos)

        diff = (furniture_vec.vector - operator_vec.vector) / (furniture_vec.vector - operator_vec.vector).Norm()
        ray_trace_frame = get_frame_from_vector(diff, operator_vec)

        # This is purely for visualization
        robot.ed.ray_trace(kdl_conversions.kdl_frame_stamped_to_pose_stamped_msg(ray_trace_frame))
    except Exception as e:
        rospy.logerr(e)
        pass

    res = RayTraceResponse()
    res.entity_id = entity_id
    res.intersection_point = furniture_pos

    return res


def get_ray_trace_from_closest_person(robot, arm_norm_threshold=0.1, upper_arm_norm_threshold=0.7):
    persons = robot.head.detect_persons_3d()  # deprecated
    valid_persons = [person for person in persons if "right_shoulder" in person]

    if not valid_persons:
        return None

    person = sorted(valid_persons, key=lambda x: x["right_shoulder"].point.z)[0]

    # Check if arms are pointing
    left_arm_valid = "left_wrist" in person and "left_elbow" in person and "left_shoulder" in person
    right_arm_valid = "right_wrist" in person and "right_elbow" in person and "right_shoulder" in person

    if left_arm_valid:
        left_wrist = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["left_wrist"]).projectToFrame("/map",
                                                                                                              robot.tf_listener)
        left_elbow = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["left_elbow"]).projectToFrame("/map",
                                                                                                              robot.tf_listener)
        left_shoulder = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["left_shoulder"]).projectToFrame(
            "/map",
            robot.tf_listener)
        left_lower_arm_vector = (left_wrist.vector - left_elbow.vector) / (left_wrist.vector - left_elbow.vector).Norm()
        left_upper_arm_vector = (left_elbow.vector - left_shoulder.vector) / (
            left_elbow.vector - left_shoulder.vector).Norm()
        left_frame = get_frame_from_vector(left_lower_arm_vector, left_wrist)
        left_arm_norm = (left_lower_arm_vector * left_upper_arm_vector).Norm()
        left_upper_arm_norm = (left_upper_arm_vector * kdl.Vector(0, 0, 1)).Norm()

        rospy.loginfo("Left arm norm: %.2f", left_arm_norm)
        rospy.loginfo("Upper left arm norm: %.2f", left_upper_arm_norm)
    else:
        rospy.loginfo("Left arm not valid because it does not contain all required bodyparts")

    if right_arm_valid:
        right_wrist = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["right_wrist"]).projectToFrame("/map",
                                                                                                                robot.tf_listener)

        right_elbow = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["right_elbow"]).projectToFrame("/map",
                                                                                                                robot.tf_listener)

        right_shoulder = kdl_conversions.kdl_vector_stamped_from_point_stamped_msg(person["right_shoulder"]).projectToFrame(
            "/map",
            robot.tf_listener)

        right_lower_arm_vector = (right_wrist.vector - right_elbow.vector) / (
            right_wrist.vector - right_elbow.vector).Norm()
        right_upper_arm_vector = (right_elbow.vector - right_shoulder.vector) / (
            right_elbow.vector - right_shoulder.vector).Norm()

        right_frame = get_frame_from_vector(right_lower_arm_vector, right_wrist)
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

    return robot.ed.ray_trace(kdl_conversions.kdl_frame_stamped_to_pose_stamped_msg(frame))


class PointingDetector(smach.State):
    """ State to fill an EdEntityDesignator depending on the direction of an operator pointing at the entity
     """

    def __init__(self, robot, designator, default_entity_id, super_type="furniture"):
        """ Constructor
        :param robot: robot object
        :param designator: Pointing designator to fill
        :param default_entity_id: entity id for fallback scenario
        :param super_type: string indicating the required super type. If the RayTraceResult does not provide an entity
        of the desired super type, the closest entity with that desired supertype will be computed.
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        self._designator = designator
        self._super_type = super_type
        self._default_entity_id = default_entity_id

    def execute(self, userdata=None):

        # Point head in the right direction (look at the ground 100 m in front of the robot
        self._robot.head.look_at_ground_in_front_of_robot(distance=100.0)

        # Wait until a face has been detected near the robot
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            in_range, face_pos = self._face_within_range(threshold=2.0)
            if in_range:
                break
            else:
                rospy.loginfo("PointingDetector: waiting for someone to come into view")

        self._robot.speech.speak("Hi there", block=True)
        self._robot.lights.set_color(r=1.0, g=0.0, b=0.0, a=1.0)

        # Get RayTraceResult
        start = rospy.Time.now()
        result = None
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 10.0:
            try:
                result = get_ray_trace_from_closest_person(robot=self._robot)
                if result is not None:
                    break
            except Exception as e:
                rospy.logerr("Could not get ray trace from closest person: {}".format(e))

        # If result is None: fallback scenario
        # result = RayTraceResponse()
        # result.entity_id = self._default_entity_id
        if result is None:
            result = RayTraceResponse()
            result.entity_id = self._default_entity_id
            face_pos_map = face_pos.projectToFrame("map", self._robot.tf_listener)
            face_pos_msg = kdl_conversions.kdl_vector_stamped_to_point_stamped(face_pos_map)

            e = self._robot.ed.get_entity(id=self._default_entity_id)
            from geometry_msgs.msg import PointStamped
            e_pos_msg = PointStamped()
            e_pos_msg.header.frame_id = "map"
            e_pos_msg.header.stamp = rospy.Time.now()
            e_pos_msg.point.x = e._pose.p.x()
            e_pos_msg.point.y = e._pose.p.y()
            e_pos_msg.point.z = e._pose.p.z()

            result = get_ray_trace_from_closest_person_dummy(robot=self._robot,
                                                             arm_norm_threshold=0.1,
                                                             upper_arm_norm_threshold=0.7,
                                                             entity_id=self._default_entity_id,
                                                             operator_pos=face_pos_msg,
                                                             furniture_pos=e_pos_msg)

        self._robot.lights.set_color(r=0.0, g=0.0, b=1.0, a=1.0)

        # Query the entity from ED
        entity = self._robot.ed.get_entity(id=result.entity_id)

        # If the result is of the desired super type, we can try to do something with it
        if entity.is_a(self._super_type):
            self._designator.set_id(identifier=entity.id)
            rospy.loginfo("Object pointed at: {}".format(entity.id))
            self._robot.speech.speak("Okay, here we go")
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
        self._robot.speech.speak("Okay, here we go")
        return "succeeded"

    def _face_within_range(self, threshold):
        """ Gets the closest face. If the distance from the camera is too large, None will be returned.

        :param threshold: threshold fro mthe camera in meters
        :return: tuple with bool indicating if the closest face is within the threshold and vectorstamped of the face location
        """
        # ToDo: catch the exceptiono in the detect_faces and return an empty list
        try:
            raw_detections = self._robot.perception.detect_faces()
        except Exception as e:
            raw_detections = None

        if not raw_detections:
            return False, None

        # Only take detections with operator
        detections = []
        for d in raw_detections:
            try:
                vs = self._robot.perception.project_roi(roi=d.roi)
            except Exception as e:
                rospy.logwarn("ROI Projection failed: {}".format(e))
                continue
            detections.append((d, vs))

        # Sort the detectiosn
        detections = sorted(detections, key=lambda det: det[1].vector.Norm())

        return detections[0][1].vector.Norm() < threshold, detections[0][1]


def setup_state_machine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    pointing_des = Designator()

    with sm:
        smach.StateMachine.add('DETECT_POINTING',
                               PointingDetector(robot, pointing_des, ""),
                               transitions={'succeeded': 'Done',
                                            'failed': 'Aborted'})
    return sm


if __name__ == "__main__":
    rospy.init_node('state_machine')

    startup(setup_state_machine)
