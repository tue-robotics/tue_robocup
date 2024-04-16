import rospy
from geometry_msgs.msg import PoseStamped
from smach import State
from std_msgs.msg import Header
from robot_smach_states.util.designators import is_writeable


class GetFurnitureFromOperatorPose(State):
    """
    Smach state to detect to what piece of furniture an operator is pointing at
    """

    def __init__(self, robot, entity_designator, all_possible_furniture=None):
        """
        Initialization

        :param robot: Robot API object
        :param all_possible_furniture: list with available furniture to be used
        """
        State.__init__(self, outcomes=['succeeded', 'failed'])

        is_writeable(entity_designator)

        self._robot = robot
        self.operator = None
        self.all_possible_furniture = all_possible_furniture
        self.entity_designator = entity_designator

    def execute(self, userdata=None):
        final_result = None
        while not rospy.is_shutdown() and final_result is None:
            if not self._prep_operator():  # output should be checked
                return 'failed'
            if self._get_operator() is not None:  # output should be checked
                return 'failed'
            result = None
            while not rospy.is_shutdown() and result is None and self.operator is not None:
                try:
                    map_pose = self._robot.tf_buffer.transform(PoseStamped(
                        header=Header(frame_id=self.operator.header.frame_id,
                                      stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                                      ),
                        pose=self.operator.pointing_pose
                    ), "map", timeout=rospy.Duration(1.0))
                    result = self._robot.ed.ray_trace(map_pose)  # type: RayTraceResponse
                except Exception as e:
                    rospy.logerr("Could not get ray trace from closest person: {}".format(e))
                rospy.sleep(1.0)

            rospy.loginfo("There is a ray intersection with {i} at ({p.x:.4}, {p.y:.4}, {p.z:.4})".format(
                i=result.entity_id, p=result.intersection_point.point))

            if self.all_possible_furniture:
                if result.entity_id in self.all_possible_furniture:
                    final_result = result
                else:
                    rospy.loginfo("{} is not furniture".format(result.entity_id))
                    self._robot.speech.speak("That's not furniture, you dummy.")
                    self.operator = None
                    return 'failed'
            else:
                final_result = result

        self.entity_designator.write(self._robot.ed.get_entity(final_result.entity_id))
        self._robot.speech.speak("You pointed at %s" % final_result.entity_id)
        return 'succeeded'

    def _show_image(self, timeout=5.0):
        # rgb, depth, depth_info = self._robot.perception.get_rgb_depth_caminfo()
        # self._robot.hmi.show_image_from_msg(rgb, timeout)
        image_data = self._robot.perception.get_rgb_depth_caminfo()
        if all(msg is not None for msg in image_data):
            self._robot.hmi.show_image_from_msg(image_data[0], timeout)
        else:
            rospy.logwarn("Could not get RGB, depth or camera info")
        return image_data

    def _prep_operator(self, attempts=10):
        self.operator = None
        self._robot.head.reset()
        self._robot.speech.speak("Let's point, please stand in front of me!", block=False)
        for i in range(attempts):
            image_data = self._show_image(timeout=1)
            if any(msg is None for msg in image_data):
                rospy.logwarn("Could not get RGB, depth and camera info")
                rospy.sleep(0.2)
                continue

            success, found_people_ids = self._robot.ed.detect_people(*image_data)
            if any(found_people_ids):
                break
            rospy.sleep(0.7)
            rospy.loginfo("Not found an operator yet")
        else:
            rospy.logwarn("Not found an operator at all")
            return False
        self._robot.speech.speak("Please point at a piece of furniture", block=False)
        self._show_image(timeout=1)
        self._robot.speech.speak("Three")
        self._show_image(timeout=1)
        self._robot.speech.speak("Two")
        self._show_image(timeout=1)
        self._robot.speech.speak("One")
        return True

    def _get_operator(self):
        while not rospy.is_shutdown() and self.operator is None:
            persons = self._robot.perception.detect_person_3d(*self._show_image())
            if persons:
                person = min(persons, key=lambda x: x.position.z)
                if self._verify_operator_pose(person):
                    self.operator = person
        rospy.loginfo("I see an operator at %.2f meter in front of me" % self.operator.position.z)
        return

    def _verify_operator_pose(self, person):
        if person.position.z > 2.5:
            self._robot.speech.speak("You're too far away, please get closer")
            return False
        if person.position.z < 1.5:
            self._robot.speech.speak("You're too close, please stand in my view with your full body")
            return False
        if "is_pointing" not in person.tags:
            self._robot.speech.speak("Please point with your arm stretched")
            return False
        return True
