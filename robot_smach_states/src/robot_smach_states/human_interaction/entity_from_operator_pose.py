import rospy
from geometry_msgs.msg import PoseStamped
from smach import State
from std_msgs.msg import Header


class GetFurnitureFromOperatorPose(State):
    """
    Smach state to detect to what piece of furniture an operator is pointing at
    """

    def __init__(self, robot, knowledge):
        """" Initialization

        :param robot: Robot API object
        :param knowledge: available knowledge to be used for the furniture
        """
        State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self.operator = None
        self.all_possible_furniture = knowledge.location_names

    def execute(self, userdata=None):
        rospy.loginfo("I entered the execution state!")
        final_result = None
        while not rospy.is_shutdown() and final_result is None:
            self._prep_operator()  # output should be checked
            if self._get_operator() is not None:  # output should be checked
                return 'failed'
            result = None
            while not rospy.is_shutdown() and result is None and self.operator is not None:
                try:
                    map_pose = self._robot.tf_listener.transformPose("map", PoseStamped(
                        header=Header(
                            frame_id=self.operator.header.frame_id,
                            stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                        ),
                        pose=self.operator.pointing_pose
                    ))
                    result = self._robot.ed.ray_trace(map_pose)  # type: RayTraceResponse
                except Exception as e:
                    rospy.logerr("Could not get ray trace from closest person: {}".format(e))
                rospy.sleep(1.0)

            rospy.loginfo("There is a ray intersection with {i} at ({p.x:.4}, {p.y:.4}, {p.z:.4})".format(
                i=result.entity_id, p=result.intersection_point.point))

            if result.entity_id in self.all_possible_furniture:
                final_result = result
            else:
                rospy.loginfo("{} is not furniture".format(result.entity_id))
                self._robot.speech.speak("That's not furniture, you dummy.")
                rospy.sleep(3)
                self.operator = None
                self._robot.get_arm().send_joint_goal("reset")
                return 'failed'

        self._robot.speech.speak("You pointed at %s" % final_result.entity_id)
        self._robot.get_arm().send_joint_goal("reset")
        return 'succeeded'

    def _show_image(self, timeout=5.0):  # could also use /hero/openpose/result_image as rgb (same msg type)
        rgb, depth, depth_info = self._robot.perception.get_rgb_depth_caminfo()
        self._robot.hmi.show_image_from_msg(rgb, timeout)
        return rgb, depth, depth_info

    def _prep_operator(self):
        self.operator = None

        self._robot.ed.reset()
        self._robot.head.reset()
        self._robot.speech.speak("Let's point, please stand in front of me!", block=False)
        for x in range(0, 4):
            self._show_image(timeout=2)
            rospy.sleep(0.4)

        self._robot.speech.speak("Please point at a piece of furniture", block=False)
        self._show_image(timeout=1)
        self._robot.speech.speak("Three")
        self._show_image(timeout=1)
        self._robot.speech.speak("Two")
        self._show_image(timeout=1)
        self._robot.speech.speak("One")

        return

    def _get_operator(self):
        while not rospy.is_shutdown() and self.operator is None:
            try:
                persons = self._robot.perception.detect_person_3d(*self._show_image())
            except:
                return 'failed'
                # failures should be caught
            if persons:
                persons = sorted(persons, key=lambda x: x.position.z)
                person = persons[0]
                if self._is_operator(person):
                    self.operator = person

        # robot.speech.speak("I see an operator at %.2f meter in front of me" % OPERATOR.position.z)
        rospy.loginfo("I see an operator at %.2f meter in front of me" % self.operator.position.z)

        return

    def _is_operator(self, person):
        if person.position.z > 2.5:
            self._robot.speech.speak("You're too close, please stand in my view with your full body")
            return False

        if person.position.z < 1.5:
            self._robot.speech.speak("You're too far away, please get closer")
            return False

        if "is_pointing" not in person.tags:
            self._robot.speech.speak("Please point with your arm stretched")
            return False

        return True
