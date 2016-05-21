import robot_skills.util.msg_constructors as msgs
import time
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from ed_perception.msg import PersonDetection
import rospy
from robocup_knowledge import load_knowledge
import math
import random

challenge_knowledge = load_knowledge("challenge_speech_recognition")


class RecognizePersons(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot

    def _shot_valid(self, number_of_people, operator_list, detections, operator):
        if not operator:
            return False

        if len(detections) < number_of_people:
            return False

        # Check if operator is consistent with operator list

        return True

    def _get_detections(self, external_api_request):
        z = 1.0
        self.robot.head.look_at_point(msgs.PointStamped(100, 0, z, self.robot.robot_name + "/base_link"))
        self.robot.head.wait_for_motion_done()
        time.sleep(1)

        detections = self.robot.ed.detect_persons(external_api_request=external_api_request)
        operator_candidates = [candidate for candidate in detections if candidate.name == "operator"]

        operator = None
        if operator_candidates:
            operator = max(operator_candidates, key=lambda c: c.name_score)

        return detections, operator

    def _recognize(self):
        self.robot.speech.speak("I am looking for my operator")

        # 1) Check how many people in the crowd
        shots = 5

        number_of_people = 0
        operator_list = []

        for i in range(0, shots):
            self.robot.speech.speak("%d" % i)
            detections, operator = self._get_detections(external_api_request=False)

            # Get number of people
            number_of_people = max(len(detections), number_of_people)

            # Get operator guess
            if operator:
                operator_list.append(operator)

        self.robot.speech.speak("I found %d people in the crowd!" % number_of_people)

        # 2) Get all other information with use of the external api, make sure that we have all persons here
        max_tries = 2
        try_number = 0

        best_operator = None
        best_detections = None
        while True:
            try_number += 1

            self.robot.speech.speak(random.choice(["Let's take a closer look",
                                                   "Let's see what we are dealing with",
                                                   "Let's get some more details"]))

            detections, operator = self._get_detections(external_api_request=True)
            if self._shot_valid(number_of_people, operator_list, detections, operator):
                return detections, operator

            if not best_detections or len(detections) > len(best_detections):
                best_operator = operator
                best_detections = detections

            if try_number > max_tries:
                self.robot.speech.speak("I am having trouble with my external servers, I will use my local result")
                return best_detections, best_operator

    def _describe_crowd(self, detections):
        num_females = 0
        num_males = 0

        for d in detections:
            if d.gender == 1:
                num_males += 1
            else:
                num_females += 1

        self.robot.speech.speak("There are %d males and %d females in the crowd" % (num_males, num_females))

    def _describe_operator(self, operator):
        if operator.gender == 1:
            gender = "male"
        else:
            gender = "female"

        z = operator.pose.pose.position.z
        pose_str = "standing"
        if z < 1.4:
            pose_str = "sitting"
        if z < 0.6:
            pose_str = "lying"

        self.robot.speech.speak("The operator is a %s" % gender)
        self.robot.speech.speak("The operator is %s" % pose_str)

        pose_base_link = self._robot.tf_listener.transformPose(target_frame=self._robot.robot_name + '/base_link', pose=operator.pose)
        x = pose_base_link.pose.position.x
        y = pose_base_link.pose.position.y

        self.robot.speech.speak("The operator is at x %.1f and y %.1f in my base link, I will turn towards you" % (x, y))
        th = math.atan2(y, x)
        vth = 0.5

        self.robot.head.cancel_goal()
        self.robot.base.force_drive(0, 0, vth, th / vth)

        self.robot.speech.speak("I will now point in your direction!")
        self.robot.rightArm.send_goal(0.5, -0.2, 0.9, 0, 0, 0, 60)

        self.robot.head.look_at_ground_in_front_of_robot(distance=100)

    def execute(self, userdata=None):

        detections, operator = self._recognize()
        if not detections or not operator:
            return "failed"

        self._describe_crowd(detections)

        self._describe_operator(operator)

        return 'succeeded'


class Detect(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        self.turned = False

        with self:

            @smach.cb_interface(outcomes=['done'])
            def wait_a_sec(userdata):
                robot.speech.speak("I will wait for 10 seconds for you to join the crowd", block=False)
                time.sleep(10)
                return 'done'

            smach.StateMachine.add('WAIT_FOR_OPERATOR_TO_JOIN',
                                   smach.CBState(wait_a_sec),
                                   transitions={'done': 'FORCE_DRIVE'})

            @smach.cb_interface(outcomes=['done'])
            def force_drive(userdata):
                if not self.turned:
                    vth = 0.5
                    th = 3.1415
                    robot.head.cancel_goal()
                    robot.base.force_drive(0, 0, vth, th / vth)
                    self.turned = True
                return 'done'

            smach.StateMachine.add('FORCE_DRIVE',
                                   smach.CBState(force_drive),
                                   transitions={'done': 'SAY_SEARCHING_CROWD'})

            smach.StateMachine.add('SAY_SEARCHING_CROWD',
                                    states.Say(robot, "I'm looking for the crowd.", block=False),
                                    transitions={'spoken': 'RECOGNIZE_PERSONS'})

            smach.StateMachine.add('RECOGNIZE_PERSONS', RecognizePersons(robot),
                                   transitions={'succeeded': 'succeeded', 'failed': 'failed'})
