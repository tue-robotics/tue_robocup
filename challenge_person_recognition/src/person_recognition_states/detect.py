import robot_skills.util.msg_constructors as msgs
from robot_skills.util.kdl_conversions import VectorStamped
import time
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
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
        z = 1.5
        self.robot.head.look_at_point(VectorStamped(100, 0, z, self.robot.robot_name + "/base_link"))
        self.robot.head.wait_for_motion_done()
        time.sleep(1)

        rospy.logerr("ed.detect _persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
        rospy.logerr("I will return an empty detection list!")
        detections = []

        if not detections:
            detections = []

        operator_candidates = [candidate for candidate in detections if candidate.name == "operator"]

        rospy.loginfo("Detections: %s", detections)
        rospy.loginfo("Operator candidates: %s", operator_candidates)

        operator = None
        if operator_candidates:
            operator = max(operator_candidates, key=lambda c: c.name_score)

        rospy.loginfo("Operator: %s", operator)

        return detections, operator

    def _recognize(self):
        z = 1.5
        self.robot.head.look_at_point(VectorStamped(100, 0, z, self.robot.robot_name + "/base_link"))
        self.robot.speech.speak("I am looking for my operator", block=False)

        # 1) Check how many people in the crowd
        shots = 3

        number_of_people = 0
        operator_list = []

        sentences = ["You are all looking great today!            Keep looking in my camera!", "I like it when everybody is staring at me; being in the center of attention!"]

        for i in range(0, shots):
            self.robot.speech.speak(sentences[i % (shots - 1)], block=False)
            detections, operator = self._get_detections(external_api_request=False)

            # Get number of people
            number_of_people = max(len(detections), number_of_people)

            # Get operator guess
            if operator:
                operator_list.append(operator)

        # 2) Get all other information with use of the external api, make sure that we have all persons here
        max_tries = 5
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
                self.robot.speech.speak("I am having trouble with my external servers, I will use my local result") # we just didnt pass our criteria
                return best_detections, best_operator

    def _describe_crowd(self, detections):
        num_females = 0
        num_males = 0
        num_ppl = 0

        has_additional_information = False

        for d in detections:
            num_ppl += 1
            if d.gender_score:
                has_additional_information = True
                if d.gender == 1:
                    num_males += 1
                else:
                    num_females += 1
            else:
                num_males += 1

        if not has_additional_information:
            self.robot.speech.speak("Unfortunately, I could not access the internet for better classification.")

        self.robot.speech.speak("I found %d people in the crowd" % num_ppl)
        self.robot.speech.speak("There are %d males and %d females in the crowd" % (num_males, num_females))

    def _describe_operator(self, operator):
        gender = "male"
        if operator.gender == 2:
            gender = "female"

        z = operator.pose.pose.frame.p.z()
        pose_str = "standing"
        if z < 1.4:
            pose_str = "sitting"
        if z < 0.6:
            pose_str = "lying"

        self.robot.speech.speak("The operator is a %s" % gender)
        self.robot.speech.speak("The operator is %s" % pose_str)

        pose_base_link = operator.pose.projectToFrame(self.robot.robot_name + '/base_link', self.robot.tf_listener)
        x = pose_base_link.pose.frame.p.x()
        y = pose_base_link.pose.frame.p.y()

        self.robot.speech.speak("The operator is at x %.1f and y %.1f in my base link, I will turn towards you" % (x, y))
        th = math.atan2(y, x)
        vth = 0.5

        self.robot.head.cancel_goal()
        self.robot.base.force_drive(0, 0, math.copysign(1, th) * vth, abs(th / vth))

        self.robot.speech.speak("I will now point in your direction!")

        self.robot.head.look_at_ground_in_front_of_robot(distance=100)
        self.robot.rightArm.send_joint_trajectory("point_to_operator")

        self.robot.speech.speak("You are right there operator!")

        self.robot.speech.speak("Cook cook!")

        self.robot.rightArm.reset()
        self.robot.rightArm.wait_for_motion_done()

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

            smach.StateMachine.add('SAY_SEARCHING_CROWD',
                                    states.Say(robot, "I'm looking for the crowd.", block=False),
                                    transitions={'spoken': 'RECOGNIZE_PERSONS'})

            smach.StateMachine.add('RECOGNIZE_PERSONS', RecognizePersons(robot),
                                   transitions={'succeeded': 'succeeded', 'failed': 'failed'})
