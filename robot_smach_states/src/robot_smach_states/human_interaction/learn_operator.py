# System
import random

# ROS
import rospy
import smach

# TU/e
from robot_skills.util import kdl_conversions


class LearnOperator(smach.State):
    def __init__(self, robot, operator_timeout=20, learn_person_timeout=10.0, detection_threshold=6):
        """ Constructor

        N.B.: human_interaction.LearnPerson might also fit your needs

        :param robot: robot object (amigo, sergio)
        :param operator_timeout: maximum time to locate a possible operator
        :param learn_person_timeout: maximum time it is allowed to take to learn an operator
        :param detection_threshold: amount of detections needed before an operator is confirmed
        """
        smach.State.__init__(self, outcomes=['done', 'failed', 'aborted'],
                             input_keys=['operator_learn_in'],
                             output_keys=['operator_learn_out'])
        self._robot = robot
        self._operator_timeout = operator_timeout
        self._learn_person_timeout = learn_person_timeout
        self._operator_name = "operator"
        self._detection_threshold = detection_threshold
        random.seed()

    def execute(self, userdata):
        """ In this function an operator is located and their face is learned

        :param userdata: contains operator which is initially none, for global definition purposes
        :return: failed in case something went wrong
                done if an operator is found
                aborted if preempt is requested
        """

        start_time = rospy.Time.now()
        self._robot.head.look_at_standing_person()
        operator = userdata.operator_learn_in
        while not operator:
            r = rospy.Rate(1.0)
            if self.preempt_requested():
                return 'aborted'

            if(rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return 'failed'

            operator = self._robot.ed.get_closest_laser_entity(
                radius=0.5,
                center_point=kdl_conversions.VectorStamped(x=1.0, y=0, z=1,
                                                           frame_id="/{}/base_link".format(self._robot.robot_name)))
            rospy.loginfo("Operator: {op}".format(op=operator))
            if not operator:
                options = ["Please stand in front of me.",
                           "My laser can't see you, please get closer.",
                           "Where are you? Please get closer."]
                sentence = random.choice(options)
                self._robot.speech.speak(sentence)
            else:
                self._robot.speech.speak("Please look at me while I learn to recognize you.",
                                         block=False)
                self._robot.head.look_at_standing_person()
                learn_person_start_time = rospy.Time.now()
                detection_counter = 0
                while detection_counter < self._detection_threshold:
                    if self._robot.perception.learn_person(self._operator_name):
                        rospy.loginfo("Successfully detected you %i times" % (detection_counter + 1))
                        detection_counter += 1
                    elif (rospy.Time.now() - learn_person_start_time).to_sec() > self._learn_person_timeout:
                        self._robot.speech.speak("Please stand in front of me and look at me")
                        operator = None
                        break
            r.sleep()
        rospy.loginfo("We have a new operator: %s" % operator.id)
        self._robot.speech.speak("Who is that handsome person? Oh, it is you!", mood='Excited')
        self._robot.speech.speak(
            "I will follow you now, please say , {}, stop , when we are at the car.".format(self._robot.robot_name))
        self._robot.head.close()
        userdata.operator_learn_out = operator
        return 'done'
