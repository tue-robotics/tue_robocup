#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup

from speech_interpreter.srv import AskUser # for speech_to_text only

from psi import *

###########################
# Created by: Erik Geerts #
###########################

class AskQuestions(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.ask_user_service_questions = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        rospy.loginfo("----Possible questions for now: -----------------")
        rospy.loginfo("--- What is the capital of Germany? -------------")
        rospy.loginfo("--- What is the heaviest animal in the world?----")
        rospy.loginfo("--- Who is the president of America?-------------")
        rospy.loginfo("--- Who is your example?-------------------------")
        rospy.loginfo("--- When do the olympics start?------------------")
        rospy.loginfo("--- Which football club is the best?-------------")
        rospy.loginfo("--- Who is the best looking person around here?--")
        rospy.loginfo("--- Which person is not able to say yes?---------")
        rospy.loginfo("--- Which town has been bombed?------------------")
        rospy.loginfo("--- What is your motto?--------------------------")


        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
        self.response = self.ask_user_service_questions("questions", 10 , rospy.Duration(60))

        if self.response.keys[0] == "answer":
            return "succeeded"
        else:
            return "failed"

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['detected','waiting','timed_out','failed'])
        self.robot = robot

        self.counter = 0
    
    def execute(self, userdata=None):

        if self.counter > 3:
            self.robot.head.look_up(tilt_vel=0.75)
            return 'timed_out'

        self.robot.spindle.reset()
        self.robot.head.set_pan_tilt(tilt=-0.2)
        
        if self.counter == 0:
            rospy.sleep(1.5)        

        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.logerr("Face segmentation failed to start")
            self.robot.head.look_up(tilt_vel=0.75)
            return "failed"

        wait_machine = states.Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            self.robot.lights.set_color(1, 0, 0)
            rospy.sleep(0.1)  
            self.robot.lights.set_color(0, 0, 1)
            rospy.loginfo("Failed stopping face segmentation")

        if wait_result == "timed_out":
            self.counter = self.counter + 1
            return "waiting"

        elif wait_result == "query_true":
            answers = self.robot.reasoner.query(query_detect_person)
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]
            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.high()
                rospy.logdebug("Spindle should come up now!")

            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point,timeout=0)

            return 'detected'


class WhatDidYouSay(smach.StateMachine):

    def __init__(self, robot=None):
        smach.StateMachine.__init__(self, outcomes=["Done"])
        self.robot = robot

        rospy.loginfo("-------------------- WHAT DIT YOU SAY ---------------------")
        rospy.loginfo("-----------------------------------------------------------")
        rospy.loginfo("------------ MAKE SURE YOU ADDED THE SENTENCES ------------")
        rospy.loginfo("-------------- speech_interpreter.pl in map ---------------")
        rospy.loginfo("----------- tue_reasoner/tue_knowledge/prolog -------------")
        rospy.loginfo("-------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -----------")
        rospy.loginfo("-----------------------------------------------------------")
            
        with self:


            smach.StateMachine.add("WAIT_FOR_PERSON",
                                    WaitForPerson(robot),
                                    transitions={'detected':'SAY_FIRST_QUESTION',
                                                 'waiting':'WAIT_FOR_PERSON',
                                                 'timed_out':'SAY_FIRST_QUESTION',
                                                 'failed':'SAY_FIRST_QUESTION'})


            smach.StateMachine.add("SAY_FIRST_QUESTION",
                                    states.Say(robot,"Hello! What is your first question?"),
                                    transitions={'spoken':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("ASK_FIRST_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_SECOND_QUESTION',
                                                 'failed':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("SAY_SECOND_QUESTION",
                                    states.Say(robot,"What is your second question?"),
                                    transitions={'spoken':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("ASK_SECOND_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_THIRD_QUESTION',
                                                 'failed':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("SAY_THIRD_QUESTION",
                                    states.Say(robot,"What is your third question?"),
                                    transitions={'spoken':'ASK_THIRD_QUESTION'})

            smach.StateMachine.add("ASK_THIRD_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'Done',
                                                 'failed':'ASK_THIRD_QUESTION'})

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    startup(WhatDidYouSay)