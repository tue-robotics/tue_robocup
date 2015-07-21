#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_gpsr')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach
import sys

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import *
from robot_skills.util import msg_constructors as geom
import ed.msg

from datetime import datetime, date, timedelta

from robot_smach_states.util.geometry_helpers import *
from cb_planner_msgs_srvs.msg import PositionConstraint

from visualization_msgs.msg import Marker, MarkerArray

#import data
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')

class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.ed.configure_kinect_segmentation(continuous=False)
        #self.robot.ed.configure_perception(continuous=False)
        self.robot.ed.disable_plugins(plugin_names=["laser_integration"])

        return "done"

class FakeShutdownRobot(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()
        self.robot.spindle.low()
        self.robot.head.look_at_ground_in_front_of_robot(1)
        self.robot.lights.set_color(0,0,0,1)

        return "done"

class FakeStartupRobot(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()
        self.robot.spindle.reset()
        self.robot.head.reset()
        
        dx = 0.01
        x = 0.02
        for i in range(0,100):
            self.robot.lights.set_color(0,0,1,dx*i)
            rospy.sleep(x)

        return "done"


class WaitForEntity(smach.State):

    def __init__(self, robot, ed_entity_name):
        smach.State.__init__(self, outcomes=['entity_exists'])
        self.robot = robot
        self.ed_entity_name = ed_entity_name

    def execute(self, userdata=None):

        while not self.robot.ed.get_entity(id=self.ed_entity_name):
            rospy.sleep(0.2)

        return "entity_exists"


class Sleep(smach.State):
    def __init__(self, robot,sleep=1):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        self.sleep = sleep

    def execute(self, userdata=None):

        rospy.sleep(self.sleep)

        return "done"

class ForceDriveToTheRight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata):

        self.robot.base.force_drive(0, -0.5, 0, 2.0)    

        return "done"


class Ask_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What can I do for you?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.spec1_amigo_task, choices=challenge_knowledge.choices1_amigo_task, time_out=rospy.Duration(30))
        self.robot.head.cancel_goal()
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                say_result_filter_me = self.replace_word(res.result,"me","you")
                say_result = self.replace_word(say_result_filter_me,"your","my")
                self.robot.speech.speak("Okay I will {0}".format(say_result))
               # self.robot.speech.speak("Is that correct?")



                print say_result
                self.save_action(res)



            else:
                self.robot.speech.speak("Sorry, I did not hear you. Please come closer to me if you can.")
                return "failed"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "done"


def setup_statemachine(robot):

    robot.reasoner.load_database("challenge_gpsr","prolog/prolog_data.pl")
    robot.reasoner.query("retractall(current_action(_))")
    robot.reasoner.query("retractall(action_info(_,_,_))")

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
    arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])

    with sm:


        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INIT_WM',
                                                'abort':'Aborted'})

        smach.StateMachine.add( "INIT_WM",
                                    InitializeWorldModel(robot),
                                    transitions={'done'    :'FAKESHUTDOWN'})

        ## ?? TODO WHICH SENSORS FOR ED SHOULD BE

        smach.StateMachine.add('FAKESHUTDOWN',
                                    FakeShutdownRobot(robot),
                                    transitions={   'done':'TEMPORARY_SLEEP'})


        # WAIT FOR TRIGGER NOT USED ANYMORE, but just in case..
        # smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
        #                             states.WaitForTrigger(robot,['amigo_startup'],topic="/"+robot.robot_name+"/trigger"),
        #                             transitions={   'amigo_startup'        :'FAKESTARTUP',
        #                                             'preempted'            :'FAKESTARTUP'})




        #TODO: add entity
        # smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
        #                             WaitForEntity(robot,ed_entity_name='walls2'),
        #                             transitions={   'entity_exists'        :'FAKESTARTUP'})


        smach.StateMachine.add('TEMPORARY_SLEEP',
                                    Sleep(robot,7),
                                    transitions={   'done':'FAKESTARTUP'})
    


        smach.StateMachine.add('FAKESTARTUP',
                                    FakeStartupRobot(robot),
                                    transitions={   'done':'FORCEDRIVE_TO_THE_RIGHT'})

        smach.StateMachine.add('FORCEDRIVE_TO_THE_RIGHT',
                                    ForceDriveToTheRight(robot),
                                    transitions={   'done':'SET_INITIAL_POSE_TEST'})

        smach.StateMachine.add( "SET_INITIAL_POSE_TEST",
                                    states.Say(robot, ["Only for testing, now initial pose is needed. Probably a crash ;)"], block=True),
                                    transitions={   'spoken'            :'SET_INITIAL_POSE'})


        smach.StateMachine.add('SET_INITIAL_POSE',
                                states.SetInitialPose(robot, challenge_knowledge.initial_pose_amigo),
                                transitions={   'done'          :'Done',
                                                'preempted'     :'Done',
                                                'error'         :'Done'})


        ######################################################
        ##################### ASK STATE  #####################             
        ######################################################














    return sm



if __name__ == "__main__":
    rospy.init_node('amigo_final_rwc2015_exec')
    rospy.loginfo("-----------------------------------------------------------------")
    rospy.loginfo("----------------------- FINAL CHINA 2015 ------------------------")
    rospy.loginfo("----------------------------- AMIGO -----------------------------")
    rospy.loginfo("-----------------------------------------------------------------")
        
    # if len(sys.argv) > 1:
    #     robot_name = sys.argv[1]
    #     ROBOT_NAME_SPECIAL = robot_name
    # else:
    #     print "[CHALLENGE FINAL] Please provide robot name as argument."
    #     exit(1)

    #rospy.sleep(1)
    #states.util.startup(setup_statemachine, robot_name=robot_name)
    states.util.startup(setup_statemachine)