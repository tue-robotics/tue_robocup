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

import timer_states as timer

#import data
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')

GRASP_LOC = []
PERSON_LOC = []
NUMBER_OF_TRIES = 0

###### Janno ######
# Driving to the operator (Sjoerd)
BAR_DESIGNATOR = None #EdEntityDesignator(robot=robot, id='rwc2015/bar-0')
BAR_TYPE_DESIGNATOR = None #Designator(initial_value='bar', resolve_type=str) # Designator that returns a string with the bar type
OPERATOR_DESIGNATOR = None #PersonDesignator(robot=robot, furniture_designator=bar_type_designator)      # Designator that returns the operator

# Driving to the operator (Sjoerd)
BED_DESIGNATOR = None #VariableDesignator(resolve_type=EntityInfo)
BED_TYPE_DESIGNATOR = None #VariableDesignator(resolve_type=str) # Designator that returns a string with the bar type
LUIS_DESIGNATOR = None #PersonDesignator(robot=robot, furniture_designator=bed_type_designator)      # Designator that returns the operator
###################

class Initialize(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['initialized',
                                             'abort'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.lights.set_color(0,0,0)  #be sure lights are blue

        self.robot.head.reset()
        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close',0.0)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close',0.0)
        self.robot.torso.reset()

        ## Check if TF link between /map and /base_link is set, if not error at initialize in stead of during first navigate execution
        rospy.loginfo("TF link between /map and /base_link is checked. If it takes longer than a second, probably an error. Do a restart!!!")
        self.robot.base.get_location()

        return 'initialized'

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
        for i in range(0,20):
            self.robot.lights.set_color(0,0,1,1)
            rospy.sleep(dx*i)
            self.robot.lights.set_color(0,0,0,1)
            rospy.sleep(dx*i)

        self.robot.lights.set_color(0,0,1,1)
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

class WaitForEntityInitialPose(smach.State):

    def __init__(self, robot, ed_entity_name):
        smach.State.__init__(self, outcomes=['entity_exists','initial_pose_to_be_set_manually'])
        self.robot = robot
        self.ed_entity_name = ed_entity_name

    def execute(self, userdata=None):

        for i in range (0,100):
            if not self.robot.ed.get_entity(id=self.ed_entity_name):
                print "Initial pose not known"
                rospy.sleep(0.1)
            else:
                print "Initial pose known!!"
                return "entity_exists"

        return "initial_pose_to_be_set_manually"

                


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


class AskAction(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["drive_near_loc_for_person", "grasp_object","time_asked_for","no_action", "failed"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What can I do for you?")

        result = self.robot.ears.recognize(spec=challenge_knowledge.spec1_amigo_task, choices=challenge_knowledge.choices1_amigo_task, time_out=rospy.Duration(60))
        self.robot.head.cancel_goal()

        try:
            if result:
                if "name" in result.choices:
                    return "drive_near_loc_for_person"

                elif "grasp_location" in result.choices:
                    say_result_filter_me = self.replace_word(result.result,"me","you")
                    self.robot.speech.speak("Okay I will {0}".format(say_result_filter_me))
                    GRASP_LOC.append(result.choices['grasp_location'])
                    print "GRASP_LOC = ", GRASP_LOC
                    return "grasp_object"

                elif "time" in result.choices:
                    return "time_asked_for"
            else:
                self.robot.speech.speak("Sorry, I did not hear you.")
                return "no_action"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "no_action"

    def replace_word(self,string,word_in,word_out):
        try:
            if string[:(len(word_in)+1)] == (word_in+" "):
                string = string.replace(string[:len(word_in)],word_out)

            if string[(len(string)-len(word_in)-1):] == (" "+word_in):
                string = string.replace(string[(len(string)-len(word_in)):],word_out)

            string = string.replace(" "+word_in+" "," "+word_out+" ")

        except KeyError:
            print "[gpsr] Received action is to short."

        return string


class AskPersonLoc(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["drive_near_loc_for_person", "no_action", "no_multiple_actions_heard", "failed"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("Where can I find him?")
        result = self.robot.ears.recognize(spec=challenge_knowledge.spec2_amigo_task_followup, choices=challenge_knowledge.choices2_amigo_task_followup, time_out=rospy.Duration(15))
        self.robot.head.cancel_goal()

        try:
            if result:
                if "location" in result.choices:
                    self.robot.speech.speak("Okay I will find him near the {0}".format(result.choices['location']))
                    print "PERSON_LOC = ", PERSON_LOC
                    PERSON_LOC.append(result.choices['location'])

                    # Query location to create the entity
                    e = self.robot.ed.get_entity(id='rwc2015/' + loc + '-0')
                    if not e: 
                        return failed
                    BED_DESIGNATOR.current = e
                    BED_TYPE_DESIGNATOR.current = result.choices['location']
                    return "drive_near_loc_for_person"
                else:
                    self.robot.speech.speak("Sorry, I did not understand you.")
            else:
                self.robot.speech.speak("Sorry, I did not hear you.")
                return "no_action"
        except KeyError:
            print "[FINAL] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        NUMBER_OF_TRIES = NUMBER_OF_TRIES + 1

        if NUMBER_OF_TRIES > 3:
            NUMBER_OF_TRIES = 0
            return "no_multiple_actions_heard"
        else:
            return "no_action"

###### Janno ######
class PersonDesignator(Designator):
    """ Designator that can be used to drive to a person near a furniture object 
    :param robot robot
    :furniture_designator Designator with resolve type string
    """
    def __init__(self, robot, furniture_designator):
        # ToDo: maybe an EdEntityDesignator is more convenient than a string designator
        super(PersonDesignator, self).__init__(resolve_type=EntityInfo)
        self._robot = robot
        self._furniture_designator = furniture_designator

    def resolve(self):
        # Get furniture entity
        furniture_id = self._furniture_designator.resolve()
        furniture_id = 'rwc2015/' + furniture_id + '-0`'
        entities = self._robot.ed.get_entities()
        f = None
        for e in entities:
            if e.id == furniture_id:
                f = e

        if not f:
            rospy.logwarn('Entity with id {0} not found'.format(furniture_id))
            return None

        person = self._robot.ed.get_closest_possible_person_entity(type="possible_human", center_point=f.pose.position, radius=10)
        if not person:
            rospy.logwarn('No person found near the {0}'.format(furniture_id))
            return None
        else:
            return person
###################

def setup_statemachine(robot):

    robot.reasoner.load_database("challenge_gpsr","prolog/prolog_data.pl")
    robot.reasoner.query("retractall(current_action(_))")
    robot.reasoner.query("retractall(action_info(_,_,_))")

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
    arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])

    ###### Janno ######
    # Driving to the operator (Sjoerd)
    bar_designator = EdEntityDesignator(robot=robot, id='rwc2015/bar-0')
    bar_type_designator = Designator(initial_value='bar', resolve_type=str) # Designator that returns a string with the bar type
    operator_designator = PersonDesignator(robot=robot, furniture_designator=bar_type_designator)      # Designator that returns the operator
    
    # Driving to the operator (Sjoerd)
    bed_designator = VariableDesignator(resolve_type=EntityInfo)
    bed_type_designator = VariableDesignator(resolve_type=str) # Designator that returns a string with the bar type
    luis_designator = PersonDesignator(robot=robot, furniture_designator=bed_type_designator)      # Designator that returns the operator
    ###################

    with sm:


        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE_NO_ED',
                                Initialize(robot),
                                transitions={   'initialized':'INIT_WM',
                                                'abort':'Aborted'})

        smach.StateMachine.add( "INIT_WM",
                                    InitializeWorldModel(robot),
                                    transitions={'done'    :'FAKESHUTDOWN'})

        ## ?? TODO WHICH SENSORS FOR ED SHOULD BE

        smach.StateMachine.add('FAKESHUTDOWN',
                                    FakeShutdownRobot(robot),
                                    transitions={   'done':'WAIT_FOR_TRIGGER_TO_START'})
       
        smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
                                    WaitForEntity(robot,ed_entity_name='walls'),
                                    transitions={   'entity_exists'        :'FAKESTARTUP'})
    
        smach.StateMachine.add('FAKESTARTUP',
                                    FakeStartupRobot(robot),
                                    transitions={   'done':'FORCEDRIVE_TO_THE_RIGHT'})

        smach.StateMachine.add('FORCEDRIVE_TO_THE_RIGHT',
                                    ForceDriveToTheRight(robot),
                                    transitions={   'done':'SET_INITIAL_POSE_TEST'})

        smach.StateMachine.add( "SET_INITIAL_POSE_TEST",
                                    states.Say(robot, ["Only for testing,gui now initial pose is needed. Probably a crash ;)"], block=True),
                                    transitions={   'spoken'            :'WAIT_FOR_ENTITY_INITIAL_POSE'})

        smach.StateMachine.add("WAIT_FOR_ENTITY_INITIAL_POSE", 
                                    WaitForEntityInitialPose(robot,ed_entity_name=challenge_knowledge.initial_pose_amigo),
                                    transitions={   'entity_exists'                   : 'SET_INITIAL_POSE',
                                                    'initial_pose_to_be_set_manually' : 'ASK_ACTION'})

        smach.StateMachine.add('SET_INITIAL_POSE',
                                states.SetInitialPose(robot, challenge_knowledge.initial_pose_amigo),
                                transitions={   'done'          :'ASK_ACTION',
                                                'preempted'     :'ASK_ACTION',
                                                'error'         :'ASK_ACTION'})


        ######################################################
        ##################### ASK STATE  #####################             
        ######################################################


        smach.StateMachine.add("ASK_ACTION",
                                AskAction(robot),
                                transitions={'drive_near_loc_for_person':'ASK_LOCATION_PERSON',
                                             'grasp_object':'ASK_ACTION',#'GOTO_GRASP_LOCATION',
                                             'time_asked_for':'SAY_TIME_LEFT',
                                             'no_action':'ASK_ACTION',
                                             'failed':'ASK_ACTION'})

        smach.StateMachine.add("SAY_TIME_LEFT",
                                timer.SayRemainingTime(robot,block=True),
                                transitions={'done':'ASK_ACTION',
                                             'failed':'ASK_ACTION'})

        smach.StateMachine.add("ASK_LOCATION_PERSON",
                                AskPersonLoc(robot),
                                transitions={'drive_near_loc_for_person':'GOTO_SECOND_OPERATOR_FURNITURE',#'DRIVE_TO_LOC_FOR_PERSON',
                                             'no_action':'ASK_LOCATION_PERSON',
                                             'no_multiple_actions_heard':'ASK_ACTION', # In case multiple times location was not heard.
                                             'failed':'ASK_ACTION'})

        ###### Janno ######
        ''' GoTo Sjoerd '''
        smach.StateMachine.add('GOTO_OPERATOR_FURNITURE',
                                states.NavigateToObserve(robot=robot, entity_designator=bar_designator, radius=2.0),
                                transitions={   'arrived'           : 'GOTO_OPERATOR',
                                                'unreachable'       : 'GOTO_OPERATOR',
                                                'goal_not_defined'  : 'GOTO_OPERATOR'})

        smach.StateMachine.add('GOTO_OPERATOR',
                                states.NavigateToObserve(robot=robot, entity_designator=operator_designator, radius=0.7),
                                transitions={   'arrived'           : 'Done',
                                                'unreachable'       : 'Done',
                                                'goal_not_defined'  : 'Done'})

        ''' GoTo Luis '''
        smach.StateMachine.add('GOTO_SECOND_OPERATOR_FURNITURE',
                                states.NavigateToObserve(robot=robot, entity_designator=bed_designator, radius=2.0),
                                transitions={   'arrived'           : 'GOTO_SECOND_OPERATOR',
                                                'unreachable'       : 'GOTO_SECOND_OPERATOR',
                                                'goal_not_defined'  : 'GOTO_SECOND_OPERATOR'})

        smach.StateMachine.add('GOTO_SECOND_OPERATOR',
                                states.NavigateToObserve(robot=robot, entity_designator=luis_designator, radius=0.7),
                                transitions={   'arrived'           : 'Done',
                                                'unreachable'       : 'Done',
                                                'goal_not_defined'  : 'Done'})
        ###################



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