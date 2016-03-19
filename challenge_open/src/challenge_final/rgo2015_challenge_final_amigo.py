#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_skills.util import msg_constructors as geom

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_open')
INITIAL_POSE = challenge_knowledge.initial_pose_amigo

from robot_smach_states.state import State

OBJECTS_LIST = []

LOCATION_LIST = challenge_knowledge.location_options
CHOSEN_LOCATION = ['table']

CURRENT_LOCATION_LIST = []

class StartChallengeFinal(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    class GoToEntryPoint(State):
        def __init__(self, robot, initial_pose, use_entry_points = False):
            State.__init__(self, locals(), outcomes=["no_goal" , "found", "not_found", "all_unreachable"])

        def run(self, robot, initial_pose, use_entry_points):
            print "TODO: IMPLEMENT THIS STATE"
            return "no_goal"

    class ForceDrive(State):
        def __init__(self, robot):
            State.__init__(self, locals(), outcomes=["done"])

        def run(self, robot):
            #self.robot.speech.speak("As a back-up scenario I will now drive through the door with my eyes closed.", block=False)  # Amigo should not say that it uses force drive, looks stupid.
            rospy.loginfo("AMIGO uses force drive as a back-up scenario!")
            robot.base.force_drive(0.25, 0, 0, 5.0)    # x, y, z, time in seconds
            robot.ed.reset()
            return "done"

    def __init__(self, robot, initial_pose, use_entry_points = False):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        assert hasattr(robot, "base")
        # assert hasattr(robot, "reasoner")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add( "INITIALIZE",
                                    states.Initialize(robot),
                                    transitions={   "initialized"   :"WAIT_FOR_DOOR",
                                                    "abort"         :"Aborted"})

             # Start laser sensor that may change the state of the door if the door is open:
            smach.StateMachine.add( "WAIT_FOR_DOOR",
                                    states.WaitForDoorOpen(robot, timeout=10),
                                    transitions={   "closed":"WAIT_FOR_DOOR",
                                                    "open":"INIT_POSE"})

            # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set,
            # since it is thinks amigo is standing in front of a wall if door is closed and localization can(/will) be messed up.
            smach.StateMachine.add('INIT_POSE',
                                states.SetInitialPose(robot, initial_pose),
                                transitions={   'done':'FORCE_DRIVE_THROUGH_DOOR',
                                                'preempted':'Aborted',  # This transition will never happen at the moment.
                                                'error':'FORCE_DRIVE_THROUGH_DOOR'})  # It should never go to aborted.

            smach.StateMachine.add('FORCE_DRIVE_THROUGH_DOOR',
                                    self.ForceDrive(robot),
                                    transitions={   "done":"GO_TO_ENTRY_POINT"})

            smach.StateMachine.add('GO_TO_ENTRY_POINT',
                                    self.GoToEntryPoint(robot, initial_pose, use_entry_points),
                                    transitions={   "found":"Done",
                                                    "not_found":"GO_TO_ENTRY_POINT",
                                                    "no_goal":"Done",
                                                    "all_unreachable":"Done"})

class AwaitTriggerAndSave(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):

        order_object = states.WaitForTrigger(self.robot, challenge_knowledge.object_options,"amigo/trigger")

        OBJECTS_LIST.append(str(order_object.execute(None)))

        return "done"

class SayFinal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot

    def execute(self, userdata):

        sentence = "Sergio triggered me! Let's get a "+ OBJECTS_LIST[0] +" for the boss!"

        say_final = states.Say(self.robot, sentence,block=False)

        say_final.execute(None)

        return "spoken"

class SayFinal2(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot

    def execute(self, userdata):

        sentence = "Sir, I have got the "+ OBJECTS_LIST[0] +" for you!"

        say_final = states.Say(self.robot, sentence, block=False)

        say_final.execute(None)

        return "spoken"

class SayFinalFailed(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot

    def execute(self, userdata):

        sentence = "I'm really sorry, although I have the arms to pick up the "+ OBJECTS_LIST[0] +", I was not able to get it."

        say_final = states.Say(self.robot, sentence, block=True)

        say_final.execute(None)

        return "spoken"

class SayFinalGoToCheckLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot

    def execute(self, userdata):

        sentence = "I will check if Sergio mapped the "+ CHOSEN_LOCATION[0] +" correctly!"

        say_final = states.Say(self.robot, sentence, block=True)

        say_final.execute(None)

        return "spoken"

# class SayFinalGeneric(smach.State):
#     def __init__(self, robot, sentence_part1, variable_in_sentence, sentence_part2=" ", block=True):
#         smach.State.__init__(self, outcomes=["spoken"])
#         self.robot = robot
#         self.sentence_part1 = sentence_part1
#         self.sentence_part2 = sentence_part2
#         self.variable_in_sentence = variable_in_sentence
#         self.block = block

#     def execute(self, userdata):
#         sentence = str(self.sentence_part1) + " " + str(self.variable_in_sentence) + " " + str(self.sentence_part2)
#         print "sentence =", sentence

#         say_final = states.Say(self.robot, str(sentence), block=self.block)

#         say_final.execute(None)

#         return "spoken"

class GrabFinal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done","failed"])
        self.robot = robot

    def execute(self, userdata):

        empty_arm_designator = UnoccupiedArmDesignator(self.robot.arms, self.robot.leftArm)
        drink = EdEntityDesignator(self.robot, type=OBJECTS_LIST[0])

        grab_coke = states.Grab(self.robot, drink, empty_arm_designator)

        status = grab_coke.execute(None)

        return status


class ChooseLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["location_found", "location_list_empty"])
        self.robot = robot

        for x in range(len(LOCATION_LIST)):
            CURRENT_LOCATION_LIST.append(LOCATION_LIST[x])

    def execute(self, userdata):

        table_entity = None

        while not table_entity:
            if len(CURRENT_LOCATION_LIST) > 0:
                #print "CURRENT_LOCATION_LIST =", CURRENT_LOCATION_LIST
                random_location = random.choice(CURRENT_LOCATION_LIST)
                #print "random_location =", random_location
                table_entity = self.robot.ed.get_entities(type=str(random_location))

                if len(table_entity) > 0:
                    table_entity = table_entity[0]

                #print "table_entity", table_entity

                CURRENT_LOCATION_LIST.remove(random_location)
            else:
                return "location_list_empty"

        #print "CHOSEN_LOCATION before emptying = ",CHOSEN_LOCATION
        del CHOSEN_LOCATION[:]
        #print "CHOSEN_LOCATION after emptying = ",CHOSEN_LOCATION

        CHOSEN_LOCATION.append(str(random_location))

        #print "CHOSEN_LOCATION after adding new location = ",CHOSEN_LOCATION

        print "CHOSEN_LOCATION =", CHOSEN_LOCATION

        return "location_found"


class UpdateLocationList(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata):

        for x in range(len(LOCATION_LIST)):
            CURRENT_LOCATION_LIST.append(LOCATION_LIST[x])

        rospy.sleep(1)

        return "done"

class NavigateCheckLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["arrived","unreachable","goal_not_defined"])
        self.robot = robot

    def execute(self, userdata):

        navigate_location = states.NavigateToObserve(self.robot, EdEntityDesignator(self.robot, type=CHOSEN_LOCATION[0]), radius=0.8)

        status = navigate_location.execute(None)

        return status

class Look_point(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking"])
        self.robot = robot

    def execute(self, userdata):

        table_entity = self.robot.ed.get_entities(type=CHOSEN_LOCATION[0])

        print "table_entity = ", table_entity

        goal = geom.PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "/map" #"/"+self._robot_name+"/base_link" # HACK TO LOOK AT RIGHT POINT.
        goal.point = table_entity[0].pose.position

        self.robot.head.look_at_point(goal)
        rospy.sleep(2)
        return "looking"

class Stop_looking(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["stopped_looking"])
        self.robot = robot

    def execute(self, userdata):

        self.robot.head.cancel_goal()
        return "stopped_looking"

############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])

    with sm:
        smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
                                    AwaitTriggerAndSave(robot),
                                    transitions={   'done'              :'INITIALIZE',
                                                    'failed'            :'INITIALIZE'})
        smach.StateMachine.add("INITIALIZE",
                                StartChallengeFinal(robot, INITIAL_POSE, use_entry_points = True),
                                transitions={   "Done"              :   "SAY_GET_OBJECT",
                                                "Aborted"           :   "SAY_GET_OBJECT",
                                                "Failed"            :   "SAY_GET_OBJECT"})

        smach.StateMachine.add( 'SAY_GET_OBJECT',
                                SayFinal(robot),
                                transitions={'spoken':'GRAB_OBJECT'})

        smach.StateMachine.add( "GRAB_OBJECT",
                                    GrabFinal(robot),
                                    transitions={   'done'              :'SAY_SUCCESS',
                                                    'failed'            :'GOTO_DINNERTABLE'})       

        smach.StateMachine.add('GOTO_DINNERTABLE',
                                    states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.explore_location_2), radius=0.2),
                                    transitions={   'arrived':'GRAB_OBJECT_BACKUP',
                                                    'unreachable':'GRAB_OBJECT_BACKUP',
                                                    'goal_not_defined':'GRAB_OBJECT_BACKUP'})

        smach.StateMachine.add( "GRAB_OBJECT_BACKUP",
                                    GrabFinal(robot),
                                    transitions={   'done'              :'SAY_SUCCESS',
                                                    'failed'            :'SAY_NO_SUCCES'})

        smach.StateMachine.add( "SAY_NO_SUCCES",
                                    states.Say(robot, ["Oh no! I can't grab it."], block=False),
                                    transitions={   'spoken'            :'GOTO_BOSS_FAILURE'})

        smach.StateMachine.add('GOTO_BOSS_FAILURE',
                                    states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.3),
                                    transitions={   'arrived':'SAY_FAILED',
                                                    'unreachable':'SAY_FAILED',
                                                    'goal_not_defined':'SAY_FAILED'})

        smach.StateMachine.add( "SAY_FAILED",
                                    SayFinalFailed(robot),
                                    transitions={   'spoken'            :'REST_ARMS'})

        smach.StateMachine.add( "SAY_SUCCESS",
                                    states.Say(robot, ["All right!"], block=False),
                                    transitions={   'spoken'            :'GOTO_BOSS'})

        smach.StateMachine.add('GOTO_BOSS',
                                    states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.1),
                                    transitions={   'arrived':'HANDOVER_TO_BOSS',
                                                    'unreachable':'GOTO_BOSS_BACKUP',
                                                    'goal_not_defined':'GOTO_BOSS_BACKUP'})

        smach.StateMachine.add('GOTO_BOSS_BACKUP',
                                    states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.3),
                                    transitions={   'arrived':'SAY_HANDOVER',
                                                    'unreachable':'SAY_HANDOVER',
                                                    'goal_not_defined':'SAY_HANDOVER'})

        smach.StateMachine.add( "SAY_HANDOVER",
                                    SayFinal2(robot),
                                    transitions={   'spoken'            :'HANDOVER_TO_BOSS'})

        smach.StateMachine.add('HANDOVER_TO_BOSS',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'REST_ARMS',
                                                    'failed'            :'REST_ARMS'})

        smach.StateMachine.add( "REST_ARMS",
                                    states.ResetArms(robot),
                                    transitions={   'done'            :'CHOOSE_RANDOM_LOCATION'})

# ###################
#         smach.StateMachine.add("GOTO_FINAL_WAYPOINT",
#                                 states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.end_location_amigo), radius=0.15),
#                                 transitions={   'arrived'                  :'Done',
#                                                 'unreachable'              :'Done',
#                                                 'goal_not_defined'         :'Done'})

######################

        smach.StateMachine.add('CHOOSE_RANDOM_LOCATION',
                                   ChooseLocation(robot),
                                   transitions={    'location_found'          :'SAY_LOC_FOUND',
                                                    'location_list_empty'     :'UPDATE_LOC_LIST'})

        smach.StateMachine.add( "SAY_LOC_FOUND",
                                    SayFinalGoToCheckLocation(robot),
                                    transitions={   'spoken'            :'GOTO_RANDOM_ENTITY_LOCATION'})


        smach.StateMachine.add('GOTO_RANDOM_ENTITY_LOCATION',
                                    NavigateCheckLocation(robot),
                                    transitions={   'arrived':'LOOK_AT_LOCATION',
                                                    'unreachable':'SAY_LOC_NOT_FOUND',
                                                    'goal_not_defined':'SAY_LOC_NOT_FOUND'})

        smach.StateMachine.add("LOOK_AT_LOCATION",
                                     Look_point(robot),
                                     transitions={  'looking'         :'SAY_JOB_WELL_DONE'})

        smach.StateMachine.add( "SAY_JOB_WELL_DONE",
                                states.Say(robot, "This looks good. Well done Sergio.", block=True),
                                transitions={   'spoken'            :'STOP_LOOKING_AT_OBJECT'})

        smach.StateMachine.add("STOP_LOOKING_AT_OBJECT",
                                 Stop_looking(robot),
                                 transitions={  'stopped_looking'         :'CHOOSE_RANDOM_LOCATION'})

        smach.StateMachine.add( "SAY_LOC_NOT_FOUND",
                                    states.Say(robot, ["Sorry, i can not reach the location. Maybe another location."], block=True),
                                    transitions={   'spoken'            :'CHOOSE_RANDOM_LOCATION'})

        smach.StateMachine.add( "UPDATE_LOC_LIST",
                                    UpdateLocationList(robot),
                                    transitions={   'done'            :'CHOOSE_RANDOM_LOCATION'})


    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='amigo')
