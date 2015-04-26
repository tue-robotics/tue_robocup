#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')
INITIAL_POSE = challenge_knowledge.initial_pose_amigo

from robot_smach_states.state import State

OBJECTS_LIST = []

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
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.explore_location_2), radius=0.2),
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
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.3),
                                    transitions={   'arrived':'SAY_FAILED',
                                                    'unreachable':'SAY_FAILED',
                                                    'goal_not_defined':'SAY_FAILED'})

        smach.StateMachine.add( "SAY_FAILED",
                                    states.Say(robot, ["I'm really sorry, although I have the arms to pick up the drink."], block=False),
                                    transitions={   'spoken'            :'REST_ARMS'})

        smach.StateMachine.add( "SAY_SUCCESS",
                                    states.Say(robot, ["All right!"], block=False),
                                    transitions={   'spoken'            :'GOTO_BOSS'})

        smach.StateMachine.add('GOTO_BOSS',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.1),
                                    transitions={   'arrived':'HANDOVER_TO_BOSS',
                                                    'unreachable':'GOTO_BOSS_BACKUP',
                                                    'goal_not_defined':'GOTO_BOSS_BACKUP'})

        smach.StateMachine.add('GOTO_BOSS_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=challenge_knowledge.task_location_sergio), radius=0.3),
                                    transitions={   'arrived':'HANDOVER_TO_BOSS',
                                                    'unreachable':'HANDOVER_TO_BOSS',
                                                    'goal_not_defined':'HANDOVER_TO_BOSS'})

        smach.StateMachine.add('HANDOVER_TO_BOSS',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'REST_ARMS', #DETECT_ACTION',
                                                    'failed'            :'REST_ARMS'}) #DETECT_ACTION'})

        smach.StateMachine.add( "REST_ARMS",
                                    states.ResetArms(robot),
                                    transitions={   'done'            :'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='amigo')