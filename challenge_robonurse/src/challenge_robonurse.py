#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/RoboNurse.tex

In short, the robot goes from it start in the corner of the room to Granny.
Granny, instructs the robot to get some pills for her.
At the shelf, there are a bunch of bottles with pills.
The robot must describe the bottles and let Granny choose a bottle.
The robot must grab the bottle and bring it to Granny.

Then, part 2 start which involves action recognition.
Granny does 1 of 3 things to which the robot must respond.
"""

import rospy
import smach
import sys
import random

from robot_smach_states.util.designators import ArmDesignator, Designator, EdEntityDesignator, EdEntityCollectionDesignator, LockingDesignator, UnoccupiedArmDesignator, ArmHoldingEntityDesignator, VariableDesignator, check_resolve_type
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations
from collections import OrderedDict
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
import operator
from robot_smach_states.util.geometry_helpers import *
from bottle_description import DescribeBottles
from action_detection import DetectAction

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_robonurse')
ROOM = challenge_knowledge.room
GRANNIES_TABLE_KB = challenge_knowledge.grannies_table
BOTTLE_SHELF = challenge_knowledge.bottle_shelf



class Look_point(smach.State):
    def __init__(self, robot, x, y, z):
        smach.State.__init__(self, outcomes=["looking"])
        self.robot = robot
        self.x = x
        self.y = y
        self.z = z

    def execute(self, userdata):

        goal = geom.PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "/map" #"/"+self._robot_name+"/base_link" # HACK TO LOOK AT RIGHT POINT.
        goal.point.x = self.x
        goal.point.y = self.y
        goal.point.z = self.z

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


class Ask_pills(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["red", "white", "blue", "failed"])
        self.robot = robot

    def execute(self, userdata):

        self.robot.speech.speak("Which color of pills would you like to have?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.spec, choices=challenge_knowledge.choices, time_out = rospy.Duration(30))
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                print "res =", res
                self.robot.speech.speak("Okay I will get the {0} pills".format(res.choices['color']))
                return res.choices['color']
            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "red"


class StartPhase(smach.StateMachine):
    def __init__(self, robot, grannies_table):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INIT_POSE',
                                                'abort':'Aborted'})

            smach.StateMachine.add('INIT_POSE',
                                states.SetInitialPose(robot, 'robonurse_initial'),
                                transitions={   'done':'HEAR_GRANNY',
                                                'preempted':'Aborted',  # This transition will never happen at the moment.
                                                'error':'HEAR_GRANNY'})  # It should never go to aborted.
            
            smach.StateMachine.add('HEAR_GRANNY',
                                states.Hear(robot, spec="((Help me)|hello|please|(please come)|amigo|sergio|come|(hi there)|hi|pills|robot|(give me my pills))",time_out=rospy.Duration(30)),
                                transitions={   'heard':'SAY_HI',
                                                'not_heard':'SAY_HI'})  # It should never go to aborted.

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["I hear you!"], block=False),
                                    transitions={"spoken":"GOTO_GRANNY"})

            smach.StateMachine.add( "GOTO_GRANNY",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'Done',
                                                    'unreachable'       :'Done',
                                                    'goal_not_defined'  :'Done'})


class RoboNurse(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        granny = EdEntityDesignator(robot, type='human')
        grannies_table = EdEntityDesignator(robot, id=GRANNIES_TABLE_KB)
        shelf = EdEntityDesignator(robot, id=BOTTLE_SHELF)

        def small(entity):
            return abs(entity.z_min - entity.z_max) < 0.20

        def minimal_height_from_floor(entity):
            return entity.z_min > 0.50

        def type_unknown_or_not_room(entity):
            return entity.type == "" or entity.type not in ["room"] or "shelf" not in entity.type

        bottle_criteria = [small, minimal_height_from_floor, type_unknown_or_not_room]

        described_bottle = LockingDesignator(EdEntityDesignator(robot, criteriafuncs=bottle_criteria, debug=False)) #Criteria funcs will be added based on what granny says

        empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])  #ArmHoldingEntityDesignator(robot.arms, robot.arms['left']) #described_bottle)




        def size(entity):
            return abs(entity.z_max - entity.z_min) < 0.4

        def on_top(entity):
            container_entity = robot.ed.get_entity(id=GRANNIES_TABLE_KB)
            return onTopOff(entity, container_entity)

        # Don't pass the weight_function, might screw up if phone is not near the robot
        phone = EdEntityDesignator(robot, criteriafuncs=[size, on_top], debug=False)

        def in_box(entity):
            print entity
            x_ok = 5.5 < entity.center_point.x < 6.4
            y_ok = 8.1 < entity.center_point.y < 8.5

            return x_ok and y_ok

        red_pills = EdEntityDesignator(robot, type="coke")#, criteriafuncs=[in_box])
        blue_pills = EdEntityDesignator(robot, type="bubblemint")#, criteriafuncs=[in_box])
        white_pills = EdEntityDesignator(robot, type="mints")#, criteriafuncs=[in_box])

        with self: 
            smach.StateMachine.add( "START_PHASE",
                                    StartPhase(robot, grannies_table),
                                    transitions={   'Done'              :'ASK_GRANNY',
                                                    'Aborted'           :'Aborted'})

            smach.StateMachine.add( "ASK_GRANNY",
                                    states.Say(robot, ["Hello Granny, Shall I bring you your pills?"], block=True),
                                    transitions={   'spoken'            :'HEAR_ANSWER'})

            smach.StateMachine.add('HEAR_ANSWER',
                                    states.Hear(robot, '(continue|yes|please|okay)',time_out=rospy.Duration(10)),
                                    transitions={'heard':'GOTO_SHELF','not_heard':'GOTO_SHELF'})

            # smach.StateMachine.add( "GOTO_SHELF",
            #                         states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
            #                         transitions={   'arrived'           :'LOOKAT_SHELF',
            #                                         'unreachable'       :'LOOKAT_SHELF',
            #                                         'goal_not_defined'  :'LOOKAT_SHELF'})

            smach.StateMachine.add('GOTO_SHELF',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=BOTTLE_SHELF), radius=0.1),
                                    transitions={   'arrived':'LOOKAT_SHELF',
                                                    'unreachable':'GOTO_SHELF_BACKUP',
                                                    'goal_not_defined':'GOTO_SHELF_BACKUP'})

            smach.StateMachine.add('GOTO_SHELF_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=BOTTLE_SHELF), radius=0.2),
                                    transitions={   'arrived':'LOOKAT_SHELF',
                                                    'unreachable':'LOOKAT_SHELF',
                                                    'goal_not_defined':'LOOKAT_SHELF'})

            smach.StateMachine.add("LOOKAT_SHELF",
                                     Look_point(robot, 5.958, 8.337, 0.8),
                                     transitions={  'looking'         :'DESCRIBE_OBJECTS'})

            #START Loy's version
            ask_bottles_spec = VariableDesignator(resolve_type=str)
            ask_bottles_choices = VariableDesignator(resolve_type=dict)
            smach.StateMachine.add( "DESCRIBE_OBJECTS",
                                    DescribeBottles(robot, 
                                        EdEntityCollectionDesignator(robot, type="", criteriafuncs=bottle_criteria),  # Type should be bottle or only check position+size/volume
                                        spec_designator=ask_bottles_spec,
                                        choices_designator=ask_bottles_choices),
                                    transitions={   'succeeded'         :'ASK_WHICH_BOTTLE',
                                                    'failed'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            ask_bottles_answer = VariableDesignator(resolve_type=GetSpeechResponse)
            smach.StateMachine.add( "ASK_WHICH_BOTTLE",
                                    states.HearOptionsExtra(robot, ask_bottles_spec, ask_bottles_choices, ask_bottles_answer),
                                    transitions={   'heard'             :'CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR',
                                                    'no_result'         :'SAY_NOTHING_HEARD'})

            smach.StateMachine.add( "SAY_NOTHING_HEARD",
                                    states.Say(robot, ["Granny, I didn't hear you, please tell me wich bottles you want"]),
                                    transitions={   'spoken'            :'ASK_WHICH_BOTTLE'})

            @smach.cb_interface(outcomes=['described'])
            def designate_bottle(userdata):
                # import ipdb; ipdb.set_trace()
                described_bottle.criteriafuncs += lambda entity: get_entity_color(entity) == ask_bottles_answer['color']
                described_bottle.criteriafuncs += lambda entity: get_entity_size(entity) == ask_bottles_answer['size']              
                described_bottle.criteriafuncs += lambda entity: entity.data["label"] == ask_bottles_answer['label']
                return 'described'
            smach.StateMachine.add( "CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR",
                                    smach.CBState(designate_bottle),
                                    transitions={'described'            :"GRAB_BOTTLE"})

            smach.StateMachine.add( "GRAB_BOTTLE",
                                    Grab(robot, described_bottle, empty_arm_designator),
                                    transitions={   'done'              :'GOTO_HANDOVER_GRANNY',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, "I couldn't grab the bottle, sorry Granny"),
                                    transitions={   'spoken'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            smach.StateMachine.add( "GOTO_GRANNY_WITHOUT_BOTTLE",
                                    states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
                                    transitions={   'arrived'           :'REST_ARMS_1',#DETECT_ACTION'
                                                    'unreachable'       :'REST_ARMS_1',#DETECT_ACTION'
                                                    'goal_not_defined'  :'REST_ARMS_1'})#DETECT_ACTION'
            #END Loy's version

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
                                                    'unreachable'       :'GOTO_HANDOVER_GRANNY_BACKUP',
                                                    'goal_not_defined'  :'SAY_HANDOVER_BOTTLE'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_BACKUP",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
                                                    'unreachable'       :'SAY_HANDOVER_BOTTLE',
                                                    'goal_not_defined'  :'SAY_HANDOVER_BOTTLE'})

            # smach.StateMachine.add( "GOTO_GRANNY_WITH_BOTTLE",
            #                         states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
            #                         transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
            #                                         'unreachable'       :'DETECT_ACTION',
            #                                         'goal_not_defined'  :'DETECT_ACTION'})

            smach.StateMachine.add( "SAY_HANDOVER_BOTTLE",
                                    states.Say(robot, ["Here are your pills, Granny.", "Granny, here are your pills."], block=False),
                                    transitions={   'spoken'            :'HANDOVER_TO_GRANNY'})

            smach.StateMachine.add('HANDOVER_TO_GRANNY',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'REST_ARMS_1', #DETECT_ACTION',
                                                    'failed'            :'REST_ARMS_1'}) #DETECT_ACTION'})

            smach.StateMachine.add( "REST_ARMS_1",
                                    states.ResetArms(robot),
                                    transitions={   'done'            :'WAIT_TIME'})

            smach.StateMachine.add( "WAIT_TIME",
                                    states.Wait_time(robot, waittime=10),
                                    transitions={   'waited'    : 'SAY_FELL',
                                                    'preempted' : 'SAY_FELL'})

            smach.StateMachine.add( "SAY_FELL",
                                    states.Say(robot, "Oh no, you fell! I will give you the phone.", block=True),
                                    transitions={   'spoken' : 'GOTO_COUCHTABLE'})

            smach.StateMachine.add('GOTO_COUCHTABLE',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=GRANNIES_TABLE_KB), radius=0.1),
                                    transitions={   'arrived':'LOOKAT_COUCHTABLE',
                                                    'unreachable':'GOTO_COUCHTABLE_BACKUP',
                                                    'goal_not_defined':'GOTO_COUCHTABLE_BACKUP'})

            smach.StateMachine.add('GOTO_COUCHTABLE_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=GRANNIES_TABLE_KB), radius=0.2),
                                    transitions={   'arrived':'LOOKAT_COUCHTABLE',
                                                    'unreachable':'LOOKAT_COUCHTABLE',
                                                    'goal_not_defined':'LOOKAT_COUCHTABLE'})

            smach.StateMachine.add("LOOKAT_COUCHTABLE",
                                     Look_point(robot,8.055, 6.662, 0.4),
                                     transitions={  'looking'         :'SAY_TRY_GRAB_PHONE'})

            smach.StateMachine.add( "SAY_TRY_GRAB_PHONE",
                                    states.Say(robot, "I am trying to grab the phone.", block=False),
                                    transitions={   'spoken'            :'WAIT_TIME_TO_UPDATE_MODEL'})

            smach.StateMachine.add( "WAIT_TIME_TO_UPDATE_MODEL",
                                    states.Wait_time(robot, waittime=4),
                                    transitions={   'waited'    : 'STOP_LOOKING_COUCHTABLE',
                                                    'preempted' : 'STOP_LOOKING_COUCHTABLE'})

            smach.StateMachine.add("STOP_LOOKING_COUCHTABLE",
                                     Stop_looking(robot),
                                     transitions={  'stopped_looking'         :'GRAB_PHONE'})

            #phone = EdEntityDesignator(robot, type="deodorant")        


            smach.StateMachine.add( "GRAB_PHONE",
                                    Grab(robot, phone, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PHONE_TAKEN',
                                                    'failed'            :'SAY_PHONE_TAKEN_FAILED'})

            smach.StateMachine.add( "SAY_PHONE_TAKEN",
                                    states.Say(robot, "I have the phone!"),
                                    transitions={   'spoken'            :'GOTO_HANDOVER_GRANNY_PHONE'})

            smach.StateMachine.add( "SAY_PHONE_TAKEN_FAILED",
                                    states.Say(robot, "I am sorry, I was not able to get the phone, please try to get it yourself. You can do it! Good luck!"),
                                    transitions={   'spoken'            :'Done'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_PHONE",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_PHONE',
                                                    'unreachable'       :'GOTO_HANDOVER_GRANNY_PHONE_BACKUP',
                                                    'goal_not_defined'  :'SAY_HANDOVER_PHONE'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_PHONE_BACKUP",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_PHONE',
                                                    'unreachable'       :'SAY_HANDOVER_PHONE',
                                                    'goal_not_defined'  :'SAY_HANDOVER_PHONE'})

            smach.StateMachine.add( "SAY_HANDOVER_PHONE",
                                    states.Say(robot, ["Here is the phone, Granny."]),
                                    transitions={   'spoken'            :'HANDOVER_PHONE_TO_GRANNY'})

            smach.StateMachine.add('HANDOVER_PHONE_TO_GRANNY',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'REST_ARMS_2', #DETECT_ACTION',
                                                    'failed'            :'SAY_PHONE_TAKEN_FAILED'}) #DETECT_ACTION'})

            smach.StateMachine.add( "REST_ARMS_2",
                                    states.ResetArms(robot),
                                    transitions={   'done'            :'SAY_CALL_FOR_HELP'})

            smach.StateMachine.add( "SAY_CALL_FOR_HELP",
                                    states.Say(robot, ["Please use the phone to call for help!"]),
                                    transitions={   'spoken' :'Done'})





            # smach.StateMachine.add('DETECT_ACTION',
            #                        DetectAction(robot, granny),
            #                        transitions={    "drop_blanket"      :"PICKUP_BLANKET", 
            #                                         "fall"              :"BRING_PHONE",
            #                                         "walk_and_sit"      :"FOLLOW_TAKE_CANE"})

            # smach.StateMachine.add( "PICKUP_BLANKET",
            #                         states.Say(robot, ["I will pick up your blanket"]),
            #                         transitions={   'spoken'            :'Done'})

            # smach.StateMachine.add( "BRING_PHONE",
            #                         states.Say(robot, ["I will bring you a phone"]),
            #                         transitions={   'spoken'            :'Done'})

            # smach.StateMachine.add( "FOLLOW_TAKE_CANE",
            #                         states.Say(robot, ["I will hold your cane"]),
            #                         transitions={   'spoken'            :'Done'})


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('robonurse_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    startup(RoboNurse, robot_name=robot_name)
