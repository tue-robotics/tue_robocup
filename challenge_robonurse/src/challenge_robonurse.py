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

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_robonurse')
ROOM = challenge_knowledge.room
GRANNIES_TABLE_KB = challenge_knowledge.grannies_table
BOTTLE_SHELF = challenge_knowledge.bottle_shelf

def raw_input_timeout(prompt, timeout=10):
    from select import select

    print prompt
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        s = sys.stdin.readline()
        return s
    else:
        print "No input. Moving on..."


class BottleDescription(object):
    def __init__(self, size=None, color=None, label=None):
        self.size = size
        self.color = color
        self.label = label


def get_entity_color(entity):
        try:
            return max(entity.data['perception_result']['color_matcher']['colors'], key=lambda d: d['value'])['name']
        except KeyError, ke:
            rospy.logwarn(ke)
            return ""
        except TypeError, te:
            rospy.logwarn(te)
            return ""


def get_entity_size(entity):
    size = ""
    try:
        height = abs(entity.z_min - entity.z_max)
        if height < 0.05:
            size = "small"
        elif 0.05 <= height < 0.10:
            size = "normal sized"
        elif 0.10 <= height:
            size = "big"
        rospy.loginfo("Height of object {0} is {1} so classifying as {2}".format(entity.id, height, size))
    except:
        pass

    return size


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


class DescribeBottles(smach.State):
    def __init__(self, robot, bottle_collection_designator, spec_designator, choices_designator):
        """
        @param robot the robot to run this with
        @bottle_collection_designator designates a bunch of bottles/entities
        @param spec_designator based on the descriptions read aloud by the robot, a spec for speech interpretation is created and stored in this VariableDesignator
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.robot = robot
        check_resolve_type(bottle_collection_designator, [EntityInfo])
        self.bottle_collection_designator = bottle_collection_designator

        self.spec_designator = spec_designator
        self.choices_designator = choices_designator

    def execute(self, userdata=None):
        bottles = self.bottle_collection_designator.resolve()
        if not bottles:
            return "failed"

        #TODO: Sort bottles by their Y-coord wrt base_link. We go from large to small, so the leftmost if first
        bottle_to_y_dict = {}
        for bottle in bottles:
            in_map = geom.PointStamped(point=bottle.center_point, frame_id=bottle.id)
            in_base_link = transformations.tf_transform(in_map, "/map", "/"+self.robot.robot_name+"/base_link", self.robot.tf_listener)
            bottle_to_y_dict[bottle] = in_base_link.y

        sorted_bottles = sorted(bottle_to_y_dict.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

        descriptions = OrderedDict()
        for bottle_at_y in sorted_bottles:
            descriptions[bottle_at_y] = self.describe_bottle(bottle_at_y)

        self.robot.speech.speak("I see {0} bottles, which do you want?".format(len(descriptions)))
        self.robot.speech.speak("From left to right, I have a")
        for bottle, description in descriptions.iteritems():
            desc_sentence = "a {size}, {color} one".format(size=description.size, color=description.color)
            if description.label:
                desc_sentence += " labeled {label}".format(label=description.label)
            self.robot.speech.speak(desc_sentence)
        self.robot.speech.speak("Which do you want?")

        colors = set([desc.color for desc in descriptions.values()])
        sizes = set([desc.size for desc in descriptions.values()])
        labels = set([desc.label for desc in descriptions.values()])
        choices = {"color": colors, "size": sizes, "label": labels}

        # import ipdb; ipdb.set_trace()
        self.spec_designator.current = "Give me the <size> <color> bottle labeled <label>"  # TODO: allow more sentences
        self.choices_designator.current = choices

        return "succeeded"

    def describe_bottle(self, bottle_at_y):
        bottle_entity, y = bottle_at_y

        # import ipdb; ipdb.set_trace()
        most_probable_color = get_entity_color(bottle_entity)
        size = get_entity_size(bottle_entity)

        return BottleDescription(   size=size,
                                    color=most_probable_color,
                                    label=random.choice(["aspirin", "ibuprofen", ""]))


class DetectAction(smach.State):
    def __init__(self, robot, person_to_analyse):
        smach.State.__init__(self, outcomes=["drop_blanket", "fall", "walk_and_sit"])
        self.robot = robot
        self.person_to_analyse = person_to_analyse

    def execute(self, userdata):
        which = raw_input_timeout("Which action has been performed? : {0}".format({i:v for i, v in enumerate(self.get_registered_outcomes())}))
        if which is not None:
            try:
                return self.get_registered_outcomes()[int(which)]
            except:
                rospy.logerr("No valid input received, picking a random action")
                return random.choice(self.get_registered_outcomes())
        else:
            rospy.logerr("No valid input received, picking a random action")
            return random.choice(self.get_registered_outcomes())


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
                                    transitions={   'arrived'           :'ASK_GRANNY',
                                                    'unreachable'       :'ASK_GRANNY',
                                                    'goal_not_defined'  :'ASK_GRANNY'})

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
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="gpsr_cupboard"), radius=0.1),
                                    transitions={   'arrived':'LOOKAT_SHELF',
                                                    'unreachable':'GOTO_SHELF_BACKUP',
                                                    'goal_not_defined':'GOTO_SHELF_BACKUP'})

            smach.StateMachine.add('GOTO_SHELF_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="gpsr_cupboard"), radius=0.2),
                                    transitions={   'arrived':'LOOKAT_SHELF',
                                                    'unreachable':'LOOKAT_SHELF',
                                                    'goal_not_defined':'LOOKAT_SHELF'})

            smach.StateMachine.add("LOOKAT_SHELF",
                                     Look_point(robot, 5.958, 8.337, 0.8),
                                     transitions={  'looking'         :'SAY_FOUND_OBJECTS'})

            smach.StateMachine.add( "SAY_FOUND_OBJECTS",
                                    states.Say(robot, "I see a red bottle, a blue bottle and a white bottle", block=True),
                                    transitions={   'spoken'            :'WAIT_TIME_SHELF'})

            smach.StateMachine.add( "WAIT_TIME_SHELF",
                                    states.Wait_time(robot, waittime=2),
                                    transitions={   'waited'    : 'STOP_LOOKING_STRAIGHT',
                                                    'preempted' : 'STOP_LOOKING_STRAIGHT'})

            smach.StateMachine.add("STOP_LOOKING_STRAIGHT",
                                     Stop_looking(robot),
                                     transitions={  'stopped_looking'         :'GOTO_ASK_GRANNY'})

            smach.StateMachine.add( "GOTO_ASK_GRANNY",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'ASK_WHICH_PILLS',
                                                    'unreachable'       :'ASK_WHICH_PILLS',
                                                    'goal_not_defined'  :'ASK_WHICH_PILLS'})

            smach.StateMachine.add("ASK_WHICH_PILLS",
                                    Ask_pills(robot),
                                    transitions={'red':'RED_GOTO_SHELF',     #HACK, ASSUME RED IS COKE, WHITE IS MINTS and BLUE = bubblemint
                                             'white':'WHITE_GOTO_SHELF',
                                             'blue':'BLUE_GOTO_SHELF',
                                             'failed':'RED_GOTO_SHELF'})

            #####################################

            smach.StateMachine.add( "RED_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'RED_LOOKAT_SHELF',
                                                    'unreachable'       :'RED_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'RED_LOOKAT_SHELF'})

            smach.StateMachine.add("RED_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf),
                                     transitions={  'succeeded'         :'WAIT_TIME_RED'})

            smach.StateMachine.add( "WAIT_TIME_RED",
                                    states.Wait_time(robot, waittime=4),
                                    transitions={   'waited'    : 'GRAB_COKE',
                                                    'preempted' : 'GRAB_COKE'})

            #####################################

            smach.StateMachine.add( "WHITE_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'WHITE_LOOKAT_SHELF',
                                                    'unreachable'       :'WHITE_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'WHITE_LOOKAT_SHELF'})

            smach.StateMachine.add("WHITE_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf),
                                     transitions={  'succeeded'         :'WAIT_TIME_WHITE'})

            smach.StateMachine.add( "WAIT_TIME_WHITE",
                                    states.Wait_time(robot, waittime=4),
                                    transitions={   'waited'    : 'GRAB_MINTS',
                                                    'preempted' : 'GRAB_MINTS'})

            #####################################

            smach.StateMachine.add( "BLUE_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'BLUE_LOOKAT_SHELF',
                                                    'unreachable'       :'BLUE_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'BLUE_LOOKAT_SHELF'})

            smach.StateMachine.add("BLUE_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf),
                                     transitions={  'succeeded'         :'WAIT_TIME_BLUE'})

            smach.StateMachine.add( "WAIT_TIME_BLUE",
                                    states.Wait_time(robot, waittime=4),
                                    transitions={   'waited'    : 'GRAB_BUBBLEMINT',
                                                    'preempted' : 'GRAB_BUBBLEMINT'})

            #####################################

            red_pills = EdEntityDesignator(robot, type="coke")
            blue_pills = EdEntityDesignator(robot, type="mints")
            white_pills = EdEntityDesignator(robot, type="bubblemint")

            #####################################

            smach.StateMachine.add( "GRAB_COKE",
                                    Grab(robot, red_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_COKE_BACKUP_BUBBLEMINT'})

            smach.StateMachine.add( "GRAB_COKE_BACKUP_BUBBLEMINT",
                                    Grab(robot, blue_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_COKE_BACKUP_MINTS'})

            smach.StateMachine.add( "GRAB_COKE_BACKUP_MINTS",
                                    Grab(robot, white_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            #####################################

            smach.StateMachine.add( "GRAB_BUBBLEMINT",
                                    Grab(robot, blue_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_BUBBLEMINT_BACKUP_COKE'})

            smach.StateMachine.add( "GRAB_BUBBLEMINT_BACKUP_COKE",
                                    Grab(robot, red_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_BUBBLEMINT_BACKUP_MINTS'})

            smach.StateMachine.add( "GRAB_BUBBLEMINT_BACKUP_MINTS",
                                    Grab(robot, white_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            #####################################

            smach.StateMachine.add( "GRAB_MINTS",
                                    Grab(robot, white_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_MINTS_BACKUP_COKE'})

            smach.StateMachine.add( "GRAB_MINTS_BACKUP_COKE",
                                    Grab(robot, red_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'GRAB_MINTS_BACKUP_BUBBLEMINT'})

            smach.StateMachine.add( "GRAB_MINTS_BACKUP_BUBBLEMINT",
                                    Grab(robot, blue_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            #####################################

            smach.StateMachine.add( "SAY_PILLS_TAKEN",
                                    states.Say(robot, "I have the right pills!"),
                                    transitions={   'spoken'            :'GOTO_HANDOVER_GRANNY'})

            smach.StateMachine.add( "SAY_PILLS_TAKEN_FAILED",
                                    states.Say(robot, "I am sorry, I was not able to get the right pills!"),
                                    transitions={   'spoken'            :'GOTO_HANDOVER_GRANNY'})


            # ask_bottles_spec = VariableDesignator(resolve_type=str)
            # ask_bottles_choices = VariableDesignator(resolve_type=dict)
            # smach.StateMachine.add( "DESCRIBE_OBJECTS",
            #                         DescribeBottles(robot, 
            #                             EdEntityCollectionDesignator(robot, type="", criteriafuncs=bottle_criteria),  # Type should be bottle or only check position+size/volume
            #                             spec_designator=ask_bottles_spec,
            #                             choices_designator=ask_bottles_choices),
            #                         transitions={   'succeeded'         :'ASK_WHICH_BOTTLE',
            #                                         'failed'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            # ask_bottles_answer = VariableDesignator(resolve_type=GetSpeechResponse)
            # smach.StateMachine.add( "ASK_WHICH_BOTTLE",
            #                         states.HearOptionsExtra(robot, ask_bottles_spec, ask_bottles_choices, ask_bottles_answer),
            #                         transitions={   'heard'             :'GRAB_BOTTLE',
            #                                         'no_result'         :'SAY_NOTHING_HEARD'})

            # smach.StateMachine.add( "SAY_NOTHING_HEARD",
            #                         states.Say(robot, ["Granny, I didn't hear you, please tell me wich bottles you want"]),
            #                         transitions={   'spoken'            :'ASK_WHICH_BOTTLE'})

            # @smach.cb_interface(outcomes=['described'])
            # def designate_bottle(userdata):
            #     # import ipdb; ipdb.set_trace()
            #     described_bottle.criteriafuncs += lambda entity: get_entity_color(entity) == ask_bottles_answer['color']
            #     described_bottle.criteriafuncs += lambda entity: get_entity_size(entity) == ask_bottles_answer['size']              
            #     described_bottle.criteriafuncs += lambda entity: entity.data["label"] == ask_bottles_answer['label']
            #     return 'described'
            # smach.StateMachine.add( "CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR",
            #                         smach.CBState(designate_bottle),
            #                         transitions={'described'            :"GRAB_BOTTLE"})

            # smach.StateMachine.add( "GRAB_BOTTLE",
            #                         Grab(robot, described_bottle, empty_arm_designator),
            #                         transitions={   'done'              :'GOTO_GRANNY_WITH_BOTTLE',
            #                                         'failed'            :'SAY_GRAB_FAILED'})

            # smach.StateMachine.add( "SAY_GRAB_FAILED",
            #                         states.Say(robot, "I couldn't grab the bottle, sorry"),
            #                         transitions={   'spoken'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            # smach.StateMachine.add( "GOTO_GRANNY_WITHOUT_BOTTLE",
            #                         states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
            #                         transitions={   'arrived'           :'DETECT_ACTION',
            #                                         'unreachable'       :'DETECT_ACTION',
            #                                         'goal_not_defined'  :'DETECT_ACTION'})

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
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="gpsr_couchtable"), radius=0.1),
                                    transitions={   'arrived':'LOOKAT_COUCHTABLE',
                                                    'unreachable':'GOTO_COUCHTABLE_BACKUP',
                                                    'goal_not_defined':'GOTO_COUCHTABLE_BACKUP'})

            smach.StateMachine.add('GOTO_COUCHTABLE_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="gpsr_couchtable"), radius=0.2),
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
