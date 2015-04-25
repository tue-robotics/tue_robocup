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

from robot_smach_states.util.designators import Designator, EdEntityDesignator, EdEntityCollectionDesignator, LockingDesignator, UnoccupiedArmDesignator, ArmHoldingEntityDesignator, VariableDesignator, check_resolve_type
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
        arm_with_item_designator = ArmHoldingEntityDesignator(robot.arms, described_bottle)

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
                                states.Hear(robot, spec="((Help me)|hello|please|(please come)|amigo|sergio|come|(hi there)|hi|pills|robot|(give me my pills))",time_out=rospy.Duration(20)),
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
                                    transitions={   'spoken'            :'GOTO_SHELF'})

            smach.StateMachine.add( "GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'LOOKAT_SHELF',
                                                    'unreachable'       :'LOOKAT_SHELF',
                                                    'goal_not_defined'  :'LOOKAT_SHELF'})

            smach.StateMachine.add("LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf, keep_following=True),
                                     transitions={  'succeeded'         :'SAY_FOUND_OBJECTS'})

            smach.StateMachine.add( "SAY_FOUND_OBJECTS",
                                    states.Say(robot, "I see a red bottle, a blue bottle and a white bottle"),
                                    transitions={   'spoken'            :'GOTO_ASK_GRANNY'})

            smach.StateMachine.add('GOTO_ASK_GRANNY',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="robonurse_granny_loc"), radius=0.2),
                                    transitions={   'arrived':'ASK_WHICH_PILLS',
                                                    'unreachable':'ASK_WHICH_PILLS',
                                                    'goal_not_defined':'ASK_WHICH_PILLS'})

            smach.StateMachine.add("ASK_WHICH_PILLS",
                                Ask_pills(robot),
                                transitions={'red':'RED_GOTO_SHELF',     #HACK, ASSUME RED IS COKE, WHITE IS MINTS and BLUE = bubblemint
                                             'white':'WHITE_GOTO_SHELF',
                                             'blue':'BLUE_GOTO_SHELF',
                                             'failed':'WHITE_GOTO_SHELF'})

            smach.StateMachine.add( "RED_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'RED_LOOKAT_SHELF',
                                                    'unreachable'       :'RED_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'RED_LOOKAT_SHELF'})

            smach.StateMachine.add("RED_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf, keep_following=True),
                                     transitions={  'succeeded'         :'GRAB_COKE'})

            smach.StateMachine.add( "WHITE_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'WHITE_LOOKAT_SHELF',
                                                    'unreachable'       :'WHITE_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'WHITE_LOOKAT_SHELF'})

            smach.StateMachine.add("WHITE_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf, keep_following=True),
                                     transitions={  'succeeded'         :'GRAB_MINTS'})

            smach.StateMachine.add( "BLUE_GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'BLUE_LOOKAT_SHELF',
                                                    'unreachable'       :'BLUE_LOOKAT_SHELF',
                                                    'goal_not_defined'  :'BLUE_LOOKAT_SHELF'})

            smach.StateMachine.add("BLUE_LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf, keep_following=True),
                                     transitions={  'succeeded'         :'GRAB_BUBBLEMINT'})


            red_pills = EdEntityDesignator(robot, type="coke")
            blue_pills = EdEntityDesignator(robot, type="mints")
            white_pills = EdEntityDesignator(robot, type="bubblemint")

            smach.StateMachine.add( "GRAB_COKE",
                                    Grab(robot, red_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            smach.StateMachine.add( "GRAB_BUBBLEMINT",
                                    Grab(robot, blue_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            smach.StateMachine.add( "GRAB_MINTS",
                                    Grab(robot, white_pills, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PILLS_TAKEN',
                                                    'failed'            :'SAY_PILLS_TAKEN_FAILED'})

            smach.StateMachine.add( "SAY_PILLS_TAKEN",
                                    states.Say(robot, "I have the right pills!"),
                                    transitions={   'spoken'            :'HANDOVER_TO_GRANNY'})

            smach.StateMachine.add( "SAY_PILLS_TAKEN_FAILED",
                                    states.Say(robot, "I am sorry, I was not able to get the right pills!"),
                                    transitions={   'spoken'            :'Done'})


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

            # smach.StateMachine.add( "GOTO_GRANNY_WITH_BOTTLE",
            #                         states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
            #                         transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
            #                                         'unreachable'       :'DETECT_ACTION',
            #                                         'goal_not_defined'  :'DETECT_ACTION'})

            # smach.StateMachine.add( "SAY_HANDOVER_BOTTLE",
            #                         states.Say(robot, ["Here are your pills, Granny.", "Granny, here are your pills."]),
            #                         transitions={   'spoken'            :'HANDOVER_TO_GRANNY'})

            smach.StateMachine.add('HANDOVER_TO_GRANNY',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'Done', #DETECT_ACTION',
                                                    'failed'            :'Done'}) #DETECT_ACTION'})

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
