#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/RoboNurse.tex

In short, the robot goes from it start in the corner of the room to Granny.
Granny, instructs the robot to get some pills for her.
At the shelf, there are a bunch of bottles with pills.
The robot must describe the bottles and let Granny choose a bottle.
The robot must grab the bottle and bring it to Granny.

Then, part 2 start which involves action recognition.
Granny does 1 of 3 things to which the robot must respond.

TODO: Actual action detection with a hack.
    One idea is to record a the coordinates of an entity during tracking and apply some heuristics (see dummy_action_recognition and recognize_action)
TODO: Test Take cane
"""

import rospy
import smach
import sys

import robot_smach_states.util.designators as ds
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_skills.util import msg_constructors as geom
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robot_smach_states.util.geometry_helpers import *
from bottle_description import DescribeBottles, get_entity_color, get_entity_size, BottleDescription
from action_detection import DetectAction
from ed.msg import EntityInfo
import numpy as np

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_robonurse')
ROOM = challenge_knowledge.room
GRANNIES_TABLE_KB = challenge_knowledge.grannies_table
BOTTLE_SHELF = challenge_knowledge.bottle_shelf
BOTTLE_SHELF_WAYPOINT = challenge_knowledge.bottle_shelf_waypoint



import rospy
from tf import TransformListener
from robot_skills import world_model_ed
from geometry_msgs.msg import Point
import math

LASER_GRANNY = ds.VariableDesignator(resolve_type=EntityInfo, name="LASER_GRANNY")

def define_designators(robot):
    '''Define core designators in a separate function so that can be used when testing parts of the challenge separately.'''
    granny = LASER_GRANNY
    grannies_table = ds.EntityByIdDesignator(robot, id=GRANNIES_TABLE_KB, name="grannies_table")
    shelf = ds.EntityByIdDesignator(robot, id=BOTTLE_SHELF, name="shelf")

    return granny, grannies_table, shelf

class StoreGrannyPose(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        # Hardcoded center pointS!!!
        table_centerpoint = gm.Point(3.12, -6.725, 0.0)
        possible_humans = self.robot.ed.get_closest_possible_person_entity(type="", center_point=table_centerpoint)

        self.robot.head.look_at_ground_in_front_of_robot(10)

        if not possible_humans:
            return 'failed'

        #if len(possible_humans) == 0:
        #    rospy.logwarn("No possible_humans found")
        #    return 'failed'

        # Granny is the first one
        self.designator.write(possible_humans)

        return 'succeeded'


class DetectFallingGranny(smach.State):
    def __init__(self, robot, timeout = 30):
        smach.State.__init__(self, outcomes=["sit", "fall", "walk"])
        self.robot = robot
        self.timeout = timeout

    def _get_entity_from_roi_and_center_point(self, ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y):
        entity = ed.get_closest_entity(center_point=Point(x=closest_x, y=closest_y))
        if not entity:
            return None
        if entity.pose.position.x > roi_x_start and entity.pose.position.x < roi_x_end and entity.pose.position.y > roi_y_start and entity.pose.position.y < roi_y_end:
            return entity
        return None


    def detect_action(self, ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y, timeout, distance_threshold):
        print "Find the person in the room in ROI x: [%fx%f], y: [%fx%f] from closest (x,y) : (%f,%f)" % (roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)

        total_start = rospy.Time.now()

        # Find an operator
        operator = None
        while not operator:
            if rospy.Time.now() - total_start > rospy.Duration(timeout):
                return "sit"

            tmp_operator = self._get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)
            if tmp_operator:
                start = rospy.Time.now()

                # Check if ID is alive for more than 3 seconds
                while rospy.Time.now() - start < rospy.Duration(1):
                    # check if id is the same
                    check_operator = self._get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)
                    if not check_operator:
                        operator = None
                        break

                    if check_operator.id != tmp_operator.id:
                        operator = None
                        break

                    operator = tmp_operator

                    rospy.sleep(0.2)

            rospy.sleep(0.2)

        # We found an operator
        print "We have found an operator: %s"%operator.id

        last_seen_operator = None

        # Continuously the operator
        while rospy.Time.now() - total_start < rospy.Duration(timeout):
            tmp_operator = ed.get_entity(id=operator.id)
            if tmp_operator:
                last_seen_operator = tmp_operator
            else:
                if last_seen_operator:
                    tmp_operator = self._get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, last_seen_operator.pose.position.x, last_seen_operator.pose.position.y)
                    if tmp_operator:
                        last_seen_operator = tmp_operator
                else:
                    tmp_operator = self._get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, operator.pose.position.x, operator.pose.position.y)
                    if tmp_operator:
                        last_seen_operator = tmp_operator

            rospy.sleep(0.2)

        # Check the result
        if not last_seen_operator:
            return "fall"

        dr = math.hypot(last_seen_operator.pose.position.x - operator.pose.position.x, last_seen_operator.pose.position.y - operator.pose.position.y)
        dt = (last_seen_operator.last_update_time - operator.last_update_time).to_sec()

        print "dr: %f" % dr
        print "dt: %f" % dt

        # Compare last seen with original
        if dr > distance_threshold:
            return "walk"

        if dt > 0.9 * timeout:
            return "sit"

        return "fall"

    def execute(self, userdata=None):
        # self.robot.head.reset()
        # import ipdb; ipdb.set_trace()

        result = self.detect_action(self.robot.ed, 1.59, 4.7, -9.17, -4, 2.9, -7.4, self.timeout, 2.0)

        return result




class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.ed.configure_kinect_segmentation(continuous=False)
        self.robot.ed.configure_perception(continuous=False)

        self.robot.ed.reset()

        return "done"

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
                                transitions={   'initialized':'HEAR_GRANNY',
                                                'abort':'Aborted'})

            # smach.StateMachine.add('INIT_POSE',
            #                     states.SetInitialPose(robot, 'robonurse_initial'),
            #                     transitions={   'done':'HEAR_GRANNY',
            #                                     'preempted':'Aborted',  # This transition will never happen at the moment.
            #                                     'error':'HEAR_GRANNY'})  # It should never go to aborted.

            smach.StateMachine.add('HEAR_GRANNY',
                                states.Hear(robot, spec="((Help me)|hello|please|(please come)|amigo|sergio|come|(hi there)|hi|pills|robot|(give me my pills))",time_out=rospy.Duration(30)),
                                transitions={   'heard':'SAY_HI',
                                                'not_heard':'SAY_HI'})  # It should never go to aborted.

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["I hear you!"], block=False),
                                    transitions={"spoken":"GOTO_GRANNY"})

            smach.StateMachine.add( "GOTO_GRANNY",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", ds.EntityByIdDesignator(robot, id=ROOM, name="GOTO_GRANNY_room") : "in"}, grannies_table),
                                    transitions={   'arrived'           :'Done',
                                                    'unreachable'       :'Done',
                                                    'goal_not_defined'  :'Done'})


class LookAtEntities(smach.StateMachine):
    def __init__(self, robot, entity_collection_designator, inspect_time=1.0):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        #We want one list and iterate over that. It should not update and regenerate every iteration
        locked_entity_collection_designator = ds.LockingDesignator(entity_collection_designator, name="locked_entity_collection_designator")

        element_designator = ds.VariableDesignator(resolve_type=EntityInfo, name="element_designator")

        with self:
            #We want one list and iterate over that. It should not update and regenerate every iteration.
            #So, before we start iterating, we lock the list
            smach.StateMachine.add( "LOCK_ENTITIES",
                                    states.LockDesignator(locked_entity_collection_designator),
                                    transitions={   "locked"        :"SELECT_ENTITY"})

            #At every iteration, an element of the collection is put into element_designator
            smach.StateMachine.add( "SELECT_ENTITY",
                                    states.IteratorState(locked_entity_collection_designator, element_designator),
                                    transitions={   "next"          :"LOOK_AT_ENTITY",
                                                    "stop_iteration":"UNLOCK_ENTITIES"})

            smach.StateMachine.add( "LOOK_AT_ENTITY",
                                     states.LookAtEntity(robot, element_designator, waittime=inspect_time),
                                     transitions={  'succeeded'         :'SELECT_ENTITY',
                                                    'failed'            :'SELECT_ENTITY'})

            smach.StateMachine.add( "UNLOCK_ENTITIES",
                                    states.LockDesignator(locked_entity_collection_designator),
                                    transitions={   "locked"        :"succeeded"})


class GetPills(smach.StateMachine):
    def __init__(self, robot, shelf, grannies_table, granny):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.arms['left'], name="empty_arm_designator")
        arm_with_item_designator = ds.ArmDesignator(robot.arms, robot.arms['left'], name="arm_with_item_designator")  #ArmHoldingEntityDesignator(robot.arms, robot.arms['left']) #described_bottle, name="arm_with_item_designator")

        def small(entity):
            return abs(entity.z_max - entity.z_min) < 0.20

        def not_too_small(entity):
            return abs(entity.z_max - entity.z_min) > 0.03

        def minimal_height_from_floor(entity):
            return (entity.z_min + entity.pose.position.z)> 0.50

        def type_unknown_or_not_room(entity):
            return entity.type == "" or entity.type not in ["room"] or "shelf" not in entity.type

        def not_bookcase_part(entity):
            return not BOTTLE_SHELF in entity.id #Bookcase has elements named "bookcase/shelf1" etc. Ditch those

        # import ipdb; ipdb.set_trace()
        def on_top(entity):
            container_entity = shelf.resolve()
            return onTopOff(entity, container_entity)

        bottle_criteria = [minimal_height_from_floor, type_unknown_or_not_room, not_bookcase_part, onTopOffForDesignator(shelf), small, not_too_small]

        # shelves = ds.EdEntityCollectionDesignator(robot, criteriafuncs=[lambda e: "bookcase" in e.id], name="shelves")
        bottles_to_describe = ds.EdEntityCollectionDesignator(robot, type="", criteriafuncs=bottle_criteria, debug=False, name="bottles_to_describe")
        described_bottle = ds.EdEntityDesignator(robot, debug=False, name="described_bottle") #ID will be decided by the description given by granny
        locked_described_bottle = ds.LockingDesignator(described_bottle, name="locked_described_bottle")
        room_designator = ds.EntityByIdDesignator(robot, id=ROOM, name="room_designator")

        with self:
            # smach.StateMachine.add( "SAY_FOUND_3_BOTTLEES",
            #                          states.Say(robot, ["I see three bottles"], block=False),
            #                          transitions={  'spoken'            :'LOOKAT_SHELF'})

            smach.StateMachine.add( "LOOKAT_SHELF",
                                     states.LookAtEntity(robot, shelf, waittime=1.0),
                                     transitions={  'succeeded'         :'DESCRIBE_OBJECTS',
                                                    'failed'            :'failed'}) #If you can't look at objects, you can't describe them

            ask_bottles_spec = ds.VariableDesignator(resolve_type=str, name="ask_bottles_spec")
            ask_bottles_choices = ds.VariableDesignator(resolve_type=dict, name="ask_bottles_choices")
            bottle_description_map_desig = ds.VariableDesignator(resolve_type=dict, name="bottle_description_map_desig")

            smach.StateMachine.add( "DESCRIBE_OBJECTS",
                                    DescribeBottles(robot, bottles_to_describe,
                                        spec_designator=ask_bottles_spec.writeable,
                                        choices_designator=ask_bottles_choices.writeable,
                                        bottle_desc_mapping_designator=bottle_description_map_desig.writeable),
                                    transitions={   'succeeded'         :'GOTO_GRANNY_ASK_BOTTLE',
                                                    'failed'            :'SAY_LOOKAT_SHELF_2'})

            #If the description fails at first, say something to also wait a little bit for entities to pop into ED and then try again
            smach.StateMachine.add( "SAY_LOOKAT_SHELF_2",
                                     states.Say(robot, ["Looking"]),
                                     transitions={  'spoken'            :'LOOKAT_SHELF_2'})

            smach.StateMachine.add( "LOOKAT_SHELF_2",
                                     states.LookAtEntity(robot, shelf),
                                     transitions={  'succeeded'         :'DESCRIBE_OBJECTS_2',
                                                    'failed'            :'failed'}) #If you can't look at objects, you can't describe them

            smach.StateMachine.add( "DESCRIBE_OBJECTS_2",
                                    DescribeBottles(robot, bottles_to_describe,
                                        spec_designator=ask_bottles_spec.writeable,
                                        choices_designator=ask_bottles_choices.writeable,
                                        bottle_desc_mapping_designator=bottle_description_map_desig.writeable),
                                    transitions={   'succeeded'         :'GOTO_GRANNY_ASK_BOTTLE',
                                                    'failed'            :'SAY_FAILED_NO_BOTTLES'})

            smach.StateMachine.add( "SAY_FAILED_NO_BOTTLES",
                                    states.Say(robot, ["Sorry granny, I could not grasp any bottle"]),
                                    transitions={   'spoken'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            smach.StateMachine.add( "GOTO_GRANNY_ASK_BOTTLE",
                                    states.NavigateToSymbolic(robot, {granny:"near", room_designator:"in"}, granny),
                                    transitions={   'arrived'           :'GOTO_GRANNY_ASK_BOTTLE_BACKUP',#DETECT_ACTION'
                                                    'unreachable'       :'GOTO_GRANNY_ASK_BOTTLE_BACKUP',#DETECT_ACTION'
                                                    'goal_not_defined'  :'GOTO_GRANNY_ASK_BOTTLE_BACKUP'})#DETECT_ACTION'

            smach.StateMachine.add('GOTO_GRANNY_ASK_BOTTLE_BACKUP',
                                    states.NavigateToSymbolic(robot,
                                        {grannies_table: "in_front_of_pos2" },
                                                              grannies_table),
                                    transitions={   'arrived'           :   'GOTO_GRANNY_ASK_BOTTLE_BACKUP_23',
                                                    'unreachable'       :   'GOTO_GRANNY_ASK_BOTTLE_BACKUP_23',
                                                    'goal_not_defined'  :   'GOTO_GRANNY_ASK_BOTTLE_BACKUP_23'})



            smach.StateMachine.add( "GOTO_GRANNY_ASK_BOTTLE_BACKUP_23",
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", room_designator: "in"}, grannies_table),
                                    transitions={   'arrived'           :'ASK_WHICH_BOTTLE',#DETECT_ACTION'
                                                    'unreachable'       :'ASK_WHICH_BOTTLE',#DETECT_ACTION'
                                                    'goal_not_defined'  :'ASK_WHICH_BOTTLE'})#DETECT_ACTION'



            ask_bottles_answer = ds.VariableDesignator(resolve_type=GetSpeechResponse, name="ask_bottles_answer")
            smach.StateMachine.add( "ASK_WHICH_BOTTLE",
                                    states.HearOptionsExtra(robot, ask_bottles_spec, ask_bottles_choices,
                                                            ask_bottles_answer.writeable,
                                                            look_at_standing_person=False),
                                    transitions={   'heard'             :'CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR',
                                                    'no_result'         :'SAY_NOTHING_HEARD'})

            smach.StateMachine.add( "SAY_NOTHING_HEARD",
                                    states.Say(robot, ["Granny, I didn't hear you, please tell me wich bottles you want"]),
                                    transitions={   'spoken'            :'ASK_WHICH_BOTTLE_2'})

            smach.StateMachine.add( "ASK_WHICH_BOTTLE_2",
                                    states.HearOptionsExtra(robot, ask_bottles_spec, ask_bottles_choices,
                                                            ask_bottles_answer.writeable, look_at_standing_person=False),
                                    transitions={   'heard'             :'CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR',
                                                    'no_result'         :'SAY_NOTHING_HEARD_2'})

            smach.StateMachine.add( "SAY_NOTHING_HEARD_2",
                                    states.Say(robot, ["Granny, I didn't hear you, please tell me wich bottles you want"]),
                                    transitions={   'spoken'            :'DESIGNATE_RANDOM_BOTTLE'})

            @smach.cb_interface(outcomes=['described', 'no_bottles'])
            def designate_random_bottle(userdata):
                bottle_description_map = bottle_description_map_desig.resolve() #Resolves to OrderedDict of EntityInfo:BottleDescription
                if bottle_description_map:
                    #import ipdb; ipdb.set_trace()
                    described_bottle.id = bottle_description_map.keys()[0].id #The ID of a random described bottle. [0] means first thing, second [0] is because keys are a (entity, y-coord)-tuple
                    return "described"
                else:
                    return 'no_bottles' #This cannot happen, since then DescribeBottles would have failed and we would not ask if Granny want a bottle. There are none
            smach.StateMachine.add( "DESIGNATE_RANDOM_BOTTLE",
                                    smach.CBState(designate_random_bottle),
                                    transitions={'described'            :"GRAB_BOTTLE",
                                                 'no_bottles'           :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            @smach.cb_interface(outcomes=['described', 'no_match'])
            def designate_bottle(userdata):
                """The DescribeBottles-state creates a mapping of entity:BottleDescription.
                Here, we get convert Granny's spoken description to a BottleDescription
                    and select the entity/entities that have that description"""
                answer = ask_bottles_answer.resolve()
                bottle_description_map = bottle_description_map_desig.resolve() #Resolves to OrderedDict of EntityInfo:BottleDescription
                if answer:
                    choices = answer.choices
                    grannies_desc = BottleDescription()
                    if 'color' in choices and choices['color'] != '': grannies_desc.color = choices['color']
                    if 'height_desc'  in choices and choices['height_desc']  != '': grannies_desc.height_description =  choices['height_desc']
                    if 'label' in choices and choices['label'] != '': grannies_desc.label = choices['label']
                    if 'position' in choices and choices['position'] != '': grannies_desc.position_description = choices['position']

                    # import ipdb; ipdb.set_trace()
                    matching_bottles = [bottle for bottle, bottle_desc in bottle_description_map.iteritems() if bottle_desc == grannies_desc]
                    if matching_bottles:
                        selected_bottle_id = matching_bottles[0].id #TODO: Select an easy to grasp one or try each one that matches the description
                        rospy.loginfo("Selected bottle.id {} ".format(selected_bottle_id))

                        described_bottle.id = selected_bottle_id
                        spoken_description = "the "
                        if 'color' in choices:
                            spoken_description += " " + choices['color']
                        if 'height_desc' in choices:
                            spoken_description += " " + choices['height_desc']
                        if 'label' in choices:
                            spoken_description += " labeled " + choices['label']

                        spoken_description += " one "

                        if 'position' in choices:
                            spoken_description += " on the " + choices['position']
                        # spoken_description += "with I D " + str(selected_bottle_id)[:5]

                        robot.speech.speak("OK, I will get {}".format(spoken_description))
                        rospy.loginfo("Granny chose & described: {0}".format(grannies_desc))
                        return 'described'
                    else:
                        rospy.logerr("There are no bottles matching {}".format(grannies_desc))
                        robot.speech.speak("Hey, there are no such bottles!")
                        return "no_match"
                else:
                    return "no_match"

            smach.StateMachine.add( "CONVERT_SPEECH_DESCRIPTION_TO_DESIGNATOR",
                                    smach.CBState(designate_bottle),
                                    transitions={'described'            :"GRAB_BOTTLE",
                                                 'no_match'             :"ASK_WHICH_BOTTLE"})

            # smach.StateMachine.add( "LOOKAT_CHOSEN_BOTTLE",
            #                          states.LookAtEntity(robot, locked_described_bottle),
            #                          transitions={  'succeeded'         :'GRAB_BOTTLE',
            #                                         'failed'            :'GRAB_BOTTLE'})

            # smach.StateMachine.add( "STOP_LOOKING",
            #                         Stop_looking(robot),
            #                         transitions={   'stopped_looking'   :'GRAB_BOTTLE'})

            smach.StateMachine.add( "GRAB_BOTTLE",
                                    Grab(robot, locked_described_bottle, empty_arm_designator),
                                    transitions={   'done'              :'GOTO_HANDOVER_GRANNY',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, "I couldn't grab the bottle, sorry Granny"),
                                    transitions={   'spoken'            :'GOTO_GRANNY_WITHOUT_BOTTLE_BACKUP_1'})

            smach.StateMachine.add( "GOTO_GRANNY_WITHOUT_BOTTLE",
                                    states.NavigateToSymbolic(robot, {granny:"near", room_designator:"in"}, granny),
                                    transitions={   'arrived'           :'failed',#DETECT_ACTION'
                                                    'unreachable'       :'GOTO_GRANNYS_TABLE_WITHOUT_BOTTLE',#DETECT_ACTION'
                                                    'goal_not_defined'  :'GOTO_GRANNYS_TABLE_WITHOUT_BOTTLE'})#DETECT_ACTION'

            smach.StateMachine.add('GOTO_GRANNY_WITHOUT_BOTTLE_BACKUP_1',
                                    states.NavigateToSymbolic(robot,
                                        {grannies_table: "in_front_of_pos2" },
                                                              grannies_table),
                                    transitions={   'arrived'           :   'failed',
                                                    'unreachable'       :   'GOTO_GRANNY_WITHOUT_BOTTLE',
                                                    'goal_not_defined'  :   'GOTO_GRANNY_WITHOUT_BOTTLE'})

            smach.StateMachine.add( "GOTO_GRANNYS_TABLE_WITHOUT_BOTTLE",
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", room_designator: "in"}, grannies_table),
                                    transitions={   'arrived'           :'failed',#DETECT_ACTION'
                                                    'unreachable'       :'failed',#DETECT_ACTION'
                                                    'goal_not_defined'  :'failed'})#DETECT_ACTION'

            # smach.StateMachine.add( "GOTO_HANDOVER_GRANNY",
            #                         #states.NavigateToPose(robot, 0, 0, 0),
            #                         states.NavigateToSymbolic(robot, {grannies_table:"near", ds.EntityByIdDesignator(robot, id=ROOM) : "in"}, grannies_table),
            #                         transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
            #                                         'unreachable'       :'GOTO_HANDOVER_GRANNY_BACKUP',
            #                                         'goal_not_defined'  :'SAY_HANDOVER_BOTTLE'})

            smach.StateMachine.add('GOTO_HANDOVER_GRANNY',
                                    states.NavigateToSymbolic(robot,
                                        {grannies_table: "in_front_of_pos2" },
                                                              grannies_table),
                                    transitions={   'arrived'           :   'SAY_HANDOVER_BOTTLE',
                                                    'unreachable'       :   'GOTO_HANDOVER_GRANNY_BACKUP',
                                                    'goal_not_defined'  :   'GOTO_HANDOVER_GRANNY_BACKUP'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_BACKUP",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", room_designator: "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
                                                    'unreachable'       :'SAY_HANDOVER_BOTTLE',
                                                    'goal_not_defined'  :'SAY_HANDOVER_BOTTLE'})

            # smach.StateMachine.add( "GOTO_GRANNY_WITH_BOTTLE",
            #                         states.NavigateToSymbolic(robot, {granny:"near", ds.EntityByIdDesignator(robot, id=ROOM):"in"}, granny),
            #                         transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
            #                                         'unreachable'       :'DETECT_ACTION',
            #                                         'goal_not_defined'  :'DETECT_ACTION'})

            smach.StateMachine.add( "SAY_HANDOVER_BOTTLE",
                                    states.Say(robot, ["Here are your pills, Granny.", "Granny, here are your pills."], block=False),
                                    transitions={   'spoken'            :'HANDOVER_TO_GRANNY'})

            smach.StateMachine.add('HANDOVER_TO_GRANNY',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'succeeded', #DETECT_ACTION',
                                                    'failed'            :'failed'}) #DETECT_ACTION'})


class HandleBlanket(smach.StateMachine):
    def __init__(self, robot, grannies_table, granny):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add( "PICKUP_BLANKET",
                                    states.Say(robot, [ "I would like to pick up your blanket, but I know I can't reach it. Sorry",
                                                        "Sorry, your blanket fell, but I can't reach it"]),
                                    transitions={   'spoken'            :'succeeded'})


class HandleFall(smach.StateMachine):
    def __init__(self, robot, grannies_table, granny):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        def size(entity):
            return abs(entity.z_max - entity.z_min) < 0.4

        def on_top(entity):
            container_entity = robot.ed.get_entity(id=GRANNIES_TABLE_KB)
            is_on_top = onTopOff(entity, container_entity)
            rospy.loginfo("{} is {}op top of {}".format(entity.id, {True:"", False:"NOT "}[is_on_top], container_entity.id))
            return is_on_top

        # Don't pass the weight_function, might screw up if phone is not near the robot
        phone = ds.EdEntityDesignator(robot, criteriafuncs=[size, on_top], debug=False, name="phone")
        empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator")
        arm_with_item_designator = ds.ArmDesignator(robot.arms, robot.arms['left'])  #ArmHoldingEntityDesignator(robot.arms, robot.arms['left']) #described_bottle, name="arm_with_item_designator")

        with self:
            smach.StateMachine.add( "SAY_FELL",
                                    states.Say(robot, "Oh no, you fell! I will give you the phone.", block=True),
                                    transitions={   'spoken' : 'GOTO_COUCHTABLE'})

            smach.StateMachine.add('GOTO_COUCHTABLE',
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", ds.EntityByIdDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived':'LOOKAT_COUCHTABLE',
                                                    'unreachable':'GOTO_COUCHTABLE_BACKUP',
                                                    'goal_not_defined':'GOTO_COUCHTABLE_BACKUP'})

            smach.StateMachine.add('GOTO_COUCHTABLE_BACKUP',
                                    states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=GRANNIES_TABLE_KB), radius=0.2),
                                    transitions={   'arrived':'LOOKAT_COUCHTABLE',
                                                    'unreachable':'LOOKAT_COUCHTABLE',
                                                    'goal_not_defined':'LOOKAT_COUCHTABLE'})

            smach.StateMachine.add("LOOKAT_COUCHTABLE",
                                     states.LookAtEntity(robot, grannies_table),
                                     transitions={  'succeeded'         :'SAY_TRY_GRAB_PHONE',
                                                    'failed'            :'SAY_TRY_GRAB_PHONE'})

            smach.StateMachine.add( "SAY_TRY_GRAB_PHONE",
                                    states.Say(robot, "I am trying to grab the phone.", block=False),
                                    transitions={   'spoken'            :'WAIT_TIME_TO_UPDATE_MODEL'})

            smach.StateMachine.add( "WAIT_TIME_TO_UPDATE_MODEL",
                                    states.WaitTime(robot, waittime=4),
                                    transitions={   'waited'    : 'STOP_LOOKING_COUCHTABLE',
                                                    'preempted' : 'STOP_LOOKING_COUCHTABLE'})

            smach.StateMachine.add("STOP_LOOKING_COUCHTABLE",
                                     Stop_looking(robot),
                                     transitions={  'stopped_looking'         :'GRAB_PHONE'})

            smach.StateMachine.add( "GRAB_PHONE",
                                    Grab(robot, phone, empty_arm_designator),
                                    transitions={   'done'              :'SAY_PHONE_TAKEN',
                                                    'failed'            :'SAY_PHONE_TAKEN_FAILED'})

            smach.StateMachine.add( "SAY_PHONE_TAKEN",
                                    states.Say(robot, "I have the phone!"),
                                    transitions={   'spoken'            :'GOTO_HANDOVER_GRANNY_PHONE'})

            smach.StateMachine.add( "SAY_PHONE_TAKEN_FAILED",
                                    states.Say(robot, "I am sorry, I was not able to get the phone, please try to get it yourself. You can do it! Good luck!"),
                                    transitions={   'spoken'            :'succeeded'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_PHONE",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", ds.EntityByIdDesignator(robot, id=ROOM) : "in"}, grannies_table),
                                    transitions={   'arrived'           :'SAY_HANDOVER_PHONE',
                                                    'unreachable'       :'GOTO_HANDOVER_GRANNY_PHONE_BACKUP',
                                                    'goal_not_defined'  :'SAY_HANDOVER_PHONE'})

            smach.StateMachine.add( "GOTO_HANDOVER_GRANNY_PHONE_BACKUP",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, {grannies_table:"near", ds.EntityByIdDesignator(robot, id=ROOM) : "in"}, grannies_table),
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
                                    transitions={   'spoken' :'succeeded'})


class HandleWalkAndSit(smach.StateMachine):
    def __init__(self, robot, grannies_table, granny):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        arm_for_cane = ds.UnoccupiedArmDesignator(robot.arms, robot.arms["left"], name="arm_for_cane")

        with self:
            smach.StateMachine.add( 'FOLLOW_GRANNY',
                                    states.FollowOperator(robot),
                                    transitions={   'stopped'       :'SAY_TAKE_CANE',
                                                    'no_operator'   : 'SAY_TAKE_CANE',
                                                    'lost_operator' :'SAY_TAKE_CANE'})

            smach.StateMachine.add( "SAY_TAKE_CANE",
                                    states.Say(robot, [ "You can give me the cane, granny"]),
                                    transitions={   'spoken'            :'HANDOVER_CANE'})

            smach.StateMachine.add( "HANDOVER_CANE",
                                    states.HandoverFromHuman(robot, arm_for_cane, grabbed_entity_label='walking_cane'),
                                    transitions={   'succeeded'            :'succeeded',
                                                    'failed'               :'CLOSE_GRIPPER_AFTER_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER_AFTER_FAIL',
                                    states.SetGripper(robot, arm_for_cane, gripperstate='close', timeout=1.0),
                                    transitions={'succeeded'               :'succeeded',
                                                 'failed'                  :'failed'})


class RespondToAction(smach.StateMachine):
    def __init__(self, robot, grannies_table, granny):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add( 'DETECT_ACTION',
                                    # DetectAction(robot, granny),
                                    # transitions={   "drop_blanket"      :"HANDLE_BLANKET",
                                    #                 "fall"              :"HANDLE_FALL",
                                    #                 "walk_and_sit"      :"HANDLE_WALK_AND_SIT"})
                                    DetectFallingGranny(robot, timeout=30),
                                    transitions={   "sit"       :"HANDLE_BLANKET",
                                                    "fall"      :"HANDLE_FALL",
                                                    "walk"      :"HANDLE_WALK_AND_SIT"})

            smach.StateMachine.add( "HANDLE_BLANKET",
                                    HandleBlanket(robot, grannies_table, granny),
                                    transitions={   'succeeded'            :'succeeded',
                                                    'failed'               :'failed'})

            smach.StateMachine.add( "HANDLE_FALL",
                                    HandleFall(robot, grannies_table, granny),
                                    transitions={   'succeeded'            :'succeeded',
                                                    'failed'               :'failed'})

            smach.StateMachine.add( "HANDLE_WALK_AND_SIT",
                                    HandleWalkAndSit(robot, grannies_table, granny),
                                    transitions={   'succeeded'            :'succeeded',
                                                    'failed'               :'failed'})


class RoboNurse(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])


        granny, grannies_table, shelf = define_designators(robot)

        def in_box(entity):
            print entity
            x_ok = 5.5 < entity.pose.position.x < 6.4
            y_ok = 8.1 < entity.pose.position.y < 8.5

            return x_ok and y_ok

        with self:
            smach.StateMachine.add( "INIT_WM",
                                    InitializeWorldModel(robot),
                                    transitions={'done'                 :'STORE_GRANNY_POSE'})

            smach.StateMachine.add( "STORE_GRANNY_POSE",
                                    StoreGrannyPose(robot, LASER_GRANNY.writeable),
                                    transitions={   'succeeded'         : 'START_PHASE',
                                                    'failed'            : 'START_PHASE'})

            smach.StateMachine.add( "START_PHASE",
                                    StartPhase(robot, grannies_table),
                                    transitions={   'Done'              :'ASK_GRANNY',
                                                    'Aborted'           :'Aborted'})

            smach.StateMachine.add( "ASK_GRANNY",
                                    states.Say(robot, ["What can I do for you?"], block=True),
                                    transitions={   'spoken'            :'HEAR_ANSWER'})

            smach.StateMachine.add('HEAR_ANSWER',
                                    states.Hear(robot, '(continue|i need my pills|(please) get me my pills)',time_out=rospy.Duration(10),look_at_standing_person=False),
                                    transitions={'heard':'GOTO_SHELF','not_heard':'GOTO_SHELF'})

            smach.StateMachine.add( "GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"in_front_of"}, shelf),
                                    transitions={   'arrived'           :'GET_PILLS',
                                                    'unreachable'       :'GET_PILLS',
                                                    'goal_not_defined'  :'GET_PILLS'})

            # smach.StateMachine.add('GOTO_SHELF',
            #                         states.NavigateToObserve(robot, shelf), # , radius=0.1
            #                         transitions={   'arrived':'GET_PILLS',
            #                                         'unreachable':'GOTO_SHELF_BACKUP',
            #                                         'goal_not_defined':'GOTO_SHELF_BACKUP'})

            smach.StateMachine.add('GOTO_SHELF_BACKUP',
                                    states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=BOTTLE_SHELF_WAYPOINT), radius=0.2),
                                    transitions={   'arrived':'GET_PILLS',
                                                    'unreachable':'GET_PILLS',
                                                    'goal_not_defined':'GET_PILLS'})

            smach.StateMachine.add( "GET_PILLS",
                                    GetPills(robot, shelf, grannies_table, granny),
                                    transitions={   'succeeded'     : "REST_ARMS_1",
                                                    'failed'        : "REST_ARMS_1"})

            smach.StateMachine.add( "REST_ARMS_1",
                                    states.ResetArms(robot),
                                    transitions={   'done'           :'GOTO_TABLE_DECTTTTE'})

            smach.StateMachine.add('GOTO_TABLE_DECTTTTE',
                                    states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id="granny_waypoint"), radius=0.2),
                                    transitions={   'arrived':'RESPOND_TO_ACTION',
                                                    'unreachable':'RESPOND_TO_ACTION',
                                                    'goal_not_defined':'RESPOND_TO_ACTION'})

            smach.StateMachine.add( "RESPOND_TO_ACTION",
                                    RespondToAction(robot, grannies_table, granny),
                                    transitions={   'succeeded'     :'Done',
                                                    'failed'        :'Failed'})

            # smach.StateMachine.add( "SAY_GO_BACK",
            #                         states.Say(robot, ["I'll just go back", "Heading back"], block=True),
            #                         transitions={   'spoken'            :'GO_BACK_TO_START'})

            # smach.StateMachine.add('GO_BACK_TO_START',
            #                         states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id="robonurse_initial"), radius=0.2),
            #                         transitions={   'arrived':'Done',
            #                                         'unreachable':'Done',
            #                                         'goal_not_defined':'Done'})

            ds.analyse_designators(self, "robonurse")


def test_look_at_entities(robot):
    shelves = ds.EdEntityCollectionDesignator(robot, criteriafuncs=[lambda e: "bookcase" in e.id], name="shelves")
    l = LookAtEntities(robot, shelves)
    l.execute(None)

def test_get_pills(robot):
    granny, grannies_table, shelf = define_designators(robot)
    getpills = GetPills(robot, shelf, grannies_table, granny)
    getpills.execute(None)



def test_describe_pills(robot):
    granny, grannies_table, shelf = define_designators(robot)

    empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.arms['left'], name="empty_arm_designator")
    arm_with_item_designator = ds.ArmDesignator(robot.arms, robot.arms['left'])  #ArmHoldingEntityDesignator(robot.arms, robot.arms['left']) #described_bottle, name="arm_with_item_designator")

    def small(entity):
        return abs(entity.z_max - entity.z_min) < 0.20

    def not_too_small(entity):
        return abs(entity.z_max - entity.z_min) > 0.03

    def minimal_height_from_floor(entity):
        return (entity.z_min + entity.pose.position.z)> 0.50

    def type_unknown_or_not_room(entity):
        return entity.type == "" or entity.type not in ["room"] or "shelf" not in entity.type

    def not_bookcase_part(entity):
        return not BOTTLE_SHELF in entity.id #Bookcase has elements named "bookcase/shelf1" etc. Ditch those

    bottle_shelf = ds.EntityByIdDesignator(robot, id=BOTTLE_SHELF, name="bottle_shelf")

    # import ipdb; ipdb.set_trace()
    def on_top(entity):
        container_entity = bottle_shelf.resolve()
        return onTopOff(entity, container_entity)

    bottle_criteria = [minimal_height_from_floor, type_unknown_or_not_room, not_bookcase_part, onTopOffForDesignator(bottle_shelf), small, not_too_small]

    # shelves = ds.EdEntityCollectionDesignator(robot, criteriafuncs=[lambda e: "bookcase" in e.id], name="shelves")
    bottles_to_describe = ds.EdEntityCollectionDesignator(robot, type="", criteriafuncs=bottle_criteria, debug=False, name="bottles_to_describe")
    described_bottle = ds.EdEntityDesignator(robot, debug=False) #ID will be decided by the description given by grann, name="described_bottle"y
    locked_described_bottle = ds.LockingDesignator(described_bottle, name="locked_described_bottle")

    # lookat = states.LookAtEntity(robot, bottle_shelf, waittime=1.0)
    # lookat.execute(None)

    ask_bottles_spec = ds.VariableDesignator(resolve_type=str, name="ask_bottles_spec")
    ask_bottles_choices = ds.VariableDesignator(resolve_type=dict, name="ask_bottles_choices")
    bottle_description_map_desig = ds.VariableDesignator(resolve_type=dict, name="bottle_description_map_desig")

    state_describe = DescribeBottles(robot, bottles_to_describe,
                                    spec_designator=ask_bottles_spec,
                                    choices_designator=ask_bottles_choices,
                                    bottle_desc_mapping_designator=bottle_description_map_desig)

    state_describe.execute(None)



def test_respond_to_action(robot):
    granny, grannies_table, shelf = define_designators(robot)
    respond = RespondToAction(robot, grannies_table, granny)
    respond.execute(None)

def xydistance(a, b):
    import math
    # print a, b
    return math.hypot(a[0]-b[0], a[1]-b[1])

def recognize_action(coordinates):
    #heuristic for stand up drop blanket: the z_max goes up and then down. x & y stay roughly the same (less than 1.0m)
    #heuristic for walking away: the x & y at start / end are far apart, more than 2m
    #heuristic for falling: the z_max gets very low, under 0.5m

    if len(coordinates) < 20: return None

    xy_dist = xydistance(coordinates[0], coordinates[-1])

    z_maxes = [coord[3] for coord in coordinates]
    z_max_min = min(z_maxes)
    z_max_max = max(z_maxes)
    z_max_diff = z_max_max - z_max_min

    xy_dist_large = xy_dist > 1.5 #start and end x&y are far apart
    xy_dist_small = xy_dist < 0.75 #start and end x&y are close together
    z_max_went_low = z_maxes[-1] < 0.5 #top of the object is very low

    z_max_went_up_and_back_down = z_maxes[0] < z_max_max and z_maxes[-1] < z_max_max and z_max_diff > 0.15

    fall = z_max_went_low
    drop_blanket = xy_dist_small and z_max_went_up_and_back_down and not fall
    walk_and_sit = xy_dist_large and not fall

    rospy.loginfo("z_max_min: {}, z_max_max: {}, z_max_diff: {}".format(z_max_min, z_max_max, z_max_diff))
    rospy.logwarn("xy_dist_large: {}, xy_dist_small: {}, z_max_went_low: {}, z+-: {}, ".format(xy_dist_large, xy_dist_small, z_max_went_low, z_max_went_up_and_back_down))

    if fall:
        # rospy.loginfo("Detected fall because z_max_went_low < 0.5: z_maxes[-1] = {}".format(z_maxes[-1]))
        return "fall"
    elif drop_blanket:
        return "drop_blanket"
    elif walk_and_sit:
        return "walk_and_sit"
    return None

def dummy_action_recognition(robot, max_measurements=200, _id=None):
    # import numpy as np
    from robot_skills.util import transformations
    import pandas as pd

    granny = ds.EdEntityDesignator(robot, type='human', name="granny")
    if _id:
        granny = ds.EntityByIdDesignator(robot, id=robot.ed.get_full_id(_id), name="granny")

    states.LookAtEntity(robot, granny).execute(None)

    action = None
    coords = []
    record = True
    # import ipdb; ipdb.set_trace()
    #break 600
    while record:
        try:
            entity = granny.resolve()
            p = transformations.tf_transform(entity.pose.position, entity.id, "/map", robot.tf_listener)
            # print str(p).replace('\n', ',')

            coords += [(p.x, p.y, p.z, entity.z_max)]

            if len(coords) > 1:
                action = recognize_action(coords)
                rospy.logwarn("Current action estimation: {}".format(action))
                # if action:
                #     record = False
            if len(coords) > max_measurements-1: record = False
        except KeyboardInterrupt, e:
            rospy.logwarn(e)
            record = False
        except AttributeError, e:
            rospy.logwarn(e)
            record = False #We lost the entity

    print "Detected action {} from {} coords".format(action, len(coords))

    coords_array = pd.DataFrame(coords, columns=["X", "Y", "Z", "Z_max"])
    import time
    timestr = time.strftime("%Y%m%d_%H%M%S")
    coords_array.to_csv("tracking_{}.csv".format(timestr))

    return action, coords




############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('robonurse_exec')

    startup(RoboNurse, challenge_name="robonurse")
