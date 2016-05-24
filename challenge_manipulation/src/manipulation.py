#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Manipulation.tex

In short, the robot starts at 1-1.5m from a bookcase and must wait until started by an operator (by voice or a start button)

This bookcase has a couple of shelves on which some items are placed.
**The middle shelve starts empty**, this is where the objects need to be placed.

The robot must take objects form the shelves and place them on the middle shelve and indicate the class of each grasped object.

After the robot is started by voice or a button,
    the ManipRecogSingleItem state machine is repeated at least 5 times (for 5 objects).
Afterwards, a PDF report has to be made:
'After the test is completed or the time has run out,
    the robot may upload a single PDF report file including the list of recognized objects with a picture showing:
    - the object,
    - the object name,
    - the bounding box of the object.'
"""

import rospy
import smach
import random

# ED
from ed_gui_server.msg import EntityInfo

# Robot Smach States
import robot_smach_states.util.designators as ds
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_smach_states.util.geometry_helpers import *

# Robot Skills
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations

# RoboCup knowledge
from robocup_knowledge import load_knowledge

# PDF writer
import pdf

challenge_knowledge = load_knowledge('challenge_manipulation')
CABINET = challenge_knowledge.cabinet
OBJECT_SHELVES = challenge_knowledge.object_shelves
PICK_SHELF = challenge_knowledge.grasp_shelf
PLACE_SHELF = challenge_knowledge.place_shelf
ROOM = challenge_knowledge.room
OBJECT_TYPES = challenge_knowledge.object_types
MAX_NUM_ENTITIES_IN_PDF = 10
MIN_GRASP_HEIGHT = challenge_knowledge.min_grasp_height
MAX_GRASP_HEIGHT = challenge_knowledge.max_grasp_height

DETECTED_OBJECTS_WITH_PROBS = []  # List with entities and types. This is used to write to PDF
SEGMENTED_ENTITIES = []  # List with segmented entities such that we can also grasp unknown entities

PREFERED_ARM="left"  # Must be "left" or "right"

DEBUG = False

''' Sanity check '''
if PLACE_SHELF in OBJECT_SHELVES:
    rospy.logerr("Place shelve {0} will not contain objects, but is still in object shelves, will remove".format(PLACE_SHELF))
# if PICK_SHELF not in OBJECT_SHELVES:
#     rospy.logerr("Pick shelf {0} not in object shelves, will add".format(PICK_SHELF))
#     OBJECT_SHELVES.append(PICK_SHELF)

ignore_ids = ['robotics_testlabs']
ignore_types = ['waypoint', 'floor', 'room']
PLACE_HEIGHT = 1.0

# Criteria
not_ignored = lambda entity: not entity.type in ignore_types and not entity.id in ignore_ids
size = lambda entity: abs(entity.z_max - entity.z_min) < 0.4
has_type = lambda entity: entity.type != ""
min_entity_height = lambda entity: abs(entity.z_max - entity.z_min) > 0.04

def max_width(entity):
    max_bb_x = max(ch.x for ch in entity.convex_hull)
    min_bb_x = min(ch.x for ch in entity.convex_hull)
    max_bb_y = max(ch.y for ch in entity.convex_hull)
    min_bb_y = min(ch.y for ch in entity.convex_hull)

    x_size = abs(max_bb_x - min_bb_x)
    y_size = abs(max_bb_y - min_bb_y)

    x_ok = 0.02 < x_size < 0.15
    y_ok = 0.02 < y_size < 0.15

    return x_ok and y_ok

# ----------------------------------------------------------------------------------------------------


class EntityDescriptionDesignator(ds.Designator):
    """EntityDescriptionDesignator"""
    def __init__(self, entity_designator, name=None):
        super(EntityDescriptionDesignator, self).__init__(resolve_type=str, name=name)
        self.entity_designator = entity_designator
        self.known_formats = "I'm trying to grab the {type}"
        self.unknown_formats = "I'm trying to grab this thing"

    def _resolve(self):
        entity = self.entity_designator.resolve()
        if not entity:
            return self.unknown_formats
        short_id = entity.id[:5]
        typ = entity.type
        if typ:
            sentence = self.known_formats.format(type=typ)
        else:
            sentence = self.unknown_formats
        return sentence

# ----------------------------------------------------------------------------------------------------


class InspectShelves(smach.State):
    """ Inspect all object shelves """

    def __init__(self, robot, object_shelves):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'nothing_found'])
        self.robot = robot
        self.object_shelves = object_shelves

    def execute(self, userdata):

        global SEGMENTED_ENTITIES
        global DETECTED_OBJECTS_WITH_PROBS

        ''' Get cabinet entity '''
        cabinet_entity = self.robot.ed.get_entity(id=CABINET, parse=True)

        ''' Get the pose of all shelves '''
        shelves = []
        for area in cabinet_entity.data['areas']:
            ''' See if the area is in the list of inspection areas '''
            if area['name'] in OBJECT_SHELVES:
                ''' Check if we have a shape '''
                if 'shape' not in area:
                    rospy.logwarn("No shape in area {0}".format(area['name']))
                    continue
                ''' Check if length of shape equals one '''
                if not len(area['shape']) == 1:
                    rospy.logwarn("Shape of area {0} contains multiple entries, don't know what to do".format(area['name']))
                    continue
                ''' Check if the first entry is a box '''
                if not 'box' in area['shape'][0]:
                    rospy.logwarn("No box in {0}".format(area['name']))
                    continue
                box = area['shape'][0]['box']
                if 'min' not in box or 'max' not in box:
                    rospy.logwarn("Box in {0} either does not contain min or max".format(area['name']))
                    continue

                x = 0.5 * (box['min']['x'] + box['max']['x'])
                y = 0.5 * (box['min']['y'] + box['max']['y'])
                z = 0.5 * (box['min']['z'] + box['max']['z'])
                shelves.append({'ps': geom.PointStamped(x, y, z, cabinet_entity.id), 'name': area['name']})
            else:
                rospy.loginfo("{0} not in object shelves".format(area['name']))

        # rospy.loginfo("Inspection points: {0}".format(shelves))
        # ''' Loop over shelves '''
        # for shelf in self.object_shelves:
        for shelf in shelves:

            ps = shelf['ps']
            cp = ps.point

            # ''' Get entities '''
            # shelf_entity = self.robot.ed.get_entity(id=shelf, parse=False)

            # if shelf_entity:

            # ''' Extract center point '''
            # cp = shelf_entity.pose.position

            ''' Look at target '''
            self.robot.head.look_at_point(ps)

            ''' Move spindle
                Implemented only for AMIGO (hence the hardcoding)
                Assume table height of 0.8 corresponds with spindle reset = 0.35 '''
            # def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
            # ToDo: do head and torso simultaneously
            height = min(0.4, max(0.1, cp.z-0.55))
            self.robot.torso._send_goal([height], timeout=5.0)

            ''' Sleep for 1 second '''
            import os; do_wait = os.environ.get('ROBOT_REAL')
            if do_wait == 'true':
                rospy.sleep(3.0) # ToDo: remove???
                rospy.logwarn("Do we have to wait this long???")

            if DEBUG:
                rospy.loginfo('Stopping: debug mode. Press c to continue to the next point')
                import ipdb;ipdb.set_trace()
                continue

            ''' Enable kinect segmentation plugin (only one image frame) '''
            # entity_ids = self.robot.ed.segment_kinect(max_sensor_range=2)  ## Old
            # segmented_entities = self.robot.ed.update_kinect("{} {}".format("on_top_of", shelf))
            segmented_entities = self.robot.ed.update_kinect("{} {}".format(shelf['name'], cabinet_entity.id))

            for id_ in segmented_entities.new_ids:
                entity = self.robot.ed.get_entity(id=id_, parse=False)  # In simulation, the entity type is not yet updated...
                SEGMENTED_ENTITIES.append((entity, id_))

            entity_types_and_probs = self.robot.ed.classify(ids=segmented_entities.new_ids, types=OBJECT_TYPES)

            # Recite entities
            for etp in entity_types_and_probs:
                self.robot.speech.speak("I have seen {0}".format(etp.type), block=False)

            # Lock entities
            self.robot.ed.lock_entities(lock_ids=[e.id for e in entity_types_and_probs], unlock_ids=[])

            # DETECTED_OBJECTS_WITH_PROBS = [(e.id, e.type) for e in entity_types_and_probs]
            # DETECTED_OBJECTS_WITH_PROBS = [(e.id, e.type) for e in sorted(entity_types_and_probs, key=lambda o: o[1], reverse=True)]
            for e in entity_types_and_probs:
                entity = self.robot.ed.get_entity(id=e.id, parse=False)  # In simulation, the entity type is not yet updated...
                DETECTED_OBJECTS_WITH_PROBS.append((entity, e.probability))

            # print "Detected obs with props 1: {0}".format(DETECTED_OBJECTS_WITH_PROBS)
            DETECTED_OBJECTS_WITH_PROBS = sorted(DETECTED_OBJECTS_WITH_PROBS, key=lambda  o: o[1], reverse=True)
            # print "Detected obs with props 2: {0}".format(DETECTED_OBJECTS_WITH_PROBS)

        if not DETECTED_OBJECTS_WITH_PROBS:
            return "nothing_found"

        # Sort based on probability
        # DETECTED_OBJECTS_WITH_PROBS = sorted(DETECTED_OBJECTS_WITH_PROBS, key=lambda o: o[1], reverse=True)

        return 'succeeded'

# ----------------------------------------------------------------------------------------------------

class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        # self.robot.ed.configure_kinect_segmentation(continuous=False)
        # self.robot.ed.configure_perception(continuous=False)
        self.robot.ed.disable_plugins(plugin_names=["laser_integration"])
        self.robot.ed.reset()

        return "done"

# ----------------------------------------------------------------------------------------------------

class ManipRecogSingleItem(smach.StateMachine):
    """The ManipRecogSingleItem state machine (for one object) is:
    - Stand of front of the bookcase
    - Look at the bookcase
    - Select an item, which is:
        - inside the bookcase
        - not yet grasped/not on the middle shelve
    - Grab that item
    - Say the class of the grabbed item
    - Place the item in an open spot on the middle shelve. """

    def __init__(self, robot, manipulated_items):
        """@param manipulated_items is VariableDesignator that will be a list of items manipulated by the robot."""
        self.manipulated_items = manipulated_items
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
        # self.place_shelf = ds.EntityByIdDesignator(robot, id=PLACE_SHELF, name="place_shelf")

        not_manipulated = lambda entity: not entity in self.manipulated_items.resolve()

        def detected(entity):
            """ Checks if the entity is in the (global) list of detected objects
            :param entity:
            :return:
            """
            # for tup in DETECTED_OBJECTS_WITH_PROBS:
            for tup in SEGMENTED_ENTITIES:
                if tup[0].id == entity.id:
                    return True
            return False

        def entity_z_pos(entity):
            """ Checks if the entity is between the minimum and maximum grasp height
            :param entity:
            :return:
            """
            if not entity.has_pose:
                return False
            return MIN_GRASP_HEIGHT < entity.pose.position.z < MAX_GRASP_HEIGHT



        # select the entity closest in x direction to the robot in base_link frame
        def weight_function(entity):
            # TODO: return x coordinate of entity.center_point in base_link frame
            p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
            return p.x*p.x

        self.current_item = ds.LockingDesignator(ds.EdEntityDesignator(robot,
            criteriafuncs=[not_ignored, size, not_manipulated, detected, min_entity_height, entity_z_pos, max_width],
            weight_function=weight_function, debug=False, name="item"), name="current_item")

        #This makes that the empty spot is resolved only once, even when the robot moves. This is important because the sort is based on distance between robot and constrait-area
        self.place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, self.cabinet, name="placement", area=PLACE_SHELF), name="place_position")

        if PREFERED_ARM == "left":
            prefered_arm = robot.leftArm
        elif PREFERED_ARM == "right":
            prefered_arm = robot.rightArm
        else:
            rospy.logwarn("Impossible preferred arm: {0}, defaulting to left".format(PREFERED_ARM))
            prefered_arm = robot.leftArm

        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, prefered_arm, name="empty_arm_designator")
        self.arm_with_item_designator = ds.ArmHoldingEntityDesignator(robot.arms, self.current_item, name="arm_with_item_designator")

        # print "{0} = pick_shelf".format(self.pick_shelf)
        # print "{0} = current_item".format(self.current_item)
        # print "{0} = place_position".format(self.place_position)
        # print "{0} = empty_arm_designator".format(self.empty_arm_designator)
        # print "{0} = arm_with_item_designator".format(self.arm_with_item_designator)

        with self:
            # smach.StateMachine.add( "NAV_TO_OBSERVE_PICK_SHELF",
            #                         #states.NavigateToObserve(robot, self.pick_shelf),
            #                         states.NavigateToSymbolic(robot, {self.pick_shelf:"in_front_of", EntityByIdDesignator(robot, id=ROOM):"in"}, self.pick_shelf),
            #                         transitions={   'arrived'           :'LOOKAT_PICK_SHELF',
            #                                         'unreachable'       :'LOOKAT_PICK_SHELF',
            #                                         'goal_not_defined'  :'LOOKAT_PICK_SHELF'})

            ''' Look at pick shelf '''
            smach.StateMachine.add("LOOKAT_PICK_SHELF",
                                     states.LookAtArea(robot, self.cabinet, area=PICK_SHELF),
                                     transitions={  'succeeded'         :'LOCK_ITEM'})

            @smach.cb_interface(outcomes=['locked'])
            def lock(userdata):
                self.current_item.lock() #This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                if self.current_item.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(self.current_item.resolve().id))

                self.place_position.lock() #This determines that self.place_position will lock/cache its result after its resolved the first time.
                return 'locked'
            smach.StateMachine.add('LOCK_ITEM',
                                   smach.CBState(lock),
                                   transitions={'locked':'ANNOUNCE_ITEM'})

            smach.StateMachine.add( "ANNOUNCE_ITEM",
                                    states.Say(robot, EntityDescriptionDesignator(self.current_item,
                                                                                  name="current_item_desc"),
                                               block=False),
                                    transitions={   'spoken'            :'GRAB_ITEM'})

            smach.StateMachine.add( "GRAB_ITEM",
                                    Grab(robot, self.current_item, self.empty_arm_designator),
                                    transitions={   'done'              :'STORE_ITEM',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, ["I couldn't grab this thing"], mood="sad"),
                                    transitions={   'spoken'            :'UNLOCK_ITEM_AFTER_FAILED_GRAB'}) # Not sure whether to fail or keep looping with NAV_TO_OBSERVE_PICK_SHELF

            @smach.cb_interface(outcomes=['unlocked'])
            def unlock_and_ignore(userdata):
                global ignore_ids
                # import ipdb; ipdb.set_trace()
                if self.current_item.resolve():
                    ignore_ids += [self.current_item.resolve().id]
                    rospy.loginfo("Current_item WAS now locked to {0}".format(self.current_item.resolve().id))
                self.current_item.unlock() #This determines that self.current_item can now resolve to a new value on the next call
                self.place_position.unlock() #This determines that self.place_position can now resolve to a new position on the next call
                return 'unlocked'
            smach.StateMachine.add('UNLOCK_ITEM_AFTER_FAILED_GRAB',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})

            @smach.cb_interface(outcomes=['stored'])
            def store_as_manipulated(userdata):
                # manipulated_items.current += [self.current_item.current]
                item_list = manipulated_items.resolve()
                item_list += [self.current_item.resolve()]
                w = ds.VariableWriter(manipulated_items)
                w.write(item_list)
                return 'stored'

            smach.StateMachine.add('STORE_ITEM',
                                   smach.CBState(store_as_manipulated),
                                   transitions={'stored':'LOOKAT_PLACE_SHELF'})

            smach.StateMachine.add("LOOKAT_PLACE_SHELF",
                                     states.LookAtArea(robot, self.cabinet, area=PLACE_SHELF),
                                     transitions={  'succeeded'         :'PLACE_ITEM'})

            smach.StateMachine.add( "PLACE_ITEM",
                                    Place(robot, self.current_item, self.place_position, self.arm_with_item_designator),
                                    transitions={   'done'              :'RESET_HEAD_PLACE',
                                                    'failed'            :'RESET_HEAD_HUMAN'})

            smach.StateMachine.add( "RESET_HEAD_PLACE",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE'})

            smach.StateMachine.add( "RESET_HEAD_HUMAN",
                                    states.CancelHead(robot),
                                    transitions={   'done'               :'SAY_HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'succeeded'})

            smach.StateMachine.add( "SAY_HANDOVER_TO_HUMAN",
                                    states.Say(robot, ["I'm can't get rid of this item  myself, can somebody help me maybe?"]),
                                    transitions={   'spoken'            :'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('HANDOVER_TO_HUMAN',
                                   states.HandoverToHuman(robot, self.arm_with_item_designator),
                                   transitions={   'succeeded'         :'UNLOCK_AFTER_HANDOVER',
                                                    'failed'           :'UNLOCK_AFTER_HANDOVER'})

            smach.StateMachine.add('UNLOCK_AFTER_HANDOVER',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    start_waypoint = ds.EntityByIdDesignator(robot, id="manipulation_init_pose", name="start_waypoint")
    placed_items = []

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INIT_WM',
                                                'abort':'Aborted'})

        smach.StateMachine.add("INIT_WM",
                               InitializeWorldModel(robot),
                               transitions={'done'                      :'AWAIT_START'})

        smach.StateMachine.add("AWAIT_START",
                               states.AskContinue(robot),
                               transitions={'continue'                  :'NAV_TO_START',
                                            'no_response'               :'AWAIT_START'})

        cabinet = ds.EntityByIdDesignator(robot, id=CABINET)
        room = ds.EntityByIdDesignator(robot, id=ROOM)
        smach.StateMachine.add( "NAV_TO_START",
                                states.NavigateToSymbolic(robot,
                                                          {cabinet:"in_front_of", room:"in"},
                                                          cabinet),
                                transitions={   'arrived'           :'RESET_ED',
                                                'unreachable'       :'RESET_ED',
                                                'goal_not_defined'  :'RESET_ED'})

        smach.StateMachine.add("RESET_ED",
                                states.ResetED(robot),
                                transitions={'done'                     :'INSPECT_SHELVES'})

        smach.StateMachine.add("INSPECT_SHELVES",
                                InspectShelves(robot, OBJECT_SHELVES),
                                transitions={'succeeded'                :'EXPORT_PDF',
                                             'nothing_found'            :'EXPORT_PDF',
                                             'failed'                   :'EXPORT_PDF'})

        @smach.cb_interface(outcomes=["exported"])
        def export_to_pdf(userdata):
            global DETECTED_OBJECTS_WITH_PROBS

            entities = [ e[0] for e in DETECTED_OBJECTS_WITH_PROBS ]

            # Export images (Only best MAX_NUM_ENTITIES_IN_PDF)
            pdf.entities_to_pdf(robot.ed, entities[:MAX_NUM_ENTITIES_IN_PDF], "tech_united_manipulation_challenge")

            return "exported"
        smach.StateMachine.add('EXPORT_PDF',
                                smach.CBState(export_to_pdf),
                                transitions={'exported':'RANGE_ITERATOR'})

        # Begin setup iterator
        range_iterator = smach.Iterator(    outcomes = ['succeeded','failed'], #Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it = lambda: range(5),
                                            it_label = 'index',
                                            exhausted_outcome = 'succeeded') #The exhausted argument should be set to the preffered state machine outcome

        with range_iterator:
            single_item = ManipRecogSingleItem(robot, ds.VariableDesignator(placed_items, [EntityInfo], name="placed_items"))

            smach.Iterator.set_contained_state( 'SINGLE_ITEM',
                                                single_item,
                                                loop_outcomes=['succeeded','failed'])

        smach.StateMachine.add('RANGE_ITERATOR', range_iterator,
                        {   'succeeded'                                     :'AT_END',
                            'failed'                                        :'Aborted'})
        # End setup iterator


        smach.StateMachine.add('AT_END',
                               states.Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})

        ds.analyse_designators(sm, "manipulation")

    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('manipulation_exec')

    startup(setup_statemachine)
