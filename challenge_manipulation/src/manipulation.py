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

# ED
from ed_robocup_msgs.srv import FitEntityInImage, FitEntityInImageRequest

# Robot Smach States
import robot_smach_states.util.designators as ds
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util.kdl_conversions import VectorStamped

# Robot Skills
from robot_skills.util.entity import Entity
from robot_skills import arms

# RoboCup knowledge
from robocup_knowledge import load_knowledge

from empty_shelf_designator import EmptyShelfDesignator

# PDF writer
import pdf

USE_SLAM = True  # Indicates whether or not to use SLAM for localization

challenge_knowledge = load_knowledge('challenge_manipulation')
if USE_SLAM:
    CABINET = challenge_knowledge.cabinet_slam
else:
    CABINET = challenge_knowledge.cabinet_amcl

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

PREFERRED_ARM = "left"  # Must be "left" or "right"

DEBUG = False

''' Sanity check '''
if PLACE_SHELF in OBJECT_SHELVES:
    rospy.logerr("Place shelve {0} will not contain objects, but is still in object shelves, "
                 "will remove".format(PLACE_SHELF))
# if PICK_SHELF not in OBJECT_SHELVES:
#     rospy.logerr("Pick shelf {0} not in object shelves, will add".format(PICK_SHELF))
#     OBJECT_SHELVES.append(PICK_SHELF)

ignore_ids = ['robotics_testlabs']
ignore_types = ['waypoint', 'floor', 'room']
PLACE_HEIGHT = 1.0

# Criteria
not_ignored = lambda entity: not entity.type in ignore_types and not entity.id in ignore_ids
size = lambda entity: abs(entity.shape.z_max - entity.shape.z_min) < 0.4
has_type = lambda entity: entity.type != ""
min_entity_height = lambda entity: abs(entity.shape.z_max - entity.shape.z_min) > 0.04

def max_width(entity):
    x_size = abs(entity.shape.x_max - entity.shape.x_min)
    y_size = abs(entity.shape.y_max - entity.shape.y_min)

    x_ok = 0.02 < x_size < 0.15
    y_ok = 0.02 < y_size < 0.15

    ok = x_ok and y_ok

    if not ok:
        rospy.logwarn("Entity(id={id}: x_size={x}, y_size={y}".format(x=x_size, y=y_size, id=entity.id))
    return ok

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


class ForceDrive(smach.State):
    """ Force drives... """
    def __init__(self, robot, vx, vy, vth, duration):
        """
        Constructor

        :param robot: robot object
        :param vx: velocity in x-direction
        :param vy: velocity in y-direction
        :param vth: yaw-velocity
        :param duration: float indicating how long to drive
        """
        smach.State.__init__(self, outcomes=['done'])
        self._robot = robot
        self._vx = vx
        self._vy = vy
        self._vth = vth
        self._duration = duration

    def execute(self, userdata=None):
        """ Executes the state """
        self._robot.base.force_drive(self._vx, self._vy, self._vth, self._duration)
        return 'done'

# ----------------------------------------------------------------------------------------------------


class ForceRotate(smach.State):
    """
    Force forth and back. If a timeout is exceeded, we won't do this anymore

    State is exited with
    - done: rotated back and forth
    - timedout: this takes too long
    """

    def __init__(self, robot, vth, duration, timeout):
        """
        Constructor

        :param robot: robot object
        :param vth: yaw-velocity
        :param duration: float indicating how long to drive
        :param timeout: after this, timedout is returned
        """
        smach.State.__init__(self, outcomes=['done', 'timedout'])
        self._robot = robot
        self._vth = vth
        self._duration = duration
        self._timeout = timeout
        self._first_stamp = None

    def execute(self, userdata=None):
        """ Executes the state """
        if self._first_stamp is None:
            self._first_stamp = rospy.Time.now()

        if (rospy.Time.now() - self._first_stamp).to_sec() > self._timeout:
            rospy.loginfo("ForceRotate timed out {0}, timeout is {1}...".format((rospy.Time.now() - self._first_stamp).to_sec(), self._timeout))
            return 'timedout'

        self._robot.base.force_drive(0, 0, self._vth, self._duration)
        self._vth = -self._vth
        self._robot.base.force_drive(0, 0, self._vth, self._duration)
        return 'done'


# ----------------------------------------------------------------------------------------------------


class FitEntity(smach.State):
    """ Fits an entity """

    def __init__(self, robot, entity_str):
        """
        Constructor

        :param robot: robot object
        :param entity_str: string with the entity type to fit
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._srv = rospy.ServiceProxy(robot.robot_name + '/ed/fit_entity_in_image', FitEntityInImage)
        self._entity_str = entity_str

    def execute(self, userdata=None):
        """ Executes the state """
        # Make sure the robot looks at the entity
        self._robot.head.reset()  # ToDo: this is abuse of the reset function
        self._robot.head.wait_for_motion_done(5.0)

        rospy.sleep(rospy.Duration(1.0))

        rospy.loginfo("Trying to fit...")

        # Try to fit the object
        req = FitEntityInImageRequest()
        req.entity_type = self._entity_str  # 1280 1024
        req.px = 0.5
        req.py = 0.5
        result = self._srv(req)

        # Cancel the head goal and return
        self._robot.head.cancel_goal()
        if False: # result.error_msg:
            rospy.logerr("Fit entity: {0}".format(result))
            return 'failed'
        else:
            return 'succeeded'


# ----------------------------------------------------------------------------------------------------


class InspectShelves(smach.State):
    """ Inspect all object shelves """

    def __init__(self, robot, object_shelves):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'nothing_found'])
        self.robot = robot
        self.object_shelves = object_shelves

    def execute(self, userdata=None):

        global SEGMENTED_ENTITIES
        global DETECTED_OBJECTS_WITH_PROBS

        ''' Get cabinet entity '''
        rospy.sleep(rospy.Duration(0.25))  # Sleep for a while to make
        # sure that the robot is actually in ED
        cabinet_entity = self.robot.ed.get_entity(id=CABINET, parse=True)

        ''' Get the pose of all shelves '''
        shelves = []
        for name, volume in cabinet_entity.volumes.iteritems():
            ''' See if the area is in the list of inspection areas '''
            if name in OBJECT_SHELVES:
                center_point = volume.center_point
                shelves.append({'vs': VectorStamped(vector=center_point, frame_id=cabinet_entity.id), 'name': name})
            else:
                rospy.loginfo("Volume {0} not in object shelves for entity {1}".format(name, cabinet_entity.id))

        # rospy.loginfo("Inspection points: {0}".format(shelves))
        # ''' Loop over shelves '''
        # for shelf in self.object_shelves:
        for shelf in shelves:

            vector_stamped = shelf['vs']
            center_vector = vector_stamped.vector

            # ''' Get entities '''
            # shelf_entity = self.robot.ed.get_entity(id=shelf, parse=False)

            # if shelf_entity:

            # ''' Extract center point '''
            # cp = shelf_entity.pose.position

            ''' Look at target '''
            self.robot.head.look_at_point(vector_stamped)

            ''' Move spindle
                Implemented only for AMIGO (hence the hardcoding)
                Assume table height of 0.8 corresponds with spindle reset = 0.35 '''
            # def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
            # ToDo: do head and torso simultaneously
            height = min(0.4, max(0.1, center_vector.z()-0.55))
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
        self.robot.ed.reset()

        return "done"

# ----------------------------------------------------------------------------------------------------


class RemoveSegmentedEntities(smach.State):
    """ Removes all entities that have no shape (except _root) """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):

        entities = self.robot.ed.get_entities(parse=False)

        for e in entities:
            if not e.is_a("furniture") and e.id != '_root':
                # import ipdb; ipdb.set_trace()
                self.robot.ed.remove_entity(e.id)

        return "done"

# ----------------------------------------------------------------------------------------------------


class SegmentShelf(smach.State):
    """ Segments the entities on a specific shelf. This assumes that the robot is already looking in the right
    direction. """
    def __init__(self, robot, entity_id, area_id):
        """
        :param robot: robot object
        :param entity_id: string with the id of the entity
        :param area_id: string with the id of the area
        """
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot
        self._entity_id = entity_id
        self._area_id = area_id

    def execute(self, userdata=None):
        self.robot.ed.update_kinect("{} {}".format(self._area_id, self._entity_id))
        rospy.sleep(rospy.Duration(0.5))  # Is this necessary???

        return 'done'

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

        def entity_z_pos(entity):
            """
            Checks if the entity is between the minimum and maximum grasp height

            :param entity:
            :return:
            """
            if not entity._pose:
                return False
            return MIN_GRASP_HEIGHT < entity._pose.p.z() < MAX_GRASP_HEIGHT

        # select the entity closest in x direction to the robot in base_link frame
        def weight_function(entity):
            # TODO: return x coordinate of entity.center_point in base_link frame
            epose = entity.pose.projectToFrame(robot.robot_name+"/base_link", robot.tf_listener)  # Get position in base_link
            p = epose.frame.p
            return p.x()**2

        self.current_item = ds.LockingDesignator(ds.EdEntityDesignator(robot,
                                                                       criteriafuncs=[not_ignored, size,
                                                                                      not_manipulated,
                                                                                      min_entity_height, entity_z_pos,
                                                                                      max_width],
                                                                       weight_function=weight_function, debug=False,
                                                                       name="item"), name="current_item")

        #This makes that the empty spot is resolved only once, even when the robot moves. This is important because the sort is based on distance between robot and constraint-area
        # self.place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, self.cabinet, name="placement", area=PLACE_SHELF), name="place_position")
        self.place_position = ds.LockingDesignator(EmptyShelfDesignator(robot, self.cabinet,
                                                                        name="placement", area=PLACE_SHELF),
                                                   name="place_position")

        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot,
                                                               {'required_trajectories': ['prepare_grasp'],
                                                                'required_goals': ['carrying_pose'],
                                                                'required_gripper_types': [arms.GripperTypes.GRASPING],
                                                                'required_arm_name': PREFERRED_ARM},
                                                               name="empty_arm_designator")
        self.arm_with_item_designator = ds.ArmHoldingEntityDesignator(robot,
                                                                      {'required_objects': [self.current_item]},
                                                                      {"required_trajectories": ["prepare_place"],
                                                                       "required_goals": ["reset", "handover_to_human"],
                                                                       'required_gripper_types': [
                                                                           arms.GripperTypes.GRASPING]},
                                                                      name="arm_with_item_designator")

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

            smach.StateMachine.add("REMOVE_ENTITIES",
                                   RemoveSegmentedEntities(robot=robot),
                                   transitions={'done': 'LOOKAT_PICK_SHELF'})

            smach.StateMachine.add("LOOKAT_PICK_SHELF",
                                     states.LookAtArea(robot, self.cabinet, area=PICK_SHELF),
                                     transitions={  'succeeded'         :'SEGMENT_SHELF'})

            smach.StateMachine.add("SEGMENT_SHELF",
                                   SegmentShelf(robot, entity_id=CABINET, area_id=PICK_SHELF),
                                   transitions={'done': 'LOCK_ITEM'})

            @smach.cb_interface(outcomes=['locked'])
            def lock(userdata=None):
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
            def unlock_and_ignore(userdata=None):
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
            def store_as_manipulated(userdata=None):
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
                               transitions={'initialized': 'INIT_WM',
                                            'abort': 'Aborted'})

        smach.StateMachine.add("INIT_WM",
                               InitializeWorldModel(robot),
                               transitions={'done': 'AWAIT_START'})

        # smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
        #                        states.Say(robot, ["Hi there, I will now wait until you remove the cup",
        #                                           "I'm waiting for you to remove the cup"], block=False),
        #                        transitions={"spoken": "WAIT_FOR_DOOR"})
        #
        # smach.StateMachine.add("WAIT_FOR_DOOR",
        #                        states.WaitForDoorOpen(robot, timeout=10),
        #                        transitions={"closed": "DOOR_CLOSED",
        #                                     "open": "AWAIT_START"})
        #
        # smach.StateMachine.add("DOOR_CLOSED",
        #                        states.Say(robot, ["I am waiting for you to remove the cup",
        #                                           "I'd start, if you remove the cup from my laser"]),
        #                        transitions={"spoken": "WAIT_FOR_DOOR"})

        if USE_SLAM:
            drive_state = "RESET_ED_SLAM"
        else:
            drive_state = "NAV_TO_START"
        smach.StateMachine.add("AWAIT_START",
                               states.AskContinue(robot),
                               transitions={'continue': drive_state,
                                            'no_response': 'AWAIT_START'})

        cabinet = ds.EntityByIdDesignator(robot, id=CABINET)
        room = ds.EntityByIdDesignator(robot, id=ROOM)

        if USE_SLAM:
            # vth = 1.0
            # smach.StateMachine.add("NAV_TO_FIT_POSE",
            #                        ForceDrive(robot, 0, 0, vth, 3.14/vth),
            #                        transitions={'done': 'FIT_ENTITY'})

            smach.StateMachine.add("RESET_ED_SLAM",
                                   states.ResetED(robot),
                                   transitions={'done': 'FIT_ENTITY'})

            smach.StateMachine.add("FIT_ENTITY",
                                   FitEntity(robot, CABINET),
                                   transitions={'succeeded': 'NAV_TO_START',
                                                'failed': 'SAY_FITTING_FAILED'})

            smach.StateMachine.add("SAY_FITTING_FAILED",
                                   states.Say(robot, ["Fitting the {0} failed, I will stop now.".format(CABINET)],
                                              mood="sad"),
                                   transitions={'spoken': 'Aborted'})

        smach.StateMachine.add( "NAV_TO_START",
                                states.NavigateToSymbolic(robot,
                                                          {cabinet: "in_front_of"},
                                                           cabinet),
                                transitions={   'arrived'           :'INSPECT_SHELVES',
                                                'unreachable'       :'FORCE_ROTATE',
                                                'goal_not_defined'  :'INSPECT_SHELVES'})

        smach.StateMachine.add("FORCE_ROTATE",
                               ForceRotate(robot, 0.5, 2.0, 30.0),
                               transitions={'done': "NAV_TO_START",
                                            'timedout': "INSPECT_SHELVES"})

        # smach.StateMachine.add("RESET_ED",
        #                         states.ResetED(robot),
        #                         transitions={'done'                     :'INSPECT_SHELVES'})

        smach.StateMachine.add("INSPECT_SHELVES",
                                InspectShelves(robot, OBJECT_SHELVES),
                                transitions={'succeeded'                :'EXPORT_PDF',
                                             'nothing_found'            :'EXPORT_PDF',
                                             'failed'                   :'EXPORT_PDF'})

        @smach.cb_interface(outcomes=["exported"])
        def export_to_pdf(userdata=None):
            global DETECTED_OBJECTS_WITH_PROBS

            entities = [ e[0] for e in DETECTED_OBJECTS_WITH_PROBS ]

            # Export images (Only best MAX_NUM_ENTITIES_IN_PDF)
            # pdf.entities_to_pdf(robot.ed, entities[:MAX_NUM_ENTITIES_IN_PDF], "tech_united_manipulation_challenge")

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
            single_item = ManipRecogSingleItem(robot, ds.VariableDesignator(placed_items, [Entity], name="placed_items"))

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
