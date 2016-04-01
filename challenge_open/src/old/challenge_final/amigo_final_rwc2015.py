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

from robot_skills.util import transformations

#import data
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_open')

GRASP_LOC = []
PERSON_LOC = []
global NUMBER_OF_TRIES
NUMBER_OF_TRIES = 0

##### For current item designator #####
size = lambda entity: abs(entity.z_max - entity.z_min) < 0.5
has_type = lambda entity: entity.type != ""
min_height = lambda entity: entity.min_z > 0.1 #0.3 
min_entity_height = lambda entity: abs(entity.z_max - entity.z_min) > 0.1


def max_width(entity):
    max_bb_x = max(ch.x for ch in entity.convex_hull)
    min_bb_x = min(ch.x for ch in entity.convex_hull)
    max_bb_y = max(ch.y for ch in entity.convex_hull)
    min_bb_y = min(ch.y for ch in entity.convex_hull)

    x_size = abs(max_bb_x - min_bb_x)
    y_size = abs(max_bb_y - min_bb_y)

    x_ok = 0.02 < x_size < 0.2
    y_ok = 0.02 < y_size < 0.2

    return x_ok and y_ok
def weight_function(entity, robot):
    # TODO: return x coordinate of entity.center_point in base_link frame
    p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
    return p.x*p.x
#####


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
        #self.robot.ed.disable_plugins(plugin_names=["laser_integration"])

        return "done"

class SetInitialPose(smach.State):
    ## To call upon this state:
    # example 1: Set_initial_pose(robot, "front_of_door"),
    # OR
    # Set_initial_pose(robot, [1,0,0])
    def __init__(self, robot, init_position):
        smach.State.__init__(self, outcomes=["done", "preempted", "error"])

        self.robot = robot
        self.preempted = False

        self.initial_position = init_position

    def location_2d(self, location):
        e_loc = self.robot.ed.get_entity(id=location)

        if not e_loc:
            rospy.logerr("SetInitialPose: ED entity '" + location + "' does not exist.")
            return []

        print e_loc

        try:
            rz = transformations.euler_z_from_quaternion(e_loc.pose.orientation)
            print "rz = ", rz
        except KeyError:
            rz = 0

        return e_loc.pose.position.x, e_loc.pose.position.y, rz

    def execute(self, userdata):
        if isinstance(self.initial_position, str):
            x,y,phi = self.location_2d(self.initial_position)
        elif len(self.initial_position) == 3: #Tuple or list
            x = self.initial_position[0]
            y = self.initial_position[1]
            phi = self.initial_position[2]
        else:
            rospy.logerr("Initial pose {0} could not be set".format(self.initial_position))
            return "error"

        rospy.loginfo('Set initial pose to {0}, {1}, {2}'.format(x, y, phi))

        self.robot.base.set_initial_pose(x, y, phi)

        # Reset costmap: costmap is obviously entirely off if the localization was wrong before giving the initial pose
        # self.robot.base.reset_costmap()
        # Wait 0.5 s just to be sure
        rospy.sleep(rospy.Duration(0.5))

        return "done"

class FakeShutdownRobot(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()
        self.robot.spindle.low()
        self.robot.head.look_at_ground_in_front_of_robot(10)
        rospy.sleep(0.1)
        self.robot.head.look_at_ground_in_front_of_robot(2)
        rospy.sleep(0.1)
        self.robot.head.look_at_ground_in_front_of_robot(1)
        rospy.sleep(0.1)
        # Lichten worden niet altijd goed gezet. Vandaar even deze lelijke code:
        self.robot.lights.set_color(0,0,0,1)
        rospy.sleep(0.1)
        self.robot.lights.set_color(0,0,0,1)
        rospy.sleep(0.1)
        self.robot.lights.set_color(0,0,0,1)
        rospy.sleep(0.1)
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


                    # Query location to create the entity
                    e = self.robot.ed.get_entity(id='rwc2015/' + str(result.choices['grasp_location']) + '-0')
                    #FOR TESTING (CHECKCOMMENTED)
                    #e = self.robot.ed.get_entity(id=str(result.choices['grasp_location']))
                    if not e:
                        return 'failed'
                    PICKUPLOC_DESIGNATOR.writeable.write(e)
                    #PICKUPLOC_DESIGNATOR.writeable.write(result.choices['grasp_location'])
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
                    e = self.robot.ed.get_entity(id='rwc2015/' + str(result.choices['location']) + '-0')
                    #FOR TESTING (CHECKCOMMENTED)
                    #e = self.robot.ed.get_entity(id=str(result.choices['location']))
                    
                    if not e:
                        return 'failed'
                    BED_DESIGNATOR.writeable.write(e)
                    print result.choices['location']
                    #import ipdb; ipdb.set_trace()
                    BED_TYPE_DESIGNATOR.writeable.write(result.choices['location'])

                    #LOC_FIND_PERSON = result.choices['location']

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

    def _resolve(self):
        #import ipdb; ipdb.set_trace()
        # Get furniture entity
        furniture_id = self._furniture_designator.resolve()
        print "furniture_id 1 = ", furniture_id
        furniture_id = 'rwc2015/' + str(furniture_id) + '-0'
        print "furniture_id 2 = ", furniture_id

        ## FOR TESTING, no committing!!! (CHECKCOMMITTING)
        #furniture_id = furniture_id

        entities = self._robot.ed.get_entities()
        f = None
        for e in entities:
            if e.id == furniture_id:
                f = e
        if not f:
            rospy.logwarn('Entity with id {0} not found'.format(furniture_id))
            return None

        #self._robot.ed.enable_plugins(plugin_names=["laser_integration"])
        #rospy.sleep(0.1)
        person = self._robot.ed.get_closest_possible_person_entity(type="possible_human", center_point=f.pose.position, radius=10)

        #self._robot.ed.disable_plugins(plugin_names=["laser_integration"])
        if not person:
            rospy.logwarn('No person found near the {0}'.format(furniture_id))
            return None
        else:
            return person




class PickupObject(smach.StateMachine):

    def __init__(self, robot):

        #update_entity(self, id, type = None, posestamped = None, flags = None, add_flags = [], remove_flags = []

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        location_designator = PICKUPLOC_DESIGNATOR
        
        object_designator = VariableDesignator(resolve_type=str)

        point = msgs.Point(0, 0, 0)

        with self:
            # smach.StateMachine.add('STORE_POINT',
            #                         StorePoint(location_designator, point),
            #                         transitions={   'succeeded'         : 'FLAG_DYNAMIC',
            #                                         'failed'            : 'FLAG_DYNAMIC'})

            # # Flag entity to dynamic
            # smach.StateMachine.add('FLAG_DYNAMIC',
            #                         ChangeFlag(robot=robot, designator=location_designator, add_flags=['dynamic']),
            #                         transitions={   'succeeded'         : 'GOTO_LOCATION',
            #                                         'failed'            : 'GOTO_LOCATION'}) # ToDo: change backup???

            # First goto location
            smach.StateMachine.add('GOTO_LOCATION',
                                    states.NavigateToObserve(robot, entity_designator=location_designator, radius = 0.7),
                                    transitions={   'arrived'           : 'CHECK_POINT',
                                                    'unreachable'       : 'failed',
                                                    'goal_not_defined'  : 'SAY_LOCATION_UNKNOWN'})

            smach.StateMachine.add( "SAY_LOCATION_UNKNOWN",
                                    states.Say(robot, ["I'm sorry, but at this moment I do not know where this location is."], block=False),
                                    transitions={   'spoken'            :'failed'}) 


            smach.StateMachine.add('CHECK_POINT',
                                    CheckPoint(robot, location_designator, point),
                                    transitions={   'succeeded'         : 'UNFLAG_DYNAMIC',
                                                    'failed'            : 'UNFLAG_DYNAMIC'})

            smach.StateMachine.add('UNFLAG_DYNAMIC',
                                    ChangeFlag(robot=robot, designator=location_designator, remove_flags=['dynamic']),
                                    transitions={   'succeeded'         : 'GOTO_LOCATION2',
                                                    'failed'            : 'GOTO_LOCATION2'})

            # Second goto location: this is there to account for the possible movement of the furniture object
            smach.StateMachine.add('GOTO_LOCATION2',
                                    states.NavigateToObserve(robot, entity_designator=location_designator, radius = 0.7),
                                    transitions={   'arrived'           : 'MANIPULATE_ITEM',
                                                    'unreachable'       : 'failed',
                                                    'goal_not_defined'  : 'failed'})

            smach.StateMachine.add('MANIPULATE_ITEM',
                                    ManipRecogSingleItem(robot, location_designator=location_designator, object_designator=object_designator),
                                    transitions={   'succeeded'         : 'succeeded',
                                                    'failed'            : 'failed'})

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

    def __init__(self, robot, location_designator, object_designator):
        """@param manipulated_items is VariableDesignator that will be a list of items manipulated by the robot."""
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        # size = lambda entity: abs(entity.z_max - entity.z_min) < 0.4
        # has_type = lambda entity: entity.type != ""
        # min_entity_height = lambda entity: abs(entity.z_max - entity.z_min) > 0.04

        def on_top(entity):
            container_entity = location_designator.resolve()
            return onTopOff(entity, container_entity)

        # select the entity closest in x direction to the robot in base_link frame
        # def weight_function(entity):
        #     # TODO: return x coordinate of entity.center_point in base_link frame
        #     p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
        #     return p.x*p.x

        # current_item = LockingDesignator(EdEntityDesignator(robot,
        #     criteriafuncs=[size, has_type, on_top, min_entity_height], weight_function=weight_function, debug=False))
        current_item = EdEntityDesignator(robot)

        empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])

        # print "{0} = pick_shelf".format(pick_shelf)
        # print "{0} = current_item".format(current_item)
        # print "{0} = place_position".format(place_position)
        # print "{0} = empty_arm_designator".format(empty_arm_designator)
        # print "{0} = arm_with_item_designator".format(arm_with_item_designator)

        with self:

            ''' Look at pick shelf '''
            # smach.StateMachine.add("LOOKAT_PICK_SHELF",
            #                          states.LookAtEntity(robot, location_designator, keep_following=True),
            #                          transitions={  'succeeded'         :'SAY_LOOKAT_PICK_SHELF'})

            smach.StateMachine.add("SAY_LOOKAT_PICK_SHELF",
                                   states.Say(robot, ["Let's see what I can see here"]),
                                   transitions={   'spoken'            :'INSPECT_LOCATION'})

            smach.StateMachine.add("INSPECT_LOCATION",
                                   FindObjectOnFurniture(robot=robot, location_designator=location_designator, object_designator=object_designator, return_designator=current_item),
                                   transitions={    'found'         : 'GRAB_ITEM',
                                                    'not_found'     : 'RESET_HEAD_FAILED',
                                                    'failed'        : 'RESET_HEAD_FAILED'})

            smach.StateMachine.add( "GRAB_ITEM",
                                    states.Grab(robot, current_item, empty_arm_designator),
                                    transitions={   'done'              :'SAY_GRAB_SUCCEEDED',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_SUCCEEDED",
                                    states.Say(robot, ["Let's go back"], mood="excited", block=False),
                                    transitions={   'spoken'            :'RESET_HEAD_SUCCEEDED'}) 

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, ["I couldn't grab this thing"], mood="sad", block=False),
                                    transitions={   'spoken'            :'RESET_HEAD_FAILED'}) 

            # ToDo: is this necessary?
            smach.StateMachine.add( "RESET_HEAD_SUCCEEDED",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'GOTO_OPERATOR_SUCCEEDED'})

            # ToDo: is this necessary?
            smach.StateMachine.add( "RESET_HEAD_FAILED",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'RESET_ARMS_SPINDLE'})

            smach.StateMachine.add("RESET_ARMS_SPINDLE",
                                    states.ResetArmsSpindle(robot),
                                    transitions={   'done'            :'GOTO_OPERATOR_FAILED'})

            smach.StateMachine.add('GOTO_OPERATOR_SUCCEEDED',
                                    states.NavigateToObserve(robot=robot, entity_designator=BED_DESIGNATOR, radius = 0.7),
                                    transitions={   'arrived'           : 'GOTO_OPERATOR_SUCCEEDED_BACKUP',
                                                    'unreachable'       : 'GOTO_OPERATOR_SUCCEEDED_BACKUP',
                                                    'goal_not_defined'  : 'GOTO_OPERATOR_SUCCEEDED_BACKUP'})

            smach.StateMachine.add('GOTO_OPERATOR_SUCCEEDED_BACKUP',
                                    states.NavigateToObserve(robot=robot, entity_designator=LUIS_DESIGNATOR, radius = 0.7),
                                    transitions={   'arrived'           : 'SAY_GRASP_SUCCEEDED',
                                                    'unreachable'       : 'SAY_GRASP_SUCCEEDED',
                                                    'goal_not_defined'  : 'SAY_GRASP_SUCCEEDED'})

            smach.StateMachine.add('GOTO_OPERATOR_FAILED',
                                    states.NavigateToObserve(robot=robot, entity_designator=BED_DESIGNATOR, radius = 0.7),
                                    transitions={   'arrived'           : 'GOTO_OPERATOR_FAILED_BACKUP',
                                                    'unreachable'       : 'GOTO_OPERATOR_FAILED_BACKUP',
                                                    'goal_not_defined'  : 'GOTO_OPERATOR_FAILED_BACKUP'})

            smach.StateMachine.add('GOTO_OPERATOR_FAILED_BACKUP',
                                    states.NavigateToObserve(robot=robot, entity_designator=LUIS_DESIGNATOR, radius = 0.7),
                                    transitions={   'arrived'           : 'SAY_FAILED',
                                                    'unreachable'       : 'SAY_FAILED',
                                                    'goal_not_defined'  : 'SAY_FAILED'})


            smach.StateMachine.add( "SAY_GRASP_SUCCEEDED",
                                    states.Say(robot, ["Here you go, please take it from my hand"]),
                                    transitions={   'spoken'            : 'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( "SAY_FAILED",# ToDo: say failed
                                    states.Say(robot, ["I'm sorry but I did not get what you were asking for. I better go and do something else"]),
                                    transitions={   'spoken'            : 'failed'})

            smach.StateMachine.add( "HANDOVER_TO_HUMAN",
                                    states.HandoverToHuman(robot, arm_with_item_designator),
                                    transitions={   'succeeded'         : 'succeeded',
                                                    'failed'            : 'succeeded'})


class ChangeFlag(smach.State):
    """ Smach state to add or remove ED flags
    """
    def __init__(self, robot, designator, add_flags=[], remove_flags=[]):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.designator = designator
        self.add_flags = add_flags
        self.remove_flags = remove_flags

    def execute(self, userdata):
        e = self.designator.resolve()
        if e == None:
            return 'failed'

        self.robot.ed.update_entity(id=e.id, add_flags=self.add_flags, remove_flags=self.remove_flags)

        return 'succeeded'

class CheckPoint(smach.State):
    def __init__(self, robot, designator, point, distance=0.5):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.designator = designator
        self.point = point
        self.distance = distance
    def execute(self, ud):
        # Get entity
        entity = self.designator.resolve()
        if entity == None:
            return 'failed'

        # Compare position
        dist = math.hypot( (entity.pose.position.x - self.point.x), (entity.pose.position.y - self.point.y) )

        # Only say something if distance is larger than the threshold
        if dist > self.distance:
            obj_type = entity.type.split('/')[-1]
            self.robot.speech.speak('Hey, I can see the %s has move approximately %2i cm'%(obj_type, 100*dist), block=False)
        return 'succeeded'


class FindObjectOnFurniture(smach.State):
    """ Class to find an object designated by the object designator on a piece of furniture 
        designated by the location designator. The robot looks to the location designator and enables the segmentation plugin. 
        If not 
        If specified, the current id is stored in the return designator.
        Else, all objects are stored 
    """
    def __init__(self, robot, location_designator, object_designator, return_designator=None):
        """ Constructor

        :param robot robot object
        :param location_designator: EdEntityDesignator returning the furniture object
        :param object_designator: string designator returning the type of the object to look for 
        :param return_designator: EdEntityDesignator. If specified, the 'id' is set to one of the ID's found here. IF desired, more logic can be applied here
        """
        smach.State.__init__(self, outcomes=['found', 'not_found', 'failed'])
        self.robot = robot
        self.location_designator = location_designator
        self.object_designator = object_designator # (Obsolete)
        self.return_designator = return_designator

    def execute(self, userdate):
        # Move head)
        location_entity = self.location_designator.resolve()
        if location_entity == None:
            rospy.logerr('Do not know where to look for objects')
            return 'failed'

        cp = location_entity.pose.position
        self.robot.head.look_at_point(msgs.PointStamped(cp.x,cp.y,0.8,"/map"))
        height = min(0.4, max(0.1, 0.8-0.55)) # 0.8 was cp.z
        self.robot.torso._send_goal([height], timeout=5.0)

        rospy.sleep(2)

        ''' Enable kinect segmentation plugin (only one image frame) '''
        entity_ids = self.robot.ed.segment_kinect(max_sensor_range=2)

        print "entity_ids are :", entity_ids


        ''' Get all entities that are returned by the segmentation and are on top of the shelf '''
        id_list = [] # List with entities that are flagged with 'perception'                
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            if e and onTopOff(e, location_entity) and not e.type and size(e) and min_entity_height(e) and max_width(e):
                # ToDo: filter on size in x, y, z
                # self.robot.ed.update_entity(id=e.id, flags=[{"add":"perception"}])
                id_list.append(e.id)
                print "yes appended"

            print "e.type, size(e), min_entity_height(e), max_width(e):"
            print e.type
            print size(e)
            print min_entity_height(e)
            print max_width(e)

        # ToDo: add weight function???
        if len(id_list) > 0:
            self.return_designator.id = id_list[0]
            return 'found'
        else:
            return 'not_found'

        # ''' Try to classify the objects on the shelf '''
        # object_type = self.object_designator.resolve()
        # if object_type == None:
        #     rospy.logerr('Object type not specified')
        #     return 'failed'
        # entity_types = self.robot.ed.classify(ids=id_list, types=[object_type])

        ########## Testmode ######
        # if TESTMODE:
        #     if len(entity_types) > 0:
        #         entity_types[0] = object_type
        ###### End testmode ######3

        # ''' Zip the lists and sort '''
        # ziplist = zip(id_list, entity_types)
        # filterend_list = [z for z in ziplist if z[1] == object_type]

        # ''' If the filtered list is empty, the object has not been found '''
        # if len(filterend_list) == 0:
        #     rospy.loginfo('No entities of type {0} found on the {1}'.format(object_type, location_entity.id))
        #     return 'not_found'

        # ''' Else, set the id and return that the object has been found '''
        # if self.return_designator:
        #     self.return_designator.id = ziplist[0][0]
        # return 'found'


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
    global BAR_DESIGNATOR
    global BAR_TYPE_DESIGNATOR
    global OPERATOR_DESIGNATOR
    BAR_DESIGNATOR = EntityByIdDesignator(robot=robot, id='rwc2015/bar-0')

    ## DO NOT COMMIT THIS LINE UNCOMMETED BELOW, for testing (CHECKCOMMENTED)
    #BAR_DESIGNATOR = EntityByIdDesignator(robot=robot, id='bar')

    BAR_TYPE_DESIGNATOR = Designator(initial_value='bar', resolve_type=str) # Designator that returns a string with the bar type
    OPERATOR_DESIGNATOR = PersonDesignator(robot=robot, furniture_designator=BAR_TYPE_DESIGNATOR)      # Designator that returns the operator

    global BED_DESIGNATOR
    global BED_TYPE_DESIGNATOR
    global LUIS_DESIGNATOR
    # Driving to the operator (Sjoerd)
    BED_DESIGNATOR = VariableDesignator(resolve_type=EntityInfo)
    BED_TYPE_DESIGNATOR = VariableDesignator(resolve_type=str) # Designator that returns a string with the bar type

    LUIS_DESIGNATOR = PersonDesignator(robot=robot, furniture_designator=BED_TYPE_DESIGNATOR)      # Designator that returns the operator

    global PICKUPLOC_DESIGNATOR
    # Driving to the operator (Sjoerd)
    PICKUPLOC_DESIGNATOR = VariableDesignator(resolve_type=EntityInfo)

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
                                    WaitForEntity(robot,ed_entity_name='rwc2015/bar-0'),
                                    transitions={   'entity_exists'        :'FAKESTARTUP'})
    
        smach.StateMachine.add('FAKESTARTUP',
                                    FakeStartupRobot(robot),
                                    transitions={   'done':'FORCEDRIVE_TO_THE_RIGHT'})

        smach.StateMachine.add('FORCEDRIVE_TO_THE_RIGHT',
                                    ForceDriveToTheRight(robot),
                                    transitions={   'done':'WAIT_FOR_ENTITY_INITIAL_POSE'})

        smach.StateMachine.add("WAIT_FOR_ENTITY_INITIAL_POSE", 
                                    WaitForEntityInitialPose(robot,ed_entity_name=challenge_knowledge.initial_pose_amigo),
                                    transitions={   'entity_exists'                   : 'SET_INITIAL_POSE',
                                                    'initial_pose_to_be_set_manually' : 'GOTO_BAR_FOR_OPERATOR'})

        smach.StateMachine.add('SET_INITIAL_POSE',
                                SetInitialPose(robot, challenge_knowledge.initial_pose_amigo),
                                transitions={   'done'          :'GOTO_BAR_FOR_OPERATOR',
                                                'preempted'     :'GOTO_BAR_FOR_OPERATOR',
                                                'error'         :'GOTO_BAR_FOR_OPERATOR'})


        ######################################################
        ##################### ASK STATE  #####################             
        ######################################################


        smach.StateMachine.add("ASK_ACTION",
                                AskAction(robot),
                                transitions={'drive_near_loc_for_person':'ASK_LOCATION_PERSON',
                                             'grasp_object':'PICKUP_OBJECT',
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
        smach.StateMachine.add('GOTO_BAR_FOR_OPERATOR',
                                states.NavigateToObserve(robot=robot, entity_designator=BAR_DESIGNATOR, radius=1.4),
                                transitions={   'arrived'           : 'GOTO_OPERATOR',
                                                'unreachable'       : 'GOTO_OPERATOR',
                                                'goal_not_defined'  : 'GOTO_OPERATOR'})

        smach.StateMachine.add('GOTO_OPERATOR',
                                states.NavigateToObserve(robot=robot, entity_designator=OPERATOR_DESIGNATOR, radius=0.7),
                                transitions={   'arrived'           : 'ASK_ACTION',
                                                'unreachable'       : 'ASK_ACTION',
                                                'goal_not_defined'  : 'ASK_ACTION'})

        ''' GoTo Luis '''
        smach.StateMachine.add('GOTO_SECOND_OPERATOR_FURNITURE',
                                states.NavigateToObserve(robot=robot, entity_designator=BED_DESIGNATOR, radius=1.4),
                                transitions={   'arrived'           : 'GOTO_SECOND_OPERATOR',
                                                'unreachable'       : 'GOTO_SECOND_OPERATOR',
                                                'goal_not_defined'  : 'GOTO_SECOND_OPERATOR'})

        smach.StateMachine.add('GOTO_SECOND_OPERATOR',
                                states.NavigateToObserve(robot=robot, entity_designator=LUIS_DESIGNATOR, radius=0.7),
                                transitions={   'arrived'           : 'ASK_ACTION',
                                                'unreachable'       : 'ASK_ACTION',
                                                'goal_not_defined'  : 'ASK_ACTION'})
        ###################


        smach.StateMachine.add('PICKUP_OBJECT',
                                PickupObject(robot=robot),
                                transitions={   'succeeded' : 'ASK_ACTION',
                                                'failed'    : 'GOTO_OPERATOR'})

        # GOTO OPERATOR DONE IN PICKUPOBJECT
        # smach.StateMachine.add('GO_BACK_TO_OPERATOR',
        #                         states.NavigateToObserve(robot=robot, entity_designator=OPERATOR_DESIGNATOR, radius=0.7),
        #                         transitions={   'arrived'           : 'Done',
        #                                         'unreachable'       : 'Done',
        #                                         'goal_not_defined'  : 'Done'})






    analyse_designators(sm, "amigo_final_rwc2015")

    return sm

if __name__ == "__main__":
    rospy.init_node('amigo_final_rwc2015_exec')
    rospy.loginfo("-----------------------------------------------------------------")
    rospy.loginfo("----------------------- FINAL CHINA 2015 ------------------------")
    rospy.loginfo("----------------------------- AMIGO -----------------------------")
    rospy.loginfo("-----------------------------------------------------------------")
        

    states.util.startup(setup_statemachine, challenge_name="final")
