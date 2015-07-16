#!/usr/bin/python
import rospy
import smach
import sys
import random
import math
import numpy
import operator

import tf
import robot_smach_states as states
from robot_smach_states.util.designators import *
from robot_smach_states.util.startup import startup
from robot_skills.util import msg_constructors as msgs
from robot_skills.util import transformations
from robot_smach_states.util.geometry_helpers import *
from ed_sensor_integration.srv import GetPOIs, MakeSnapshot

from cb_planner_msgs_srvs.msg import *

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_open')
EXPLORATION_TARGETS = challenge_knowledge.exploration_targets

class ExplorationDesignator(EdEntityDesignator):
    """ Designator to determine the waypoint where the robot should go in its exploration phase 
        if no interesting point of interest is found
    """
    def __init__(self, robot):
        super(EdEntityDesignator, self).__init__(resolve_type=EntityInfo)
        self.robot = robot
        self.explored_ids = []

    def resolve(self):
        # Get entities
        entities = self.robot.ed.get_entities(type='waypoint')
        filtered_entities = []

        # Filter on id (needs to contain explore) and already visited
        for e in entities:
            if ("explore" in e.id) and not (e.id in self.explored_ids):
                filtered_entities.append(e)

        # Sort based on distance
        base_pose = self.robot.base.get_location()
        # sortf = lambda e: math.hypot( (e.pose.position.x-base_pose.pose.position.x), (e.pose.position.y-base_pose.pose.position.y) )
        sortf = lambda e: self.robot.base.global_planner.computePathLength(self.robot.base.global_planner.getPlan(PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(e.pose.position.x, e.pose.position.y, 0.1), frame="/map")))
        filtered_entities.sort(key=sortf)

        if len(filtered_entities) == 0:
            return None
        else:
            self.explored_ids.append(filtered_entities[0].id)
            return filtered_entities[0]

class PoiDesignator(EdEntityDesignator):
    """ Designator to select the point of interest to visit 
    """
    def __init__(self, robot, radius):
        super(EdEntityDesignator, self).__init__(resolve_type=EntityInfo)
        self.robot = robot
        self.radius = radius
        self.poi_srv = rospy.ServiceProxy('/%s/ed/get_pois'%robot.robot_name, GetPOIs) 
        self.pois = []
        self.visited_ids = []

    def resolve(self):
        resp = self.poi_srv()
        # Add new pois
        for i in range(len(self.pois), len(resp.pois)):
            poi = {'poi': resp.pois[i], 'poiid': '%i'%len(self.pois)}
            self.pois.append(poi)

        # Remove visited items
        filtered_pois = [poi for poi in self.pois if not poi['poiid'] in self.visited_ids]
        # filtered_pois = []
        # for poi in self.pois:
        #     if not poi['poiid'] in self.visited_ids:
        #         filtered_pois.append(poi)
        if len(filtered_pois) == 0:
            rospy.logwarn("No pois found")
            return None

        # Sort list
        def computePoiPathLength(poi):
            x = poi['poi'].point.x
            y = poi['poi'].point.y
            constraint="(x-%f)^2+(y-%f)^2 < %f^2 and (x-%f)^2+(y-%f)^2 > %f^2"%(x, y, self.radius+0.075, x, y, self.radius-0.075)
            pc = PositionConstraint(constraint=constraint, frame="/map")
            plan = self.robot.base.global_planner.getPlan(position_constraint=pc)
            length = self.robot.base.global_planner.computePathLength(plan)
            return length
        sortf = lambda poi: computePoiPathLength(poi)
        filtered_pois.sort(key=sortf)

        # Wrap first item in EntityInfo
        poi = filtered_pois[0]
        self.visited_ids.append(poi['poiid'])

        out = EntityInfo()
        poips = poi['poi']
        out.pose.position.x = poips.point.x 
        out.pose.position.y = poips.point.y
        out.pose.position.z = poips.point.z

        return out

class CheckCommand(smach.State):
    '''
    This state will block execution until a suitable trigger command is received on the channel /trigger
    It will receive std_msgs.String and will compare it to the strings in the array that is given.

    Example to wait for one of the strings 'allow' or 'deny' (could be sent from a gui):

        WaitForTrigger(robot, ['allow', 'deny'], /trigger"),
                       transitions={    'allow':     'DO_SOMETHING',
                                        'deny':      'DO_SOMETHING',
                                        'timout':    'TIMEOUT',
                                        'preempted': 'failed'})
    '''

    def __init__(self, robot, triggers, topic, rate = 1.0, timeout=0.0):
        smach.State.__init__(self,
                             outcomes=triggers+['timeout','preempted'])
        self.robot = robot
        self.triggers = triggers
        self.rate = rate
        self.timeout = timeout
        topic = topic

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback, queue_size=1)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

        self.trigger_received = None

    def execute(self, userdata):
        starttime = rospy.Time.now()
        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

            # Check timeout
            if (self.timeout > 0.0) and ( (rospy.Time.now() - starttime) > rospy.Duration(self.timeout) ):
                return 'timeout'

        if self.trigger_received:
            trigger = self.trigger_received
            self.trigger_received = None
            return trigger
        else:
            return 'preempted'

    def callback(self, data):
        if data.data in self.triggers:
            rospy.loginfo('trigger received: %s', data.data)
            self.trigger_received = data.data
        else:
            rospy.logwarn('wrong trigger received: %s', data.data)

class LookBaseLinkPoint(smach.State):
    def __init__(self, robot, x, y, z, timeout = 2.5, waittime = 0.0, endtime=20.0):
        """ 
        Sends a goal to the head in base link frame of the robot_name
        x, y, z: coordinates
        timeout: timeout of the call to the head ref action (hence is a maximum)
        waittime: additional waiting time 
        endtime: endtime which is passed to head ref 
        """
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.x = x
        self.y = y
        self.z = z
        self.timeout = timeout
        self.waittime = waittime
        self.endtime = endtime

    def execute(self, userdata):
        self.robot.head.look_at_point(msgs.PointStamped(x=self.x, y=self.y, z=self.z, frame_id=self.robot.robot_name+"/base_link"))
        #,            timeout=self.timeout, end_time=self.endtime)
        rospy.sleep(rospy.Duration(self.waittime))
        return 'succeeded'

class TakeSnapShot(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.snapshot_srv = rospy.ServiceProxy('/%s/ed/make_snapshot'%robot.robot_name, MakeSnapshot) 

    def execute(self, userdata):
        rospy.loginfo("Taking snapshot")
        self.snapshot_srv()
        return 'succeeded'

        
############################## explore state machine #####################
class ExploreScenario(smach.StateMachine):

    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['done', 'call_received', 'shutdown_received'])

        with self:

            radius = 1.5 # Radius for NavigateToExplore
            exploration_target_designator = ExplorationDesignator(robot)
            poi_designator = PoiDesignator(robot, radius)

            ''' Determine what to do '''
            smach.StateMachine.add('CHECK_TRIGGER',
                                    CheckCommand(robot=robot, triggers=['call_robot', 'exit_robot'], topic="/amigo/trigger", rate = 100, timeout=0.1),
                                    transitions={   'call_robot'        : 'call_received',
                                                    'exit_robot'        : 'shutdown_received',
                                                    'timeout'           : 'GOTO_POINT_OF_INTEREST',
                                                    'preempted'         : 'done'})

            ''' Go to point of interest '''
            smach.StateMachine.add('GOTO_POINT_OF_INTEREST',
                                    states.NavigateToObserve(robot=robot, entity_designator=poi_designator, radius = radius),
                                    transitions={   'arrived'           : 'LOOK_AT_OBJECT',
                                                    'unreachable'       : 'GOTO_POINT_OF_INTEREST',
                                                    'goal_not_defined'  : 'GOTO_HARDCODED_WAYPOINT'})

            ''' Backup: if no point of interest: go to hardcoded waypoint '''
            smach.StateMachine.add('GOTO_HARDCODED_WAYPOINT',
                                    states.NavigateToWaypoint(robot=robot, waypoint_designator=exploration_target_designator, radius = 0.15),
                                    transitions={   'arrived'           : 'CHECK_TRIGGER',
                                                    'unreachable'       : 'CHECK_TRIGGER',
                                                    'goal_not_defined'  : 'done'})

            ''' Look at thing '''
            smach.StateMachine.add("LOOK_AT_OBJECT",
                                    LookBaseLinkPoint(robot, x=radius, y=0, z=0, timeout=5.0, waittime=3.0),
                                    transitions={   'succeeded'                 :'TAKE_SNAPSHOT',
                                                    'failed'                    :'TAKE_SNAPSHOT'})

            ''' Take snapshot '''
            smach.StateMachine.add("TAKE_SNAPSHOT",
                                    TakeSnapShot(robot),
                                    transitions={   'succeeded'                 :'CHECK_TRIGGER',
                                                    'failed'                    :'CHECK_TRIGGER'})

####################################  STATES FOR GUI CALLBACK ####################################################333
class ConversationWithOperator(smach.State):
    def __init__(self, robot, furniture_designator, object_designator):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        # Designator where to look for the object
        self.furniture_designator = furniture_designator 
        # Object designator
        self.object_designator = object_designator

    def execute(self, userdata):

        # Ask what to to
        self.robot.speech.speak("What can I do for you?")

        # Get location options
        entities = self.robot.ed.get_entities(parse=False)
        # furniture_list = []
        # for e in entity:
        #     if 'furniture' in e.flags:
        #         furniture = e.type.split('/')[-1]
        #         furniture_list.append(furniture)

        # # Alternative
        ''' Maps the entities to strings containing the 'stripped type' '''
        furniture_list = {e:e.type.split('/')[-1] for e in entities if 'furniture' in e.flags}

        # Listen to result
        speech_options = {'object':challenge_knowledge.object_options, 'location': furniture_list.values()}
        res = self.robot.ears.recognize(spec=challenge_knowledge.speech_spec, choices=speech_options, time_out=rospy.Duration(10.0))

        # res = self.robot.ears.recognize(spec=challenge_knowledge.operator_object_spec, choices=challenge_knowledge.operator_object_choices, time_out = rospy.Duration(20))

        # Put result in designators
        if not res:
            ''' Get random furniture object (only the string) '''
            furniture = random.choice(furniture_list.keys())
            ''' Set the designator with the corresponding entity '''
            self.furniture_designator.current = furniture
            self.robot.speech.speak("My ears are not working properly, I'll go to the {0} to see what I can find there".format(furniture_list[furniture]))
            return "failed"
        else:
            obj = res.choices['object']
            loc = res.choices['location']
            self.robot.speech.speak("All right, I will go to the {0} to grab the {1}".format(loc, obj), block=False)
            for entity, stripped_type in furniture_list.iteritems():
                if stripped_type == loc:
                    self.furniture_designator.current = entity
            self.object_designator.current = obj
            rospy.logerr('ConversationWithOperator ALWAYS returns Failed for debugging purposes!!!')
            return 'failed'
        # Return
        # 
        # try:
        #     if res.result:
        #         object_string = res.choices['object']
        #         self.robot.speech.speak("I am very sorry, but I do not have an arm to get a {0} for you. But my friend Amigo could get you one! I will call upon him!".format(object_string))
        #         self.robot.speech.speak("Amigo, please bring my boss a {0}".format(object_string))
                
        #         ''' Publish trigger for AMIGO to start its task '''
        #         msg = std_msgs.msg.String(object_string)
        #         counter = 0
        #         while counter < 10:
        #             self.trigger_pub.publish(msg)
        #             counter += 1
        #             rospy.sleep(rospy.Duration(0.1))

        #         return "succeeded"
        #     else:
        #         self.robot.speech.speak("Sorry, I did not hear you properly")
        #         return "failed"
        # except KeyError:
        #     print "KEYERROR FINAL, should not happen!"
        #     return "failed"
class StorePoint(smach.State):
    def __init__(self, designator, point):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.designator = designator
        self.point = point
    def execute(self, ud):
        entity = self.designator.resolve()
        if entity == None:
            return 'failed'
        self.point.x = entity.pose.position.x
        self.point.y = entity.pose.position.y
        self.point.z = entity.pose.position.z
        return 'succeeded'

class CheckPoint(smach.State):
    def __init__(self, robot, designator, point, distance=0.5):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
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
            robot.speech.speak('Hey, I can see the %s has move approximately %2i cm'%(obj_type, 100*dist), block=False)
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
        self.object_designator = object_designator
        self.return_designator = return_designator

    def execute(self, userdate):
        # Move head)
        location_entity = self.location_designator.resolve()
        if location_entity == None:
            rospy.logerr('Do not know where to look for objects')
            return 'failed'

        cp = location_entity.pose.position
        self.robot.head.look_at_point(msgs.PointStamped(cp.x,cp.y,cp.z,"/map"))
        height = min(0.4, max(0.1, cp.z-0.55))
        self.robot.torso._send_goal([height], timeout=5.0)

        ''' Enable kinect segmentation plugin (only one image frame) '''
        entity_ids = self.robot.ed.segment_kinect(max_sensor_range=2)

        ''' Get all entities that are returned by the segmentation and are on top of the shelf '''
        id_list = [] # List with entities that are flagged with 'perception'                
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            if e and onTopOff(e, location_entity) and not e.type:
                # ToDo: filter on size in x, y, z
                # self.robot.ed.update_entity(id=e.id, flags=[{"add":"perception"}])
                id_list.append(e.id)

        ''' Try to classify the objects on the shelf '''
        object_type = self.object_designator.resolve()
        if object_type == None:
            rospy.logerr('Object type not specified')
            return 'failed'
        entity_types = self.robot.ed.classify(ids=id_list, types=[object_type])

        ''' Zip the lists and sort '''
        ziplist = zip(id_list, entity_types)
        filterend_list = [z for z in ziplist if z[1] == object_type]

        ''' If the filtered list is empty, the object has not been found '''
        if len(filterend_list) == 0:
            rospy.loginfo('No entities of type {0} found on the {1}'.format(object_type, location_entity.id))
            return 'not_found'

        ''' Else, set the id and return that the object has been found '''
        if self.return_designator:
            self.return_designator.id = ziplist[0][0]
        return 'found'


################### MANIPULATION STATE MACHINE ##########################################################

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

        size = lambda entity: abs(entity.z_max - entity.z_min) < 0.4
        has_type = lambda entity: entity.type != ""
        min_entity_height = lambda entity: abs(entity.z_max - entity.z_min) > 0.04

        def on_top(entity):
            container_entity = location_designator.resolve()
            return onTopOff(entity, container_entity)

        # select the entity closest in x direction to the robot in base_link frame
        def weight_function(entity):
            # TODO: return x coordinate of entity.center_point in base_link frame
            p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
            return p.x*p.x

        # current_item = EdEntityDesignator(robot, id="beer1")  # TODO: For testing only
        # current_item = LockingDesignator(EdEntityDesignator(robot,
        #     center_point=geom.PointStamped(frame_id="/"+PICK_SHELF), radius=2.0,
        #     criteriafuncs=[not_ignored, size, not_manipulated, has_type, on_top], debug=False))
        current_item = LockingDesignator(EdEntityDesignator(robot,
            criteriafuncs=[size, has_type, on_top, min_entity_height], weight_function=weight_function, debug=False))

        empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        arm_with_item_designator = ArmHoldingEntityDesignator(robot.arms, current_item)

        # print "{0} = pick_shelf".format(pick_shelf)
        # print "{0} = current_item".format(current_item)
        # print "{0} = place_position".format(place_position)
        # print "{0} = empty_arm_designator".format(empty_arm_designator)
        # print "{0} = arm_with_item_designator".format(arm_with_item_designator)

        with self:
            # smach.StateMachine.add( "NAV_TO_OBSERVE_PICK_SHELF",
            #                         #states.NavigateToObserve(robot, pick_shelf),
            #                         states.NavigateToSymbolic(robot, {pick_shelf:"in_front_of", EdEntityDesignator(robot, id=ROOM):"in"}, pick_shelf),
            #                         transitions={   'arrived'           :'LOOKAT_PICK_SHELF',
            #                                         'unreachable'       :'LOOKAT_PICK_SHELF',
            #                                         'goal_not_defined'  :'LOOKAT_PICK_SHELF'})

            ''' Look at pick shelf '''
            # smach.StateMachine.add("LOOKAT_PICK_SHELF",
            #                          states.LookAtEntity(robot, location_designator, keep_following=True),
            #                          transitions={  'succeeded'         :'SAY_LOOKAT_PICK_SHELF'})

            # smach.StateMachine.add("SAY_LOOKAT_PICK_SHELF",
            #                        states.Say(robot, ["I'm looking at the {0} to see what items I can find".format(location_designator.id.split('/')[-1])]),
            #                        transitions={   'spoken'            :'LOCK_ITEM'})

            smach.StateMachine.add("SAY_LOOKAT_PICK_SHELF",
                                   states.Say(robot, ["Let's see what I can see here"]),
                                   transitions={   'spoken'            :'INSPECT_LOCATION'})

            smach.StateMachine.add("INSPECT_LOCATION",
                                   FindObjectOnFurniture(robot=robot, location_designator=location_designator, object_designator=object_designator, return_designator=current_item),
                                   transitions={    'found'         : 'failed',
                                                    'not_found'     : 'RESET_HEAD_FAILED',
                                                    'failed'        : 'RESET_HEAD_FAILED'})

            # @smach.cb_interface(outcomes=['locked'])
            # def lock(userdata):
            #     current_item.lock() #This determines that current_item cannot not resolve to a new value until it is unlocked again.
            #     if current_item.resolve():
            #         rospy.loginfo("Current_item is now locked to {0}".format(current_item.resolve().id))

            #     place_position.lock() #This determines that place_position will lock/cache its result after its resolved the first time.
            #     return 'locked'
            # ToDo: replace: snapshot, segment, recognize, see if it's there
            # smach.StateMachine.add('LOCK_ITEM',
            #                        smach.CBState(lock),
            #                        transitions={'locked':'ANNOUNCE_ITEM'})

            # smach.StateMachine.add( "ANNOUNCE_ITEM",
            #                         states.Say(robot, EntityDescriptionDesignator("I'm trying to grab item {id} which is a {type}.", current_item), block=False),
            #                         transitions={   'spoken'            :'GRAB_ITEM'})
            # ToDo: new designator???
            smach.StateMachine.add( "GRAB_ITEM",
                                    states.Grab(robot, current_item, empty_arm_designator),
                                    transitions={   'done'              :'SAY_GRAB_SUCCEEDED',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_SUCCEEDED",
                                    states.Say(robot, ["Let's take this back to the boss"], mood="excited", block=False),
                                    transitions={   'spoken'            :'RESET_HEAD_SUCCEEDED'}) 

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, ["I couldn't grab this thing"], mood="sad", block=False),
                                    transitions={   'spoken'            :'RESET_HEAD_FAILED'}) 

            # @smach.cb_interface(outcomes=['unlocked'])
            # def unlock_and_ignore(userdata):
            #     global ignore_ids
            #     # import ipdb; ipdb.set_trace()
            #     if current_item.resolve():
            #         ignore_ids += [current_item.resolve().id]
            #         rospy.loginfo("Current_item WAS now locked to {0}".format(current_item.resolve().id))
            #     current_item.unlock() #This determines that current_item can now resolve to a new value on the next call
            #     place_position.unlock() #This determines that place_position can now resolve to a new position on the next call
            #     return 'unlocked'
            # smach.StateMachine.add('UNLOCK_ITEM_AFTER_FAILED_GRAB',
            #                        smach.CBState(unlock_and_ignore),
            #                        transitions={'unlocked'              :'failed'})

            # @smach.cb_interface(outcomes=['stored'])
            # def store_as_manipulated(userdata):
            #     manipulated_items.current += [current_item.current]
            #     return 'stored'

            # smach.StateMachine.add('STORE_ITEM',
            #                        smach.CBState(store_as_manipulated),
            #                        transitions={'stored':'ANNOUNCE_CLASS'})

            # smach.StateMachine.add( "ANNOUNCE_CLASS",
            #                         states.Say(robot, FormattedSentenceDesignator("This is a {item.type}.", item=current_item), block=False),
            #                         transitions={   'spoken'            :'LOOKAT_PLACE_SHELF'})

            # smach.StateMachine.add("LOOKAT_PLACE_SHELF",
            #                          states.LookAtEntity(robot, pick_shelf, keep_following=True),
            #                          transitions={  'succeeded'         :'PLACE_ITEM'})

            # smach.StateMachine.add( "PLACE_ITEM",
            #                         Place(robot, current_item, place_position, arm_with_item_designator),
            #                         transitions={   'done'              :'RESET_HEAD_PLACE',
            #                                         'failed'            :'RESET_HEAD_HUMAN'})

            # ToDo: is this necessary?
            smach.StateMachine.add( "RESET_HEAD_SUCCEEDED",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'GOTO_OPERATOR_SUCCEEDED'})

            # ToDo: is this necessary?
            smach.StateMachine.add( "RESET_HEAD_FAILED",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'GOTO_OPERATOR_FAILED'})

            smach.StateMachine.add('GOTO_OPERATOR_SUCCEEDED',
                                    states.NavigateToObserve(robot=robot, entity_designator=EdEntityDesignator(robot=robot, id=challenge_knowledge.operator_waypoint_id), radius = 1.0),
                                    transitions={   'arrived'           : 'SAY_GRASP_SUCCEEDED',
                                                    'unreachable'       : 'SAY_GRASP_SUCCEEDED',
                                                    'goal_not_defined'  : 'SAY_GRASP_SUCCEEDED'})

            smach.StateMachine.add('GOTO_OPERATOR_FAILED',
                                    states.NavigateToObserve(robot=robot, entity_designator=EdEntityDesignator(robot=robot, id=challenge_knowledge.operator_waypoint_id), radius = 1.0),
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

            # smach.StateMachine.add( "UNLOCK_AFTER_HANDOVER",
            #                         smach.CBState(unlock_and_ignore),
            #                         transitions={'unlocked'              :'failed'})

############################## gui callback state machine #####################
class GuiCallCallback(smach.StateMachine):

    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        location_designator = VariableDesignator(resolve_type=EntityInfo)
        object_designator = VariableDesignator(resolve_type=str)
        point = msgs.Point(0, 0, 0)

        with self:
            smach.StateMachine.add('GOTO_OPERATOR',
                                    states.NavigateToObserve(robot=robot, entity_designator=EdEntityDesignator(robot=robot, id=challenge_knowledge.operator_waypoint_id), radius = 1.0),
                                    transitions={   'arrived'           : 'HUMAN_ROBOT_INTERACTION',
                                                    'unreachable'       : 'SAY_GOTO_OPERATOR_FAILED',
                                                    'goal_not_defined'  : 'SAY_GOTO_OPERATOR_FAILED'})

            smach.StateMachine.add('SAY_GOTO_OPERATOR_FAILED',
                                    states.Say(robot, ["I was not able to reach my operator, please come to me"], block=False),
                                    transitions={   'spoken'            : 'HUMAN_ROBOT_INTERACTION'})

            smach.StateMachine.add('HUMAN_ROBOT_INTERACTION',
                                    ConversationWithOperator(robot=robot, furniture_designator=location_designator, object_designator=object_designator),
                                    transitions={   'succeeded'         : 'STORE_POINT',
                                                    'failed'            : 'STORE_POINT'})

            smach.StateMachine.add('STORE_POINT',
                                    StorePoint(location_designator, point),
                                    transitions={   'succeeded'         : 'GOTO_LOCATION',
                                                    'failed'            : 'GOTO_LOCATION'})

            # First goto location
            smach.StateMachine.add('GOTO_LOCATION',
                                    states.NavigateToObserve(robot, entity_designator=location_designator, radius = 0.7),
                                    transitions={   'arrived'           : 'CHECK_POINT',
                                                    'unreachable'       : 'failed',
                                                    'goal_not_defined'  : 'failed'})

            smach.StateMachine.add('CHECK_POINT',
                                    CheckPoint(robot, location_designator, point),
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

############################## state machine #############################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:

        # smach.StateMachine.add('SET_INITIAL_POSE',
        #                         states.SetInitialPose(robot, challenge_knowledge.initial_pose),
        #                         transitions={   'done'          :'INITIALIZE',
        #                                         'preempted'     :'Aborted',
        #                                         'error'         :'Aborted'})

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized'   :'SAY_EXPLORE',
                                                'abort'         :'Aborted'})

        smach.StateMachine.add('SAY_EXPLORE',
                                states.Say(robot, ["I do not have much knowledge about this room, I better go and explore it"], block=False),
                                transitions={   'spoken'        :'EXPLORE'})

        smach.StateMachine.add('EXPLORE',
                                ExploreScenario(robot),
                                transitions={   'done'              :   'HANDLE_GUI_CALL',
                                                'call_received'     :   'SAY_RECEIVED_CALL',
                                                'shutdown_received' :   'SAY_GOTO_EXIT'})

        smach.StateMachine.add('SAY_RECEIVED_CALL',
                                states.Say(robot, ["My operator called me, I better go and see what he wants"], block=False),
                                transitions={   'spoken'            :   'HANDLE_GUI_CALL'})

        smach.StateMachine.add('HANDLE_GUI_CALL',
                                GuiCallCallback(robot),
                                transitions={   'succeeded'         :   'EXPLORE',
                                                'failed'            :   'SAY_FAILURE'})

        smach.StateMachine.add('SAY_FAILURE',
                                states.Say(robot, ["Something went wrong, I'll go and explore some more"], block=False),
                                transitions={   'spoken'            :   'EXPLORE'})

        smach.StateMachine.add('SAY_GOTO_EXIT',
                                states.Say(robot, ["My work here is done, I'm leaving"], block=False),
                                transitions={   'spoken'            :   'GOTO_EXIT'})

        smach.StateMachine.add('GOTO_EXIT',
                                states.NavigateToWaypoint(robot=robot, waypoint_designator=EdEntityDesignator(robot, id=challenge_knowledge.exit_waypoint_id), radius = 1.0),
                                transitions={   'arrived'           : 'Done',
                                                'unreachable'       : 'Done',
                                                'goal_not_defined'  : 'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('open_challenge_exec')

    ''' Now, we will use AMIGO, but in the future we might change that '''
    robot_name = 'amigo'

    startup(setup_statemachine, robot_name=robot_name)
