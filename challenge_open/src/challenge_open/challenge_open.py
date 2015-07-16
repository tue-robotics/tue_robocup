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
# OBJECT_SHELVES = challenge_knowledge.object_shelves
# CABINET = challenge_knowledge.cabinet
# TABLE1 = challenge_knowledge.table1
# TABLE2 = challenge_knowledge.table2
# TABLE3 = challenge_knowledge.table3
# OBJECT_SHELVES = challenge_knowledge.object_shelves
# LOOK_POSES = {}
# LOCKED_ITEMS = {}

# class LookRight(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=['done'])
#         self.robot = robot

#     def execute(self, userdata):
#         self.robot.head.look_at_point(msgs.PointStamped(x=0.2, y=1.0, z=1.8, frame_id=self.robot.robot_name+"/base_link"))
#         return 'done'

# class LockEntities(smach.State):
#     """docstring for LockEntities"""
#     def __init__(self, robot, table_id):
#         smach.State.__init__(self, outcomes=['locked'])
#         self.robot = robot
#         self.table_id = table_id

#     def execute(self, userdata):

#         ''' Get table entity '''
#         table_entity = self.robot.ed.get_entity(id=self.table_id, parse=False)

#         ''' Get all entities '''
#         entities = self.robot.ed.get_entities(parse=False)
#         lock_entities = []
#         for entity in entities:
#             if onTopOff(entity, table_entity):
#                 self.robot.ed.lock_entities(lock_ids=[entity.id], unlock_ids=[])
#                 lock_entities.append(entity)

#         LOCKED_ITEMS[self.table_id] = lock_entities

#         return 'locked'


# class StorePose(smach.State):
#     def __init__(self, robot, id):
#         smach.State.__init__(self, outcomes=['stored'])
#         self.robot = robot
#         self.id = id

#     def execute(self, userdata):
#         pose = self.robot.base.get_location()
#         LOOK_POSES[id] = pose
#         return 'stored'

# class AskItems(smach.State):
#     def __init__(self, robot, table_id):
#         smach.State.__init__(self, outcomes=['succeeded','failed'])
#         self.robot = robot
#         self.table_id = table_id

#     def execute(self, userdata):

# 	self.robot.head.look_at_standing_person()

#         self.robot.speech.speak("What object do I see here?")

#         res = self.robot.ears.recognize(spec=challenge_knowledge.spec, choices=challenge_knowledge.choices, time_out = rospy.Duration(30))

#         if not res:
#             self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
#             return "failed"
#         try:
#             if res.result:
#                 self.speak(res)
#                 self.addToED(res)
#             else:
#                 self.robot.speech.speak("Sorry, could you please repeat?")
#                 return "failed"
#         except KeyError:
#             print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
#             return "failed"

#         return "succeeded"

#     def addToED(self, res):
#         ''' Find entities on location '''

#         #entities = self.robot.ed.get_entities(parse=False)
#         entities = LOCKED_ITEMS[self.table_id]

#         ''' Check numbers??? '''
#         ''' Backup scenario is difficult, operator should take care!!! '''

#         ''' Transform to robot pose '''
#         y_bl = {}
#         pose_bl = self.robot.base.get_location()
#         mat = self.pose_to_mat(pose_bl.pose)
#         #t_result = R1_inv * t2 + t1_inv

#         # Calculate inverse of robot pose
#         mat_inv = mat.copy()
#         mat_inv[0:3,0:3] = mat[0:3,0:3].getT()
#         mat_inv[0:3, 3:] = mat_inv[0:3,0:3] * -mat[0:3, 3:]

#         for entity in entities:
#             if onTopOff(entity, self.robot.ed.get_entity(id=self.table_id)):
#                 entity_pose_mat_MAP = self.pose_to_mat(entity.pose)
#                 entity_pose_mat_BL = mat_inv * entity_pose_mat_MAP
#                 y_bl[entity.id] = entity_pose_mat_BL[1, 3]

#         ''' Sort on y coordinate '''
#         sorted_y_bl = sorted(y_bl.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

#         ''' Assert to world model '''
#         values = [ v for k,v in res.choices.iteritems() if "object" in k]

#         updates = zip(sorted_y_bl, values)
#         for update in updates:
#             rospy.loginfo("Updating entity: {0} is {1}".format(update[0][0], update[1]))

#             self.robot.ed.update_entity(id=update[0][0], type=update[1])

#     def pose_to_mat(self, pose):
#         '''Convert a pose message to a 4x4 numpy matrix.

#         Args:
#             pose (geometry_msgs.msg.Pose): Pose rospy message class.
#         Returns:
#             mat (numpy.matrix): 4x4 numpy matrix
#         '''
#         quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#         pos = numpy.matrix([pose.position.x, pose.position.y, pose.position.z]).T
#         mat = numpy.matrix(tf.transformations.quaternion_matrix(quat))
#         mat[0:3, 3] = pos
#         return mat

#     def speak(self, res):
#         # ToDo: make nice
#         location = self.table_id
#         if 'object3' in res.choices:
#             object1  = res.choices['object1']
#             object2  = res.choices['object2']
#             object3  = res.choices['object3']
#             self.robot.speech.speak("All right, so I can find {0}, {1} and {2} on the {3}".format(object1, object2, object3, location))
#         elif 'object2' in res.choices:
#             object1  = res.choices['object1']
#             object2  = res.choices['object2']
#             self.robot.speech.speak("Okay, so there are {0} and {1} on the {2}".format(object1, object2, location))
#         else:
#             object1  = res.choices['object1']
#             self.robot.speech.speak("So I guess I see {0} on the {1}".format(object1, location))


# class ExploreTable(smach.StateMachine):

#     def __init__(self, robot, table_id):

#         smach.StateMachine.__init__(self, outcomes=['done'])


#         ObjectOfInterestDesignator = EdEntityDesignator(robot)

#         with self:
#             smach.StateMachine.add('GOTO_TABLE',
#                                 states.NavigateToSymbolic(robot,
#                                                           {EdEntityDesignator(robot, id=table_id):"in_front_of"},
#                                                           EdEntityDesignator(robot, id=table_id)),
#                                 transitions={   'arrived'           :   'STORE_TABLE',
#                                                 'unreachable'       :   'STORE_TABLE',
#                                                 'goal_not_defined'  :   'STORE_TABLE'})

#             smach.StateMachine.add("STORE_TABLE",
#                                     StorePose(robot, table_id),
#                                     transitions={  'stored'         :   'LOOKAT_TABLE'})

#             smach.StateMachine.add("LOOKAT_TABLE",
#                                      states.LookAtEntity(robot, EdEntityDesignator(robot, id=table_id), keep_following=True, waittime=2.0),
#                                      transitions={  'succeeded'     :   'LOCK_ENTITIES'})

#             smach.StateMachine.add("LOCK_ENTITIES",
#                                     LockEntities(robot, table_id=table_id),
#                                     transitions={   'locked'        :   'ASK_ITEMS'})

#             smach.StateMachine.add("LOOK_RIGHT",
#                                     LookRight(robot),
#                                     transitions={   'done'          :   'ASK_ITEMS'})

#             smach.StateMachine.add('ASK_ITEMS',
#                                 AskItems(robot, table_id),
#                                 transitions={   'succeeded'         :   'RESET_HEAD_TABLE',
#                                                 'failed'            :   'RESET_HEAD_TABLE'} )

#             smach.StateMachine.add( "RESET_HEAD_TABLE",
#                                     states.CancelHead(robot),
#                                     transitions={   'done'              :'done'})

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
        filtered_pois = []
        for poi in self.pois:
            if not poi['poiid'] in self.visited_ids:
                filtered_pois.append(poi)

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

        smach.StateMachine.__init__(self, outcomes=['done', 'call_received'])

        with self:
            # id_has_explore = lambda entity: "explore" in entity.id
            # wped = EdEntityDesignator(robot, type='waypoint', criteriafuncs=[id_has_explore]) # Helper designator to get Ed entity representing the target to explore
            # waypoint_designator = PointStampedOfEntityDesignator(wped)  # Designator to pass to the navigation state

            radius = 1.5 # Radius for NavigateToExplore
            exploration_target_designator = ExplorationDesignator(robot)
            poi_designator = PoiDesignator(robot, radius)

            ''' Determine what to do '''
            smach.StateMachine.add('CHECK_TRIGGER',
                                    CheckCommand(robot=robot, triggers=['call'], topic="robot_call", rate = 100, timeout=0.1),
                                    transitions={   'call'              : 'call_received',
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
                                    transitions={   'arrived'           : 'GOTO_HARDCODED_WAYPOINT',
                                                    'unreachable'       : 'GOTO_HARDCODED_WAYPOINT',
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

        # with self:
        #     smach.StateMachine.add('EXPLORE_TABLE1',
        #                             ExploreTable(robot, TABLE1),
        #                             transitions={   'done'          :   'EXPLORE_TABLE2'})

        #     smach.StateMachine.add('EXPLORE_TABLE2',
        #                             ExploreTable(robot, TABLE2),
        #                             transitions={   'done'          :   'EXPLORE_TABLE3'})

        #     smach.StateMachine.add('EXPLORE_TABLE3',
        #                             ExploreTable(robot, TABLE3),
        #                             transitions={   'done'          :   'done'})

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
                                transitions={   'done'              :   'GOTO_OPERATOR',
                                                'call_received'     :   'Done'})

        smach.StateMachine.add('GOTO_OPERATOR',
                                states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="open_challenge_start"), radius = 0.75),
                                transitions={   'arrived'           :   'Done',
                                                'unreachable'       :   'Done',
                                                'goal_not_defined'  :   'Done'})

        # smach.StateMachine.add('CITE_UNKNOWN_ITEMS',
        #                         CiteItems(robot),
        #                         transitions={   'succeeded'         :   'ASK_ITEMS1',
        #                                         'failed'            :   'ASK_ITEMS1'} )

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('open_challenge_exec')

    ''' Now, we will use AMIGO, but in the future we might change that '''
    robot_name = 'amigo'

    startup(setup_statemachine, robot_name=robot_name)
