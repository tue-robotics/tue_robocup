#!/usr/bin/python
import rospy
import smach
import sys
import random
import math
import numpy
import operator

import tf
import geometry_msgs.msg as gm
from visualization_msgs.msg import Marker

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

# Init pose AMIGO identifier
INITIAL_POSE_AMIGO = challenge_knowledge.initial_pose_amigo
OPERATOR_ID = challenge_knowledge.operator_id
EXIT_WAYPOINT_ID = challenge_knowledge.operator_id # Robot returns to operator at the end

# AMIGO starts 1.0 m left of SERGIO
ROBOTS_OFFSET = gm.Pose()

class StoreWaypoint(smach.State):
    """ Stores current position of the robot in ED as a waypoint, with id and offset
        PLEASE NOTE: ORIENTATION OFFSETS ARE NOT IMPLEMENTED
    """
    def __init__(self, robot, waypoint_id, offset=gm.Pose):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._waypoint_id = waypoint_id
        self._offset = offset
        self._pub = rospy.Publisher("/operator_waypoint", Marker, queue_size=1)

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        store_pose = self._robot.base.get_location()

        store_pose.pose.position.x += self._offset.position.x
        store_pose.pose.position.y += self._offset.position.y
        store_pose.pose.position.z += self._offset.position.z

        m = Marker()
        m.color.r = 1
        m.color.a = 1
        m.pose = store_pose.pose
        m.header = store_pose.header
        m.type = 0 #Arrow
        m.scale.x = 1.0
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.action = 0
        m.ns = "arrow"
        self._pub.publish(m)
        m.type = 9
        m.text = "operator pose"
        m.ns = "text"
        m.pose.position.z = 0.5
        self._pub.publish(m)

        # Store waypoint in world model
        print "\n\n\n\nCURRENT BASE POSE:\n\n\n"
        print store_pose
        print "\n\n\n"
        self._robot.ed.update_entity(id=self._waypoint_id, posestamped=store_pose, type="waypoint")

        return "done"


class ExplorationDesignator(EdEntityDesignator):
    """ Designator to determine the waypoint where the robot should go in its exploration phase 
        if no interesting point of interest is found
    """
    def __init__(self, robot):
        super(EdEntityDesignator, self).__init__(resolve_type=EntityInfo)
        self.robot = robot
        self.explored_ids = []

    def _resolve(self):
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

    def _resolve(self):
        resp = self.poi_srv()
        # Add new pois
        for i in range(len(self.pois), len(resp.pois)):
            poi = {'poi': resp.pois[i], 'poiid': '%i'%len(self.pois)}
            self.pois.append(poi)

        # Remove visited items
        filtered_pois = [poi for poi in self.pois if not poi['poiid'] in self.visited_ids]

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
        # else:
            # rospy.logwarn('wrong trigger received: %s', data.data)

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

            # ToDo: remove shutdown request???
            ''' Determine what to do '''
            smach.StateMachine.add('CHECK_TRIGGER',
                                    CheckCommand(robot=robot, triggers=['call_robot', 'exit_robot'], topic="/"+robot.robot_name+"/trigger", rate = 100, timeout=0.5),
                                    transitions={   'call_robot'        : 'call_received',
                                                    'exit_robot'        : 'shutdown_received',
                                                    'timeout'           : 'GOTO_POINT_OF_INTEREST',
                                                    'preempted'         : 'done'})

            ''' Go to point of interest '''
            smach.StateMachine.add('GOTO_POINT_OF_INTEREST',
                                    states.NavigateToObserve(robot=robot, entity_designator=poi_designator, radius = radius),
                                    transitions={   'arrived'           : 'LOOK_AT_OBJECT',
                                                    'unreachable'       : 'GOTO_POINT_OF_INTEREST',
                                                    'goal_not_defined'  : 'CHECK_TRIGGER'})

            ''' Look at thing (choose either of the two options below (second one not yet operational) '''
            smach.StateMachine.add("LOOK_AT_OBJECT",
                                    LookBaseLinkPoint(robot, x=radius, y=0, z=1.0, timeout=5.0, waittime=3.0),
                                    transitions={   'succeeded'         : 'TAKE_SNAPSHOT',
                                                    'failed'            : 'TAKE_SNAPSHOT'}) # ToDo: update waittime???
            # smach.StateMachine.add("LOOK_AT_OBJECT",
            #                          states.LookAtEntity(robot, pick_shelf, keep_following=True),
            #                          transitions={  'succeeded'         : 'TAKE_SNAPSHOT'})

            ''' Take snapshot '''
            smach.StateMachine.add("TAKE_SNAPSHOT",
                                    TakeSnapShot(robot),
                                    transitions={   'succeeded'                 :'CHECK_TRIGGER',
                                                    'failed'                    :'CHECK_TRIGGER'})

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


############################## gui callback state machine #####################
class GuiCallCallback(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        location_designator = VariableDesignator(resolve_type=EntityInfo)

        point = msgs.Point(0, 0, 0)

        with self:
            smach.StateMachine.add('GOTO_OPERATOR',
                                    states.NavigateToObserve(robot=robot, entity_designator=EntityByIdDesignator(robot=robot, id=INITIAL_POSE_AMIGO), radius = 1.0),
                                    transitions={   'arrived'           : 'SAY_GOTO_OPERATOR_FAILED',
                                                    'unreachable'       : 'SAY_GOTO_OPERATOR_FAILED',
                                                    'goal_not_defined'  : 'SAY_GOTO_OPERATOR_FAILED'})

            smach.StateMachine.add('SAY_GOTO_OPERATOR_FAILED',
                                    states.Say(robot, ["I do not have anything useful to do yet, so I will continue exploration"], block=False),
                                    transitions={   'spoken'            : 'succeeded'})


############################## state machine #############################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    ##### To start in a different state #####
    # if not TEST_GRASP_LOC == None:
    #     sm.set_initial_state(["HANDLE_GUI_CALL"])
    #########################################

    with sm:

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized'       : 'LOOK_IN_FRONT',
                                                'abort'             : 'Aborted'})

        smach.StateMachine.add('LOOK_IN_FRONT',
                                LookBaseLinkPoint(robot=robot, x=3, y=0, z=0, timeout=2.5, waittime=0.0),
                                transitions={   'succeeded'         : 'STORE_OPERATOR_WAYPOINT',
                                                'failed'            : 'STORE_OPERATOR_WAYPOINT'})# ToDo: abort???

        smach.StateMachine.add('STORE_OPERATOR_WAYPOINT',
                                StoreWaypoint(robot=robot, waypoint_id=INITIAL_POSE_AMIGO, offset=ROBOTS_OFFSET),
                                transitions={   'done'              : 'WAIT_FOR_COMMAND'})

        smach.StateMachine.add('WAIT_FOR_COMMAND',
                               states.WaitForTrigger(robot=robot, triggers=['call_robot'], topic="/"+robot.robot_name+"/trigger", rate=1.0),
                               transitions={   'call_robot'        : 'SAY_RECEIVED_CALL',
                                               'preempted'         : 'SAY_RECEIVED_CALL'})

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
                                states.NavigateToWaypoint(robot=robot, waypoint_designator=EntityByIdDesignator(robot, id=EXIT_WAYPOINT_ID), radius = 1.0),
                                transitions={   'arrived'           : 'Done',
                                                'unreachable'       : 'Done',
                                                'goal_not_defined'  : 'Done'})



    analyse_designators(sm, "sergio_final_rwc2015")
    return sm

############################## initializing program ######################
if __name__ == '__main__':

    rospy.init_node('sergio_final_challenge_exec')


    startup(setup_statemachine, challenge_name="final")
