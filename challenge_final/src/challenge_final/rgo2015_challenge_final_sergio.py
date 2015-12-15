#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

import std_msgs

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states.util.geometry_helpers import *
from robot_skills.util import msg_constructors as msgs

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')
INITIAL_POSE = challenge_knowledge.initial_pose_sergio

MESH_IDS = [] #''' List with the IDs of the meshes of which a snapshot has been taken '''
SMALL_MESH_IDS = [] #   ''' List with the IDs of the small meshes which have been locked '''

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

    def execute(self, userdata):
        MESH_IDS.append('mesh{0}'.format(len(MESH_IDS))) # ToDo: make nice
        rospy.logwarn("Taking snapshot, id = {0}".format(MESH_IDS[-1]))
        self.robot.ed.mesh_entity_in_view(id=MESH_IDS[-1])
        rospy.sleep(rospy.Duration(3.0))
        return 'succeeded'

class AskWhatDoISee(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):

        self.robot.speech.speak("What do I see here?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.mesh_spec, choices=challenge_knowledge.mesh_choices, time_out = rospy.Duration(20))
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                name_object = res.choices['object']
                self.robot.speech.speak("Okay, I will call this the {0}".format(name_object))
                print "name_object = ", name_object
                
                ''' Assert type to ed '''
                if len(MESH_IDS) > 0:
                    rospy.logwarn("Mesh id: {0}, type: {1}".format(MESH_IDS[-1], name_object))
                    self.robot.ed.update_entity(id=MESH_IDS[-1], type = name_object)
                else:
                    rospy.logerr("Challenge final: Cannot update mesh type: id unknown")
                return "succeeded"
            else:
                self.robot.speech.speak("Sorry, I did not hear you properly")
                rospy.logerr("No speech result")
                return "failed"
        except KeyError:
            print "KEYERROR FINAL, should not happen!"
            return "failed"

class ExploreWaypoint(smach.StateMachine):
    def __init__(self, robot, waypoint, x=1.6, y=0, z=0):
        """
        waypoint: string with desired waypoint 
        x, y, z: head target in base link frame_id
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        waypoint_designator = EntityByIdDesignator(robot, id=waypoint)

        with self:
            smach.StateMachine.add("GOTO_WAYPOINT",
                                    states.NavigateToWaypoint(robot, waypoint_designator),
                                    transitions={   'arrived'                  :'LOOK_AT_MESH',
                                                    'unreachable'              :'LOOK_AT_MESH',
                                                    'goal_not_defined'         :'LOOK_AT_MESH'})

            ''' Look at thing '''
            smach.StateMachine.add("LOOK_AT_MESH",
                                    LookBaseLinkPoint(robot, x=x, y=y, z=z, timeout=5.0, waittime=3.0),
                                    transitions={   'succeeded'                 :'TAKE_SNAPSHOT',
                                                    'failed'                    :'TAKE_SNAPSHOT'})

            ''' Take snapshot '''
            smach.StateMachine.add("TAKE_SNAPSHOT",
                                    TakeSnapShot(robot),
                                    transitions={   'succeeded'                 :'LOOK_ASIDE',
                                                    'failed'                    :'LOOK_ASIDE'})

            ''' Look aside '''
            smach.StateMachine.add("LOOK_ASIDE",
                                    LookBaseLinkPoint(robot, x=1.0, y=2.0, z=1.8, timeout=0.0, waittime=0.0),
                                    transitions={   'succeeded'                 :'ASK_SEE',
                                                    'failed'                    :'ASK_SEE'})

            ''' Ask for entity '''
            smach.StateMachine.add("ASK_SEE",
                                    AskWhatDoISee(robot),
                                    transitions={   'succeeded'                 :'succeeded',
                                                    'failed'                    :'succeeded'})

############################## conversation with operator ######################

class ConversationWithOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.trigger_pub = rospy.Publisher("/amigo/trigger", std_msgs.msg.String, queue_size=10)

    def execute(self, userdata):

        self.robot.speech.speak("What can I do for you?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.operator_object_spec, choices=challenge_knowledge.operator_object_choices, time_out = rospy.Duration(20))
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                object_string = res.choices['object']
                self.robot.speech.speak("I am very sorry, but I do not have an arm to get a {0} for you. But my friend Amigo could get you one! I will call upon him!".format(object_string))
                self.robot.speech.speak("Amigo, please bring my boss a {0}".format(object_string))
                
                ''' Publish trigger for AMIGO to start its task '''
                msg = std_msgs.msg.String(object_string)
                counter = 0
                while counter < 10:
                    self.trigger_pub.publish(msg)
                    counter += 1
                    rospy.sleep(rospy.Duration(0.1))

                return "succeeded"
            else:
                self.robot.speech.speak("Sorry, I did not hear you properly")
                return "failed"
        except KeyError:
            print "KEYERROR FINAL, should not happen!"
            return "failed"

class HumanRobotInteraction(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        waypoint_designator = EntityByIdDesignator(robot, id=challenge_knowledge.task_location_sergio)

        with self:
            smach.StateMachine.add("GOTO_WAYPOINT",
                                    states.NavigateToWaypoint(robot, waypoint_designator),
                                    transitions={   'arrived'                   :'TALK_TO_OPERATOR',
                                                    'unreachable'               :'TALK_TO_OPERATOR',
                                                    'goal_not_defined'          :'TALK_TO_OPERATOR'})

            smach.StateMachine.add("TALK_TO_OPERATOR",
                                    ConversationWithOperator(robot),
                                    transitions={   'succeeded'                 :'succeeded',
                                                    'failed'                    :'succeeded'})

class CheckSmallObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['object_found','no_object_found'])
        self.robot = robot

    def execute(self, userdata):
        ''' For now, assume the ID of the mesh is always the latest that has been snapshot '''
        table_entity = self.robot.ed.get_entity(id=MESH_IDS[-1])
        if not table_entity:
            rospy.logerr("Not seeing an entity, returning no object found")
            return 'no_object_found'

        ''' Get all entities and check which one is on the table '''
        entities = self.robot.ed.get_entities()
        entities_on_table = []
        for entity in entities:
            if onTopOff(entity, table_entity):
                entities_on_table.append(entity)

        ''' If exactly one entity: perfect! '''
        small_mesh_id = None
        if len(entities_on_table) == 0:
            rospy.logwarn('No objects found on this table')
            return 'no_object_found'
        elif len(entities_on_table) == 1:
            small_mesh_id = entities_on_table[0].id
        else:
            z_pos_filtered_entities = []
            for entity in entities_on_table:
                z_pos = entity.pose.position.z
                if 0.6 < z_pos < 0.9:
                    z_pos_filtered_entities.append(entity)

            if len(z_pos_filtered_entities) == 0:
                rospy.logwarn("No entities left, assert first of original list")
                small_mesh_id = entities_on_table[0].id
            elif len(z_pos_filtered_entities) == 1:
                small_mesh_id = z_pos_filtered_entities[0].id
            else: 
                rospy.logwarn("multiple entities remaining, fingers crossed")
                small_mesh_id = z_pos_filtered_entities[0].id

        rospy.logwarn("Locking entity with ID: {0}".format(small_mesh_id))
        self.robot.ed.lock_entities(lock_ids=[small_mesh_id], unlock_ids=[])
        rospy.sleep(rospy.Duration(2.0))
        SMALL_MESH_IDS.append(small_mesh_id)
        return 'object_found'

class AskSmallObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):

        self.robot.speech.speak("What is this thing on the table?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.object_spec, choices=challenge_knowledge.object_choices, time_out = rospy.Duration(20))
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                name_object = res.choices['object']
                self.robot.speech.speak("Okay, now I know this is {0}".format(name_object))
                print "name_object = ", name_object
                
                ''' Assert type to ed '''
                if len(SMALL_MESH_IDS) > 0:
                    self.robot.ed.update_entity(id=SMALL_MESH_IDS[-1], type = name_object)
                else:
                    rospy.logerr("Challenge final: Cannot update small mesh type: id unknown")
                return "succeeded"
            else:
                self.robot.speech.speak("Sorry, I did not hear you properly")
                rospy.logerr("No speech result")
                return "failed"
        except KeyError:
            print "KEYERROR FINAL, should not happen!"
            return "failed"        

class SmallObjectHandling(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            
            ''' Wait for the operator to put something on the table '''
            smach.StateMachine.add("WAIT",
                                    states.WaitTime(robot, waittime=3.0),
                                    transitions={   'waited'                    :'SAY_MORE_TO_SEE',
                                                    'preempted'                 :'SAY_MORE_TO_SEE'})
                                                    
            ''' Say more to see '''
            smach.StateMachine.add("SAY_MORE_TO_SEE",
                                    states.Say(robot, 'Lets see if I can discover anything else'),
                                    transitions={   'spoken'                    :'LOOK_AT_MESH'})

            ''' Look at thing '''
            smach.StateMachine.add("LOOK_AT_MESH",
                                    LookBaseLinkPoint(robot, x=2.5, y=0, z=0, timeout=2.5, waittime=3.0),
                                    transitions={   'succeeded'                 :'CHECK_SMALL_OBJECT',
                                                    'failed'                    :'CHECK_SMALL_OBJECT'})

            ''' Check if object present and assert '''
            smach.StateMachine.add("CHECK_SMALL_OBJECT",
                                    CheckSmallObject(robot),
                                    transitions={   'object_found'              :'LOOK_ASIDE',
                                                    'no_object_found'           :'LOOK_ASIDE'})

            ''' If asserted, turn head '''
            smach.StateMachine.add("LOOK_ASIDE",
                                    LookBaseLinkPoint(robot, x=1.0, y=2.0, z=1.8, timeout=0.0, waittime=0.0),
                                    transitions={   'succeeded'                 :'ASK_SMALL_OBJECT',
                                                    'failed'                    :'ASK_SMALL_OBJECT'})

            ''' If asserted, ask what it is '''
            smach.StateMachine.add("ASK_SMALL_OBJECT",
                                    AskSmallObject(robot),
                                    transitions={   'succeeded'                 :'succeeded',
                                                    'failed'                    :'succeeded'})


############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
    	smach.StateMachine.add("INITIALIZE",
    							states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = False),
                                transitions={   "Done"              :   "EXPLORE1",
                                                "Aborted"           :   "EXPLORE1",
                                                "Failed"            :   "EXPLORE1"})

        smach.StateMachine.add("EXPLORE1",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_1),
                                transitions={   "succeeded"        :   "EXPLORE2",
                                                "failed"           :   "EXPLORE2"})

        smach.StateMachine.add("EXPLORE2",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_2),
                                transitions={   "succeeded"        :   "SMALL_OBJECT_1",
                                                "failed"           :   "SMALL_OBJECT_1"})

        smach.StateMachine.add("SMALL_OBJECT_1",
                                SmallObjectHandling(robot),
                                transitions={   "succeeded"         :   "EXPLORE3",
                                                "failed"            :   "EXPLORE3"})

        smach.StateMachine.add("EXPLORE3",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_3),
                                transitions={   "succeeded"        :   "SMALL_OBJECT_2",
                                                "failed"           :   "SMALL_OBJECT_2"})

        smach.StateMachine.add("SMALL_OBJECT_2",
                                SmallObjectHandling(robot),
                                transitions={   "succeeded"         :   "HUMAN_ROBOT_INTERACTION",
                                                "failed"            :   "HUMAN_ROBOT_INTERACTION"})

        smach.StateMachine.add("HUMAN_ROBOT_INTERACTION",
                                HumanRobotInteraction(robot),
                                transitions={   "succeeded"         :   "EXPLORE4",
                                                "failed"            :   "EXPLORE4"})

        smach.StateMachine.add("EXPLORE4",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_4),
                                transitions={   "succeeded"        :   "EXPLORE5",
                                                "failed"           :   "EXPLORE5"})

        smach.StateMachine.add("EXPLORE5",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_5),
                                transitions={   "succeeded"        :   "EXPLORE6",
                                                "failed"           :   "EXPLORE6"})

        smach.StateMachine.add("EXPLORE6",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_6),
                                transitions={   "succeeded"        :   "EXPLORE7",
                                                "failed"           :   "EXPLORE7"})

        smach.StateMachine.add("EXPLORE7",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_7),
                                transitions={   "succeeded"        :   "EXPLORE8",
                                                "failed"           :   "EXPLORE8"})

        smach.StateMachine.add("EXPLORE8",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_8),
                                transitions={   "succeeded"        :   "EXPLORE9",
                                                "failed"           :   "EXPLORE9"})

        smach.StateMachine.add("EXPLORE9",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_9),
                                transitions={   "succeeded"        :   "EXPLORE10",
                                                "failed"           :   "EXPLORE10"})

        smach.StateMachine.add("EXPLORE10",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_10),
                                transitions={   "succeeded"        :   "END_CHALLENGE",
                                                "failed"           :   "END_CHALLENGE"})

        smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, I am going to the kitchen and watch the crowd!", block=False),
                                   transitions={'spoken':'GOTO_FINAL_WAYPOINT'})

        smach.StateMachine.add("GOTO_FINAL_WAYPOINT",
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.end_location_sergio), radius=0.15),
                                transitions={   'arrived'                  :'Done',
                                                'unreachable'              :'Done',
                                                'goal_not_defined'         :'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_sergio')

    startup(setup_statemachine, robot_name='sergio')
