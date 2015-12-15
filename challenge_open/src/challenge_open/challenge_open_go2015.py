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

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_open')
CABINET = challenge_knowledge.cabinet
TABLE1 = challenge_knowledge.table1
TABLE2 = challenge_knowledge.table2
TABLE3 = challenge_knowledge.table3
OBJECT_SHELVES = challenge_knowledge.object_shelves
LOOK_POSES = {}
LOCKED_ITEMS = {}

class LookRight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_point(msgs.PointStamped(x=0.2, y=1.0, z=1.8, frame_id=self.robot.robot_name+"/base_link"))
        return 'done'

class LockEntities(smach.State):
    """docstring for LockEntities"""
    def __init__(self, robot, table_id):
        smach.State.__init__(self, outcomes=['locked'])
        self.robot = robot
        self.table_id = table_id

    def execute(self, userdata):

        ''' Get table entity '''
        table_entity = self.robot.ed.get_entity(id=self.table_id, parse=False)

        ''' Get all entities '''
        entities = self.robot.ed.get_entities(parse=False)
        lock_entities = []
        for entity in entities:
            if onTopOff(entity, table_entity):
                self.robot.ed.lock_entities(lock_ids=[entity.id], unlock_ids=[])
                lock_entities.append(entity)

        LOCKED_ITEMS[self.table_id] = lock_entities

        return 'locked'


class StorePose(smach.State):
    def __init__(self, robot, id):
        smach.State.__init__(self, outcomes=['stored'])
        self.robot = robot
        self.id = id

    def execute(self, userdata):
        pose = self.robot.base.get_location()
        LOOK_POSES[id] = pose
        return 'stored'

class AskItems(smach.State):
    def __init__(self, robot, table_id):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.table_id = table_id

    def execute(self, userdata):

	self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What object do I see here?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.spec, choices=challenge_knowledge.choices, time_out = rospy.Duration(30))

        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                self.speak(res)
                self.addToED(res)
            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "succeeded"

    def addToED(self, res):
        ''' Find entities on location '''

        #entities = self.robot.ed.get_entities(parse=False)
        entities = LOCKED_ITEMS[self.table_id]

        ''' Check numbers??? '''
        ''' Backup scenario is difficult, operator should take care!!! '''

        ''' Transform to robot pose '''
        y_bl = {}
        pose_bl = self.robot.base.get_location()
        mat = self.pose_to_mat(pose_bl.pose)
        #t_result = R1_inv * t2 + t1_inv

        # Calculate inverse of robot pose
        mat_inv = mat.copy()
        mat_inv[0:3,0:3] = mat[0:3,0:3].getT()
        mat_inv[0:3, 3:] = mat_inv[0:3,0:3] * -mat[0:3, 3:]

        for entity in entities:
            if onTopOff(entity, self.robot.ed.get_entity(id=self.table_id)):
                entity_pose_mat_MAP = self.pose_to_mat(entity.pose)
                entity_pose_mat_BL = mat_inv * entity_pose_mat_MAP
                y_bl[entity.id] = entity_pose_mat_BL[1, 3]

        ''' Sort on y coordinate '''
        sorted_y_bl = sorted(y_bl.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

        ''' Assert to world model '''
        values = [ v for k,v in res.choices.iteritems() if "object" in k]

        updates = zip(sorted_y_bl, values)
        for update in updates:
            rospy.loginfo("Updating entity: {0} is {1}".format(update[0][0], update[1]))

            self.robot.ed.update_entity(id=update[0][0], type=update[1])

    def pose_to_mat(self, pose):
        '''Convert a pose message to a 4x4 numpy matrix.

        Args:
            pose (geometry_msgs.msg.Pose): Pose rospy message class.
        Returns:
            mat (numpy.matrix): 4x4 numpy matrix
        '''
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        pos = numpy.matrix([pose.position.x, pose.position.y, pose.position.z]).T
        mat = numpy.matrix(tf.transformations.quaternion_matrix(quat))
        mat[0:3, 3] = pos
        return mat

    def speak(self, res):
        # ToDo: make nice
        location = self.table_id
        if 'object3' in res.choices:
            object1  = res.choices['object1']
            object2  = res.choices['object2']
            object3  = res.choices['object3']
            self.robot.speech.speak("All right, so I can find {0}, {1} and {2} on the {3}".format(object1, object2, object3, location))
        elif 'object2' in res.choices:
            object1  = res.choices['object1']
            object2  = res.choices['object2']
            self.robot.speech.speak("Okay, so there are {0} and {1} on the {2}".format(object1, object2, location))
        else:
            object1  = res.choices['object1']
            self.robot.speech.speak("So I guess I see {0} on the {1}".format(object1, location))


# class CiteItems(smach.State):
#     """ Cite all items that have been found """
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=['succeeded','failed'])
#         self.robot = robot

#     def execute(self, userdata):

#         entities = self.robot.ed.get_entities()
#         if len(entities) == 0:
#             rospy.logerr("Something went terribly wrong, I have not found any entities")
#             self.robot.speech.speak("I must be blind")
#             return 'failed'

#         self.robot.speech.speak("I have found some items that I don't know")

#         surface = self.robot.ed.get_entity(id=TABLE1)
#         if surface:
#             count = 0
#             for entity in entities:
#                 if (onTopOff(entity, surface)):
#                     count += 1
#             self.robot.speech.speak("I have found {0} items on the {1}".format(count, TABLE1))

#         count = 0
#         for shelf in OBJECT_SHELVES:
#             surface = self.robot.ed.get_entity(id=shelf)
#             if surface:
#                 for entity in entities:
#                     if (onTopOff(entity, surface)):
#                         count += 1
#         self.robot.speech.speak("I have found {0} items in the bookcase".format(count))

#         surface = self.robot.ed.get_entity(id=TABLE2)
#         if surface:
#             count = 0
#             for entity in entities:
#                 if (onTopOff(entity, surface)):
#                     count += 1
#         self.robot.speech.speak("I have found {0} items on the {1}".format(count, TABLE2))

#         return 'succeeded'

# class InspectShelves(smach.State):
#     """ Inspect all object shelves """

#     def __init__(self, robot, object_shelves):
#         smach.State.__init__(self, outcomes=['succeeded','failed'])
#         self.robot = robot
#         self.object_shelves = object_shelves

#     def execute(self, userdata):

#         ''' Loop over shelves '''
#         for shelf in self.object_shelves:

#             rospy.loginfo("Shelf: {0}".format(shelf))

#             ''' Get entities '''
#             entity = self.robot.ed.get_entity(id=shelf, parse=False)

#             if entity:

#                 ''' Extract center point '''
#                 cp = entity.center_point

#                 ''' Look at target '''
#                 self.robot.head.look_at_point(msgs.PointStamped(cp.x,cp.y,cp.z,"/map"))

#                 ''' Move spindle
#                     Implemented only for AMIGO (hence the hardcoding)
#                     Assume table height of 0.8 corresponds with spindle reset = 0.35 '''
#                 # def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
#                 height = min(0.4, max(0.1, cp.z-0.55))
#                 self.robot.torso._send_goal([height], timeout=5.0)

#                 ''' Sleep for 1 second '''
#                 rospy.sleep(1.0)

#         return 'succeeded'

class ExploreTable(smach.StateMachine):

    def __init__(self, robot, table_id):

        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('GOTO_TABLE',
                                states.NavigateToSymbolic(robot,
                                                          {EntityByIdDesignator(robot, id=table_id):"in_front_of"},
                                                          EntityByIdDesignator(robot, id=table_id)),
                                transitions={   'arrived'           :   'STORE_TABLE',
                                                'unreachable'       :   'STORE_TABLE',
                                                'goal_not_defined'  :   'STORE_TABLE'})

            smach.StateMachine.add("STORE_TABLE",
                                    StorePose(robot, table_id),
                                    transitions={  'stored'         :   'LOOKAT_TABLE'})

            smach.StateMachine.add("LOOKAT_TABLE",
                                     states.LookAtEntity(robot, EntityByIdDesignator(robot, id=table_id), keep_following=True, waittime=2.0),
                                     transitions={  'succeeded'     :   'LOCK_ENTITIES'})

            smach.StateMachine.add("LOCK_ENTITIES",
                                    LockEntities(robot, table_id=table_id),
                                    transitions={   'locked'        :   'ASK_ITEMS'})

            smach.StateMachine.add("LOOK_RIGHT",
                                    LookRight(robot),
                                    transitions={   'done'          :   'ASK_ITEMS'})

            smach.StateMachine.add('ASK_ITEMS',
                                AskItems(robot, table_id),
                                transitions={   'succeeded'         :   'RESET_HEAD_TABLE',
                                                'failed'            :   'RESET_HEAD_TABLE'} )

            smach.StateMachine.add( "RESET_HEAD_TABLE",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'done'})

############################## explore state machine #####################
class ExploreScenario(smach.StateMachine):

    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('EXPLORE_TABLE1',
                                    ExploreTable(robot, TABLE1),
                                    transitions={   'done'          :   'EXPLORE_TABLE2'})

            smach.StateMachine.add('EXPLORE_TABLE2',
                                    ExploreTable(robot, TABLE2),
                                    transitions={   'done'          :   'EXPLORE_TABLE3'})

            smach.StateMachine.add('EXPLORE_TABLE3',
                                    ExploreTable(robot, TABLE3),
                                    transitions={   'done'          :   'done'})

############################## state machine #############################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:

        smach.StateMachine.add('SET_INITIAL_POSE',
                                states.SetInitialPose(robot, "open_challenge_start"),
                                transitions={   'done'          :'INITIALIZE',
                                                'preempted'     :'Aborted',
                                                'error'         :'Aborted'})

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized'   :'SAY_EXPLORE',
                                                'abort'         :'Aborted'})

        smach.StateMachine.add('SAY_EXPLORE',
                                states.Say(robot, ["I do not have much knowledge about this room, I better go and explore it"], block=False),
                                transitions={   'spoken'        :'EXPLORE'})

        smach.StateMachine.add('EXPLORE',
                                ExploreScenario(robot),
                                transitions={   'done'        :'GOTO_OPERATOR'})

        smach.StateMachine.add('GOTO_OPERATOR',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="open_challenge_start"), radius = 0.75),
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
