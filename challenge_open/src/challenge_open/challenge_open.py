#!/usr/bin/python
import rospy
import smach
import sys
import random
import math
import numpy
import operator

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
OBJECT_SHELVES = challenge_knowledge.object_shelves
LOOK_POSES = {}

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
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What can you tell me?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.spec, choices=challenge_knowledge.choices, time_out = rospy.Duration(30))
        self.robot.head.cancel_goal()
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                self.speak(res)
            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "succeeded"

    def addToED(self, res):
        ''' Find entities on location '''
        entities = self.robot.ed.get_entities(parse=False)

        ''' Check numbers??? '''
        ''' Backup scenario is difficult, operator should take care!!! '''

        ''' Transform to robot pose '''
        y_bl = {}
        mat = self.pose_to_mat(pose_bl.pose)
        #t_result = R1_inv * t2 + t1_inv

        for entity in entities:
            if onTopOff(entity, self.robot.ed.get_entity(id=res.choices['location'])):
                vec_bl = mat[0:3,0:3].getT()*numpy.matrix([entity.center_point.x, entity.center_point.y, entity.center_point.z]) - mat[0:3, 3]
                y_bl[entity.id] = vec_bl.item(1)

        ''' Sort on y coordinate '''
        sorted_y_bl = sorted(y_bl.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

        ''' Assert to world model '''
        updates = zip(sorted_y_bl, res.choices)
        for update in updates:
            self.robot.ed.update_entity(id=update[0], type=update[1])

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
        location = res.choices['location']
        if 'object3' in res.choices:
            object1  = res.choices['object1']
            object2  = res.choices['object2']
            object3  = res.choices['object3']
            self.robot.speech.speak("All right, so I can find {0}, {1} and {2} on the {3}".format(object1, object2, object3, location))
        elif 'object2' in res.choices:
            object1  = res.choices['object1']
            object2  = res.choices['object2']
            self.robot.speech.speak("Okay, so there is {0} and {1} on the {2}".format(object1, object2, location))
        else:
            object1  = res.choices['object1']
            self.robot.speech.speak("So I guess I see {0} on the {1}".format(object1, location))


class CiteItems(smach.State):
    """ Cite all items that have been found """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):

        entities = self.robot.ed.get_entities()
        if len(entities) == 0:
            rospy.logerr("Something went terribly wrong, I have not found any entities")
            self.robot.speech.speak("I must be blind")
            return 'failed'

        self.robot.speech.speak("I have found some items that I don't know")

        surface = self.robot.ed.get_entity(id=TABLE1)
        if surface:
            count = 0
            for entity in entities:
                if (onTopOff(entity, surface)):
                    count += 1
            self.robot.speech.speak("I have found {0} items on the {1}".format(count, TABLE1))

        count = 0
        for shelf in OBJECT_SHELVES:
            surface = self.robot.ed.get_entity(id=shelf)
            if surface:
                for entity in entities:
                    if (onTopOff(entity, surface)):
                        count += 1
        self.robot.speech.speak("I have found {0} items in the bookcase".format(count))

        surface = self.robot.ed.get_entity(id=TABLE2)
        if surface:
            count = 0
            for entity in entities:
                if (onTopOff(entity, surface)):
                    count += 1
        self.robot.speech.speak("I have found {0} items on the {1}".format(count, TABLE2))

        return 'succeeded'

class InspectShelves(smach.State):
    """ Inspect all object shelves """

    def __init__(self, robot, object_shelves):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.object_shelves = object_shelves

    def execute(self, userdata):

        ''' Loop over shelves '''
        for shelf in self.object_shelves:

            rospy.loginfo("Shelf: {0}".format(shelf))

            ''' Get entities '''
            entity = self.robot.ed.get_entity(id=shelf, parse=False)

            if entity:

                ''' Extract center point '''
                cp = entity.center_point

                ''' Look at target '''
                self.robot.head.look_at_point(msgs.PointStamped(cp.x,cp.y,cp.z,"/map"))

                ''' Move spindle
                    Implemented only for AMIGO (hence the hardcoding)
                    Assume table height of 0.8 corresponds with spindle reset = 0.35 '''
                # def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
                height = min(0.4, max(0.1, cp.z-0.55))
                self.robot.torso._send_goal([height], timeout=5.0)

                ''' Sleep for 1 second '''
                rospy.sleep(1.0)

        return 'succeeded'

############################## explore state machine #####################
class ExploreScenario(smach.StateMachine):
    
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('GOTO_TABLE1',
                                states.NavigateToSymbolic(robot, 
                                                          {EdEntityDesignator(robot, id=TABLE1):"in_front_of"}, 
                                                          EdEntityDesignator(robot, id=TABLE1)),
                                transitions={   'arrived'           :   'STORE_TABLE1',
                                                'unreachable'       :   'STORE_TABLE1',
                                                'goal_not_defined'  :   'STORE_TABLE1'})

            smach.StateMachine.add("STORE_TABLE1",
                                    StorePose(robot, TABLE1),
                                    transitions={  'stored'         :   'LOOKAT_TABLE1'})

            smach.StateMachine.add("LOOKAT_TABLE1",
                                     states.LookAtEntity(robot, EdEntityDesignator(robot, id=TABLE1), keep_following=True),
                                     transitions={  'succeeded'         :'ASK_ITEMS1'})

            smach.StateMachine.add('ASK_ITEMS1',
                                AskItems(robot),
                                transitions={   'succeeded'         :   'RESET_HEAD_TABLE1',
                                                'failed'            :   'RESET_HEAD_TABLE1'} )

            smach.StateMachine.add( "RESET_HEAD_TABLE1",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'GOTO_CABINET'})

            smach.StateMachine.add('GOTO_CABINET',
                                states.NavigateToSymbolic(robot, 
                                                          {EdEntityDesignator(robot, id=CABINET):"in_front_of"}, 
                                                          EdEntityDesignator(robot, id=CABINET)),
                                transitions={   'arrived'           :   'STORE_CABINET',
                                                'unreachable'       :   'STORE_CABINET',
                                                'goal_not_defined'  :   'STORE_CABINET'})

            smach.StateMachine.add("STORE_CABINET",
                                StorePose(robot, TABLE1),
                                transitions={  'stored'         :   'LOOKAT_CABINET'})

            smach.StateMachine.add("LOOKAT_CABINET",
                                InspectShelves(robot, OBJECT_SHELVES),
                                transitions={'succeeded'                :'ASK_ITEMS2',
                                             'failed'                   :'ASK_ITEMS2'})

            smach.StateMachine.add('ASK_ITEMS2',
                                AskItems(robot),
                                transitions={   'succeeded'         :   'RESET_HEAD_CABINET',
                                                'failed'            :   'RESET_HEAD_CABINET'} )

            smach.StateMachine.add( "RESET_HEAD_CABINET",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'GOTO_TABLE2'})

            smach.StateMachine.add('GOTO_TABLE2',
                                states.NavigateToSymbolic(robot, 
                                                          {EdEntityDesignator(robot, id=TABLE2):"in_front_of"}, 
                                                          EdEntityDesignator(robot, id=TABLE2)),
                                transitions={   'arrived'           :   'STORE_TABLE2',
                                                'unreachable'       :   'STORE_TABLE2',
                                                'goal_not_defined'  :   'STORE_TABLE2'})

            smach.StateMachine.add("STORE_TABLE2",
                                    StorePose(robot, TABLE2),
                                    transitions={  'stored'         :   'LOOKAT_TABLE2'})

            smach.StateMachine.add("LOOKAT_TABLE2",
                                     states.LookAtEntity(robot, EdEntityDesignator(robot, id=TABLE2), keep_following=True),
                                     transitions={  'succeeded'         :'ASK_ITEMS3'})

            smach.StateMachine.add('ASK_ITEMS3',
                                AskItems(robot),
                                transitions={   'succeeded'         :   'RESET_HEAD_TABLE2',
                                                'failed'            :   'RESET_HEAD_TABLE2'} )

            smach.StateMachine.add( "RESET_HEAD_TABLE2",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'done'})

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