#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_skills.util import msg_constructors as msgs

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')
INITIAL_POSE = challenge_knowledge.initial_pose_sergio

MESH_IDS = []

class LookBaseLinkPoint(smach.State):
    def __init__(self, robot, x, y, z, timeout = 2.5, waittime = 0.0):
        """ 
        Sends a goal to the head in base link frame of the robot_name
        x, y, z: coordinates
        timeout: timeout of the call to the head ref action (hence is a maximum)
        waittime: additional waiting time 
        """
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.x = x
        self.y = y
        self.z = z
        self.timeout = timeout
        self.waittime = waittime

    def execute(self, userdata):
        self.robot.head.look_at_point(msgs.PointStamped(x=self.x, y=self.y, z=self.z, frame_id=self.robot.robot_name+"/base_link"), self.timeout)
        rospy.sleep(rospy.Duration(self.waittime))
        return 'succeeded'

class TakeSnapShot(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):
        MESH_IDS.append('mesh'.format(len(MESH_IDS))) # ToDo: make nice
        self.robot.ed.mesh_entity_in_view(id=MESH_IDS[-1])
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
                    self.robot.ed.update_entity(id=MESH_IDS[-1], type = name_object)
                else:
                    rospy.logerr("Challenge final: Cannot update mesh type: id unknown")
                return "succeeded"
            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "KEYERROR FINAL, should not happen!"
            return "failed"

class ExploreWaypoint(smach.StateMachine):
    def __init__(self, robot, waypoint):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        waypoint_designator = EdEntityDesignator(robot, id=waypoint)

        with self:
            smach.StateMachine.add("GOTO_WAYPOINT",
                                    states.NavigateToWaypoint(robot, waypoint_designator),
                                    transitions={   'arrived'                  :'LOOK_AT_MESH',
                                                    'unreachable'              :'LOOK_AT_MESH',
                                                    'goal_not_defined'         :'LOOK_AT_MESH'})

            ''' Look at thing '''
            smach.StateMachine.add("LOOK_AT_MESH",
                                    LookBaseLinkPoint(robot, x=2.5, y=0, z=1, timeout=2.5, waittime=1.5),
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
                                transitions={   "succeeded"        :   "EXPLORE3",
                                                "failed"           :   "EXPLORE3"})

        smach.StateMachine.add("EXPLORE3",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_3),
                                transitions={   "succeeded"        :   "EXPLORE4",
                                                "failed"           :   "EXPLORE4"})

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
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_sergio')

    startup(setup_statemachine, robot_name='sergio')