#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo_2014')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Compound
from robot_smach_states.util.startup import startup
import std_msgs.msg


class WaitForTrigger(smach.State):

    def __init__(self, robot, triggers):
        smach.State.__init__(self, 
                             outcomes=triggers+['preempted'])
        self.robot = robot
        self.triggers = triggers

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic     = rospy.get_param('~topic', 'trigger')
        
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata):
        self.trigger_received = False

        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

        if self.trigger_received:
            return self.trigger_received
        else:
            return 'preempted'

    def callback(self, data):
        # Simply print out values in our custom message.
        rospy.loginfo('trigger_received: %s', data.data)
        if data.data in self.triggers:
            self.trigger_received = data.data

class ChallengeDemo2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        #retract old facts
        #TODO: maybe retract more facts like other challenges?
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))

        #Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        #Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "challenge_demo_2014")))

        query_start = Compound("waypoint", "start",         Compound("pose_2d", "X", "Y", "Phi"))
        query_door  = Compound("waypoint", "behind_door",   Compound("pose_2d", "X", "Y", "Phi"))
        
        with self:
            smach.StateMachine.add('INITIALIZE_FIRST',
                                    states.Initialize(robot),
                                    transitions={   'initialized':'NAVIGATE_TO_START',
                                                    'abort':'Aborted'})

            smach.StateMachine.add('NAVIGATE_TO_START',
                                    states.NavigateGeneric(robot, goal_query=query_start),
                                    transitions={   "arrived":"SAY_START_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})
            
            smach.StateMachine.add("SAY_START_REACHED", 
                                    states.Say(robot,"I'm going to wait for further instructions", block=False),
                                    transitions={   'spoken':'WAIT_FOR_TRIGGER'})

            smach.StateMachine.add("WAIT_FOR_TRIGGER", 
                                    WaitForTrigger(robot, ['doorbell']),
                                    transitions={   'doorbell':'SAY_TRIGGER_RECEIVED',
                                                    'preempted': 'Aborted'})
            
            smach.StateMachine.add("SAY_TRIGGER_RECEIVED", 
                                    states.Say(robot,"That was the doorbell, I must hurry", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_DOOR'})

            smach.StateMachine.add('NAVIGATE_TO_DOOR',
                                    states.NavigateGeneric(robot, goal_query=query_door),
                                    transitions={   "arrived":"SAY_DOOR_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add("SAY_DOOR_REACHED", 
                                    states.Say(robot,"Can I receive your package?", block=False),
                                    transitions={   'spoken':'RECEIVE_POSE_LEFT'})
            
            smach.StateMachine.add("RECEIVE_POSE_LEFT", 
                                    states.Carrying_pose(robot.leftArm, robot),
                                    transitions={   'succeeded':'RECEIVE_POSE_RIGHT',
                                                    'failed':'RECEIVE_POSE_RIGHT'})

            smach.StateMachine.add("RECEIVE_POSE_RIGHT", 
                                    states.Carrying_pose(robot.rightArm, robot),
                                    transitions={   'succeeded':'WAIT_FOR_LOAD',
                                                    'failed':'WAIT_FOR_LOAD'})

            smach.StateMachine.add( 'WAIT_FOR_LOAD',
                        states.Wait_time(robot, waittime=5),
                                    transitions={   'waited':'SAY_PACKAGE_RECEIVED',
                                                    'preempted':'SAY_PACKAGE_RECEIVED'})
            
            smach.StateMachine.add("SAY_PACKAGE_RECEIVED", 
                                    states.Say(robot,"Thank you, I will notify my owner", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_START_2'})

            smach.StateMachine.add('NAVIGATE_TO_START_2',
                                    states.NavigateGeneric(robot, goal_query=query_start),
                                    transitions={   "arrived":"DRIVE_TO_CLOSEST_PERSON",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'DRIVE_TO_CLOSEST_PERSON',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add( 'DRIVE_TO_CLOSEST_PERSON',
                                    states.DriveToClosestPerson(robot),
                                    transitions={   "Done":"SAY_PERSON_FOUND",
                                                    "Aborted":"SAY_PERSON_FOUND",
                                                    "Failed":"SAY_PERSON_FOUND"})

            smach.StateMachine.add("SAY_PERSON_FOUND", 
                                    states.Say(robot,"Hey, there is my owner", block=False),
                                    transitions={   'spoken':'Aborted'})

            # navigation states
            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot,"Sorry, the goal is unreachable. Aborting now."),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot,"Sorry, I don't know where to go. Please specify my waypoint. Aborting now."),
                                    transitions={   'spoken':'Aborted'})

def WaitForTriggerTester():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:
        smach.StateMachine.add("WAIT_FOR_TRIGGER", 
                                WaitForTrigger(None, ['doorbell']),
                                transitions={   'doorbell': 'Done',
                                                'Aborted':  'Done'})
    sm.execute()


if __name__ == "__main__":
    rospy.init_node('challenge_demo_2014_exec')
    #WaitForTriggerTester()
    startup(ChallengeDemo2014)
