#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Compound

import std_msgs.msg


class WaitForTrigger(smach.State):

    trigger_received = False

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['trigger_received', 'Aborted'])

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '1.0'))
        rospy.loginfo('listening for trigger with a rate of %d Hz', self.rate)

        topic = rospy.get_param('~topic', 'trigger')
        rospy.loginfo('topic = %s', topic)
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitForTrigger')

        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

        if self.trigger_received:
            return 'trigger_received'
        else:
            return 'Aborted'

    def callback(self, data):
        # Simply print out values in our custom message.
        rospy.loginfo('trigger_received: %s', data.data)
        self.trigger_received = True

class AvoidThat(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))

        robot.reasoner.assertz(Compound("challenge", "challenge_demo_2014"))

        query_waypoint =  Compound("waypoint", "goal", Compound("pose_2d", "X", "Y", "Phi"))
        
        with self:

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT',
                                    states.NavigateGeneric(robot, goal_query=query_waypoint),
                                    transitions={   "arrived":"SAY_GOAL_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add("START_CHALLENGE", 
                                    states.Say(robot, [ "I received a trigger, I'm starting."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot, [ "Sorry, the goal is unreachable. Aborting now."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot, [ "Sorry, I don't know where to go. Please specify my waypoint. Aborting now."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_REACHED", 
                                    states.Say(robot, [ "Yeah, I reached my goal. Exiting challenge."]),
                                    transitions={   'spoken':'Aborted'})

def AvoidThatTester():

    sm = smach.StateMachine(outcomes=['Done'])
    #sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAITFORTRIGGER', WaitForTrigger(), 
                               transitions={'trigger_received': 'Done',
                                            'Aborted':          'Done'})

    # Execute SMACH plan
    sm.execute()


if __name__ == "__main__":
    rospy.init_node('challenge_demo_2014_exec')
    AvoidThatTester()
