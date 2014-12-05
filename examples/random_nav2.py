#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import random

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint

from robot_smach_states.util.startup import startup
import std_msgs.msg

class SelectAction(smach.State):
    def __init__(self, outcomes=['continue', 'pause', 'stop']):
        self.outcomes= outcomes
        smach.State.__init__(self, outcomes=self.outcomes)
        self.outcome = 'continue'
        
        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic     = rospy.get_param('~topic', '/nav_test_control')
        
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)
        
        rospy.loginfo("Use 'navc' to continue, 'navp' to pause and 'navs' to stop this node")
        
    def execute(self, userdata):
        if self.outcome == 'pause':
            rospy.sleep(rospy.Duration(1/self.rate))
        return self.outcome
        
    def callback(self, msg):
        if msg.data in self.outcomes:
            self.outcome = msg.data
        else:
            rospy.logwarn("{0} is not a possible outcome for SelectAction, possibilities are{1}".format(msg.data, self.outcomes))

class RandomNav(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])
        
        self.robot = robot
        
        self.position_constraint = PositionConstraint()
        self.orientation_constraint = OrientationConstraint()
        self.position_constraint.constraint = "x^2+y^2 < 1"
                                  
        self.requested_location = None
        rospy.Subscriber("/location_request", std_msgs.msg.String, self.requestedLocationcallback)
        
        with self:
            
            smach.StateMachine.add( "WAIT_A_SEC", 
                                    states.Wait_time(robot, waittime=1.0),
                                    transitions={'waited'   :"SELECT_ACTION",
                                                 'preempted':"Aborted"})
                                                 
            smach.StateMachine.add( "SELECT_ACTION",
                                    SelectAction(),
                                    transitions={   'continue'  : "DETERMINE_TARGET",
                                                    'pause'     : "SELECT_ACTION",
                                                    'stop'      : "SAY_DONE"})
            
            @smach.cb_interface(outcomes=['target_determined', 'no_targets_available'], 
                                input_keys=[], 
                                output_keys=[])
            def determine_target(userdata):
                if self.requested_location != None:
                    # Query nav interface ramon
                    target = self.requested_location
                    requested_location = None
                else:
                    # Query ED
                    targets = [e.id for e in self.robot.ed.getEntities() if e.type != "" and len(e.id) != 32]
                    target = random.choice(targets)

                # Set the constraints
                self.position_constraint.frame = target
                self.orientation_constraint.frame = target
                
                sentences = ["Lets go look at the %s", "Lets have a look at the %s", "Lets go to the %s", "Lets move to the %s" ,"I will go to the %s", "I will now move to the %s", "I will now drive to the %s", "I will look the %s", "The %s will be my next location", "The %s it is", "New goal, the %s", "Going to look at the %s", "Moving to the %s", "Driving to the %s", "On to the %s", "On the move to the %s", "Going to the %s"]
                robot.speech.speak(random.choice(sentences)%target, block=False)

                return 'target_determined'
            
            smach.StateMachine.add('DETERMINE_TARGET', smach.CBState(determine_target),
                                    transitions={   'target_determined':'DRIVE',
                                                    'no_targets_available':'SELECT_ACTION'})

            smach.StateMachine.add( 'DRIVE',
                                    states.NavigateWithConstraints(robot, self.position_constraint, self.orientation_constraint),
                                    transitions={   "arrived":"SAY_SUCCEEDED",
                                                    "unreachable":'SAY_UNREACHABLE',
                                                    "goal_not_defined":'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_SUCCEEDED", 
                                    states.Say(robot, [ "I am here", "Goal succeeded", "Another goal succeeded", "Goal reached", "Another goal reached","Target reached", "Another target reached", "Destination reached","Another destination reached",  "I have arrived", "I have arrived at my goal", "I have arrived at my target", "I have arrived at my destination", "I am at my goal", "I am at my target", "I am at my destination", "Here I am",]),
                                    transitions={   'spoken':'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_UNREACHABLE", 
                                    states.Say(robot, [ "I can't find a way to my goal, better try something else", "This goal is unreachable, I better find somewhere else to go", "I am having a hard time getting there so I will look for a new target"]),
                                    transitions={   'spoken':'SELECT_ACTION'})
                                    
            smach.StateMachine.add("SAY_DONE", 
                                    states.Say(robot, [ "That's all folks", "I'll stay here for a while", "Goodbye"]),
                                    transitions={   'spoken':'Done'})
                                    
    def requestedLocationcallback(self, msg):
        self.requested_location = msg.data
        self.robot.speech.speak("I got a request to go to location %"%self.requested_location)
        rospy.loginfo("Requested location is %s"%self.requested_location)

if __name__ == "__main__":
    rospy.init_node('random_nav_exec')
    
    startup(RandomNav)
