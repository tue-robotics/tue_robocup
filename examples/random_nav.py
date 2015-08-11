#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import random

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound
from robot_skills.arms import State as ArmState
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
        
        ''' Query given to nav state '''
        goal_query = Conjunction( Compound("current_target", "Target"),
                                  Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi")))
                                  
        self.requested_location = None
        rospy.Subscriber("/location_request", std_msgs.msg.String, self.requestedLocaltioncallback)
        
        with self:
            
            smach.StateMachine.add( "WAIT_A_SEC", 
                                    states.WaitTime(robot, waittime=1.0),
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
                    rospy.loginfo("I should perform a different query now")

                    # The waypoint we're looking for
                    q1 = Compound("waypoint", Compound(self.requested_location, "Ext"), Compound("pose_2d", "X", "Y", "Phi"))
                    # All waypoints with name poored in "Target"
                    q2 = Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi"))
                    # Combining these two
                    query_targets = Conjunction(q1,q2)
                    
                    answers = robot.reasoner.query(query_targets)
                    self.requested_location = None
                else:
                    rospy.loginfo("I'll do the random query here")
                    query_targets = Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi"))
                    answers = robot.reasoner.query(query_targets)
                
                rospy.loginfo("Answers for {0}: {1}".format(query_targets, answers))
                if not answers:
                    # no more exporation targets found
                    return 'no_targets_available'
                else:         
                    # Pick random target
                    goal = answers[random.randint(0,len(answers)-1)]["Target"]

                    rospy.loginfo("Available targets: {0}".format(answers))
                    rospy.loginfo("Selected target: {0}".format(goal))
                    #target = answers[0]["Target"]

                    # remove current target
                    robot.reasoner.query(Compound("retractall", Compound("current_target", "X")))

                    # add new target
                    robot.reasoner.assertz(Compound("current_target", goal))

                    string_target = str(goal)
                    rospy.loginfo("Goal string = {0}".format(string_target))
                    try:
                        target_index = str(string_target).index("(")
                        speak_target = string_target[0:target_index]
                    except:
                        speak_target = string_target
                    
                    sentences = ["Lets go look at the {0}".format(speak_target).replace("_", " "),
                                    "Lets have a look at the {0}".format(speak_target).replace("_", " "),
                                    "Lets go to the {0}".format(speak_target).replace("_", " "),
                                    "Lets move to the {0}".format(speak_target).replace("_", " "),
                                    "I will go to the {0}".format(speak_target).replace("_", " "),
                                    "I will now move to the {0}".format(speak_target).replace("_", " "),
                                    "I will now drive to the {0}".format(speak_target).replace("_", " "),
                                    "I will look the {0}".format(speak_target).replace("_", " "),
                                    "The {0} will be my next location".format(speak_target).replace("_", " "),
                                    "The {0} it is".format(speak_target).replace("_", " "),
                                    "New goal, the {0}".format(speak_target).replace("_", " "),
                                    "Going to look at the {0}".format(speak_target).replace("_", " "),
                                    "Moving to the {0}".format(speak_target).replace("_", " "),
                                    "Driving to the {0}".format(speak_target).replace("_", " "),
                                    "On to the {0}".format(speak_target).replace("_", " "),
                                    "On the move to the {0}".format(speak_target).replace("_", " "),
                                    "Going to the {0}".format(speak_target).replace("_", " "),
                                    ]
                    robot.speech.speak(sentences[random.randint(0,len(sentences)-1)], block=False)

                    return 'target_determined'
            
            smach.StateMachine.add('DETERMINE_TARGET', smach.CBState(determine_target),
                                    transitions={   'target_determined':'DRIVE',
                                                    'no_targets_available':'SELECT_ACTION'})

            smach.StateMachine.add( 'DRIVE',
                                    states.NavigateGeneric(robot, goal_query=goal_query, look_at_path_distance=1.75, goal_area_radius=0.30),
                                    transitions={   "arrived":"SAY_SUCCEEDED",
                                                    "unreachable":'SAY_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_SUCCEEDED", 
                                    states.Say(robot, [ "I am here", 
                                                        "Goal succeeded", 
                                                        "Another goal succeeded",
                                                        "Goal reached",
                                                        "Another goal reached",
                                                        "Target reached",
                                                        "Another target reached",
                                                        "Destination reached",
                                                        "Another destination reached",
                                                        "I have arrived",
                                                        "I have arrived at my goal",
                                                        "I have arrived at my target",
                                                        "I have arrived at my destination",
                                                        "I am at my goal",
                                                        "I am at my target",
                                                        "I am at my destination",
                                                        "Here I am",]),
                                    transitions={   'spoken':'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_UNREACHABLE", 
                                    states.Say(robot, [ "I can't find a way to my goal, better try something else", 
                                                        "This goal is unreachable, I better find somewhere else to go", 
                                                        "I am having a hard time getting there so I will look for a new target"]),
                                    transitions={   'spoken':'SELECT_ACTION'})
                                    
            smach.StateMachine.add("SAY_DONE", 
                                    states.Say(robot, [ "That's all folks",
                                                        "I'll stay here for a while",
                                                        "Goodbye"]),
                                    transitions={   'spoken':'Done'})
                                    
    def requestedLocaltioncallback(self, msg):
        self.requested_location = msg.data
        self.robot.speech.speak("I got a request to go to location {0}".format(self.requested_location))
        rospy.loginfo("Requested location is {0}".format(self.requested_location))

if __name__ == "__main__":
    rospy.init_node('random_nav_exec')
    
    startup(RandomNav)
