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
        
        query_exploration_target_in_room = Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi"))
        
        goal_query = Conjunction( Compound("current_target", "Target"),
                                  Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi")))
        
        with self:
            
            smach.StateMachine.add( "WAIT_A_SEC", 
                                    states.Wait_time(robot, waittime=1.0),
                                    transitions={'waited'   :"SELECT_ACTION",
                                                 'preempted':"Aborted"})
                                                 
            smach.StateMachine.add( "SELECT_ACTION",
                                    SelectAction(),
                                    transitions={   'continue'  : "DETERMINE_TARGET",
                                                    'pause'     : "SELECT_ACTION",
                                                    'stop'      : "Done"})
            
            @smach.cb_interface(outcomes=['target_determined', 'done'], 
                                input_keys=[], 
                                output_keys=[])
            def determine_target(userdata):            
                # Ask the reaoner for a target:
                answers = robot.reasoner.query(query_exploration_target_in_room)
                rospy.loginfo("Answers for {0}: {1}".format(query_exploration_target_in_room, answers))
                #import ipdb; ipdb.set_trace()
                if not answers:
                    # no more exporation targets found
                    return 'done'
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
                    
                    robot.speech.speak("Lets go look at the {0}".format(speak_target).replace("_", " "), block=False)

                    return 'target_determined'
            
            smach.StateMachine.add('DETERMINE_TARGET', smach.CBState(determine_target),
                                    transitions={   'target_determined':'DRIVE',
                                                    'done':'Done'})

            smach.StateMachine.add( 'DRIVE',
                                    states.NavigateGeneric(robot, goal_query=goal_query),
                                    transitions={   "arrived":"SAY_SUCCEEDED",
                                                    "unreachable":'SAY_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_SUCCEEDED", 
                                    states.Say(robot, [ "I am here", 
                                                        "Goal succeeded", 
                                                        "Another target reached"]),
                                    transitions={   'spoken':'SELECT_ACTION'})
                                                    
            smach.StateMachine.add("SAY_UNREACHABLE", 
                                    states.Say(robot, [ "I can't find a way to my goal, better try something else", 
                                                        "This goal is unreachable, I better find somewhere else to go", 
                                                        "I am having a hard time getting there so I will look for a new target"]),
                                    transitions={   'spoken':'SELECT_ACTION'})

if __name__ == "__main__":
    rospy.init_node('random_nav_exec')
    
    startup(RandomNav)
