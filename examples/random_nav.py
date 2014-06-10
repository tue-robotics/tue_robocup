#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy
import random

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound
from robot_skills.arms import State as ArmState
from robot_smach_states.util.startup import startup

class RandomNav(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])
        
        query_exploration_target_in_room = Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi"))
        
        goal_query = Conjunction( Compound("current_target", "Target"),
                                  Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi")))
        
        with self:
            
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
                    target_index = str(string_target).index("(")

                    speak_target = string_target[0:target_index]

                    robot.speech.speak("Lets go look at the {0}".format(speak_target).replace("_", " "), block=False)

                    return 'target_determined'
            
            smach.StateMachine.add('DETERMINE_TARGET', smach.CBState(determine_target),
                                    transitions={   'target_determined':'DRIVE',
                                                    'done':'Done'})

            ################################################################
            smach.StateMachine.add( 'DRIVE',
                                    states.NavigateGeneric(robot, goal_query=goal_query),
                                    transitions={   "arrived":"SAY_SUCCEEDED",
                                                    "unreachable":'SAY_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'DETERMINE_TARGET'})
                                                    
            smach.StateMachine.add("SAY_SUCCEEDED", 
                                    states.Say(robot, [ "I am here", 
                                                        "Goal succeeded", 
                                                        "Another target reached"]),
                                    transitions={   'spoken':'DETERMINE_TARGET'})
                                                    
            smach.StateMachine.add("SAY_UNREACHABLE", 
                                    states.Say(robot, [ "I can't find a way to my goal, better try something else", 
                                                        "This goal is unreachable, I better find somewhere else to go", 
                                                        "I am having a hard time getting there so I will find a new target"]),
                                    transitions={   'spoken':'DETERMINE_TARGET'})

if __name__ == "__main__":
    rospy.init_node('random_nav_exec')
    
    startup(RandomNav)
