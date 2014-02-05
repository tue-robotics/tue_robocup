#!/usr/bin/env python
import roslib
roslib.load_manifest('amigo_demo')
import rospy

from robot_skills.reasoner import Compound, Conjunction, Sequence
import smach
import robot_smach_states as states
from std_srvs.srv import Empty

class DriveToClosestPerson(smach.StateMachine):
    """Scan (with the torso laser) for persons, 
    go to the closest one and say something nice."""

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self.robot = robot
        self.human_query = Conjunction( Compound("instance_of", "ObjectID", "person"), 
                                        Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        with self:
            smach.StateMachine.add( "TOGGLE_PEOPLE_DETECTION",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"WAIT_FOR_DETECTION"})

            smach.StateMachine.add( "WAIT_FOR_DETECTION",
                                    states.Wait_query_true(robot, self.human_query, timeout=5),
                                    transitions={   "query_true":"TOGGLE_OFF_OK",
                                                    "timed_out":"Failed",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add( "TOGGLE_OFF_OK",
                                    states.TogglePeopleDetector(robot, on=False),
                                    transitions={   "toggled":"GOTO_PERSON"})

            smach.StateMachine.add( "GOTO_PERSON",
                                    states.NavigateGeneric(robot, lookat_query=self.human_query, xy_dist_to_goal_tuple=(1.0,0)),
                                    transitions={   "arrived":"LOOK_UP_FOR_SAY",
                                                    "unreachable":'SAY_FAILED',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_FAILED'})
            
            smach.StateMachine.add( "LOOK_UP_FOR_SAY",
                                  states.ResetHead(robot),
                                  transitions={"done":"SAY_SOMETHING"})

            smach.StateMachine.add( "SAY_SOMETHING",
                                  states.Say(robot, ["I found someone!"]),
                                  transitions={"spoken":"Done"})

            smach.StateMachine.add( "SAY_FAILED",
                                  states.Say(robot, ["I didn't find anyone"]),
                                  transitions={"spoken":"LOOK_UP_RESET"})

            smach.StateMachine.add( "LOOK_UP_RESET",
                                  states.ResetHead(robot),
                                  transitions={"done":"Failed"})

'looking',
'object_found',
'no_object_found',
'abort'