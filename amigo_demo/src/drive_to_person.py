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
                                    transitions={   "query_true":"GOTO_PERSON",
                                                    "timed_out":"Failed",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add( "GOTO_PERSON",
                                    states.NavigateGeneric(robot, lookat_query=self.human_query),
                                    transitions={   "arrived":"TOGGLE_OFF_OK",
                                                    "unreachable":'TOGGLE_OFF_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'TOGGLE_OFF_NO_FOUND'})

            smach.StateMachine.add( "TOGGLE_OFF_OK",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"SAY_SOMETHING"})

            smach.StateMachine.add( "SAY_SOMETHING",
                                  states.Say(robot, ["I found someone!"]),
                                  transitions={"spoken":"LOOKAT_PERSON"})

            smach.StateMachine.add( "LOOKAT_PERSON", 
                        states.LookForObjectsAtROI(robot, lookat_query=self.human_query, object_query=self.human_query),
                        transitions={   'looking':"Done",
                                        'object_found':'Done',
                                        'no_object_found':'Done',
                                        'abort':'Aborted'})


            smach.StateMachine.add( "TOGGLE_OFF_UNREACHABLE",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"SAY_GOAL_UNREACHABLE"})
            smach.StateMachine.add( "SAY_GOAL_UNREACHABLE",
                                  states.Say(robot, ["I can't reach the humans I was going to"], mood="sad"),
                                  transitions={"spoken":"Failed"})


            smach.StateMachine.add( "TOGGLE_OFF_NO_FOUND",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"Failed"})

'looking',
'object_found',
'no_object_found',
'abort'