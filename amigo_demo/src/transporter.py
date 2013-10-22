#!/usr/bin/env python
import roslib
roslib.load_manifest('amigo_demo')
import rospy

from robot_skills.reasoner import Compound, Conjunction
import smach
import robot_smach_states as states
from std_srvs.srv import Empty


class ScanRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot
        self.demo_laser_service = rospy.ServiceProxy('/toggle_demo_laser', Empty)  

    def execute(self, userdata):
        self.demo_laser_service()
        return "done"


class Transporter(smach.StateMachine):

    """Transport an object from one point to another.
     This uses the demo_laser package to fit a model and put two waypoints and point_of_interest's relative to that.
     Amigo should continuously take an object from the first waypoint to the second"""

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        arm = robot.leftArm

        robot.reasoner.query(
            Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(
            Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))

        wp1_query = Compound(
            "waypoint", "waypoint1", Compound("pose_2d", "X", "Y", "Phi"))
        wp2_query = Compound(
            "waypoint", "waypoint2", Compound("pose_2d", "X", "Y", "Phi"))

        poi1_query = Compound(
            "point_of_interest", "waypoint1", Compound("point_3d", "X", "Y", "Z"))
        poi2_query = Compound(
            "point_of_interest", "waypoint2", Compound("point_3d", "X", "Y", "Z"))

        currobj_query = Conjunction(
                                Compound("current_object", "ObjectID"), #Of the current object
                                Compound("instance_of",    "ObjectID",   
                                    Compound("exact", "ObjectType"))) #Gets its type

        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")) #We don't care what objects, just get something close to the POI

        query_grabpoint = Conjunction(  
                            Compound("current_object", "ObjectID"),
                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        with self:
            smach.StateMachine.add("SCAN",
                ScanRoom(robot),
                transitions={   "done":"DRIVE_TO_WP1"})

            smach.StateMachine.add( "DRIVE_TO_WP1",
                states.NavigateGeneric(robot, goal_query=wp1_query),
                transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
                                "unreachable":'SAY_GOAL_UNREACHABLE',
                                "preempted":'Aborted',
                                "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add('SAY_GOAL_UNREACHABLE',
                states.Say(robot, ["I can't get where I wanted to go."]),
                transitions={   'spoken':'Aborted' })

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                states.Say(robot, ["Lets see what I can find here.", "What do we have here?", "What can I find here?", "Lets see if there are any objects here"]),
                transitions={   'spoken':'LOOK_FOR_OBJECTS'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                states.Say(robot, ["I don't know where to go"]),
                transitions={   'spoken':'Aborted'})

            smach.StateMachine.add('LOOK_FOR_OBJECTS',
                states.LookForObjectsAtROI(robot, poi1_query, query_object, maxdist=1.3),
                transitions={   'looking':'LOOK_FOR_OBJECTS',
                                'object_found':'SAY_FOUND_SOMETHING',
                                'no_object_found':'SAY_FOUND_NOTHING',
                                'abort':'Aborted'})

            smach.StateMachine.add('SAY_FOUND_NOTHING',
                states.Say(robot, ["I didn't find anything here", "No objects here", "There are no objects here"]),
                transitions={ 'spoken':'Aborted' })

            def generate_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(currobj_query)
                    _type = answers[0]["ObjectType"]
                    return "I have found a {0}.".format(_type).replace("_", " ")
                except Exception, e:
                    rospy.logerr(e)
                return "I have found something, but I'm not sure what it is."
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                states.Say_generated(robot, sentence_creator=generate_object_sentence),
                transitions={ 'spoken':'GRAB' })

            smach.StateMachine.add('GRAB',
                states.GrabMachine(arm, robot, query_grabpoint),
                transitions={   'succeeded':'DROPOFF_OBJECT',
                                'failed':'HUMAN_HANDOVER' })

            smach.StateMachine.add('HUMAN_HANDOVER',
                states.Human_handover(arm,robot),
                transitions={   'succeeded':'DROPOFF_OBJECT',
                                'failed':'Failed'})

            smach.StateMachine.add("DROPOFF_OBJECT",
                states.DropObject(arm, robot, poi2_query),
                transitions={   'succeeded':'Done',
                                'failed':'Failed',
                                'target_lost':'Failed'})