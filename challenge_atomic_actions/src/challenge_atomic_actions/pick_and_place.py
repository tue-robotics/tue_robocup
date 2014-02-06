#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction
from robot_smach_states.util.startup import startup

class PickAndPlace(smach.StateMachine):

    def __init__(self, robot, poi_lookat="desk_1", grasp_arm="left"):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        if grasp_arm == "left":
            arm = robot.leftArm
        elif grasp_arm == "right":
            arm = robot.rightArm
        else:
            rospy.logerr("{0} is not a good grasp_arm, defaulting to left".grasp_arm)
            arm = robot.leftArm

        #retract old facts
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
        
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects_tue.pl'))
        rospy.logwarn("Loading objects_tue, why are there two prolog files???")
        
        #robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/cleanup_test.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "pick_and_place"))

        query_lookat = Compound("point_of_interest", poi_lookat, Compound("point_3d", "X", "Y", "Z"))

        #ToDo: if disposed not relevant. Rather have the Object with the highest probability!
        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))

        query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        query_current_object_class = Conjunction(
                                Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                                Compound("instance_of",         "Obj_to_Dispose",   Compound("exact", "ObjectType")))

        query_dropoff_loc = Conjunction(
                                    Compound("current_object", "Obj_to_Dispose"), #Of the current object
                                    Compound("instance_of",    "Obj_to_Dispose",   "ObjectType"), #Gets its type
                                    Compound("storage_class",  "ObjectType",       "Disposal_type"), #Find AT what sort of thing it should be disposed, e.g. a trash_bin
                                    Compound("dropoff_point",  "Disposal_type", Compound("point_3d", "X", "Y", "Z")))

        query_dropoff_loc_backup = Compound("dropoff_point", "trash_bin", Compound("point_3d", "X", "Y", "Z"))

        with self:
            smach.StateMachine.add('INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "LOOK",
                                                 "abort":       "Aborted"})

            smach.StateMachine.add('LOOK',
                                    LookForObjectsAtROI(robot, query_lookat, query_object),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'SAY_FOUND_NOTHING',
                                                    'abort':'Aborted'})

            smach.StateMachine.add('SAY_FOUND_NOTHING',
                                    Say(robot, ["I didn't find anything here", "No objects here", "There are no objects here", "I do not see anything here"]),
                                    transitions={ 'spoken':'RESET' })

            def generate_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(query_dropoff_loc)
                    _type = answers[0]["ObjectType"]
                    dropoff = answers[0]["Disposal_type"]
                    return "I have found a {0}. I'll' bring it to the {1}".format(_type, dropoff).replace("_", " ")
                except Exception, e:
                    rospy.logerr(e)
                    try:
                        type_only = robot.reasoner.query(query_current_object_class)[0]["ObjectType"]
                        return "I found something called {0}.".format(type_only).replace("_", " ")
                    except Exception, e:
                        rospy.logerr(e)
                        #If we end up here, override the query_dropoff_loc to always go to the trashbin
                        query_dropoff_loc = query_dropoff_loc_backup 
                        pass
                    return "I have found something, but I'm not sure what it is. I'll throw it in the trashbin"
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    Say_generated(robot, sentence_creator=generate_object_sentence),
                                    transitions={ 'spoken':'GRAB' })

            smach.StateMachine.add('GRAB',
                                    GrabMachine(arm, robot, query_grabpoint),
                                    transitions={   'succeeded':'DROPOFF_OBJECT',
                                                    'failed':'HUMAN_HANDOVER' })

            smach.StateMachine.add('HUMAN_HANDOVER',
                                    Human_handover(arm,robot),
                                    transitions={   'succeeded':'DROPOFF_OBJECT',
                                                    'failed':'RESET'})

            smach.StateMachine.add("DROPOFF_OBJECT",
                                    DropObject(arm, robot, query_dropoff_loc),
                                    transitions={   'succeeded':'Done',
                                                    'failed':'Failed',
                                                    'target_lost':'SAY_HUMAN_HANDOVER'})
            ''' In case of failed: the DropObject state has already done a human handover at the dropoff location
                In case of target_lost, it cannot go to the dropoff location but has to hand over the object anyway in order to proceed to the next task'''

            smach.StateMachine.add( 'SAY_HUMAN_HANDOVER', 
                                    Say(robot, [ "I am terribly sorry, but I cannot place the object. Can you please take it from me", 
                                                        "My apologies, but i cannot place the object. Would you be so kind to take it from me"]),
                                     transitions={   'spoken':'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( 'HANDOVER_TO_HUMAN', 
                                    HandoverToHuman(arm, self.robot),
                                    transitions={'succeeded'    : 'RESET',
                                                 'failed'       : 'RESET'})

            smach.StateMachine.add('RESET',
                                    Initialize(robot),
                                    transitions={"initialized": "Done",
                                                 "abort":       "Aborted"})

if __name__ == "__main__":
    rospy.init_node('pick_and_place_exec')
    startup(PickAndPlace)
