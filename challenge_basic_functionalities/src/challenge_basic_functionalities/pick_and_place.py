#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_basic_functionalities')
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
from robot_smach_states.util.startup import startup
import robot_skills.util.msg_constructors as msgs
from robot_smach_states.designator.designator import Designator, VariableDesignator

from pein_srvs.srv import SetObjects

class PickAndPlace(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        ## if only certain types of objects are possible to be recognized, use understanding code
        self.set_objects = rospy.ServiceProxy('/pein/set_object_models',SetObjects)
        response = self.set_objects(['beer','orange_juice', 'milk', 'noodles', 'coffee', 'unknown'])

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
        robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
        
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
        
        #robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/cleanup_test.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "basic_functionalities"))

        #query_lookat = Compound("point_of_interest", poi_lookat, Compound("point_3d", "X", "Y", "Z"))
        #query_lookat = Compound("=", "Poi", Compound("pick_poi", "Option"))
        
        query_lookat = Conjunction(  Compound("=", "Poi", Compound("pick_poi", "Option")),
                                     Compound("point_of_interest", "Poi", Compound("point_3d", "X", "Y", "Phi")))

        rospy.logwarn("query_lookat = {0}".format(query_lookat))

        # #ToDo: if disposed not relevant. Rather have the Object with the highest probability!
        # query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))

        #In the end, we just want to grab some entity with some ID. How we get this ID should not matter, so we can use a designator.
        grab_entity_designator = Designator("dummy_entity_id") 


        # query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
        #                                 Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        query_current_object_class = Conjunction(
                                Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                                Compound("instance_of",         "Obj_to_Dispose",   "ObjectType"))

        query_dropoff_loc = Conjunction(
                                    Compound("current_object", "Obj_to_Dispose"), #Of the current object
                                    Compound("instance_of",    "Obj_to_Dispose",   "ObjectType"), #Gets its type
                                    Compound("storage_class",  "ObjectType",       "Disposal_type"), #Find AT what sort of thing it should be disposed, e.g. a trash_bin
                                    Compound("dropoff_point",  "Disposal_type", Compound("point_3d", "X", "Y", "Z")))

        query_dropoff_loc_backup = Compound("dropoff_point", Compound("waste_bin","Loc"), Compound("point_3d", "X", "Y", "Z"))

        with self:
            smach.StateMachine.add('INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "PICKUP_OBJECT",
                                                 "abort":       "Aborted"})

            smach.StateMachine.add('PICKUP_OBJECT',
                                    GetObject(robot=robot, 
                                              side=arm, 
                                              roi_query=query_lookat, 
                                              object_query=grab_entity_designator),
                                    transitions={    'Done'   : 'SAY_DROPOFF',
                                                     'Aborted': 'SAY_FOUND_NOTHING',
                                                     'Failed' : 'HUMAN_HANDOVER',
                                                     'Timeout': 'RESET'})

            def generate_drop_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(query_dropoff_loc)
                    if answers:
                        _type = answers[0]["ObjectType"]
                        dropoff = answers[0]["Disposal_type"]
                        return "I will bring this {0} to the {1}".format(_type, dropoff).replace("_", " ")
                    else:
                        return "I will throw this in the wastebin"
                except Exception, e:
                    rospy.logerr(e)
                    return "I'll throw this in the wastebin. I don't know what I'm actually doing"
            smach.StateMachine.add('SAY_DROPOFF',
                                    Say_generated(robot, sentence_creator=generate_drop_object_sentence, block=False),
                                    transitions={ 'spoken':'CHECK_FOR_ANSWERS_OBJECT_CLASS' })

            smach.StateMachine.add( "CHECK_FOR_ANSWERS_OBJECT_CLASS",
                                    Wait_query_true(robot, query_current_object_class, timeout=2),
                                    transitions={   "query_true":"CHECK_FOR_ANSWERS_DROPOFF",
                                                    "timed_out":"CHECK_FOR_ANSWERS_DROPOFF",
                                                    "preempted":"CHECK_FOR_ANSWERS_DROPOFF"})

            smach.StateMachine.add( "CHECK_FOR_ANSWERS_DROPOFF",
                                    Wait_query_true(robot, query_dropoff_loc, timeout=2),
                                    transitions={   "query_true":"DROPOFF_OBJECT",
                                                    "timed_out":"DROPOFF_OBJECT_TRASHBIN",
                                                    "preempted":"DROPOFF_OBJECT_TRASHBIN"})
            
            smach.StateMachine.add('SAY_FOUND_NOTHING',
                                    Say(robot, ["I didn't find anything here", "No objects here", "There are no objects here", "I do not see anything here"]),
                                    transitions={ 'spoken':'RESET' })

            smach.StateMachine.add('HUMAN_HANDOVER',
                                    Human_handover(arm,robot),
                                    transitions={   'succeeded':'DROPOFF_OBJECT',
                                                    'failed':'RESET'})

            smach.StateMachine.add("DROPOFF_OBJECT",
                                    DropObject(arm, robot, query_dropoff_loc),
                                    transitions={   'succeeded':'RESET',
                                                    'failed':'DROPOFF_OBJECT_TRASHBIN',
                                                    'target_lost':'DROPOFF_OBJECT_TRASHBIN'})
            ''' In case of failed: the DropObject state has already done a human handover at the dropoff location (ERIK: ??? GIVE AMIGO THE UNKNOWN OBJECT AND HE WILL PUT IT IN THE TRASHBIN)
                In case of target_lost, it cannot go to the dropoff location but has to hand over the object anyway in order to proceed to the next task'''

            smach.StateMachine.add("DROPOFF_OBJECT_TRASHBIN",
                                    DropObject(arm, robot, query_dropoff_loc_backup),
                                    transitions={   'succeeded':'RESET',
                                                    'failed':'Failed',
                                                    'target_lost':'SAY_HUMAN_HANDOVER'})

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
