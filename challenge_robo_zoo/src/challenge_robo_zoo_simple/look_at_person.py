#! /usr/bin/env python
import rospy

import smach

from psi import Conjunction, Compound, Sequence
import robot_smach_states as states

class LookAtPerson(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        with self:
            smach.StateMachine.add("RESET_SPINDLE_HEAD_UP",
                    states.ResetSpindleHeadUp(robot),
                    transitions={'done':'RESET_REASONER'})
                    
            @smach.cb_interface(outcomes=['done'])
            def reset_reasoner(*args, **kwargs):
                robot.reasoner.reset()
                return 'done'
            smach.StateMachine.add( "RESET_REASONER",
                                    smach.CBState(reset_reasoner),
                                    transitions={"done":"LOOK"})

            smach.StateMachine.add("LOOK",
                    states.LookAtItem(robot, ["face_recognition"], states.LookAtItem.face_in_front_query),
                    transitions={   'Done'      :'Done',
                                    'Aborted'   :'Aborted',
                                    'Failed'    :'Failed'})


class IterateLookAtPerson(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        ignore_predicate = "ignore"
        person_to_look_at_predicate = "person_to_look_at"
        person_to_look_at = Conjunction(   Compound(person_to_look_at_predicate, "ObjectID"), 
                                         Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))
        
	#Make sure the predicate exists and we don't get stupid existence errors
        robot.reasoner.assertz(ignore_predicate, "dummy")
        robot.reasoner.query(Compound("retractall", Compound(ignore_predicate, "X")))

        item_query = Conjunction(
                        Conjunction(Compound("property_expected", "ObjectID", "class_label", "person"),
                                    Compound("property_expected", "ObjectID", "position", Sequence("X","Y","IgnoredZ"))),
                                    Compound("=", "Z", "1.65"),
                        Compound("not", Compound(ignore_predicate, "ObjectID")))

        print item_query

        with self:                   
            @smach.cb_interface(outcomes=['done'])
            def reset_reasoner(*args, **kwargs):
                robot.reasoner.reset()
                return 'done'
            smach.StateMachine.add( "RESET_REASONER",
                                    smach.CBState(reset_reasoner),
                                    transitions={"done":"WAIT_FOR_POSSIBLE_DETECTION"})

            #Wait until there are some objects in the WM that match the query. The object should not be marked to ignore them.
            #If we can't find a match, this can mean there are nu objects that have the right class, 
            #   or that each of those are already ignored. 
            #   If all objects of a class are already ignored, we're done and should unignore the ignored objects
            smach.StateMachine.add( "WAIT_FOR_POSSIBLE_DETECTION",
                                    states.Wait_queried_perception(robot, ["ppl_detection"], item_query, timeout=60),
                                    transitions={   "query_true"    :"SELECT_ITEM_TO_LOOK_AT",
                                                    "timed_out"     :"RETRACT_IGNORE",
                                                    "preempted"     :"Aborted"})

            #Then, of those matching objects, select one to look at and mark that object as the object we should look at.
            smach.StateMachine.add( 'SELECT_ITEM_TO_LOOK_AT', 
                                    states.Select_object(robot, item_query, person_to_look_at_predicate),
                                    transitions={   'selected'      :'LOOK_AT_POSSIBLE_PERSON',
                                                    'no_answers'    :'Failed'})

            #Then, finally, look at it.
            smach.StateMachine.add('LOOK_AT_POSSIBLE_PERSON',
                                    states.LookAtPoint(robot, person_to_look_at),
                                    transitions={   'looking'       :'LOOK_AT_FACE',
                                                    'no_point_found':'Failed',
                                                    'abort'         :'Aborted'})

            smach.StateMachine.add("LOOK_AT_FACE",
                    states.LookAtItem(robot, ["face_recognition"], states.LookAtItem.face_in_front_query, timeout=3),
                    transitions={   'Done'      :'IGNORE_CURRENTLY_LOOKED_AT',
                                    'Aborted'   :'Aborted',
                                    'Failed'    :'IGNORE_CURRENTLY_LOOKED_AT'})

            #Mark the selected object as that it should be ignored. If there is no such item, retract all ignores
            #If we did succesfully ignore the item we just looked at, unmark the object as the object we should look at. 
            #   The result is that there is no item to look at
            smach.StateMachine.add( 'IGNORE_CURRENTLY_LOOKED_AT', 
                                    states.Select_object(robot, person_to_look_at, ignore_predicate, retract_previous=False),
                                    transitions={   'selected'      :'RETRACT_ITEM_TO_LOOK_AT',
                                                    'no_answers'    :'RETRACT_IGNORE'})

            #Un-select the object we were looking at
            smach.StateMachine.add( 'RETRACT_ITEM_TO_LOOK_AT', 
                                    states.Retract_facts(robot, Compound(person_to_look_at_predicate, "Item")),
                                    transitions={   'retracted'     :'WAIT_FOR_POSSIBLE_DETECTION'})

            #When we're done, and can't find matches to the WM query, retract all ignores we introduced so the WM is normal again
            smach.StateMachine.add( 'RETRACT_IGNORE', 
                                    states.Retract_facts(robot, Compound(ignore_predicate, "Item")),
                                    transitions={   'retracted'     :'Done'})

if __name__ == "__main__":
    rospy.init_node("executive_look_at_person")
    
    from robot_smach_states.util.startup import startup
    startup(IterateLookAtPerson)
