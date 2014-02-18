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
    go to the closest one and say something nice.

    Finding persons should use the following logic:
        First, switch on the laser_people_detector made by Jos. Gives a lot of false positives, but that OK as we filter them later on. 
            This step should assert "possible_person" objects in the world model. Turn it off directly after detecting any person.

        Second, for each of the possible_persons, look at them and turn on Luis's human_tracking. 
        If it finds anyone, it will put a "person" in the world model. 
        After looking, retract the possible_person-fact for the current iteration.
        Then, we could even trigger face_segmentation, but as we want to identify a person, this already happens.

    The sequence or actions and knowledge present should be something like this:

    - Toggle the laser ppl_detection, to given lots of person, with lots of false positives.
      This yields some facts in the world model:
        instance_of(obj1, person).
        instance_of(obj2, person).
        instance_of(obj3, person).                #obj3 is a false positive, so NOT actually a person
        property_expected(obj1, position, 1,0,0).
        property_expected(obj2, position, 2,0,0).
        property_expected(obj3, position, 3,0,0). #obj3 is NOT an actual person

    - Then we toggle the laser ppl_detection off again.

    - We determine the current_possible_person by getting the first answer to the possible_person_query
      This asserts current_possible_person(obj1) to the world model.
    - We look at the position of obj1, so 1,0,0, by finding the answer to self.current_possible_person_query
    - human_tracking is turned on, which puts these facts in the WM:
        instance_of(obj4, validated_person).   #obj4 is a new object!
        property_expected(obj4, position, 1,0,0).
    - Whatever the detection, we ignore the current_possible_person, because we don't want to look at that person again. So we add:
        ignored_person(obj1).

    - We determine the current_possible_person by getting the first answer to the possible_person_query.
      This asserts current_possible_person(obj2) to the world model, because the query ignores obj1 already
    - We look at the position of obj2, so 2,0,0, by finding the answer to self.current_possible_person_query
    - human_tracking is turned on, which puts these facts in the WM:
        instance_of(obj5, validated_person).   #obj5 is a new object!
        property_expected(obj5, position, 2,0,0).
    - Whatever the detection, we ignore the current_possible_person, because we don't want to look at that person again. So we add:
        ignored_person(obj2).

    - We determine the current_possible_person by getting the first answer to the possible_person_query.
      This asserts current_possible_person(obj3) to the world model, because the query ignores obj1 already
    - We look at the position of obj3, so 3,0,0, by finding the answer to self.current_possible_person_query
    - human_tracking is turned on, we can't find a new object, as obj3 was a false positive
    - Whatever the detection, we ignore the current_possible_person, because we don't want to look at that person again. So we add:
        ignored_person(obj3).
   - So, now we have these facts in the world model:
       instance_of(obj1, person).
        instance_of(obj2, person).
        instance_of(obj3, person).
        property_expected(obj1, position, 1,0,0).
        property_expected(obj2, position, 2,0,0).
        property_expected(obj3, position, 3,0,0).
        instance_of(obj4, validated_person).
        property_expected(obj4, position, 1,0,0).
        ignored_person(obj1).
        instance_of(obj5, validated_person).
        property_expected(obj5, position, 2,0,0).
        ignored_person(obj2).
        ignored_person(obj3).
      Thus, we have 2 validated_persons. We can iterate over them or go to the closest one.

    """

    def __init__(self, robot, detect_persons=True):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self.robot = robot
        self.possible_person_query = Conjunction(   Compound("instance_of", "ObjectID", "person"), 
                                                    Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                                                    Compound("not", Compound("ignored_person", "ObjectID")))

        self.current_possible_person_query = Conjunction(   Compound("current_possible_person", "ObjectID"), 
                                                            Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        self.person_query = Conjunction(  Compound("instance_of", "ObjectID", "validated_person"), 
                                          Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        robot.reasoner.assertz(Compound("ignored_person", "nobody")) #Required to make the predicate known.
        robot.reasoner.retractall(Compound("current_possible_person", "ObjectID"))
        #robot.perception.toggle([])	

        with self:
            if detect_persons:
                #Turn on ppl_detection and switch off after we detected a person
                smach.StateMachine.add( "WAIT_FOR_POSSIBLE_DETECTION",
                                        states.Wait_queried_perception(robot, ["ppl_detection"], self.possible_person_query, timeout=3),
                                        transitions={   "query_true":"SET_CURRENT_POSSIBLE_PERSON",
                                                        "timed_out":"Failed",
                                                        "preempted":"Aborted"})

            smach.StateMachine.add( 'SET_CURRENT_POSSIBLE_PERSON', 
                                    states.Select_object(robot, self.possible_person_query, "current_possible_person"),
                                    transitions={   'selected':'LOOK_AT_POSSIBLE_PERSON',
                                                    'no_answers':'GOTO_PERSON'})

            #Look at the possible_person detection, to verify through Luis's human_tracking them in a next state
            smach.StateMachine.add('LOOK_AT_POSSIBLE_PERSON',
                                    states.LookAtPoint(robot, self.current_possible_person_query),
                                    transitions={   'looking':'WAIT_FOR_HUMAN_DETECTION',
                                                    'no_point_found':'SAY_FAILED',
                                                    'abort':'Aborted'})

            #Turn on human_tracking with the kinect and wait for a match of it in the world model
            smach.StateMachine.add( "WAIT_FOR_HUMAN_DETECTION",
                                    states.Wait_queried_perception(robot, ["human_tracking"], self.person_query, timeout=10),
                                    transitions={   "query_true":"RETRACT_POSSIBLE",
                                                    "timed_out":"RETRACT_POSSIBLE",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add('RETRACT_POSSIBLE', 
                                    states.Select_object(robot, self.current_possible_person_query, "ignored_person"),
                                    transitions={   'selected':'GOTO_PERSON',#Yes this is a loop, but is is supposed to tick off all persons one by one.
                                                    'no_answers':'GOTO_PERSON'}) 

            smach.StateMachine.add( "GOTO_PERSON",
                                    states.NavigateGeneric(robot, lookat_query=self.person_query, xy_dist_to_goal_tuple=(1.0,0)),
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
