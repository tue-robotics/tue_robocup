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

    def __init__(self, robot):
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
#        robot.perception.toggle([])	

        with self:
            #Turn on LASER
            smach.StateMachine.add( "TOGGLE_POSSIBLE_PEOPLE_DETECTION",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"WAIT_FOR_POSSIBLE_DETECTION"})

            smach.StateMachine.add( "WAIT_FOR_POSSIBLE_DETECTION",
                                    states.Wait_query_true(robot, self.possible_person_query, timeout=3),
                                    transitions={   "query_true":"TOGGLE_OFF_POSSIBLE_PEOPLE_DETECTION",
                                                    "timed_out":"Failed",
                                                    "preempted":"Aborted"})

            #Turn off LASER
            smach.StateMachine.add( "TOGGLE_OFF_POSSIBLE_PEOPLE_DETECTION",
                                    states.TogglePeopleDetector(robot, on=False),
                                    transitions={   "toggled":"SET_CURRENT_POSSIBLE_PERSON"})

            #TODO: I have to determine the first person to look at and set that as the current_possible_person
            @smach.cb_interface(outcomes=['asserted', 'all_iterated'])
            def set_current_possible_person(userdata):
		#TODO LOY: If we can't find any person, or they are all ignopred, navigate tot a validated_person.
                answers = self.robot.reasoner.query(self.possible_person_query)
                rospy.loginfo("current possible_persons: " +", ".join([str(ans["ObjectID"]) for ans in answers]))
                if answers:
                    possible_person = answers[0]["ObjectID"] #Just get the first one?
                    rospy.loginfo("Asserting ObjectID {0} as a current_possible_person".format(possible_person))
#		    import ipdb; ipdb.set_trace()
                    self.robot.reasoner.query(Compound("retractall", (Compound("current_possible_person", "X"))))
                    self.robot.reasoner.assertz(Compound("current_possible_person", possible_person))
                    return "asserted"
                else:
                    return "all_iterated"
            smach.StateMachine.add('SET_CURRENT_POSSIBLE_PERSON', smach.CBState(set_current_possible_person),
                                    transitions={   'asserted':'LOOK_AT_POSSIBLE_PERSON',
                                                    'all_iterated':'GOTO_PERSON'})

            #Look at the possible_person detection, to verify through Luis's human_tracking them in a next state
            smach.StateMachine.add('LOOK_AT_POSSIBLE_PERSON',
                                    states.LookAtPoint(robot, self.current_possible_person_query),
                                    transitions={   'looking':'TOGGLE_ON_HUMAN_TRACKING',
                                                    'no_point_found':'SAY_FAILED',
                                                    'abort':'Aborted'})

            #Turn on human_tracking with the kinect
            smach.StateMachine.add( "TOGGLE_ON_HUMAN_TRACKING",
                                    states.ToggleModules(robot, modules=["human_tracking"]),
                                    transitions={   "toggled":"WAIT_FOR_ACTUAL_DETECTION"})

            smach.StateMachine.add( "WAIT_FOR_ACTUAL_DETECTION",
                                    states.Wait_query_true(robot, self.person_query, timeout=10),
                                    transitions={   "query_true":"RETRACT_POSSIBLE",
                                                    "timed_out":"RETRACT_POSSIBLE",
                                                    "preempted":"Aborted"})

            @smach.cb_interface(outcomes=['asserted'])
            def retract_possible_person(userdata):
                answers = self.robot.reasoner.query(self.current_possible_person_query)
                rospy.loginfo("current possible_persons: " +", ".join([str(ans["ObjectID"]) for ans in answers]))
                if answers:
                    possible_person = answers[0]["ObjectID"]
                    rospy.loginfo("ObjectID {0} is current_possible_person, retracting that.".format(possible_person))
                    self.robot.reasoner.assertz(Compound("ignored_person", possible_person))
                    return "asserted"
                else:
                    return "asserted"
            smach.StateMachine.add('RETRACT_POSSIBLE', smach.CBState(retract_possible_person),
                                    transitions={   'asserted':'TOGGLE_OFF_HUMAN_TRACKING'}) #Yes this is a loop, but is is supposed to tick off all persons one by one.


            #Turn off human_tracking with the kinect #TODO: Also do this when the SM fails
            smach.StateMachine.add( "TOGGLE_OFF_HUMAN_TRACKING",
                                    states.ToggleModules(robot, modules=[]),
                                    transitions={   "toggled":"SET_CURRENT_POSSIBLE_PERSON"}) 

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
