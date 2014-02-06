#!/usr/bin/env python
import roslib
roslib.load_manifest('challenge_atomic_actions')
import rospy

from robot_skills.reasoner import Compound, Conjunction, Sequence
import smach
import robot_smach_states as states
from robot_smach_states.util.startup import startup

class FindMe(smach.StateMachine):
    """The Find me basic functionality. 
    The robot must find a person again after it moves to a different room.
    In that room, the robot has to find the right person from a group of 5 persons.
    After it found the right person, 
        it must announce that it did and then go to the side of the person where he/she (probable a he) points to, left or right.

    The state machine is:

    Learn_Person
        |
        v
    Wait(10s)
        |
    NavigateGeneric(PeopleRoom)
        |
    TogglePeopleDetector
        |
        v
    LookAtQueryPoint(PossileOperator) {----------------+
        |                                              |
        v                                              |
    Identify()----->Assert(currentPersonIsNotOperator)-+
        |
        v
    Say(Hi)
        |
        v
    DetectLeftRight
        |
        v
    NavigateGeneric(DetectedPoint)
        |
        v
      Done
    """
    
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        
        self.robot = robot

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
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    
        #robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/cleanup_test.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "find_me"))

        self.room_query = Compound("waypoint", "find_me_room", Compound("pose_2d", "X", "Y", "Phi"))

        #I'm looking for a person of which I don't yet know that is not my operator, so persons that *could* be the operator.
        self.human_query = Conjunction( Compound("instance_of", "ObjectID", "person"), 
                                        Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                                        Compound("not", Compound("not_operator", "ObjectID"))) 

        query_detect_person = Conjunction(  Compound("property_expected", "ObjectID", "class_label", "person"),
                                            Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        with self:
            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={  'initialized': 'LEARN_FACE',
                                                   'abort':       'LEARN_FACE'})
            smach.StateMachine.add( 'LEARN_FACE',
                                    states.Learn_Person(robot, "operator"),
                                    transitions={   'face_learned':'SAY_WAIT',
                                                    'learn_failed':'SAY_WAIT'}) #Even if we fail, still go to the other room and try to recgnize a person

            smach.StateMachine.add( 'SAY_WAIT',
                                    states.Say(robot, ["I'll come after you in a couple of seconds"]),
                                    transitions={"spoken":"WAIT"})

            smach.StateMachine.add( 'WAIT',
                                    states.Wait_time(robot, waittime=5),
                                    transitions={   'waited'   :"GOTO_ROOM",
                                                    'preempted':"GOTO_ROOM"})

            smach.StateMachine.add('GOTO_ROOM',
                                    states.NavigateGeneric(robot, goal_query=self.room_query),
                                    transitions={   "arrived":"DETECT_PERSONS", 
                                                    "unreachable":"DETECT_PERSONS", 
                                                    "preempted":"DETECT_PERSONS", 
                                                    "goal_not_defined":"DETECT_PERSONS"})

            ########## In the room, find persons and look at them to identify them##########
            smach.StateMachine.add( "DETECT_PERSONS",
                                    states.TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"WAIT_FOR_DETECTION"})

            smach.StateMachine.add( "WAIT_FOR_DETECTION",
                                    states.Wait_query_true(robot, self.human_query, timeout=5),
                                    transitions={   "query_true":"TOGGLE_OFF",
                                                    "timed_out":"SAY_FAILED_NO_PERSONS",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add( "SAY_FAILED_NO_PERSONS",
                                  states.Say(robot, ["I couldn't find anyone, sorry"], mood="sad"),
                                  transitions={"spoken":"Failed"})

            smach.StateMachine.add( "TOGGLE_OFF",
                                    states.TogglePeopleDetector(robot, on=False),
                                    transitions={   "toggled":"GOTO_PERSON"})

            smach.StateMachine.add( "GOTO_PERSON",
                                    states.NavigateGeneric(robot, lookat_query=self.human_query, xy_dist_to_goal_tuple=(1.5,0)),
                                    transitions={   "arrived":"SAY_SOMETHING",
                                                    "unreachable":'SAY_SOMETHING', #TODO: Not sure this is wise to, we could be facing someone OR not...
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_COULD_NOT_FIND_PERSON'})

            smach.StateMachine.add( "SAY_SOMETHING",
                                  states.Say(robot, ["Hi there, let me see if you are my operator"]),
                                  transitions={"spoken":"IDENTIFY_OPERATOR"})

            #### FAILING CASES ####
            smach.StateMachine.add( "SAY_COULD_NOT_FIND_PERSON",
                                  states.Say(robot, ["I couldn't find a person just yet, but lets take a look around"], mood="sad"),
                                  transitions={"spoken":"IDENTIFY_OPERATOR"})

            smach.StateMachine.add( "IDENTIFY_OPERATOR",
                                  states.Say(robot, ["Hmm, are you indeed my operator?"]),
                                  transitions={"spoken":"FOUND_OPERATOR"})
                                  #TODO: replace with actual identifcation state. one transition should be NOT_OPERATOR

            smach.StateMachine.add( "NOT_OPERATOR", #TODO: Not yet called from anywhere. IDENTIFY_OPERATOR should be a new Identify state
                                  states.Say(robot, ["Nope, sorry, i'm not looking for you. "], mood="sad"),
                                  transitions={"spoken":"ASSERT_CURRENT_NOT_OPERATOR"})

            @smach.cb_interface(outcomes=['asserted'])
            def assert_current_not_operator(userdata):
                answers = self.robot.reasoner.query(query_detect_person)
                rospy.loginfo("These IDs are in from of me: " +", ".join([ans["ObjectID"] for ans in answers]))
                if answers:
                    person = answers[0]["ObjectID"]
                    rospy.loginfo("ObjectID {0} is in front of me but not operator.".format(person))
                    self.robot.reasoner.assertz(Compound("not_operator", person))
                else:
                    return "asserted"
            smach.StateMachine.add('ASSERT_CURRENT_NOT_OPERATOR', smach.CBState(assert_current_not_operator),
                                    transitions={   'asserted':'GOTO_PERSON'}) #Yes this is a loop, but is is supposed to tick off all persons one by one. 

            #### SUCCESS! ####
            smach.StateMachine.add( "FOUND_OPERATOR",
                                  states.Say(robot, ["Hey, I found you!"], mood="excited"),
                                  transitions={"spoken":"DETECT_LEFT_RIGHT"})

            smach.StateMachine.add( "DETECT_LEFT_RIGHT",
                                  states.Say(robot, ["Hmmm, let me see if I should go to your left or right"], mood="neutral"),
                                  transitions={"spoken":"GOTO_RIGHT"})

            smach.StateMachine.add( "GOTO_RIGHT",
                                    states.NavigateGeneric(robot, lookat_query=self.human_query, goal_area_radius=0.5), #just go left or right, good enough
                                    transitions={   "arrived":"Done",
                                                    "unreachable":'Failed',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Failed'})

if __name__ == "__main__":
    rospy.init_node('find_me_exec')
    
    startup(FindMe)
