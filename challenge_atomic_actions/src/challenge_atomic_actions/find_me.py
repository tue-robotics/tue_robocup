#!/usr/bin/env python
import roslib
roslib.load_manifest('challenge_atomic_actions')
import rospy

from robot_skills.reasoner import Compound, Conjunction, Sequence
import smach
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from drive_to_person import DriveToClosestPerson

class Identify(smach.StateMachine):

    def __init__(self, robot, desired_person=None):
        smach.StateMachine.__init__(self, outcomes=['name_found', 'name_not_found'])
        
        self.robot = robot

        self.desired_person = str(desired_person).lower() if desired_person else "Name" #if we care about the name, use that or else make it variable and don't care.

        self.name_in_front = Conjunction(  Compound("property_expected", "ObjectID", "class_label", "person"),
                                            Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")),
                                            Compound("property_expected", "ObjectID", "name", self.desired_person))

        with self:
            smach.StateMachine.add( "TOGGLE_ON_FACE_RECOGNITION",
                                    states.ToggleModules(robot, modules=["face_recognition"]),
                                    transitions={   "toggled":"WAIT_FOR_FACE_IN_FRONT"})
            
            smach.StateMachine.add( "WAIT_FOR_FACE_IN_FRONT",
                                    states.Wait_query_true(robot, self.name_in_front, timeout=5), #Wait until we have something with a name in front of us
                                    transitions={   "query_true":"TOGGLE_OFF_FACE_RECOGNITION",
                                                    "timed_out":"TOGGLE_OFF_FACE_RECOGNITION_FAILED",
                                                    "preempted":"TOGGLE_OFF_FACE_RECOGNITION_FAILED"})

            smach.StateMachine.add( "TOGGLE_OFF_FACE_RECOGNITION",
                                    states.ToggleModules(robot, modules=[]),
                                    transitions={   "toggled":"name_found"})

            smach.StateMachine.add( "TOGGLE_OFF_FACE_RECOGNITION_FAILED",
                                    states.ToggleModules(robot, modules=[]),
                                    transitions={   "toggled":"name_not_found"})

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
        |
        v   DriveToClosestPerson
   +---------------------------------------------+
   |TogglePeopleDetector                         |
   |    |                                        |
   |    v                                        |
   |LookAround                                   |
   |    |                                        |
   |    v                                        |
   |NavigateGeneric(PossileOperator)             |{----+
   |    |                                        |     |
   +----+----------------------------------------+     |
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

        self.query_detect_person = Conjunction(  Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                            Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.query_detect_face = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.pointing_query = Conjunction(self.query_detect_person, Compound("pointing_at", "ObjectID", "Direction"))

        with self:
            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={  'initialized': 'TOGGLE_ON_FACE_SEGMENTATION',
                                                   'abort':       'TOGGLE_ON_FACE_SEGMENTATION'})

            #### WAIT FOR A PERSON IN FROM OF THE ROBOT
            smach.StateMachine.add( "TOGGLE_ON_FACE_SEGMENTATION",
                                    states.ToggleModules(robot, modules=["human_tracking"]),
                                    transitions={   "toggled":"WAIT_FOR_FACE_IN_FRONT"})
            
            smach.StateMachine.add( "WAIT_FOR_FACE_IN_FRONT",
                                    states.Wait_query_true(robot, self.query_detect_person, timeout=5),
                                    transitions={   "query_true":"TOGGLE_OFF_FACE_SEGMENTATION",
                                                    "timed_out":"TOGGLE_OFF_FACE_SEGMENTATION",
                                                    "preempted":"TOGGLE_OFF_FACE_SEGMENTATION"})

            smach.StateMachine.add( "TOGGLE_OFF_FACE_SEGMENTATION",
                                    states.ToggleModules(robot, modules=[]),
                                    transitions={   "toggled":"SAY_HI"})

            smach.StateMachine.add( 'SAY_HI',
                                    states.Say(robot, ["Hi there. Let me learn your face before we play hide and seek."]),
                                    transitions={"spoken":"LEARN_FACE"})

            smach.StateMachine.add( 'LEARN_FACE',
                                    states.Learn_Person(robot, "operator"),
                                    transitions={   'face_learned':'SAY_WAIT',
                                                    'learn_failed':'SAY_WAIT'}) #Even if we fail, still go to the other room and try to recgnize a person

            smach.StateMachine.add( 'SAY_WAIT',
                                    states.Say(robot, ["I'll come after you in a couple of seconds"]),
                                    transitions={"spoken":"ASSERT_LEARNED_NOT_OPERATOR"})

            #Ignore the validated_person we just learn in the first room somehow. If that's the closest, don't go back.
            #Obviously, the current person IS in fact the operator, but we don't track that person so it gets a new ID in the other room
            smach.StateMachine.add( "ASSERT_LEARNED_NOT_OPERATOR",
                                    states.Select_object(robot, self.query_detect_person, "not_operator", retract_previous=False),
                                    transitions={   'selected':'WAIT', 
                                                    'no_answers':'WAIT'})

            smach.StateMachine.add( 'WAIT',
                                    states.Wait_time(robot, waittime=5),
                                    transitions={   'waited'   :"GOTO_ROOM",
                                                    'preempted':"GOTO_ROOM"})

            smach.StateMachine.add('GOTO_ROOM',
                                    states.NavigateGeneric(robot, goal_query=self.room_query),
                                    transitions={   "arrived":"GOTO_CLOSEST_PERSON", 
                                                    "unreachable":"GOTO_CLOSEST_PERSON", 
                                                    "preempted":"GOTO_CLOSEST_PERSON", 
                                                    "goal_not_defined":"GOTO_CLOSEST_PERSON"})

            ########## In the room, find persons and look at them to identify them##########
            #TODO: We should be able to optionally only drive to a closest person within an ROI, or a room.
            smach.StateMachine.add( "GOTO_CLOSEST_PERSON",
                                    DriveToClosestPerson(robot),
                                    transitions={   'Done':"SAY_SOMETHING",
                                                    'Aborted':"Aborted",
                                                    'Failed':"SAY_COULD_NOT_FIND_PERSON"})

            smach.StateMachine.add( "SAY_SOMETHING",
                                  states.Say(robot, ["Hi there, let me see if you are my operator"]),
                                  transitions={"spoken":"IDENTIFY_OPERATOR"})

            #### FAILING CASES ####
            smach.StateMachine.add( "SAY_COULD_NOT_FIND_PERSON",
                                  states.Say(robot, ["I couldn't find a person just yet, but lets take a look around"], mood="sad"),
                                  transitions={"spoken":"IDENTIFY_OPERATOR"})

            smach.StateMachine.add( "IDENTIFY_OPERATOR",
                                  Identify(robot, desired_person="operator"),
                                  transitions={ "name_found":"FOUND_OPERATOR",
                                                "name_not_found":"NOT_OPERATOR"})
                                  #TODO: replace with actual identifcation state. one transition should be NOT_OPERATOR

            smach.StateMachine.add( "NOT_OPERATOR", #TODO: Not yet called from anywhere. IDENTIFY_OPERATOR should be a new Identify state
                                    states.Say(robot, ["Nope, sorry, i'm not looking for you. "], mood="sad"),
                                    transitions={"spoken":"ASSERT_CURRENT_NOT_OPERATOR"})

            smach.StateMachine.add( "ASSERT_CURRENT_NOT_OPERATOR",
                                    states.Select_object(robot, self.query_detect_person, "not_operator"),
                                    transitions={   'selected':'GOTO_CLOSEST_PERSON', 
                                                    'no_answers':'GOTO_CLOSEST_PERSON'})

            #### SUCCESS! ####
            smach.StateMachine.add( "FOUND_OPERATOR",
                                  states.Say(robot, ["Hey, I found you!"], mood="excited"),
                                  transitions={"spoken":"DETECT_LEFT_RIGHT"})

            smach.StateMachine.add( "DETECT_LEFT_RIGHT",
                                    states.Wait_queried_perception(robot, ["human_tracking"], self.pointing_query, timeout=5),
                                    transitions={   "query_true":"GOTO_SIDE",
                                                    "timed_out":"GOTO_SIDE", #TODO: Is this wise to do?
                                                    "preempted":"Aborted"})

            smach.StateMachine.add( "GOTO_SIDE",
                                    states.NavigateGeneric(robot, lookat_query=self.query_detect_person, goal_area_radius=0.5, xy_dist_to_goal_tuple=(0.0,1.0)), #just go left or right, good enough
                                    transitions={   "arrived":"Done",
                                                    "unreachable":'Failed',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Failed'})

if __name__ == "__main__":
    rospy.init_node('find_me_exec')
    
    startup(FindMe)
