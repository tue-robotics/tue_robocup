#! /usr/bin/env python
import roslib; roslib.load_manifest("robot_smach_states")
import rospy
import smach

import utility_states
import human_interaction
import perception
import navigation
import manipulation
import reasoning

from psi import Conjunction, Compound

# Wait_for_door state thought the reasoner 
class Check_object_found_before(smach.State):
    def __init__(self, robot, object_query):
        smach.State.__init__(self, outcomes=['object_found','no_object_found'])
        
        self.robot = robot
        self.object_query = object_query
        
    def execute(self,userdata):

        # Query reasoner for objects
        object_answers = self.robot.reasoner.query(self.object_query)

        if not object_answers:

            return 'no_object_found'
        else:
             # TODO Loy: Go to object closest to amigo 
             # and 
             # TODO ERIK: has not been picked up yet! Since Amigo does not removes objects after transporting.
             #           This should be asserted to reasoner object_picked_up(ObjectID) and checked with object_query
            return 'object_found'     



class StartChallenge(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    def __init__(self, robot, initial_pose, goto_query):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add( "INITIALIZE", 
                                    utility_states.Initialize(robot), 
                                    transitions={   "initialized"   :"INIT_POSE",
                                                    "abort"         :"Aborted"})

            smach.StateMachine.add('INIT_POSE',
                                utility_states.Set_initial_pose(robot, initial_pose),
                                transitions={   'done':'INSTRUCT_WAIT_FOR_DOOR',
                                                'preempted':'Aborted',
                                                'error':'Aborted'})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                    human_interaction.Say(robot, [  "I will now wait until the door is opened", 
                                                                    "Knockknock, may I please come in?"]),
                                    transitions={   "spoken":"ASSESS_DOOR"})


             # Start laser sensor that may change the state of the door if the door is open:
            smach.StateMachine.add( "ASSESS_DOOR", 
                                    perception.Read_laser(robot, "entrance_door"),
                                    transitions={   "laser_read":"WAIT_FOR_DOOR"})       
            
            # define query for the question wether the door is open in the state WAIT_FOR_DOOR
            dooropen_query = robot.reasoner.state("entrance_door","open")
        
            # Query if the door is open:
            smach.StateMachine.add( "WAIT_FOR_DOOR", 
                                    reasoning.Ask_query_true(robot, dooropen_query),
                                    transitions={   "query_false":"ASSESS_DOOR",
                                                    "query_true":"THROUGH_DOOR",
                                                    "waiting":"DOOR_CLOSED",
                                                    "preempted":"Aborted"})

            # If the door is still closed after certain number of iterations, defined in Ask_query_true 
            # in perception.py, amigo will speak and check again if the door is open
            smach.StateMachine.add( "DOOR_CLOSED",
                                    human_interaction.Say(robot, "Door is closed, please open the door"),
                                    transitions={   "spoken":"ASSESS_DOOR"}) 

            # If the door is open, amigo will say that it goes to the registration table
            smach.StateMachine.add( "THROUGH_DOOR",
                                    human_interaction.Say(robot, "Door is open, so I will start my task"),
                                    transitions={   "spoken":"ENTER_ROOM"}) 

            smach.StateMachine.add('ENTER_ROOM',
                                    navigation.Navigate_to_queryoutcome(robot, goto_query, X="X", Y="Y", Phi="Phi"),
                                    transitions={   "arrived":"Done", 
                                                    "preempted":"Aborted", 
                                                    "unreachable":"Failed", 
                                                    "goal_not_defined":"Failed"})

class StartChallengeRobust(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    def __init__(self, robot, initial_pose):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add( "INITIALIZE", 
                                    utility_states.Initialize(robot), 
                                    transitions={   "initialized"   :"INIT_POSE",
                                                    "abort"         :"Aborted"})

            smach.StateMachine.add('INIT_POSE',
                                utility_states.Set_initial_pose(robot, initial_pose),
                                transitions={   'done':'INSTRUCT_WAIT_FOR_DOOR',
                                                'preempted':'Aborted',
                                                'error':'Aborted'})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                    human_interaction.Say(robot, [  "I will now wait until the door is opened", 
                                                                    "Knockknock, may I please come in?"]),
                                    transitions={   "spoken":"ASSESS_DOOR"})


             # Start laser sensor that may change the state of the door if the door is open:
            smach.StateMachine.add( "ASSESS_DOOR", 
                                    perception.Read_laser(robot, "entrance_door"),
                                    transitions={   "laser_read":"WAIT_FOR_DOOR"})       
            
            # define query for the question wether the door is open in the state WAIT_FOR_DOOR
            dooropen_query = robot.reasoner.state("entrance_door","open")
        
            # Query if the door is open:
            smach.StateMachine.add( "WAIT_FOR_DOOR", 
                                    reasoning.Ask_query_true(robot, dooropen_query),
                                    transitions={   "query_false":"ASSESS_DOOR",
                                                    "query_true":"THROUGH_DOOR",
                                                    "waiting":"DOOR_CLOSED",
                                                    "preempted":"Aborted"})

            # If the door is still closed after certain number of iterations, defined in Ask_query_true 
            # in perception.py, amigo will speak and check again if the door is open
            smach.StateMachine.add( "DOOR_CLOSED",
                                    human_interaction.Say(robot, "Door is closed, please open the door"),
                                    transitions={   "spoken":"ASSESS_DOOR"}) 

            # If the door is open, amigo will say that it goes to the registration table
            smach.StateMachine.add( "THROUGH_DOOR",
                                    human_interaction.Say(robot, "Door is open, so I will start my task"),
                                    transitions={   "spoken":"ENTER_ROOM"}) 

            smach.StateMachine.add('ENTER_ROOM',
                                    GotoForMeetingpoint(robot),
                                    transitions={   "found":"Done", 
                                                    "not_found":"ENTER_ROOM", 
                                                    "no_goal":"FORCE_DRIVE_THROUGH_DOOR",
                                                    "all_unreachable":"FORCE_DRIVE_THROUGH_DOOR"})

            smach.StateMachine.add('FORCE_DRIVE_THROUGH_DOOR',
                                    ForceDrive(robot),
                                    transitions={   "done":"Done"})

class GotoForMeetingpoint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["no_goal" , "found", "not_found", "all_unreachable"])
        self.robot = robot
        self.goto_query = Compound("waypoint", Compound("meeting_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi"))

    def execute(self, userdata=None):
        # Move to the next waypoint in the storage room        
        reachable_goal_answers = self.robot.reasoner.query(
                                    Conjunction(
                                        Compound("waypoint", Compound("meeting_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("unreachable", Compound("meeting_point", "Waypoint")))))

        if not reachable_goal_answers:
            # check if there are any meeting points (someone may have forgotten to specify them)            
            all_goal_answers = self.robot.reasoner.query(self.goto_query)
            if not all_goal_answers:
                self.robot.speech.speak("No-one has specified meeting locations. Please do so in the locations file!")
                return "all_unreachable"
            else:
                self.robot.speech.speak("There are a couple of meeting points, but they are all unreachable. Sorry.")
                return "all_unreachable"

        # for now, take the first goal found
        goal_answer = reachable_goal_answers[0]

        self.robot.speech.speak("I'm coming to the meeting point!")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = navigation.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        #import ipdb; ipdb.set_trace()

        if nav_result == "unreachable":  #Compound("meeting_point", waypoint_name)
            self.robot.reasoner.query(Compound("assert", Compound("unreachable", Compound("meeting_point", waypoint_name))))
            return "not_found"
        elif nav_result == "preempted":
            return "not_found"
        elif nav_result == "arrived":
            self.robot.speech.speak("I reached a meeting point")
            self.robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
            return "found"
        else: #goal not defined
            self.robot.speech.speak("I really don't know where to go, oops.")
            return "no_goal"

class ForceDrive(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.speech.speak("As a back-up scenario I will now drive through the door with my eyes closed.")
        self.robot.base.force_drive(0.25, 0, 0, 6.0)    # x, y, z, time in seconds
        return "done"

class Learn_Person_SM(smach.StateMachine):
    def __init__(self, robot, testmode=False):
        smach.StateMachine.__init__(self, 
            outcomes=["person_learned", "learning_failed"],
            input_keys=['rate'], #TODO Loy Eric Janno: Why do we need command everywhere? Look needed it, now removed
            output_keys=["name", "ID"])
            
        self.robot = robot
        assert hasattr(self.robot, "perception")
        assert hasattr(self.robot, "reasoner")
            
        with self:
            smach.StateMachine.add( "RETRACT_OLD_NAMES",
                                    reasoning.Retract_facts(robot, [Compound("name_to_learn", "X")]),
                                    transitions={   'retracted':'SAY_AWAIT_PERSON'})

            smach.StateMachine.add( "SAY_AWAIT_PERSON",
                                    human_interaction.Say(robot, 
                                        "I'm waiting for a new person to appear, please step in front of me."),
                                        # ["I'm waiting for a new person to appear, please step in front of me.",
                                        #  "I'm ready for a new person to be learned, please step in front of me."]),
                                    transitions={   "spoken":"GREET"})
                                    #transitions={   "spoken":"WAIT_PERSON"})

            # wait_person_query = Conjunction(
            #     Compound( "type", "Object", "person"),
            #     #Compound( "type", "Object", "ComplexType"), #TODO Sjoerd/Loy: someway query persons here.
            #     Compound("position", "Object", Compound("point", "X", "Y", "Z")))
            # smach.StateMachine.add( "WAIT_PERSON",
            #                         reasoning.Wait_query_true(robot,wait_person_query, 30, pre_callback=lambda ud: self.robot.perception.toggle_recognition(faces=True)),
            #                         transitions={   "query_true":"LOOK_AT_PERSON",
            #                                         "timed_out" :"SAY_NO_PERSON",
            #                                         "preempted" :"learning_failed"})

            # smach.StateMachine.add( "SAY_NO_PERSON",
            #                         human_interaction.Say(robot, ["This took me too long, I'm very sorry"]),
            #                         transitions={   "spoken":"learning_failed"})
            
            # query_object = Conjunction(Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
            # smach.StateMachine.add('LOOK_AT_PERSON',
            #                     perception.LookForObjectsAtROI(robot, query_object, wait_person_query),
            #                     transitions={   'looking':'LOOK_AT_PERSON',
            #                                     'object_found':'GREET',
            #                                     'no_object_found':'GREET',
            #                                     'abort':'learning_failed'}) 
            
            smach.StateMachine.add('GREET', 
                                   human_interaction.Say(self.robot, sentence="Hello. My name is Amigo."),
                                   transitions={'spoken':'ASK_NAME'})
            
            #explicitly do not set the default_option
            #Get the sentence from userdata.
            names_dict = {  "charlie"   :Compound("name_to_learn", "charlie"),
                            "john"      :Compound("name_to_learn", "john"),
                            "william"   :Compound("name_to_learn", "william"),
                            "maria"     :Compound("name_to_learn", "maria"),
                            "alex"      :Compound("name_to_learn", "alex")}

            smach.StateMachine.add('ASK_NAME',
                                   human_interaction.Timedout_QuestionMachine(robot=self.robot, 
                                                            default_option=False,  
                                                            sentence="What is your name?",
                                                            options=names_dict), 
                                   remapping={'answer':'name'},
                                   transitions={'answered':'INSTRUCT',
                                                'not_answered':'GENERATE_ALT_NAME'})
            
            def generate_instruction(userdata):
                query = Compound("name_to_learn", "Name")
                answers = self.robot.reasoner.query(query)
                if answers:
                    current_person = answers[0]["Name"]
                    return "Hi {0}, I'll now learn what you look like, please look into my eyes.".format(current_person)
                else:
                    "Something went terribly wrong, I forgot your name already."
            smach.StateMachine.add('INSTRUCT',
                                   human_interaction.Say_generated(robot=robot, 
                                                 sentence_creator=generate_instruction),
                                   transitions={'spoken':'CALL_LEARN_PERSON'})
            
            '''GENERATE_ALT_NAME generates a sentence specific for the user and stores it in userdata.sentence'''
            alternative_names = ['Z', 'Y', 'X']
            @smach.cb_interface(input_keys=['name'],
                output_keys=['name'],
                outcomes=['name_generated'])
            def generate_alt_name(userdata):
                index = alternative_names.pop()
                name = "mr {0}".format(index)
                self.robot.reasoner.assertz(Compound("name_to_learn", name))
                self.robot.speech.speak("I will call you {0} from now on.".format(name))
                return 'name_generated'
                    
            smach.StateMachine.add('GENERATE_ALT_NAME', 
                                   smach.CBState(generate_alt_name),
                                   transitions={'name_generated':'INSTRUCT'})
            
            # @smach.cb_interface(input_keys=['name', 'target'],
            #     outcomes=['face_learned', 'learn_failed'])
            # def call_learn_person(userdata, robot):
            #     q = Compound("name_to_learn", "Name")
            #     answers = self.robot.reasoner.query(q)
            #     if answers:
            #         current_person = answers[0]["Name"]
            #         result = robot.perception.learn_person(current_person)
            #         if result:
            #             return 'face_learned'
            #         else:
            #             return 'learn_failed'
            #     else:
            #         return 'learn_failed'
                    
            smach.StateMachine.add('CALL_LEARN_PERSON', 
                                   perception.Learn_Person(self.robot),#smach.CBState(call_learn_person, cb_kwargs={'robot':self.robot}),
                                   transitions={'face_learned':'INFORM_SUCCESS',
                                                'learn_failed':'INFORM_FAIL'})
            
            smach.StateMachine.add('INFORM_FAIL',
                                   human_interaction.Say(self.robot, "I could not learn you as well as I wanted, \
                                                    but I will do my best to recognize you."),
                                   transitions={'spoken':'learning_failed'}) #TODO: Test recognition
            
            smach.StateMachine.add('INFORM_SUCCESS',
                                   human_interaction.Say(self.robot, "Allright, I think I know you now."),
                                   transitions={'spoken':'person_learned'})
            
            # @smach.cb_interface(input_keys =['target', 'name', 'ID'],
            #                     outcomes=['recognized','failed'],
            #                     output_keys=['ID'])
            # def recognize(userdata, robot):
            #     rospy.loginfo("Starting face recognition...")
            #     robot.perception.toggle_recognition(faces=True)
            #     rospy.loginfo("Started face recognition")
            #     rospy.sleep(3)
            #     robot.perception.toggle_recognition(faces=False)
            #     rospy.loginfo("Stopped face recognition")
                
            #     #closest_person = robot.closest_target(class_label='person')

                
            #     #import pdb; pdb.set_trace()
            #     if closest_person and closest_person.name.lower() == userdata.name.lower():
            #         userdata.ID = closest_person.ID
            #         return 'recognized'
            #     else:
            #         # wrong_target = robot.closest_target()
            #         # rospy.loginfo('The closest target is (instead of a person):')
            #         # robot.worldmodel.print_info()
            #         return 'failed'
            
            # smach.StateMachine.add('TEST_RECOGNITION',
            #                        smach.CBState(recognize, 
            #                                      cb_kwargs={'robot':self.robot}),#Say(self.robot, "Loy has to program me so that I test if I really learned to recognize you"),
            #                        transitions={'recognized':'person_learned',
            #                                     'failed':'INFORM_FAIL_2'})
            
            # smach.StateMachine.add('INFORM_FAIL_2',
            #                        Say(self.robot, "I did not recognize you when I tested my recognition, but I will do my best to recognize you later on anyway. Fingers crossed!"),
            #                        transitions={'spoken':'learning_failed'})

class GetObject(smach.StateMachine):
    def __init__(self, robot, roi_query, object_query, object_identifier="Object"):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        self.robot = robot
        self.roi_query = roi_query
        self.object_query = object_query

        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "perception")

        with self:
 
            smach.StateMachine.add('CHECK_OBJECT_QUERY',                                            # TODO ERIK: Test this state. Not yet done
                                    Check_object_found_before(robot, self.object_query),
                                    transitions={   'object_found':'SAY_FOUND_SOMETHING_BEFORE',
                                                    'no_object_found':'DRIVE_TO_SEARCHPOS' })
            #import ipdb; ipdb.set_trace()
            smach.StateMachine.add("DRIVE_TO_SEARCHPOS",
                                    navigation.Visit_query_outcome_3d(self.robot, 
                                                                      self.roi_query, 
                                                                      x_offset=0.7, y_offset=0.0001,
                                                                      identifier=object_identifier),  #TODO Bas: when this is 0.0, amingo_inverse_reachability returns a 0,0,0,0,0,0,0 pose
                                    transitions={   'arrived':'SAY_LOOK_FOR_OBJECTS',
                                                    'unreachable':'Failed',
                                                    'preempted':'Aborted',
                                                    'goal_not_defined':'Failed',
                                                    'all_matches_tried':'Failed'})

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    human_interaction.Say(robot, ["Lets see what I can find here."]),
                                    transitions={   'spoken':'LOOK'})

            smach.StateMachine.add('LOOK',
                                    perception.LookForObjectsAtROI(robot, self.roi_query, self.object_query),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'DRIVE_TO_SEARCHPOS',
                                                    'abort':'Aborted'})
                                                    
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    human_interaction.Say(robot, ["I have found something"]),
                                    transitions={ 'spoken':'GRAB' })

            smach.StateMachine.add('SAY_FOUND_SOMETHING_BEFORE',
                                    human_interaction.Say(robot, ["I have seen the desired object before!"]),
                                    transitions={ 'spoken':'GRAB' })

            query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
            smach.StateMachine.add('GRAB',
                                    manipulation.GrabMachine(robot.leftArm, robot, query_grabpoint),
                                    transitions={   'succeeded':'Done',
                                                    'failed':'Failed' })
        
class Say_and_Navigate(smach.StateMachine):
    ## This class gives the ability to say something and at the same time start to navigate to the desired location.
    ## This makes the robot faster and 'look' smarter.
    ##
    ## The input_keys should be defined as follows:
    ## input_keys = ['navigate_named','location_in_database'] 
    ## or
    ## input_keys = ['navigate_goal_location',*3 choices: loc_from, loc_to or object_action*] 
    ## or
    ## input_keys = ['navigate_to_queryoutcome','query'] 
    ## or
    ## input_keys = ['navigate_exact',2.1,0.1,0.2]

    def __init__(self, 
                 sentence, input_keys, robot):
        smach.StateMachine.__init__(self, 
                                    outcomes=["succeeded", "not_at_loc"])
        self.robot = robot
        self.say_sentence = sentence
        
        self.navigate_option = input_keys[0]

        if self.navigate_option == 'navigate_named' or self.navigate_option == 'navigate_goal_location':
            self.navigate_named_location = input_keys[1]
        elif self.navigate_option == 'navigate_exact':
            self.navigate_x = input_keys[1]
            self.navigate_y = input_keys[2]
            self.navigate_phi = input_keys[3]
        elif self.navigate_option == 'navigate_to_queryoutcome':     
            self.nav_query = input_keys[1]

        with self:
            cc = smach.Concurrence(outcomes = ['succeeded', 'not_at_loc'],
                                   default_outcome = 'not_at_loc',
                                   outcome_map = {'succeeded':{'EXECUTE_SAY':'spoken',
                                                                'GO_TO_LOCATION':'arrived'},
                                                  'not_at_loc':{'EXECUTE_SAY':'spoken',
                                                                'GO_TO_LOCATION':'preempted'},
                                                  'not_at_loc':{'EXECUTE_SAY':'spoken',
                                                                'GO_TO_LOCATION':'unreachable'},
                                                  'not_at_loc':{'EXECUTE_SAY':'spoken',
                                                                'GO_TO_LOCATION':'goal_not_defined'}})

            with cc:
                smach.Concurrence.add("EXECUTE_SAY",
                                       human_interaction.Say(robot, self.say_sentence))
                if self.navigate_option == 'navigate_named':
                    smach.Concurrence.add('GO_TO_LOCATION', 
                                                navigation.Navigate_named(robot, self.navigate_named_location))
                elif self.navigate_option == 'navigate_exact':
                    smach.Concurrence.add('GO_TO_LOCATION', 
                                                navigation.Navigate_exact(robot,self.navigate_x,self.navigate_y,self.navigate_phi))
                elif self.navigate_option == 'navigate_goal_location':
                    smach.Concurrence.add('GO_TO_LOCATION', 
                                                navigation.Navigate_goal_location(robot, self.navigate_named_location))
                elif self.navigate_option == 'navigate_to_queryoutcome':
                    smach.Concurrence.add('GO_TO_LOCATION', 
                                                navigation.Navigate_to_queryoutcome(robot, self.nav_query, X="X", Y="Y", Phi="Phi"))
                
            smach.StateMachine.add('SUB_CONT_SAY_GO',
                                    cc)

class PointObject(smach.StateMachine):
    def __init__(self, robot, roi_query, object_query, object_identifier="Object"):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        self.robot = robot
        self.roi_query = roi_query
        self.object_query = object_query

        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "perception")

        with self:
            #import ipdb; ipdb.set_trace()
            smach.StateMachine.add("DRIVE_TO_SEARCHPOS",
                                    navigation.Visit_query_outcome_3d(self.robot, 
                                                                      self.roi_query, 
                                                                      x_offset=0.7, y_offset=0.0001,
                                                                      identifier=object_identifier),  #TODO Bas: when this is 0.0, amingo_inverse_reachability returns a 0,0,0,0,0,0,0 pose
                                    transitions={   'arrived':'SAY_LOOK_FOR_OBJECTS',
                                                    'unreachable':'Failed',
                                                    'preempted':'Aborted',
                                                    'goal_not_defined':'Failed',
                                                    'all_matches_tried':'Failed'})

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    human_interaction.Say(robot, ["Lets see if I can find the object."]),
                                    transitions={   'spoken':'LOOK'})

            smach.StateMachine.add('LOOK',
                                    perception.LookForObjectsAtROI(robot, self.roi_query, self.object_query),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'DRIVE_TO_SEARCHPOS',
                                                    'abort':'Aborted'})
                                                    
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    human_interaction.Say(robot, ["I have found what you are looking for and I will try to point at it!"]),
                                    transitions={ 'spoken':'POINT' })

            query_point = Conjunction(  Compound("current_object", "ObjectID"),
                                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
            smach.StateMachine.add('POINT',
                                    manipulation.PointMachine(robot.leftArm, robot, query_point),
                                    transitions={   'succeeded':'Done',
                                                    'failed':'SAY_FAILED_POINTING' }) 

            smach.StateMachine.add('SAY_FAILED_POINTING',
                                    human_interaction.Say(robot, ["Although I was not able to point at the object, you can find it in front of me!"]),
                                    transitions={ 'spoken':'Done' })

class Say_and_point_location(smach.StateMachine):
    def __init__(self, 
                 sentence, side, robot):
        smach.StateMachine.__init__(self, 
                                    outcomes=["succeeded"])
        self.robot = robot
        self.say_sentence = sentence
        self.side = side

        with self:
            cc = smach.Concurrence(outcomes = ['succeeded'],
                                   default_outcome = 'succeeded',
                                   outcome_map = {'succeeded':{'EXECUTE_SAY':'spoken',
                                                               'POINT_AT_LOCATION':'pointed'}})
                                                  
            with cc:
                smach.Concurrence.add("EXECUTE_SAY",
                                       human_interaction.Say(robot, self.say_sentence))
                smach.Concurrence.add("POINT_AT_LOCATION",
                                       manipulation.Point_location_hardcoded(robot, self.side))

            smach.StateMachine.add('SUB_CONT_SAY_POINT',
                                    cc)
