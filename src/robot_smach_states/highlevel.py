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
                                    transitions={   "initialized"   :"INSTRUCT_WAIT_FOR_DOOR",
                                                    "abort"         :"Aborted"})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                    human_interaction.Say(robot, [  "I will now wait until the door is opened", 
                                                                    "Knock knock, may I please come in?"]),
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
                                                    "query_true":"DOOR_OPEN",
                                                    "waiting":"DOOR_CLOSED",
                                                    "preempted":"Aborted"})

            # If the door is still closed after certain number of iterations, defined in Ask_query_true 
            # in perception.py, amigo will speak and check again if the door is open
            smach.StateMachine.add( "DOOR_CLOSED",
                                    human_interaction.Say(robot, "Door is closed, please open the door"),
                                    transitions={   "spoken":"ASSESS_DOOR"}) 

            smach.StateMachine.add( "DOOR_OPEN",
                                    human_interaction.Say(robot, "Door is open!"),
                                    transitions={   "spoken":"INIT_POSE"}) 

            # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set,
            # since it is thinks amigo is standing in front of a wall if door is closed and localization can(/will) be messed up.
            smach.StateMachine.add('INIT_POSE',
                                utility_states.Set_initial_pose(robot, initial_pose),
                                transitions={   'done':'ENTER_ROOM',
                                                'preempted':'Aborted',  # This transition will never happen at the moment.
                                                'error':'ENTER_ROOM'})  # It should never go to aborted.

            # Enter the arena with force drive as back-up
            smach.StateMachine.add('ENTER_ROOM',
                                    EnterArena(robot),
                                    transitions={   "done":"Done" })
            

# Enter the arena with force drive as back-up
class EnterArena(smach.StateMachine):

    class GotoEntryPoint(smach.State):
        def __init__(self, robot):
            smach.State.__init__(self, outcomes=["no_goal" , "found", "not_found", "all_unreachable"])
            self.robot = robot
            self.goto_query = Compound("waypoint", Compound("entry_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi"))

        def execute(self, userdata=None):
            # Move to the next waypoint in the storage room        
            reachable_goal_answers = self.robot.reasoner.query(
                                        Conjunction(
                                            Compound("waypoint", Compound("entry_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi")),
                                            Compound("not", Compound("unreachable", Compound("entry_point", "Waypoint")))))

            if not reachable_goal_answers:
                # check if there are any entry points (someone may have forgotten to specify them)            
                all_goal_answers = self.robot.reasoner.query(self.goto_query)
                if not all_goal_answers:
                    #self.robot.speech.speak("No-one has specified entry_point locations. Please do so in the locations file!")
                    rospy.loginfo("There are no entry points defined. Do so in the locations file!")
                    return "all_unreachable"
                else:
                    #self.robot.speech.speak("There are a couple of entry points, but they are all unreachable. Sorry.")
                    rospy.loginfo("Entry points are all unreachable.")

                    return "all_unreachable"

            # for now, take the first goal found
            goal_answer = reachable_goal_answers[0]

            goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
            waypoint_name = goal_answer["Waypoint"]

            nav = navigation.NavigateGeneric(self.robot, goal_pose_2d=goal)  #, goal_area_radius=0.5)
            nav_result = nav.execute()

            #import ipdb; ipdb.set_trace()

            if nav_result == "unreachable":  #Compound("entry_point", waypoint_name)
                self.robot.reasoner.query(Compound("assert", Compound("unreachable", Compound("entry_point", waypoint_name))))
                return "not_found"
            elif nav_result == "preempted":
                return "not_found"
            elif nav_result == "arrived":
                rospy.logdebug("AMIGO should be in the arena")
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
            #self.robot.speech.speak("As a back-up scenario I will now drive through the door with my eyes closed.", block=False)  # Amigo should not say that it uses force drive, looks stupid.
            rospy.loginfo("AMIGO uses force drive as a back-up scenario!")
            self.robot.base.force_drive(0.25, 0, 0, 6.0)    # x, y, z, time in seconds
            return "done"

    def __init__(self, robot):
        smach.StateMachine.__init__(self,outcomes=['done'])
        self.robot = robot

        with self:
            # If the door is open, amigo will say that it goes to the registration table
            smach.StateMachine.add( "THROUGH_DOOR",
                                    human_interaction.Say(robot, "I will start my task now", block=True),
                                    transitions={   "spoken":"ENTER_ROOM"}) 

            smach.StateMachine.add('ENTER_ROOM',
                                    self.GotoEntryPoint(robot),
                                    transitions={   "found":"done", 
                                                    "not_found":"ENTER_ROOM", 
                                                    "no_goal":"FORCE_DRIVE_THROUGH_DOOR",
                                                    "all_unreachable":"FORCE_DRIVE_THROUGH_DOOR"})

            smach.StateMachine.add('FORCE_DRIVE_THROUGH_DOOR',
                                    self.ForceDrive(robot),
                                    transitions={   "done":"done"})            


class GotoMeetingPoint(smach.State):
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

        self.robot.speech.speak("I'm coming to the meeting point!", block=False)

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
            #self.robot.speech.speak("I reached a meeting point", block=False)
            self.robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
            return "found"
        else: #goal not defined
            self.robot.speech.speak("I really don't know where to go, oops.")
            return "no_goal"


class GetObject(smach.StateMachine):
    def __init__(self, robot, side, roi_query, object_query, object_identifier="Object", max_duration=rospy.Duration(3600)):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed", "Timeout"])

        self.robot = robot
        self.side = side
        self.roi_query = roi_query
        self.object_query = object_query

        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "perception")

        with self:

            smach.StateMachine.add('SET_TIME_MARKER',
                                    utility_states.SetTimeMarker(robot, "get_object_start"),
                                    transitions={   'done':'DRIVE_TO_SEARCHPOS' })
 
            # smach.StateMachine.add('CHECK_OBJECT_QUERY',                                            
            #                         Check_object_found_before(robot, self.object_query),
            #                         transitions={   'object_found':'SAY_FOUND_SOMETHING_BEFORE',
            #                                         'no_object_found':'DRIVE_TO_SEARCHPOS' })
            #import ipdb; ipdb.set_trace()
            smach.StateMachine.add("DRIVE_TO_SEARCHPOS",
                                    navigation.Visit_query_outcome_3d(self.robot, 
                                                                      self.roi_query, 
                                                                      x_offset=0.7, y_offset=0.0,
                                                                      identifier=object_identifier),  #TODO Bas: when this is 0.0, amingo_inverse_reachability returns a 0,0,0,0,0,0,0 pose
                                    transitions={   'arrived':'SAY_LOOK_FOR_OBJECTS',
                                                    'unreachable':'DRIVE_TO_SEARCHPOS',
                                                    'preempted':'RESET_HEAD_AND_SPINDLE_UPON_ABORTED',
                                                    'goal_not_defined':'RESET_HEAD_AND_SPINDLE_UPON_FAILURE',      # End State
                                                    'all_matches_tried':'RESET_HEAD_AND_SPINDLE_UPON_FAILURE'})    # End State

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    human_interaction.Say(robot, ["Lets see what I can find here."],block=False),
                                    transitions={   'spoken':'LOOK'})

            lookatquery = Compound("current_poi","POI", Compound("point_3d","X","Y","Z"))

            smach.StateMachine.add('LOOK',
                                    perception.LookForObjectsAtROI(robot, lookatquery, self.object_query),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'RESET_HEAD_AND_SPINDLE',
                                                    'abort':'RESET_HEAD_AND_SPINDLE_UPON_ABORTED'})      # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'CHECK_TIME'})   # End State

            smach.StateMachine.add('CHECK_TIME',
                                    utility_states.CheckTime(robot, "get_object_start", max_duration),
                                    transitions={   'ok':'DRIVE_TO_SEARCHPOS',
                                                    'timeout':'RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT' })   # End State
                                                    
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    human_interaction.Say(robot, ["I have found something"],block=False),
                                    transitions={ 'spoken':'GRAB' })

            # smach.StateMachine.add('SAY_FOUND_SOMETHING_BEFORE',
            #                         human_interaction.Say(robot, ["I have seen the desired object before!"],block=False),
            #                         transitions={ 'spoken':'GRAB' })

            query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
            smach.StateMachine.add('GRAB',
                                    manipulation.GrabMachine(self.side, robot, query_grabpoint),
                                    transitions={   'succeeded':'RESET_HEAD_AND_SPINDLE_UPON_SUCCES',
                                                    'failed':'SAY_FAILED_GRABBING' })  

            smach.StateMachine.add('SAY_FAILED_GRABBING',
                                    human_interaction.Say(robot, ["Although I was not able to grab the object, you should be able to find it in \
                                                                   front of me! I will continue to grab another one for as long as I have the time."], block=False),
                                    transitions={ 'spoken':'MARK_DISPOSED' })

            #Mark the current_object as disposed
            @smach.cb_interface(outcomes=['done'])
            def deactivate_current_object(userdata):
                try:
                    #robot.speech.speak("I need some debugging in cleanup, please think with me here.")
                    #import ipdb; ipdb.set_trace()
                    objectID = robot.reasoner.query(Compound("current_object", "Disposed_ObjectID"))[0]["Disposed_ObjectID"]
                    robot.reasoner.query(Compound("assertz", Compound("disposed", objectID)))
                    rospy.loginfo("objectID = {0} is DISPOSED".format(objectID))

                    try:
                        robot.reasoner.detach_all_from_gripper("/amigo/grippoint_left")
                    except KeyError, ke:
                        rospy.loginfo("Could not detach object from gripper, do not know which ID: {0}".format(ke))
                    rospy.loginfo("object should be detached from gripper!")

                except:
                    pass #Just continue
                return 'done'
            smach.StateMachine.add('MARK_DISPOSED', smach.CBState(deactivate_current_object),
                                    transitions={'done':'CHECK_TIME_AFTER_FAILED_GRAB'}) 

            smach.StateMachine.add('CHECK_TIME_AFTER_FAILED_GRAB',
                                    utility_states.CheckTime(robot, "get_object_start", max_duration),
                                    transitions={   'ok':'DRIVE_TO_SEARCHPOS',
                                                    'timeout':'RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT' })   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_ABORTED',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Aborted'})   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_FAILURE',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Failed'})   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Timeout'})   # End State            

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_SUCCES',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Done'})   # End State

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
    def __init__(self, robot, side, roi_query, object_query, object_identifier="Object", max_duration=rospy.Duration(3600)):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed", "Timeout"])

        self.robot = robot
        self.side = side
        self.roi_query = roi_query
        self.object_query = object_query

        assert hasattr(robot, "base")
        assert hasattr(robot, "reasoner")
        assert hasattr(robot, "perception")

        with self:
            #import ipdb; ipdb.set_trace()
            smach.StateMachine.add('SET_TIME_MARKER',
                                    utility_states.SetTimeMarker(robot, "get_object_start"),
                                    transitions={   'done':'DRIVE_TO_SEARCHPOS' })

            # smach.StateMachine.add('CHECK_OBJECT_QUERY',                                            
            #                         Check_object_found_before(robot, self.object_query),
            #                         transitions={   'object_found':'SAY_FOUND_SOMETHING_BEFORE',
            #                                         'no_object_found':'RESET_HEAD_AND_SPINDLE_UPON_FAILURE' })

            smach.StateMachine.add("DRIVE_TO_SEARCHPOS",
                                    navigation.Visit_query_outcome_3d(self.robot, 
                                                                      self.roi_query, 
                                                                      x_offset=0.7, y_offset=0.0001,
                                                                      identifier=object_identifier),  #TODO Bas: when this is 0.0, amingo_inverse_reachability returns a 0,0,0,0,0,0,0 pose
                                    transitions={   'arrived':'SAY_LOOK_FOR_OBJECTS',
                                                    'unreachable':'DRIVE_TO_SEARCHPOS',
                                                    'preempted':'RESET_HEAD_AND_SPINDLE_UPON_ABORTED',
                                                    'goal_not_defined':'RESET_HEAD_AND_SPINDLE_UPON_FAILURE',
                                                    'all_matches_tried':'RESET_HEAD_AND_SPINDLE_UPON_FAILURE'})

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    human_interaction.Say(robot, ["Lets see if I can find the object."],block=False),
                                    transitions={   'spoken':'LOOK'})

            lookatquery = Compound("current_poi","POI", Compound("point_3d","X","Y","Z"))

            smach.StateMachine.add('LOOK',
                                    perception.LookForObjectsAtROI(robot, lookatquery, self.object_query),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'RESET_HEAD_AND_SPINDLE',
                                                    'abort':'RESET_HEAD_AND_SPINDLE_UPON_ABORTED'})

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'CHECK_TIME'})   # End State

            smach.StateMachine.add('CHECK_TIME',
                                    utility_states.CheckTime(robot, "get_object_start", max_duration),
                                    transitions={   'ok':'DRIVE_TO_SEARCHPOS',
                                                    'timeout':'RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT' })   # End State
                                                    
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    human_interaction.Say(robot, ["I have found what you are looking for and I will try to point at it!"],block=False),
                                    transitions={ 'spoken':'POINT' })

            # smach.StateMachine.add('SAY_FOUND_SOMETHING_BEFORE',
            #                         human_interaction.Say(robot, ["I have seen the desired object before!"],block=False),
            #                         transitions={ 'spoken':'POINT' })

            query_point = Conjunction(  Compound("current_object", "ObjectID"),
                                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

            smach.StateMachine.add('POINT',
                                    manipulation.PointMachine(self.side, robot, query_point),
                                    transitions={   'succeeded':'RESET_HEAD_AND_SPINDLE_UPON_SUCCES',
                                                    'failed':'SAY_FAILED_POINTING' }) 

            smach.StateMachine.add('SAY_FAILED_POINTING',
                                    human_interaction.Say(robot, ["Although I was not able to point at the object, you should be able to find it in \
                                                                   front of me! I will continue to find another one for as long as I have the time."], block=False),
                                    transitions={ 'spoken':'MARK_DISPOSED' })

            #Mark the current_object as disposed
            @smach.cb_interface(outcomes=['done'])
            def deactivate_current_object(userdata):
                try:
                    #robot.speech.speak("I need some debugging in cleanup, please think with me here.")
                    #import ipdb; ipdb.set_trace()
                    objectID = robot.reasoner.query(Compound("current_object", "Disposed_ObjectID"))[0]["Disposed_ObjectID"]
                    robot.reasoner.query(Compound("assertz", Compound("disposed", objectID)))
                    rospy.loginfo("objectID = {0} is DISPOSED".format(objectID))

                    try:
                        robot.reasoner.detach_all_from_gripper("/amigo/grippoint_left")
                    except KeyError, ke:
                        rospy.loginfo("Could not detach object from gripper, do not know which ID: {0}".format(ke))
                    rospy.loginfo("object should be detached from gripper!")

                except:
                    pass #Just continue
                return 'done'
            smach.StateMachine.add('MARK_DISPOSED', smach.CBState(deactivate_current_object),
                                    transitions={'done':'CHECK_TIME_AFTER_FAILED_POINT'}) 

            smach.StateMachine.add('CHECK_TIME_AFTER_FAILED_POINT',
                                    utility_states.CheckTime(robot, "get_object_start", max_duration),
                                    transitions={   'ok':'DRIVE_TO_SEARCHPOS',
                                                    'timeout':'RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT' })   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_ABORTED',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Aborted'})   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_FAILURE',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Failed'})   # End State

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_TIMEOUT',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Timeout'})   # End State            

            smach.StateMachine.add('RESET_HEAD_AND_SPINDLE_UPON_SUCCES',
                                    utility_states.ResetHeadSpindle(robot),
                                    transitions={   'done':'Done'})   # End State


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
