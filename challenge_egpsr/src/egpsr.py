#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_egpsr')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_smach_states.util.startup import startup

from speech_interpreter.srv import GetAction # for speech_to_text only

from psi import *

###########################
# Created by: Erik Geerts #
###########################

#######################
##### TODO LIST!! #####
## updated 15-4-2013 ##
#######################

# - Add all kinds of objects in the reasoner
# - While executing a task, failure handling is very weak. This should be optimized.

# - Placing objects on tables/etc at transport.


##########################################
############## What to run: ##############
############ updated 15-4-2013 ###########
##########################################
# - see README file

#############################################################
## Locations that must be defined in database on forehand: ##
##################### updated 15-4-2013 #####################
#############################################################
# - initial
# - meeting_point
# - exit_1
# - exit_2

############################
### Action possibilities ###
#### updated 15-4-2013 #####
############################

# See /challenge_egpsr/input_speech_not_used/sentences.corpus for available sentences to say during questioning.
# Available locations and objects can be found in /challenge_egpsr/input_speech_not_used/tue_test_lab/

# If speech files for tue_test_lab are used ONLY DRINKS AND BATHROOMSTUFF can be questioned at this point!

#################
### Questions ###
#################
# - See questions on top of every action in the statemachine
# - Is AMIGO easily able to put something down? I haven't seen him doing this. 
# - In case amigo is unable to say something, then class Say_and_Navigate will get into a deadlock. 
#   has this ever happened before (that amigo is not able to say something)?


class Ask_action(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["done", "no_action"])

        self.robot = robot
        self.rate = rate
        self.get_action_service = rospy.ServiceProxy('interpreter/get_action_user', GetAction)

    def execute(self, userdata):

        try:
            self.response = self.get_action_service(3600.0)  # = 1 hour because amigo has to be on standby to receive an action in e-gpsr

            self.robot.reasoner.query(Compound("assertz", Compound("goal", self.response.action, self.response.start_location, self.response.end_location, self.response.object, self.response.object_room, self.response.object_location)))
            
            if self.response.object == "no_answer" or self.response.object == "wrong_answer":
                return "no_action"
            ## Show values for action/start_location/end_location/object      
            #rospy.loginfo("action = {0}".format(self.response.action))
            #rospy.loginfo("start_location = {0}".format(self.response.start_location))
            #rospy.loginfo("end_location = {0}".format(self.response.end_location))
            #rospy.loginfo("object = {0}".format(self.response.object))
            
            return "done"

        except rospy.ServiceException, e:

            rospy.loginfo("No action is heared")
            #rospy.loginfo("Service call failed ({0})".format(e))

            return "no_action"


class Ask_action_without_mic(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "no_action"])

        self.robot = robot

    def execute(self, userdata):

        if self.robot.reasoner.query(Compound("goal","A","B","C","D","E","F")):

            return "done"

        else: 
            rospy.sleep(2)
            return "no_action"


class Query_specific_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["action_get", "action_transport","action_point","action_find","action_leave","error"])

        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        raw_goals = self.robot.reasoner.query(Compound("goal", "Action","Loc_from","Loc_to","Object_action","Object_room","Object_location"))
        
        goals = [(answer["Action"], answer["Loc_from"], answer["Loc_to"], answer["Object_action"],answer["Object_room"],answer["Object_location"]) for answer in raw_goals]       
        action,loc_from,loc_to,object_action,object_room,object_location = min(goals)

        action = action.get_string()
        loc_from = loc_from.get_string()
        loc_to = loc_to.get_string()
        object_action = object_action.get_string()
        object_room = object_room.get_string()
        object_location = object_location.get_string()

        if action == 'get':
            return "action_get"
        elif action == 'transport':
            return "action_transport"
        elif action == 'point':
            return "action_point"
        elif action == 'find':
            return "action_find"
        elif action == 'leave':
            return "action_leave"
        else:
            return "error"


class Navigate_to_queryoutcome_point_location(states.Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Z="Z"):
        states.Navigate_abstract.__init__(self, robot)

        assert isinstance(query, Term)

        self.queryTerm = query
        self.X, self.Y, self.Z = X, Y, Z
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """     

        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
            return None
        else:
            chosen_answer = answers[0]
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Z])) for answer in answers]

            x,y,z = possible_locations[0]
            
            goal = possible_locations[0]
            rospy.loginfo("goal = {0}".format(goal))
            roi_name = chosen_answer["ROI_Location"]

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,z)))
            
            self.robot.reasoner.query(Compound("assert", Compound("point_roi_tried", roi_name))) 
            
            look_point = geometry_msgs.msg.PointStamped()
            look_point.point = self.robot.base.point(x,y)
            pose = states.util.msg_constructors.Quaternion(z=1.0)

            base_pose_for_point = self.robot.base.get_base_pose(look_point, 0.7, 0.0001)
            if base_pose_for_point.pose.position.x == 0 and base_pose_for_point.pose.position.y == 0:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.

            return base_pose_for_point.pose.position, base_pose_for_point.pose.orientation


class Finished_goal(smach.State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=["new_task", "tasks_completed"])

        self.robot = robot

    def execute(self, userdata):

        if not self.robot.reasoner.query(Compound("tasks_done", "X")):
            
            self.robot.reasoner.query(Compound("assertz", Compound("tasks_done", "1.0")))
            new_tasks_done = 1.0

        else:
            raw_tasks_done = self.robot.reasoner.query(Compound("tasks_done","X"))
            tasks_done = [(answer["X"]) for answer in raw_tasks_done]
            
            new_tasks_done = 1.0 + tasks_done[0].get_number()

            # Update status tasks_done in reasoner
            self.robot.reasoner.query(Compound("retract", Compound("tasks_done", tasks_done[0])))
            self.robot.reasoner.query(Compound("assertz", Compound("tasks_done", new_tasks_done)))

        raw_goals = self.robot.reasoner.query(Compound("goal", "Action","Loc_from","Loc_to","Object_action","Object_room","Object_location"))
        
        goals = [(answer["Action"], answer["Loc_from"], answer["Loc_to"], answer["Object_action"],answer["Object_room"],answer["Object_location"]) for answer in raw_goals]       
        action,loc_from,loc_to,object_action,object_room,object_location = min(goals)

        action = action.get_string()
        loc_from = loc_from.get_string()
        loc_to = loc_to.get_string()
        object_action = object_action.get_string()
        object_room = object_room.get_string()
        object_location = object_location.get_string()

        self.robot.reasoner.query(Compound("assertz", Compound("goal_done", new_tasks_done,action,loc_from,loc_to,object_action,object_room,object_location)))
        self.robot.reasoner.query(Compound("retract", Compound("goal", action,loc_from,loc_to,object_action,object_room,object_location)))


        # Check if tasks done equals maximum number of tasks that needs to be done
        raw_tasks_max = self.robot.reasoner.query(Compound("tasks_max", "X"))
        tasks_max = [(answer["X"]) for answer in raw_tasks_max]
        
        new_tasks_max = tasks_max[0].get_number()

        #rospy.loginfo("Maximum number of tasks" = {0}".format(new_tasks_max))           
        #rospy.loginfo("Number of tasks finished succesfully = {0}".format(new_tasks_done))

        if new_tasks_max == new_tasks_done:
            return "tasks_completed"
        else:
            return "new_task"


class Failed_goal(smach.State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=["new_task"])

        self.robot = robot

    def execute(self, userdata):

        if not self.robot.reasoner.query(Compound("tasks_failed", "X")):
            
            self.robot.reasoner.query(Compound("assertz", Compound("tasks_failed", "1.0")))
            new_tasks_failed = 1.0

        else:
            raw_tasks_failed = self.robot.reasoner.query(Compound("tasks_failed","X"))
            tasks_failed = [(answer["X"]) for answer in raw_tasks_failed]
            
            new_tasks_failed = 1.0 + tasks_failed[0].get_number()

            # Update status tasks_done in reasoner
            self.robot.reasoner.query(Compound("retract", Compound("tasks_failed", tasks_failed[0])))
            self.robot.reasoner.query(Compound("assertz", Compound("tasks_failed", new_tasks_failed)))

        raw_goals = self.robot.reasoner.query(Compound("goal", "Action","Loc_from","Loc_to","Object_action","Object_room","Object_location"))
        
        goals = [(answer["Action"], answer["Loc_from"], answer["Loc_to"], answer["Object_action"],answer["Object_room"],answer["Object_location"]) for answer in raw_goals]       
        action,loc_from,loc_to,object_action,object_room,object_location = min(goals)

        action = action.get_string()
        loc_from = loc_from.get_string()
        loc_to = loc_to.get_string()
        object_action = object_action.get_string()
        object_room = object_room.get_string()
        object_location = object_location.get_string()

        self.robot.reasoner.query(Compound("assertz", Compound("goal_failed", new_tasks_failed,action,loc_from,loc_to,object_action,object_room,object_location)))
        self.robot.reasoner.query(Compound("retract", Compound("goal", action,loc_from,loc_to,object_action,object_room,object_location)))

        #rospy.loginfo("Number of failed tasks = {0}".format(new_tasks_failed))           

        return "new_task"


class Navigate_to_queryoutcome_waypoint_location1(states.Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi"):
        states.Navigate_abstract.__init__(self, robot)

        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = possible_locations[0]
            
            goal = possible_locations[0]
            rospy.loginfo("goal = {0}".format(goal))

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)

#### USED FOR NAV TO MEETING_POINT2_BACKUP
class Navigate_to_queryoutcome_waypoint_location2(states.Navigate_abstract):  
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi"):
        states.Navigate_abstract.__init__(self, robot)


        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = possible_locations[1]
            
            goal = possible_locations[1]
            rospy.loginfo("goal = {0}".format(goal))

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)

#### USED FOR NAV TO MEETING_POINT3_BACKUP
class Navigate_to_queryoutcome_waypoint_location3(states.Navigate_abstract):  
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi"):
        states.Navigate_abstract.__init__(self, robot)

        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = possible_locations[2]
            
            goal = possible_locations[2]
            rospy.loginfo("goal = {0}".format(goal))

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)


# It is important for the EGPSR to get back to the meeting point!! Otherwise restart, therefore understanding solution.
class GotoMeetingPointRobustEGPSR(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "not_at_loc"])


        with self:
            navigate_meeting_point_1 = Conjunction(  Compound("=", "Waypoint", Compound("initial_egpsr_1", "a")),
                                                     Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

            smach.StateMachine.add('GO_TO_MEETING_POINT_EGPSR_1', 
                                    Navigate_to_queryoutcome_waypoint_location1(robot, navigate_meeting_point_1, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'succeeded', 
                                                    'preempted':'FAILED_FIRST_ATTEMPT', 
                                                    'unreachable':'FAILED_FIRST_ATTEMPT', 
                                                    'goal_not_defined':'FAILED_FIRST_ATTEMPT'})

            smach.StateMachine.add("FAILED_FIRST_ATTEMPT",
                                    states.Say(robot, ["I was not able to reach the meeting point at first attempt, I will try it again."]),
                                    transitions={   "spoken":"GO_TO_MEETING_POINT_EGPSR_2"})

            navigate_meeting_point_2 = Conjunction(  Compound("=", "Waypoint", Compound("initial_egpsr_1", "a")),
                                                     Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

            smach.StateMachine.add('GO_TO_MEETING_POINT_EGPSR_2', 
                                    Navigate_to_queryoutcome_waypoint_location2(robot, navigate_meeting_point_2, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'succeeded', 
                                                    'preempted':'FAILED_SECOND_ATTEMPT', 
                                                    'unreachable':'FAILED_SECOND_ATTEMPT', 
                                                    'goal_not_defined':'FAILED_SECOND_ATTEMPT'})

            smach.StateMachine.add("FAILED_SECOND_ATTEMPT",
                                    states.Say(robot, [  "Also my second attempt was not succesful. One last try."]),
                                    transitions={   "spoken":"GO_TO_MEETING_POINT_EGPSR_3"})

            navigate_meeting_point_3 = Conjunction(  Compound("=", "Waypoint", Compound("initial_egpsr_1", "a")),
                                                     Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

            smach.StateMachine.add('GO_TO_MEETING_POINT_EGPSR_3', 
                                    Navigate_to_queryoutcome_waypoint_location3(robot, navigate_meeting_point_3, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'succeeded', 
                                                    'preempted':'not_at_loc', 
                                                    'unreachable':'not_at_loc', 
                                                    'goal_not_defined':'not_at_loc'})


########################
##### STATEMACHINE #####
########################


def setup_statemachine(robot):

    #retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("goal", "U","V","W", "X", "Y", "Z")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))
    robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
    robot.reasoner.query(Compound("retractall", Compound("point_roi_tried", "X")))   

    robot.reasoner.query(Compound("retractall", Compound("tasks_done", "X")))
    robot.reasoner.query(Compound("retractall", Compound("tasks_max", "X")))
    robot.reasoner.query(Compound("retractall", Compound("tasks_failed", "X")))
    
    #Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/egpsr.pl'))

    #Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "egpsr")))
    robot.reasoner.query(Compound("assertz",Compound("tasks_done", "0.0")))
    robot.reasoner.query(Compound("assertz",Compound("tasks_max", "10.0")))  # Define how many tasks you want to perform

    #Assert not_visited locations
    robot.reasoner.query(Compound("init_not_roi","Location"))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        # DURING A CHALLENGE, AMIGO STARTS AT A DESIGNATED POSITION, NOT IN FRONT OF A DOOR

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRODUCE_SHORT',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})


        ######################################################
        #################### INSTRUCTIONS ####################             
        ######################################################


        smach.StateMachine.add("INTRODUCE_SHORT",
                               states.Say(robot,"Hi, my name is Amigo. I will just wait here and wonder if I can do something for you"),
                               transitions={'spoken':'INIT_POSE'})

        smach.StateMachine.add('INIT_POSE',
                                states.Set_initial_pose(robot, 'initial_egpsr_1'),
                                transitions={   'done':'ASK_ACTION',
                                                'preempted':'ASK_ACTION',
                                                'error':'ASK_ACTION'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'RETRACT_POINT_ROI',
                                             'no_action':'ASK_ACTION'})

        smach.StateMachine.add("GIVE_ACTION_WITHOUT_MIC",
                                Ask_action_without_mic(robot),
                                transitions={'done':'RETRACT_POINT_ROI',
                                             'no_action':'GIVE_ACTION_WITHOUT_MIC'})

        smach.StateMachine.add("RETRACT_POINT_ROI",
                                states.Retract_facts(robot, [Compound("point_roi_tried", "X")]),
                                transitions={'retracted':'QUERY_SPECIFIC_ACTION'})


        ######################################################
        ################### EXECUTE ACTION ###################
        ######################################################

        smach.StateMachine.add("QUERY_SPECIFIC_ACTION",
                                Query_specific_action(robot),
                                transitions={   'action_get':'SUB_SM_GET',
                                                'action_transport':'SUB_SM_TRANSPORT',
                                                'action_point':'SUB_SM_POINT',
                                                'action_find':'SUB_SM_FIND',
                                                'action_leave':'SUB_SM_LEAVE',
                                                'error':'FINISHED_TASK'})

        #################### EXECUTE GET #####################

        ## QUESTION: Get is now defined as getting something to the questioner at the meeting point. 
        ##           Is this allways the case? (check things possible to say)

        sm_get = smach.StateMachine(outcomes=['Done','Failed'])

        with sm_get:

            smach.StateMachine.add('GO_TO_GETOBJECT',
                                   states.Say(robot,"I will get it right away!"),
                                   transitions={'spoken':'GET_OBJECT'})

            search_query            = Compound("search_query","ROI_Location",Compound("point_3d","X","Y","Z"))
            object_identifier_query = "ROI_Location"
            object_query            = Conjunction( 
                                         Compound("object_query","ObjectID", Sequence("X","Y","Z")),
                                         Compound("not", Compound("disposed", "ObjectID")))
            
            smach.StateMachine.add('GET_OBJECT',
                                    states.GetObject(robot, search_query, object_query, object_identifier=object_identifier_query, max_duration=rospy.Duration(180)), 
                                    transitions={'Done':'SAY_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Failed':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Aborted':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Timeout':'GET_OBJECT_TIMED_OUT' })

            # smach.StateMachine.add('GET_OBJECT_TIMED_OUT',
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I am sorry, I have to return to the meeting point to be back in time.",                               
            #                             input_keys=['navigate_named', 'initial_egpsr_1']),         
            #                             transitions={'succeeded':'FAILED_AT_MEETING_POINT',
            #                                          'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})


            smach.StateMachine.add("GET_OBJECT_TIMED_OUT",
                                   states.Say(robot,"I am sorry, I have to return to the meeting point to be back in time."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_10'}) 

            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_10',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'FAILED_AT_MEETING_POINT',
                                                     'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})


            # smach.StateMachine.add('SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',                               
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I failed picking up the object. I am sorry, but I will go back to the meeting point and \
            #                                         ask for another task",                               
            #                             input_keys=['navigate_named','initial_egpsr_1']),         
            #                             transitions={'succeeded':'FAILED_AT_MEETING_POINT',
            #                                          'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"I failed picking up the object. I am sorry, but I will go back to the meeting point and ask for another task."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_11'}) 

            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_11',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'FAILED_AT_MEETING_POINT',
                                                     'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("FAILED_AT_MEETING_POINT",
                                   states.Say(robot,"I am at the meeting point again."),
                                   transitions={'spoken':'Failed'})

            smach.StateMachine.add("FAILED_NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I could not reach the meeting point the way I wanted. I am sorry."),
                                   transitions={'spoken':'Failed'})

            # smach.StateMachine.add('SAY_AT_GOAL_NAVIGATE_TO_LOC_TO', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "Since I have the object in my hands, I will go to the meeting point and handover the object.",
            #                             input_keys=['navigate_goal_location','loc_to']),                
            #                             transitions={'succeeded':'AT_LOC_TO',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("SAY_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"Since I have the object in my hands, I will go to the meeting point and handover the object."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_12'}) 

            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_12',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'AT_LOC_TO',
                                                     'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("AT_LOC_TO",
                                   states.Say(robot,"I am at the meeting point again, I will open my gripper now so that you are \
                                                     able to get it out of my hands."),
                                   transitions={'spoken':'DROP_OBJECT'})

            smach.StateMachine.add("NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I could not reach the meeting point the way I wanted. I am sorry. But could someone take the object out of my hands, I will open my gripper now."),
                                   transitions={'spoken':'DROP_OBJECT'})

            smach.StateMachine.add( 'DROP_OBJECT', states.SetGripper(robot, robot.leftArm, gripperstate=0),         #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed'   :'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP', states.SetGripper(robot, robot.leftArm, gripperstate=1),    #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed'   :'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    states.ArmToPose(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'MARK_DISPOSED',
                                                    'failed':'MARK_DISPOSED'})

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
                        robot.reasoner.detach_all_from_gripper("/grippoint_left")
                    except KeyError, ke:
                        rospy.loginfo("Could not detach object from gripper, do not know which ID: {0}".format(ke))
                    rospy.loginfo("object should be detached from gripper!")

                except:
                    pass #Just continue
                return 'done'
            smach.StateMachine.add('MARK_DISPOSED', smach.CBState(deactivate_current_object),
                                    transitions={'done':'Done'})

        smach.StateMachine.add("SUB_SM_GET",
                                sm_get,
                                transitions={'Done':'FINISHED_TASK',
                                             'Failed':'FAILED_TASK'})

        ################## EXECUTE TRANSPORT #################

        ## QUESTION: Transporting something: will the end location never be the meeting point? 
        ##           If this could be the case, then there should be a check whether loc_to is the meeting point or somewhere else.
        ##           If it is somewhere else, it should go to the meeting point in the end.
    
        sm_transport = smach.StateMachine(outcomes=['Done','Failed'])

        with sm_transport:

            smach.StateMachine.add('GO_TO_GETOBJECT',
                                   states.Say(robot,"I will get it right away!"),
                                   transitions={'spoken':'GET_OBJECT'})

            search_query            = Compound("search_query","ROI_Location",Compound("point_3d","X","Y","Z"))
            object_identifier_query = "ROI_Location"
            object_query            = Conjunction( 
                                         Compound("object_query","ObjectID", Sequence("X","Y","Z")),
                                         Compound("not", Compound("disposed", "ObjectID")))     

            #object_query            = Compound("object_query","ObjectID", Sequence("X","Y","Z")) 

            smach.StateMachine.add('GET_OBJECT',
                                    states.GetObject(robot, search_query, object_query, object_identifier=object_identifier_query), 
                                    transitions={'Done':'SAY_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Failed':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Aborted':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Timeout':'GET_OBJECT_TIMED_OUT' })

            # smach.StateMachine.add('GET_OBJECT_TIMED_OUT',
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I am sorry, I have to return to the meeting point to be back in time.",                               
            #                             input_keys=['navigate_named', 'initial_egpsr_1']),         
            #                             transitions={'succeeded':'FAILED_AT_MEETING_POINT',
            #                                          'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("GET_OBJECT_TIMED_OUT",
                                   states.Say(robot,"I am sorry, I have to return to the meeting point to be back in time."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_21'}) 

            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_21',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'FAILED_AT_MEETING_POINT',
                                                     'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})


            # In case goal object couldt be reached, return directly. In class Finished_goal it thinks that it has \
            # completed the goal correctly. So something should be implemented here that the goal is not finished.

            smach.StateMachine.add("SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"I failed picking up the object. I am sorry, but I will go back to the meeting point and ask for another task."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_22'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_22',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'FAILED_AT_MEETING_POINT',
                                                     'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            # smach.StateMachine.add('SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I failed picking up the object. I am sorry, but I will go back to the \
            #                                         meeting point and ask for another task",
            #                             input_keys=['navigate_named','initial_egpsr_1']),         
            #                             transitions={'succeeded':'FAILED_AT_MEETING_POINT',
            #                                          'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("FAILED_AT_MEETING_POINT",
                                   states.Say(robot,"I am at the meeting point again."),
                                   transitions={'spoken':'Failed'})

            smach.StateMachine.add("FAILED_NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I could not reach the meeting point the way I wanted. I am sorry."),
                                   transitions={'spoken':'Failed'})
            # # At goal object
            # smach.StateMachine.add('SAY_AT_GOAL_NAVIGATE_TO_LOC_TO', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "Since I have the object in my hands, I will go to the drop off location \
            #                                         and handover the object.",
            #                             input_keys=['navigate_goal_location','loc_to']),
            #                             transitions={'succeeded':'AT_LOC_TO',
            #                                          'not_at_loc':'SAY_NOT_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT'})

            query_point = Conjunction(  Compound("point_location","ROI_Location", Compound("point_3d", "X", "Y", "Z")),
                                        Compound("not",Compound("point_roi_tried","ROI_Location")))


            smach.StateMachine.add("SAY_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"Since I have the object in my hands, I will go to the drop off location \
                                                    and handover the object."),
                                   transitions={'spoken':'AT_GOAL_NAVIGATE_TO_LOC_TO_100'})

            smach.StateMachine.add('AT_GOAL_NAVIGATE_TO_LOC_TO_100', 
                                    Navigate_to_queryoutcome_point_location(robot, query_point, X="X", Y="Y", Z="Z"),
                                    transitions={   'arrived':'AT_LOC_TO', 
                                                    'preempted':'AT_GOAL_NAVIGATE_TO_LOC_TO_100', 
                                                    'unreachable':'AT_GOAL_NAVIGATE_TO_LOC_TO_100', 
                                                    'goal_not_defined':'SAY_NOT_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT'})

            # In case drop off succeeded:

            smach.StateMachine.add("AT_LOC_TO",
                                   states.Say(robot,"I am at the drop off location, I will open my gripper now so that you are \
                                                     able to get it out of my hands."),
                                   transitions={'spoken':'DROP_OBJECT'})


            smach.StateMachine.add( 'DROP_OBJECT', states.SetGripper(robot, robot.leftArm, gripperstate=0),         #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed'   :'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP', states.SetGripper(robot, robot.leftArm, gripperstate=1),    #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed'   :'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    states.ArmToPose(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),  #Copied from demo_executioner NORMAL
                                    transitions={   'done':'SAY_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT',
                                                    'failed':'SAY_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT'})

            # smach.StateMachine.add('SAY_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "You should have the object now. Therefore I will drive back to the meeting point.",
            #                             input_keys=['navigate_named','initial_egpsr_1']),
            #                             transitions={'succeeded':'AT_MEETING_POINT',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("SAY_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT",
                                   states.Say(robot,"You should have the object now. Therefore I will drive back to the meeting point."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_23'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_23',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'AT_MEETING_POINT',
                                                     'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("AT_MEETING_POINT",
                                   states.Say(robot,"I am at the meeting point again."),
                                   transitions={'spoken':'MARK_DISPOSED'})

            smach.StateMachine.add("NOT_AT_MEETING_POINT",
                                   states.Say(robot,"Unfortunately I was not able to reach the meeting point the way I wanted. I am sorry."),
                                   transitions={'spoken':'MARK_DISPOSED'})

            # In case drop off didn't succeed, AMIGO will try to deliver it at the meeting point:

            # smach.StateMachine.add('SAY_NOT_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I could not reach the drop off location. So I will drive back to the \
            #                                         meeting point and try to deliver the object there. I am sorry!",
            #                             input_keys=['navigate_named','initial_egpsr_1']),
            #                             transitions={'succeeded':'AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED'})



            # smach.StateMachine.add('SAY_NOT_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I could not reach the drop off location. So I will drive back to the \
            #                                         meeting point and try to deliver the object there. I am sorry!",
            #                             input_keys=['navigate_named','initial_egpsr_1']),
            #                             transitions={'succeeded':'AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED'})


            smach.StateMachine.add("SAY_NOT_AT_LOC_TO_NAVIGATE_TO_MEETING_POINT",
                                   states.Say(robot,"I could not reach the drop off location. So I will drive back to the \
                                                    meeting point and try to deliver the object there. I am sorry!"),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_25'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_25',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED',
                                                     'not_at_loc':'NOT_AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED'})


            smach.StateMachine.add("AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED",
                                   states.Say(robot,"I am at the meeting point, I will open my gripper now so that you are \
                                                     able to get it out of my hands."),
                                   transitions={'spoken':'DROP_OBJECT_FAILURE'})

            smach.StateMachine.add("NOT_AT_MEETING_POINT_AND_PACKAGE_NOT_DELIVERED",
                                   states.Say(robot,"Unfortunately I was not able to reach the meeting point the way I wanted. I am sorry. Therefore I will place the object back on its place."),
                                   transitions={'spoken':'DROP_OBJECT_FAILURE'})

            smach.StateMachine.add( 'DROP_OBJECT_FAILURE', states.SetGripper(robot, robot.leftArm, gripperstate=0),    #open
                                    transitions={   'succeeded':'CLOSE_AFTER_FAILURE',
                                                    'failed'   :'CLOSE_AFTER_FAILURE'})
            smach.StateMachine.add( 'CLOSE_AFTER_FAILURE', states.SetGripper(robot, robot.leftArm, gripperstate=1),    #close
                                    transitions={   'succeeded':'RESET_ARM_FAILURE',
                                                    'failed'   :'RESET_ARM_FAILURE'})
            smach.StateMachine.add('RESET_ARM_FAILURE', 
                                    states.ArmToPose(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),  #Copied from demo_executioner NORMAL
                                    transitions={   'done':'MARK_DISPOSED',
                                                    'failed':'MARK_DISPOSED'})

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
                        robot.reasoner.detach_all_from_gripper("/grippoint_left")
                    except KeyError, ke:
                        rospy.loginfo("Could not detach object from gripper, do not know which ID: {0}".format(ke))
                    rospy.loginfo("object should be detached from gripper!")

                except:
                    pass #Just continue
                return 'done'
            smach.StateMachine.add('MARK_DISPOSED', smach.CBState(deactivate_current_object),
                                    transitions={'done':'Done'})

        smach.StateMachine.add("SUB_SM_TRANSPORT", 
                                sm_transport,
                                transitions={'Done':'FINISHED_TASK',
                                             'Failed':'FAILED_TASK'})


        #################### EXECUTE POINT ###################
    

        sm_point= smach.StateMachine(outcomes=['Done','Failed'])

        with sm_point:

            smach.StateMachine.add('SAY_GO_TO_POINT_LOCATION',
                                    states.Say(robot,"I will try to find the location!"),
                                    transitions={'spoken':'GO_TO_POINT_LOCATION'})


            query_point = Conjunction(  Compound("point_location","ROI_Location", Compound("point_3d", "X", "Y", "Z")),
                                        Compound("not",Compound("point_roi_tried","ROI_Location")))

            object_identifier_query = "ROI_Location"

            # smach.StateMachine.add("GO_TO_POINT_LOCATION",
            #                         states.Visit_query_outcome_3d(robot, 
            #                                                           query_point, 
            #                                                           x_offset=0.7, y_offset=0.0001,
            #                                                           identifier=object_identifier_query),  #TODO Bas: when this is 0.0, amingo_inverse_reachability returns a 0,0,0,0,0,0,0 pose
            #                         transitions={   'arrived':'SAY_AND_POINT_LOCATION',
            #                                         'unreachable':'FAILED_DRIVING_TO_LOCATION',
            #                                         'preempted':'FAILED_DRIVING_TO_LOCATION',
            #                                         'goal_not_defined':'FAILED_DRIVING_TO_LOCATION',
            #                                         'all_matches_tried':'FAILED_DRIVING_TO_LOCATION'})


            smach.StateMachine.add('GO_TO_POINT_LOCATION', 
                                    Navigate_to_queryoutcome_point_location(robot, query_point, X="X", Y="Y", Z="Z"),
                                    transitions={   'arrived':'SAY_AND_POINT_LOCATION', 
                                                    'preempted':'GO_TO_POINT_LOCATION', 
                                                    'unreachable':'GO_TO_POINT_LOCATION', 
                                                    'goal_not_defined':'FAILED_DRIVING_TO_LOCATION'})

            smach.StateMachine.add("FAILED_DRIVING_TO_LOCATION",
                                    states.Say(robot,"I am sorry but I was not able to drive to the desired location!"),
                                    transitions={'spoken':'Failed'})

            smach.StateMachine.add('SAY_AND_POINT_LOCATION', 
                                    states.Say_and_point_location(
                                        robot=robot,
                                        sentence = "I will try to point at the desired location with my left hand",
                                        side = robot.leftArm),
                                        transitions={'succeeded':'SAY_AT_GOAL_NAVIGATE_TO_LOC_TO'})
                                                     
            # smach.StateMachine.add('SAY_AT_GOAL_NAVIGATE_TO_LOC_TO', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "Am I right? I will go back to the meeting point now.",
            #                             input_keys=['navigate_named','initial_egpsr_1']),               
            #                             transitions={'succeeded':'AT_LOC_TO',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("SAY_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"Am I right? I will go back to the meeting point now."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_31'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_31',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'AT_LOC_TO',
                                                     'not_at_loc':'NOT_AT_MEETING_POINT'})


            smach.StateMachine.add("AT_LOC_TO",
                                   states.Say(robot,"I'm at the meeting point again."),
                                   transitions={'spoken':'Done'})
                
            smach.StateMachine.add("NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I couldn't reach the meeting point the way I wanted. I'm sorry."),
                                   transitions={'spoken':'Done'})


        smach.StateMachine.add("SUB_SM_POINT",
                                sm_point,
                                transitions={'Done':'FINISHED_TASK',
                                             'Failed':'FAILED_TASK'})

        #################### EXECUTE FIND ####################
       
        sm_find= smach.StateMachine(outcomes=['Done','Failed'])

        with sm_find:

            smach.StateMachine.add('GO_TO_POINT_OBJECT',
                                   states.Say(robot,"I will find it right away!"),
                                   transitions={'spoken':'POINT_OBJECT'})

            search_query            = Compound("search_query","ROI_Location",Compound("point_3d","X","Y","Z"))
            object_identifier_query = "ROI_Location"
            object_query            = Conjunction( 
                                         Compound("object_query","ObjectID", Sequence("X","Y","Z")),
                                         Compound("not", Compound("disposed", "ObjectID")))     

            smach.StateMachine.add('POINT_OBJECT',
                                    states.PointObject(robot, search_query, object_query, object_identifier=object_identifier_query),  
                                    transitions={'Done':'SAY_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Failed':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',
                                                 'Aborted':'SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO'})

            # smach.StateMachine.add('SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO',                              
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I failed finding the object. I am sorry, but I will go back to the meeting point and \
            #                                         ask for another task",                               
            #                             input_keys=['navigate_named','initial_egpsr_1']),         
            #                             transitions={'succeeded':'FAILED_AT_MEETING_POINT',
            #                                          'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("SAY_NOT_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"I failed picking up the object. I am sorry, but I will go back to the meeting point and ask for another task."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_41'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_41',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'FAILED_AT_MEETING_POINT',
                                                     'not_at_loc':'FAILED_NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("FAILED_AT_MEETING_POINT",
                                   states.Say(robot,"I am at the meeting point again."),
                                   transitions={'spoken':'Failed'})

            smach.StateMachine.add("FAILED_NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I could not reach the meeting point the way I wanted. I am sorry."),
                                   transitions={'spoken':'Failed'})

            # smach.StateMachine.add('SAY_AT_GOAL_NAVIGATE_TO_LOC_TO', 
            #                         states.Say_and_Navigate(
            #                             robot=robot,
            #                             sentence = "I will go back to the meeting point.",
            #                             input_keys=['navigate_goal_location','loc_to']),                
            #                             transitions={'succeeded':'AT_LOC_TO',
            #                                          'not_at_loc':'NOT_AT_MEETING_POINT'})


            smach.StateMachine.add("SAY_AT_GOAL_NAVIGATE_TO_LOC_TO",
                                   states.Say(robot,"I will go back to the meeting point."),
                                   transitions={'spoken':'NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_42'}) 


            smach.StateMachine.add('NOT_AT_GOAL_NAVIGATE_TO_LOC_TO_42',                               
                                    GotoMeetingPointRobustEGPSR(robot),
                                        transitions={'succeeded':'AT_LOC_TO',
                                                     'not_at_loc':'NOT_AT_MEETING_POINT'})

            smach.StateMachine.add("AT_LOC_TO",
                                   states.Say(robot,"I am at the meeting point again."),
                                   transitions={'spoken':'Done'})
                
            smach.StateMachine.add("NOT_AT_MEETING_POINT",
                                   states.Say(robot,"I could not reach the meeting point the way I wanted. I am sorry."),
                                   transitions={'spoken':'Done'})

        smach.StateMachine.add("SUB_SM_FIND",
                                sm_find,
                                transitions={'Done':'FINISHED_TASK',                
                                             'Failed':'FAILED_TASK'})

        #################### EXECUTE LEAVE ###################
        
        sm_leave = smach.StateMachine(outcomes=['Done','Failed'])

        with sm_leave:

            smach.StateMachine.add('GO_TO_EXIT_1', 
                                    states.Navigate_named(robot, "exit_1"),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'GO_TO_EXIT_2', 
                                                    'unreachable':'GO_TO_EXIT_2', 
                                                    'goal_not_defined':'GO_TO_EXIT_2'})

            smach.StateMachine.add('GO_TO_EXIT_2', 
                                    states.Navigate_named(robot, "exit_2"),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'GO_TO_EXIT_3', 
                                                    'unreachable':'GO_TO_EXIT_3', 
                                                    'goal_not_defined':'GO_TO_EXIT_3'})

            smach.StateMachine.add('GO_TO_EXIT_3', 
                                    states.Navigate_named(robot, "exit_3"),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'NOT_AT_EXIT', 
                                                    'unreachable':'NOT_AT_EXIT', 
                                                    'goal_not_defined':'NOT_AT_EXIT'})

            smach.StateMachine.add("AT_EXIT",
                                   states.Say(robot,"Goodbye!"),
                                   transitions={'spoken':'Done'})
                
            smach.StateMachine.add("NOT_AT_EXIT",
                                   states.Say(robot,"I was not able to reach the exit, I am sorry."),
                                   transitions={'spoken':'Failed'})

        smach.StateMachine.add("SUB_SM_LEAVE",
                                sm_leave,
                                transitions={'Done':'FINISH',                
                                             'Failed':'FINISH'})

        ######################################################
        ########### CHECK NUMBER OF TASKS COMPLETED ##########
        ######################################################

        ## maybe a reset head and reset spindle / arms over here?


        ## In case goal is given via speech interpreter:
        smach.StateMachine.add("FAILED_TASK",
                                Failed_goal(robot),
                                transitions={'new_task':'ASK_ACTION'})


        smach.StateMachine.add("FINISHED_TASK",
                                Finished_goal(robot),
                                transitions={'new_task':'ASK_ACTION',
                                              'tasks_completed':'FINISH'})


        # # In case goal is given via amigo-console:
        # smach.StateMachine.add("FAILED_TASK",
        #                         Failed_goal(robot),
        #                         transitions={'new_task':'GIVE_ACTION_WITHOUT_MIC'})


        # smach.StateMachine.add("FINISHED_TASK",
        #                         Finished_goal(robot),
        #                         transitions={'new_task':'GIVE_ACTION_WITHOUT_MIC',
        #                                       'tasks_completed':'FINISH'})
        

        ######################################################
        ###################### FINISHED ######################
        ######################################################

        smach.StateMachine.add('FINISH', states.Finish(robot),
                                        transitions={'stop':'Done'})
    return sm

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    startup(setup_statemachine)
