#! /usr/bin/env python
"""Scenario:
amigo in kamer
WM vind unknown blobs (servicecall: ed.srv.SimpleQuery.srv) op /ed/simple_query. query.type should be left empty.
blobs queryen via WM, krijgen IDs.
Vraag TF van ID op (= frame /<ID>), en beweeg naar punt in TF van blob. 
    Of use inverse reachability om in de buurt te komen
    Define a point 0,0,1 in the /<ID> frame, then transform that to /map-coordinates and call NavigateGeneric with lookat_point_3d=(that point in /map)

Als geen unknown objecten in WM, stukje draaien en weer checken of er unknowns zijn.

TODO:
- Find a way to check whether we found some object (i.e. has the right label). Without the reasoner!
---> the service Ed offer has a type-parameter. Filter on that!
- Ask the user an item and position (this is a piece of furniture, for example): Erik is bezig
- Pre-empt driving when the postion of the desired object, like table, is known.
   Do this with a concurrent state: nav || wait for a service call or publication to a topic
- While, driving, look at the clusters TFs one by ones

== Scenario ==
Stage 1
* User stands in front of Amigo
* Speech command: "Get OBJECT from POSITION"
* Turn around
* Navigate to unknown entities
** Try to label entities with use of perception routines
*** Human contour matcher
*** Label interface (web gui) --> User interacts and labels entities
*** ODU finder
*** Size Matcher
*** QR Code detector
** When entity is labeled, fit object model in sensor data (optional)
** Keep navigating to new unknown blobs until POSITION is specified
* When POSITION is specified
** Look for ITEM at POSITION
*** Labeling will be done with use of ODU finder and Size Matcher
** Grasp object
* Return object to operator

Stage 2 - If we have some time left
* Do some things with the labeled world model
** Navigate to unknown entity
*** Amigo ask where the entity is
*** Spatial relations can be used to label the entity


"""
import roslib; roslib.load_manifest('challenge_final')
import rospy

from math import radians, pi

import smach

from ed.srv import SimpleQuery, SimpleQueryRequest
from ed.srv import GetGUICommand, GetGUICommandResponse
import robot_skills.util.msg_constructors as msgs

from std_srvs.srv import Empty #Reset Ed


# Reset ed
from std_srvs.srv import Empty

import robot_smach_states as states
from psi import Compound, Sequence, Conjunction

from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint

from speech_interpreter.srv import AskUser

explore_region_center = msgs.Point(x=4, y=5, z=0) #implicitly is in map. TODO Ed: make this a pointstamped
explore_region_radius = 5

object_to_fetch = None
position_for_nav = None

class NavigateToBlob(smach.State):
    """Ask Ed (Environment Description) what the IDs of unkown blobs are. """
    def __init__(self, robot, blobtype=None, constraint='x^2 + y^2 < 1.2^2', angle=0, mark_visited=True):
        """Blobtype is a string or a function that returns a string"""
        smach.State.__init__(self, outcomes=['arrived', 'unreachable', 'preempted', 'goal_not_defined'])
        self.robot = robot

        self.blobtype = blobtype
        self.constraint = constraint
        self.angle = angle
        self.mark_visited = mark_visited

        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.tf = robot.tf_listener

        self.visited_ids = []

    def execute(self, userdata=None):
        blobtype = self.blobtype() if callable(self.blobtype) else self.blobtype #If blobtype is a function, call it, otherwise use it as is.
        rospy.loginfo("blobtype we're checking for is: '{0}'".format(str(blobtype)))

        query = SimpleQueryRequest(type=blobtype, center_point=explore_region_center, radius=explore_region_radius) #type is a reserved keyword. Maybe unpacking a dict as kwargs is cleaner
        
        object_ids = []
        
        try:
            entities = self.ed(query).entities
            entities = sorted(entities, key=lambda entity: entity.creation_time)
            object_ids = [entity.id for entity in entities]
        except Exception, e:
            rospy.logerr(e)

        #DEBUG (also run rosrun tf static_transform_publisher 5 0 0 0 0 0 /map /object_1 100)
        #DEBUG (also run rosrun tf static_transform_publisher 2 -5 0 0 0 0 /map /object_2 100)
        #import ipdb; ipdb.set_trace()
        #When we're looking for a selected object, object_ids should be one of object_1 or object_2
        # if blobtype: 
        #     object_ids = ["object_1", "object_2"]
        #END DEBUG

        rospy.loginfo("The {1} IDs with type {2} are: {0}".format(object_ids, len(object_ids), blobtype))
        rospy.loginfo("The {1} visited IDs are: {0}".format(self.visited_ids, len(self.visited_ids)))

        object_ids = list(set(object_ids) - set(self.visited_ids))
        rospy.loginfo("The {1} REMAINING IDs with type {2} are: {0}".format(object_ids, len(object_ids), blobtype))


        if object_ids:
            selected_id = object_ids[0]
            rospy.loginfo("Selected ID: {0}".format(selected_id))

            pos = PositionConstraint(selected_id, self.constraint)
            ori = OrientationConstraint(frame=selected_id, look_at=msgs.Point(0,0,0), angle_offset=self.angle)
            nav = states.NavigateWithConstraints(self.robot, pos, ori) 
            result = nav.execute()
            if result == 'arrived' and self.mark_visited:
                self.visited_ids += [selected_id]
            return result
        else:
            return "goal_not_defined"

class TurnDegrees(smach.State):
    def __init__(self, robot, degrees, turnspeed=0.5):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot
        self.degrees = degrees
        self.turnspeed = turnspeed

    def execute(self, userdata=None):
        b = self.robot.base

        angle_in_rads = radians(self.degrees)
        duration = angle_in_rads / self.turnspeed
        b.force_drive(0, 0, self.turnspeed, duration) #turn yourself around, 0.5*2PI rads = 1 pi rads = 180 degrees
        return "Done"

class ExploreStep(smach.State):
    """Drive to a random pose relatively close to the current position"""
    def __init__(self, robot, distance=2):
        smach.State.__init__(self, outcomes=['arrived', 'unreachable', 'preempted', 'goal_not_defined'])
        self.robot = robot
        self.distance = distance
        self.tf = robot.tf_listener

    def execute(self, userdata=None):
        # point_in_unknown_blob_tf = msgs.PointStamped(0,0,0, frame_id="/amigo/base_link") #TF of object is in center of object
        # map_pointstamped = self.tf.transformPoint("/map", point_in_unknown_blob_tf)
        # map_point_tuple = (map_pointstamped.point.x, map_pointstamped.point.y, map_pointstamped.point.z)

        #Go look at our current position from 2 meters. This involves find poses around our current position and selecting a 'best' from it. 
        #This means moving, so in effect some sort of exploration
        #import ipdb; ipdb.set_trace()

        self.robot.base2.pc.constraint = 'x^2 + y^2 > 1.0^2 && x^2 + y^2 < 2.0' #drive somewhere between 1 and 2 meters away
        self.robot.base2.pc.frame      = "/amigo/base_link"

        self.robot.base2.oc.look_at    = msgs.Point(5, 5, 0)
        self.robot.base2.oc.frame      = "/map"
        #self.robot.base2.oc.angle      = pi / 1 #Turn 180 degrees, so look away from where we were

        nav = states.NavigateWithConstraints(self.robot, PositionConstraint(frame="/amigo/base_link", constraint='x^2 + y^2 > 1.0^2 && x^2 + y^2 < 2.0'),
                                                         OrientationConstraint(frame="/map", look_at=msgs.Point(0,0,0))) #Look from X distance to the unknown ubject
        return nav.execute()

        # nav = states.NavigateGeneric(self.robot, lookat_point_3d=map_point_tuple, xy_dist_to_goal_tuple=(self.distance, 0)) #TODO xy_dist_to_goal_tuple is not incorporated in this mode, only when its a query
        # return nav.execute()

class FindUnknownObject(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "preempted"])
        self.robot = robot

        self.turn_angle = 60

        with self:
            find_unknown_iterator = smach.Iterator(outcomes=['found', 'not_found'],
                                                   input_keys=[],
                                                   output_keys=[],
                                                   it=lambda: range(360/self.turn_angle), 
                                                   it_label='confirmation_try', 
                                                   exhausted_outcome='not_found')
            
            with find_unknown_iterator:
                find_unknown = smach.StateMachine(outcomes=['found', 'not_found'])
                with find_unknown:
                    smach.StateMachine.add('DRIVE_TO_UNKNOWN_1',
                                        NavigateToBlob(robot, blobtype=""), #Go to unknwowns, without a type
                                        transitions={   'arrived':'found', 
                                                        'preempted':'not_found', 
                                                        'unreachable':'not_found', 
                                                        'goal_not_defined':'TURN'})
                    smach.StateMachine.add('TURN', 
                                        TurnDegrees(robot, self.turn_angle),
                                        transitions={'Done':'not_found'})

                find_unknown_iterator.set_contained_state('FIND_UNKNOWN', 
                                          find_unknown, 
                                          loop_outcomes=['not_found'], 
                                          break_outcomes=['found'])
            
            smach.StateMachine.add('FIND_UNKNOWN', 
                                   find_unknown_iterator, 
                                   transitions={'found':'Done', 
                                                'not_found':'EXPLORE'})

            smach.StateMachine.add('EXPLORE', 
                                    ExploreStep(robot, distance=3),
                                    transitions={       'arrived':'SAY_FOUND_UNKNOWN', 
                                                        'preempted':'preempted', 
                                                        'unreachable':'Failed', 
                                                        'goal_not_defined':'preempted'})

            smach.StateMachine.add( "SAY_FOUND_UNKNOWN",
                                    states.Say(robot,"I found something I don't know."),
                                    transitions={    "spoken":"Done"})

class AskObjectAndPosition(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot
        self.ask_user_service_get_action = rospy.ServiceProxy('interpreter/ask_user', AskUser)

        self.options_used = 0

    def execute(self, userdata):
        global object_to_fetch
        global position_for_nav

        self.robot.head.look_at_standing_person()

        try:
            self.response = self.ask_user_service_get_action("final_2014", 3, rospy.Duration(60))  # = 1 hour because amigo has to be on standby to receive an action in e-gpsr
            # import ipdb; ipdb.set_trace()
            response_object = "no_answer"

            for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "object":
                    response_object = self.response.values[x]
                elif self.response.keys[x] == "location":
                    response_location = self.response.values[x]

            if response_object == "no_answer" or response_object == "wrong_answer":
                if self.options_used == 0:
                    self.robot.speech.speak("I will just try to find a beer on the sideboard", block=False)
                    response_location = "sideboard"
                    response_object = "beer"
                elif  self.options_used == 1:
                    self.robot.speech.speak("I will just try to find a milk on the dinner table", block=False)
                    response_location = "dinner_table"
                    response_object = "milk"
                elif  self.options_used == 2:
                    self.robot.speech.speak("I will just try to find a beer on the bar", block=False)
                    response_location = "bar"
                    response_object = "beer"
                
                self.options_used += 1

        except rospy.ServiceException, e:

            rospy.loginfo("No action is heared: {0}".format(e))
            #rospy.loginfo("Service call failed ({0})".format(e))

            if self.options_used == 0:
                self.robot.speech.speak("I will just find a beer on the sideboard", block=False)
                response_location = "sideboard"
                response_object = "beer"
            elif  self.options_used == 1:
                self.robot.speech.speak("I will just find a milk on the dinner table", block=False)
                response_location = "dinner_table"
                response_object = "milk"
            elif  self.options_used == 2:
                self.robot.speech.speak("I will just find a beer on the bar", block=False)
                response_location = "bar"
                response_object = "beer"

            self.options_used += 1

        position_for_nav = response_location
        object_to_fetch = response_object

        # Show values for action/start_location/end_location/object      
        rospy.loginfo("Object = {0}".format(response_object))   
        rospy.loginfo("Location = {0}".format(response_location))

        return "Done"

class WaitForPositionLabeled(smach.State):
    """This state is done when there finally is an object labeled with the label we desire.
    The required label can be given by both an argument and a callback function"""

    def __init__(self, robot, blobtype=None, rate=1):
        """Blobtype is a string or a function that returns a string"""
        smach.State.__init__(self, outcomes=['label_found', 'not_found', 'preempted'])
        self.robot = robot

        self.blobtype = blobtype

        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.rate = rospy.Rate(rate)

    def execute(self, userdata=None):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self.rate.sleep()

        blobtype = self.blobtype() if callable(self.blobtype) else self.blobtype #If blobtype is a function, call it, otherwise use it as is.
        rospy.loginfo("blobtype we're checking for is: '{0}'".format(str(blobtype)))
        if not blobtype:
            return "not_found"
        query = SimpleQueryRequest(type=blobtype, center_point=explore_region_center, radius=explore_region_radius)
        
        #DEBUG
        import random
        #if random.random() > 0.95:
        #import ipdb; ipdb.set_trace()
            #DEBUG (also run rosrun tf static_transform_publisher 2 -5 0 0 0 0 /map /object_2 100)
            #DEBUG object_ids = ['object_2']
        #END DEBUG

        try:
            object_ids = [entity.id for entity in self.ed(query).entities]
            if object_ids:
                self.robot.speech.speak("Thanks, boss. I'll go to the {0} as soon as possible".format(blobtype).replace("_", " "),  block=False)
                return 'label_found'
            else:
                return 'not_found'
        except Exception, e:
            rospy.logerr(e)
        
        return 'not_found'

class GoToGuiCommandIfRequested(smach.State):
    """Poll whether the ed gui has received a (new) command. If so, execute that command (i.e go there)"""

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['arrived', 'unreachable', 'preempted', 'goal_not_defined', 'no_command_given', 'wait'])
        self.robot = robot

        self.command_poller = rospy.ServiceProxy('/ed/gui/get_gui_command', GetGUICommand)
        self.last_age = None
        self.last_command_id = None

    def execute(self, userdata=None):
        try:
            response = self.command_poller()
            rospy.loginfo(response)
            #import ipdb; ipdb.set_trace()
            self.last_age = response.age.secs
            
            if response.command == "navigate" and response.command_id != self.last_command_id:
                self.last_command_id = response.command_id
                parameters = dict(zip(response.param_names, response.param_values))

                selected_id = parameters['id']

                nav = states.NavigateWithConstraints(self.robot, PositionConstraint(selected_id, 'x^2 + y^2 < 1.2^2'),#Look from X distance to the unknown object
                                                         OrientationConstraint(frame=selected_id, look_at=msgs.Point(0,0,0))) 

                self.robot.speech.speak("I was told to check something out, lets explore!", block=False)
                return nav.execute()
            if response.command == "explore":
                self.last_command_id = response.command_id

                return 'no_command_given'
            else:
                self.last_command_id = response.command_id

                rospy.sleep(1)
                return 'wait'
        except Exception, e:
            rospy.logerr(e)
        if response.command != "wait":
            return 'no_command_given'
        else:
            return 'wait'

class FinalChallenge2014(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
        self.robot = robot

        arm = robot.leftArm

        #TODO
        object_query = Compound("ed_object_of_type_position", lambda: object_to_fetch, "X", "Y", "Z")

        ed_reset = rospy.ServiceProxy('/ed/reset', Empty)
        ed_reset()

        # exit_query = Compound("waypoint", Compound('exit', "E"), Compound("pose_2d", "X", "Y", "Phi"))
    
        # robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        # robot.reasoner.query(Compound("retractall", Compound("goal", "X")))

        # # Load database
        # robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        # # Assert the current challenge.
        # robot.reasoner.query(Compound("assertz",Compound("challenge", "final")))
            
        def determine_desired_blobtype():
            """Determine where to go """
            #import ipdb; ipdb.set_trace()
            if position_for_nav:
                return position_for_nav
            else:
                rospy.logerr("Could not recall what position to go to")
                return None

        with self:
            # smach.StateMachine.add('INITIALIZE',
            #                     states.Initialize(robot),
            #                     transitions={   'initialized':'GOTO_PERSON_START',
            #                                     'abort':'Aborted'})   


            smach.StateMachine.add('GOTO_PERSON_START',  #The robot should already be here actually, but then we at least have the correct pose
                                    states.NavigateWithConstraints( robot,                                                          #4.682, 4.145
                                                                    position_constraint=
                                                                        PositionConstraint( frame="/map", 
                                                                                            constraint="(x-1.985)^2 + (y-7.633)^2 < 0.2"),
                                                                    orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point(5.0,7.0,0))),
                                    transitions={   'arrived':'ASK_OBJECT_AND_POSITION', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'ASK_OBJECT_AND_POSITION', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add('ASK_OBJECT_AND_POSITION',
                                    AskObjectAndPosition(robot),
                                    transitions={   'Done':'GOTO_REQUESTED_POSITION_IF_REQUESTED'})
            
            smach.StateMachine.add( "GOTO_REQUESTED_POSITION_IF_REQUESTED",
                                    GoToGuiCommandIfRequested(robot),
                                    transitions={   "arrived":"GOTO_UNKNOWN_UNTIL_LABELED",
                                                    "unreachable":'GOTO_UNKNOWN_UNTIL_LABELED',
                                                    "preempted":'GOTO_UNKNOWN_UNTIL_LABELED',
                                                    "goal_not_defined":'GOTO_UNKNOWN_UNTIL_LABELED',
                                                    "no_command_given":'GOTO_UNKNOWN_UNTIL_LABELED',
                                                    "wait":"GOTO_REQUESTED_POSITION_IF_REQUESTED"}) #This means that the select object does not exit (anymore?)


            smach.StateMachine.add( 'GOTO_UNKNOWN_UNTIL_LABELED', #Actuall does not go until labeled but keeps going because we're not in the concurrency
                                    FindUnknownObject(robot),
                                    transitions={   'Done':'CHECK_IF_LABELED', 
                                                    'Failed':'CHECK_IF_LABELED', 
                                                    "preempted":'CHECK_IF_LABELED'})

            smach.StateMachine.add( "CHECK_IF_LABELED",
                WaitForPositionLabeled(robot, blobtype=determine_desired_blobtype),
                transitions={    "label_found":"GOTO_POSITION",
                                 "not_found":'SAY_STILL_HAVENT_FOUND', #"CHECK",
                                 "preempted":'GOTO_POSITION'})  
            
            #------------ END options -----------------------------# 
            
            smach.StateMachine.add( "GOTO_POSITION",
                                    NavigateToBlob(robot, blobtype=determine_desired_blobtype, mark_visited=False),
                                    transitions={   "arrived":"SAY_LOOKING",
                                                    "unreachable":'SAY_STILL_HAVENT_FOUND',
                                                    "preempted":'SAY_LOOKING',
                                                    "goal_not_defined":'SAY_STILL_HAVENT_FOUND'}) #This means there is no object of the desired type

            smach.StateMachine.add( "SAY_LOOKING",
                        states.Say(robot, [ "Looking for your object", "Lets see whart I can find here"], block=True),
                        transitions={    "spoken":"GOTO_OBJECT"})  
            
            smach.StateMachine.add( "SAY_STILL_HAVENT_FOUND",
                                    states.Say(robot, [ "Searching", 
                                                        "I'm still searching", 
                                                        "Still looking",
                                                        "I need to search more"], block=False),
                                    transitions={    "spoken":"GOTO_REQUESTED_POSITION_IF_REQUESTED"})  

            smach.StateMachine.add( "GOTO_OBJECT",
                                    NavigateToBlob(robot, blobtype=lambda: object_to_fetch),
                                    transitions={   "arrived":"SAY_FOUND",
                                                    "unreachable":'SAY_FOUND_UNREACHABLE',
                                                    "preempted":'GOTO_REQUESTED_POSITION_IF_REQUESTED',
                                                    "goal_not_defined":'GOTO_REQUESTED_POSITION_IF_REQUESTED'}) #This means there is no object of the desired type

            smach.StateMachine.add( "SAY_FOUND",
                                    states.Say(robot, ["I Found the object!"]),
                                    transitions={    "spoken":"RETURN_TO_PERSON"})  
            
            smach.StateMachine.add( "SAY_FOUND_UNREACHABLE",
                                    states.Say(robot, ["I know which object to go to, but it is unreachable, sorry"]),
                                    transitions={    "spoken":"RETURN_TO_PERSON"}) 

            # smach.StateMachine.add( 'PICKUP_OBJECT',
            #                         states.GrabMachineWithoutBase(arm, robot, object_query),
            #                         transitions={   "succeeded":"RETURN_TO_PERSON",
            #                                         "failed":'SAY_OBJECT_NOT_GRASPED' })             

            smach.StateMachine.add('RETURN_TO_PERSON',
                                    states.NavigateWithConstraints( robot,
                                                    position_constraint=PositionConstraint( frame="/map", 
                                                                                            constraint="(x-4.682)^2 + (y-4.145)^2 < 0.2"),
                                                    orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point(0,0,0))),
                                    transitions={   'arrived':'ASK_OBJECT_AND_POSITION', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'ASK_OBJECT_AND_POSITION', 
                                                    'goal_not_defined':'ASK_OBJECT_AND_POSITION'})

            # smach.StateMachine.add( "SAY_OBJECT_NOT_GRASPED",
            #                         states.Say(robot, ["I could not grasp the object, sorry.", "Sorry, I could not get the object."]),
            #                         transitions={    "spoken":"ASK_OBJECT_AND_POSITION"})   

            # smach.StateMachine.add( "SAY_BYE",
            #                         states.Say(robot,"Obri-gado. So long, and thanks for all the fish", block=False),
            #                         transitions={    "spoken":"GOTO_EXIT"})           

            # smach.StateMachine.add('GOTO_EXIT',
            #                         states.NavigateWithConstraints(robot,
            #                                         position_constraint=PositionConstraint(frame="/map", constraint="(x-0)^2 + (y-0)^2 < 0.4"),
            #                                         orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point(0,0,0))),
            #                         # states.NavigateGeneric(robot, goal_query=exit_query),
            #                         transitions={   'arrived':'Done', 
            #                                         'preempted':'Aborted', 
            #                                         'unreachable':'Done', 
            #                                         'goal_not_defined':'Failed'})

if __name__ == "__main__":
    rospy.init_node('exec_challenge_final_2014')

    states.util.startup(FinalChallenge2014, initial_state=None)
