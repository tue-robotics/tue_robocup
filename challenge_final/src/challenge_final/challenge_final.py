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
import robot_skills.util.msg_constructors as msgs

import robot_smach_states as states
from psi import Compound, Sequence, Conjunction

from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint

explore_region_center = msgs.Point(x=4, y=5, z=0) #implicitly is in map. TODO Ed: make this a pointstamped
explore_region_radius = 5

class NavigateToBlob(smach.State):
    """Ask Ed (Environment Description) what the IDs of unkown blobs are. """
    def __init__(self, robot, blobtype=None):
        """Blobtype is a string or a function that returns a string"""
        smach.State.__init__(self, outcomes=['arrived', 'unreachable', 'preempted', 'goal_not_defined'])
        self.robot = robot

        self.blobtype = blobtype

        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.tf = robot.tf_listener

        self.visited_ids = []

    def execute(self, userdata=None):
        blobtype = self.blobtype() if callable(self.blobtype) else self.blobtype #If blobtype is a function, call it, otherwise use it as is.
        query = SimpleQueryRequest(type=blobtype, center_point=explore_region_center, radius=explore_region_radius) #type is a reserved keyword. Maybe unpacking a dict as kwargs is cleaner
        
        object_ids = []
        
        try:
            object_ids = [entity.id for entity in self.ed(query).entities]
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

            self.visited_ids += [selected_id]

            nav = states.NavigateWithConstraints(self.robot, PositionConstraint(selected_id, 'x^2 + y^2 < 1.2^2'),#Look from X distance to the unknown object
                                                             OrientationConstraint(frame=selected_id, look_at=msgs.Point())) 
            return nav.execute()
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
        self.robot.base2.oc.angle      = pi / 1 #Turn 180 degrees, so look away from where we were

        nav = states.NavigateWithConstraints(self.robot) #Look from X distance to the unknown ubject
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

    def execute(self, userdata=None):

        #User says: "Get ITEM from POSITION" Position is like table or bar
        position = "fridge"
        self.robot.reasoner.assertz(Compound("goal", Compound("position", position)))
        
        item = "coke"
        self.robot.reasoner.assertz(Compound("goal", Compound("item", item)))

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
                return 'label_found'
            else:
                return 'not_found'
        except Exception, e:
            rospy.logerr(e)
        
        return 'not_found'
        
class FinalChallenge2014(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
        self.robot = robot

        arm = robot.leftArm

        #TODO
        object_query = Conjunction(  
                                    Compound("goal", Compound("item", "Object")),
                                    Compound( "property_expected", "ObjectID", "class_label", "Object"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z"))) 

        exit_query = Compound("waypoint", Compound('exit', "E"), Compound("pose_2d", "X", "Y", "Phi"))
    
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))

        # Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        # Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "final")))
            
                                    
        #TODO: I stiiiiiiil havent found what i'm loooking for (U2)
        def determine_desired_blobtype():
            """Query the reasoner to recall what position we were supposed to go to"""
            answers = robot.reasoner.query(Compound('goal', Compound("position", "Position")))
            answers_filtered = [ans for ans in answers if ans["Position"].is_constant()]
            rospy.loginfo("Position answers_filtered: {0}".format(answers_filtered))
            if answers_filtered:
                return str(answers_filtered[0]["Position"])
            else:
                rospy.logerr("Could not recall what position to go to")
                return ""

        with self:
            # smach.StateMachine.add('INITIALIZE',
            #                     states.Initialize(robot),
            #                     transitions={   'initialized':'GOTO_PERSON_START',
            #                                     'abort':'Aborted'})   


            smach.StateMachine.add('GOTO_PERSON_START',  #The robot should already be here actually, but then we at least have the correct pose
                                    states.NavigateWithConstraints( robot,                                                          #4.682, 4.145
                                                                    position_constraint=
                                                                        PositionConstraint( frame="/map", 
                                                                                            constraint="(x-4.682)^2 + (y-4.145)^2 < 0.2"),
                                                                    orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point())),
                                    transitions={   'arrived':'ASK_OBJECT_AND_POSITION', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'ASK_OBJECT_AND_POSITION', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add( 'ASK_OBJECT_AND_POSITION',
                                    AskObjectAndPosition(robot),
                                    transitions={   'Done':'GOTO_UNKNOWN_UNTIL_LABELED'})
            
            #------------ Option 1: concurrency ------------------# 
            # cc = smach.Concurrence( outcomes        = ['position_labeled', 'position_not_labeled'],
            #                         default_outcome = 'position_not_labeled',
            #                         outcome_map     = {'position_labeled'  :{  'CHECK_IF_ALREADY_FOUND':'position_labeled'}},
            #                         child_termination_cb = lambda x: False) #Pre-empt all children when one fails
            # with cc:
            #     find_unknown = FindUnknownObject(robot)

            #     iter_waiting = smach.StateMachine(outcomes=['position_labeled', 'position_not_labeled'])
            #     with iter_waiting:
            #         smach.StateMachine.add( "CHECK",
            #             WaitForPositionLabeled(robot, blobtype=determine_desired_blobtype),
            #             transitions={    "label_found":"position_labeled",
            #                              "not_found":'CHECK', #"CHECK",
            #                              "preempted":'position_not_labeled'})  

            #     smach.Concurrence.add('CHECK_IF_ALREADY_FOUND', iter_waiting)
            #     smach.Concurrence.add('FIND_UNKNOWN', find_unknown)

            # smach.StateMachine.add( "GOTO_UNKNOWN_UNTIL_LABELED",
            #                         cc,
            #                         transitions={   "position_labeled":"GOTO_POSITION",
            #                                         "position_not_labeled":'SAY_STILL_HAVENT_FOUND'}) #This means there is no object of the desired type
            
            #------------ Option 2: sequentially ------------------# 
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
                                    NavigateToBlob(robot, blobtype=determine_desired_blobtype),
                                    transitions={   "arrived":"PICKUP_OBJECT",
                                                    "unreachable":'PICKUP_OBJECT',
                                                    "preempted":'PICKUP_OBJECT',
                                                    "goal_not_defined":'SAY_STILL_HAVENT_FOUND'}) #This means there is no object of the desired type
            
            smach.StateMachine.add( "SAY_STILL_HAVENT_FOUND",
                                    states.Say(robot,"I still haven't found what I'm looking for"),
                                    transitions={    "spoken":"GOTO_UNKNOWN_UNTIL_LABELED"})  

            smach.StateMachine.add( 'PICKUP_OBJECT',
                                    states.GrabMachine(arm, robot, object_query),
                                    transitions={   "succeeded":"RETURN_TO_PERSON",
                                                    "failed":'SAY_OBJECT_NOT_GRASPED' })             

            smach.StateMachine.add('RETURN_TO_PERSON',
                                    states.NavigateWithConstraints( robot,
                                                    position_constraint=PositionConstraint( frame="/map", 
                                                                                            constraint="(x-4.682)^2 + (y-4.145)^2 < 0.2"),
                                                    orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point())),
                                    transitions={   'arrived':'GOTO_EXIT', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'GOTO_EXIT', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add( "SAY_OBJECT_NOT_GRASPED",
                                    states.Say(robot,"I could not grasp the object, sorry guys."),
                                    transitions={    "spoken":"GOTO_EXIT"})            

            smach.StateMachine.add('GOTO_EXIT',
                                    states.NavigateWithConstraints(robot,
                                                    position_constraint=PositionConstraint(frame="/map", constraint="(x-0)^2 + (y+1)^2 < 0.4"),
                                                    orientation_constraint=OrientationConstraint(frame="/map", look_at=msgs.Point())),
                                    # states.NavigateGeneric(robot, goal_query=exit_query),
                                    transitions={   'arrived':'Done', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'Done', 
                                                    'goal_not_defined':'Failed'})

if __name__ == "__main__":
    rospy.init_node('exec_challenge_final_2014')

    states.util.startup(FinalChallenge2014)
