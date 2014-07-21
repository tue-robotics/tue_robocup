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
- Ask the user an item and position (this is a piece of furniture, for example)

== Scenario ==
Stage 1
* User stands in front of Amigo
* Speech command: “Get OBJECT from POSITION”
* Turn around
* Navigate to unknown entities
** Try to label entities with use of perception routines
*** Human contour matcher
*** Label interface (web gui) → User interacts and labels entities
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

Stage 2 – If we have some time left
* Do some things with the labeled world model
** Navigate to unknown entity
*** Amigo ask where the entity is
*** Spatial relations can be used to label the entity


"""
import roslib; roslib.load_manifest('challenge_final')
import rospy

from math import radians

import smach

from ed.srv import SimpleQuery, SimpleQueryRequest
import robot_skills.util.msg_constructors as msgs

import robot_smach_states as states
from psi import Compound, Sequence, Conjunction

class NavigateToUnknownBlob(smach.State):
    """Ask Ed (Environment Description) what the IDs of unkown blobs are. """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['arrived', 'unreachable', 'preempted', 'goal_not_defined'])
        self.robot = robot

        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.tf = robot.tf_listener

        self.visited_ids = []

    def execute(self, userdata=None):
        query = SimpleQueryRequest()

        unknown_ids = self.ed(query).ids
        rospy.loginfo("The {1} unknown IDs are: {0}".format(unknown_ids, len(unknown_ids)))
        rospy.loginfo("The {1} visited IDs are: {0}".format(self.visited_ids, len(self.visited_ids)))

        unknown_ids = list(set(unknown_ids) - set(self.visited_ids))
        rospy.loginfo("The {1} REMAINING unknown IDs are: {0}".format(unknown_ids, len(unknown_ids)))

        # import ipdb; ipdb.set_trace()
        # #DEBUG (also run rosrun tf static_transform_publisher 5 0 0 0 0 0 /map /unknown_1 100)
        # if not unknown_ids: unknown_ids = ["unknown_1"]
        # #END DEBUG

        if unknown_ids:
            selected_id = unknown_ids[0]
            rospy.loginfo("Selected ID: {0}".format(selected_id))

            self.visited_ids += [selected_id]

            point_in_unknown_blob_tf = msgs.PointStamped(0,0,0, frame_id="/"+selected_id) #TF of object is in center of object
            map_pointstamped = self.tf.transformPoint("/map", point_in_unknown_blob_tf)
            map_point_tuple = (map_pointstamped.point.x, map_pointstamped.point.y, map_pointstamped.point.z)

            nav = states.NavigateGeneric(self.robot, lookat_point_3d=map_point_tuple, xy_dist_to_goal_tuple=(1.0, 0)) #Look from X distance to the unknown ubject
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
        point_in_unknown_blob_tf = msgs.PointStamped(0,0,0, frame_id="/amigo/base_link") #TF of object is in center of object
        map_pointstamped = self.tf.transformPoint("/map", point_in_unknown_blob_tf)
        map_point_tuple = (map_pointstamped.point.x, map_pointstamped.point.y, map_pointstamped.point.z)

        #Go look at our current position from 2 meters. This involves find poses around our current position and selecting a 'best' from it. 
        #This means moving, so in effect some sort of exploration
        #import ipdb; ipdb.set_trace()
        nav = states.NavigateGeneric(self.robot, lookat_point_3d=map_point_tuple, xy_dist_to_goal_tuple=(self.distance, 0)) #TODO xy_dist_to_goal_tuple is not incorporated in this mode, only when its a query
        return nav.execute()

class FindUnknownObject(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
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
                                        NavigateToUnknownBlob(robot),
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
                                    transitions={       'arrived':'Done', 
                                                        'preempted':'Aborted', 
                                                        'unreachable':'Failed', 
                                                        'goal_not_defined':'Aborted'})

class AskObjectAndPosition(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

    def execute(self, userdata=None):

        #User says: "Get ITEM from POSITION" Position is like table or bar
        position = "Fridge"
        self.robot.reasoner.assertz(Compound("goal", Compound("position", position)))
        
        item = "Coke"
        self.robot.reasoner.assertz(Compound("goal", Compound("item", item)))

        return "Done"

class FinalChallenge2014(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
        self.robot = robot

        arm = robot.leftArm
        position_query = Conjunction(  
                                    Compound("goal", Compound("position", "Position")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z"))) 

        #TODO
        object_query = Conjunction(  
                                    Compound("goal", Compound("item", "Object")),
                                    Compound( "property_expected", "ObjectID", "class_label", "Object"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z"))) 

        #robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))

        # Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        # Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "final")))

        with self:
            # smach.StateMachine.add('INITIALIZE',
            #                     states.Initialize(robot),
            #                     transitions={   'initialized':'GOTO_UNKNOWN',
            #                                     'abort':'Aborted'})        
            smach.StateMachine.add('GOTO_PERSON_START',  #The robot should already be here actually, but then we at least have the correct pose
                                states.NavigateGeneric(robot, goal_name="person_position"),
                                transitions={   'arrived':'ASK_OBJECT_AND_POSITION', 
                                                'preempted':'Aborted', 
                                                'unreachable':'ASK_OBJECT_AND_POSITION', 
                                                'goal_not_defined':'Failed'})

            smach.StateMachine.add( 'ASK_OBJECT_AND_POSITION',
                                    AskObjectAndPosition(robot),
                                    transitions={   'Done':'GOTO_UNKNOWN'})

            smach.StateMachine.add( 'GOTO_UNKNOWN',
                                    FindUnknownObject(robot),
                                    transitions={   'Done':'SAY_FOUND_UNKNOWN', 
                                                    'Failed':'SAY_FOUND_UNKNOWN', 
                                                    "Aborted":'SAY_FOUND_UNKNOWN'})

            smach.StateMachine.add( "SAY_FOUND_UNKNOWN",
                                    states.Say(robot,"I found something I don't know."),
                                    transitions={    "spoken":"CHECK_IF_POSITION_FOUND"})

            smach.StateMachine.add( "CHECK_IF_POSITION_FOUND",
                                    states.Ask_query_true(robot, position_query),
                                    transitions={'query_false':'GOTO_UNKNOWN', 
                                                 'query_true':'GOTO_POSITION', 
                                                 'waiting':'CHECK_IF_POSITION_FOUND', 
                                                 'preempted':'Aborted'})

            #TODO: I stiiiiiiil havent found what i'm loooking for (U2)

        
            smach.StateMachine.add( "GOTO_POSITION",
                                    states.NavigateGeneric(robot, lookat_query=position_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    transitions={   "arrived":"PICKUP_OBJECT",
                                                    "unreachable":'PICKUP_OBJECT',
                                                    "preempted":'PICKUP_OBJECT',
                                                    "goal_not_defined":'PICKUP_OBJECT'})

            smach.StateMachine.add( 'PICKUP_OBJECT',
                                    states.GrabMachine(arm, robot, object_query),
                                    transitions={   "succeeded":"RETURN_TO_PERSON",
                                                    "failed":'SAY_OBJECT_NOT_GRASPED' })             

            smach.StateMachine.add('RETURN_TO_PERSON',
                                    states.NavigateGeneric(robot, goal_name="person_position"),
                                    transitions={   'arrived':'GOTO_EXIT', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'GOTO_EXIT', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add( "SAY_OBJECT_NOT_GRASPED",
                                    states.Say(robot,"I could not grasp the object, sorry guys."),
                                    transitions={    "spoken":"GOTO_EXIT"})            

            smach.StateMachine.add('GOTO_EXIT',
                                    states.NavigateGeneric(robot, goal_name="exit"),
                                    transitions={   'arrived':'ASK_OBJECT_AND_POSITION', 
                                                    'preempted':'Aborted', 
                                                    'unreachable':'ASK_OBJECT_AND_POSITION', 
                                                    'goal_not_defined':'Failed'})



if __name__ == "__main__":
    rospy.init_node('exec_challenge_final_2014')

    states.util.startup(FinalChallenge2014)
