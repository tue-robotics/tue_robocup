#! /usr/bin/env python
"""Scenario:
amigo in kamer
WM vind unknown blobs (servicecall: ed.srv.SimpleQuery.srv) op /ed/simple_query. query.type should be left empty.
blobs queryen via WM, krijgen IDs.
Vraag TF van ID op (= frame /<ID>), en beweeg naar punt in TF van blob. 
    Of use inverse reachability om in de buurt te komen
    Define a point 0,0,1 in the /<ID> frame, then transform that to /map-coordinates and call NavigateGeneric with lookat_point_3d=(that point in /map)

Als geen unknown objecten in WM, stukje draaien en weer checken of er unknowns zijn.

"""
import roslib; roslib.load_manifest('challenge_final')
import rospy

from math import radians

import smach

from ed.srv import SimpleQuery, SimpleQueryRequest
import robot_skills.util.msg_constructors as msgs

import robot_smach_states as states

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

class FinalChallenge2014(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
        self.robot = robot

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'GOTO_UNKNOWN',
                                                'abort':'Aborted'})

            smach.StateMachine.add('GOTO_UNKNOWN',
                                FindUnknownObject(robot),
                                transitions={   'Done':'SAY_FOUND_UNKNOWN', 
                                                'Failed':'SAY_FOUND_UNKNOWN', 
                                                "Aborted":'SAY_FOUND_UNKNOWN'})

            smach.StateMachine.add( "SAY_FOUND_UNKNOWN",
                                states.Say(robot,"I found something I don't know."),
                                transitions={    "spoken":"GOTO_UNKNOWN"})


if __name__ == "__main__":
    rospy.init_node('exec_challenge_final_2014')

    states.util.startup(FinalChallenge2014)
