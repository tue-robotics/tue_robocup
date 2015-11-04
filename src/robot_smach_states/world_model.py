#! /usr/bin/env python

import rospy
import smach

import robot_skills.util.msg_constructors as msgs

# ----------------------------------------------------------------------------------------------------

class SetPlugins(smach.State):
    def __init__(self, robot, enable=None, disable=None):
        smach.State.__init__(self, outcomes=["done"])
        self.enable = enable
        self.disable = disable
        self.robot = robot

    def execute(self, userdata=None):
        if self.disable:  
            robot.ed.disable_plugins(self.disable)

        if self.enable:
            robot.ed.enable_plugins(self.enable)

        return 'done'



'''
    Initialize world model with a certain configuration.
    Set perception mode to non-continuos and disable laser_integration.
'''
class InitializeWorldModel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata=None):
        # TODO: the state was crashing when calling these arguments,
        #       ASK SJOERD HOW TO USE THIS NOW!
        # self.robot.ed.configure_kinect_segmentation(continuous=False)
        # self.robot.ed.configure_perception(continuous=False)
        # self.robot.ed.disable_plugins(plugin_names=["laser_integration"])
        self.robot.ed.reset()

        return 'done'


'''
    Look at an entiy and segment objects whithin the area desired
     - entity: entity that holds the objects, for example on top
     - searchArea: where are the objects wrt the entity, default = on_top_of
'''
class SegmentObjects(smach.State):
    def __init__(self, robot, objectIDsDes, entityDes, searchArea="on_top_of"):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.entityDes = entityDes
        self.searchArea = searchArea
        self.objectIDsDes = objectIDsDes

    def execute(self, userdata=None):
        entity = self.entityDes.resolve()
        objIDs = []

        # Make sure the head looks at the entity
        pos = entity.pose.position
        self.robot.head.look_at_point(msgs.PointStamped(pos.x, pos.y, 0.5, "/map"), timeout=10)

        # Inspect 'on top of' the entity
        res = self.robot.ed.update_kinect("{} {}".format(self.searchArea, entity.id))

        # string[] new_ids      # ids of new entities
        # string[] updated_ids  # ids of updated entities
        # string[] deleted_ids  # ids of deleted entities
        # string error_msg      # Empty if no errors

        objIDs = objIDs + res.new_ids
        objIDs = objIDs + res.updated_ids
        print "Segmented {} objects!".format(len(objIDs))

        # Classify and update IDs
        objClassif = self.robot.ed.classify(ids=objIDs)
        
        # import ipdb; ipdb.set_trace()
        
        for idx, obj in enumerate(objClassif): 
            print "Object {} is a '{}' (ID: {})".format(idx, obj.type, obj.id)        

        self.objectIDsDes.write(objClassif)

        # Cancel the head goal
        self.robot.head.cancel_goal()

        return 'done'

