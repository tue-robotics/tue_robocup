#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import head
import worldmodel
import base
import spindle
import speech
import arms
import perception
import ears
import ebutton
import lights
#import reasoner

import tf

import geometry_msgs

class Amigo(object):
    """
    Interface to all parts of Amigo. When initializing Amigo, you can choose a list of components
    which wont be needed
    
    # I want a blind and mute Amigo!
    >>> Amigo(['perception', 'speech'])
    
    # I want a full fledged, awesome Amigo
    >>> Amigo()
    """
    def __init__(self, dontInclude = [], wait_services=True):
        
        self.tf_listener = tf.TransformListener()

        if 'head' not in dontInclude:
            self.head = head.Head()
        if 'base' not in dontInclude:
            self.base = base.Base(self.tf_listener, wait_service=wait_services)
        if 'spindle' not in dontInclude:
            self.spindle = spindle.Spindle(wait_service=wait_services)
        if 'arms' not in dontInclude:
            self.arms = arms.Arms(self.tf_listener) #TODO: use self.tf_listener here
        if 'leftArm' not in dontInclude:
            self.leftArm = arms.Arm(arms.Side.LEFT, self.tf_listener)
        if 'rightArm' not in dontInclude:
            self.rightArm = arms.Arm(arms.Side.RIGHT, self.tf_listener)
        if 'ebutton' not in dontInclude:
            self.ebutton = ebutton.EButton()
        self.leftSide = arms.Side.LEFT
        self.rightSide = arms.Side.RIGHT
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D)
'''
    def tf_transform(self, coordinates, inputframe, outputframe):
        ps = geometry_msgs.msg.PointStamped(point= coordinates) 
        ps.header.frame_id = inputframe
        ps.header.stamp = rospy.Time()
        output_coordinates = self.tf_listener.transformPoint(outputframe, ps)
        return output_coordinates.point
        
    def tf_transform_pose(self, ps,frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps) 
        return output_pose
'''
if __name__ == "__main__":
    rospy.init_node("amigo_stopper", anonymous=True)
    amigo = Amigo(wait_services=True)
    amigo.base.cancel_goal()
    amigo.leftArm.cancel_goal()
    amigo.rightArm.cancel_goal()
    amigo.head.reset_position()
    
    '''head_reset = lambda: amigo.head.reset_position()
    head_down  = lambda: amigo.head.look_down()
    look_at = lambda id: amigo.look_at_id(id)
    right_close = lambda: amigo.rightArm.send_gripper_goal_close()
    left_close = lambda: amigo.leftArm.send_gripper_goal_close()
    right_open = lambda: amigo.rightArm.send_gripper_goal_open()
    left_open = lambda: amigo.leftArm.send_gripper_goal_open()
    speak = lambda sentence: amigo.speech.speak(sentence)
    look_at_point = lambda x, y, z: amigo.head.send_goal(amigo.head.point(x, y, z), frame_id="/base_link")
    go = amigo.base.go'''
    
