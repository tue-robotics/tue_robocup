#! /usr/bin/env python
import rospy

# Body parts
import base
import torso
import arms
import head

# Human Robot Interaction
import speech
from hmi_server.api import Api
import ears
import ebutton
import lights

# Perception
import perception_ed

# tf
import tf_server

# Reasoning/world modeling
import world_model_ed

# Misc: do we need this???
import geometry_msgs
from collections import OrderedDict


class Robot(object):
    """
    Interface to all parts of the robot.
    """
    def __init__(self, robot_name="", wait_services=False):

        self.robot_name = robot_name

        self.tf_listener = tf_server.TFClient()

        # Body parts
        self.base = base.Base(self.robot_name, self.tf_listener, wait_service=wait_services)
        self.torso = torso.Torso(self.robot_name,wait_service=wait_services)
        self.spindle = self.torso
        self.leftArm = arms.Arm(self.robot_name, "left", self.tf_listener)
        self.rightArm = arms.Arm(self.robot_name, "right", self.tf_listener)
        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)

        self.head = head.Head(self.robot_name)

        # Human Robot Interaction
        self.lights = lights.Lights(self.robot_name)
        self.speech = speech.Speech(self.robot_name, wait_services, lambda: self.lights.set_color(1,0,0), lambda: self.lights.set_color(0,0,1))
        self.hmi = Api("/" + self.robot_name + '/hmi')
        self.ears = ears.Ears(self.robot_name, lambda: self.lights.set_color(0,1,0), lambda: self.lights.set_color(0,0,1))
        self.ears._hmi = self.hmi # TODO: when ears is gone, remove this line
        self.ebutton = ebutton.EButton()

        # Perception: can we get rid of this???
        self.perception = perception_ed.PerceptionED(wait_service=wait_services)

        # Reasoning/world modeling
        self.ed = world_model_ed.ED(self.robot_name, self.tf_listener, wait_service=wait_services)

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D, queue_size=10)
        self.base_link_frame = "/"+self.robot_name+"/base_link"

        #Grasp offsets
        #TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

    def standby(self):
        if not self.robot_name == 'amigo':
            rospy.logerr('Standby only works for amigo')
            return
        self.leftArm.reset()
        self.rightArm.reset()
        self.leftArm.send_gripper_goal('close')
        self.rightArm.send_gripper_goal('close')
        self.head.look_down()
        self.torso.low()
        self.lights.set_color(0, 0, 0)

    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    def tf_transform_pose(self, ps,frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps)
        return output_pose

    def get_arm(self, side):
        """Get an arm object and a backup for that arm by giving a side as either a string or an Arm-object
        @param side Either string from robot.arms.keys() or Arm from robot.arms.values()
        >>> robot = Robot("dummy")
        >>> arm, backup_arm = robot.get_arm("left")
        >>> assert(arm == robot.leftArm)
        >>> assert(backup_arm == robot.rightArm)"""
        preferred_side = self.arms[self.arms.keys()[0]]

        #Define which arm is which's backup arm (left backs up for right etc)
        backup_arms = self.arms.values() #Get a *list* of arms i.e. the values of the arm-dict, not the keys
        backup_arms.insert(0, backup_arms.pop()) #Make the last arm the first in the list, so we shift by 1
        backup_str_dict = dict(zip(self.arms.keys(), backup_arms)) #Create a dict again that maps strings to backup-arms
        backup_obj_dict = {self.arms[side]:backup_str_dict[side] for side in self.arms.keys()} #Create a dict that maps e.g. self.LeftArm to self.rightArm

        if isinstance(side, basestring):
            try:
                preferred_side = self.arms[side]
            except KeyError:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                preferred_side = self.arms[self.arms.keys()[0]]
        elif isinstance(side, arms.Arm):
            preferred_side = side
        else:
            print "Unknown arm side:" + str(side) + ". Defaulting to '{0}'".format(preferred_side.side)

        backup_side = backup_obj_dict[preferred_side]
        return preferred_side, backup_side

    def get_left_gripper_pose_map(self):
        """ Gets the pose of the left gripper in map frame"""
        (x, y, z), (rx, ry, rz, rw) = self.tf_listener.lookupTransform("/map", "amigo/grippoint_left")
        import PyKDL as kdl
        result = kdl.Frame(kdl.Rotation.Quaternion(rx, ry, rz, rw), kdl.Vector(x, y, z))
        (roll, pitch, yaw) = result.M.GetRPY()
        print "x: {0}, y: {1}, z:{2}, roll: {3}. pitch: {4}, yaw: {5}".format(x, y, z, roll, pitch, yaw)
        return result

    def close(self):
        try:
            self.head.close()
        except: pass
        # try:
        #     self.worldmodel.close()
        # except: pass

        try:
            self.base.close()
        except: pass

        try:
            self.spindle.close()
        except: pass

        try:
            self.speech.close()
        except: pass

        try:
            self.arms.close()
        except: pass

        try:
            self.leftArm.close()
        except: pass

        try:
            self.rightArm.close()
        except: pass

        try:
            self.perception.close()
        except: pass

        try:
            self.ears.close()
        except: pass

        try:
            self.ebutton.close()
        except: pass

        try:
            self.lights.close()
        except: pass

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()


if __name__ == "__main__":
    rospy.init_node("robot")

    import doctest
    doctest.testmod()
