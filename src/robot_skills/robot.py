#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

# Body parts
import base
import torso
import arms
import head

# Human Robot Interaction
import speech
import ears
import ebutton
import lights

# Perception
import perception
import perception_ed

# tf
import tf
import tf_server

# Reasoning/world modeling
import world_model_ed
import reasoner

# Misc: do we need this???
from util import transformations
import geometry_msgs
import std_msgs.msg
from math import degrees, radians
from psi import Compound, Sequence, Conjunction

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
        self.speech = speech.Speech(self.robot_name, wait_service=wait_services)
        self.ears = ears.Ears(self.robot_name)
        self.ebutton = ebutton.EButton()
        self.lights = lights.Lights()

        # Perception: can we get rid of this???
        self.perception = perception_ed.PerceptionED(wait_service=wait_services)

        # Reasoning/world modeling
        self.ed = world_model_ed.ED(wait_service=wait_services)
        self.reasoner = reasoner.Reasoner()

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D)
        self.base_link_frame = "/"+self.robot_name+"/base_link"

        #Grasp offsets
        #TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

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

        try:
            self.reasoner.close()
        except: pass

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()

    # This function was originally located in base.py, but since the reasoner is needed to get room dimentions, it is placed here.
    def get_base_goal_poses(self, target_point_stamped, x_offset, y_offset, cost_threshold_norm=0.2):

        request = amigo_inverse_reachability.srv.GetBaseGoalPosesRequest()

        request.robot_pose = self.base.location.pose

        request.target_pose.position = target_point_stamped.point
        request.target_pose.orientation.x = 0
        request.target_pose.orientation.y = 0
        request.target_pose.orientation.z = 0
        request.target_pose.orientation.w = 1

        request.cost_threshold_norm = cost_threshold_norm

        request.x_offset = x_offset
        request.y_offset = y_offset
        rospy.logdebug("Inverse reachability request = {0}".format(request).replace("\n", " ").replace("\t", " "))
        response = self._get_base_goal_poses(request)
        rospy.logdebug("Inverse reachability response = {0}".format(response).replace("\n", " ").replace("\t", " "))

        ## Only get poses that are in the same room as the grasp point.

        rooms_dimensions = self.reasoner.query(Compound("room_dimensions", "Room", Compound("size", "Xmin", "Ymin", "Zmin", "Xmax", "Ymax", "Zmax")))

        if rooms_dimensions:
            for x in range(0,len(rooms_dimensions)):
                room_dimensions = rooms_dimensions[x]
                if (target_point_stamped.point.x > float(room_dimensions["Xmin"]) and target_point_stamped.point.x < float(room_dimensions["Xmax"]) and target_point_stamped.point.y > float(room_dimensions["Ymin"]) and target_point_stamped.point.y < float(room_dimensions["Ymax"])):
                    rospy.loginfo("Point for inverse reachability in room: {0}".format(str(room_dimensions["Room"])))
                    rospy.sleep(2)
                    break
                else:
                    room_dimensions = ""

        if rooms_dimensions and room_dimensions:

            x_min = float(room_dimensions["Xmin"])
            x_max = float(room_dimensions["Xmax"])
            y_min = float(room_dimensions["Ymin"])
            y_max = float(room_dimensions["Ymax"])
            #print "x_min = ", x_min, ", x_max = ", x_max, ", y_min = ", y_min, ", y_max = ", y_max, "\n"
            base_goal_poses = []
            for base_goal_pose in response.base_goal_poses:

                x_pose = base_goal_pose.position.x
                y_pose = base_goal_pose.position.y
                # print "x_pose = ", x_pose, ", y_pose = ", y_pose, "\n"
                if (x_pose > x_min and x_pose < x_max and y_pose > y_min and y_pose < y_max):
                    # print "Pose added\n"
                    base_goal_poses.append(geometry_msgs.msg.PoseStamped())
                    base_goal_poses[-1].header.frame_id = "/map"
                    base_goal_poses[-1].header.stamp = rospy.Time()
                    base_goal_poses[-1].pose = base_goal_pose
                # else:
                    # print "point deleted\n"
            # print "AFTER base_goal_poses length = ", len(base_goal_poses), "\n"

        else:
            base_goal_poses = []

            for base_goal_pose in response.base_goal_poses:
                base_goal_poses.append(geometry_msgs.msg.PoseStamped())
                base_goal_poses[-1].header.frame_id = "/map"
                base_goal_poses[-1].header.stamp = rospy.Time()
                base_goal_poses[-1].pose = base_goal_pose

        return base_goal_poses

if __name__ == "__main__":
    rospy.init_node("robot")
    
    import doctest
    doctest.testmod()
