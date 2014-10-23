#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import head
#import worldmodel
import base
import base2
import spindle
import speech
import arms
import perception
import perception_ed
import ears
import ebutton
import lights

import tf
import tf_server

from util import transformations
#from components import message_helper
import geometry_msgs
import std_msgs.msg

from math import degrees, radians

import amigo_inverse_reachability.srv # ToDo: rename service
from psi import Compound, Sequence, Conjunction

class Arm(object):
    pass

class Robot(object):
    """
    Interface to all parts of the robot.
    """
    def __init__(self, robot_name="", wait_services=False, armClass=None, torsoClass=spindle.Spindle):

        self.robot_name = robot_name
        
        self._get_base_goal_poses = rospy.ServiceProxy('/inverse_reachability/inverse_reachability', amigo_inverse_reachability.srv.GetBaseGoalPoses)

        self.tf_listener = tf_server.TFClient()

        # Determine if we are using ED (world model)
        self.use_ed = rospy.get_param('/use_ed', False)
        if self.use_ed:
            rospy.loginfo("Using ED")
        else:
            rospy.loginfo("NOT using ED")

        self.head = head.Head()

        if self.use_ed:
            self.base = base2.Base(self, self.tf_listener, wait_service=wait_services) # Added by Rein (new nav interface)
            self.base2 = self.base # Needed for now, because NavigateWithConstraints uses robot.base2
        else:
            self.base = base.Base(self.tf_listener, wait_service=wait_services, use_2d=None) #None indicates: sort it out yourselve

        self.spindle = torsoClass(wait_service=wait_services)
        self.speech = speech.Speech(wait_service=wait_services)
        self.arms =  armClass(self.tf_listener)   #arms.Arms(self.tf_listener) #TODO: use self.tf_listener here
        self.leftArm = arms.Arm(arms.Side.LEFT, self.tf_listener)
        self.rightArm = arms.Arm(arms.Side.RIGHT, self.tf_listener)

        if self.use_ed:
             self.perception = perception_ed.PerceptionED(wait_service=wait_services)
        else:
            try:
                self.perception = perception.Perception(wait_service=wait_services)
            except:
                rospy.logwarn("Perception could not be initialized. Is the providing node running?")
        
        self.ears = ears.Ears()
        self.ebutton = ebutton.EButton()
        self.lights = lights.Lights()
        try:
            import reasoner
            self.reasoner = reasoner.Reasoner()
        except ImportError:
            rospy.logwarn("Reasoner could not be imported into robot")
        
        self.leftSide = arms.Side.LEFT
        self.rightSide = arms.Side.RIGHT
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D)

        self.base_link_frame = "/"+self.robot_name+"/base_link"
    
    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))
        
    def tf_transform_pose(self, ps,frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps) 
        return output_pose

    def store_position_knowledge(self, label, dx=0.78, z=0.75, drop_dz=0.13, filename='/tmp/locations.pl'):

        # Query reasoner for environment name
        ans_env = self.reasoner.query(Compound("environment", "Env"))

        if not ans_env:
            rospy.logerr("Could not get environment name from reasoner.")
            return

        env_name = ans_env[0]["Env"]
        
        # Determine base position as (x, y, phi)
        pose = self.base.location
        (pos, quat) = pose.pose.position, pose.pose.orientation
        phi = self.base.phi(quat)

        base_pose = Compound("waypoint", env_name, "Challenge", label, Compound("pose_2d", round(pos.x, 3), round(pos.y, 3), round(phi, 3)))
        print base_pose

        # Determine lookat point (point of interest)

        time = rospy.Time.now()

        ps = geometry_msgs.msg.PointStamped()
        ps.header.stamp = time
        ps.header.frame_id = self.base_link_frame
        ps.point.x = dx # dx meter in front of robot
        ps.point.y = 0.0
        ps.point.z = z

        self.tf_listener.waitForTransform("/map", ps.header.frame_id, time, rospy.Duration(2.0))
        ps_MAP = self.tf_listener.transformPoint("/map", ps)

        poi = Compound("point_of_interest", env_name, "Challenge", label, Compound("point_3d", round(ps_MAP.point.x, 3), round(ps_MAP.point.y, 3), round(ps_MAP.point.z, 3)))
        print poi

        dropoff = Compound("dropoff_point", env_name, "Challenge", label, Compound("point_3d", round(ps_MAP.point.x, 3), round(ps_MAP.point.y, 3), round(ps_MAP.point.z + drop_dz, 3)))
        print dropoff        

        with open(filename, "a") as myfile:
            myfile.write(str(base_pose) + ".\n")
            myfile.write(str(poi) + ".\n")
            myfile.write(str(dropoff) + ".\n")

        # assert the facts to the reasoner
        #self.reasoner.query(Compound("assert", base_pose))
        #self.reasoner.query(Compound("assert", poi))

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

    pass
