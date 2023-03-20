#!/usr/bin/env python
simulation = False

#system import
import typing

#ros import
import rospy
import rosapi
from rosapi import srv
from std_msgs.msg import String, Header
import actionlib
from tue_msgs.msg import LocateDoorHandleGoal
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion, Twist
from pykdl_ros import FrameStamped, VectorStamped
from cb_base_navigation_msgs.msg import LocalPlannerActionResult
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList

# robot import
from ed.entity import Entity
import robot_skills
from robot_skills import get_robot

# smach import
import smach
from robot_smach_states.util.designators import Designator

#other import
import numpy
import PyKDL as kdl
import tf2_pykdl_ros as tf2_kdl
import math
from robot_smach_states.navigation import NavigateToGrasp

if simulation:
    from opening_door.srv import door_info
else:
    from robot_smach_states.srv import door_info

#function use to create geometry_msgs
def create_pose_stamped(x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def create_twist(a,b,c,d,e,f):
    pub = Twist()
    pub.angular.x = a
    pub.angular.y = b
    pub.angular.z = c
    pub.linear.x = d
    pub.linear.y = e
    pub.linear.z = f

    return pub

#fonction that allow us to know when we arrive at destination
def position_achieve():
    shared_planner_message = rospy.wait_for_message("/hero/local_planner/action_server/result", LocalPlannerActionResult)
    while(shared_planner_message is None):
        rospy.sleep(2)

    return True

def print_current_pos(robot, message):
    """
    This function is made in order to print useful information about the robot position.
    For example, if you want to find the position of the robot according to the door, you can use this function.
    """
    frame_current_pos = robot.base.get_location() #frame map 
    
    #convert it to frame door_inside
    pos_door_inside = robot.tf_buffer.transform(frame_current_pos, "door_inside", rospy.Duration(1.0))
    rospy.loginfo(message)
    rospy.loginfo(pos_door_inside)

class Door(Entity):
    HANDLE_ID = "handle_volume"
    HANDLE_BEHIND_ID = "handle_behind_volume"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"
    HANDLE_POSE = None
    FRAME_ID = "door_inside"

    #constructor.
    #no more init that a classic entity
    def __init__(self, entity: Entity):

        super().__init__(
            identifier=entity.uuid,
            object_type=entity.etype,
            frame_id=entity.frame_id,
            pose=entity.pose.frame,
            shape=entity.shape,
            volumes=entity.volumes,
            super_types=entity.super_types,
            last_update_time=entity.last_update_time,
        )
        self.my_entity = entity

    @property
    def handle_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """
        return self._get_volume_center_point_in_map(self.HANDLE_ID)

    @property
    def handle_behind_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """

        return self._get_volume_center_point_in_map(self.HANDLE_BEHIND_ID)

    @property
    def frame_points(self) -> typing.List[VectorStamped]:
        """
        Returns the ground points of the door frame in map frame
        """
        return [self._get_volume_center_point_in_map(self.FRAME_LEFT_POINT_ID),
                self._get_volume_center_point_in_map(self.FRAME_RIGHT_POINT_ID)]

    def _get_volume_center_point_in_map(self, volume_id: str) -> VectorStamped:
        """
        Gets the center point of a volume (typically defined w.r.t. the entity frame) and converts this to map frame.

        :param volume_id: id of the volume to get the center point from
        :return: center point converted to map frame
        """
        cp_entity = self.volumes[volume_id].center_point
        cp_map = self.pose.frame * cp_entity
        return VectorStamped.from_xyz(cp_map.x(), cp_map.y(), cp_map.z(), rospy.Time.now(),"map")

    def updateHandlePose(self,new_pose):
        #rospy.loginfo("new pose = " + str(new_pose))
        self.HANDLE_POSE = new_pose

    def getHandlePose(self):
        #rospy.loginfo("handle pose = " + str(self.HANDLE_POSE))
        return self.HANDLE_POSE

    def getFrameIFOdoor_face(self):
        """
        This fonction is hard coded according to the door.
        It rpresents ths position the robot has to be in the fonction goIFOdoor according to the door frame.
        This position has to be converted to the frame map before being send to the global planner.
        """
        kdl_vector = kdl.Vector(1.15, 0.05, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,-3.12)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

    def getFrameIFOhandle_face(self):
        """
        This fonction is hard coded according to the door.
        It rpresents ths position the robot has to be in the fonction goIFOhandme according to the door frame.
        This position has to be converted to the frame map before being send to the global planner.
        """
        kdl_vector = kdl.Vector(0.75, 0.081, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,2.75)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

    def getFrameIFOdoor_behind(self):
        """
        This fonction is hard coded according to the door.
        It rpresents ths position the robot has to be in the fonction goIFOdoor according to the door frame.
        This position has to be converted to the frame map before being send to the global planner.
        """
        kdl_vector = kdl.Vector(-1.15, 0.0, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,0.0444)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

    def getFrameIFOhandle_behind(self):
        """
        This fonction is hard coded according to the door.
        It rpresents ths position the robot has to be in the fonction goIFOhandme according to the door frame.
        This position has to be converted to the frame map before being send to the global planner.
        """
        kdl_vector = kdl.Vector(-0.73, 0.052, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,0.1)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

    def getRotationFrame(self):
        """
        This fonction is hard coded according to the door.
        It represents ths position the robot has to be in the fonction to end to pull the door
        It is in the frame door_inside
        """
        kdl_vector1 = kdl.Vector(-1.32, 0.05, 0)
        kdl_rot1 = kdl.Rotation.RPY(0,0,0.2933)
        kdl_frame1 = kdl.Frame(kdl_rot1, kdl_vector1)
        kdl_frame_stamped1 = FrameStamped(kdl_frame1, rospy.Time.now(), frame_id = "door_inside")

        kdl_vector2 = kdl.Vector(-1.32, 0.05, 0)
        kdl_rot2 = kdl.Rotation.RPY(0,0,-0.6444)
        kdl_frame2 = kdl.Frame(kdl_rot2, kdl_vector2)
        kdl_frame_stamped2 = FrameStamped(kdl_frame2, rospy.Time.now(), frame_id = "door_inside")
        
        return kdl_frame_stamped1, kdl_frame_stamped2
        
class getSOD(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['find_SOD', 'fail'], output_keys=['side_of_door'])

    def execute(self, userdata):
        """
        SOD means side of door
        When this function is execute, it finds the SOD that must be open by the robot.
        Nevertheless the SOD is find thanks to the current position of the robot (the closest SOD is the SOD to open).
        """
        #get frame IFO door face according to the door (it is commented in goIFOdoor)
        frameIFOdoor_door_face = self.door.getFrameIFOdoor_face()
        frameIFOdoor_map_face = self.robot.tf_buffer.transform(frameIFOdoor_door_face, "map", rospy.Duration(1.0))
        kdl_vector_face = frameIFOdoor_map_face.frame.p

        #get frame IFO door behind according to the door
        frameIFOdoor_door_behind = self.door.getFrameIFOdoor_behind()
        frameIFOdoor_map_behind = self.robot.tf_buffer.transform(frameIFOdoor_door_behind, "map", rospy.Duration(1.0))
        kdl_vector_behind = frameIFOdoor_map_behind.frame.p
        #kdl_quaternion_behind = frameIFOdoor_map_behind.frame.M.GetQuaternion()
        #pose_stamped_IFOdoor_behind = create_pose_stamped(kdl_vector_behind[0], kdl_vector_behind[1], kdl_vector_behind[2], kdl_quaternion_behind[0], kdl_quaternion_behind[1], kdl_quaternion_behind[2], kdl_quaternion_behind[3])

        frame_current_pos = self.robot.base.get_location()
        kdl_vector_pos = frame_current_pos.frame.p
        #calcul distances
        d_face = ( (kdl_vector_face[0]-kdl_vector_pos[0])**2 + (kdl_vector_face[1]-kdl_vector_pos[1])**2)**0.5
        d_behind = ( (kdl_vector_behind[0]-kdl_vector_pos[0])**2 + (kdl_vector_behind[1]-kdl_vector_pos[1])**2)**0.5

        if d_face < d_behind:
            rospy.loginfo("SOD = face")
            userdata.side_of_door = "face"

        else:
            rospy.loginfo("SOD = behind")
            userdata.side_of_door = "behind"

        return 'find_SOD'

class goIFOdoor(smach.State):
    def __init__(self, door, robot):
        smach.State.__init__(self, outcomes=['IFO_door','fail'], input_keys=['side_of_door'])
        self.door = door
        self.robot = robot
        self.pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=2)

    def execute(self, userdata):
        """ 
        IFD means In Front Of
        This function is using the global plannar to move the robot IFO the door
        """
        #according to the SOD we are working on, we are going to get and publish a different poseStamped
        if userdata.side_of_door == 'face':
            #get the frame of IFO door according to the door
            frameIFOdoor_door = self.door.getFrameIFOdoor_face()

        else:
            frameIFOdoor_door = self.door.getFrameIFOdoor_behind()

        #convert this frame in the map frame
        frameIFOdoor_map = self.robot.tf_buffer.transform(frameIFOdoor_door, "map", rospy.Duration(1.0))

        #get the vector and the quaternion
        kdl_vector = frameIFOdoor_map.frame.p
        kdl_quaternion = frameIFOdoor_map.frame.M.GetQuaternion()

        #create the pose stamped message with the position IFO the door
        pose_stamped_IFOdoor = create_pose_stamped(kdl_vector[0], kdl_vector[1], kdl_vector[2], kdl_quaternion[0], kdl_quaternion[1], kdl_quaternion[2], kdl_quaternion[3])

        #publication
        rospy.sleep(0.5)
        self.pub.publish(pose_stamped_IFOdoor)

        position_achieve()
        
        return 'IFO_door'

class updateDoorState(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['close','intermediate','open', 'fail'])
        self.rate = rospy.Rate(0.3)
        self.robot = robot

    def execute(self, userdata):
        """
        This function check the state of the door.
        
        it first extracts the segment of the door.
        If the segment doesn't exist the door is considered to be fully open
        if the segement exists, we check if the x_start and x_end are close. If yes, the door is close, if no, the door is open.

        Before doing all of this, we have to check the rotation of the robot, to be sure to use the data straight to the door, even if the robot is a bit turned according to the door
        """
        rospy.sleep(3)
        msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
        segments = msg_line_segments.line_segments # extract the segments

        start = None
        #get the useful segment
        for segment in segments:
            if segment.start[1] < 0 and segment.end[1] > 0:
                start = segment.start
                end = segment.end
        
        if start == None:
            return 'open'
        
        #convert each points in the frame map
        #self.robot.tf_buffer.transform(handle_frame, self.robot.base_link_frame, rospy.Duration(1.0))
        point_start_robot = PointStamped()
        point_start_robot.header.frame_id = self.robot.base_link_frame
        point_start_robot.point.x = start[0]
        point_start_robot.point.y = start[1]
        point_start_robot.point.z = 0
        
        point_end_robot = PointStamped()
        point_end_robot.header.frame_id = self.robot.base_link_frame
        point_end_robot.point.x = end[0]
        point_end_robot.point.y = end[1]
        point_end_robot.point.z = 0
        
        point_start_door = self.robot.tf_buffer.transform(point_start_robot, "door_inside", rospy.Duration(1.0))
        point_end_door = self.robot.tf_buffer.transform(point_end_robot, "door_inside", rospy.Duration(1.0))
        
        if abs( abs(point_start_door.point.x) - abs(point_end_door.point.x)) < 0.2 : #10 cm is big enough to consider the door is open
            return 'close'
        
        return 'intermediate'
            
class updateHandleLocationFromServiceServer(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        #service
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        smach.State.__init__(self, outcomes=['updated','fail'], input_keys=['side_of_door'])

    def execute(self, userdata):
        """ 
        This function call the service server that detect the handle and then send back the point that is the middle of the handle.
        For more information, have a look tat the service server
        """
        #get the frame of the handle
        if userdata.side_of_door == 'face':
            handle_estimate = self.door.handle_pose
        else:
            handle_estimate = self.door.handle_behind_pose

        #write y direction in service server
        PointStamped_y = PointStamped()
        PointStamped_y.header.frame_id = "door_inside"
        PointStamped_y.point.x = 0
        PointStamped_y.point.y = -1
        PointStamped_y.point.z = 0
        self.door_info.call("set_y_direction", PointStamped_y)

        #handle estimate is a vectorStamped, i have to convert it to a frameStamped
        handle_point_estimate = PointStamped()
        handle_point_estimate.header.frame_id = "map"
        handle_point_estimate.point.x = handle_estimate.vector.x()
        handle_point_estimate.point.y = handle_estimate.vector.y()
        handle_point_estimate.point.z = handle_estimate.vector.z()

        handle_point_response = self.door_info.call("write_marker", handle_point_estimate)
        
        #this info allows you to check that the location of the handle is correct and so you don't need to tstay close to the robot in case location is bad
        #for example, in the lab door, the z point is around 1.05
        rospy.loginfo("handle_point_response:")
        rospy.loginfo(handle_point_response.point_out)
        
        if handle_point_response.point_out.point.x == 0 and handle_point_response.point_out.point.y == 0 and handle_point_response.point_out.point.z == 0:
            return 'fail'
        
        rospy.sleep(1)

        self.door.updateHandlePose(handle_point_response.point_out)
        return 'updated'

class goIFOhandle(smach.State):
    def __init__(self, door, robot):
        smach.State.__init__(self, outcomes=['IFO_handle','fail'], input_keys=['side_of_door'])
        self.door = door
        self.robot = robot
        self.pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=2)
        self.pub_twist = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userdata):
        """ 
        This function uses the local planner to go to the position IFO handle
        """
        #according to the SOD we are working on, we are going to get and publish a different poseStamped
        if userdata.side_of_door == 'face':
            frameIFOhandle_door = self.door.getFrameIFOhandle_face()

        else:
            frameIFOhandle_door = self.door.getFrameIFOhandle_behind()

        #convert this frame in the map frame
        frameIFOhandle_map = self.robot.tf_buffer.transform(frameIFOhandle_door, "map", rospy.Duration(1.0))

        #get the vector and the quaternion
        kdl_vector = frameIFOhandle_map.frame.p
        kdl_quaternion = frameIFOhandle_map.frame.M.GetQuaternion()

        #create the pose stamped message with the position IFO the door
        if not simulation:
            pose_stamped_IFOhandle = create_pose_stamped(kdl_vector[0], kdl_vector[1], kdl_vector[2], kdl_quaternion[0], kdl_quaternion[1], kdl_quaternion[2], kdl_quaternion[3])

        else:
            #SIMULATION
            if userdata.side_of_door == 'face':
                pose_stamped_IFOhandle = create_pose_stamped(6.55, 0.381, 0, 0, 0, 0.001, 0.99)
            else:
                pose_stamped_IFOhandle = create_pose_stamped(kdl_vector[0], kdl_vector[1] + 0.1, kdl_vector[2], kdl_quaternion[0], kdl_quaternion[1], kdl_quaternion[2], kdl_quaternion[3])

        #publication
        rospy.sleep(0.5)
        self.pub.publish(pose_stamped_IFOhandle)

        position_achieve()
        #check the rotation
        # end_rot = False
        # while not end_rot:
        #     if userdata.side_of_door == 'face':
        #         frameIFOhandle_door = self.door.getFrameIFOhandle_face()

        #     else:
        #         frameIFOhandle_door = self.door.getFrameIFOhandle_behind()
            
        #     frameIFOhandle_robot = self.robot.tf_buffer.transform(frameIFOhandle_door, self.robot.base_link_frame, rospy.Duration(1.0))
        #     rot_y_expected = frameIFOhandle_robot.frame.M.GetRot()[2]
        #     if rot_y_expected > 0.05:
        #         twist_message = create_twist(0,0,0.05,0,0,0)
        #         self.pub_twist.publish(twist_message)
        #     elif rot_y_expected < -0.05:
        #         twist_message = create_twist(0,0,-0.05,0,0,0)
        #         self.pub_twist.publish(twist_message)
        #     else:
        #         end_rot = True

        # #check y
        # end_mov = False
        # while not end_mov:
        #     if userdata.side_of_door == 'face':
        #         frameIFOhandle_door = self.door.getFrameIFOhandle_face()

        #     else:
        #         frameIFOhandle_door = self.door.getFrameIFOhandle_behind()
                
        #     frameIFOhandle_robot = self.robot.tf_buffer.transform(frameIFOhandle_door, self.robot.base_link_frame, rospy.Duration(1.0))
        #     y_expected = frameIFOhandle_robot.frame.p[1]
            
        #     if y_expected > 0.1:
        #         twist_message = create_twist(0,0,0,0,0.05,0)
        #         self.pub_twist.publish(twist_message)
        #     elif y_expected < -0.1:
        #         twist_message = create_twist(0,0,0,0,-0.05,0)
        #         self.pub_twist.publish(twist_message)
        #     else:
        #         end_mov = True
            
        rospy.sleep(2)
        return 'IFO_handle'

class graspeHandle(smach.State):
    def __init__(self, robot, arm, door):
        self.robot = robot
        self.door = door
        self.arm = arm
        self.rate = rospy.Rate(1)
        smach.State.__init__(self, outcomes=['good_position_of_arm', 'fail'], input_keys=['side_of_door'])

    def execute(self, userdata):
        """ 
        This function is grasping the handle by using the arm and the send_goal function.
        """
        self.arm.gripper.send_goal("open")

        handle_position = self.door.getHandlePose() #get the pose (point) of the handle

        #we have to transform this vector into a frameStamped to be able to use some functions
        # we have to first create a kdl vector that will allow to create a frameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = kdl.Vector(handle_position.point.x, handle_position.point.y, handle_position.point.z)
                
        kdl_frame = kdl.Frame(kdl_rotation, kdl_vector) #frame kdl
        goal_handle = FrameStamped(kdl_frame,rospy.Time.now(), frame_id = self.robot.base_link_frame)
        
        goal_handle.frame.p[0] -= 0.05  #change x here, if we don't the robot is going to be too close to the door
        goal_handle.frame.M = kdl.Rotation.RPY(-1.57, 0.0, 0.0) #rotation of the gripper in order to be able to grasp the handle

        #move the arm and wait for the result
        result = self.arm.send_goal(goal_handle,timeout=10.0)
        self.arm.wait_for_motion_done()
                    
        if result:
            return 'good_position_of_arm'
        else:
            rospy.loginfo('grasping handle is not a success')
            return 'fail'

class closeGripper(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['gripperClose', 'fail'])

    def execute(self, userdata):
          self.arm.gripper.send_goal("close")
          return 'gripperClose'

class unlatchHandle(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['handleIsUnlatched_push', 'handleIsUnlatched_pull', 'fail'], input_keys=['side_of_door'])
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        """ 
        This function is unlatching the handle.
        It first gets the current poisition of the joint and send back the same position after changing only the position of 'arm lift joint' to go down and unlatch the handle.
        If we are behind the door, we also rotate the wrist because it seems the robot has difficulty if we don't (too much force apply on this side)
        """
        #get the position of the join
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']
        
        joint1_new_position = joint1 - 0.055 #5.5 cm
        
        if userdata.side_of_door == 'behind':
            joint5_new_position = joint5 + 0.25
        else:
            joint5_new_position = joint5
            
            
        list_trajectory = [joint1, joint2, joint3, joint4, joint5_new_position ]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        rospy.sleep(0.3)
        
        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5_new_position ]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        rospy.sleep(1)
        
        # #second part, rotate the gripper
        # if userdata.side_of_door == 'behind':
        #     joint5_new_position = joint5 + 0.2
        # else:
        #     joint5_new_position = joint5 - 0.2
            
        # list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5_new_position]  
        # self.arm._arm._send_joint_trajectory([list_trajectory])
        # self.arm.wait_for_motion_done()
        # self.rate.sleep()
        
        # #third part : go down 5cm
        # joint1_new_position = joint1 - 0.08
        # list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5_new_position]  
        # self.arm._arm._send_joint_trajectory([list_trajectory])
        # self.arm.wait_for_motion_done()
        # self.rate.sleep()
          
        if userdata.side_of_door == 'face':
            self.rate.sleep()
            return 'handleIsUnlatched_push'

        else:
            self.rate.sleep()
            return 'handleIsUnlatched_pull'

class pushDoorUnlatched(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['doorIsPushed', 'fail'])
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        """ 
        This function is pushing the door when it is unlatched.
        It is using the command force drive
        """
        self.rate.sleep()
        #get some frame
        #these are frame of the door according to the robot point of view (TT transform)
        door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
        door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right

        #get the coordinate
        x1 = door_frame_robot_left.vector.x()
        y1 = door_frame_robot_left.vector.y()
        x2 = door_frame_robot_right.vector.x()
        y2 = door_frame_robot_right.vector.y()

        #mean of right and left
        x = (x1 + x2) / 2.0
        y = (y1 + y2) / 2.0

        self.robot.base.force_drive(0.1, y / (x / 0.1), 0, x ) 

        self.rate.sleep()

        return 'doorIsPushed'

class pullDoorUnlatch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_pulled', 'fail'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
    
    def execute(self, userdata):
        """ 
        This function is pulling the door once it has been unlatched.
        """
        msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
        segments = msg_line_segments.line_segments # extract the segments

        #get the useful segment
        for segment in segments:
            if segment.start[1] < 0 and segment.end[1] > 0:
                angle = segment.angle
        if angle < 0:
            beta = math.pi/2 + angle
            x = - math.sin(beta)/20
            y = - math.cos(beta)/20
        else:
            beta = angle

            x = - math.cos(beta) / 20
            y = - math.sin(beta) / 20
        twist_msg = create_twist(0,0,0,x,y,0)
        self.pub.publish(twist_msg)
        rospy.sleep(1)
        
        return 'door_pulled'
               
class getDataDoorClose(smach.State):
    def __init__(self, door, robot):
        self.door = door
        self.robot = robot
        smach.State.__init__(self, outcomes=['dataIsGet', 'fail'], output_keys=['angle', 'distance_middle', 'y_limit'])

    def execute(self, userdata):
        """
        The goal of this class is to get some useful data we need to pull the door from the state close.
        These data, are 
            - distance middle that is the distance to the middle of the door. The middle is the mean of start and and
            - angle that is the angle according the door segment
        All the data we need are in the line extraction segment message: topic: /line_segments
        
        We also calculate y_limit. It is the point where the robot should stop pulling the door.
        """
        rospy.sleep(3)
        msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
        segments = msg_line_segments.line_segments # extract the segments

        #get the useful segment
        for segment in segments:
            if segment.start[1] < 0 and segment.end[1] > 0:
                start = segment.start
                end = segment.end
                angle = segment.angle

        userdata.angle = angle
        middle_point = [(start[0]+end[0])/2, (start[1]+end[1])/2] #calculate the distance between the robot and the start of the segment and the robot and the end of the segment
        distance_middle = math.sqrt(middle_point[0]**2 + middle_point[1]**2)
        userdata.distance_middle = distance_middle
        rospy.loginfo('distance_middle ' + str(distance_middle) + ' angle: ' + str(angle))
        #get y of the frame point
        # plan : there are one VV of each side of the door. We are going to choose the one that is the more far away from the handle.
        
        door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], "door_inside", rospy.Duration(1.0))
        door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], "door_inside", rospy.Duration(1.0))
        handle_volume = self.robot.tf_buffer.transform(self.door.handle_behind_pose, "door_inside", rospy.Duration(1.0))
        
        d_left = math.sqrt( (door_frame_robot_left.vector.x() - handle_volume.vector.x())**2 + (door_frame_robot_left.vector.y() - handle_volume.vector.y())**2 + (door_frame_robot_left.vector.z() - handle_volume.vector.z())**2 )
        d_right = math.sqrt( (door_frame_robot_right.vector.x() - handle_volume.vector.x())**2 + (door_frame_robot_right.vector.y() - handle_volume.vector.y())**2 + (door_frame_robot_right.vector.z() - handle_volume.vector.z())**2 )
        
        if d_right > d_left:
            userdata.y_limit = door_frame_robot_right.vector.y()
        else :
            userdata.y_limit = door_frame_robot_left.vector.y()
        
        return 'dataIsGet'

class pullDoorClose(smach.State):
    def __init__(self, robot):
        self.robot = robot
        smach.State.__init__(self, outcomes=['door_pulled', 'ended', 'fail'], input_keys=['angle', 'y_limit'], output_keys=['end_mov', 'counter_distance', 'counter_rot'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        self.counter = 0

    def execute(self, userdata):
        """ 
        This function pulls the door.  The direction of pulling is calculate with the angle of the robot according to the door.
        It also checks at the if the door is open enough (it means that we cross y_limit). 
        """
        self.counter += 1
        #rospy.loginfo('counter worth: ' + str(self.counter))
        userdata.end_mov = False
        userdata.counter_distance = 0
        userdata.counter_rot = 0
        
        if userdata.angle < 0:
            beta = math.pi/2 + userdata.angle
            x = - math.sin(beta)/20
            y = - math.cos(beta)/20
        else:
            beta = userdata.angle

            x = - math.cos(beta) / 20
            y = - math.sin(beta) / 20

        twist_msg = create_twist(0,0,0,x,y,0)

        self.pub.publish(twist_msg)
        rospy.sleep(0.5)
        self.pub.publish(twist_msg)
        rospy.sleep(1.5)
        
        #check if the position of the robot allows us to consider the door open
        #get the frame position and especially the y value in the door frame
        frame_current_pos_map = self.robot.base.get_location().frame #TODO it is already a frame stamped
        FrameStamped_current_pos_map = FrameStamped(frame_current_pos_map, rospy.Time.now(), frame_id = "map")
        frameStamped_current_pos_door = self.robot.tf_buffer.transform(FrameStamped_current_pos_map, "door_inside", rospy.Duration(1.0))
        #extract y
        y = frameStamped_current_pos_door.frame.p.y()

        #compare
        if userdata.y_limit < 0:
            if y < userdata.y_limit - 0.2:
                return 'ended'
        else:
            if y > userdata.y_limit + 0.2:
                return 'ended'
        
        return 'door_pulled'

class positionAngleRobotDoorClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['angle_achieve', 'position_achieve', 'fail'], input_keys=['angle', 'end_mov', 'counter_rot'], output_keys=['counter_rot'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        

    def execute(self, userdata):
        """ 
        This fucntion is made to rotate the robot between dirrefernt pulling of the door.
        The goal is to rotate until we have the same angle as we had befor pulling the door.
        We can't rotate more than 3 times for one pull of the door, this is why we are using userdata.counter_rot
        """
        end_rot = False
        userdata.counter_rot +=1
        
        if userdata.counter_rot >= 3:
            end_rot = True
                    
        while not end_rot:
            
            msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
            segments = msg_line_segments.line_segments # extract the segments
            
            #get the useful segment
            for segment in segments:
                if segment.start[1] < 0 and segment.end[1] > 0:
                    rot = segment.angle #get the angle

            if abs(abs(rot) - abs(userdata.angle)) > 0.1:
                if rot > userdata.angle:
                    if simulation:
                        twist_msg = create_twist(0,0,0.05,0,0,0)
                    else:
                        twist_msg = create_twist(0,0,0.05,0,0,0)  
                else:
                    if simulation:
                        twist_msg = create_twist(0,0,-0.05,0,0,0)
                    else:
                        twist_msg = create_twist(0,0,-0.05,0,0,0)
                        
                self.pub.publish(twist_msg)
                
                if simulation:
                    rospy.sleep(1)
                else:
                    rospy.sleep(0.8)
                if userdata.counter_rot >= 3:
                    end_rot = True
            else:
                end_rot = True  

        if userdata.end_mov == True:
            return 'position_achieve'
        else:
            return 'angle_achieve'
        
class positionDistanceRobotDoorClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['distance_achieve', 'fail'], input_keys=['angle', 'distance_middle', 'counter_distance'], output_keys=['end_mov', 'counter_distance'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userdata):
        """ 
        This fucntion is made to move the robot between different pulling of the door.
        The goal is to move until we have the same distance as we had befor pulling the door.
        Every time we move the robot, we also rotate after.
        We can't move more than 3 times for one pull of the door, this is why we are using userdata.counter_distance
        """
        userdata.counter_distance += 1
        msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
        segments = msg_line_segments.line_segments # extract the segments

        #get the useful segment
        for segment in segments:
            if segment.start[1] < 0 and segment.end[1] > 0:
                
                #get start and end vector
                angle = segment.angle
                start = segment.start
                end = segment.end

        #calculate distance
        middle_point = [(start[0]+end[0])/2, (start[1]+end[1])/2]
        distance_middle = math.sqrt(middle_point[0]**2 + middle_point[1]**2)
            
        #compare the distance
        if abs(abs(userdata.distance_middle) - abs(distance_middle)) > 0.03: #3 cm
            if userdata.angle > 0:
                beta = math.pi/2 - angle
                x = math.cos(beta) / 30
                y = - abs(math.sin(beta)) / 30
                twist_msg = create_twist(1,0,0,x,y,0)

            else:
                beta = math.pi/2 - angle
                x = - abs(math.cos(beta)) / 30 
                y = - abs(math.sin(beta)) / 30
                twist_msg = create_twist(1,0,0,x,y,0)
                    
            self.pub.publish(twist_msg)
            rospy.sleep(0.8)
            
            if userdata.counter_distance == 3:
                userdata.end_mov = True
        else:
            userdata.end_mov = True

        return 'distance_achieve'

class goUpHandle(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['handleIsUp', 'fail'], input_keys=['side_of_door'])

    def execute(self, userdata):
        """ 
        This function is going up with the handle.
        It first gets the current poisition of the joint and send back the same position after changing only the position of 'arm lift joint' to go up with the handle.
        If we are behind the door, we also rotate the wrist because we are doing it when we unlatch the handle
        """
        #get the position of the join
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']

        joint1_new_position = joint1 + 0.055
        if userdata.side_of_door == 'behind':
            joint5_new_position = joint5 - 0.25
        else:
            joint5_new_position = joint5
        
        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5_new_position]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        rospy.sleep(2)

        return 'handleIsUp'

class openGripper(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['gripperOpen', 'fail'])

    def execute(self, userdata):
          self.arm.gripper.send_goal("open")
          rospy.sleep(1)
          return 'gripperOpen'

class robotBackInitialPosition(smach.State):
    def __init__(self, arm):
        smach.State.__init__(self, outcomes=['initialPosition','fail'])
        self.arm = arm

    def execute(self, userdata):
        """ 
        This function put the robot back in its initial position
        """
        #this is the joints position for reset in the robot.
        joint1, joint2, joint3, joint4, joint5 = 0.01, 0.0, -1.57, -1.57, 0.0
        list_trajectory = [joint1, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        #rospy.sleep(8)

        return 'initialPosition'

class goAwayFromObstacle(smach.State):
    def __init__(self, robot):
        self.robot = robot
        smach.State.__init__(self, outcomes=['away', 'fail'])
        self.away = False
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute (self, userdata):
        """ 
        This function is called to go away from obstacle in order to be able to use the planner again. 
        It is base on the laser data.
        """
        if simulation:
            dmin = 0.2 # minimum away distance from obstacle
            dget = 0.1
        else:
            dmin = 0.4 # minimum away distance from obstacle
            dget = 0.1

        while dget < dmin:
            dget, angle_min = self.laserData_callback()
            x_direction = - math.cos(angle_min)
            y_direction = - math.sin(angle_min)
            twist_message = create_twist(0,0,0, x_direction/15, y_direction/15, 0) #use of /15 because we want to move slowly and to update often the minimum distance
            self.pub.publish(twist_message)
            rospy.sleep(0.5)

        return 'away'

    def laserData_callback(self):
        """ 
        this function has been made to deal with the laser data and sending back only the data we need
        """
        msg = rospy.wait_for_message("/hero/base_laser/scan", LaserScan)

        count = 1
        minimum = msg.ranges[0]
        minimum_count = 0

        length = len(msg.ranges)

        while not rospy.is_shutdown() and count < length: # length -1 maybe
            if minimum > msg.ranges[count]: #check all the ranges list
                minimum = msg.ranges[count]
                minimum_count = count

            count +=1

        #get the angle
        min_angle = msg.angle_min + msg.angle_increment * minimum_count
        return minimum, min_angle

class pushDoorOpen(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        smach.State.__init__(self, outcomes=['doorIsPushed', 'fail'])


    def execute(self, userdata):
        """ 
        This function is made to push the door that is consider beiing open
        It just go with the robot in the middle of door_frameleft and door_frame_right
        The function is dangerous is there is an obstacle because it doesn't check anything.
        
        first part, getting the position the robot has to be (especially the rotation), it depends on which SOD we are
        the expected rotation is the same as beiing IFO door because we want the robot to be straight IFO the door
        Then we also check the y and the the x
        """
        end = False # is false until we are in the point




        while not end:
            frameIFOdoor_door = self.door.getFrameIFOdoor_face()

            #get the frame of the edges of the door
            #these are frame of the door according to the robot point of view (TT transform)
            door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
            door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right

            #get the coordinate
            x1 = door_frame_robot_left.vector.x()
            y1 = door_frame_robot_left.vector.y()
            x2 = door_frame_robot_right.vector.x()
            y2 = door_frame_robot_right.vector.y()

            #mean of right and left
            x = (x1 + x2) / 2.0

            y = (y1 + y2) / 2.0

            #get rot_y expected in robot frame
            frameIFOdoor_robot = self.robot.tf_buffer.transform(frameIFOdoor_door, self.robot.base_link_frame, rospy.Duration(1.0))
            rot_y_expected = frameIFOdoor_robot.frame.M.GetRot()[2] # get the rot_y we want to have

            if rot_y_expected > 0.1:
                twist_message = create_twist(0,0,0.05,0,0,0)
                self.pub.publish(twist_message)
            elif rot_y_expected < -0.1:
                twist_message = create_twist(0,0,-0.05,0,0,0)
                self.pub.publish(twist_message)
            elif y < -0.1:
                twist_message = create_twist(0,0,0,0,-0.05,0)
                self.pub.publish(twist_message)
            elif y > 0.1:
                twist_message = create_twist(0,0,0,0,+0.05,0)
                self.pub.publish(twist_message)
            elif x > 0.1:
                twist_message = create_twist(0,0,0,0.05,0,0)
                self.pub.publish(twist_message)
            elif x < -0.1:
                twist_message = create_twist(0,0,0,-0.05,0,0)
                self.pub.publish(twist_message)
            else:
                end = True

            rospy.sleep(0.1)

        return 'doorIsPushed'

class crossDoor(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        smach.State.__init__(self, outcomes=['doorIsCrossed', 'fail'], input_keys=['side_of_door'])


    def execute(self, userdata):
        """ 
        This function is the same that pushDooropen, but the position at the end is a bit different 
        """
        end = False # is false until we are in the point
        if userdata.side_of_door == 'face':
            frameIFOdoor_door = self.door.getFrameIFOdoor_face()

        else:
            frameIFOdoor_door = self.door.getFrameIFOdoor_behind()

        while not end:
            if userdata.side_of_door == 'face':
                frameIFOdoor_door = self.door.getFrameIFOdoor_face()
            else:
                frameIFOdoor_door = self.door.getFrameIFOdoor_behind()

            #get the frame of the edges of the door
            #these are frame of the door according to the robot point of view (TT transform)
            door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
            door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right

            #get the coordinate
            x1 = door_frame_robot_left.vector.x()
            y1 = door_frame_robot_left.vector.y()
            x2 = door_frame_robot_right.vector.x()
            y2 = door_frame_robot_right.vector.y()

            #mean of right and left
            x = (x1 + x2) / 2.0
            if userdata.side_of_door == 'face':
                x = x + 1.5
            else:
                x = x + 1.5

            y = (y1 + y2) / 2.0

            if not simulation:
                if userdata.side_of_door == 'face':
                    y = y - 0.1
                
            else:
                if userdata.side_of_door == 'face':
                    y = y - 0.1
                else:
                    y = y - 0.1

            #get rot_y expected in robot frame
            frameIFOdoor_robot = self.robot.tf_buffer.transform(frameIFOdoor_door, self.robot.base_link_frame, rospy.Duration(1.0))
            rot_y_expected = frameIFOdoor_robot.frame.M.GetRot()[2] # get the rot_y we want to have

            if rot_y_expected > 0.1:
                twist_message = create_twist(0,0,0.05,0,0,0)
                self.pub.publish(twist_message)
            elif rot_y_expected < -0.1:
                twist_message = create_twist(0,0,-0.05,0,0,0)
                self.pub.publish(twist_message)
            elif y < -0.1:
                twist_message = create_twist(0,0,0,0,-0.05,0)
                self.pub.publish(twist_message)
            elif y > 0.1:
                twist_message = create_twist(0,0,0,0,+0.05,0)
                self.pub.publish(twist_message)
            elif x > 0.1:
                twist_message = create_twist(0,0,0,0.05,0,0)
                self.pub.publish(twist_message)
            elif x < -0.1:
                twist_message = create_twist(0,0,0,-0.05,0,0)
                self.pub.publish(twist_message)
            else:
                end = True

            rospy.sleep(0.1)

        return 'doorIsCrossed'

class moveTreshold(smach.State):
    """ 
    This function is called to go to the threshold of the door. The treshold is the point where we can detect and grasp the handle.
    It is based on the radar data
    """
    def __init__(self, robot):
        self.robot = robot
        smach.State.__init__(self, outcomes=['goodPosition','fail'], input_keys=['side_of_door'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userata):
        if simulation:
            #simulation
            if userata.side_of_door == 'face':
                treshold = 0.40
            else:
                treshold = 0.40
            d = 0.7
        else:
            #reality
            treshold = 0.40
            d = 0.7

        x = 0.05

        while d > treshold:
            d = self.laserData_callback()
            twist_message = create_twist(0, 0, 0, x, 0, 0)
            self.pub.publish(twist_message)
            rospy.sleep(0.5)
        
        rospy.sleep(8)
        return 'goodPosition'

    def laserData_callback(self):
        msg_laser = rospy.wait_for_message("/hero/base_laser/scan", LaserScan)
        position_angle_zero = int(msg_laser.angle_min/msg_laser.angle_increment)
        distance = msg_laser.ranges[position_angle_zero]

        return distance

class moveDoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['face', 'behind'],  input_keys=['side_of_door'])

    def execute(self, userdata):
        """ 
        Just an intermediate function to choose between the two function we can use to open a door that is intermediate
        """
        if userdata.side_of_door == 'face':
            return 'face'

        else:
            return 'behind'

class positionRobot(smach.State):
    def __init__(self, door, robot, arm):
        self.door = door
        self.robot = robot
        self.arm = arm
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        smach.State.__init__(self, outcomes=['position_achieve', 'fail'])

    def execute(self, userdata):
        """ 
        this function is called to open a door that is in state intermediate for the side behind. 
        It give the robot the right rotation and then extend the arm
        """
        end_rot = False 
        while not end_rot:
            #get the frame
            framePosition = self.door.getRotationFrame()[0]
            #convert in robot frame
            frame_pos = self.robot.tf_buffer.transform(framePosition, self.robot.base_link_frame, rospy.Duration(1.0))
            #extract rotation
            rot_y_expected = frame_pos.frame.M.GetRot()[2]
            
            
            if rot_y_expected > 0.1:
                twist_message = create_twist(0,0,0.05,0,0,0)
                self.pub.publish(twist_message)
            elif rot_y_expected < -0.1:
                twist_message = create_twist(0,0,-0.05,0,0,0)
                self.pub.publish(twist_message)
            else:
                end_rot = True
        
        #now straight the arm 
        #joint1, joint2, joint3, joint4, joint5 = 0.1, -0.8, 0, -0.8, 0
        joint1, joint2, joint3, joint4, joint5 = 0.1, -1.6, 0.0, 0, 0
        list_trajectory = [joint1, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()

        rospy.sleep(1)
        return 'position_achieve'

class rotate(smach.State):
    def __init__(self, door, robot):
        self.door = door
        self.robot = robot
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        smach.State.__init__(self, outcomes=['rotation_achieve', 'fail'])
        
    def execute(self, userdata):
        """ 
        this function is made to terminate to open the door that is in state intermediate and on the side behind. 
        It rotate with the arm axtended in order top fully open the door.
        """
        end_rot = False 
        while not end_rot:
            #get the frame
            framePosition = self.door.getRotationFrame()[1]

            #convert in robot frame
            frame_pos = self.robot.tf_buffer.transform(framePosition, self.robot.base_link_frame, rospy.Duration(1.0))
                        
            #extract rotation
            rot_y_expected = frame_pos.frame.M.GetRot()[2]       
            rospy.loginfo("rot_y_expected: " + str(rot_y_expected))
            if rot_y_expected > 0.1:
                twist_message = create_twist(0,0,0.05,0,0,0)
                self.pub.publish(twist_message)
            elif rot_y_expected < -0.1:
                twist_message = create_twist(0,0,-0.05,0,0,0)
                self.pub.publish(twist_message)
            else:
                end_rot = True
        
        return 'rotation_achieve'       
                

def sm_pull_door_close(robot, arm, my_door):
    sm_pull_door_close = smach.StateMachine(outcomes=['doorIsPulled', 'fail'], input_keys=['side_of_door'])
    sm_pull_door_close.userdata.angle = None
    sm_pull_door_close.userdata.end_mov = None
    sm_pull_door_close.userdata.distance_middle = None
    sm_pull_door_close.userdata.counter_distance = None
    sm_pull_door_close.y_limit = None
    
    with sm_pull_door_close:
        smach.StateMachine.add('pull_door_unlatch', pullDoorUnlatch(), transitions={'door_pulled' : 'go_up_handle', 'fail' : 'fail'})
        smach.StateMachine.add('go_up_handle', goUpHandle(arm), transitions={'handleIsUp' : 'get_data', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('get_data', getDataDoorClose(my_door, robot), transitions={'dataIsGet' : 'pull_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'y_limit' : 'y_limit'})
        smach.StateMachine.add('pull_door_close', pullDoorClose(robot), transitions={'door_pulled' : 'position_angle_robot_door_close', 'ended' : 'doorIsPulled', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'end_mov' : 'end_mov', 'counter_distance' : 'counter_distance', 'y_limit' : 'y_limit', 'counter_rot' : 'counter_rot'})
        smach.StateMachine.add('position_angle_robot_door_close', positionAngleRobotDoorClose(), transitions={'angle_achieve' : 'position_distance_robot_door_close', 'position_achieve' : 'pull_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'end_mov' : 'end_mov', 'counter_rot' : 'counter_rot'})
        smach.StateMachine.add('position_distance_robot_door_close', positionDistanceRobotDoorClose(), transitions={'distance_achieve' : 'position_angle_robot_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'end_mov' : 'end_mov', 'distance_middle' : 'distance_middle',  'counter_distance' : 'counter_distance'})
        
    return sm_pull_door_close

def sm_grasp_handle(robot, arm, my_door):
    #smach state machine to grasp the handle
    sm_grasp_handle = smach.StateMachine(outcomes=['handleIsGrasped', 'fail'],input_keys=['side_of_door'], output_keys = ['side_of_door'])
    with sm_grasp_handle:
        #smach.StateMachine.add('navigateToGrab', NavigateToGrasp(robot, arm, my_handle), transitions={'arrived' : 'GraspeHandle', 'unreachable' : 'fail', 'goal_not_defined' : 'fail'})
        smach.StateMachine.add('GraspeHandle', graspeHandle(robot, arm, my_door), transitions={'good_position_of_arm' : 'closeGripper', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('closeGripper', closeGripper(arm), transitions={'gripperClose' : 'handleIsGrasped', 'fail' : 'fail'})

    return sm_grasp_handle

def sm_open_door_close(robot, arm, my_door):
    #get sm_grasp_handle and sm pull door close
    sm_grasp_handle_ = sm_grasp_handle(robot, arm, my_door)
    sm_pull_door_close_ = sm_pull_door_close(robot, arm, my_door)
    #get sm
    sm_open_door_close = smach.StateMachine(outcomes=['doorIsNotClose', 'fail'], input_keys = ['side_of_door'], output_keys = ['side_of_door'])
    with sm_open_door_close:
        smach.StateMachine.add('goIFOhandle', goIFOhandle(my_door, robot), transitions={'IFO_handle' : 'goMinimumPosition', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('updateHandleLocation', updateHandleLocationFromServiceServer(robot, my_door), transitions={'updated' : 'graspingHandle', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('goMinimumPosition', moveTreshold(robot), transitions={'goodPosition' : 'updateHandleLocation', 'fail' : 'fail'})
        smach.StateMachine.add('graspingHandle', sm_grasp_handle_, transitions={'handleIsGrasped' : 'unlatchHandle', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'}) #TODO
        smach.StateMachine.add('unlatchHandle', unlatchHandle(arm), transitions={'handleIsUnlatched_push' : 'pushDoorUnlatched','handleIsUnlatched_pull' : 'pullDoorUnlatched', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('pushDoorUnlatched',pushDoorUnlatched(robot, my_door), transitions={'doorIsPushed' :'goUpHandle', 'fail' : 'fail'} )
        smach.StateMachine.add('pullDoorUnlatched',sm_pull_door_close_, transitions={'doorIsPulled' :'openGripper', 'fail' : 'fail'} )
        smach.StateMachine.add('goUpHandle', goUpHandle(arm), transitions={'handleIsUp' : 'openGripper', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('openGripper',openGripper(arm), transitions={'gripperOpen' :'doorIsNotClose', 'fail' : 'fail'} )
        
    return sm_open_door_close

def sm_open_door_open(robot, arm, my_door):
    sm_open_door_open = smach.StateMachine(outcomes=['doorIsOpen', 'fail', 'go_IFO_door'], input_keys = ['side_of_door'])
    with sm_open_door_open:
        smach.StateMachine.add('move_door_open', moveDoorOpen(), transitions={'face' : 'push_door_open', 'behind' : 'go_IFO_door'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('push_door_open', pushDoorOpen(robot, my_door), transitions={'doorIsPushed' : 'doorIsOpen', 'fail' : "fail"})
        smach.StateMachine.add('go_IFO_door', goIFOdoor(my_door, robot), transitions={'IFO_door' : 'position_robot', 'fail' : 'fail'})
        smach.StateMachine.add('position_robot', positionRobot(my_door, robot, arm), transitions={'position_achieve' : 'rotate', 'fail' : 'fail'})
        smach.StateMachine.add('rotate', rotate(my_door, robot), transitions={'rotation_achieve' : 'robot_back_pos_init', 'fail' : 'fail'})
        smach.StateMachine.add('robot_back_pos_init', robotBackInitialPosition(arm), transitions={'initialPosition' : 'doorIsOpen', 'fail' : 'fail'})
    
    return sm_open_door_open

def sm_cross_door(robot, arm, my_door):
    #get the opening of the door if it intermediate or close
    sm_open_door_close_= sm_open_door_close(robot, arm, my_door)
    sm_open_door_open_ = sm_open_door_open(robot, arm, my_door)

    sm_cross_door =  smach.StateMachine(outcomes=['doorIsCrossed', 'fail'])
    sm_cross_door.userdata.side_of_door = 'Unknown'

    with sm_cross_door:
        smach.StateMachine.add('getSOD', getSOD(robot, my_door), transitions={'find_SOD' : 'goIFOdoor', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('goIFOdoor', goIFOdoor(my_door, robot), transitions={'IFO_door' : 'update_door_state', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('update_door_state', updateDoorState(robot), transitions={'close' : 'open_door_closed', 'intermediate' : 'open_door_open', 'open' : 'cross_door', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('open_door_closed', sm_open_door_close_, transitions={'doorIsNotClose' :'robot_back_posInit', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('open_door_open', sm_open_door_open_, transitions={'doorIsOpen' : 'cross_door','go_IFO_door' : 'goIFOdoor' ,'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('robot_back_posInit', robotBackInitialPosition(arm), transitions={'initialPosition' : 'move_away_from_obstacle', 'fail' : 'fail'})
        smach.StateMachine.add('cross_door', crossDoor(robot, my_door), transitions={'doorIsCrossed' : 'doorIsCrossed', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('move_away_from_obstacle', goAwayFromObstacle(robot), transitions={'away' : 'open_door_open', 'fail' : 'fail'})
    return sm_cross_door

def detection_handle(robot, my_door, arm):
    sm_detect_handle = smach.StateMachine(outcomes=['handleIsDetected', 'fail'])
    sm_detect_handle.userdata.side_of_door = 'Unknown'

    with sm_detect_handle:
        smach.StateMachine.add('getSOD', getSOD(robot, my_door), transitions={'find_SOD' : 'goIFOhandle', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('goIFOhandle', goIFOhandle(my_door, robot), transitions={'IFO_handle' : 'goMinimumPosition', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('goMinimumPosition', moveTreshold(robot), transitions={'goodPosition' : 'updateHandleLocation', 'fail' : 'fail'})
        smach.StateMachine.add('updateHandleLocation', updateHandleLocationFromServiceServer(robot, my_door), transitions={'updated' : 'handleIsDetected', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
    return sm_detect_handle

def main():
    rospy.init_node('open_door_smach_node',anonymous=True)

    #init of devices
    robot = get_robot("hero")
    arm = robot.get_arm(force_sensor_required=True)
    door = robot.ed.get_entity(uuid="door_inside")
    my_door = Door(door)

    #smach test detection handle
    # sm_detection_handle = detection_handle(robot, my_door, arm)
    # outcome = sm_detection_handle.execute()

    #smach state machine main
    sm_main = sm_cross_door(robot, arm, my_door)
    outcome = sm_main.execute()

if __name__ == '__main__':
    main()



