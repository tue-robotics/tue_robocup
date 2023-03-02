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

#fct that has to be destroyed in the futur.
#it currently allows convertion beteween frames and to find interessant position in door_inside frame when I want it
def convert_frame(robot, a,z,e,r,t,y,u):
    Head = Header(seq = 1, stamp = rospy.Time.now(), frame_id = "map")
    point = Point(a,z,e)
    quaternion = Quaternion(r,t,y,u)
    pose = Pose(point, quaternion)
    pose_message = PoseStamped(Head, pose)
    pose_message_convert_in_frame_map = tf2_kdl.from_msg_frame(pose_message)
    pose_message_convert_in_frame_door = robot.tf_buffer.transform(pose_message_convert_in_frame_map, "door_inside", rospy.Duration(1.0))
    rospy.loginfo("position frame door")
    rospy.loginfo(pose_message_convert_in_frame_door)

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
        kdl_vector = kdl.Vector(1.32, 0.05, 0)
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
        kdl_vector = kdl.Vector(0.6248, 0.202, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,3.1)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

    def getFrameIFOdoor_behind(self):
        """
        This fonction is hard coded according to the door.
        It rpresents ths position the robot has to be in the fonction goIFOdoor according to the door frame.
        This position has to be converted to the frame map before being send to the global planner.
        """
        kdl_vector = kdl.Vector(-1.32, 0.05, 0)
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
        kdl_vector = kdl.Vector(-0.6248, 0.202, 0)
        kdl_rot = kdl.Rotation.RPY(0,0,0.0444)
        kdl_frame = kdl.Frame(kdl_rot, kdl_vector)
        kdl_frame_stamped = FrameStamped(kdl_frame, rospy.Time.now(), frame_id = "door_inside")

        return kdl_frame_stamped

class getSOD(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['find_SOD', 'fail'], output_keys=['side_of_door'])

    def execute(self, userdata):
        """
        When this function is execute, it finds the SOD that must be open by the robot.
        Nevertheless the SOD is find thanks to the current position of the robot (the closest SOD is the SOD to open).
        It means that this funtion must be called once the robot is close to the door.
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
        #onvert_frame(self.robot, 8.1, 0.37, 0, 0, 0, -0.99, 0.022)
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
    def __init__(self, robot, door):
        smach.State.__init__(self, outcomes=['close','intermediate','open', 'fail'], input_keys=['side_of_door'])
        self.rate = rospy.Rate(0.3)
        self.robot = robot
        self.door = door

    def execute(self, userdata):
        """
        This function check the state of the door.
        It first finds the distance of the door.
        Then, if the distance is big enough, the door is considered open
        if the distance is not big enough, we are using the laserdata in the following way:
            We check the distance a bit on the right and a bit on the left from the middle of the door.
            If those distance are similar, it means the door is close.
            If those distance aren't, it means the door is intermediate

        Before doing all of this, we have to check the rotation of the robot, to be sure to use the data straight to the door, even if the robot is a bit turned according to the door
        """

        #usefull variable
        size_to_check = 0.3 #it is hard-coded according to the size of the door

        # first part, getting the position the robot has to be (especially the rotation), it depends on which SOD we are
        # this process is similar to the one in goIFOdoor
        if userdata.side_of_door == 'face':
            frameIFOdoor_door = self.door.getFrameIFOdoor_face()

        else:
            frameIFOdoor_door = self.door.getFrameIFOdoor_behind()

        frameIFOdoor_map = self.robot.tf_buffer.transform(frameIFOdoor_door, "map", rospy.Duration(1.0))
        rot_y_expected = frameIFOdoor_map.frame.M.GetRot()[2] # get the rot_y we want to have

        #get the rotation angle
        position = self.robot.base.get_location()
        rot_y_real = position.frame.M.GetRot()[2] #rotation of the robot according to map frame

        rot_y = - (rot_y_expected - rot_y_real) # real reottaion difference

        #work on the message
        msg_laser = rospy.wait_for_message("/hero/base_laser/scan", LaserScan) #get the laser message
        position_angle_zero_according_robot = msg_laser.angle_min/msg_laser.angle_increment #get angle 0 according to the robot
        position_angle_zero = int(position_angle_zero_according_robot - (rot_y/msg_laser.angle_increment)) #get real angle 0, taking into account the rotation of the robot
        distance = msg_laser.ranges[position_angle_zero] #get the distance
        if distance > 1.4:
            rospy.sleep(0.5)
            #case door is fully open, no need for the rest to execute
            return 'open'

        angle_max = math.atan(size_to_check/distance) #get the angle to check
        number_of_value_from_position_angle_zero = int(angle_max/msg_laser.angle_increment) #nb of value on right and left away from angle zero

        #get the two value to check (angle right and and angle left)
        value_to_check_min = position_angle_zero - number_of_value_from_position_angle_zero;
        value_to_check_max = position_angle_zero + number_of_value_from_position_angle_zero;

        #get the distance on the right and the left
        d_angle_min = msg_laser.ranges[value_to_check_min];
        d_angle_max = msg_laser.ranges[value_to_check_max];

        rospy.sleep(0.5)
        if abs(d_angle_max-d_angle_min)>0.08:
            return 'intermediate'

        else:
            return 'close'

class updateHandleLocationFromServiceServer(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        #service
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        smach.State.__init__(self, outcomes=['updated','fail'], input_keys=['side_of_door'])
        
    def execute(self, userdata):
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
        # rospy.loginfo("handle_point_response:")
        # rospy.loginfo(handle_point_response.point_out)
        
        rospy.sleep(1)
        
        # self.door_info.call("publish_marker", handle_point_estimate)
        # rospy.sleep(1)
        
        self.door.updateHandlePose(self.robot.tf_buffer.transform(handle_point_response.point_out, "map", rospy.Duration(1.0)))
        
        return 'updated'
    
    
class updateHandleLocation(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        
        smach.State.__init__(self, outcomes=['updated','fail'], input_keys=['side_of_door'])

    def execute(self, userdata):
        #get the frame of the handle
        if userdata.side_of_door == 'face':
            handle_estimate = self.door.handle_pose
        else:
            handle_estimate = self.door.handle_behind_pose

        goal = LocateDoorHandleGoal()
        goal_estimate = PointStamped()

        #update goal and goal estimate
        goal_estimate.header.frame_id = "map"
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()
        rospy.loginfo("goal_estimate")
        rospy.loginfo(goal_estimate)
        goal.handle_location_estimate = goal_estimate
        

        #to use the 3 following fct, hero has to be IFO the door (TT RVIZ or TT service)
        self.robot.perception.locate_handle_client.send_goal(goal) #ask hero to watch
        self.robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0)) #wait for result (not mandatory)
        state = self.robot.perception.locate_handle_client.get_state() #success or no

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("detecting handle is a success")
            result = self.robot.perception.locate_handle_client.get_result() #get the result

            # rospy.loginfo("info about results")
            # rospy.loginfo(result)

            #use the result to get the middle of the handle
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])

            handle_loc = VectorStamped.from_xyz(x,y,z,rospy.Time.now(),result.handle_edge_point1.header.frame_id) #location of the handle in a vector to be able to use it
            #now we have to add this vector to door to be sure we save it somewhere
            self.door.updateHandlePose(self.robot.tf_buffer.transform(handle_loc, "map",rospy.Duration(1.0))) #1 seconde is enough for the extrapolation to the futur.
            return 'updated'

        else:
            rospy.loginfo("detecting handle is not a success")
            return 'fail'

class goIFOhandle(smach.State):
    def __init__(self, door, robot):
        smach.State.__init__(self, outcomes=['IFO_handle','fail'], input_keys=['side_of_door'])
        self.door = door
        self.robot = robot
        self.pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=2)

    def execute(self, userdata):
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
            #REALITY
            pose_stamped_IFOhandle = create_pose_stamped(kdl_vector[0], kdl_vector[1], kdl_vector[2], kdl_quaternion[0], kdl_quaternion[1], kdl_quaternion[2], kdl_quaternion[3])
        else:
            #SIMULATION
            if userdata.side_of_door == 'face':
                pose_stamped_IFOhandle = create_pose_stamped(6.55, 0.381, 0, 0, 0, 0.001, 0.99)
            else:
                pose_stamped_IFOhandle = create_pose_stamped(kdl_vector[0], kdl_vector[1], kdl_vector[2], kdl_quaternion[0], kdl_quaternion[1], kdl_quaternion[2], kdl_quaternion[3])

        #publication
        rospy.sleep(0.5)
        self.pub.publish(pose_stamped_IFOhandle)

        position_achieve()

        rospy.loginfo("we are IFO the handle")

        rospy.sleep(7)
        return 'IFO_handle'

class graspeHandle(smach.State):
    def __init__(self, robot, arm, door):
        self.robot = robot
        self.door = door
        self.arm = arm
        self.rate = rospy.Rate(1)
        smach.State.__init__(self, outcomes=['good_position_of_arm', 'fail'], input_keys=['side_of_door'])

    def execute(self, userdata):
        self.arm.gripper.send_goal("open")

        # rospy.loginfo("info about position")
        # rospy.loginfo(self.robot.base.get_location())

        handle_position = self.door.getHandlePose() #get the pose (vector) of the handle

        #we have to transform this vector into a frameStamped to be able to use some functions
        # we have to first create a kdl vector that will allow to create a frameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = kdl.Vector(handle_position.point.x, handle_position.point.y, handle_position.point.z)

        #change some value of the kdl vector before
        # if not simulation:
        #     #REALITY
        #     if userdata.side_of_door == "face":
        #         kdl_vector[0] = kdl_vector[0] -0.055
        #         kdl_vector[1]=kdl_vector[1]+0.05
        #     else:
        #         kdl_vector[0] = kdl_vector[0] + 0.053
        # else:
        #     #SIMULATION
        #     if userdata.side_of_door == "face":
        #         kdl_vector[0] = kdl_vector[0] - 0.045
        #         kdl_vector[1] = kdl_vector[1] + 0.02

        #     else:
        #         kdl_vector[0] = kdl_vector[0] + 0.045
        #         kdl_vector[1] = kdl_vector[1] + 0.02
        #         # kdl_vector[0] = 7.47
        #         # kdl_vector[1] = 0.3
        #         # kdl_vector[2] = 1.12
        if simulation:
            if not userdata == "face":
                kdl_vector[2] = kdl_vector[2] + 0.05

        kdl_frame = kdl.Frame(kdl_rotation, kdl_vector) #frame kdl
        handle_frame = FrameStamped(kdl_frame,rospy.Time.now(), frame_id = "map") #map is hard coded but it must change #get the frame of the handle from the vector
        #create a goal
        goal_handle = self.robot.tf_buffer.transform(handle_frame, self.robot.base_link_frame, rospy.Duration(1.0))
        goal_handle.frame.M = kdl.Rotation.RPY(-1.57, 0.0, 0.0) #rotation of the gripper in order to be able to grasp the handle

        #move the arm and wait for the result
        result = self.arm.send_goal(goal_handle,timeout=10.0)
        self.arm.wait_for_motion_done()

        if result:
            rospy.loginfo('arm is in the good position: handle is ready to be grasped')
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
        self.rate = rospy.Rate(0.3)

    def execute(self, userdata):

        #get the position of the join
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']

        joint1_new_position = joint1 - 0.055

        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        self.rate.sleep()

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

        self.robot.base.force_drive(0.1, y / (x / 0.1), 0, x ) #TODO

        self.rate.sleep()

        return 'doorIsPushed'

class getDataDoorClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dataIsGet', 'fail'], output_keys=['angle', 'distance_start', 'distance_end'])

    def execute(self, userdata):
        """
        The goal of this class is to get some useful data we need to pull the door from the state close.
        All the data we need are in the line extraction segment message: topic: /line_segments
        """

        msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
        segments = msg_line_segments.line_segments # extract the segments

        #get the useful segment
        for segment in segments:
            if segment.start[1] < 0 and segment.end[1] > 0:
                start = segment.start
                end = segment.end
                angle = segment.angle

        rospy.loginfo('start: ' + str(start) + ' end: ' + str(end) + ' angle: ' + str(angle))
        userdata.angle = angle
        # userdata.start = start
        # userdata.end = end

        #calculate the distance between the robot and the start of the segment and the robot and the end of the segment
        distance_start = math.sqrt(start[0]**2 + start[1]**2)
        distance_end = math.sqrt(end[0]**2 + end[1]**2)

        userdata.distance_start = distance_start
        userdata.distance_end = distance_end

        return 'dataIsGet'

class pullDoorClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_pulled', 'fail'], input_keys=['angle'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userdata):
        if userdata.angle < 0:
            beta = math.pi/2 + userdata.angle
            x = - math.sin(beta)/10
            y = - math.cos(beta)/10
        else:
            beta = userdata.angle

            x = - math.cos(beta) / 10
            y = - math.sin(beta) / 10

        twist_msg = create_twist(0,0,0,x,y,0)

        self.pub.publish(twist_msg)
        rospy.sleep(1.5)
        return 'door_pulled'

class positionRobotDoorClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['position_achieve', 'fail'], input_keys=['angle', 'distance_start', 'distance_end'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userdata):
        end_rot = False
        end_mov = False

        while not end_rot:
            msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList) #get the message
            segments = msg_line_segments.line_segments # extract the segments

            #get the useful segment
            for segment in segments:
                if segment.start[1] < 0 and segment.end[1] > 0:
                    rot = segment.angle #get the angle

            if abs(abs(rot) - abs(userdata.angle)) > 0.05:
                if userdata.angle < 0:
                    if abs(rot) - abs(userdata.angle) > 0:
                        twist_msg = create_twist(0,0,-0.05,0,0,1)
                        self.pub.publish(twist_msg)
                        rospy.sleep(3)
                    else:
                        twist_msg = create_twist(0,0,0.05,0,0,1)
                        self.pub.publish(twist_msg)
                        rospy.sleep(3)

                else:
                    
                    #userdata.angle > 0
                    if abs(rot) < abs(userdata.angle):
                        rospy.loginfo('rot < angle' + 'rot: ' + str(rot) + ' userdata.angle: ' + str(userdata.angle))
                        twist_msg = create_twist(0,0,-0.05,0,0,-1)
                        self.pub.publish(twist_msg)
                        rospy.sleep(1)
                    else:
                        #rost > userdata.angle
                        rospy.loginfo('\n')
                        rospy.loginfo('rot > angle : ' + 'rot: ' + str(rot) + ' userdata.angle: ' + str(userdata.angle))
                        rospy.sleep(1)
                        twist_msg = create_twist(0,0,0.1,0,0,-1)
                        self.pub.publish(twist_msg)
                        
            else:
                end_rot = True
        
        while not end_mov:
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
                distance_start = math.sqrt(start[0]**2 + start[1]**2)
                distance_end = math.sqrt(end[0]**2 + end[1]**2)
                
                #compare the distance
                if abs(abs(userdata.distance_start) - abs(distance_start)) > 0.035:
                    rospy.loginfo('\n')
                    rospy.loginfo("userdata.distance_start: " + str(userdata.distance_start) + " distance_start: " + str(distance_start))
                    rospy.loginfo("userdata.distance_end: " + str(userdata.distance_end) + " distance_end: " + str(distance_end))
                    rospy.sleep(3)
                    if userdata.angle > 0:
                        beta = math.pi/2 - angle
                        x = math.cos(beta) / 50
                        y = - abs(math.sin(beta)) / 50
                        twist_msg = create_twist(1,0,0,x,y,0)
                        self.pub.publish(twist_msg)
                    else:
                        beta = math.pi/2 - angle
                        x = - abs(math.cos(beta)) / 20
                        y = - abs(math.sin(beta)) / 20
                        twist_msg = create_twist(1,0,0,x,y,0)
                        self.pub.publish(twist_msg)
                else:
                    
                    end_mov = True
        
        return 'position_achieve'

class goUpHandle(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['handleIsUp', 'fail'])

    def execute(self, userdata):

        #get the position of the join
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']

        joint1_new_position = joint1 + 0.055

        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        rospy.sleep(1)

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
        if simulation:
            dmin = 0.4 # minimum away distance from obstacle
            dget = 0.3
        else:
            dmin = 0.4 # minimum away distance from obstacle
            dget = 0.3

        while dget < dmin:
            dget, angle_min = self.laserData_callback()
            x_direction = - math.cos(angle_min)
            y_direction = - math.sin(angle_min)
            twist_message = create_twist(0,0,0, x_direction/15, y_direction/15, 0) #use of /15 because we want to move slowly and to update often the minimum distance
            self.pub.publish(twist_message)
            rospy.sleep(0.5)

        return 'away'

    def laserData_callback(self):

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
        end = False # is false until we are in the point

        # first part, getting the position the robot has to be (especially the rotation), it depends on which SOD we are
        # this process is similar to the one in goIFOdoor
        # the expected rotation is the same as beiing IFO door because we want the robot to be straight IFO the door


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

            if not simulation:
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

        return 'doorIsPushed'

class pullDoorOpen(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['doorIsPulled', 'fail'])
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        return 'doorIsPulled'

class crossDoor(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
        smach.State.__init__(self, outcomes=['doorIsCrossed', 'fail'], input_keys=['side_of_door'])


    def execute(self, userdata):
        end = False # is false until we are in the point
        # first part, getting the position the robot has to be (especially the rotation), it depends on which SOD we are
        # this process is similar to the one in goIFOdoor
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
                    y = y + 0.1
            else:
                if userdata.side_of_door == 'face':
                    y = y - 0.1
                else:
                    y = y + 0.05

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
    def __init__(self, robot):
        self.robot = robot
        smach.State.__init__(self, outcomes=['goodPosition','fail'], input_keys=['side_of_door'])
        self.pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)

    def execute(self, userata):
        if simulation:
            #simulation
            if userata.side_of_door == 'face':
                treshold = 0.59
            else:
                treshold = 0.59
            d = 0.7
        else:
            #reality
            treshold = 0.40
            d = 0.7

        x = 0.05
        if userata.side_of_door == 'face':
            y = -0.01
        else:
            y = 0.01


        while d > treshold:
            d = self.laserData_callback()
            twist_message = create_twist(0, 0, 0, x, y, 0)
            self.pub.publish(twist_message)
            rospy.sleep(0.5)


        #before returning, I want the position
        # position = self.robot.base.get_location()
        # rospy.loginfo("position: " +  str(position))
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
        if userdata.side_of_door == 'face':
            return 'face'

        else:
            return 'behind'

def sm_pull_door_close():
    sm_pull_door_close = smach.StateMachine(outcomes=['doorIsPulled', 'fail'])
    sm_pull_door_close.userdata.angle = None
    sm_pull_door_close.userdata.distance_start = None

    with sm_pull_door_close:
        smach.StateMachine.add('get_data', getDataDoorClose(), transitions={'dataIsGet' : 'pull_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'distance_start' : 'distance_start', 'distance_end' : 'distance_end'})
        smach.StateMachine.add('pull_door_close', pullDoorClose(), transitions={'door_pulled' : 'position_robot_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle'})
        smach.StateMachine.add('position_robot_door_close', positionRobotDoorClose(), transitions={'position_achieve' : 'pull_door_close', 'fail' : 'fail'}, remapping = {'angle' : 'angle', 'distance_start' : 'distance_start', 'distance_end' : 'distance_end'})

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
    sm_pull_door_close_ = sm_pull_door_close()
    #get sm
    sm_open_door_close = smach.StateMachine(outcomes=['doorIsNotClose', 'fail'], input_keys = ['side_of_door'], output_keys = ['side_of_door'])
    with sm_open_door_close:
        smach.StateMachine.add('goIFOhandle', goIFOhandle(my_door, robot), transitions={'IFO_handle' : 'updateHandleLocation', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('updateHandleLocation', updateHandleLocationFromServiceServer(robot, my_door), transitions={'updated' : 'goMinimumPosition', 'fail' : 'fail'}, remapping={'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('goMinimumPosition', moveTreshold(robot), transitions={'goodPosition' : 'graspingHandle', 'fail' : 'fail'})
        smach.StateMachine.add('graspingHandle', sm_grasp_handle_, transitions={'handleIsGrasped' : 'unlatchHandle', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'}) #TODO
        smach.StateMachine.add('unlatchHandle', unlatchHandle(arm), transitions={'handleIsUnlatched_push' : 'pushDoorUnlatched','handleIsUnlatched_pull' : 'pullDoorUnlatched', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('pushDoorUnlatched',pushDoorUnlatched(robot, my_door), transitions={'doorIsPushed' :'goUpHandle', 'fail' : 'fail'} )
        smach.StateMachine.add('pullDoorUnlatched',sm_pull_door_close_, transitions={'doorIsPulled' :'goUpHandle', 'fail' : 'fail'} )
        smach.StateMachine.add('goUpHandle', goUpHandle(arm), transitions={'handleIsUp' : 'openGripper', 'fail' : 'fail'})
        smach.StateMachine.add('openGripper',openGripper(arm), transitions={'gripperOpen' :'doorIsNotClose', 'fail' : 'fail'} )

    return sm_open_door_close

def sm_open_door_open(robot, arm, my_door):
    sm_open_door_open = smach.StateMachine(outcomes=['doorIsOpen', 'fail', 'go_IFO_door'], input_keys = ['side_of_door'])
    with sm_open_door_open:
        smach.StateMachine.add('move_door_open', moveDoorOpen(), transitions={'face' : 'push_door_open', 'behind' : 'go_IFO_door'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('push_door_open', pushDoorOpen(robot, my_door), transitions={'doorIsPushed' : 'doorIsOpen', 'fail' : "fail"})
        smach.StateMachine.add('pull_door_open', pullDoorOpen(robot, my_door), transitions={'doorIsPulled' : 'go_IFO_door', 'fail' : "fail"})

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
        smach.StateMachine.add('update_door_state', updateDoorState(robot, my_door), transitions={'close' : 'open_door_closed', 'intermediate' : 'open_door_open', 'open' : 'cross_door', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('open_door_closed', sm_open_door_close_, transitions={'doorIsNotClose' :'robot_back_posInit', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('open_door_open', sm_open_door_open_, transitions={'doorIsOpen' : 'cross_door','go_IFO_door' : 'goIFOdoor' ,'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('robot_back_posInit', robotBackInitialPosition(arm), transitions={'initialPosition' : 'open_door_open', 'fail' : 'fail'})
        #smach.StateMachine.add('move_away_from_obstacle', goAwayFromObstacle(robot), transitions={'away' : 'goIFOdoor', 'fail' : 'fail'})
        smach.StateMachine.add('cross_door', crossDoor(robot, my_door), transitions={'doorIsCrossed' : 'doorIsCrossed', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})

    return sm_cross_door

def main():
    rospy.init_node('open_door_smach_node',anonymous=True)

    #init of devices
    robot = get_robot("hero")
    arm = robot.get_arm(force_sensor_required=True)
    door = robot.ed.get_entity(uuid="door_inside")
    my_door = Door(door)
    

    #smach state machine main
    sm_main = sm_cross_door(robot, arm, my_door)
    outcome = sm_main.execute()

if __name__ == '__main__':
    main()



