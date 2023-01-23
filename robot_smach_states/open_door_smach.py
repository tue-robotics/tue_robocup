#!/usr/bin/env python

#system import
import typing

#ros import
import rospy
import rosapi
from rosapi import srv
from std_msgs.msg import String
import actionlib
from tue_msgs.msg import LocateDoorHandleGoal
from geometry_msgs.msg import PointStamped, Point
from pykdl_ros import FrameStamped, VectorStamped
from robot_smach_states.srv import door_info

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

class Door(Entity):
    HANDLE_ID = "handle_volume"
    HANDLE_BEHIND_ID = "handle_behind_volume"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"
    HANDLE_POSE = None
    HANDLE_BEHIND_POSE = None

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

    def update_pose(self,new_pose):
        self.HANDLE_POSE = new_pose

    def update_behind_pose(self,new_pose):
        self.HANDLE_BEHIND_POSE = new_pose

    def getHandleBehindPose(self):
        return self.HANDLE_BEHIND_POSE

    def getPose(self):
        return self.HANDLE_POSE

class updateDoorState(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['close','intermediate','open', 'fail'])
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        self.rate = rospy.Rate(0.3)
        self.robot = robot

    def execute(self, userdata):
        position = self.robot.base.get_location()
        rot_y = position.frame.M.GetRot()[2]
        response = self.door_info.call("door_state",rot_y)
        self.rate.sleep()

        if response.output_int == 1:
            return 'open'

        elif response.output_int == 2:
            return 'intermediate'

        elif response.output_int == 3:
            return 'close'

        else:
            return 'fail'

class goIFOhandle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IFO_handle','fail'])
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        #self.rate = rospy.Rate(0.05)
        #self.rate = rospy.Rate(0.03)

    def execute(self, userata):
        self.door_info.call("goIFOhandle",0)
        rospy.sleep(20)
        rospy.loginfo("we are IFO the handle")
        return 'IFO_handle'

class goIFOdoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IFO_door','fail'])
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        #self.rate = rospy.Rate(0.05)
        #self.rate = rospy.Rate(0.2)

    def execute(self, userata):
        self.door_info.call("goIFOdoor",0)
        rospy.sleep(20)
        return 'IFO_door'

class updateHandleLocation(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['updated','fail'])

    def execute(self, userdata):
        handle_estimate = self.door.handle_pose #call handle pose to know where is the handle. return the middle pf the handle

        #dont' really know what these two fct are doing
        goal = LocateDoorHandleGoal()
        goal_estimate = PointStamped()


        #update goal and goal estimate
        goal_estimate.header.frame_id = "map"
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()
        goal.handle_location_estimate = goal_estimate

        #to use the 3 following fct, hero has to be IFO the door (TT RVIZ or TT service)
        self.robot.perception.locate_handle_client.send_goal(goal) #ask hero to watch
        self.robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0)) #wait for result (not mandatory)
        state = self.robot.perception.locate_handle_client.get_state() #success or no

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("detecting handle is a success")
            result = self.robot.perception.locate_handle_client.get_result() #get thre result

            #use the result to get the middle of the handle
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])

            handle_loc = VectorStamped.from_xyz(x,y,z,rospy.Time.now(),result.handle_edge_point1.header.frame_id) #location of the handle in a vector to be able to use it
            #now we have to add this vector to door to be sure we save it somewhere
            self.door.update_pose(self.robot.tf_buffer.transform(handle_loc, "map",rospy.Duration(1.0))) #1 seconde is enough for the extrapolation tot the futur.
            return 'updated'
        else:
            rospy.loginfo("detecting handle is not a success")
            return 'fail'

class updateHandleBehindLocation(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['updated','fail'])

    def execute(self, userdata):
        handle_estimate = self.door.handle_behind_pose #call handle pose to know where is the handle. return the middle pf the handle

        #dont' really know what these two fct are doing
        goal = LocateDoorHandleGoal()
        goal_estimate = PointStamped()


        #update goal and goal estimate
        goal_estimate.header.frame_id = "map"
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()
        goal.handle_location_estimate = goal_estimate

        #to use the 3 following fct, hero has to be IFO the door (TT RVIZ or TT service)
        self.robot.perception.locate_handle_client.send_goal(goal) #ask hero to watch
        self.robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0)) #wait for result (not mandatory)
        state = self.robot.perception.locate_handle_client.get_state() #success or no

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("detecting handle is a success")
            result = self.robot.perception.locate_handle_client.get_result() #get thre result

            #use the result to get the middle of the handle
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])

            handle_loc = VectorStamped.from_xyz(x,y,z,rospy.Time.now(),result.handle_edge_point1.header.frame_id) #location of the handle in a vector to be able to use it
            #now we have to add this vector to door to be sure we save it somewhere
            self.door.update_behind_pose(self.robot.tf_buffer.transform(handle_loc, "map",rospy.Duration(1.0))) #1 seconde is enough for the extrapolation tot the futur.
            return 'updated'
        else:
            rospy.loginfo("detecting handle is not a success")
            return 'fail'

class graspeHandle(smach.State):
    def __init__(self, robot, arm, door):
        self.robot = robot
        self.door = door
        self.arm = arm
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        self.rate = rospy.Rate(1)
        smach.State.__init__(self, outcomes=['good_position_of_arm', 'fail'], output_keys=['side_of_door'])

    def execute(self, userdata):
        #self.door_info.call("is_door_open",0)
        self.arm.gripper.send_goal("open") #open gripper
        handle_vector = self.door.getPose() #get the pose (vector) of the handle

        #we have to transform this vector into a frameStamped to be able to use some functions
        # we have to first create a kdl vector that will allow to create a frameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = handle_vector.vector

        #change some value of the kdl vector befor
        #in reality
        kdl_vector[0] = kdl_vector[0] -0.056#in order to not touch the handle
        kdl_vector[1]=kdl_vector[1]+0.05
        #in simulation
        #kdl_vector[0] = kdl_vector[0] -0.045#in order to not touch the handle
        #kdl_vector[1]=kdl_vector[1]+0.02

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
            userdata.side_of_door = 'IFO'
            return 'good_position_of_arm'
        else:
            rospy.loginfo('grasping handle is not a success')
            userdata.side_of_door = 'behind'
            return 'fail'

class graspeBehindHandle(smach.State):
    def __init__(self, robot, arm, door):
        self.robot = robot
        self.door = door
        self.arm = arm
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        self.rate = rospy.Rate(1)
        smach.State.__init__(self, outcomes=['good_position_of_arm', 'fail'])

    def execute(self, userdata):
        #self.door_info.call("is_door_open",0)
        self.arm.gripper.send_goal("open") #open gripper
        handle_vector = self.door.getHandleBehindPose() #get the pose (vector) of the handle

        #we have to transform this vector into a frameStamped to be able to use some functions
        # we have to first create a kdl vector that will allow to create a frameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = handle_vector.vector

        #change some value of the kdl vector befor c
        kdl_vector[0] = kdl_vector[0] + 0.045 #in order to not touch the handle
        #kdl_vector[1]=kdl_vector[1]+0.02
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

        self.robot.base.force_drive(0.1, y / (x / 0.1), 0, x )

        self.rate.sleep()

        return 'doorIsPushed'

class pullDoorUnlatched(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['doorIsPulled', 'fail'])
        self.rate = rospy.Rate(0.5)

    def execute(self,userdata):
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

        self.robot.base.force_drive(-0.1, y / (x / 0.1), 0, 0.5 )

        rospy.loginfo("robot has moved")

        return 'doorIsPulled'

class unlatchHandle(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['handleIsUnlatched_push', 'handleIsUnlatched_pull', 'fail'], input_keys=['side_of_door'])
        self.rate = rospy.Rate(0.3)

    def execute(self, userdata):

        #get the position of the join
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']

        joint1_new_position = joint1 - 0.05

        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
        self.arm.wait_for_motion_done()
        self.rate.sleep()

        if userdata.side_of_door == 'IFO':
            self.rate.sleep()
            return 'handleIsUnlatched_push'

        else:
            self.rate.sleep()
            return 'handleIsUnlatched_pull'

class openGripper(smach.State):
    def __init__(self, arm):
        self.arm = arm
        smach.State.__init__(self, outcomes=['gripperOpen', 'fail'])
        #self.rate = rospy.Rate(0.2)

    def execute(self, userdata):
          self.arm.gripper.send_goal("open")
          #self.rate.sleep()
          rospy.sleep(8)
          return 'gripperOpen'

class moveTreshold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodPosition','fail'])
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        #self.rate = rospy.Rate(0.05)

    def execute(self, userata):
        #self.door_info.call("is_door_open",0)
        self.door_info.call("go_treshold",0)
        rospy.sleep(10)
        return 'goodPosition'

class moveTresholdBehind(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodPosition','fail'])
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        self.rate = rospy.Rate(0.5)

    def execute(self, userata):
        #self.door_info.call("is_door_open",0)
        self.door_info.call("go_treshold_behind",0)
        self.rate.sleep()
        return 'goodPosition'

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
        rospy.sleep(8)

        return 'initialPosition'

class crossDoor(smach.State):
    def __init__(self, robot, door):
        smach.State.__init__(self, outcomes=['doorIsCrossed','fail'])
        # self.robot = robot
        # self.door = door
        # self.rate = rospy.Rate(0.5)
        self.door_info = rospy.ServiceProxy('door_info', door_info)

    def execute(self, userdata):

        # #get some frame
        # #these are frame of the door according to the robot point of view (TT transform)
        # door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
        # door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right

        # #get the coordinate
        # x1 = door_frame_robot_left.vector.x()
        # y1 = door_frame_robot_left.vector.y()
        # x2 = door_frame_robot_right.vector.x()
        # y2 = door_frame_robot_right.vector.y()

        # #mean of right and left
        # x = (x1 + x2) / 2.0
        # y = (y1 + y2) / 2.0

        # self.robot.base.force_drive(0.1, y/(x/0.007) , 0, 20)
        self.door_info.call("go_other_side",0)
        rospy.sleep(15)
        return 'doorIsCrossed'

class moveDoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IFO', 'behind','unknown'],  input_keys=['side_of_door'])

    def execute(self, userdata):
        if userdata.side_of_door == 'IFO':
            return 'IFO'

        elif userdata.side_of_door == 'behind':
            return 'behind'

        else:
            return 'unknown'

class pushDoorOpen(smach.State):
    def __init__(self, robot, door):
        # self.robot = robot
        # self.door = door
        smach.State.__init__(self, outcomes=['doorIsPushed', 'fail'])
        # self.rate = rospy.Rate(0.15)
        self.door_info = rospy.ServiceProxy('door_info', door_info)

    def execute(self, userdata):
        # self.rate.sleep()
        # #get some frame
        # #these are frame of the door according to the robot point of view (TT transform)
        # door_frame_robot_left = self.robot.tf_buffer.transform(self.door.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
        # door_frame_robot_right = self.robot.tf_buffer.transform(self.door.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right

        # #get the coordinate
        # x1 = door_frame_robot_left.vector.x()
        # y1 = door_frame_robot_left.vector.y()
        # x2 = door_frame_robot_right.vector.x()
        # y2 = door_frame_robot_right.vector.y()

        # #mean of right and left
        # x = (x1 + x2) / 2.0
        # y = (y1 + y2) / 2.0

        # self.robot.base.force_drive(0.1, y/(x/0.007) , 0, 15)
        # self.rate.sleep()

        self.door_info.call("go_other_side",0)
        rospy.sleep(15)

        return 'doorIsPushed'

class pullDoorOpen(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['doorIsPulled', 'fail'])
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        return 'doorIsPulled'

class determineSOD(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['IFO', 'behind', 'fail'])
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        return 'IFO'


def sm_grasp_handle(robot, arm, my_door):
    #“smach state machine to grasp the handle
    sm_grasp_handle = smach.StateMachine(outcomes=['handleIsGrasped', 'fail'],input_keys=['side_of_door'], output_keys = ['side_of_door'])
    with sm_grasp_handle:

        """
        #if I have to be closer behind
        #smach.StateMachine.add('GraspeHandle', graspeHandle(robot, arm, my_door), transitions={'good_position_of_arm' : 'closeGripper', 'fail' : 'goMinimumPositionBehind'}, remapping = {'side_of_door' : 'side_of_door'})
        #smach.StateMachine.add('goMinimumPositionBehind', moveTresholdBehind(), transitions={'goodPosition' : 'GraspeBehindHandle', 'fail' : 'fail'})
        #if it is the same distance behind
        """
        smach.StateMachine.add('GraspeHandle', graspeHandle(robot, arm, my_door), transitions={'good_position_of_arm' : 'closeGripper', 'fail' : 'GraspeBehindHandle'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('GraspeBehindHandle', graspeBehindHandle(robot, arm, my_door), transitions={'good_position_of_arm' : 'closeGripper', 'fail' : 'fail'})
        smach.StateMachine.add('closeGripper', closeGripper(arm), transitions={'gripperClose' : 'handleIsGrasped', 'fail' : 'fail'})

    return sm_grasp_handle

def sm_open_door_close(robot, arm, my_door):
    #get sm_grasp_handle
    sm_grasp_handle_ = sm_grasp_handle(robot, arm, my_door)

    #get sm
    sm_open_door_close = smach.StateMachine(outcomes=['doorIsNotClose', 'fail'], input_keys = ['side_of_door'], output_keys = ['side_of_door'])
    with sm_open_door_close:
        smach.StateMachine.add('goIFOhandle', goIFOhandle(), transitions={'IFO_handle' : 'updateHandleLocation', 'fail' : 'fail'})
        smach.StateMachine.add('updateHandleLocation', updateHandleLocation(robot, my_door), transitions={'updated' : 'updateHandleBehindLocation', 'fail' : 'fail'})
        smach.StateMachine.add('updateHandleBehindLocation', updateHandleBehindLocation(robot, my_door), transitions={'updated' : 'goMinimumPosition', 'fail' : 'fail'})
        smach.StateMachine.add('goMinimumPosition', moveTreshold(), transitions={'goodPosition' : 'graspingHandle', 'fail' : 'fail'})
        smach.StateMachine.add('graspingHandle', sm_grasp_handle_, transitions={'handleIsGrasped' : 'unlatchHandle', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('unlatchHandle', unlatchHandle(arm), transitions={'handleIsUnlatched_push' : 'pushDoorUnlatched','handleIsUnlatched_pull' : 'pullDoorUnlatched', 'fail' : 'fail'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('pushDoorUnlatched',pushDoorUnlatched(robot, my_door), transitions={'doorIsPushed' :'openGripper', 'fail' : 'fail'} )
        smach.StateMachine.add('pullDoorUnlatched',pullDoorUnlatched(robot, my_door), transitions={'doorIsPulled' :'openGripper', 'fail' : 'fail'} )
        smach.StateMachine.add('openGripper',openGripper(arm), transitions={'gripperOpen' :'doorIsNotClose', 'fail' : 'fail'} )

    return sm_open_door_close

def sm_open_door_open(robot, arm, my_door):
    sm_open_door_open = smach.StateMachine(outcomes=['doorIsOpen', 'fail'], input_keys = ['side_of_door'])

    with sm_open_door_open:
        smach.StateMachine.add('move_door_open', moveDoorOpen(), transitions={'IFO' : 'push_door_open', 'behind' : 'pull_door_open', 'unknown' : 'determine_SOD'}, remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('push_door_open', pushDoorOpen(robot, my_door), transitions={'doorIsPushed' : 'doorIsOpen', 'fail' : "fail"})
        smach.StateMachine.add('pull_door_open', pullDoorOpen(robot, my_door), transitions={'doorIsPulled' : 'doorIsOpen', 'fail' : "fail"})
        smach.StateMachine.add('determine_SOD', determineSOD(robot, my_door), transitions={'IFO' : 'push_door_open', 'behind' : 'pull_door_open', 'fail':'fail'})

    return sm_open_door_open

def sm_cross_door(robot, arm, my_door):
    #get the opening of the door if it intermediate or close
    sm_open_door_close_= sm_open_door_close(robot, arm, my_door)
    sm_open_door_open_ = sm_open_door_open(robot, arm, my_door)

    sm_cross_door =  smach.StateMachine(outcomes=['doorIsCrossed', 'fail'])
    sm_cross_door.userdata.side_of_door = 'Unknown'

    with sm_cross_door:
        smach.StateMachine.add('goIFOdoor', goIFOdoor(), transitions={'IFO_door' : 'update_door_state', 'fail' : 'fail'})
        smach.StateMachine.add('update_door_state', updateDoorState(robot), transitions={'close' : 'open_door_closed', 'intermediate' : 'open_door_open', 'open' : 'cross_door', 'fail' : 'fail'})
        smach.StateMachine.add('open_door_closed', sm_open_door_close_, transitions={'doorIsNotClose' :'robot_back_posInit', 'fail' : 'fail'},remapping = {'side_of_door' : 'side_of_door'})
        smach.StateMachine.add('robot_back_posInit', robotBackInitialPosition(arm), transitions={'initialPosition' : 'goIFOdoor', 'fail' : 'fail'})
        smach.StateMachine.add('open_door_open', sm_open_door_open_, transitions={'doorIsOpen' : 'cross_door', 'fail' : 'fail'})
        smach.StateMachine.add('cross_door', crossDoor(robot, my_door), transitions={'doorIsCrossed' : 'doorIsCrossed', 'fail' : 'fail'})

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
