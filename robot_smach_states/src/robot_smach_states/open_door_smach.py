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

class moveIFOdoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.Setparameter = rospy.ServiceProxy('SetParam', srv.SetParam)
        self.rate = rospy.Rate(0.5)

    def execute(self, userata):
        self.Setparameter.call("goIFOdoor","1")   
        self.rate.sleep()  
        return 'outcome1'

class moveArm(smach.State):
    def __init__(self,arm):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.arm = arm
        self.rate = rospy.Rate(0.5)

    def execute(self, userata):
        list_send=[0.26, -0.4, 0, -1.2, 1.5]
        self.arm._arm._send_joint_trajectory([list_send])
        self.rate.sleep()
        return 'outcome1'

class openGripper(smach.State):
    def __init__(self, arm):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'], input_keys=['gripper_state_in'], output_keys=['gripper_state_out'])
        self.arm = arm 
        self.rate = rospy.Rate(0.5)

    def execute(self, userdata):
        if userdata.gripper_state_in == 0:
            self.arm._arm.gripper.send_goal("open")
            userdata.gripper_state_out = 1
            self.rate.sleep()
            return 'outcome1'

        else:
            self.arm._arm.gripper.send_goal("close")
            userdata.gripper_state_out = 0
            self.rate.sleep()
            return 'outcome2'

class moveTreshold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.Setparameter = rospy.ServiceProxy('SetParam', srv.SetParam)
        self.rate = rospy.Rate(0.5)

    def execute(self, userata):
        self.Setparameter.call("go_treshold","1")   
        self.rate.sleep()  
        return 'outcome1'

class pushDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.Setparameter = rospy.ServiceProxy('SetParam', srv.SetParam)
        self.rate = rospy.Rate(0.5)

    def execute(self, userata):
        self.Setparameter.call("push_door","1")   
        self.rate.sleep()  
        return 'outcome1'

class Door(Entity):
    HANDLE_ID = "handle_volume"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"
    HANDLE_POSE = None

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
        
    def getPose(self):
        return self.HANDLE_POSE
        

class updateHandleLocation(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['success','fail'])

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
            return 'success'
        else:
            rospy.loginfo("detecting handle is not a success")   
            return 'fail'
        
class PrepareGraspingHandle(smach.State):
    def __init__(self, robot, arm, door):
        self.robot = robot
        self.door = door
        self.arm = arm
        smach.State.__init__(self, outcomes=['success', 'fail'])
        
    def execute(self, userdata):
        self.arm.gripper.send_goal("open") #open gripper
        handle_vector = self.door.getPose() #get the pose (vector) of the handle
        
        #we have to transform this vector into a frameStamped to be able to use some functions
        # we have to first create a kdl vector that will allow to create a frameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = handle_vector.vector
        
        #change some value of the kdl vector befor c
        kdl_vector[0] = kdl_vector[0] -0.03 #in order to not touch the handle
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
            return 'success'
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
  
class pushDoorOpen(smach.State):
    def __init__(self, robot, door):
        self.robot = robot
        self.door = door
        smach.State.__init__(self, outcomes=['doorIsPushed', 'fail'])
        
    def execute(self, userdata):
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
        
        rospy.loginfo("robot has moved")
        
        return 'doorIsPushed'
        
def main():
    rospy.init_node('open_door_smach_node',anonymous=True)

    #init of devices
    robot = get_robot("hero")
    arm = robot.get_arm(force_sensor_required=True)
    door = robot.ed.get_entity(uuid="door_inside")
    my_door = Door(door)

    #smach state machine
    sm = smach.StateMachine(outcomes=['doorIsOpen', 'fail'])
    sm.userdata.gripper_state = 0; #0 is close, 1 is open

    # in the container
    with sm: 
        #smach.StateMachine.add('IFO_door', moveIFOdoor(), transitions={'outcome1' : 'updateHandleLocation', 'outcome2':'fail'})
        smach.StateMachine.add('updateHandleLocation', updateHandleLocation(robot, my_door), transitions={'success' : 'PrepareGraspingHandle', 'fail' : 'fail'})
        smach.StateMachine.add('PrepareGraspingHandle', PrepareGraspingHandle(robot, arm, my_door), transitions={'success' : 'close_gripper', 'fail' : 'fail'})
        smach.StateMachine.add('close_gripper', closeGripper(arm), transitions={'gripperClose' : 'pushDoorOpen', 'fail' : 'fail'})
        smach.StateMachine.add('pushDoorOpen',pushDoorOpen(robot, my_door), transitions={'doorIsPushed' :'doorIsOpen', 'fail' : 'fail'} )
        
        """savoir quel cote de la porte on est (peute etre DetermineDoorDirection)
            preparer l'autre cote
            pullDoorOpen
            pushDoor
            PullDoor
            tourner la poignee"""
        
        #add some states 
        # sm_sub = smach.StateMachine(outcomes=['success_move','fail'])
        # with sm_sub:
        #     smach.StateMachine.add('IFO_door', moveIFOdoor(), transitions={'outcome1' : 'move_arm', 'outcome2':'fail'})
        #     smach.StateMachine.add('move_arm', moveArm(arm), transitions={'outcome1' : 'success_move', 'outcome2':'fail'})
        # smach.StateMachine.add('SUB', sm_sub, transitions={'success_move':'open_gripper'})
        # smach.StateMachine.add('open_gripper', openGripper(arm), transitions={'outcome1' : 'move_treshold','outcome2':'push_door', 'outcome3':'fail'}, remapping={'gripper_state_in':'gripper_state', 'gripper_state_out':'gripper_state'})
        # smach.StateMachine.add('move_treshold', moveTreshold(), transitions={'outcome1' : 'open_gripper', 'outcome2':'fail'})
        # smach.StateMachine.add('push_door', pushDoor(), transitions={'outcome1' : 'doorIsOpen', 'outcome2':'fail'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()