from __future__ import absolute_import

# ROS
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import rospy
import smach
import tf2_ros
import math
from geometry_msgs.msg import Twist
import tf.transformations as tft
import numpy as np

# TU/e Robotics
from ed.entity import Entity
from robot_skills.robot import Robot
from robot_skills.arm.arms import PublicArm, GripperTypes
from robot_smach_states.utility import check_arm_requirements, ResolveArm
from robot_smach_states.util.designators import check_type
from robot_smach_states.navigation.navigate_to_grasp import NavigateToGrasp

from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.manipulation.cutlery_detector import YoloSegmentor
from robot_smach_states.manipulation.active_grasp_detector import ActiveGraspDetector
from robot_skills.util.exceptions import TimeOutException



class PrepareGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],}

    def __init__(self, robot: Robot, arm: ArmDesignator) -> None:
        """
        Set the arm in the appropriate position before actually grabbing

        :param robot: robot to execute state with
        :param arm: Designator that resolves to the arm to grasp with
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None) -> str:
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"  
              
        #pre-grasp position in two steps to limit possible collisions with the table
        pre_grasp_joint_goal_upwards = [0.69, # arm lift joint. ranges from 0.0 to 0.7m
                                        0.0, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                                        0.0, # arm roll joint
                                        -1.57, # wrist flex joint. lower values move the hand down
                                        0.0] # wrist roll joint. 
        arm._arm._send_joint_trajectory([pre_grasp_joint_goal_upwards]) # send the command to the robot.
        arm.wait_for_motion_done()
        #only lifting the arm flex joint
        pre_grasp_joint_goal_outwards = [0.69, 
                                        -1.57,
                                        0.0, 
                                        -1.57, 
                                        0.0] 
        arm._arm._send_joint_trajectory([pre_grasp_joint_goal_outwards]) 
        # Open gripper
        arm.gripper.send_goal('open', timeout=0.0)
        arm.wait_for_motion_done()
        return 'succeeded'



class TopGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_goals": ["carrying_pose"], }

    def __init__(self, robot: Robot, arm: ArmDesignator, grab_entity: Designator) -> None:
        """
        Pick up an item given an arm and an entity to be picked up

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grasp with
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'grasp_failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

        assert self.robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None,\
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)
        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self.yolo_segmentor = YoloSegmentor()

    def execute(self, userdata=None) -> str:
        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"
        
        grasp_succeeded=False
        rate = rospy.Rate(10) # loop rate in hz      

# start segmentation
        self.yolo_segmentor.start()
        upwards = None
        time_difference = 5
        while upwards == None or time_difference > 1: #recency of data is important tehrefore the obtained data should be from within 1 seconds ago
            x_cutlery, y_cutlery, length, slope, upwards, time = self.yolo_segmentor.data_object() #obtain cutlery's center point from the detection
            time_difference = (rospy.Time.now()- time).to_sec()
        #stop segmentation after desired data has been obtained
        self.yolo_segmentor.stop()
        print('x,y,slope obtained')
        rospy.loginfo(f"direction upwards = {upwards}")
#rewrite pixel coordinate into the robot's frame
        height_table = 0.735 #manual input of table height
        height_gripper = 0.895 #value for gripper position in set pre-grasp-pose
        distance_camera =  height_gripper - height_table + 0.0045 

        #camera info
        K = np.array([
            [205.46963709898583, 0, 320.5],
            [0, 205.46963709898583, 240.5],
            [0, 0, 1]
        ])

        P = np.array([
            [205.46963709898583, 0.0, 320.5, -14.382874596929009],
            [0.0, 205.46963709898583, 240.5, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])
        T_x = P[0, 3]  # Translation along x-axis (baseline)

        # Transformation matrix from camera frame to base frame (example values)
        T_base_to_camera = np.array([
            [1, 0, 0, 0.039],#3.9 cm corrects for the offset between camera and hand palm link in x-direction
            [0, 1, 0, 0],
            [0, 0, 1, -0.0045],# 0.0045 corrects for the offset between hand palm link and the camera frame in z-direction
            [0, 0, 0, 1]
        ])

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        x_n = (x_cutlery - cx - T_x) / fx
        y_n = (y_cutlery - cy) / fy

        X_c = distance_camera * x_n
        Y_c = distance_camera * y_n

        cutlery_length = distance_camera*length /fx # this can be done since fx is equal to fy

        camera_coordinates = np.array([X_c, Y_c, distance_camera, 1])
        base_coordinates = T_base_to_camera @ camera_coordinates
        x_cutlery_real, y_cutlery_real = base_coordinates[:2]
        rospy.loginfo(f"X, Y, length = {x_cutlery_real, y_cutlery_real, cutlery_length}")
        print('frame rewritten')


#move gripper towards object's determined grasping point
        #move towards y coordinates with base
        velocity = 0.025 # desired robot speed, relatively slow since ony short distances are to be covered
        duration_y = abs(y_cutlery_real)/velocity

        v = Twist()
        v.linear.x = 0
        if y_cutlery_real > 0:
            v.linear.y = -velocity# linear left
        else:
            v.linear.y = velocity     
        v.angular.z = 0 # rotation speed to the left, none desired
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration_y:
            self.robot.base._cmd_vel.publish(v) # send command to the robot
 
        #move towards negative x coordinates with arm 
        if x_cutlery_real < 0:
            #current gripper coordinates in the base link frame, with new x-coordinate implemented
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)
            gripper_in_base_frame = tfBuffer.lookup_transform("base_link", "hand_palm_link",rospy.Time())
            rospy.loginfo(f"base_gripper_frame = {gripper_in_base_frame}")
           
            base_to_gripper = self.frame_from_xyzrpy((gripper_in_base_frame.transform.translation.x + x_cutlery_real), # x distance to the robot
                                                    gripper_in_base_frame.transform.translation.y, # y distance off center from the robot (fixed if rpy=0)
                                                    gripper_in_base_frame.transform.translation.z, # z height of the gripper
                                                    0, 1.57,0) # Roll pitch yaw. 0,1.57,0 for a downwards gripper.
            pose_goal = FrameStamped(base_to_gripper,
                                    rospy.Time.now(), #timestamp when this pose was created
                                    "base_link" # the frame in which the pose is expressed. base link lies in the center of the robot at the height of the floor.
                                    )
            arm.send_goal(pose_goal) # send the command to the robot.
            arm.wait_for_motion_done() # wait until the motion is complete

        #move towards positive x-coordinates with base                  
        else:
            duration_x = abs(x_cutlery_real)/velocity
            v.linear.x = velocity # forward
            v.linear.y = 0
            v.angular.z = 0 # rotation speed to the left, none desired

            start_time = rospy.Time.now()   
            while (rospy.Time.now() - start_time).to_sec() < duration_x:
                self.robot.base._cmd_vel.publish(v) # send command to the robot
        print('moved towards')
#rotate wrist according to orientation
        slope_rotated = 1/slope # invert the slope since the image is rotated

        # Calculate the angle
        angle = math.atan(slope_rotated)
        if angle < 0: # Normalize the angle to the range [0, pi]
            angle += math.pi 
        normalized_angle = angle / math.pi  # orientation in range [0, 1]

        #Obtain the arm's current joint positions
        joints_arm = arm._arm.get_joint_states()
        arm_lift_joint = joints_arm['arm_lift_joint']
        arm_flex_joint = joints_arm['arm_flex_joint']
        arm_roll_joint = joints_arm['arm_roll_joint']
        wrist_flex_joint = joints_arm['wrist_flex_joint']
        wrist_roll_joint = normalized_angle * -3.14 + 1.57  # Orientation in desired range [1.57, -1.57]

        if not upwards: #rotate gripper 180 degrees to grasp cutlery in the correct direction
            if wrist_roll_joint <= 0:
                wrist_roll_joint = 3.14 - abs(wrist_roll_joint)
            elif 0 < wrist_roll_joint <= 0.56:
                wrist_roll_joint = 3.14 + wrist_roll_joint
            elif 1.34 <= wrist_roll_joint < 1.57:    
                wrist_roll_joint = - 3.14 + wrist_roll_joint
            #wrist roll joint values between 1.34 and 0.56 don't have an 180 degrees turned opposite since this is out of range. 
            #therefore the gripper will go to the closest possible value which has a max inaccuarcy of 15 degrees    
            elif 0.95 <= wrist_roll_joint < 1.34:    
                wrist_roll_joint = -1.8
            elif 0.56 < wrist_roll_joint < 0.95:    
                wrist_roll_joint = 3.7

            #180 degree rotation of wrist_roll_joint if upwards = false leads to change in gripper coordinates. therefore the gripper should move backwards by 0.024 m (twice the offset between wrist_roll_link and hand_palm_link)
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)

            base_in_gripper_frame = tfBuffer.lookup_transform("hand_palm_link", "base_link", rospy.Time())
            rospy.loginfo(f"base_gripper_frame = {base_in_gripper_frame}")

            # Convert the original quaternion to a rotation matrix
            rotation_matrix = tft.quaternion_matrix([
                base_in_gripper_frame.transform.rotation.x,
                base_in_gripper_frame.transform.rotation.y,
                base_in_gripper_frame.transform.rotation.z,
                base_in_gripper_frame.transform.rotation.w
            ])

            # Desired movement direction in the gripper frame (positive direction)
            direction_in_gripper_frame = [ 1, 0, 0, 1 ] 

            # Transform the direction vector to the base frame
            direction_in_base_frame = rotation_matrix.dot(direction_in_gripper_frame)

            v = Twist()
            v.linear.x = direction_in_base_frame[1]/40 #velocity ranges from -0.025 to 0.025
            v.linear.y = direction_in_base_frame[0]/40
            v.angular.z = 0  # Assuming no rotation is needed
            duration = 0.024/math.sqrt(v.linear.x**2 + v.linear.y**2)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                self.robot.base._cmd_vel.publish(v) # send command to the robot

        #rotate wrist
        cutlery_direction_wrist_joint_goal = [arm_lift_joint,
                                    arm_flex_joint, 
                                    arm_roll_joint, 
                                    wrist_flex_joint, 
                                    wrist_roll_joint] 
        arm._arm._send_joint_trajectory([cutlery_direction_wrist_joint_goal]) # send the command to the robot.
        arm.wait_for_motion_done()
        print('wrist rotation done')        

#Moving arm downwards until force detection

        joints_arm = arm._arm.get_joint_states()
        arm_flex_joint = joints_arm['arm_flex_joint']
        arm_roll_joint = joints_arm['arm_roll_joint']
        wrist_flex_joint = joints_arm['wrist_flex_joint']
        wrist_roll_joint = joints_arm['wrist_roll_joint']
        move_arm = True

        while not grasp_succeeded and not rospy.is_shutdown():
            # control loop
            if (move_arm):
                downward_joint_goal = [0, # arm lift joint. ranges from 0.0 to 0.7m
                                       arm_flex_joint, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                                       arm_roll_joint, # arm roll joint
                                       wrist_flex_joint, # wrist flex joint. lower values move the hand down
                                       wrist_roll_joint] # wrist roll joint. 
                arm._arm._send_joint_trajectory([downward_joint_goal], max_joint_vel = 0.02) # send the command to the robot.
                #Move arm downwards, don't wait until motion is done, but until a force is detected
                try:
                    arm._arm.force_sensor.wait_for_edge_up(3.0)  # wait 3 seconds for a force detection
                except TimeOutException:
                    rospy.loginfo("No edge up detected within timeout")

                joints_arm = arm._arm.get_joint_states()
                arm_lift_joint = joints_arm['arm_lift_joint']   
                print(arm_lift_joint)    
                grasp_joint_goal = [(arm_lift_joint + 0.061), #change this in a position relative to obtained coordinates or table height
                                    arm_flex_joint, 
                                    arm_roll_joint, 
                                    wrist_flex_joint, 
                                    wrist_roll_joint]
                arm._arm._send_joint_trajectory([grasp_joint_goal]) # send the command to the robot.
                arm.wait_for_motion_done() # wait until the motion is complete
                move_arm = False # reset flag to move the arm.
                continue # dont wait for the rest of the loop.
            
            #grasp object    
            arm.gripper.send_goal('close', timeout=0.0, max_torque = 0.2) 
            arm.wait_for_motion_done() 

            rospy.sleep(5) #wait until gripper is closed before doing grasp detection
            #detecting if grasp has succeeded
            active_grasp_detector = ActiveGraspDetector(self.robot, self.arm_designator)
            grasp_detection = active_grasp_detector.execute()

            if grasp_detection == 'true':
                grasp_succeeded = True
        
            else: # for other options: false, cannot determine and failed, the grasp has not succeeded and grasp_succeeded is therefore false
                grasp_succeeded = False
                #move arm back
                pre_grasp_joint_goal_outwards = [0.69, 
                                        -1.57,
                                        0.0, 
                                        -1.57, 
                                        0.0] 
                arm._arm._send_joint_trajectory([pre_grasp_joint_goal_outwards]) 
                # Open gripper
                arm.gripper.send_goal('open', timeout=0.0)
                arm.wait_for_motion_done()
                return "grasp_failed"

        
            rospy.loginfo("The robot is holding something: {}!".format(grasp_succeeded))

# move arm up slightly to limit friction on table      
        joints_arm = arm._arm.get_joint_states()
        arm_lift_joint = joints_arm['arm_lift_joint']
        arm_flex_joint = joints_arm['arm_flex_joint']
        arm_roll_joint = joints_arm['arm_roll_joint']
        wrist_flex_joint = joints_arm['wrist_flex_joint']
        wrist_roll_joint = joints_arm['wrist_roll_joint']
        
        arm_upwards_joint_goal = [(arm_lift_joint + 0.003), # arm lift joint. ranges from 0.0 to 0.7m
                                       arm_flex_joint, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                                       arm_roll_joint, # arm roll joint
                                       wrist_flex_joint, # wrist flex joint. lower values move the hand down
                                       wrist_roll_joint] # wrist roll joint. 
        arm._arm._send_joint_trajectory([arm_upwards_joint_goal]) # send the command to the robot.      

#determine on which side of the table Hero is
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)
        
        base_in_table_frame = tfBuffer.lookup_transform("dinner_table", "base_link", rospy.Time())
        rospy.loginfo(f"base_table_frame = {base_in_table_frame}")

        #x and y coordinates of the base in the coordinate frame of the table
        x_base = base_in_table_frame.transform.translation.x
        y_base = base_in_table_frame.transform.translation.y

#manual input of table size
        table_length = 1.645
        table_width = 0.845

        #position around the table divided in four quarters: 1 = positive x side, 3 = negative x side, 2 = negative y side, 4 = positive y side
        if x_base > 1/2*table_length and (abs(x_base) - 1/2*table_length) >= (abs(y_base) - 1/2*table_width):
            position = 1
        elif y_base < -1/2*table_width and (abs(x_base) - 1/2*table_length) < (abs(y_base) - 1/2*table_width):
            position = 2
        elif x_base < -1/2*table_length and (abs(x_base) - 1/2*table_length) >= (abs(y_base) - 1/2*table_width): 
            position = 3   
        elif y_base > 1/2*table_width and (abs(x_base) - 1/2*table_length) < (abs(y_base) - 1/2*table_width):    
            position = 4

#Rotate wrist back in line with table edge  
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)

        gripper_in_table_frame = tfBuffer.lookup_transform("dinner_table", "hand_palm_link", rospy.Time())
        rospy.loginfo(f"gripper_table_frame = {gripper_in_table_frame}")

        #Convert the original quaternion to a rotation matrix
        rotation_matrix = tft.quaternion_matrix([
            gripper_in_table_frame.transform.rotation.x,
            gripper_in_table_frame.transform.rotation.y,
            gripper_in_table_frame.transform.rotation.z,
            gripper_in_table_frame.transform.rotation.w
        ])

        if position == 1: # y-axes should align 
            y_direction_gripper = rotation_matrix[:3, 1] # Extract the y-axis direction vector of the gripper in the table frame
            y_direction_gripper_xy = np.array([y_direction_gripper[0], y_direction_gripper[1], 0]) # Project the y-direction vector onto the x-y plane
            y_direction_gripper_xy /= np.linalg.norm(y_direction_gripper_xy) # Normalize
            y_axis_table_xy = np.array([0, 1, 0])

            # Calculate the cross product and dot product in the x-y plane
            cross_prod = np.cross(y_axis_table_xy, y_direction_gripper_xy)
            dot_prod = np.dot(y_axis_table_xy, y_direction_gripper_xy)   #

        if position == 2: # x-axis gripper should align with y-axis table
            x_direction_gripper = rotation_matrix[:3, 0] # Extract the x-axis direction vector of the gripper in the table frame
            x_direction_gripper_xy = np.array([x_direction_gripper[0], x_direction_gripper[1], 0]) # Project the x-direction vector onto the x-y plane
            x_direction_gripper_xy /= np.linalg.norm(x_direction_gripper_xy) # Normalize 
            y_axis_table_xy = np.array([0, 1, 0])

            # Calculate the cross product and dot product in the x-y plane
            cross_prod = np.cross(y_axis_table_xy, x_direction_gripper_xy)
            dot_prod = np.dot(y_axis_table_xy, x_direction_gripper_xy)    

        if position == 3: # x-axes should align 
            x_direction_gripper = rotation_matrix[:3, 0] # Extract the x-axis direction vector of the gripper in the table frame
            x_direction_gripper_xy = np.array([x_direction_gripper[0], x_direction_gripper[1], 0]) # Project the x-direction vector onto the x-y plane
            x_direction_gripper_xy /= np.linalg.norm(x_direction_gripper_xy) # Normalize 
            x_axis_table_xy = np.array([1, 0, 0])

            # Calculate the cross product and dot product in the x-y plane
            cross_prod = np.cross(x_axis_table_xy, x_direction_gripper_xy)
            dot_prod = np.dot(x_axis_table_xy, x_direction_gripper_xy) 

        if position == 4: # y-axis gripper should align with x-axis table
            y_direction_gripper = rotation_matrix[:3, 1] # Extract the y-axis direction vector of the gripper in the table frame
            y_direction_gripper_xy = np.array([y_direction_gripper[0], y_direction_gripper[1], 0]) # Project the y-direction vector onto the x-y plane
            y_direction_gripper_xy /= np.linalg.norm(y_direction_gripper_xy) # Normalize
            x_axis_table_xy = np.array([1, 0, 0])  

            # Calculate the cross product and dot product in the x-y plane
            cross_prod = np.cross(x_axis_table_xy, y_direction_gripper_xy)
            dot_prod = np.dot(x_axis_table_xy, y_direction_gripper_xy)      

        # Calculate the angle using atan2 for a more stable solution
        angle_to_align = np.arctan2(np.linalg.norm(cross_prod), dot_prod)

        # Determine the direction of rotation
        if cross_prod[2] < 0:
            angle_to_align = -angle_to_align

        # Ensure the angle is within the range -1.57 to 1.57 radians
        if angle_to_align > 1.57:
            angle_to_align = -3.14 + angle_to_align
        if angle_to_align < -1.57:
            angle_to_align = 3.14 - abs(angle_to_align)    
        
        joints_arm = arm._arm.get_joint_states()
        arm_lift_joint = joints_arm['arm_lift_joint']
        arm_flex_joint = joints_arm['arm_flex_joint']
        arm_roll_joint = joints_arm['arm_roll_joint']
        wrist_flex_joint = joints_arm['wrist_flex_joint']
        wrist_roll_joint = joints_arm['wrist_roll_joint'] + angle_to_align
        
        wrist_rotation_back_goal = [arm_lift_joint,
                                        arm_flex_joint, 
                                        arm_roll_joint, 
                                        wrist_flex_joint, 
                                        wrist_roll_joint] 
        arm._arm._send_joint_trajectory([wrist_rotation_back_goal]) # send the command to the robot.
        arm.wait_for_motion_done()
        rospy.sleep(5) #wait until the wrist has stopped turning
     

#Move towards table edge
        #movement of base should be in negative x direction of the gripper frame
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)

        base_in_gripper_frame = tfBuffer.lookup_transform("hand_palm_link", "base_link", rospy.Time())
        rospy.loginfo(f"base_gripper_frame = {base_in_gripper_frame}")

        # Convert the original quaternion to a rotation matrix
        rotation_matrix = tft.quaternion_matrix([
            base_in_gripper_frame.transform.rotation.x,
            base_in_gripper_frame.transform.rotation.y,
            base_in_gripper_frame.transform.rotation.z,
            base_in_gripper_frame.transform.rotation.w
        ])

        # Desired movement direction in the gripper frame (negative x direction)
        direction_in_gripper_frame = [ -1, 0, 0, 1 ] 

        # Transform the direction vector to the base frame
        direction_in_base_frame = rotation_matrix.dot(direction_in_gripper_frame)

        v = Twist()
        v.linear.x = direction_in_base_frame[0]/20 #velocity ranges from -0.05 to 0.05
        v.linear.y = direction_in_base_frame[1]/20
        v.angular.z = 0  # Assuming no rotation is needed

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)

        gripper_in_table_frame = tfBuffer.lookup_transform("dinner_table", "hand_palm_link", rospy.Time())
        rospy.loginfo(f"gripper_table_frame = {gripper_in_table_frame}")
        
        #this ensures 4 to 4.5 cm of the object sticks out over the edge. a margin of 5 mm is taken into account here for wheel slip
        distance_gripper_to_edge = cutlery_length/2 -0.045 + 0.005
        while abs(gripper_in_table_frame.transform.translation.x) < (1/2*table_length - distance_gripper_to_edge) and abs(gripper_in_table_frame.transform.translation.y) < (1/2*table_width - distance_gripper_to_edge):
            self.robot.base._cmd_vel.publish(v) # send command to the robot
            gripper_in_table_frame = tfBuffer.lookup_transform("dinner_table", "hand_palm_link", rospy.Time())

        rospy.sleep(5)
         
        #open gripper
        arm.gripper.send_goal('open', timeout=0.0)
        arm.wait_for_motion_done()  

        #move farther away until grasp distance from edge o grasp at 2cm from cutlery's outer end (half gripper width)
        duration = ((cutlery_length/2)-0.02)/math.sqrt(v.linear.x**2 + v.linear.y**2)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.robot.base._cmd_vel.publish(v) # send command to the robot
        
#rotate wrist around the cutlery, towards pre-grasp pose for a sideways grasp
        #determine towards which side hero will grasp the object
        x_gripper = gripper_in_table_frame.transform.translation.x,
        y_gripper = gripper_in_table_frame.transform.translation.y
        side = None
        if (position == 1 and y_gripper >= (y_base - 0.078)) or (position == 2 and x_gripper >= (x_base - 0.078)) or (position == 3 and y_gripper <= (y_base + 0.078)) or (position == 4 and x_gripper <= (x_base + 0.078)):
            side = 'right' #gripper is right of base              
        else:
            side = 'left' #gripper is left of base
        print(side, position)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)

        base_in_gripper_frame = tfBuffer.lookup_transform("hand_palm_link", "base_link", rospy.Time())
        rospy.loginfo(f"base_in_gripper_frame = {base_in_gripper_frame}")

        #Convert the original quaternion to a rotation matrix
        rotation_matrix = tft.quaternion_matrix([
            base_in_gripper_frame.transform.rotation.x,
            base_in_gripper_frame.transform.rotation.y,
            base_in_gripper_frame.transform.rotation.z,
            base_in_gripper_frame.transform.rotation.w
        ])

        x_direction_base = rotation_matrix[:3, 0] # Extract the x-axis direction vector of the gripper in the table frame
        negative_x_direction_base_xy = np.array([-x_direction_base[0], -x_direction_base[1], 0]) # Project the x-direction vector onto the x-y plane
        negative_x_direction_base_xy /= np.linalg.norm(negative_x_direction_base_xy) # Normalize 
        x_axis_gripper_xy = np.array([1, 0, 0])
        
        #negative x-axis base should align with x-axis gripper
        cross_prod = np.cross(x_axis_gripper_xy, negative_x_direction_base_xy)
        dot_prod = np.dot(x_axis_gripper_xy, negative_x_direction_base_xy)
        angle_to_align = np.arctan2(np.linalg.norm(cross_prod), dot_prod)          

        # Determine the direction of rotation
        if cross_prod[2] < 0:
            angle_to_align = -angle_to_align
        rotation_y = 1 - abs(angle_to_align)/1.57
        rotation_x = abs(angle_to_align)/1.57
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rospy.sleep(1)

        base_in_gripper_frame = tfBuffer.lookup_transform("hand_palm_link", "base_link", rospy.Time())
        rospy.loginfo(f"base_gripper_frame = {base_in_gripper_frame}")

        # Convert the original quaternion to a rotation matrix
        rotation_matrix = tft.quaternion_matrix([
            base_in_gripper_frame.transform.rotation.x,
            base_in_gripper_frame.transform.rotation.y,
            base_in_gripper_frame.transform.rotation.z,
            base_in_gripper_frame.transform.rotation.w
        ])

        # Desired movement direction in the gripper frame (positive direction)
        direction_in_gripper_frame = [ 0, 1, 0, 1 ] 

        # Transform the direction vector to the base frame
        direction_in_base_frame = rotation_matrix.dot(direction_in_gripper_frame)

        if side == 'right':
            v = Twist()
            v.linear.x = direction_in_base_frame[1]/20 #velocity ranges from -0.05 to 0.05
            v.linear.y = direction_in_base_frame[0]/20
            v.angular.z = 0  # Assuming no rotation is needed
            duration = 0.2305/math.sqrt(v.linear.x**2 + v.linear.y**2)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                self.robot.base._cmd_vel.publish(v) # send command to the robot
            rospy.sleep(5) #wait until base is done moving

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)
            gripper_position_before= tfBuffer.lookup_transform("dinner_table", "hand_palm_link",rospy.Time())
            rospy.loginfo(f"gripper_position_before = {gripper_position_before}")

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)
            gripper_in_base_frame = tfBuffer.lookup_transform("base_link", "hand_palm_link",rospy.Time())
            rospy.loginfo(f"base_gripper_frame = {gripper_in_base_frame}")


            base_to_gripper = self.frame_from_xyzrpy((gripper_in_base_frame.transform.translation.x + 0.1405*rotation_x), # x distance to the robot
                                                (gripper_in_base_frame.transform.translation.y - 0.1405*rotation_y), # y distance off center from the robot (fixed if rpy=0)
                                                (gripper_in_base_frame.transform.translation.z - 0.095), # z height of the gripper
                                                1.57, 0, (-1.57 - angle_to_align))

            pose_goal = FrameStamped(base_to_gripper,
                                rospy.Time.now(), #timestamp when this pose was created
                                "base_link" # the frame in which the pose is expressed. base link lies in the center of the robot at the height of the floor.
                                )
            arm.send_goal(pose_goal) # send the command to the robot.
            arm.wait_for_motion_done() # wait until the motion is complete

        if side == 'left':
            v = Twist()
            v.linear.x = direction_in_base_frame[1]/20 #velocity ranges from -0.05 to 0.05
            v.linear.y = -direction_in_base_frame[0]/20
            v.angular.z = 0  # Assuming no rotation is needed
            duration = 0.2305/math.sqrt(v.linear.x**2 + v.linear.y**2)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                self.robot.base._cmd_vel.publish(v) # send command to the robot
            rospy.sleep(5) #wait until base is done moving

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)
            gripper_position_before= tfBuffer.lookup_transform("dinner_table", "hand_palm_link",rospy.Time())
            rospy.loginfo(f"gripper_position_before = {gripper_position_before}")

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rospy.sleep(1)
            gripper_in_base_frame = tfBuffer.lookup_transform("base_link", "hand_palm_link",rospy.Time())
            rospy.loginfo(f"base_gripper_frame = {gripper_in_base_frame}")


            base_to_gripper = self.frame_from_xyzrpy((gripper_in_base_frame.transform.translation.x+ 0.1405*rotation_x), # x distance to the robot
                                                (gripper_in_base_frame.transform.translation.y + 0.1405*rotation_y), # y distance off center from the robot (fixed if rpy=0)
                                                (gripper_in_base_frame.transform.translation.z -0.095), # z height of the gripper
                                                -1.57, 0, (1.57 - angle_to_align))

            pose_goal = FrameStamped(base_to_gripper,
                                rospy.Time.now(), #timestamp when this pose was created
                                "base_link" # the frame in which the pose is expressed. base link lies in the center of the robot at the height of the floor.
                                )
            arm.send_goal(pose_goal) # send the command to the robot.
            arm.wait_for_motion_done() # wait until the motion is complete


#close gripper 
        arm.gripper.send_goal('close', timeout=0.0, max_torque = 0.3) # option given by max_torque to close the gripper with more force
        arm.wait_for_motion_done()

        #detecting if grasp has succeeded
        rospy.sleep(5)
        active_grasp_detector = ActiveGraspDetector(self.robot, self.arm_designator)
        grasp_detection = active_grasp_detector.execute()


        if grasp_detection == 'true':
            grasp_succeeded = True
        else: # for other options: false, cannot determine and failed, the grasp has not succeeded and grasp_succeeded is therefore false
            grasp_succeeded = False
            return "failed"
        
        rospy.loginfo("The robot is holding something: {}!".format(grasp_succeeded))


#lift up, go to original stance



        return "succeeded"

    @staticmethod
    def frame_from_xyzrpy(x, y, z, roll, pitch, yaw):
        """
        Helper function to create a kdl frame based on an x,y,z position and a RPY rotation
        """
        return kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x, y, z))


class ResetOnFailure(smach.State):
    """ Class to reset the robot after a grab has failed """

    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot, arm):
        """
        Constructor

        :param robot: robot object
        :param arm: arm designator
        """
        smach.State.__init__(self, outcomes=['done'])

        self._robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None):
        """ Execute hook """
        arm = self.arm_designator.resolve()
        arm.reset()

        if self._robot.robot_name == "hero":
            self._robot.torso.reset()  # Move up to make resetting of the arm safer.
        if arm is not None:
            arm.gripper.send_goal('open')
        self._robot.head.reset()  # Sends a goal
        self._robot.head.cancel_goal()  # And cancels it...
        if arm is not None:
            arm.reset()
        self._robot.torso.reset()
        return 'done'


class TopGrab(smach.StateMachine):
    def __init__(self, robot: Robot, item: Designator, arm: ArmDesignator):
        """
        Let the given robot move to an entity and grab that entity using some arm

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. E.g. UnoccupiedArmDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(arm, PublicArm)

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'NAVIGATE_TO_GRAB',
                                                'failed': 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, arm, item),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PREPARE_GRASP'})

            smach.StateMachine.add('PREPARE_GRASP', PrepareGrasp(robot, arm),
                                   transitions={'succeeded': 'GRAB',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('GRAB', TopGrasp(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE',
                                                'grasp_failed': 'GRAB'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})

        check_arm_requirements(self, robot)