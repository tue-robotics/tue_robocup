#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
from tue_manipulation.msg._GraspPrecomputeGoal import GraspPrecomputeGoal
import actionlib
#from tue_manipulation.msg._MoveArmAction import MoveArmAction
from actionlib_msgs.msg._GoalStatus import GoalStatus
import amigo_actions
import amigo_actions.msg
from tue_manipulation.msg._GraspPrecomputeAction import GraspPrecomputeAction
from geometry_msgs.msg import TwistStamped, Twist, Quaternion

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

# Whole-body control/planning
#from amigo_whole_body_controller.msg._ArmTaskAction import ArmTaskAction
#from amigo_whole_body_controller.msg._ArmTaskGoal import ArmTaskGoal

import threading
import util.concurrent_util

"""TF"""
import tf
from tf.transformations import euler_from_quaternion
import tf_server

"""Marker publisher"""
import visualization_msgs.msg

from math import degrees, radians
    
class State:
    """Specifies a State either OPEN or CLOSE"""
    OPEN = 0
    CLOSE = 1

class Offset(object):
    def __init__(self, parameter_name):
        params = rospy.get_param(parameter_name)
        self.x     = params['x'] 
        self.y     = params['y']
        self.z     = params['z']
        self.roll  = params['roll']
        self.pitch = params['pitch']
        self.yaw   = params['yaw']

class Arm(object):
    """
    A single arm can be either left or right, extends Arms:
    Use left or right to get arm while running from the python console
    
    Examples:
    >>> left.send_goal(0.265, 1, 0.816, 0, 0, 0, 60)
    or Equivalently:
    >>> left.send_goal(px=0.265, py=1, pz=0.816, yaw=0, pitch=0, roll=0, time_out=60, pre_grasp=False, frame_id='/amigo/base_link')
    
    #To open left gripper
    >>> left.send_gripper_goal_open(10)
    """
    def __init__(self, robot_name, side, tf_listener):
        self.robot_name = robot_name
        self.side = side
        if (self.side is "left") or (self.side is "right"):
            pass
        else:
            raise Exception("Side should be either: left or right")
        self.tf_listener = tf_listener

        self.occupied = False

        # Get stuff from the parameter server
        self.offset = Offset('/'+self.robot_name+'/skills/arm/offset/'+self.side)
        self.marker_to_grippoint_offset = Offset('/'+self.robot_name+'/skills/arm/offset/marker_to_grippoint/')
        self.joint_names = rospy.get_param('/'+self.robot_name+'/skills/arm/joint_names')
        for i in range(len(self.joint_names)):
            self.joint_names[i] = self.joint_names[i] + "_" + self.side
        print self.joint_names
        self.default_configurations = rospy.get_param('/'+self.robot_name+'/skills/arm/default_configurations')
        print self.default_configurations

        # Init measurement subscriber
        self.arm_measurement_sub = rospy.Subscriber("/"+self.robot_name+"/left_arm/measurements", JointState, self._arm_measurement_callback)

        # Init gripper actionlib
        self._ac_gripper = actionlib.SimpleActionClient("gripper_server_"+self.side, amigo_actions.msg.AmigoGripperCommandAction)

        # Init graps precompute actionlib
        self._ac_grasp_precompute = actionlib.SimpleActionClient("grasp_precompute_"+self.side, GraspPrecomputeAction)
        
        # Init joint trajectory action server
        self._ac_joint_traj = actionlib.SimpleActionClient("/joint_trajectory_action_"+self.side,  FollowJointTrajectoryAction)

        # ToDo: don't hardcode?
        server_timeout = 1.0
        if not self._ac_gripper.wait_for_server(timeout=rospy.Duration(server_timeout)):
            rospy.logwarn("Cannot find gripper {0} server".format(self.side))
        elif not self._ac_grasp_precompute.wait_for_server(timeout=rospy.Duration(server_timeout)):
            rospy.logwarn("Cannot find grasp precompute {0} server".format(self.side))
        elif not self._ac_joint_traj.wait_for_server(timeout=rospy.Duration(server_timeout)):
            rospy.logwarn("Cannot find joint trajectory action server {0}".format(self.side))

        # Init marker publisher
        self._marker_publisher = rospy.Publisher("grasp_target", visualization_msgs.msg.Marker)

    def close(self):
        try:
            rospy.loginfo("{0} arm cancelling all goals on all arm-related ACs on close".format(self.side))
        except AttributeError:
            print "Arm cancelling all goals on all arm-related ACs on close. Rospy is already deleted."
        
        self._ac_gripper.cancel_all_goals()
        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def send_goal(self, px, py, pz, roll, pitch, yaw, 
        timeout=30, 
        pre_grasp = False, 
        frame_id = '/base_link', 
        use_offset = False, 
        first_joint_pos_only=False):
        """Send a arm to a goal: 
        Using a position px,py,pz. An orientation roll,pitch,yaw. A time out time_out. And a side Side.LEFT or Side.RIGHT
        
        Optional parameters are if a pre_grasp should be performed and a frame_id which defaults to base_link """

        # If necessary, prefix frame_id
        if frame_id.find(self.robot_name) < 0:
            frame_id = "/"+self.robot_name+frame_id
            rospy.loginfo("Grasp precompute frame id = {0}".format(frame_id))
        
        # Create goal:
        grasp_precompute_goal = GraspPrecomputeGoal()
        grasp_precompute_goal.goal.header.frame_id = frame_id
        grasp_precompute_goal.goal.header.stamp = rospy.Time.now()
        
        grasp_precompute_goal.PERFORM_PRE_GRASP = pre_grasp
        grasp_precompute_goal.FIRST_JOINT_POS_ONLY = first_joint_pos_only
        
        grasp_precompute_goal.goal.x = px + self.offset.x
        grasp_precompute_goal.goal.y = py + self.offset.y
        grasp_precompute_goal.goal.z = pz + self.offset.z
        
        grasp_precompute_goal.goal.roll  = roll  + self.offset.roll
        grasp_precompute_goal.goal.pitch = pitch + self.offset.pitch
        grasp_precompute_goal.goal.yaw   = yaw   + self.offset.yaw
        
        #rospy.loginfo("Arm goal: {0}".format(grasp_precompute_goal))
        
        self._publish_marker(grasp_precompute_goal, "red")
    
        # Send goal:
        self._ac_grasp_precompute.send_goal(grasp_precompute_goal)
        if timeout == 0.0:
            return True
        else:
            self._ac_grasp_precompute.wait_for_result(rospy.Duration(timeout))
            if self._ac_grasp_precompute.get_state() == GoalStatus.SUCCEEDED:
                return True
            else:
                return False

    def send_joint_goal(self, configuration):
        if configuration in self.default_configurations:
            return self._send_joint_goal(self.default_configurations[configuration])
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def reset(self):
        return self.send_joint_goal('reset')

    def _send_joint_goal(self, joint_references, timeout=0):
        """Send a goal to the arms in joint coordinates, using an action client"""
        p = JointTrajectoryPoint()
        p.positions = joint_references

        if (len(joint_references) != len(self.joint_names)):
            rospy.logwarn('Please use the correct {0} number of joint references (current = {1}'.format(len(self.joint_names),len(joint_references)))
                
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory.points = [p]
        traj_goal.trajectory.joint_names = self.joint_names

        rospy.loginfo("Send {0} arm to jointcoords {1}".format(self.side,p.positions))
        
        self._ac_joint_traj.send_goal(traj_goal)
        if timeout == 0.0:
            return True
        else:
            self._ac_joint_traj.wait_for_result(rospy.Duration(timeout))
            if current_ac.get_state() == GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.logwarn("Cannot reach joint goal {0}".format(traj_goal))
                return False

    def _publish_marker(self, goal, color):
        
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = goal.goal.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.pose.position.x = goal.goal.x
        marker.pose.position.y = goal.goal.y
        marker.pose.position.z = goal.goal.z
        marker.lifetime = rospy.Duration(5.0)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1;
        if color == "red":
            marker.color.r = 1
        elif color == "blue":
            marker.color.b = 1

        self._marker_publisher.publish(marker)

    # ToDo: why is this here???
    _lock = threading.RLock()

    @util.concurrent_util.synchronized(_lock)
    def _arm_measurement_callback(self, jointstate):
        self._joint_pos = jointstate.position
    
    # @add_side_argument
    # def send_goal(self, *args, **kwargs):
    #     """Send arm to a goal: using a position px,py,pz and orientation roll,pitch,yaw and a timeout
    #     Optional parameters are if a pre_grasp should be performed and a frame_id which defaults to /amigo/base_link """
    #     return super(Arm, self).send_goal(*args, **kwargs)
    
    # @add_side_argument
    # def send_delta_goal(self, *args, **kwargs):
    #     """Send arm to an offset with respect to current position: using a position px,py,pz and orientation roll,pitch,yaw and a time out time_out
    #     Optional parameters are if a pre_grasp should be performed and a frame_id which defaults to /amigo/base_link """        
    #     return super(Arm, self).send_delta_goal(*args, **kwargs)
        
    # def send_joint_goal(self, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0, q7=0, timeout=0):
    #     """Send a goal to the arms in joint coordinates"""
    #     return super(Arm, self).send_joint_goal(q1,q2,q3,q4,q5,q6,q7,self.side, timeout=timeout)
    
    # def send_joint_trajectory(self, positions, timeout=0):
    #     """Send a goal to the arms in joint coordinates"""
    #     return super(Arm, self).send_joint_trajectory(positions,self.side, timeout=timeout)
    
    # def send_delta_joint_goal(self, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0, q7=0, timeout=0):
    #     """Move the arm joints by some angle (in radians)
    #     >>> from math import radians
    #     >>> some_arm.send_delta_joint_goal(q1=radians(-20)) #e.g. amigo.leftArm.send_delta_joint_goal(q1=radians(-20))"""
    #     return super(Arm, self).send_delta_joint_goal(q1,q2,q3,q4,q5,q6,q7,self.side, timeout=timeout)
    
    # def send_delta_joint_trajectory(self, delta_dict_list, timeout=5.0, origin=None):
    #     """@param delta_dict_list is a list of dictionaries with deltas per joint, per step, e.g. [{q1=-0.1, q4=0.4}, {q6=1.57}]
    #     @param origin The joint position list to start from, in order to optionally have a defined start. If empty, uses the current position"""
    #     return super(Arm, self).send_delta_joint_trajectory(delta_dict_list,self.side, timeout=timeout, origin=origin)

    # def send_arm_task(self, *args, **kwargs):
    #     """Send a goal to the whole-body planner"""
    #     return super(Arm, self).send_arm_task(*args, **kwargs)

    # def reset_arm(self):
    #     """Send the arm to a suitable (natural looking) (driving) position"""
    #     #return super(Arm, self).send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0,self.side)
    #     return super(Arm, self).send_joint_goal(side=self.side,*(self.RESET_POSE))
    
    # def cancel_goal(self):
    #      """Cancel arm goal """
    #      return super(Arm, self).cancel_goal(self.side)
    
    # def send_gripper_goal(self, state, timeout=10):
    #     """Generic open or close gripper goal method. Expects a state: State.OPEN or State.CLOSE and a time_out"""
    #     return super(Arm, self).send_gripper_goal(state, self.side, timeout)
    
    # def send_gripper_goal_open(self, timeout=10):
    #      """ Open gripper, expects a time_out parameter"""
    #      return self.send_gripper_goal(State.OPEN, timeout)
    
    # def send_gripper_goal_close(self,  timeout=10):
    #      """ Close gripper, expects a time_out parameter"""
    #      return self.send_gripper_goal(State.CLOSE, timeout)
     
    # def check_gripper_content(self):
    #     """ Check if the gripper has successfully grasped something """
    #     return super(Arm, self).check_gripper_content(self.side)
    
    # def send_twist(self, twist, duration):
    #     super(Arm, self).send_twist(twist, duration, self.side)
        
    # def update_correction(self):
    #     """ Update correction factor """
    #     return super(Arm, self).update_correction(self.side)

    
    # def get_pose(self, root_frame_id):
    #     """ Get the pose of the end-effector with respect to the specified root_frame_id"""
    #     return super(Arm, self).get_pose(root_frame_id,self.side)

    # @property
    # def joint_pos(self):
    #     """The joint positions for all joints. Index individual joints via Arms.SHOULDER_..., ELBOW_.. and WRIST_..."""
    #     return self._joint_pos[self.side]
        

if __name__ == "__main__":
    rospy.init_node('amigo_arms_executioner', anonymous=True)
    #Easy enum access
    leftSide = Side.LEFT
    rightSide = Side.RIGHT
    
    openState = State.OPEN
    closeState = State.CLOSE
    
    tf_listener = tf_server.TFClient()

    arms = Arms(tf_listener)
    left = Arm(leftSide, tf_listener)
    right = Arm(rightSide, tf_listener)
    
