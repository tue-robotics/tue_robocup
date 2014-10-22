import rospy
import arms
import visualization_msgs.msg
import threading
import util.concurrent_util

##intialize as a static class
actionClients = arms.ArmActionClients()

class DummyArms(arms.Arms):
    def __init__(self, tf_listener):
        super(DummyArms, self).__init__(tf_listener)

    joint_names = ['shoulder_yaw_joint_{side}', 'shoulder_pitch_joint_{side}', 'shoulder_roll_joint_{side}', 'elbow_pitch_joint_{side}', 'elbow_roll_joint_{side}', 'wrist_pitch_joint_{side}', 'wrist_yaw_joint_{side}']

    SHOULDER_YAW = 0
    SHOULDER_PITCH =1 
    SHOULDER_ROLL = 2
    ELBOW_PITCH = 3
    ELBOW_ROLL = 4
    WRIST_PITCH = 5
    WRIST_YAW = 6

    POINT_AT_OBJECT_BACKWARD = [-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000]
    POINT_AT_OBJECT_FORWARD = [-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000]
    HOLD_TRAY_POSE = [-0.1, 0.13, 0.4, 1.5, 0, 0.5, 0]
    SUPPORT_PERSON_POSE = [-0.1, -1.57, 0, 1.57, 0,0,0]
    RESET_POSE = [-0.1,-0.2,0.2,0.8,0.0,0.0,0.0] # This is the usual

    def close(self):
        actionClients.close()

    def send_goal(self, px, py, pz, roll, pitch, yaw, timeout=30, side=None, pre_grasp = False, frame_id = '/amigo/base_link', use_offset = False, first_joint_pos_only=False):
        return False
    
    def send_delta_goal(self, px, py, pz, roll, pitch, yaw, timeout=30, side=None, pre_grasp = False, frame_id = '/amigo/base_link', use_offset = False, first_joint_pos_only=False):
        return False
     
    ################################# function cancel arm right goal ############################
    def cancel_goal(self, side=None):
        pass
    
    def __cancel_right_goal(self):
        pass
    
    def __cancel_left_goal(self):
        pass
    
    ################################# function send gripper goal ############################
    
    def send_gripper_goal_open(self, side, timeout=10):
        return False
        
    def send_gripper_goal_close(self, side, timeout=10):
        return False
    
    def send_gripper_goal(self, state, side, timeout=10):
        return False
        
    def check_gripper_content(self, side):
        return False

    def send_joint_goal(self, q1=None, q2=None, q3=None, q4=None, q5=None, q6=None, q7=None, side=None, timeout=0):
        return False

    def send_joint_trajectory(self, joint_positions, side=None, timeout=0):
        return False

    def send_delta_joint_goal(self, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0, q7=0, side=None, timeout=0):
        return False
    
    def update_correction(self, side=None):
        rospy.logerr("Function currently not implemented")
        return False
        
    def get_pose(self, root_frame_id, side=None):
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

    _lock = threading.RLock()

    @util.concurrent_util.synchronized(_lock)
    def _receive_arm_left_joints(self, jointstate):
        self._joint_pos[Side.LEFT] = jointstate.position

    @util.concurrent_util.synchronized(_lock)
    def _receive_arm_right_joints(self, jointstate):
        self._joint_pos[Side.RIGHT] = jointstate.position

def add_side_argument(ArmMethod):
    def wrapper(self, *args, **kwargs):
        print "add_side_argument.wrapper"
        kwargs["side"] = self.side
        return ArmMethod(self, *args, **kwargs)
    return wrapper

