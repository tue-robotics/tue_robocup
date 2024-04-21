import rospy
import tf2_ros
import geometry_msgs.msg

from pykdl_ros import FrameStamped
from robot_skills.robot import Robot
from robot_smach_states.util.designators.arm import ArmDesignator

class GripperTfListener:
    def __init__(self, arm: ArmDesignator) -> None:
        self.arm_designator = arm

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.gripper_coordinates = rospy.Publisher('/gripper_coordinates', geometry_msgs.msg.PoseStamped, queue_size=1)
    
        self.rate = rospy.Rate(10.0)
    
    def listening(self):
        while not rospy.is_shutdown():
            
            try:
                gripper_bl = self.tfBuffer.lookup_transform('hand_palm_link', 'base_link', rospy.Time())
                arm = self.arm_designator.resolve()
                gripper_bl.frame = arm._arm.offset.Inverse() * gripper_bl.frame  # compensate for the offset in hand palm link

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            # PoseStamped coordinates
            msg = geometry_msgs.msg.PoseStamped()
            msg.header = gripper_bl.header
            msg.pose.position.x = gripper_bl.transform.translation.x
            msg.pose.position.y = gripper_bl.transform.translation.y
            msg.pose.position.z = gripper_bl.transform.translation.z
            msg.pose.orientation = gripper_bl.transform.rotation


            self.gripper_coordinates.publish(msg)

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tf2_gripper_listener')   
    gripper_tf_listener = GripperTfListener(arm)
    gripper_tf_listener.listening()     

