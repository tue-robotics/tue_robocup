import rospy
import tf2_ros
import geometry_msgs.msg

from pykdl_ros import FrameStamped
from robot_skills.robot import Robot
from robot_smach_states.util.designators.arm import ArmDesignator

if __name__ == '__main__':
    rospy.init_node('gripper_tf2_listener')

    robot = Robot()
    arm = ArmDesignator()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    gripper_coordinates = rospy.Publisher('/gripper_coordinates', geometry_msgs.msg.PoseStamped, queue_size=1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            gripper_bl = tfBuffer.lookup_transform('hand_palm_link', 'base_link', rospy.Time(0))
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


        gripper_coordinates.publish(msg)

        rate.sleep()