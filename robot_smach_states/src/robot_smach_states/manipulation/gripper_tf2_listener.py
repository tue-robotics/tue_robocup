import rospy
import tf2_ros
import geometry_msgs.msg
from pykdl_ros import FrameStamped

if __name__ == '__main__':
    rospy.init_node('tf2_gripper_listener')   

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    gripper_coordinates = rospy.Publisher('/gripper_coordinates', geometry_msgs.msg.PoseStamped, queue_size=1)
    
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            gripper_id = FrameStamped.from_xyz_rpy(0, 0, 0, 0, 0, 0, rospy.Time(), frame_id="hand_palm_link") #generates the frame of the gripper 
            gripper_bl = tfBuffer.lookup_transform("hand_palm_link", "base_link", rospy.Time()) #calculates the transform between the origin of the base frame and that of the gripper frame
            
            # PoseStamped coordinates
            # obtains coordinates and orientation of the gripper transformed into the coordinate frame of the robot's base
            msg = geometry_msgs.msg.PoseStamped()
            msg.header = gripper_bl.header
            msg.pose.position.x = gripper_bl.transform.translation.x
            msg.pose.position.y = gripper_bl.transform.translation.y
            msg.pose.position.z = gripper_bl.transform.translation.z
            msg.pose.orientation = gripper_bl.transform.rotation

            # publishing coordinates
            gripper_coordinates.publish(msg)

    
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
            rate.sleep()
            continue

    rate.sleep()




  