import rospy
from geometry_msgs.msg import WrenchStamped
from .robot_smach_states.manipulation.grab import execute, PickUp
import time


def callback(data):
    rospy.loginfo("I felt: \n %.s", data.wrench)

    # Threshold for the torque around y axis in [Nm]
    threshold_torque_y = -0.1

    # Start the measuring of torque_y
    start_measuring = 

    object_present1 = False
    object_present2 = False

    if data.wrench.torque.y <= threshold_torque_y:
        object_present1 = True
        print('Initial grasp: SUCCESSFUL')

        time.sleep(2)

        if data.wrench.torque.y <= threshold_torque_y:
            object_present2 = True
            print('Total grasp: SUCCESSFUL')
        else:
            object_present2 = False
            print('Initial grasp: SUCCESSFUL, total grasp: FAILED')

    else:
        object_present1 = False
        print('Initial and total grasp: FAILED')

    return object_present1 and object_present2


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hero/wrist_wrench/compensated", WrenchStamped, callback)


if __name__ == "__main__":
    l = listener()
    rospy.spin()
