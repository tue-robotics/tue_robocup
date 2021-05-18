import rospy
from geometry_msgs.msg import WrenchStamped

import time


def callback(data):
    rospy.loginfo("I felt: \n %.s", data.wrench)
    threshold_torque_y = 0.1  # Threshold for the torque around y axis in [Nm]
    object_present1 = False
    object_present2 = False

    if data.wrench.torque.y >= threshold_torque_y:
        object_present1 = True
        print('Initial grasp: SUCCESSFUL')

        time.sleep(2)

        if data.wrench.torque.y >= threshold_torque_y and object_present1 == True:
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
