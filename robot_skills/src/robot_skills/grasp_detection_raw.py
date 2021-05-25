import rospy
from geometry_msgs.msg import WrenchStamped
from .robot_smach_states.manipulation.grab import execute, PickUp


def callback(data):
    rospy.loginfo("I felt: \n %.s", data.wrench)

    # Threshold for the torque around y axis in [Nm]
    threshold_torque_y = -0.5 # MUST BE CHECKED

    torque_y_matrix = []

    if rospy.loginfo("Start lift"):  # check this condition!

        start_measuring_time = data.header.stamp.sec + data.header.stamp.nsec * 10 ** -9

        if not rospy.loginfo("Grasping motion completed"):  # check this condition!
            torque_y_matrix.append(data.wrench.torque.y)

        else:  # check this condition!
            end_measuring_time = data.header.stamp.sec + data.header.stamp.nsec * 10 ** -9
            tot_measuring_time = end_measuring_time - start_measuring_time

            mean_torque_y = mean(torque_y_matrix)
            if mean_torque_y <= threshold_torque_y:
                grasp = True
            else:
                grasp = False
        # if data.wrench.torque.y <= threshold_torque_y:
        #     object_present1 = True
        #     print('Initial grasp: SUCCESSFUL')
        #
        #     time.sleep(2)
        #
        #     if data.wrench.torque.y <= threshold_torque_y:
        #         object_present2 = True
        #         print('Total grasp: SUCCESSFUL')
        #     else:
        #         object_present2 = False
        #         print('Initial grasp: SUCCESSFUL, total grasp: FAILED')
        #
        # else:
        #     object_present1 = False
        #     print('Initial and total grasp: FAILED')
        #
        # return object_present1 and object_present2


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hero/wrist_wrench/raw", WrenchStamped, callback)


if __name__ == "__main__":
    l = listener()
    rospy.spin()
