import rospy

from robot_skills.arm.grasp_detector import GraspDetector

rospy.init_node("example_grasp_detector")

wrench_topic = "/hero/wrist_wrench/raw"  # topic to listen to
gd = GraspDetector("Hero", None, wrench_topic)

while not rospy.is_shutdown():
    input("Press enter to query the grasp detector")
    if gd.detect():
        print("robot is currently holding something!!! , ", "Torque y: {}".format(gd.msg_list[0:4]), gd.threshold_torque_y)
    else:
        print("Not holding anything :( , ", "Torque y: {}".format(gd.msg_list[0:4]), gd.threshold_torque_y)
