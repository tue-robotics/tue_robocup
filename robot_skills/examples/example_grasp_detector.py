import rospy

from robot_skills.arm.grasp_detector import GraspDetector

rospy.init_node("example_grasp_detector")

wrench_topic = "/hero/wrist_wrench/raw"  # topic to listen to
gd = GraspDetector("Hero", None, wrench_topic)

while not rospy.is_shutdown():
    input("Press enter to query the grasp detector")
    if gd.detect():
        print("robot is currently holding something!!!", "Last T_y: {}".format(gd.msg_list[-1]))
    else:
        print("Not holding anything :(", "Last T_y: {}".format(gd.msg_list[-1]))
