import rospy

from robot_skills.arm.grasp_detector import GraspDetector

wrench_topic = "/hero/wrist_wrench/raw"  # topic to listen to
gd = GraspDetector("Hero", None, wrench_topic)

rospy.init_node("example_grasp_detector")

while not rospy.is_shutdown():
    raw_input("Press enter to query the grasp detector")
    if gd.detect():
        print("robot is currently holding something!!!")
    else:
        print("Not holding anything :(")
