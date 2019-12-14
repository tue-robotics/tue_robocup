import argparse
import pickle
import rospy
import smach
from challenge_final import FindPeople
from challenge_final.find_people import _filter_and_cluster_images
from robot_skills import get_robot


if __name__ == "__main__":

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--robot", default="hero", help="Name of the robot you want to use")
    parser.add_argument("--room_id", default="living_room", help="Room where this stuff takes place")
    args = parser.parse_args()

    rospy.init_node("test_furniture_inspection")

    # Robot
    robot = get_robot(args.robot)

    # Test data
    user_data = smach.UserData()

    with open('/home/amigo/Downloads/floorplan-2019-07-05-12-02-17.pickle', 'r') as f:
        raw_detections = pickle.load(f)
        rospy.loginfo("Loaded %d persons", len(raw_detections))

    detected_people = _filter_and_cluster_images(robot, raw_detections, args.room_id)

    for idx, person in enumerate(detected_people):
        rospy.loginfo("Person {} at \n{}".format(idx, person["map_ps"].point))
