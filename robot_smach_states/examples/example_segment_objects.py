# ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.util import designators as ds
from robot_smach_states.world_model import SegmentObjects

# Robot_skills
from robot_skills.classification_result import ClassificationResult


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test if it can segment objects")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("Entity", default="dinner_table", help="Name of the entity on which to segment objects")

    args = parser.parse_args()

    rospy.init_node("example_segment_objects")

    robot = get_robot(args.robot)
    location_name = args.Entity

    rospy.loginfo("Creating location designator")
    location_designator = ds.EdEntityDesignator(robot, uuid = location_name)
    rospy.loginfo("Creating found entity designator")
    segmented_entities_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

    rospy.loginfo("Creating segment objects state machine")
    sm = SegmentObjects(robot, segmented_entities_designator.writeable, location_designator,
                        segmentation_area="on_top_of", unknown_threshold=0.0, filter_threshold=0.0)

    rospy.loginfo("Executing segment objects state machine")

    sm.execute()

    rospy.loginfo("Resulting classification result: \n {}".format(segmented_entities_designator.resolve()))

    rospy.loginfo("Done")
