# ROS
import rospy
import sys

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states.util import designators as ds
from robot_smach_states import SegmentObjects

from robot_skills.classification_result import ClassificationResult


if __name__ == "__main__":
    assert len(sys.argv) == 3, "Please provide the robot name and the entity on which to segment objects" \
                               "e.g., 'python example_something.py hero dinner_table'"

    rospy.init_node("example_segment_objects")

    robot = get_robot_from_argv(index=1)
    location_name = sys.argv[2]

    rospy.loginfo("Creating location designator")
    location_designator = ds.EntityByIdDesignator(robot, location_name)

    rospy.loginfo("Creating found entity designator")
    segmented_entities_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

    rospy.loginfo("Creating segment objects state")
    sm = SegmentObjects(robot, segmented_entities_designator.writeable, location_designator, "on_top_of")

    rospy.loginfo("Executing segment objects state")
    sm.execute()

    rospy.loginfo("Resulting classification result: \n {}".format(segmented_entities_designator.resolve()))

    rospy.loginfo("Done")
