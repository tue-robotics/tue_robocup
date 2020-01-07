# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states import Grab, Inspect, ClassificationResult, NavigateToWaypoint

try:
    from typing import List
except ImportError:
    pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Measure the time it takes to grasp an entity when starting from a given pose. "
                                                 "After the grasping is done, go back to the waypoint, "
                                                 "turn around and drop the item, "
                                                 "then turn around once more to be in the start configuration again")
    parser.add_argument("cls", type=str, help="class of entity to grasp (e.g. 'coke'")
    parser.add_argument("support", type=str, help="ID of entity to grasp FROM ('grasp' entity is on-top-of this 'support' entity)")
    parser.add_argument("waypoint", type=str, help="ID of waypoint for start and end position of the robot")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("benchmark_grasping")

    robot = get_robot(args.robot)

    grasp_cls = ds.Designator(robot, args.cls)
    support_entity = ds.EdEntityDesignator(robot, id=args.support)

    entity_ids = ds.VariableDesignator([], resolve_type=[ClassificationResult])

    nav_to_start = NavigateToWaypoint(robot,
                                      waypoint_designator=ds.EdEntityDesignator(robot, id=args.waypoint),
                                      look_at_designator=support_entity)

    assert nav_to_start.execute() == 'arrived'

    inspect = Inspect(robot=robot, entityDes=support_entity, objectIDsDes=entity_ids)
    assert inspect.execute() == 'done'

    inspection_result = entity_ids.resolve()  # type: List[ClassificationResult]
    if inspection_result:
        matching_entity_ids = [result.id for result in inspection_result if result.type == grasp_cls]
        if matching_entity_ids:
            selected_entity_id = matching_entity_ids[0]
            rospy.loginfo("Selected entity {} for grasping".format(selected_entity_id))
            grasp_entity = ds.EdEntityDesignator(robot, id=selected_entity_id)

            arm = ds.UnoccupiedArmDesignator(robot, {})

            # TODO: Do timing of all
            grab_state = Grab(robot, grasp_entity, arm)
            grab_state.execute()
        else:
            rospy.logerr("No entities found of the given class '{}'".format(grasp_cls))
    else:
        rospy.logerr("No entities found at all :-(")

    assert nav_to_start.execute() == 'arrived'
