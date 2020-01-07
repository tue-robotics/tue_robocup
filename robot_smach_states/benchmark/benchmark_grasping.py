#! /usr/bin/env python

# System
import argparse
import csv
import time

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states import Grab, Inspect, ClassificationResult, NavigateToWaypoint, ForceDrive, SetGripper

try:
    from typing import List
except ImportError:
    pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Measure the time it takes to grasp an entity when starting from a given pose. "
                                                 "After the grasping is done, go back to the waypoint, "
                                                 "turn around and drop the item, "
                                                 "then turn around once more to be in the start configuration again")
    parser.add_argument("cls", type=str, help="class of entity to grasp, eg. 'coke'")
    parser.add_argument("support", type=str, help="ID of entity to grasp FROM ('grasp' entity is on-top-of this 'support' entity), eg. 'cabinet'")
    parser.add_argument("waypoint", type=str, help="ID of waypoint for start and end position of the robot")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("--output", default="grasp_benchmark.csv", help="Output of the benchmark results")

    args = parser.parse_args()

    rospy.init_node("benchmark_grasping")

    with open(args.output, 'a+') as csv_file:
        reader = csv.DictReader(csv_file)

        fields = ['robot', 'start_waypoint', 'class', 'id', 'inspect_start', 'inspect_end', 'grab_start', 'grab_end']
        results_writer = csv.DictWriter(csv_file, fieldnames=fields)

        if reader.fieldnames != results_writer.fieldnames:
            results_writer.writeheader()

        robot = get_robot(args.robot)

        grasp_cls = ds.Designator(args.cls, name='grasp_cls')
        support_entity = ds.EdEntityDesignator(robot, id=args.support, name='support_entity')
        entity_ids = ds.VariableDesignator([], resolve_type=[ClassificationResult], name='entity_ids')
        waypoint = ds.EdEntityDesignator(robot, id=args.waypoint, name='waypoint')

        arm = ds.LockingDesignator(ds.UnoccupiedArmDesignator(robot, {}))
        arm.lock()

        record = {'robot': args.robot, 'start_waypoint': args.waypoint, 'class': args.cls,
                  'id': None, 'inspect_start': None, 'inspect_end': None, 'grab_start': None, 'grab_end': None}
        overall_start = time.time()

        try:
            nav_to_start = NavigateToWaypoint(robot,
                                              waypoint_designator=waypoint,
                                              look_at_designator=support_entity)

            assert nav_to_start.execute() == 'arrived'

            inspect = Inspect(robot=robot, entityDes=support_entity, objectIDsDes=entity_ids)

            record['inspect_start'] = time.time()
            assert inspect.execute() == 'done'
            record['inspect_end'] = time.time() - record['inspect_start']

            inspection_result = entity_ids.resolve()  # type: List[ClassificationResult]
            if inspection_result:
                matching_entity_ids = [result.id for result in inspection_result if result.type == grasp_cls.resolve()]
                if matching_entity_ids:
                    selected_entity_id = matching_entity_ids[0]
                    rospy.loginfo("Selected entity {} for grasping".format(selected_entity_id))
                    grasp_entity = ds.EdEntityDesignator(robot, id=selected_entity_id)

                    grab_state = Grab(robot, grasp_entity, arm)
                    record['grab_start'] = time.time() - record['inspect_end']
                    grab_state.execute()
                    record['grab_end'] = time.time() - record['grab_start']

                    assert nav_to_start.execute() == 'arrived'

                    rospy.logwarn("Robot will turn around to drop the {}".format(grasp_cls.resolve()))

                    force_drive = ForceDrive(robot, 0, 0, 1.57, 3)
                    force_drive.execute()

                    drop_it = SetGripper(robot, arm_designator=arm, grab_entity_designator=grasp_entity)
                    drop_it.execute()

                    nav_to_start.execute()
                else:
                    rospy.logerr("No entities found of the given class '{}'".format(grasp_cls.resolve()))
            else:
                rospy.logerr("No entities found at all :-(")
        finally:
            results_writer.writerow(record)
