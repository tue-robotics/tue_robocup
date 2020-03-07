#! /usr/bin/env python

# System
import argparse
import csv
import time
import datetime

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot
from robot_skills import arms

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states import Grab, Inspect, ClassificationResult, NavigateToWaypoint, ForceDrive, SetGripper, \
    Say


RESULT_FIELDS = [ 'timestamp', 'robot',
                  'start_waypoint',
                  'expected_class', 'observed_class',
                  'id',
                  'inspect_duration',
                  'grab_duration',
                  'x', 'y', 'z']
BATCH_CONFIG_FIELDS = ['cls', 'support', 'waypoint', 'inspect_from_area', 'search_area']
try:
    from typing import List
except ImportError:
    pass

ANY_OPTIONS = ['ANY', '*', 'any']

def single_item(robot, results_writer, cls, support, waypoint, inspect_from_area=None, non_strict_class=False, search_area='on_top_of'):
    """
    Benchmark grasping for a single item. Outputs a record dictionary

    :param robot: an instance of Robot
    :param results_writer: a csv.DictWriter to which the output record is written
    :param cls: class/type of item to grab
    :param support: ID of the entity supporting the item-to-grab
    :param waypoint: From where should the robot start the grasp
    :param inspect_from_area: Which area of the support-entity should the robot be in to start the inspection
    :param non_strict_class: If set to True, the robot is not strict about the type of item it grabs, eg. it continues grasping with another type of object
    :param search_area: which area of the support-entity to search/inspect for an item of the given class
    :return: a dict with the benchmark result
    """
    grasp_cls = ds.Designator(cls, name='grasp_cls')
    support_entity = ds.EdEntityDesignator(robot, id=support, name='support_entity')
    entity_ids = ds.VariableDesignator([], resolve_type=[ClassificationResult], name='entity_ids')
    waypoint_des = ds.EdEntityDesignator(robot, id=waypoint, name='waypoint')

    arm = ds.LockingDesignator(ds.UnoccupiedArmDesignator(robot, name='unoccupied-arm',
                                                          arm_properties={"required_trajectories": ["prepare_grasp"],
                                                                          "required_goals": ["carrying_pose"],
                                                                          "required_gripper_types": [arms.GripperTypes.GRASPING]}))
    arm.lock()

    record = {'robot': robot.robot_name, 'start_waypoint': waypoint, 'expected_class': cls, 'id': None,
              'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

    try:
        say_announce = Say(robot, sentence="Please put a {cls} {search_area} the {support}"
                           .format(cls=cls,support=support,search_area=search_area))
        say_announce.execute()

        nav_to_start = NavigateToWaypoint(robot,
                                          waypoint_designator=waypoint_des,
                                          look_at_designator=support_entity)

        assert nav_to_start.execute() == 'arrived', "I did not arrive at the start"

        inspect = Inspect(robot=robot, entityDes=support_entity, objectIDsDes=entity_ids,
                          navigation_area=inspect_from_area, searchArea=search_area)

        record['inspect_start'] = time.time()
        assert inspect.execute() == 'done', "I could not inspect the support entity"
        record['inspect_end'] = time.time()
        record['inspect_duration'] = '{:3.3f}'.format(record['inspect_end'] - record['inspect_start'])

        inspection_result = entity_ids.resolve()  # type: List[ClassificationResult]
        if inspection_result:
            if grasp_cls.resolve() in ANY_OPTIONS or non_strict_class:
                rospy.loginfo("Any item will match")
                matching_results = inspection_result
            else:
                matching_results = [result for result in inspection_result if result.type == grasp_cls.resolve()]
                rospy.loginfo("Found {} items of class {}".format(len(matching_results), grasp_cls.resolve()))

            if matching_results:
                if len(matching_results) > 1:
                    rospy.logwarn("There are multiple items OK to grab, will select the last one")
                    record['observed_class'] = ' '.join([result.type for result in matching_results])
                else:
                    record['observed_class'] = matching_results[-1].type
                selected_entity_id = matching_results[-1].id

                rospy.loginfo("Selected entity {} for grasping".format(selected_entity_id))
                grasp_entity = ds.EdEntityDesignator(robot, id=selected_entity_id)
                record['id'] = selected_entity_id[:6]

                entity = grasp_entity.resolve()  # type: Entity
                if entity:
                    vector_stamped = entity.pose.extractVectorStamped()  # type: VectorStamped
                    record['x'] = '{:.3f}'.format(vector_stamped.vector.x())
                    record['y'] = '{:.3f}'.format(vector_stamped.vector.y())
                    record['z'] = '{:.3f}'.format(vector_stamped.vector.z())

                grab_state = Grab(robot, grasp_entity, arm)

                record['grab_start'] = time.time()
                assert grab_state.execute() == 'done', "I couldn't grasp"
                record['grab_end'] = time.time()
                record['grab_duration'] = '{:3.3f}'.format(record['grab_end'] - record['grab_start'])

                assert nav_to_start.execute() == 'arrived', "I could not navigate back to the start"

                rospy.logwarn("Robot will turn around to drop the {}".format(grasp_cls.resolve()))

                force_drive = ForceDrive(robot, 0, 0, 1, 3.14)  # rotate 180 degs in pi seconds
                force_drive.execute()

                say_drop = Say(robot, sentence="I'm going to drop the item, please hold it!")
                say_drop.execute()

                drop_it = SetGripper(robot, arm_designator=arm, grab_entity_designator=grasp_entity)
                drop_it.execute()

                force_drive_back = ForceDrive(robot, 0, 0, -1, 3.14)  # rotate -180 degs in pi seconds
                force_drive_back.execute()
                nav_to_start.execute()
            else:
                raise AssertionError("No {} found".format(grasp_cls.resolve()))
        else:
            rospy.logerr("No entities found at all :-(")
    except AssertionError as assertion_err:
        say_fail = Say(robot, sentence=assertion_err.message + ", sorry")
        say_fail.execute()
    finally:
        results_writer.writerow(record)

    return record


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark grasping, for a single item or multiple items at various locations.'
                                                 'By specifying ANY as the class, any class of object will be picked up,'
                                                 'ignoring the object type'
                                                 'By specifying a specific class *but* adding the --non-strict-class flag, '
                                                 'the robot will grab something else if the specified thing is not available')
    parser.add_argument("--robot", default="hero",
                        help="Robot name (amigo, hero, sergio, mockbot)")
    parser.add_argument("--output", default="grasp_benchmark.csv",
                        help="Output of the benchmark results")
    parser.add_argument("--non-strict-class", action='store_true',
                        help="Only grasp an item if it matches the specified class. "
                             "Pass --non-strict-class=False to also grab another class of items "
                             "if none of the expected class is observed. "
                             "Use this if you only care about grasping, not object recognition")

    subparsers = parser.add_subparsers(help='single or batch mode', dest='subcommand')
    single = subparsers.add_parser(name='single', description="Measure the time it takes to grasp an entity when starting from a given pose. "
                                                 "After the grasping is done, go back to the waypoint, "
                                                 "turn around and drop the item, "
                                                 "then turn around once more to be in the start configuration again")
    single.add_argument("cls", type=str,
                        help="class of entity to grasp, eg. 'coke' or one of {} if you don't care".format(ANY_OPTIONS))
    single.add_argument("support", type=str,
                        help="ID of entity to grasp FROM ('grasp' entity is on-top-of this 'support' entity), eg. 'cabinet'")
    single.add_argument("waypoint", type=str,
                        help="ID of waypoint for start and end position of the robot")
    single.add_argument("--inspect_from_area", type=str,
                        help="What area of the support-entity to inspect from", default='in_front_of')

    batch = subparsers.add_parser(name='batch', description="Perform the single case repeatedly, "
                                                            "taking configurations from a .csv file, "
                                                            "with columns {}".format(','.join(BATCH_CONFIG_FIELDS)))
    batch.add_argument("--configuration", type=str, default='grasp_benchmark_config.csv')

    args = parser.parse_args()
    rospy.init_node("benchmark_grasping")

    with open(args.output, 'a+') as csv_file:
        reader = csv.DictReader(csv_file)

        results_writer = csv.DictWriter(csv_file, fieldnames=RESULT_FIELDS, extrasaction='ignore')  # Ignore extra keys in the dicts

        if reader.fieldnames != results_writer.fieldnames:
            results_writer.writeheader()

        robot = get_robot(args.robot)

        if args.subcommand == 'single':
            single_item(robot, results_writer,
                        args.cls, args.support, args.waypoint, args.inspect_from_area,
                        args.non_strict_class)
        elif args.subcommand == 'batch':
            with open(args.configuration) as config_file:
                config_reader = csv.DictReader(config_file)
                assert config_reader.fieldnames == BATCH_CONFIG_FIELDS, "CSV file need fields {}".format(','.join(BATCH_CONFIG_FIELDS))

                configs = []

                for config_row in config_reader:
                    configs += [config_row]
                    item_config_description = "Put a {cls} {search_area} the {support}"\
                        .format(cls=config_row['cls'],
                                support=config_row['support'],
                                search_area=config_row['search_area'])
                    rospy.logwarn(item_config_description)

                records = []
                for config_row in configs:
                    rospy.loginfo(config_row)
                    record = single_item(robot, results_writer,
                                         cls=config_row['cls'],
                                         waypoint=config_row['waypoint'],
                                         support=config_row['support'],
                                         inspect_from_area=config_row['inspect_from_area'],
                                         non_strict_class=args.non_strict_class,
                                         search_area=config_row['search_area'])
                    records += [record]

