#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Janno Lunenburg

# System
import math

# ROS
import geometry_msgs.msg
import rospy
import tf2_ros
from tf2_pykdl_ros import VectorStamped
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

import smach

# TU/e Robotics
from ed.entity import Entity, Volume

import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction import Say
from robot_smach_states.world_model import Inspect
from robot_smach_states.utility import WriteDesignator

from robot_skills import get_robot_from_argv
from robot_skills.robot import Robot
from robot_skills.classification_result import ClassificationResult

from robocup_knowledge import load_knowledge

# Items with x- or y-dimension larger than this value will be filtered out
SIZE_LIMIT = 0.2
# Items with a ratio between x- and y-dimensions outside this value are considered 'faulty' segmentations
RATIO_LIMIT = 4.0


class InspectFurniture(smach.StateMachine):
    def __init__(self, robot, furniture_designator, entity_designator, max_number_items=2):
        # type: (Robot, object) -> None
        """
        Drives to the designated furniture object, inspects this and selects the entity that will be pointed to

        :param robot: (Robot) robot API object
        :param furniture_designator: (EdEntityDesignator) designates the furniture object that was pointed to.
        :param entity_designator: (EdEntityDesignator) writeable EdEntityDesignator
        :param max_number_items: Max number of items in the entity designator
        """
        # ToDo: we need to add userdata
        smach.StateMachine.__init__(self,
                                    outcomes=["succeeded", "failed"],
                                    input_keys=["laser_dot"])

        assert ds.is_writeable(entity_designator), "Entity designator must be writeable for this purpose"
        object_ids_des = ds.VariableDesignator([], resolve_type=[ClassificationResult])
        inspect_area_des = ds.VariableDesignator(resolve_type=str)
        nav_area_des = ds.VariableDesignator(resolve_type=str)

        common_knowledge = load_knowledge('common')

        @smach.cb_interface(outcomes=["done", "failed"], input_keys=["laser_dot"])
        def _select_inspect_area(userdata, robot, furniture_des, inspect_area_des):
            def _ray_volume_check(ray_vs: VectorStamped, vol: Volume):
                if vol.min_corner.z() <= ray_vs.vector.z() <= vol.max_corner.z():
                    return 0
                else:
                    return min(abs(ray_vs.vector.z()-vol.min_corner.z()), abs(ray_vs.vector.z()-vol.max_corner.z()))

            entity_to_inspect = furniture_des.resolve()
            if entity_to_inspect is None:
                inspect_area_des.write("on_top_of")
                return "failed"

            laser_dot = tf2_ros.convert(userdata["laser_dot"], VectorStamped)
            laser_dot.header.stamp = rospy.Time()
            laser_dot_entity_frame = robot.tf_buffer.transform(laser_dot, entity_to_inspect.uuid)

            best_volumes = []
            for vol_name, vol in entity_to_inspect.volumes.items():
                if vol_name not in common_knowledge.get_inspect_areas(entity_to_inspect.uuid):
                    continue

                best_volumes.append((vol_name, _ray_volume_check(laser_dot_entity_frame, vol)))

            if not best_volumes:
                rospy.loginfo("No volumes found from knowledge, using 'on_top_of'")
                inspect_area_des.write("on_top_of")
                return "failed"

            best_volumes.sort(key=lambda tup: tup[1], reverse=False)
            rospy.loginfo(f"Volumes: {best_volumes}")
            best_vol = best_volumes[0]
            rospy.loginfo(f"Using volume: {best_vol[0]}")

            inspect_area_des.write(best_vol[0])
            return "done"

        @smach.cb_interface(outcomes=["done", "failed"])
        def _select_nav_area(userdata, furniture_des, inspect_area_des, nav_area_des):
            entity_to_inspect = furniture_des.resolve()
            inspect_area = inspect_area_des.resolve()
            if entity_to_inspect is None or not inspect_area:
                rospy.loginfo("Using backup nav_area: 'in_front_of'")
                nav_area_des.write("in_front_of")
                return "failed"

            nav_area = common_knowledge.get_inspect_position(entity_to_inspect.uuid, inspect_area)
            rospy.loginfo(f"For {entity_to_inspect.uuid}:{inspect_area} selected '{nav_area}' to navigate")
            nav_area_des.write(nav_area)
            return "done"

        with self:

            smach.StateMachine.add("SAY_GO",
                                   Say(robot, "Let's go to the {furniture_object}",
                                       furniture_object=ds.AttrDesignator(furniture_designator, "uuid",
                                                                          resolve_type=str)),
                                   transitions={"spoken": "CLEAR_FOUND_ENTITY_DESIGNATOR"})

            smach.StateMachine.add('CLEAR_FOUND_ENTITY_DESIGNATOR',
                                   WriteDesignator(object_ids_des.writeable, []),
                                   transitions={'written': 'SELECT_INSPECT_AREA'})

            smach.StateMachine.add("SELECT_INSPECT_AREA", smach.CBState(_select_inspect_area,
                                                                        cb_args=[robot,
                                                                                 furniture_designator,
                                                                                 inspect_area_des.writeable]),
                                   transitions={"done": "SELECT_NAV_AREA",
                                                "failed": "SELECT_NAV_AREA"})

            smach.StateMachine.add("SELECT_NAV_AREA", smach.CBState(_select_nav_area,
                                                                        cb_args=[furniture_designator,
                                                                                 inspect_area_des.writeable,
                                                                                 nav_area_des.writeable]),
                                   transitions={"done": "INSPECT_FURNITURE",
                                                "failed": "INSPECT_FURNITURE"})

            smach.StateMachine.add("INSPECT_FURNITURE",
                                   Inspect(robot=robot, entityDes=furniture_designator, searchArea=inspect_area_des,
                                           objectIDsDes=object_ids_des, navigation_area=nav_area_des),
                                   transitions={"done": "SELECT_ENTITY",
                                                "failed": "SAY_INSPECTION_FAILED"})  # ToDo: fallback?

            smach.StateMachine.add("SAY_INSPECTION_FAILED",
                                   Say(robot, "I am sorry but I was not able to reach the {furniture_object}",
                                       furniture_object=ds.AttrDesignator(furniture_designator, "uuid",
                                                                          resolve_type=str)),
                                   transitions={"spoken": "failed"})

            @smach.cb_interface(outcomes=["succeeded", "no_entities"],
                                input_keys=["laser_dot"])
            def select_entity(userdata):
                """
                Selects the entity that the robot believes the operator has pointed to and that the robot will
                identify later on.

                Userdata contains key 'laser_dot' with value geometry_msgs.msg.PointStamped where the operator pointed
                at.

                :param userdata: (dict)
                :return: (str) outcome
                """
                assert userdata.laser_dot.header.frame_id.endswith("map"), "Provide your laser dot in map frame"

                # Extract classification results
                entity_ids = [cr.uuid for cr in object_ids_des.resolve()]
                rospy.loginfo("Segmented entities: {}".format(entity_ids))

                # Obtain all corresponding entities
                all_entities = robot.ed.get_entities()
                segmented_entities = [e for e in all_entities if e.uuid in entity_ids]

                # Filter out 'unprobable' entities
                candidates = []
                for entity in segmented_entities:  # type: Entity

                    # The following filtering has been 'copied' from the cleanup challenge
                    # It can be considered a first step but does not take the orientation of the convex hull into
                    # account
                    shape = entity.shape
                    size_x = max(shape.x_max - shape.x_min, 0.001)
                    size_y = max(shape.y_max - shape.y_min, 0.001)

                    if size_x > SIZE_LIMIT or size_y > SIZE_LIMIT:
                        continue

                    if not 1 / min(RATIO_LIMIT, 1000) <= size_x / size_y <= min(RATIO_LIMIT, 1000):
                        continue

                    candidates.append(entity)

                # If no entities left: don't bother continuing
                if not candidates:
                    rospy.logwarn("No 'probable' entities left")
                    return "no_entities"

                candidates_distance = []
                # Select entity closest to the point where the operator pointed at (i.e., closest in 2D)
                closest_tuple = (None, None)
                x_ref = userdata.laser_dot.point.x
                y_ref = userdata.laser_dot.point.y
                # ToDo: use sorting for this...
                for e in candidates:  # type: Entity
                    x_e = e.pose.frame.p.x()
                    y_e = e.pose.frame.p.y()
                    d_2d = math.hypot(x_ref - x_e, y_ref - y_e)
                    rospy.loginfo("Entity {} at {}, {}: distance = {}".format(e.uuid, x_e, y_e, d_2d))
                    candidates_distance.append((e, d_2d))

                    # if closest_tuple[0] is None or distance_2d < closest_tuple[1]:
                    #     closest_tuple = (e, distance_2d)
                candidates_distance.sort(key=lambda tup: tup[1], reverse=False)
                best_candidate = candidates_distance[0]

                rospy.loginfo("Best entity: {} at {}".format(best_candidate[0].uuid, best_candidate[1]))
                entity_designator.write(list(list(zip(*candidates_distance))[0][:max_number_items]))

                return "succeeded"

            smach.StateMachine.add("SELECT_ENTITY",
                                   smach.CBState(select_entity),
                                   transitions={"succeeded": "succeeded",
                                                "no_entities": "failed"})


if __name__ == "__main__":

    rospy.init_node("test_furniture_inspection")

    # Robot
    _robot = get_robot_from_argv(index=1)

    # Test data
    furniture = ds.EdEntityDesignator(robot=_robot, uuid="kitchen_shelf")
    entity_designator = ds.VariableDesignator(resolve_type=Entity)

    ps = geometry_msgs.msg.PointStamped()
    ps.header.frame_id = "map"
    ps.point.x = 4.8
    ps.point.y = 1.15
    ps.point.z = 0.7
    user_data = smach.UserData()
    user_data["laser_dot"] = ps

    sm = InspectFurniture(robot=_robot, furniture_designator=furniture, entity_designator=entity_designator.writeable)
    sm.execute(user_data)
