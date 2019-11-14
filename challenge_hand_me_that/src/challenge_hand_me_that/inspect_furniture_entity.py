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
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import get_robot_from_argv
from robot_skills.robot import Robot
from robot_skills.util.entity import Entity

# Items with x- or y-dimension larger than this value will be filtered out
SIZE_LIMIT = 0.2
# Items with a ratio between x- and y-dimensions outside this value are considered 'faulty' segmentations
RATIO_LIMIT = 4.0


class InspectFurniture(smach.StateMachine):
    def __init__(self, robot, furniture_designator, entity_designator):
        # type: (Robot, object) -> None
        """
        Drives to the designated furniture object, inspects this and selects the entity that will be pointed to

        :param robot: (Robot) robot API object
        :param furniture_designator: (EdEntityDesignator) designates the furniture object that was pointed to.
        :param entity_designator: (EdEntityDesignator) writeable EdEntityDesignator
        """
        # ToDo: we need to add userdata
        smach.StateMachine.__init__(self,
                                    outcomes=["succeeded", "failed"],
                                    input_keys=["laser_dot"])

        assert ds.is_writeable(entity_designator), "Entity designator must be writeable for this purpose"
        object_ids_des = ds.VariableDesignator([], resolve_type=[states.ClassificationResult])

        with self:

            smach.StateMachine.add("SAY_GO",
                                   states.Say(robot, "Let's go to the {furniture_object}",
                                              furniture_object=ds.AttrDesignator(furniture_designator, "id",
                                                                                 resolve_type=str)),
                                   transitions={"spoken": "INSPECT_FURNITURE"})

            smach.StateMachine.add("INSPECT_FURNITURE",
                                   states.Inspect(robot=robot,
                                                  entityDes=furniture_designator,
                                                  objectIDsDes=object_ids_des,
                                                  navigation_area="in_front_of",
                                                  ),
                                   transitions={"done": "SELECT_ENTITY",
                                                "failed": "SAY_INSPECTION_FAILED"})  # ToDo: fallback?

            smach.StateMachine.add("SAY_INSPECTION_FAILED",
                                   states.Say(robot, "I am sorry but I was not able to reach the {furniture_object}",
                                              furniture_object=ds.AttrDesignator(furniture_designator, "id",
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
                assert userdata.laser_dot.header.frame_id.endswith("map"), "Provide your laser  dot in map frame"

                # Extract classification results
                entity_ids = [cr.id for cr in object_ids_des.resolve()]
                rospy.loginfo("Segmented entities: {}".format(entity_ids))

                # Obtain all corresponding entities
                all_entities = robot.ed.get_entities()
                segmented_entities = [e for e in all_entities if e.id in entity_ids]

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

                # Select entity closest to the point where the operator pointed at (i.e., closest in 2D)
                closest_tuple = (None, None)
                x_ref = userdata.laser_dot.point.x
                y_ref = userdata.laser_dot.point.y
                # ToDo: use sorting for this...
                for e in candidates:  # type: Entity
                    x_e = e.pose.frame.p.x()
                    y_e = e.pose.frame.p.y()
                    distance_2d = math.hypot(x_ref - x_e, y_ref - y_e)
                    rospy.loginfo("Entity {} at {}, {}: distance = {}".format(e.id, x_e, y_e, distance_2d))

                    if closest_tuple[0] is None or distance_2d < closest_tuple[1]:
                        closest_tuple = (e, distance_2d)

                rospy.loginfo("Best entity: {} at {}".format(closest_tuple[0].id, closest_tuple[1]))
                entity_designator.write(closest_tuple[0])

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
    furniture = ds.EdEntityDesignator(robot=_robot, id="desk")
    entity_designator = ds.VariableDesignator(resolve_type=Entity)

    ps = geometry_msgs.msg.PointStamped()
    ps.header.frame_id = "/map"
    ps.point.x = 2.0
    ps.point.y = -1.0
    ps.point.z = 1.0
    user_data = smach.UserData()
    user_data["laser_dot"] = ps

    sm = InspectFurniture(robot=_robot, furniture_designator=furniture, entity_designator=entity_designator.writeable)
    sm.execute(user_data)
