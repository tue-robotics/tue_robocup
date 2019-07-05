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


class InspectFurniture(smach.StateMachine):
    def __init__(self, robot, furniture_designator):
        # type: (Robot, object) -> None
        """
        Drives to the designated furniture object, inspects this and selects the entity that will be pointed to

        :param robot: (Robot) robot API object
        :param furniture_designator: (EdEntityDesignator) designates the furniture object that was pointed to.
        """
        # ToDo: we need to add userdata
        smach.StateMachine.__init__(self,
                                    outcomes=["succeeded", "failed"],
                                    input_keys=["laser_dot"])

        object_ids_des = ds.VariableDesignator([], resolve_type=[states.ClassificationResult])

        with self:

            smach.StateMachine.add("SAY_GO",
                                   states.SayFormatted(
                                       robot,
                                       "Let's go to the {furniture_object}",
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
                                                "failed": "failed"})  # ToDo: fallback?

            @smach.cb_interface(outcomes=["succeeded", "no_entities"],
                                input_keys=["laser_dot"])
            def select_entity(userdata):
                """
                Selects the entity that the robot believes the operator has pointed to and that the robot will
                identify later on.

                Userdata contains key 'laser_dot' with value geometry_msgs.msg.PointStamped where the operator pointed
                at.

                :param userdata: (dict)
                :return: (srt) outcome
                """
                # ToDo: check frame ids
                # Extract classification results
                entity_ids = [cr.id for cr in object_ids_des.resolve()]
                rospy.loginfo("Segmented entities: {}".format(entity_ids))

                # Obtain all corresponding entities
                all_entities = robot.ed.get_entities()
                segmented_entities = [e for e in all_entities if e.id in entity_ids]

                # ToDo: filter out 'unprobable' entities

                if not segmented_entities:
                    rospy.logwarn("No 'probable' entities left")
                    return "no_entities"

                # Select entity closest to the point where the operator pointed at (i.e., closest in 2D)
                closest_tuple = (None, None)
                x_ref = userdata.laser_dot.point.x
                y_ref = userdata.laser_dot.point.y
                for e in segmented_entities:  # type: Entity
                    x_e = e.pose.frame.p.x()
                    y_e = e.pose.frame.p.y()
                    distance_2d = math.hypot(x_ref - x_e, y_ref - y_e)
                    rospy.loginfo("Entity {} at {}, {}: distance = {}".format(e.id, x_e, y_e, distance_2d))

                    if closest_tuple[0] is None or distance_2d < closest_tuple[1]:
                        closest_tuple = (e, distance_2d)

                rospy.loginfo("Best entity: {} at {}".format(closest_tuple[0].id, closest_tuple[1]))

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

    ps = geometry_msgs.msg.PointStamped()
    ps.header.frame_id = "/map"
    ps.point.x = 2.0
    ps.point.y = -1.0
    ps.point.z = 1.0
    user_data = smach.UserData()
    user_data["laser_dot"] = ps

    sm = InspectFurniture(robot=_robot, furniture_designator=furniture)
    sm.execute(user_data)
