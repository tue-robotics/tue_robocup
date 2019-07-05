#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Janno Lunenburg

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import get_robot_from_argv
from robot_skills.robot import Robot


class InspectFurniture(smach.StateMachine):
    def __init__(self, robot, furniture_designator):
        # type: (Robot, object) -> None
        """
        Drives to the designated furniture object, inspects this and selects the entity that will be pointed to

        :param robot: (Robot) robot API object
        :param furniture_designator: (EdEntityDesignator) designates the furniture object that was pointed to.
        """
        # ToDo: we need to add userdata
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

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

            @smach.cb_interface(outcomes=["done"])
            def select_entity(userdata):
                """
                Selects the entity that the robot believes the operator has pointed to and that the robot will
                identify later on

                :param userdata: (dict)
                :return: (srt) outcome
                """
                return "done"

            smach.StateMachine.add("SELECT_ENTITY",
                                   smach.CBState(select_entity),
                                   transitions={"done": "succeeded"})


if __name__ == "__main__":

    rospy.init_node("test_furniture_inspection")

    _robot = get_robot_from_argv(index=1)

    furniture = ds.EdEntityDesignator(robot=_robot, id="desk")

    sm = InspectFurniture(robot=_robot, furniture_designator=furniture)
    sm.execute()
