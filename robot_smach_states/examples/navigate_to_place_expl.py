# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv
from robot_skills.util.kdl_conversions import FrameStamped

# Robot smach states
import robot_smach_states.util.designators as ds
from robot_smach_states import NavigateToPlace


# class PlacePoseDesignator(ds.Designator):
#     def __init__(self, robot, entity_id):
#         super(PlacePoseDesignator, self).__init__(resolve_type=FrameStamped)
#         self._robot = robot
#         self._entity_id = entity_id
#
#     def resolve(self):
#         entity = self._robot.ed.get_entity(id=self._entity_id)
#         return entity.pose


if __name__ == "__main__":

    rospy.init_node("test_navigate_to_place")

    robot = get_robot_from_argv(index=1)

    # place_pose_designator = PlacePoseDesignator(robot, "trash_bin")
    place_pose_designator = ds.EmptySpotDesignator(robot, ds.EdEntityDesignator(robot, id="trash_bin"))

    arm_designator = ds.UnoccupiedArmDesignator(robot, {})

    s = NavigateToPlace(robot, place_pose_designator, arm_designator)
    s.execute()
