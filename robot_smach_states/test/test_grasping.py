# ROS
import PyKDL as kdl
import rospy

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states import Grab


if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(index=1)

    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0, 0, -1.57), kdl.Vector(2.6, -0.95, 0.8)), frame_id="/map")

    robot.ed.update_entity(id=entity_id, frame_stamped=pose)

    item = ds.EdEntityDesignator(robot, id=entity_id)

    arm = ds.UnoccupiedArmDesignator(robot, {})

    grab_state = Grab(robot, item, arm)
    grab_state.execute()
