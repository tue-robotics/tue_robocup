import math
import os

import rospy
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped
from robot_skills import get_robot
from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from smach import cb_interface, StateMachine, CBState
from std_msgs.msg import Header
from tf_conversions import transformations


class ControlToTrashBin(StateMachine):
    def __init__(self, robot, trashbin_id, radius, yaw_offset):
        StateMachine.__init__(self, outcomes=['done'])

        @cb_interface(outcomes=['done'])
        def control_to_pose(_):
            trash_bin_frame = robot.ed.get_entity(trashbin_id).pose.frame
            trash_bin_position = trash_bin_frame.p
            base_frame = robot.base.get_location().frame
            base_position = base_frame.p

            trash_bin_to_base_vector = base_position - trash_bin_position
            trash_bin_to_base_vector.Normalize()

            desired_base_position = trash_bin_position + radius * trash_bin_to_base_vector

            angle = math.atan2(trash_bin_to_base_vector.y(), trash_bin_to_base_vector.x())
            min_angle = min(angle, 2*math.pi - angle)

            goal_pose = PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                pose=Pose(
                    position=Point(
                        x=desired_base_position.x(),
                        y=desired_base_position.y()
                    ),
                    orientation=Quaternion(
                        *transformations.quaternion_from_euler(
                            0, 0, min_angle - math.pi + yaw_offset
                        )
                    )
                )
            )
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1)).execute({})
            return 'done'

        with self:
            self.add('CONTROL_TO_TRASH_BIN', CBState(control_to_pose), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()
    ControlToTrashBin(robot_instance, 'trash_bin', 0.3, -0.2).execute()
    robot_instance.leftArm().send_joint_goal('grab_trash_bag')
