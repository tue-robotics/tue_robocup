import math
import os

import rospy
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped
from robot_skills import Hero
from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from smach import cb_interface, StateMachine, CBState
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


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
                        *quaternion_from_euler(
                            0, 0, math.atan2(trash_bin_to_base_vector.y(),
                                             trash_bin_to_base_vector.x()) - math.pi + yaw_offset
                        )
                    )
                )
            )
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.2)).execute({})
            return 'done'

        with self:
            self.add('CONTROL_TO_TRASH_BIN', CBState(control_to_pose), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    ControlToTrashBin(hero, 'trash_bin', 0.45, -0.2).execute()
    hero.leftArm().send_joint_goal('grab_trash_bag')
