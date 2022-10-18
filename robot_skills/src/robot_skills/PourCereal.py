import os

import PyKDL
import rospy

from robot_skills import get_robot
from robot_skills.tuning import POUR_OFFSET_X, POUR_OFFSET_Y,  ITEM_VECTOR_DICT, item_vector_to_item_frame, item_frame_to_pose,  JOINTS_PRE_POUR, JOINTS_POUR

from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from smach import StateMachine, cb_interface, CBState

class PourCereal(StateMachine):
    def __init__(self, robot, table_id):
        StateMachine.__init__(self, outcomes=["succeeded"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        @cb_interface(outcomes=["done"])
        def _align_pour(_):
            item_placement_vector = ITEM_VECTOR_DICT["cereal_box"] + PyKDL.Vector(POUR_OFFSET_X, POUR_OFFSET_Y, 0)
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Moving to pouring pose at {}".format(goal_pose))
            robot.head.look_down()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"])
        def _pour(_):
            robot.speech.speak("Hope this goes well", block=False)
            send_joint_goal(JOINTS_PRE_POUR)
            send_joint_goal(JOINTS_POUR)
            send_joint_goal(JOINTS_PRE_POUR)
            return "done"

        with self:
            self.add("ALIGN_POUR", CBState(_align_pour), transitions={"done": "POUR"})
            self.add("POUR", CBState(_pour), transitions={"done": "ALIGN_PLACE"})

if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    PourCereal(robot_instance, "dinner_table").execute()


