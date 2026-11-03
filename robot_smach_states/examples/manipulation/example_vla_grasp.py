# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation import VLAGrasp


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Grasp with the VLA policy server using the 'VLAGrasp' smach state. "
                                                 "Position the robot in front of the table with the head tilted down "
                                                 "first. Without --execute the state runs in shadow mode and only logs "
                                                 "what the policy would do.")
    parser.add_argument("instruction", nargs="?", default="pick up the mug", help="Instruction for the policy")
    parser.add_argument("--robot", default="hero", help="Robot name (hero)")
    parser.add_argument("--execute", action="store_true", help="Actually command the robot")
    parser.add_argument("--duration", type=float, default=30.0, help="Maximum duration [s]")
    parser.add_argument("--steps", type=int, default=15, help="Rows of every chunk to execute before re-predicting")
    parser.add_argument("--prepare", action="store_true",
                        help="First reset the arm and tilt the head down, the pose the policy was trained from")
    args = parser.parse_args()

    rospy.init_node("example_vla_grasp")

    robot = get_robot(args.robot)
    arm = ds.UnoccupiedArmDesignator(robot).lockable()

    if args.prepare:
        prepared_arm = arm.resolve()
        prepared_arm.send_joint_goal("reset")
        robot.head.look_down()
        prepared_arm.wait_for_motion_done()
        robot.head.wait_for_motion_done()

    state = VLAGrasp(robot, arm, ds.Designator(args.instruction), execute=args.execute, max_duration=args.duration,
                     steps_per_chunk=args.steps)
    rospy.loginfo("VLAGrasp outcome: %s", state.execute())
