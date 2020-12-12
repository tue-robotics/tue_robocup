import functools
import rospy
import types

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def AddForceSensingArm(arm):
    arm.move_down_until_force_sensor_edge_up = types.MethodType(move_down_until_force_sensor_edge_up, arm)

def move_down_until_force_sensor_edge_up(self, force_sensor=None, timeout=10, retract_distance=0.01):
    """
    Move down the arm (hero specific, only joint arm_lift_joint) until the force sensor detects an edge up

    A 'force_sensor.TimeOutException' will be raised if no edge up is detected within timeout

    :param timeout: Max duration for edge up detection
    :param retract_distance: How much to retract if we have reached a surface
    """
    if not force_sensor:
        force_sensor = self.parts["force_sensor"]

    # Fill with required joint names (desired in hardware / gazebo impl)
    current_joint_state = self.get_joint_states()
    current_joint_state['arm_lift_joint'] = 0
    self._ac_joint_traj.send_goal(make_goal(self.joint_names, current_joint_state, timeout))

    force_sensor.wait_for_edge_up(timeout)
    self.cancel_goals()

    current_joint_state = self.get_joint_states()
    current_joint_state['arm_lift_joint'] += retract_distance
    self._ac_joint_traj.send_goal(self._make_goal(current_joint_state, 0.5))

def make_goal(joint_names, current_joint_state, timeout):
    positions = [current_joint_state[n] for n in joint_names]
    points = [JointTrajectoryPoint(positions=positions,
                                   time_from_start=rospy.Duration.from_sec(timeout))]
    trajectory = JointTrajectory(joint_names=joint_names, points=points)
    return FollowJointTrajectoryGoal(trajectory=trajectory)
