from functionalities.robot_functionality import RobotFunc

from robot_skills.arm.arms import Arm
from robot_skills.arm.force_sensor import ForceSensor

from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GuardedMotionFunc(RobotFunc):
    def __init__(self):
        super(GuardedMotionFunc, self).__init__("guarded_motion",
                                                Arm,
                                                {"move_down_until_force_sensor_edge_up" : move_down_until_force_sensor_edge_up,
                                                 "_create_lower_force_sensing_goal": _create_lower_force_sensing_goal,
                                                 "_create_retract_force_sensing_goal": _create_retract_force_sensing_goal})

    def check_requirements(self, arm):
        "Check that the arm has at least one force sensor"
        for part in arm.parts:
            if isinstance(arm.parts[part], ForceSensor):
                return True
        return False

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
    goal = self._create_lower_force_sensing_goal(timeout)
    self._ac_joint_traj.send_goal(goal)

    force_sensor.wait_for_edge_up(timeout)
    self.cancel_goals()

    goal = self._create_retract_force_sensing_goal(0.5, timeout)
    self._ac_joint_traj.send_goal(goal)

def _create_lower_force_sensing_goal(self, timeout):
    current_joint_state = self.get_joint_states()
    current_joint_state['arm_lift_joint'] = 0   # TODO make this function not HERO-specific
    return create_force_sensing_goal(self.joint_names, current_joint_state, timeout)

def _create_retract_force_sensing_goal(self, retract_distance, timeout):
    current_joint_state = self.get_joint_states()
    current_joint_state['arm_lift_joint'] += retract_distance # TODO make this function not HERO-specific
    return create_force_sensing_goal(self.joint_names, current_joint_state, timeout)

def create_force_sensing_goal(joint_names, current_joint_state, timeout):
    positions = [current_joint_state[n] for n in joint_names]
    points = [JointTrajectoryPoint(
        positions=positions,
        time_from_start=rospy.Duration.from_sec(timeout))
    ]
    trajectory = JointTrajectory(joint_names=joint_names, points=points)
    return FollowJointTrajectoryGoal(trajectory=trajectory)
