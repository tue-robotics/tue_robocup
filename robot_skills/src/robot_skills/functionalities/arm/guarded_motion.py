import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_skills.arm.arms import Arm
from robot_skills.arm.force_sensor import ForceSensor
from robot_skills.functionalities import RobotFunc


class GuardedMotionFunc(RobotFunc):
    def __init__(self):
        super(GuardedMotionFunc, self).__init__("guarded_motion",
                                                Arm,
                                                {
                                                    "move_down_until_force_sensor_edge_up": move_down_until_force_sensor_edge_up,
                                                    "_create_lower_force_sensing_goal": _create_lower_force_sensing_goal,
                                                    "_create_retract_force_sensing_goal": _create_retract_force_sensing_goal
                                                })

    def check_requirements(self, arm):
        """
        Check that the arm has at least one force sensor
        """
        for part in arm.parts:
            if isinstance(arm.parts[part], ForceSensor):
                return True
        return False


def move_down_until_force_sensor_edge_up(self, force_sensor=None, timeout=10, retract_distance=0.01,
                                         distance_move_down=None):
    """
    Move down the arm (hero specific, only joint arm_lift_joint) until one of 2 things:
        - Force sensor detects an edge up
        - Timeout

    A 'force_sensor.TimeOutException' will be raised if no edge up is detected within timeout

    :param force_sensor: ForceSensor of the arm
    :param timeout: Max duration for edge up detection
    :param retract_distance: How much to retract if we have reached a surface
    :param distance_move_down: Maximum distance to move down the arm_lift_joint
    """
    if force_sensor is None:
        force_sensor = self.parts["force_sensor"]

    # Fill with required joint names (desired in hardware / gazebo impl)
    goal = self._create_lower_force_sensing_goal(distance_move_down, timeout)
    self._ac_joint_traj.send_goal(goal)

    force_sensor.wait_for_edge_up(timeout)
    self.cancel_goals()

    goal = self._create_retract_force_sensing_goal(retract_distance, timeout)
    self._ac_joint_traj.send_goal(goal)


def _create_lower_force_sensing_goal(self, distance_move_down, timeout):
    current_joint_state = self.get_joint_states()

    # Sets the joint state to move down a certain distance (or to 0 if distance is not set)
    if distance_move_down is None:
        current_joint_state['arm_lift_joint'] = 0  # TODO make this function not HERO-specific
    else:
        current_joint_state['arm_lift_joint'] = max(0, current_joint_state['arm_lift_joint'] - distance_move_down)

    # Creates goal for force_sensor
    return create_force_sensing_goal(self.joint_names, current_joint_state, timeout)


def _create_retract_force_sensing_goal(self, retract_distance, timeout):
    current_joint_state = self.get_joint_states()

    # Changes state to retract arm after force sensor edge up
    current_joint_state['arm_lift_joint'] += retract_distance  # TODO make this function not HERO-specific

    # Creates joint states goal
    return create_force_sensing_goal(self.joint_names, current_joint_state, timeout)


def create_force_sensing_goal(joint_names, current_joint_state, timeout):
    positions = [current_joint_state[name] for name in joint_names]

    points = [JointTrajectoryPoint(
        positions=positions,
        time_from_start=rospy.Duration.from_sec(timeout))
    ]
    trajectory = JointTrajectory(joint_names=joint_names, points=points)
    return FollowJointTrajectoryGoal(trajectory=trajectory)
