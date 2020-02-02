# System
import time

# ROS
from actionlib_msgs.msg import GoalStatus
import control_msgs.msg
import rospy
import trajectory_msgs.msg

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class Torso(RobotPart):
    def __init__(self, robot_name, tf_listener, get_joint_states):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        """
        super(Torso, self).__init__(robot_name=robot_name, tf_listener=tf_listener)

        self.joint_names = self.load_param('skills/torso/joint_names')
        self._arm_joint_names = self.load_param('skills/arm/joint_names')
        self.default_configurations = self.load_param('skills/torso/default_configurations')
        self.default_tolerance = self.load_param('/skills/torso/default_tolerance')
        self.lower_limit = self.default_configurations['lower_limit']
        self.upper_limit = self.default_configurations['upper_limit']

        # Init action client
        self.ac_move_torso = self.create_simple_action_client('/' + self.robot_name + '/body/joint_trajectory_action',
                                                              control_msgs.msg.FollowJointTrajectoryAction)

        self.subscribe_hardware_status('spindle')
        self._get_joint_states = get_joint_states

    def close(self):
        """
        Cancels all active goals for the torso
        :return: no return
        """
        try:
            rospy.loginfo("Torso cancelling all goals on close")
        except AttributeError:
            print("Torso cancelling all goals on close. Rospy is already deleted.")

        self.ac_move_torso.cancel_all_goals()

    def send_goal(self, configuration, timeout=0.0, tolerance=[]):
        """
        Send a named joint goal (pose) defined in the parameter default_configurations to the torso

        :param configuration: name of configuration, configuration should be loaded as parameter
        :param timeout: timeout in seconds; in case of 0.0, not waiting for motion done
        :param tolerance: list of position tolerances with the length equal to the number of joints
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_configurations:
            return self._send_goal(self.default_configurations[configuration], timeout=timeout, tolerance=tolerance)
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def _send_goal(self, torso_pos, timeout=0.0, tolerance=[], start_time=1.5):
        """
        Send a joint goal to the torso

        :param torso_pos: list of joint positions with the length equal to the number of joints
        :param timeout: timeout in seconds; in case of 0.0, not waiting for motion done
        :param tolerance: list of position tolerances with the length equal to the number of joints
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        torso_pos = list(torso_pos)

        rospy.logdebug("Send torso goal {0}, timeout = {1}".format(torso_pos, timeout))

        if len(torso_pos) != len(self.joint_names):
            rospy.logwarn(
                'Length of desired torso pos {0} does not correspond with number of joints {1}'.format(
                    len(torso_pos),
                    len(self.joint_names)))
            return False

        ''' Check limits '''
        for i in range(0, len(self.joint_names)):
            if torso_pos[i] < self.lower_limit[i] or torso_pos[i] > self.upper_limit:
                rospy.logwarn("Desired position {0} for joint {1} exceeds limits [{2}, {3}]".format(torso_pos[i],
                                                                                                    self.joint_names[i],
                                                                                                    self.lower_limit[i],
                                                                                                    self.upper_limit[
                                                                                                        i]))
                return False

        torso_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        torso_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
        torso_goal.trajectory.joint_names = list(self.joint_names)
        torso_goal_point.positions = torso_pos
        torso_goal_point.time_from_start = rospy.Duration.from_sec(start_time)
        torso_goal.trajectory.points.append(torso_goal_point)

        for i in range(0, len(self.joint_names)):
            goal_tolerance = control_msgs.msg.JointTolerance()
            goal_tolerance.name = self.joint_names[i]
            if len(tolerance) == len(self.joint_names):
                goal_tolerance.position = tolerance[i]
            else:
                goal_tolerance.position = self.default_tolerance[i]
            torso_goal.goal_tolerance.append(goal_tolerance)

        rospy.logdebug("Sending torso_goal: {0}".format(torso_goal))

        time.sleep(0.01)  # This is dangerous now we are changing the Ts of the TrajectoryActionLib

        # This is necessary: the rtt_actionlib in the hardware seems
        # to only have a queue size of 1 and runs at 1000 hz. This
        # means that if two goals are send approximately at the same
        # time (e.g. an arm goal and a torso goal), one of the two
        # goals probably won't make it. This sleep makes sure the
        # goals will always arrive in different update hooks in the
        # hardware TrajectoryActionLib server.

        # Fill with required joint names (desired in hardware / gazebo impl)
        current_joint_state = self._get_joint_states()
        missing_joint_names = [n for n in self._arm_joint_names if n not in self.joint_names]
        # This bit is needed because in some robots some joints are part of both arm(s) and torso.
        # Thus both need to be controlled.
        # In robots where these are disjoint sets (arm and torso joints do not overlap), missing_joint_names will be
        # empty and thus no change is incurred.
        # To fix this the entire concept of separate joint groups should be kicked out
        torso_goal.trajectory.joint_names += missing_joint_names
        torso_goal.trajectory.points[0].positions += [current_joint_state[n] for n in missing_joint_names]
        torso_goal.goal_tolerance += [control_msgs.msg.JointTolerance(
            name=n,
            position=goal_tolerance.position
        ) for n in missing_joint_names]

        self.ac_move_torso.send_goal(torso_goal)

        if timeout == 0.0:
            return True
        else:
            return self.wait_for_motion_done(timeout)

    def high(self):
        """
        Sends the torso to its upper limit

        :return: True or False
        """
        return self._send_goal(self.upper_limit)

    def medium(self):
        """
        Sends the torso to the middle position of each joint

        :return: True or False
        """
        goal = []
        # ToDo: make nice
        for i in range(0, len(self.joint_names)):
            goal.append(self.lower_limit[i] + (self.upper_limit[i] - self.lower_limit[i]) / 2)
        return self._send_goal(goal)

    def low(self):
        """
        Sends the torso to its lower limit

        :return: True or False
        """
        return self._send_goal(self.lower_limit)

    def reset(self):
        """
        Sends the torso to the reset pose

        :return:
        """
        return self.send_goal('reset')

    def cancel_goal(self):
        """
        Cancels all active goals for the torso; Doing the same as close()

        :return: no return
        """
        self.ac_move_torso.cancel_goal()
        # return True

    def wait_for_motion_done(self, timeout=10, cancel=False):
        """ Waits until all action clients are done

        :param timeout: double with time (defaults to 10.0 seconds)
        :param cancel: bool specifying whether goals should be cancelled if timeout is exceeded
        :return: bool indicates whether motion was done (True if reached, False otherwise)
        """
        if self.ac_move_torso.gh:
            self.ac_move_torso.wait_for_result(rospy.Duration(timeout))
            if self.ac_move_torso.get_state() == GoalStatus.SUCCEEDED:
                rospy.logdebug("Torso target reached")
                return True
            else:
                rospy.logerr("Reaching torso target failed")
                if cancel:
                    rospy.loginfo("Torso: cancelling all goals (1)")
                    self.cancel_goal()
                return False

    def wait(self, timeout=10):
        import warnings
        warnings.warn("Please use wait_for_motion_done instead", Warning)
        self.wait_for_motion_done(timeout)

    def get_position(self):
        """
        Get the current joint positions

        :return: list of current positions
        """
        # return self.current_position.position
        joint_states = self._get_joint_states()
        return [joint_states[joint_name] for joint_name in self.joint_names]
