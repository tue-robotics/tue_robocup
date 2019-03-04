#
#  Rein Appeldoorn
#  October '14
#

# System
import math

# ROS
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg
import rospy
import tf

# TU/e Robotics
from cb_planner_msgs_srvs.msg import LocalPlannerAction, OrientationConstraint, PositionConstraint, LocalPlannerGoal
from cb_planner_msgs_srvs.srv import GetPlan, CheckPlan
from robot_skills.robot_part import RobotPart
from robot_skills.util.kdl_conversions import kdl_frame_stamped_from_pose_stamped_msg
from robot_skills.util import nav_analyzer, transformations


class LocalPlanner(RobotPart):
    def __init__(self, robot_name, tf_listener, analyzer):
        super(LocalPlanner, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.analyzer = analyzer
        self._action_client = self.create_simple_action_client('/' + robot_name + '/local_planner/action_server',
                                                               LocalPlannerAction)

        self._position_constraint = None
        self._orientation_constraint = None

        # Public members!
        self._status = "idle"  # idle, controlling, blocked, arrived
        self._obstacle_point = None
        self._dtg = None
        self._plan = None
        self._goal_handle = None

    def setPlan(self, plan, position_constraint, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self._orientation_constraint = orientation_constraint
        self._action_client.send_goal(goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback)
        self._goal_handle = self._action_client.gh
        rospy.logdebug("Goal handle = {0}".format(self._goal_handle))
        self.__setState("controlling", None, None, plan)

    def cancelCurrentPlan(self):
        if self._goal_handle:
            state = self._action_client.get_state()
            # Only cancel goal when pending or active
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._action_client.cancel_goal()
                self.__setState("idle")

    def reset(self):
        self._action_client.cancel_all_goals()
        # cancel_all_goals doesn't return anything useful to check, so we just return True
        # https://docs.ros.org/kinetic/api/actionlib/html/simple__action__client_8py_source.html#l00192
        return True

    def getGoalHandle(self):
        return self._goal_handle

    def getStatus(self):
        return self._status

    def getDistanceToGoal(self):
        return self._dtg

    def getObstaclePoint(self):
        return self._obstacle_point

    def getCurrentOrientationConstraint(self):
        return self._orientation_constraint

    def __feedbackCallback(self, feedback):
        if feedback.blocked:
            self.__setState("blocked", feedback.point_blocked, feedback.dtg, self._plan)
        else:
            self.__setState("controlling", None, feedback.dtg, self._plan)

    def __doneCallback(self, terminal_state, result):
        self.__setState("arrived")

    def __setState(self, status, obstacle_point=None, dtg=None, plan=None):
        self._status = status
        self._obstacle_point = obstacle_point
        self._dtg = dtg
        self._plan = plan


class GlobalPlanner(RobotPart):
    def __init__(self, robot_name, tf_listener, analyzer):
        super(GlobalPlanner, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.analyzer = analyzer
        self._position_constraint = None
        self._get_plan_client = self.create_service_client("/" + robot_name + "/global_planner/get_plan_srv", GetPlan)
        self._check_plan_client = self.create_service_client("/" + robot_name +"/global_planner/check_plan_srv", CheckPlan)

    def getPlan(self, position_constraint):

        self._position_constraint = position_constraint

        pcs = [position_constraint]

        start_time = rospy.Time.now()

        try:
            resp = self._get_plan_client(pcs)
        except Exception as e:
            rospy.logerr("Could not get plan from global planner via service call, is the global planner running?")
            rospy.logerr(e)
            return None

        if not resp.succes:
            rospy.logerr("Global planner couldn't plan a path to the specified constraints. Are the constraints you specified valid?")
            return None

        end_time = rospy.Time.now()
        plan_time = (end_time-start_time).to_sec()

        path_length = self.computePathLength(resp.plan)

        return resp.plan

    def checkPlan(self, plan):
        try:
            resp = self._check_plan_client(plan)
        except:
            rospy.logerr("Could not check plan, is the global planner running?")
            return False

        return resp.valid

    def getCurrentPositionConstraint(self):
        return self._position_constraint

    def computePathLength(self, path):
        #rospy.logwarn("Please use the other computepathlength")
        distance = 0.0
        for index, pose in enumerate(path):
            if not index == 0:
                dx = path[index].pose.position.x - path[index-1].pose.position.x
                dy = path[index].pose.position.y - path[index-1].pose.position.y
                distance += math.sqrt( dx*dx + dy*dy)
        return distance


class Base(RobotPart):
    def __init__(self, robot_name, tf_listener, cmd_vel_topic=None, initial_pose_topic=None):
        super(Base, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        if cmd_vel_topic is None:
            cmd_vel_topic = '/' + robot_name + '/base/references'
        if initial_pose_topic is None:
            initial_pose_topic = '/' + robot_name + '/initialpose'
        self._cmd_vel = rospy.Publisher(cmd_vel_topic, geometry_msgs.msg.Twist, queue_size=10)
        self._initial_pose_publisher = rospy.Publisher(initial_pose_topic,
                                                       geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

        self.analyzer = nav_analyzer.NavAnalyzer(robot_name)

        # The planners
        self.global_planner = GlobalPlanner(robot_name, tf_listener, self.analyzer)
        self.local_planner = LocalPlanner(robot_name, tf_listener, self.analyzer)

        self.subscribe_hardware_status('base')

    def wait_for_connections(self, timeout):
        """ Waits for the connections until they are connected
        :param timeout: timeout in seconds
        :return: bool indicating whether all connections are connected
        """
        return self.global_planner.wait_for_connections(timeout=timeout) and \
            self.local_planner.wait_for_connections(timeout=timeout)

    def move(self, position_constraint_string, frame):
        p = PositionConstraint()
        p.constraint = position_constraint_string
        p.frame = frame
        plan = self.global_planner.getPlan(p)
        if plan:
            o = OrientationConstraint()
            o.frame = frame
            self.local_planner.setPlan(plan, p, o)

        return plan

    def force_drive(self, vx, vy, vth, timeout):
        """ Forces the robot to drive by sending a command velocity directly to the base controller. N.B.: all collision
        avoidance is bypassed.

        :param vx: forward velocity in m/s
        :param vy: sideways velocity in m/s
        :param vth: rotational velocity in rad/s
        :param timeout: duration for this motion in seconds
        """
        # Cancel the local planner goal
        self.local_planner.cancelCurrentPlan()

        v = geometry_msgs.msg.Twist()        # Initialize velocity
        t_start = rospy.Time.now()

        # Drive
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z= vth
        while (rospy.Time.now() - t_start) < rospy.Duration(timeout):
            self._cmd_vel.publish(v)
            rospy.sleep(0.1)

        # Stop driving
        v.linear.x = 0.0
        v.linear.y = 0.0
        v.angular.z = 0.0
        self._cmd_vel.publish(v)

        return True

    def get_location(self):
        """ Returns a FrameStamped with the robot pose
        :return: FrameStamped with robot pose
        """
        return get_location(self.robot_name, self.tf_listener)

    def set_initial_pose(self, x, y, phi):

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "/map"

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi)
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        # We have to do this twice for some reason, somewhere in ed_localization
        self._initial_pose_publisher.publish(initial_pose)
        rospy.sleep(0.5)
        self._initial_pose_publisher.publish(initial_pose)

        return True

    def reset(self):
        loc = self.local_planner.reset()
        glob = self.global_planner.reset()
        return loc and glob

    ########################################################
    ###### Are the following functions deprecated ??? ######
    ########################################################
    def go(self, x, y, phi, frame="/map", timeout=0):
        rospy.logwarn("[base.py] Function 'go' of 'Base' is obsolete.")
        return True

    def reset_costmap(self):
        rospy.logwarn("[base.py] Function 'reset_costmap' of 'Base' is obsolete.")
        return True

    def cancel_goal(self):
        rospy.logwarn("[base.py] Function 'cancel_goal' of 'Base' is obsolete.")
        return True

    ########################################################
    ########################################################
    ########################################################


def get_location(robot_name, tf_listener):

    try:
        time = rospy.Time.now()
        tf_listener.waitForTransform("/map", "/" + robot_name + "/base_link", time, rospy.Duration(20.0))
        (ro_trans, ro_rot) = tf_listener.lookupTransform("/map", "/" + robot_name + "/base_link", time)

        position = geometry_msgs.msg.Point()
        orientation = geometry_msgs.msg.Quaternion()

        position.x = ro_trans[0]
        position.y = ro_trans[1]
        orientation.x = ro_rot[0]
        orientation.y = ro_rot[1]
        orientation.z = ro_rot[2]
        orientation.w = ro_rot[3]

        target_pose = geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
        target_pose.header.frame_id = "/map"
        target_pose.header.stamp = time
        return kdl_frame_stamped_from_pose_stamped_msg(target_pose)

    except (tf.LookupException, tf.ConnectivityException):
        rospy.logerr("tf request failed!!!")
        target_pose = geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
        target_pose.header.frame_id = "/map"
        return kdl_frame_stamped_from_pose_stamped_msg(target_pose)


def computePathLength(path):
    distance = 0.0
    for index, pose in enumerate(path):
        if not index == 0:
            dx = path[index].pose.position.x - path[index-1].pose.position.x
            dy = path[index].pose.position.y - path[index-1].pose.position.y
            distance += math.sqrt(dx*dx + dy*dy)
    return distance
