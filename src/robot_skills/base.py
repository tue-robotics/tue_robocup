#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf
import std_msgs.msg
import std_srvs.srv
import threading
import tue_move_base_msgs.msg
import tue_move_base_msgs.srv
import tue_costmap_msgs.msg
import tue_costmap_msgs.srv
import amigo_inverse_reachability.srv
import octomap_msgs.srv

import util.concurrent_util
from util import transformations

## TODO: Stop force drive (to prevent overshoot)

class Base(object):
    """Interface to Amigo's base and (tue_)move_base"""

    _lock = threading.RLock() #Reentrant lock. Only the thread that acquired the lock can unlock it again.
    
    def __init__(self, tf_listener, wait_service=True, use_2d=None):
        self._lock = threading.RLock()

        if use_2d == None:
            try:
                #If this somehow stops working:
                #if [toplist for toplist in rospy.client.get_published_topics() if '/move_base_3d/goal' in toplist]: print "we have 3d"
                rospy.wait_for_service('/move_base/get_plan', timeout=0.5)
                rospy.loginfo("2d navigation is running, so I'll use that")
                use_2d = True
            except rospy.exceptions.ROSException:
                rospy.loginfo("2d navigation is not running, so I assume 3D *is* running")
                use_2d = False

        if use_2d:
            self.planner_feedback = rospy.Subscriber('/move_base/AStarPlannerROS/result',std_msgs.msg.Bool, self.__planner_callback) #/move_base_3d/AStarPlannerROS/result
        else:
            self.planner_feedback = rospy.Subscriber('/move_base_3d/AStarPlannerROS/result',std_msgs.msg.Bool, self.__planner_callback)

        
        self.cmd_vel = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist)
        
        self.move_simple_base = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped)

        self.initial_pose_publisher = rospy.Publisher('/initialpose',geometry_msgs.msg.PoseWithCovarianceStamped)

        self.tf_listener    = tf_listener
        # WORKAROUND for missing transform between /map and /amigo/base_link frame, to make sure that TF is correctly initialised
        rospy.sleep(2)
        rospy.loginfo("Done with waiting for TF to become avaiable")

        #clear obstacles
        if use_2d:
            self.clear_service  = rospy.ServiceProxy('/move_base/clear_costmap',   std_srvs.srv.Empty)
        else:
            self.clear_service  = rospy.ServiceProxy('/move_base_3d/clear_bbx',       octomap_msgs.srv.BoundingBoxQuery)
            self.unknown_to_free_service  = rospy.ServiceProxy('/move_base_3d/set_unknown_to_free_bbx',  octomap_msgs.srv.BoundingBoxQuery)
            self.reset_costmap_service = rospy.ServiceProxy('/move_base_3d/reset', std_srvs.srv.Empty)

        #Get plan
        if use_2d:
            self._get_plan_service      = rospy.ServiceProxy('/move_base/get_plan',         tue_move_base_msgs.srv.GetPath)
        else:
            self._get_plan_service      = rospy.ServiceProxy('/move_base_3d/get_plan',         tue_move_base_msgs.srv.GetPath)
        
        #query costmap
        if use_2d:
            self._query_costmap = rospy.ServiceProxy('/move_base/query_costmap',    tue_costmap_msgs.srv.PointQuery)
        else:
            self._query_costmap = rospy.ServiceProxy('/move_base_3d/query_costmap',    tue_costmap_msgs.srv.PointQuery)

        self._get_base_goal_poses = rospy.ServiceProxy('/inverse_reachability/inverse_reachability', amigo_inverse_reachability.srv.GetBaseGoalPoses)
        
        if use_2d:
            self.ac_move_base   = actionlib.SimpleActionClient('/move_base',        tue_move_base_msgs.msg.MoveBaseAction)
        else:
            self.ac_move_base   = actionlib.SimpleActionClient('/move_base_3d',        tue_move_base_msgs.msg.MoveBaseAction)

        if wait_service:
            rospy.loginfo("waiting for move base server in Base.__init__")
            self.ac_move_base.wait_for_server(timeout=rospy.Duration(2.0))
        
        self.plan_possible  = None
        self.poses_to_goal = 0
        self.replan_timeout = 0
        self.base_pose = None
        self.obstacle_position = None
        self.use_2d = use_2d # Necessary to switch between 2D and 3D

        # By Sjoerd, ask him:
        self.path_blocked = False
        self.reached_blocked_timeout = False
        self.last_replan_timeout = 0
               
    def __planner_callback(self, msg):
        with self._lock:
            self.plan_possible = msg.data
    
    def get_plan(self, position, orientation, frame_id="/map", goal_area_radius=0.1):
        target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
        target_pose.header.frame_id = frame_id

        return self.__get_plan(target_pose)

    def get_plan_OLD(self, position, orientation, frame_id="/map", goal_area_radius=0.1):
        path_request = tue_move_base_msgs.srv.GetPathRequest()

        path_request.target_pose.header.frame_id = frame_id
        path_request.target_pose.header.stamp = rospy.Time.now()
        path_request.target_pose.pose.position.x = position.x
        path_request.target_pose.pose.position.y = position.y 
        path_request.target_pose.pose.orientation = orientation

        try:
            #path_request.path_resolution = 1
            #path_request.goal_area_size = 0
            #path_request.nr_area_samples = 1
            path_request.goal_area_radius = goal_area_radius
        except AttributeError, ae:
            rospy.logerr("Attribute could not be set, please update to correct to move_base_msgs: {0}".format(ae))
            
        path = self._get_plan_service(path_request)

        if not path:
            rospy.logwarn("No path could be found to get to target pose {0}".format(path_request.target_pose))

        return path

    def __get_plan(self, target_pose, goal_area_radius=0.1):
        path_request = tue_move_base_msgs.srv.GetPathRequest()

        path_request.target_pose = target_pose

        try:
            #path_request.path_resolution = 1
            #path_request.goal_area_size = 0
            #path_request.nr_area_samples = 1
            path_request.goal_area_radius = goal_area_radius
        except AttributeError, ae:
            rospy.logerr("Attribute could not be set, please update to correct to move_base_msgs: {0}".format(ae))
            
        path = self._get_plan_service(path_request)

        if not path:
            rospy.logwarn("No path could be found to get to target pose {0}".format(path_request.target_pose))

        return path


    @util.concurrent_util.synchronized(_lock)
    def execute_plan(self, path, time=0, block=True):
        base_goal = tue_move_base_msgs.msg.MoveBaseGoal()
        base_goal.path = path.path

        self.ac_move_base.send_goal(base_goal, feedback_cb=self.movebase_feedback)

        goalformat = " call made to execute plan of length {0} to ({1.x}, {1.y})".format(len(path.path), path.path[-1:][0].pose.position)

        #TODO Loy & Bas: Use percent_complete from feedback to monitor the plan. Somehow. opercent_complete is not yet implemented in tue_move_base

        if time == 0 and not block:
            rospy.loginfo("Nonblocking" + goalformat)
            return True
        elif time == 0 and block:
            rospy.loginfo("Blocking " + goalformat)
            self.ac_move_base.wait_for_result()
            return (self.ac_move_base.get_state() == 3)
        else:
            rospy.loginfo("Waiting {0}s on ".format(time) + goalformat)
            result = self.ac_move_base.wait_for_result(rospy.Duration(time))
            return (self.ac_move_base.get_state() == 3) and result

    @util.concurrent_util.synchronized(_lock)
    def movebase_feedback(self, feedback, result=None):
        self.obstacle_position      = feedback.obstacle_position
        self.poses_to_goal          = feedback.nr_poses_to_goal
        self.base_pose              = feedback.base_position
        self.path                   = feedback.path
        self.replan_timeout         = feedback.nr_sec_till_replan_execution

        if self.replan_timeout < 0:
            self.replan_timeout = 0  # Move_base sometimes returns negative times; this should be fixed, but before
                                     # that time, we can simply deal with it this way    

        if self.replan_timeout > 0:
            self.path_blocked = True

            if self.replan_timeout < 1.0:
                self.reached_blocked_timeout = True
            else:
                self.reached_blocked_timeout = False

        else:
            self.path_blocked = False        

        #rospy.loginfo("Feedback from move_base = {0}".format(feedback))

        #print feedback.obstacle_position

        #print self.poses_to_goal, self.base_pose_in_path

        if result:
            print result

    def query_costmap(self, points, frame_id="/map"):
        query_points = []
        for p in points:
            if isinstance(p, geometry_msgs.msg.Point):
                stamped_point = geometry_msgs.msg.PointStamped()
                stamped_point.header.frame_id = frame_id
                stamped_point.point = p

                query_points += [stamped_point]

            if isinstance(p, geometry_msgs.msg.PointStamped):
                query_points += [p]

        query = tue_costmap_msgs.srv.PointQueryRequest()
        query.points = query_points
        points_info = self._query_costmap(query)

        #import ipdb; ipdb.set_trace()
        query_result_tuples = [pointinfo for pointinfo in points_info.points_info]

        return query_result_tuples

    def send_goal(self, position, orientation_quaternion, frame_id ='/map', time=0, block=True, goal_area_radius=0.1):
        path_result = self.get_plan(position, orientation_quaternion, frame_id, goal_area_radius)

        self.path = path_result.path

        rospy.loginfo("Sending goal to {0}".format(position).replace('\n', ' '))

        if not self.path:
            rospy.loginfo("No feasible plan to ({0.x},{0.y})".format(position))
            return False
        else:
            result = self.execute_plan(path_result, time, block)
            return result

    def send_goal_topic(self, pos, orientation_quaternion, frame_id='/map'):
        pos.z = 0
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id = frame_id
        base_goal = geometry_msgs.msg.PoseStamped(header,geometry_msgs.msg.Pose(position=pos, orientation=orientation_quaternion))
        self.running = True
        self.move_simple_base.publish(base_goal)
        
    def cancel_goal(self):
        self.ac_move_base.cancel_all_goals() #Does not return anything
        return True
    
    def wait(self, wait_time=10):
        self.ac_move_base.wait_for_result(rospy.Duration(wait_time))
        
        if self.ac_move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Base target reached")
            return True
        else:
            rospy.loginfo("Reaching base target failed")
            return False
    
    def force_drive(self, vx,vy,vthe,time):
        
        # Cancel base goal to prevent interference between 
        # move_base and force_drive
        self.cancel_goal()
        
        velocity = geometry_msgs.msg.Twist()        # Initialize velocity
        start_time = rospy.Time.now()
        
        # Drive
        velocity.linear.x = vx
        velocity.linear.y = vy
        velocity.angular.z= vthe
        while (rospy.Time.now() - start_time) < rospy.Duration(time):
            self.cmd_vel.publish(velocity)
            rospy.sleep(0.1)
        
        # Stop driving
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.angular.z= 0.0
        self.cmd_vel.publish(velocity)
        
        return True
    
    def get_location(self):
        try:
            #tf_listener = tf.TransformListener() 
            time = rospy.Time.now()
            self.tf_listener.waitForTransform("/map", "/amigo/base_link", time, rospy.Duration(20.0))
            (ro_trans, ro_rot) = self.tf_listener.lookupTransform("/map", "/amigo/base_link", time)
            
            position = geometry_msgs.msg.Point()
            orientation = geometry_msgs.msg.Quaternion()
            
            position.x = ro_trans[0]
            position.y = ro_trans[1]
            orientation.x = ro_rot[0]
            orientation.y = ro_rot[1]
            orientation.z = ro_rot[2]
            orientation.w = ro_rot[3]
            return position, orientation
        
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("tf request failed!!!")
            return position, orientation
        
    location = property(get_location)
    
    def close(self):
        try:
            rospy.loginfo("Base cancelling goal on close")
        except AttributeError:
            print "Base cancelling goal on close. Rospy is already deleted."
        
        self.cancel_goal()
        
    def point(self, x, y, z=0, stamped=False, frame_id="/map"):
        """
        Helper method for creating a geometry_msgs Point
        """
        p = geometry_msgs.msg.Point(x=x, y=y, z=z)

        if stamped:
            ps = geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(frame_id=frame_id), point=p)
            return ps
        else:
            return p
    
    def orient(self, phi):
        return transformations.euler_z_to_quaternion(phi)

    def phi(self, orient):
        return transformations.euler_z_from_quaternion(orient)
    
    def go(self, x, y, phi, frame="/map", time=0):
        return self.send_goal(self.point(x,y), self.orient(phi), frame, time)
    
    def clear_costmap(self, window_size=1.0):
        if self.use_2d:
            self.clear_service()
            rospy.sleep(rospy.Duration(2.0))
        else:
            if self.base_pose == None:
                # If no path request has been given yet, self.base_pose equals None, but we might get the base pose from tf
                [position, orientation] = self.get_location()
                pos_x = position.x
                pos_y = position.y
            else:
                pos_x = self.base_pose.pose.position.x
                pos_y = self.base_pose.pose.position.y

            bbx_request = octomap_msgs.srv.BoundingBoxQueryRequest()
            bbx_request.min.x = pos_x - (window_size/2)
            bbx_request.min.y = pos_y - (window_size/2)
            bbx_request.min.z = -1
            bbx_request.max.x = pos_x + (window_size/2)
            bbx_request.max.y = pos_y + (window_size/2)
            bbx_request.max.z = 2
            self.clear_service(bbx_request)
            # Sleep for a while to ensure the map is cleared and obstacles are inserted again
            rospy.sleep(rospy.Duration(2.0))
    
    def free_unknown_space(self, window_size=0.2):
        if self.use_2d:
            rospy.logwarn("this case is not implemented!")
        else:
            if self.base_pose == None:
                # If no path request has been given yet, self.base_pose equals None, but we might get the base pose from tf
                [position, orientation] = self.get_location()
                pos_x = position.x
                pos_y = position.y
            else:
                pos_x = self.base_pose.pose.position.x
                pos_y = self.base_pose.pose.position.y

            bbx_request = octomap_msgs.srv.BoundingBoxQueryRequest()
            bbx_request.min.x = pos_x - (window_size/2)
            bbx_request.min.y = pos_y - (window_size/2)
            bbx_request.min.z = -1
            bbx_request.max.x = pos_x + (window_size/2)
            bbx_request.max.y = pos_y + (window_size/2)
            bbx_request.max.z = 2
            self.unknown_to_free_service(bbx_request)
            # Sleep for a while to ensure the map is cleared and obstacles are inserted again
            rospy.sleep(rospy.Duration(2.0))

    def reset_costmap(self):
        if self.use_2d:
            try:
                self.clear_service()
                # Sleep for a while to ensure the map is cleared and obstacles are inserted again
                rospy.sleep(rospy.Duration(2.0))
            except:
                rospy.logerr("Clear costmap service does not return correctly")
            return True
        else:
            try:
                self.reset_costmap_service()
                # Sleep for a while to ensure the map is cleared and obstacles are inserted again
                rospy.sleep(rospy.Duration(2.0))
            except:
                rospy.logerr("Clear costmap service does not return correctly")
            return True

        
    def get_base_goal_poses(self, target_point_stamped, x_offset, y_offset, cost_threshold_norm=0.2):

        request = amigo_inverse_reachability.srv.GetBaseGoalPosesRequest()
        
        [position, orientation] = self.get_location()
        robot_pose = geometry_msgs.msg.Pose()
        robot_pose.position = position
        robot_pose.orientation = orientation
        request.robot_pose = robot_pose

        request.target_pose.position = target_point_stamped.point
        request.target_pose.orientation.x = 0
        request.target_pose.orientation.y = 0
        request.target_pose.orientation.z = 0
        request.target_pose.orientation.w = 1

        request.cost_threshold_norm = cost_threshold_norm
        
        request.x_offset = x_offset
        request.y_offset = y_offset
        rospy.logdebug("Inverse reachability request = {0}".format(request).replace("\n", " ").replace("\t", " "))
        response = self._get_base_goal_poses(request)
        rospy.logdebug("Inverse reachability response = {0}".format(response).replace("\n", " ").replace("\t", " "))
        
        base_goal_poses = []

        for base_goal_pose in response.base_goal_poses:
            base_goal_poses.append(geometry_msgs.msg.PoseStamped())
            base_goal_poses[-1].header.frame_id = "/map"
            base_goal_poses[-1].header.stamp = rospy.Time()
            base_goal_poses[-1].pose = base_goal_pose

        return base_goal_poses

    def set_initial_pose(self, x, y, phi):

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        
        initial_pose.header.frame_id = "/map"
        
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi) 
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        self.initial_pose_publisher.publish(initial_pose)

        return True

if __name__ == "__main__":
    rospy.init_node("amigo_base_executioner", anonymous=True)

    tf_listener = tf.TransformListener()

    base = Base(tf_listener)
