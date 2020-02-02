# System
import math

# ROS
import PyKDL as kdl
import rospy
from visualization_msgs.msg import Marker, MarkerArray

# TU/e Robotics
from robot_smach_states.util.geometry_helpers import offsetConvexHull
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector, FrameStamped, kdl_frame_to_pose_msg


class GraspPointDeterminant(object):
    """ Computes grasp points """

    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """

        self._robot = robot
        self._marker_array_pub = rospy.Publisher('/grasp_markers', MarkerArray, queue_size=1)

        self._width_treshold = 0.1  # ToDo: make variable!!!

    def get_grasp_pose(self, entity, arm):
        """ Computes the most suitable grasp pose to grasp the specified entity with the specified arm

        :param entity: entity to grasp
        :param arm: arm to use
        :return: FrameStamped with grasp pose in map frame
        """
        candidates = []
        starttime = rospy.Time.now()
        # ToDo: divide into functions
        ''' Create a grasp vector for every side of the convex hull '''
        ''' First: check if container actually has a convex hull '''
        if entity.shape is None:
            rospy.logerr("Entity {0} has no shape. We need to do something with this".format(entity.id))
            return False

        ''' Second: turn points into KDL objects and offset chull to get it in map frame '''
        center_pose = entity._pose  #TODO: Access to private member

        # chull_obj = [point_msg_to_kdl_vector(p) for p in entity.shape._convex_hull]   # convex hull in object frame
        # chull = offsetConvexHull(chull_obj, center_pose)    # convex hull in map frame
        chull = offsetConvexHull(entity.shape.convex_hull, center_pose)  # convex hull in map frame
        # import ipdb;ipdb.set_trace()

        ''' Get robot pose as a kdl frame (is required later on) '''
        robot_frame = self._robot.base.get_location()
        robot_frame_inv = robot_frame.frame.Inverse()

        ''' Loop over lines of chull '''
        for i in xrange(len(chull)):
            j = (i+1)%len(chull)

            dx = chull[j].x() - chull[i].x()
            dy = chull[j].y() - chull[i].y()
            if math.hypot(dx, dy) < 0.0001:
                # Points are probably too close to get a decent angle estimate
                continue

            yaw = math.atan2(dx, -dy)

            ''' Filter on object width '''
            # Normalize
            n = math.hypot(dx, dy) # Norm
            vx = dx/n
            vy = dy/n

            # Loop over all points
            wmin = 0
            wmax = 0
            for c in chull:

                # Compute vector
                tx = c.x() - chull[i].x()
                ty = c.y() - chull[i].y()

                # Perform projection
                # ToDo: continue when offset in x direction is too large
                offset = tx * vx + ty * vy

                # Update min and max
                wmin = min(wmin, offset)
                wmax = max(wmax, offset)

            width = wmax - wmin

            if width > self._width_treshold:
                continue
            else:
                score = 1.0

            ''' Compute candidate vector '''
            # Middle between point i and point j
            # x = 0.5 * ( chull[i].x() + chull[j].x() )
            # y = 0.5 * ( chull[i].y() + chull[j].y() )
            # cvec = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw),
            #                kdl.Vector(x, y, entity.pose.position.z))

            # Divide width in two
             # * kdl.Vector(0.5 * (wmin+wmax, 0, 0)
            tvec = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0, 0, yaw),
                                kdl.Vector(chull[i].x(), chull[i].y(), entity.pose.frame.p.z())),
                                frame_id="/map") # Helper frame

            cvec = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0, 0, yaw),
                                tvec.frame * kdl.Vector(0, -0.5 * (wmin+wmax), 0)),
                                frame_id="/map")

            ''' Optimize over yaw offset w.r.t. robot '''
            # robot_in_map * entity_in_robot = entity_in_map
            # --> entity_in_robot = robot_in_map^-1 * entity_in_map
            gvec = robot_frame_inv * cvec.frame
            (R, P, Y) = gvec.M.GetRPY()

            rscore = 1.0 - (abs(Y)/math.pi)
            score = min(score, rscore)

            candidates.append({'vector': cvec, 'score': score})

        candidates = sorted(candidates, key=lambda candidate: candidate['score'], reverse=True)

        self.visualize(candidates)
        rospy.loginfo("GPD took %f seconds" % (rospy.Time.now() - starttime).to_sec())

        return candidates[0]['vector']

    def visualize(self, candidates):
        """ Visualizes the candidate grasp vectors

        :param candidates: list with candidates containing a vector and a score
        """
        msg = MarkerArray()
        for i, c in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = c['vector'].frame_id
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.pose = kdl_frame_to_pose_msg(c['vector'].frame)
            if i == 0: # The 'best' one is blue...
                marker.color.b = 1.0
            elif 'score' in c:
                if c['score'] <= 0.0:
                    marker.color.r = 1.0
                elif c['score'] < 0.5:
                    marker.color.r = 1.0
                    marker.color.g = 2 * c['score']
                elif c['score'] < 1.0:
                    marker.color.r = 1.0 - 2 * (c['score']-0.5)
                    marker.color.g = 1.0
                else:
                    marker.color.g = 1.0
            else:
                marker.color.b = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.lifetime = rospy.Duration(30.0)
            msg.markers.append(marker)

            ##### # If desired to plot markers one by one
            # msg.markers = [marker]
            # self._marker_array_pub.publish(msg)
            # import ipdb; ipdb.set_trace()
            #####

        self._marker_array_pub.publish(msg)




