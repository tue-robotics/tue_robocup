from cb_planner_msgs_srvs.msg import PositionConstraint

import math
import rospy
from robot_smach_states.util.designators import Designator
import geometry_msgs.msg as gm
from visualization_msgs.msg import MarkerArray, Marker
import robot_skills.util.msg_constructors as msg_constructors
from robot_skills.util.kdl_conversions import FrameStamped, pose_msg_to_kdl_frame, kdl_frame_stamped_from_XYZRPY
import PyKDL as kdl
import copy



class EmptyShelfDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by querying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot.

    To test this in the robotics_test_lab with amigo-console:
    robot = amigo
    CABINET = "bookcase"
    PLACE_SHELF = "shelf2"
    cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
    place_position = ds.LockingDesignator(ds.EmptyShelfDesignator(robot, cabinet, name="placement", area=PLACE_SHELF), name="place_position")
    """
    def __init__(self, robot, place_location_designator, name=None, area=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity

        :param robot: Robot whose worldmodel to use
        :param place_location_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param name: name for introspection purposes
        :param area: (optional) area where the item should be placed
        """
        super(EmptyShelfDesignator, self).__init__(resolve_type=FrameStamped, name=name)
        self.robot = robot

        if area is None:
            rospy.logerr("EmptyShelfDesignator: please provide an area")

        self.place_location_designator = place_location_designator
        self._edge_distance = 0.06                   # Distance to table edge
        self._spacing = 0.12
        self._area = area

        self.marker_pub = rospy.Publisher('/empty_spots', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

        self._candidate_list_obj = []  # List of candidate points in object frame

        self._count = 0

    def _resolve(self):
        place_location = self.place_location_designator.resolve()

        framestamped_of_interest = self.determine_points_of_interest_with_area(place_location, self._area)

        # # Feasible POIS: discard
        # feasible_POIs = []
        # for tup in open_POIs_dist:
        #     if tup[1]:
        #          feasible_POIs.append(tup)
        #
        # if any(feasible_POIs):
        #     feasible_POIs.sort(key=lambda tup: tup[1])  # sorts in place
        #     best_poi = feasible_POIs[0][0] # Get the POI of the best match
        #     placement = msg_constructors.PoseStamped(pointstamped=best_poi)
        #     # rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
        #
        #     selection = self.create_selection_marker(placement)
        #     self.marker_pub.publish(MarkerArray([selection]))
        #
        #     return placement
        # else:
        #     rospy.logerr("Could not find an empty spot")
        #     return None

        if self._count >= 5:
            rospy.logerr("Could not find an empty spot")
            return None
        else:
            fs = framestamped_of_interest[self._count]
            self._count += 1
            rospy.loginfo("Place pose at: {0}".format(fs))
            return fs


    def create_marker(self, x, y, z, frame_id="/map"):
        marker = Marker()
        marker.id = len(self.marker_array.markers)
        marker.type = 2
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1
        marker.color.a = 1

        marker.lifetime = rospy.Duration(10.0)
        return marker

    def create_selection_marker(self, selected_pose):
        marker = Marker()
        marker.id = len(self.marker_array.markers) + 1
        marker.type = 2
        marker.header.frame_id = selected_pose.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = selected_pose.vector.p.x()
        marker.pose.position.y = selected_pose.vector.p.y()
        marker.pose.position.z = selected_pose.vector.p.z()
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1
        marker.color.a = 0.7

        marker.lifetime = rospy.Duration(30.0)
        return marker

    def determine_points_of_interest_with_area(self, e, area):
        """
        Determines the points of interest using an area

        :param e:
        :param area:
        :return:
        """
        # Just to be sure, copy e
        e = self.robot.ed.get_entity(id=e.id, parse=True)

        # We want to give it a convex hull using the designated area
        if area in e.volumes:
            box = e.volumes[area]
        else:
            rospy.logwarn("Entity {0} has no volume named {1}".format(e.id, area))

        if not self._candidate_list_obj:
            for y in [0, self._spacing, -self._spacing, 2.0 * self._spacing, -2.0 * self._spacing]:
                if y < box.min_corner.y() or y > box.max_corner.y():
                    rospy.logerr("Spacing of empty spot designator is too large!!!")
                    continue

                frame_stamped = kdl_frame_stamped_from_XYZRPY(x=box.max_corner.x() - self._edge_distance,
                                                              y=y,
                                                              z=box.min_corner.z() - 0.04,  # 0.04 is the usual offset)
                                                              frame_id=e.id)

                # e.convex_hull = []
                # e.convex_hull.append(gm.Point(box['min']['x'], box['min']['y'], box['min']['z']))  # 1
                # e.convex_hull.append(gm.Point(box['max']['x'], box['min']['y'], box['min']['z']))  # 2
                # e.convex_hull.append(gm.Point(box['max']['x'], box['max']['y'], box['min']['z']))  # 3
                # e.convex_hull.append(gm.Point(box['min']['x'], box['max']['y'], box['min']['z']))  # 4
                #
                # # Make sure we overwrite the e.z_max
                # e.z_max = box['min']['z'] - 0.04  # 0.04 is the usual offset
                # return self.determinePointsOfInterest(e)

                self._candidate_list_obj.append(frame_stamped)

        # publish marker
        self.marker_array = MarkerArray()
        for frame_stamped in self._candidate_list_obj:
            self.marker_array.markers.append(self.create_marker(frame_stamped.frame.p.x(),
                                                                frame_stamped.frame.p.y(),
                                                                frame_stamped.frame.p.z(),
                                                                e.id))
        self.marker_pub.publish(self.marker_array)

        return self._candidate_list_obj


    # def determinePointsOfInterest(self, e):
    #
    #     points = []
    #
    #     x = e.pose.position.x
    #     y = e.pose.position.y
    #
    #     if len(e.convex_hull) == 0:
    #         rospy.logerr('Entity: {0} has an empty convex hull'.format(e.id))
    #         return []
    #
    #     ''' Convert convex hull to map frame '''
    #
    #     center_pose = pose_msg_to_kdl_frame(e.pose)
    #     ch = list()
    #     for point in e.convex_hull:
    #         p = point_msg_to_kdl_vector(point)
    #         # p = center_pose * p
    #         # p = p * center_pose
    #         pf = kdl.Frame(kdl.Rotation(), p)
    #         # pf = pf * center_pose  # Original
    #         pf = center_pose * pf  # Test
    #         p = pf.p
    #         ch.append(copy.deepcopy(p))  # Needed to fix "RuntimeError: underlying C/C++ object has been deleted"
    #
    #     ''' Loop over hulls '''
    #     self.marker_array.markers = []
    #
    #     ch.append(ch[0])
    #     for i in xrange(len(ch) - 1):
    #             dx = ch[i+1].x() - ch[i].x()
    #             dy = ch[i+1].y() - ch[i].y()
    #             length = math.hypot(dx, dy)
    #
    #             d = self._edge_distance
    #             while d < (length-self._edge_distance):
    #
    #                 ''' Point on edge '''
    #                 xs = ch[i].x() + d/length*dx
    #                 ys = ch[i].y() + d/length*dy
    #
    #                 ''' Shift point inwards and fill message'''
    #                 ps = gm.PointStamped()
    #                 ps.header.frame_id = "/map"
    #                 ps.point.x = xs - dy/length * self._edge_distance
    #                 ps.point.y = ys + dx/length * self._edge_distance
    #                 ps.point.z = e.pose.position.z + e.z_max
    #                 points.append(ps)
    #
    #                 self.marker_array.markers.append(self.create_marker(ps.point.x, ps.point.y, ps.point.z))
    #
    #                 # ToDo: check if still within hull???
    #                 d += self._spacing
    #
    #     self.marker_pub.publish(self.marker_array)
    #
    #     return points
