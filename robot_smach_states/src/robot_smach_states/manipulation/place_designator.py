# System
import rospy
import math

# ROS
import PyKDL as kdl
from visualization_msgs.msg import MarkerArray, Marker

# TUe robotics
from robot_skills.util.kdl_conversions import kdl_frame_stamped_from_XYZRPY, FrameStamped
from robot_smach_states.util.designators.ed_designators import Designator
from cb_planner_msgs_srvs.msg import PositionConstraint
from robot_smach_states.util.geometry_helpers import offsetConvexHull


class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by querying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot.

    To test this in the robotics_test_lab with amigo-console:
    robot = amigo
    CABINET = "bookcase"
    PLACE_SHELF = "shelf2"
    cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
    arm = ds.UnoccupiedArmDesignator(robot, {})
    place_position = ds.LockingDesignator(EmptySpotDesignator(robot, cabinet, arm, name="placement", area=PLACE_SHELF),
                                          name="place_position")
    """

    def __init__(self, robot, place_location_designator, arm_designator, name=None, area=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity
        :param robot: Robot whose worldmodel to use
        :param place_location_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param arm_designator: Designator resolving to an arm robot part, e.g. OccupiedArmDesignator
        :param name: name for introspection purposes
        :param area: (optional) area where the item should be placed
        :param nav_threshold:
        """
        super(EmptySpotDesignator, self).__init__(resolve_type=FrameStamped, name=name)
        self.robot = robot

        self.place_location_designator = place_location_designator
        self.arm_designator = arm_designator
        self._edge_distance = 0.05  # Distance to table edge
        self._spacing = 0.15
        self._area = area
        self._nav_threshold = 0.3   # Distance we are willing to drive further for better edge_score

        self.marker_pub = rospy.Publisher('/empty_spots', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def _resolve(self):
        """
        :return: Where can an object be placed
        :returns: FrameStamped
        """
        place_location = self.place_location_designator.resolve()
        place_frame = FrameStamped(frame=place_location._pose, frame_id="/map")

        # points_of_interest = []
        if self._area:
            vectors_of_interest = self._determine_points_of_interest_with_area(place_location, self._area)
        else:
            vectors_of_interest = self._determine_points_of_interest(place_frame.frame, z_max=place_location.shape.z_max,
                                                                    convex_hull=place_location.shape.convex_hull)

        assert all(isinstance(v, FrameStamped) for v in vectors_of_interest)

        open_POIs = filter(lambda pose: self._is_poi_unoccupied(pose, place_location), vectors_of_interest)

        base_pose = self.robot.base.get_location()
        arm = self.arm_designator.resolve()
        open_POIs_dist = [(poi, self._distance_to_poi_area_heuristic(poi, base_pose, arm)) for poi in open_POIs]

        # We don't care about small differences
        open_POIs_dist.sort(key=lambda tup:tup[1])
        open_POIs_dist = [f for f in open_POIs_dist if (f[1] - open_POIs_dist[0][1]) < self._nav_threshold]

        open_POIs_dist.sort(key=lambda tup: tup[0].edge_score, reverse=True) # sorts in place

        for poi in open_POIs_dist:
            if self._distance_to_poi_area(poi[0], arm):
                selection = self._create_selection_marker(poi[0])
                self.marker_pub.publish(MarkerArray([selection]))
                return poi[0]

        rospy.logerr("Could not find an empty spot")
        return None

    def _is_poi_unoccupied(self, frame_stamped, surface_entity):
        entities_at_poi = self.robot.ed.get_entities(center_point=frame_stamped.extractVectorStamped(),
                                                     radius=self._spacing)
        entities_at_poi = [entity for entity in entities_at_poi if entity.id != surface_entity.id]
        return not any(entities_at_poi)

    def _distance_to_poi_area_heuristic(self, frame_stamped, base_pose, arm):
        """
        :return: direct distance between a point and and the place offset of the arm
        :rtype: double [meters]
        """
        bo = arm.base_offset

        offset_pose = base_pose.frame * bo
        offset_pose_x = offset_pose[0]
        offset_pose_y = offset_pose[1]

        x = frame_stamped.frame.p.x()
        y = frame_stamped.frame.p.y()

        dist = math.hypot(offset_pose_x - x, offset_pose_y - y)
        return dist

    def _distance_to_poi_area(self, frame_stamped, arm):
        """
        :return: length of the path the robot would need to drive to place at the given point
        :rtype: int [plan steps]
        """
        base_offset = arm.base_offset
        radius = math.hypot(base_offset.x(), base_offset.y())

        x = frame_stamped.frame.p.x()
        y = frame_stamped.frame.p.y()
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
        pos_constraint = PositionConstraint(constraint=ri + " and " + ro, frame=frame_stamped.frame_id)

        plan_to_poi = self.robot.base.global_planner.getPlan(pos_constraint)

        if plan_to_poi:
            distance = len(plan_to_poi)
            # print "Distance to {fs}: {dist}".format(dist=distance, fs=frame_stamped.frame.p)
        else:
            distance = None
        return distance

    def _create_marker(self, x, y, z):
        marker = Marker()
        marker.id = len(self.marker_array.markers)
        marker.type = 2
        marker.header.frame_id = "/map"
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

    def _create_selection_marker(self, selected_pose):
        marker = Marker()
        marker.id = len(self.marker_array.markers) + 1
        marker.type = 2
        marker.header.frame_id = selected_pose.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = selected_pose.frame.p.x()
        marker.pose.position.y = selected_pose.frame.p.y()
        marker.pose.position.z = selected_pose.frame.p.z()
        marker.pose.orientation.w = 1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.g = 1
        marker.color.a = 0.7

        marker.lifetime = rospy.Duration(30.0)
        return marker

    def _determine_points_of_interest_with_area(self, entity, area):
        """ Determines the points of interest using an area
        :type entity: Entity
        :param area: str indicating which volume of the entity to look at
        :rtype: [FrameStamped]
        """

        # We want to give it a convex hull using the designated area

        if area not in entity.volumes:
            return []

        box = entity.volumes[area]

        if not hasattr(box, "bottom_area"):
            rospy.logerr("Entity {0} has no shape with a bottom_area".format(entity.id))

        # Now we're sure to have the correct bounding box
        # Make sure we offset the bottom of the box
        top_z = box.min_corner.z() - 0.04  # 0.04 is the usual offset
        return self._determine_points_of_interest(entity._pose, top_z, box.bottom_area)

    def _determine_points_of_interest(self, center_frame, z_max, convex_hull):
        """
        Determine candidates for place poses
        :param center_frame: kdl.Frame, center of the Entity to place on top of
        :param z_max: float, height of the entity to place on, w.r.t. the entity
        :param convex_hull: [kdl.Vector], convex hull of the entity
        :return: [FrameStamped] of candidates for placing
        """

        points = []

        if len(convex_hull) == 0:
            rospy.logerr('determine_points_of_interest: Empty convex hull')
            return []

        # Convert convex hull to map frame
        ch = offsetConvexHull(convex_hull, center_frame)

        # Loop over hulls
        self.marker_array.markers = []

        for i in xrange(len(ch)):
            j = (i + 1) % len(ch)

            dx = ch[j].x() - ch[i].x()
            dy = ch[j].y() - ch[i].y()

            length = kdl.diff(ch[j], ch[i]).Norm()

            d = self._edge_distance
            while d < (length - self._edge_distance):
                # Point on edge
                xs = ch[i].x() + d / length * dx
                ys = ch[i].y() + d / length * dy

                # Shift point inwards and fill message
                fs = kdl_frame_stamped_from_XYZRPY(x=xs - dy / length * self._edge_distance,
                                                   y=ys + dx / length * self._edge_distance,
                                                   z=center_frame.p.z() + z_max,
                                                   frame_id="/map")

                # It's nice to put an object on the middle of a long edge. In case of a cabinet, e.g., this might
                # prevent the robot from hitting the cabinet edges
                # print "Length: {}, edge score: {}".format(length, min(d, length-d))
                setattr(fs, 'edge_score', min(d, length-d))

                points += [fs]

                self.marker_array.markers.append(self._create_marker(fs.frame.p.x(), fs.frame.p.y(), fs.frame.p.z()))

                # ToDo: check if still within hull???
                d += self._spacing

        self.marker_pub.publish(self.marker_array)

        return points

    def __repr__(self):
        return "EmptySpotDesignator(place_location_designator={}, name='{}', area='{}')"\
                    .format(self.place_location_designator, self._name, self._area)
