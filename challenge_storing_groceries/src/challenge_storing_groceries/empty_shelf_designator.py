import rospy
from robot_smach_states.util.designators import Designator
from visualization_msgs.msg import MarkerArray, Marker
from robot_skills.util.kdl_conversions import FrameStamped, kdl_frame_stamped_from_XYZRPY


class EmptyShelfDesignator(Designator):
    """
    Designates an empty spot on the empty placement-shelve.
    It does this by querying ED for entities that occupy some space. If the result is no entities, then we found an open spot.

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

        points_of_interest = self.determine_points_of_interest_with_area(place_location, self._area)

        if self._count >= 5:
            rospy.logerr("Could not find an empty spot")
            return None
        else:
            ret = points_of_interest[self._count]
            self._count += 1
            rospy.loginfo("Place pose at: {0}".format(ret))
            return ret


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
        marker.header.frame_id = selected_pose.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = selected_pose.pose.position.x
        marker.pose.position.y = selected_pose.pose.position.y
        marker.pose.position.z = selected_pose.pose.position.z
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

                fs = kdl_frame_stamped_from_XYZRPY(frame_id=e.id,
                                                   x=box.max_corner.x() - self._edge_distance,
                                                   y=y,
                                                   z=box.min_corner.z() - 0.04)  # 0.04 is the usual z offset

                self._candidate_list_obj.append(fs)

        # publish marker
        self.marker_array = MarkerArray()
        for ps in self._candidate_list_obj:
            self.marker_array.markers.append(self.create_marker(fs.vector.p.x,
                                                                fs.vector.p.y,
                                                                fs.vector.p.z,
                                                                e.id))
        self.marker_pub.publish(self.marker_array)

        return self._candidate_list_obj
