# System
import rospy
import math
from numpy import arange

# ROS
from visualization_msgs.msg import MarkerArray, Marker

# TUe robotics
from pykdl_ros import FrameStamped, VectorStamped
from robot_smach_states.util.designators.ed_designators import Designator


class NearObjectSpotDesignator(Designator):
    """Designates an empty spot near another object.
    It does this by querying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot.
    """

    def __init__(self, robot, near_entity_designator, supporting_entity_designator, name=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity
        :param robot: Robot whose worldmodel to use
        :param near_entity_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param supporting_entity_designator: Designator resolving to an Entity
        :param name: name for introspection purposes
        """
        super(NearObjectSpotDesignator, self).__init__(resolve_type=FrameStamped, name=name)
        self.robot = robot

        self.near_entity_designator = near_entity_designator
        self.supporting_entity_designator = supporting_entity_designator
        self._edge_distance = 0.05  # Distance to table edge
        self._spacing = 0.05
        self._radius = 0.2  # desired distance between near_entity and the place pose

        self.marker_pub = rospy.Publisher('/empty_spots', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def _resolve(self):
        """
        :return: Where can an object be placed
        :returns: FrameStamped
        """
        near_entity = self.near_entity_designator.resolve()
        supporting_entity = self.supporting_entity_designator.resolve()
        #TODO derive area from the position of near_entity in supporting entity

        open_POIs = self._determine_points_of_interest(near_entity, self._radius, self._spacing)

        assert all(isinstance(v, FrameStamped) for v in open_POIs)

        # filter poi's that fall outside of the supporting surface
        #open_POIs = filter(lambda pose: self._is_poi_in_area(pose, supporting_entity, area), open_POIs)

        # filter poi's occupied by other entities
        open_POIs = list(filter(lambda pose: self._is_poi_unoccupied(pose, supporting_entity), open_POIs))
        if not open_POIs:
            rospy.logdebug("No suitable place position found")
            return None
        self._create_markers_from_pois(open_POIs)

        # sort POI's based on distance
        base_pose = self.robot.base.get_location()
        open_POIs_dist = [(poi, self._distance_to_poi_area_heuristic(poi, base_pose)) for poi in open_POIs]
        open_POIs_dist.sort(key=lambda tup:tup[1])

        selection = self._create_selection_marker(open_POIs_dist[0][0])
        self.marker_array.markers.append(selection)
        self.marker_pub.publish(self.marker_array)
        return open_POIs_dist[0][0]

    def _is_poi_unoccupied(self, poi, surface_entity):
        """
        Check whether or not the poi is occupied by another entity
        :param poi: FrameStamped
        :param surface_entity: Entity
        :return: bool
        """
        entities_at_poi = self.robot.ed.get_entities(center_point=VectorStamped(poi.frame.p, poi.header.stamp, poi.header.frame_id),
                                                     radius=0.05)
        entities_at_poi = [entity for entity in entities_at_poi if entity.uuid != surface_entity.uuid]
        return not any(entities_at_poi)

    def _is_poi_in_area(self, poi, entity, area):
        """
        Check whether or not a poi is within an area

        :param poi: FrameStamped
        :param entity: Entity to place on
        :param area: Area to place in
        :return: bool
        """
        if area not in entity.volumes:
            rospy.logerr_throttle("{} not an area of {}".format(area, entity.uuid))
            return False

        poi_in_entity_frame = poi.projectToFrame(entity.frame_id, self.robot.tf_listener)
        return entity.in_volume(poi_in_entity_frame.extractVectorStamped(), area)

    def _distance_to_poi_area_heuristic(self, frame_stamped, base_pose):
        """
        :param frame_stamped: FrameStamped
        :param base_pose: FrameStamped
        :return: direct distance in meters between a point and the robot
        """
        pose_x = base_pose.frame.p[0]
        pose_y = base_pose.frame.p[1]

        x = frame_stamped.frame.p.x()
        y = frame_stamped.frame.p.y()

        dist = math.hypot(pose_x - x, pose_y - y)
        return dist

    def _create_markers_from_pois(self, pois):
        for poi in pois:
            self.marker_array.markers.append(self._create_marker(poi.frame.p.x(), poi.frame.p.y(), poi.frame.p.z()))

    def _create_marker(self, x, y, z):
        marker = Marker()
        marker.id = len(self.marker_array.markers)
        marker.type = 2
        marker.header.frame_id = "map"
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
        marker.header.frame_id = selected_pose.header.frame_id
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

    def _determine_points_of_interest(self, near_entity, radius, spacing):
        near_place = near_entity.pose.frame.p
        points = []
        for a in arange(0, 2*math.pi, spacing/radius):
            dx = radius * math.cos(a)
            dy = radius * math.sin(a)
            fs = FrameStamped.from_xyz_rpy(x=near_place.x() + dx,
                                           y=near_place.y() + dy,
                                           z=near_place.z(),
                                           roll=0.0,
                                           pitch=0.0,
                                           yaw=0.0,
                                           stamp=rospy.Time.now(),
                                           frame_id="map")
            points += [fs]
        return points

    def __repr__(self):
        return f"NearObjectSpotDesignator(near_object_designator={self.near_entity_designator}, supporting_entity_designator={self.supporting_entity_designator} name='{self._name}')"


if __name__ == '__main__':
    import sys
    from robot_skills.get_robot import get_robot
    from robot_smach_states.util.designators import EntityByIdDesignator

    if len(sys.argv) < 4:
        print(f"usage: python {sys.argv[0]} ROBOT ENTITY_ID SHELF_ID")
        sys.exit()

    rospy.init_node('test_near_object')

    robot = get_robot(sys.argv[1])
    entityDes = EntityByIdDesignator(robot, uuid=sys.argv[2])
    shelfDes = EntityByIdDesignator(robot, uuid=sys.argv[3])

    nearObjectDes = NearObjectSpotDesignator(robot, entityDes, shelfDes)
    place_pose = nearObjectDes.resolve()
    rospy.loginfo(f"Designator {nearObjectDes} resolves to {place_pose}")
