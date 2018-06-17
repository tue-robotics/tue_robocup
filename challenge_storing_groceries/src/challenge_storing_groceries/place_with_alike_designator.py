import rospy
import numpy as np
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped, FrameStamped, kdl_frame_stamped_from_XYZRPY

class PlaceWithAlikeObjectDesignator(ds.EmptySpotDesignator):
    """Find a FrameStamped where to place an object if a class 'A' besides another object of class 'A'
    """

    def __init__(self, robot, entity_to_place_designator, place_location_designator, areas=None, name=None, debug=False):
        super(PlaceWithAlikeObjectDesignator, self).__init__(robot, place_location_designator, name, area=areas[0])

        self.entity_to_place_designator = entity_to_place_designator
        self.areas = areas

        self._debug = debug

    def _resolve(self):
        """
        :return: Where can an object be placed to put it besides an object of the same type
        :returns: FrameStamped
        """
        if self._debug:
            import ipdb; ipdb.set_trace()

        entity_to_place = self.entity_to_place_designator.resolve()
        assert isinstance(entity_to_place, Entity)

        rospy.loginfo("The grasped entity is a '{}'".format(entity_to_place.type))

        all_entities = self.robot.ed.get_entities(parse=False)  # type: List[Entity]
        place_location = self.place_location_designator.resolve()
        entities_inside_placement = []
        for area in self.areas:
            entities_inside_placement += place_location.entities_in_volume(all_entities, area)

        rospy.loginfo("{l} entities inside the place_location '{pl}': {eip}".format(l=len(entities_inside_placement),
                                                                                    pl=place_location.id,
                                                                                    eip=[(e.id[:6], e.type) for e in entities_inside_placement]))

        # Entities that the entity_to_place could be placed besides (of same or similar type)
        candidate_entities_for_pairing = [e for e in entities_inside_placement if self._same_class(entity_to_place, e)]  # type: List[Entity]

        rospy.loginfo("Entities we could pair with: {}".format([(e.id[:6], e.type) for e in candidate_entities_for_pairing]))

        # FrameStamped's the entity_to_place could be placed at. Each maps to the entity it is besides.
        # This way can relate and refer to the entity we pick eventually and say something about it
        all_placement_candidates = {}  # type: List[FrameStamped]

        for candidate_ent in candidate_entities_for_pairing:
            placements = self._generate_placements_beside(candidate_ent)
            for placement in placements:
                all_placement_candidates[placement] = candidate_ent

        def _is_in_a_volume(placement_candidate):
            point = placement_candidate.extractVectorStamped()
            return any(place_location.in_volume(point, vol) for vol in self.areas)

        rospy.loginfo("We have {} placement_candidates".format(len(all_placement_candidates)))

        placement_candidates_in_volume = filter(_is_in_a_volume, all_placement_candidates.keys())
        rospy.loginfo("We have {} placement_candidates_in_volume".format(len(placement_candidates_in_volume)))

        unoccupied_placement_candidates = filter(self.is_poi_occupied, placement_candidates_in_volume)
        rospy.loginfo("We have {} unoccupied_placement_candidates".format(len(placement_candidates_in_volume)))

        self.marker_array.markers += [self.create_marker(fs.frame.p.x(), fs.frame.p.y(), fs.frame.p.z()) for fs in unoccupied_placement_candidates]
        self.marker_pub.publish(self.marker_array)

        best_placement = self.select_best_feasible_poi(unoccupied_placement_candidates)
        if best_placement:
            place_besides = all_placement_candidates[best_placement]
            rospy.loginfo("Placing besides {} at {}".format(place_besides, best_placement))
            return best_placement
        else:
            rospy.logwarn("Could not find an entity we could place besides so just placing somewhere")
            return super(PlaceWithAlikeObjectDesignator, self)._resolve()

    def _same_class(self, entity_a, entity_b):
        if entity_a.type == entity_b.type:
            return True
        elif entity_b.super_types and entity_a.is_a(entity_b.super_types[0]):
            return True
        else:
            return False

    def _generate_placements_beside(self, entity):
        placements = self._generate_around(entity.pose, 0.2, 8)

        # TODO: This is an ugly hack to make select_best_feasible_poi work, because super().determine_points_of_interest() also does this.
        for placement in placements:
            setattr(placement, 'edge_score', 0)
        return placements

    @staticmethod
    def _generate_around(framestamped, radius, n):
        angles = np.linspace(0, 2 * np.pi, n)
        vector = framestamped.extractVectorStamped().vector
        x, y, z = vector.x(), vector.y(), vector.z()
        xs = radius*np.cos(angles) + x
        ys = radius*np.sin(angles) + y
        frame_stampeds = [kdl_frame_stamped_from_XYZRPY(xi, yi, z, 0, 0, 0, frame_id=framestamped.frame_id) for
                                   xi, yi in zip(xs, ys)]
        return frame_stampeds
