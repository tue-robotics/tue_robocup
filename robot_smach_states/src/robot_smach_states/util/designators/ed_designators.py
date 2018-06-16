#! /usr/bin/env python

# System
import inspect
import math
import pprint

# ROS
import PyKDL as kdl
import rospy
from visualization_msgs.msg import MarkerArray, Marker

# TU/e Robotics
from cb_planner_msgs_srvs.msg import PositionConstraint
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector, VectorStamped, FrameStamped,\
    kdl_frame_stamped_from_XYZRPY
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.checks import check_resolve_type
from robot_smach_states.util.geometry_helpers import offsetConvexHull


__author__ = 'loy'


class EdEntityCollectionDesignator(Designator):
    """
    Resolves to a collection of Ed entities

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> entities = EdEntityCollectionDesignator(robot)
    >>> check_resolve_type(entities, [Entity]) #This is more a test for check_resolve_type to be honest :-/
    """

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None,
                 type_designator=None, center_point_designator=None, id_designator=None, debug=False, name=None):
        """Designates a collection of entities of some type, within a radius of some center_point, with some id,
        that match some given criteria functions.
        @param robot the robot to use for Ed queries
        @param type the type of the entity to resolve to (default: any type)
        @param center_point combined with radius: a sphere to search an entity in
        @param radius combined with center_point: a sphere to search an entity in
        @param id the ID of the object to get info about
        @param parse whether to parse the data string associated with the object model or entity
        @param type_designator same as type but dynamically resolved trhough a designator. Mutually exclusive with type
        @param center_point_designator same as center_point but dynamically resolved through a designator. Mutually exclusive with center_point
        @param id_designator same as id but dynamically resolved through a designator. Mutually exclusive with id"""
        super(EdEntityCollectionDesignator, self).__init__(resolve_type=[Entity], name=name)
        self.ed = robot.ed
        if type != "" and type_designator != None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point != None and center_point_designator != None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point == None and center_point_designator == None:
            center_point = VectorStamped()
        if id != "" and id_designator != None:
            raise TypeError("Specify either id or id_designator, not both")

        self.type = type
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []

        if type_designator: check_resolve_type(type_designator, str)
        self.type_designator = type_designator

        if center_point_designator: check_resolve_type(center_point_designator, VectorStamped)
        self.center_point_designator = center_point_designator

        if id_designator: check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def _resolve(self):
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if self.debug:
            import ipdb;
            ipdb.set_trace()
        if entities:
            for criterium in _criteria:
                entities = filter(criterium, entities)
                criterium_code = inspect.getsource(criterium)
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(entities)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([ent.id for ent in entities])))
            if entities:
                self._current = entities
                return self.current

        rospy.logerr("No entities found in {0}".format(self))
        return None

        # def __repr__(self):
        #     return "EdEntityCollectionDesignator(robot, type={0}, center_point={1}, radius={2}, id={3}, parse={4}, criteriafuncs={5})".format(
        #         self.type, str(self.center_point).replace("\n", " "), self.radius, self.id, self.parse, self.criteriafuncs)


class EdEntityDesignator(Designator):
    """
    Resolves to an entity from an Ed query
    """

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None,
                 weight_function=None,
                 type_designator=None, center_point_designator=None, id_designator=None, debug=False, name=None):
        """Designates an entity of some type, within a radius of some center_point, with some id,
        that match some given criteria functions.
        @param robot the robot to use for Ed queries
        @param type the type of the entity to resolve to (default: any type)
        @param center_point combined with radius: a sphere to search an entity in
        @param radius combined with center_point: a sphere to search an entity in
        @param id the ID of the object to get info about
        @param parse whether to parse the data string associated with the object model or entity
        @param criteriafuncs a list of functions that take an entity and return a bool (True if criterium met)
        @param weight_function returns a weight for each entity, the one with the lowest weight will be selected (could be a distance calculation)
        @param type_designator same as type but dynamically resolved trhough a designator. Mutually exclusive with type
        @param center_point_designator same as center_point but dynamically resolved trhough a designator. Mutually exclusive with center_point
        @param id_designator same as id but dynamically resolved through a designator. Mutually exclusive with id"""
        super(EdEntityDesignator, self).__init__(resolve_type=Entity, name=name)
        self.robot = robot
        self.ed = robot.ed
        if type != "" and type_designator != None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point != None and center_point_designator != None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point == None and center_point_designator == None:
            center_point = VectorStamped()
        if id != "" and id_designator != None:
            raise TypeError("Specify either id or id_designator, not both")

        self.type = type
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []
        self.weight_function = weight_function or (lambda entity: 0)

        if type_designator: check_resolve_type(type_designator, str,
                                               list)  # the resolve type of type_designator can be either st or list
        self.type_designator = type_designator

        if center_point_designator: check_resolve_type(center_point_designator,
                                                       VectorStamped)  # the resolve type of type_designator can be either st or list
        self.center_point_designator = center_point_designator

        if id_designator: check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def lockable(self):
        return LockToId(self.robot, self)

    def _resolve(self):
        if self.debug:
            import ipdb;
            ipdb.set_trace()
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        if self.type_designator and not _type:
            rospy.logwarn("type_designator {0} failed to resolve: {1}".format(self.type_designator, _type))
        if self.center_point_designator and not _center_point:
            rospy.logwarn("center_point_designator {0} failed to resolve: {1}".format(self.center_point_designator,
                                                                                      _center_point))
        if self.id_designator and not _id:
            rospy.logwarn("id_designator {0} failed to resolve: {1}".format(self.id_designator, _id))

        if isinstance(_type, list):
            _type = ""  # Do the check not in Ed but in code here
            typechecker = lambda entity: entity.type in _type
            _criteria += [typechecker]

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if entities:
            for criterium in _criteria:
                criterium_code = inspect.getsource(criterium)
                filtered_entities = []
                for entity in entities:
                    try:
                        if criterium(entity):
                            filtered_entities += [entity]
                    except Exception as exp:
                        rospy.logerr("{id} cannot be filtered with criterum '{crit}': {exp}".format(id=entity.id,
                                                                                                    crit=criterium_code,
                                                                                                    exp=exp))

                entities = filtered_entities
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(entities)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([ent.id for ent in entities])))

            if entities:
                weights = [self.weight_function(entity) for entity in entities]
                names = [entity.id for entity in entities]

                if len(entities) >= 1:
                    rospy.loginfo('choosing best entity from this list (name->weight):\n\t%s', zip(names, weights))
                    return min(entities, key=self.weight_function)
                else:
                    return entities[0]

        rospy.logerr("No entities found in {0}".format(self))
        return None

        # def __repr__(self):
        #     criteria_code = [inspect.getsource(criterium).strip().replace('\\n', '\n') for criterium in self.criteriafuncs]

        #     return "EdEntityDesignator(robot, type={0}, center_point={1}, radius={2}, id={3}, parse={4}, criteriafuncs={5})".format(
        #         self.type, str(self.center_point).replace("\n", " "), self.radius, self.id, self.parse, pprint.pformat(criteria_code))


class EntityByIdDesignator(Designator):
    def __init__(self, robot, id, parse=True, name=None):
        """
        Designate an entity by its ID. Resolves to the entity with that ID
        :param robot: Robot who's worldmodel to use
        :param id: ID of the entity. If no such ID, resolves to None
        :param parse: Whether to parse the Entity's data-field
        :param name: Name of the designator for introspection purposes
        """
        super(EntityByIdDesignator, self).__init__(resolve_type=Entity, name=name)
        self.ed = robot.ed
        self.id_ = id
        self.parse = parse

    def _resolve(self):
        entities = self.ed.get_entities(id=self.id_, parse=self.parse)
        if entities:
            return entities[0]
        else:
            return None

class ReasonedEntityDesignator(Designator):
    def __init__(self, robot, query, name=None):
        """
        Designate an entity by its ID. Resolves to the entity with that ID
        :param robot: Robot who's worldmodel and reasoner to use. Robot must have a reasoner
        :param query: query to the reasoner. The first answer is cast to string and used as ID
        :param name: Name of the designator for introspection purposes
        """
        super(ReasonedEntityDesignator, self).__init__(resolve_type=Entity, name=name)
        assert hasattr(robot, "reasoner")
        self.robot = robot
        self.querystring = query

        self._locker = None

    def _resolve(self):
        first_answer = self.robot.reasoner.query_first_answer(self.reasoner_query)
        if not first_answer:
            return None
        print "first_answer is:", str(first_answer)

        entities = self.ed.get_entities(id=str(first_answer), parse=True)
        if entities:
            return entities[0]
        else:
            return None

    def lockable(self):
        if not self._locker:
            self._locker = LockToId(self.robot, self)
        return self._locker


class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by querying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot.

    To test this in the robotics_test_lab with amigo-console:
    robot = amigo
    CABINET = "bookcase"
    PLACE_SHELF = "shelf2"
    cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
    place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, cabinet, name="placement", area=PLACE_SHELF), name="place_position")
    """

    def __init__(self, robot, place_location_designator, name=None, area=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity
        :param robot: Robot whose worldmodel to use
        :param place_location_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param name: name for introspection purposes
        :param area: (optional) area where the item should be placed
        """
        super(EmptySpotDesignator, self).__init__(resolve_type=FrameStamped, name=name)
        self.robot = robot

        self.place_location_designator = place_location_designator
        self._edge_distance = 0.05  # Distance to table edge
        self._spacing = 0.15
        self._area = area

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
            vectors_of_interest = self.determine_points_of_interest_with_area(place_location, self._area)
        else:
            vectors_of_interest = self.determine_points_of_interest(place_frame.frame, z_max=place_location.shape.z_max,
                                                                    convex_hull=place_location.shape.convex_hull)

        assert all(isinstance(v, FrameStamped) for v in vectors_of_interest)

        def is_poi_occupied(frame_stamped):
            entities_at_poi = self.robot.ed.get_entities(center_point=frame_stamped.extractVectorStamped(),
                                                         radius=self._spacing)
            return not any(entities_at_poi)

        open_POIs = filter(is_poi_occupied, vectors_of_interest)

        def distance_to_poi_area(frame_stamped):
            # Derived from navigate_to_place
            radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)
            x = frame_stamped.frame.p.x()
            y = frame_stamped.frame.p.y()
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

        # List with tuples containing both the POI and the distance the
        # robot needs to travel in order to place there
        open_POIs_dist = [(poi, distance_to_poi_area(poi)) for poi in open_POIs]

        # Feasible POIS: discard
        feasible_POIs = []
        for tup in open_POIs_dist:
            if tup[1]:
                feasible_POIs.append(tup)

        if any(feasible_POIs):
            feasible_POIs.sort(key=lambda tup: tup[1])  # sorts in place

            # We don't care about small differences
            nav_threshold = 0.5 / 0.05  # Distance (0.5 m) divided by resolution (0.05)
            feasible_POIs = [f for f in feasible_POIs if (f[1]-feasible_POIs[0][1]) < nav_threshold]

            feasible_POIs.sort(key=lambda tup: tup[0].edge_score, reverse=True)
            best_poi = feasible_POIs[0][0]  # Get the POI of the best match

            selection = self.create_selection_marker(best_poi)
            self.marker_pub.publish(MarkerArray([selection]))

            return best_poi
        else:
            rospy.logerr("Could not find an empty spot")
            return None

    def create_marker(self, x, y, z):
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

    def create_selection_marker(self, selected_pose):
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

    def determine_points_of_interest_with_area(self, entity, area):
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
        return self.determine_points_of_interest(entity._pose, top_z, box.bottom_area)

    def determine_points_of_interest(self, center_frame, z_max, convex_hull):
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

                self.marker_array.markers.append(self.create_marker(fs.frame.p.x(), fs.frame.p.y(), fs.frame.p.z()))

                # ToDo: check if still within hull???
                d += self._spacing

        self.marker_pub.publish(self.marker_array)

        return points


class LockToId(Designator):
    """An Entity...Designator's resolve() method may return a different Entity everytime.
    For some cases, this may be unwanted because a process has to be done with the same Entity for tha action to be
    consistent.
    In that case, a designator resolving to a different object every time is not usable.
    A LockToId will resolve to the same Entity after a call to .lock() and
    will only resolve to a different Entity after an unlock() call.
    This is done by remembering the Entity's ID"""

    def __init__(self, robot, to_be_locked, name=None):
        """ Constructor

        :param robot: robot object
        :param to_be_locked: designator to be locked
        :param name: (optional) might come in handy for debugging
        """
        super(LockToId, self).__init__(resolve_type=to_be_locked.resolve_type, name=name)
        self.robot = robot
        self.to_be_locked = to_be_locked
        self._locked_to_id = None
        self._locked = False

    def lock(self):
        self._locked = True

    def unlock(self):
        self._locked_to_id = None
        self._locked = False

    def _resolve(self):
        if self._locked:  # If we should resolve to a remembered thing
            if not self._locked_to_id:  # but we haven't remembered anything yet
                entity = self.to_be_locked.resolve()  # Then find  out what we should remember
                if entity:  # If we can find what to remember
                    self._locked_to_id = entity.id  # remember its ID.
            else:  # If we do remember something already, recall that remembered ID:
                return self.robot.ed.get_entity(id=self._locked_to_id)
        else:
            entity = self.to_be_locked.resolve()
            rospy.loginfo("{0} resolved to {1}, but is *not locked* to it".format(self, entity.id))
            return entity

    def __repr__(self):
        return "LockToId({})._locked = {}".format(self.to_be_locked, self._locked)


if __name__ == "__main__":

    import doctest

    doctest.testmod()
