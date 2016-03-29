#! /usr/bin/env python

import inspect
import pprint
import math

from cb_planner_msgs_srvs.msg import PositionConstraint
from ed.msg import EntityInfo
from ed.srv import SimpleQuery, SimpleQueryRequest
from geometry_msgs import msg as gm
import rospy
from std_msgs import msg as std
from visualization_msgs.msg import MarkerArray, Marker

from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.checks import check_resolve_type

from robot_smach_states.util.geometry_helpers import poseMsgToKdlFrame, pointMsgToKdlVector
import robot_smach_states.util.geometry_helpers as geom

import robot_skills.util.msg_constructors as msg_constructors

__author__ = 'loy'

class PointStampedOfEntityDesignator(Designator):
    def __init__(self, entity_designator, name=None):
        """Resolves to the PointStamped-part of the designated entity
        :param entity_designator entity of which we want to know the position as a PointStamped
        """
        super(PointStampedOfEntityDesignator, self).__init__(resolve_type=gm.PointStamped, name=name)
        self.entity_designator = entity_designator
        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

    def _resolve(self):
        # type is a reserved keyword. Maybe unpacking a dict as kwargs is
        # cleaner
        query = SimpleQueryRequest(id=self.entity_designator.resolve())
        entities = self.ed(query).entities
        if entities:
            entity = entities[0]
            pointstamped = gm.PointStamped(point=entity.pose.position,
                                           header=std.Header(
                                               entity.id, rospy.get_rostime())
                                           )  # ID is also the frame ID. Ed just works that way
            self._current = pointstamped
            return self.current
        else:
            return None


class EdEntityCollectionDesignator(Designator):
    """
    Resolves to a collection of Ed entities

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> entities = EdEntityCollectionDesignator(robot)
    >>> check_resolve_type(entities, [EntityInfo]) #This is more a test for check_resolve_type to be honest :-/
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
        super(EdEntityCollectionDesignator, self).__init__(resolve_type=[EntityInfo],name=name)
        self.ed = robot.ed
        if type != "" and type_designator != None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point != None and center_point_designator != None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point == None and center_point_designator == None:
            center_point = gm.Point()
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

        if center_point_designator: check_resolve_type(center_point_designator, gm.PointStamped)
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
            import ipdb; ipdb.set_trace()
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

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None, weight_function=None,
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
        super(EdEntityDesignator, self).__init__(resolve_type=EntityInfo, name=name)
        self.robot = robot
        self.ed = robot.ed
        if type != "" and type_designator != None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point != None and center_point_designator != None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point == None and center_point_designator == None:
            center_point = gm.Point()
        if id != "" and id_designator != None:
            raise TypeError("Specify either id or id_designator, not both")

        self.type = type
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []
        self.weight_function = weight_function or (lambda entity: 0)

        if type_designator: check_resolve_type(type_designator, str, list) #the resolve type of type_designator can be either st or list
        self.type_designator = type_designator

        if center_point_designator: check_resolve_type(center_point_designator, gm.PointStamped) #the resolve type of type_designator can be either st or list
        self.center_point_designator = center_point_designator

        if id_designator: check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def lockable(self):
        return LockToId(self.robot, self)

    def _resolve(self):
        if self.debug:
            import ipdb; ipdb.set_trace()
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        if self.type_designator and not _type:
            rospy.logwarn("type_designator {0} failed to resolve: {1}".format(self.type_designator, _type))
        if self.center_point_designator and not _center_point:
            rospy.logwarn("center_point_designator {0} failed to resolve: {1}".format(self.center_point_designator, _center_point))
        if self.id_designator and not _id:
            rospy.logwarn("id_designator {0} failed to resolve: {1}".format(self.id_designator, _id))

        if isinstance(_type, list):
            _type = "" #Do the check not in Ed but in code here
            typechecker = lambda entity: entity.type in _type
            _criteria += [typechecker]

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if entities:
            for criterium in _criteria:
                entities = filter(criterium, entities)
                criterium_code = inspect.getsource(criterium)
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
        super(EntityByIdDesignator, self).__init__(resolve_type=EntityInfo, name=name)
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
        super(ReasonedEntityDesignator, self).__init__(resolve_type=EntityInfo, name=name)
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
    It does this by queying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot."""
    def __init__(self, robot, place_location_designator, name=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity
        :param robot: Robot whose worldmodel to use
        :param place_location_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param name: name for introspection purposes
        """
        super(EmptySpotDesignator, self).__init__(resolve_type=gm.PoseStamped, name=name)
        self.robot = robot

        self.place_location_designator = place_location_designator
        self._edge_distance = 0.1                   # Distance to table edge
        self._spacing = 0.15

        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def _resolve(self):
        place_location = self.place_location_designator.resolve()

        # points_of_interest = []
        points_of_interest = self.determinePointsOfInterest(place_location)

        def is_poi_occupied(poi):
            entities_at_poi = self.robot.ed.get_entities(center_point=poi, radius=self._spacing)
            return not any(entities_at_poi)

        open_POIs = filter(is_poi_occupied, points_of_interest)

        def distance_to_poi_area(poi):
            #Derived from navigate_to_place
            radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)
            x = poi.point.x
            y = poi.point.y
            ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
            pos_constraint = PositionConstraint(constraint=ri+" and "+ro, frame="/map")

            plan_to_poi = self.robot.base.global_planner.getPlan(pos_constraint)

            if plan_to_poi:
                distance = len(plan_to_poi)
                print "Distance: %s"%distance
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
            best_poi = feasible_POIs[0][0] # Get the POI of the best match
            placement = msg_constructors.PoseStamped(pointstamped=best_poi)
            # rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
            return placement
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

    def determinePointsOfInterest(self, e):

        points = []

        x = e.pose.position.x
        y = e.pose.position.y

        if len(e.convex_hull) == 0:
            rospy.logerr('Entity: {0} has an empty convex hull'.format(e.id))
            return []

        ''' Convert convex hull to map frame '''
        center_pose = poseMsgToKdlFrame(e.pose)
        ch = []
        for point in e.convex_hull:
            p = pointMsgToKdlVector(point)
            # p = center_pose * p
            # p = p * center_pose
            import PyKDL as kdl
            pf = kdl.Frame(kdl.Rotation(), p)
            pf = pf * center_pose
            p = pf.p
            ch.append(p)

        ''' Loop over hulls '''
        self.marker_array.markers = []
        ch.append(ch[0])
        for i in xrange(len(ch) - 1):
                dx = ch[i+1].x() - ch[i].x()
                dy = ch[i+1].y() - ch[i].y()
                length = math.hypot(dx, dy)

                d = self._edge_distance
                while d < (length-self._edge_distance):

                    ''' Point on edge '''
                    xs = ch[i].x() + d/length*dx
                    ys = ch[i].y() + d/length*dy

                    ''' Shift point inwards and fill message'''
                    ps = gm.PointStamped()
                    ps.header.frame_id = "/map"
                    ps.point.x = xs - dy/length * self._edge_distance
                    ps.point.y = ys + dx/length * self._edge_distance
                    ps.point.z = e.pose.position.z + e.z_max
                    points.append(ps)

                    self.marker_array.markers.append(self.create_marker(ps.point.x, ps.point.y, ps.point.z))

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
        if self._locked: # If we should resolve to a remembered thing
            if not self._locked_to_id: # but we haven't remembered anything yet
                entity = self.to_be_locked.resolve() # Then find  out what we should remember
                if entity: # If we can find what to remember
                    self._locked_to_id = entity.id # remember its ID.
                else:
                    pass # If we cannot find what to remember, to_be_locked.resolve() return None and we return that too
                # rospy.loginfo("{0} locked to ID {1}".format(self, self._locked_to_id))
            else: # If we do remember something already, recall that remembered ID:
                return self.robot.ed.get_entities(id=self._locked_to_id)
        else:
            entity = self.to_be_locked.resolve()
            rospy.loginfo("{0} resolved to {1}, but is *not locked* to it".format(self, entity.id))
            return entity

    def __repr__(self):
        return "LockToId({})._locked = {}".format(self.to_be_locked, self._locked)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
