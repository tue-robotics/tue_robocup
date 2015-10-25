#! /usr/bin/env python

import inspect
import pprint
from cb_planner_msgs_srvs.msg import PositionConstraint
from ed.msg import EntityInfo
from ed.srv import SimpleQuery, SimpleQueryRequest
from geometry_msgs import msg as gm
import rospy
from std_msgs import msg as std
from visualization_msgs.msg import MarkerArray, Marker

from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.checks import check_resolve_type

__author__ = 'loy'


class PointStampedOfEntityDesignator(Designator):

    def __init__(self, entity_designator, name=None):
        super(VariableDesignator, self).__init__(resolve_type=gm.PointStamped, name=name)
        self.entity_designator
        self.ed = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

    def resolve(self):
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

    def resolve(self):
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

    def resolve(self):
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
                    self._current = min(entities, key=self.weight_function)
                else:
                    self._current = entities[0]
                return self.current

        rospy.logerr("No entities found in {0}".format(self))
        return None

    # def __repr__(self):
    #     criteria_code = [inspect.getsource(criterium).strip().replace('\\n', '\n') for criterium in self.criteriafuncs]

    #     return "EdEntityDesignator(robot, type={0}, center_point={1}, radius={2}, id={3}, parse={4}, criteriafuncs={5})".format(
    #         self.type, str(self.center_point).replace("\n", " "), self.radius, self.id, self.parse, pprint.pformat(criteria_code))


class EntityByIdDesignator(Designator):
    def __init__(self, robot, id_, parse=True, name=None):
        super(EntityByIdDesignator, self).__init__(resolve_type=EntityInfo, name=name)
        self.ed = robot.ed
        self.id_ = id_
        self.parse = parse

    def resolve(self):
        entities = self.ed.get_entities(id=self.id_, parse=self.parse)
        if entities:
            return entities[0]
        else:
            return None


class ReasonedEntityDesignator(Designator):
    def __init__(self, robot, querystring, name=None):
        super(ReasonedEntityDesignator, self).__init__(resolve_type=EntityInfo, name=name)
        assert hasattr(robot, "reasoner")
        self.robot = robot
        self.querystring = querystring

    def resolve(self):
        first_answer = self.robot.reasoner.query_first_answer(self.reasoner_query)
        if not first_answer:
            return None
        print "first_answer is:", str(first_answer)

        entities = self.ed.get_entities(id=str(first_answer), parse=True)
        if entities:
            return entities[0]
        else:
            return None


class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by queying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot."""
    def __init__(self, robot, place_location_designator, name=None):
        super(EmptySpotDesignator, self).__init__(resolve_type=gm.PoseStamped, name=name)
        self.robot = robot
        self.place_location_designator = place_location_designator
        self._edge_distance = 0.1                   # Distance to table edge
        self._spacing = 0.15

        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def resolve(self):
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

            distance = 10**10 #Just a really really big number for empty plans so they seem far away and are thus unfavorable
            if plan_to_poi:
                distance = len(plan_to_poi)
            print "Distance: %s"%distance
            return distance

        if any(open_POIs):
            best_poi = min(open_POIs, key=distance_to_poi_area)
            placement = geom.PoseStamped(pointstamped=best_poi)
            rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
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
                    ps = geom.PointStamped()
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

if __name__ == "__main__":
    import doctest
    doctest.testmod()