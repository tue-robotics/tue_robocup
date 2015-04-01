#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest
from ed.srv import GetGUICommand, GetGUICommandResponse
from ed_navigation.srv import GetGoalConstraint
from cb_planner_msgs_srvs.msg import PositionConstraint
from geometry_msgs.msg import Point, PointStamped
import robot_skills.util.msg_constructors as msgs
from robot_skills.util import transformations
from math import hypot

from std_srvs.srv import Empty #Reset Ed

import yaml

class Navigation:
    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._get_constraint_srv = rospy.ServiceProxy('/%s/ed/navigation/get_constraint'%robot_name, GetGoalConstraint)

    def get_position_constraint(self, entity_id_area_name_map):
        try:
            res = self._get_constraint_srv(entity_ids=[ k for k in entity_id_area_name_map ], area_names=[ v for k,v in entity_id_area_name_map.iteritems() ])
        except Exception, e:
            rospy.logerr(e)
            return None

        if res.error_msg != '':
            rospy.logerr(res.error_msg)
            return None

        return PositionConstraint(constraint=res.position_constraint_map_frame, frame="/map")

class ED:
    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._ed_simple_query_srv = rospy.ServiceProxy('/%s/ed/simple_query'%robot_name, SimpleQuery)
        self._ed_reset_srv = rospy.ServiceProxy('/%s/ed/reset'%robot_name, Empty)

        self._tf_listener = tf_listener

        self.navigation = Navigation(robot_name, tf_listener, wait_service)

    def get_entities(self, type="", center_point=Point(), radius=0, id="", parse=True):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        query = SimpleQueryRequest(id=id, type=type, center_point=center_point, radius=radius) 

        try:
            entities = self._ed_simple_query_srv(query).entities
        except Exception, e:
            rospy.logerr(e)
            return []

        # Parse to data strings to yaml
        if parse:
            for e in entities:
                e.data = yaml.load(e.data)
        
        return entities

    def get_closest_entity(self, type="", center_point=Point(), radius=0):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)
            
        entities = self.get_entities(type=type, center_point=center_point, radius=radius)
        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.center_point.x, center_point.y - entity.center_point.y))
        except:
            return None

        return entities[0]

    def get_entity(self, id, parse=True):
        entities = self.get_entities(id=id, parse=parse)
        if len(entities) == 0:
            return None

        return entities[0]

    def reset(self):
        try:
            self._ed_reset_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Could not reset ED: {0}".format(e))
        rospy.sleep(1.0)

    def _transform_center_point_to_map(self, pointstamped):
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "/map", self._tf_listener)
        return point_in_map


