#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest
from ed.srv import GetGUICommand, GetGUICommandResponse
from geometry_msgs.msg import Point, PointStamped
import robot_skills.util.msg_constructors as msgs
from robot_skills.util import transformations
from math import hypot

from std_srvs.srv import Empty #Reset Ed

import yaml

class ED:
    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._ed_simple_query_srv = rospy.ServiceProxy('/%s/ed/simple_query'%robot_name, SimpleQuery)
        self._ed_reset_srv = rospy.ServiceProxy('/%s/ed/reset'%robot_name, Empty)

        self.tf_listener = tf_listener

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

    def get_entity(self, id):
        entities = self.get_entities(id=id)
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
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "/map", self.tf_listener)
        return point_in_map


