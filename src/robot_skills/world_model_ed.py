#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest
from ed.srv import GetGUICommand, GetGUICommandResponse
from geometry_msgs.msg import Point
import robot_skills.util.msg_constructors as msgs
from math import hypot

from std_srvs.srv import Empty #Reset Ed

class ED:
    def __init__(self, wait_service=False):
        self._ed_simple_query_srv = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)
        self._ed_reset_srv = rospy.ServiceProxy('/ed/reset', Empty)

    def getEntities(self, type="", center_point=Point(), radius=0, id=""):
        query = SimpleQueryRequest(type=type, center_point=center_point, radius=radius) 

        try:
            entities = self._ed_simple_query_srv(query).entities
        except Exception, e:
            rospy.logerr(e)
            return []

        return entities

    def getClosestEntity(self, type="", center_point=Point(), radius=0):
        entities = self.getEntities(type=type, center_point=center_point, radius=radius)
        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.center_point.x, center_point.y - entity.center_point.y))
        except:
            return None

        return entities[0]

    def getEntity(self, id):
        entities = self.getEntities(id=id)
        if len(entities) == 0:
            return None

        return entities[0]

    def reset(self):
        self._ed_reset_srv()


